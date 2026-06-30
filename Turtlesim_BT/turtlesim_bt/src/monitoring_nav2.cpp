/**
 * monitoring_nav2.cpp
 *
 * Monitors a running Nav2 follow_point behavior tree using behaviortree_cpp_v3.
 * Each BT node subscribes to the corresponding Nav2 ROS2 topic / action status
 * to reflect the real Nav2 state. Groot connects via ZMQ (ports 1666/1667) for
 * live visualization.
 *
 * Namespace support:
 *   Pass ROS parameter  nav2_namespace:=<ns>  (e.g. nav2_namespace:=apple)
 *   All topics become  /<ns>/topic  instead of  /topic
 *
 * Tree structure (mirrors follow_point.xml):
 *   PipelineSequence
 *   ├── ControllerSelector  → /<ns>/controller_selector
 *   ├── PlannerSelector     → /<ns>/planner_selector
 *   ├── RateController hz=1.0
 *   │   └── Sequence
 *   │       ├── GoalUpdater               → /<ns>/goal_update  (Nav2 param: goal_updater_topic)
 *   │       │   └── ComputePathToPose     → /<ns>/compute_path_to_pose/_action/status
 *   │       └── TruncatePath              → pure computation: reads {path} from BT blackboard
 *   └── KeepRunningUntilFailure
 *       └── FollowPath                    → /<ns>/follow_path/_action/status
 */

#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <string>
#include <chrono>
#include <thread>

using namespace BT;
using namespace std::chrono_literals;

// Shared ROS2 node — all monitoring subscriptions live here.
static rclcpp::Node::SharedPtr g_ros_node;

// Nav2 namespace (e.g. "apple"). Empty string means no namespace prefix.
static std::string g_nav2_namespace;

// ─────────────────────────────────────────────────────────────────────────────
// ns_topic: build a fully-qualified topic name with the Nav2 namespace.
//   ns_topic("compute_path_to_pose/_action/status")
//     → "/apple/compute_path_to_pose/_action/status"  (when ns = "apple")
//     → "/compute_path_to_pose/_action/status"         (when ns = "")
//
// Any leading '/' in the input is stripped so callers can pass either form.
// ─────────────────────────────────────────────────────────────────────────────
static std::string ns_topic(const std::string& topic)
{
    // Strip accidental leading slash
    const std::string t = (topic.front() == '/') ? topic.substr(1) : topic;
    return g_nav2_namespace.empty() ? ("/" + t) : ("/" + g_nav2_namespace + "/" + t);
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: translate the latest ROS2 action goal status → BT NodeStatus.
// ─────────────────────────────────────────────────────────────────────────────
static BT::NodeStatus actionStatusToNodeStatus(
    const action_msgs::msg::GoalStatusArray::SharedPtr& msg)
{
    if (!msg || msg->status_list.empty())
        return BT::NodeStatus::FAILURE;

    using S = action_msgs::msg::GoalStatus;
    const uint8_t s = msg->status_list.back().status;

    if (s == S::STATUS_EXECUTING || s == S::STATUS_ACCEPTED)
        return BT::NodeStatus::RUNNING;
    if (s == S::STATUS_SUCCEEDED)
        return BT::NodeStatus::SUCCESS;

    return BT::NodeStatus::FAILURE;
}

// ─────────────────────────────────────────────────────────────────────────────
// PipelineSequence (Nav2 custom Control node)
// Ticks ALL children every tick; returns FAILURE if any child fails,
// otherwise returns RUNNING (navigation loop never self-terminates).
// ─────────────────────────────────────────────────────────────────────────────
class PipelineSequence : public BT::ControlNode
{
public:
    PipelineSequence(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ControlNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        for (auto* child : children_nodes_)
        {
            auto status = child->executeTick();
            if (status == BT::NodeStatus::FAILURE)
            {
                haltChildren();
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// ControllerSelector
// Subscribes to /<ns>/controller_selector (or value from topic_name port).
// Writes the received string to the selected_controller output port.
// ─────────────────────────────────────────────────────────────────────────────
class ControllerSelector : public BT::SyncActionNode
{
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::string selected_;

public:
    ControllerSelector(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        // topic_name port holds a relative name (e.g. "controller_selector")
        auto port_topic = getInput<std::string>("topic_name").value_or("controller_selector");
        const std::string full_topic = ns_topic(port_topic);
        RCLCPP_INFO(g_ros_node->get_logger(), "ControllerSelector subscribing to '%s'", full_topic.c_str());

        sub_ = g_ros_node->create_subscription<std_msgs::msg::String>(
            full_topic, rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                selected_ = msg->data;
                RCLCPP_INFO(g_ros_node->get_logger(), "ControllerSelector: '%s'", selected_.c_str());
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name", "controller_selector", "Relative topic name"),
            BT::InputPort<std::string>("default_controller", "FollowPath", "Default controller"),
            BT::OutputPort<std::string>("selected_controller"),
        };
    }

    BT::NodeStatus tick() override
    {
        rclcpp::spin_some(g_ros_node);
        auto def = getInput<std::string>("default_controller").value_or("FollowPath");
        setOutput("selected_controller", selected_.empty() ? def : selected_);
        return BT::NodeStatus::SUCCESS;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// PlannerSelector
// Subscribes to /<ns>/planner_selector (or value from topic_name port).
// Writes the received string to the selected_planner output port.
// ─────────────────────────────────────────────────────────────────────────────
class PlannerSelector : public BT::SyncActionNode
{
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::string selected_;

public:
    PlannerSelector(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        auto port_topic = getInput<std::string>("topic_name").value_or("planner_selector");
        const std::string full_topic = ns_topic(port_topic);
        RCLCPP_INFO(g_ros_node->get_logger(), "PlannerSelector subscribing to '%s'", full_topic.c_str());

        sub_ = g_ros_node->create_subscription<std_msgs::msg::String>(
            full_topic, rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                selected_ = msg->data;
                RCLCPP_INFO(g_ros_node->get_logger(), "PlannerSelector: '%s'", selected_.c_str());
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name", "planner_selector", "Relative topic name"),
            BT::InputPort<std::string>("default_planner", "GridBased", "Default planner"),
            BT::OutputPort<std::string>("selected_planner"),
        };
    }

    BT::NodeStatus tick() override
    {
        rclcpp::spin_some(g_ros_node);
        auto def = getInput<std::string>("default_planner").value_or("GridBased");
        setOutput("selected_planner", selected_.empty() ? def : selected_);
        return BT::NodeStatus::SUCCESS;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// RateController (Decorator)
// Limits child ticks to `hz` Hz using rclcpp::Time.
// ─────────────────────────────────────────────────────────────────────────────
class RateController : public BT::DecoratorNode
{
    double hz_;
    rclcpp::Time last_tick_time_;
    BT::NodeStatus last_child_status_ = BT::NodeStatus::RUNNING;

public:
    RateController(const std::string& name, const BT::NodeConfiguration& config)
        : BT::DecoratorNode(name, config)
    {
        hz_ = getInput<double>("hz").value_or(1.0);
        last_tick_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("hz", 1.0, "Tick rate in Hz") };
    }

    BT::NodeStatus tick() override
    {
        const double period_sec = 1.0 / hz_;
        if ((g_ros_node->now() - last_tick_time_).seconds() >= period_sec)
        {
            last_tick_time_ = g_ros_node->now();
            last_child_status_ = child_node_->executeTick();
        }
        return last_child_status_;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// GoalUpdater (Decorator)
// Subscribes to /<ns>/goal_update  (Nav2 reads ROS param "goal_updater_topic",
// default value "goal_update" — see goal_updater_node.cpp).
// Always ticks its child; writes latest received goal to the output_goal port.
// ─────────────────────────────────────────────────────────────────────────────
class GoalUpdater : public BT::DecoratorNode
{
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    geometry_msgs::msg::PoseStamped latest_goal_;
    bool goal_received_ = false;

public:
    GoalUpdater(const std::string& name, const BT::NodeConfiguration& config)
        : BT::DecoratorNode(name, config)
    {
        // Nav2's GoalUpdater reads the topic name from the "goal_updater_topic"
        // ROS parameter (default: "goal_update"). We mirror that default here.
        const std::string full_topic = ns_topic("goal_update");
        RCLCPP_INFO(g_ros_node->get_logger(), "GoalUpdater subscribing to '%s'", full_topic.c_str());

        sub_ = g_ros_node->create_subscription<geometry_msgs::msg::PoseStamped>(
            full_topic, 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                latest_goal_ = *msg;
                goal_received_ = true;
                RCLCPP_INFO(g_ros_node->get_logger(), "GoalUpdater: new goal (%.2f, %.2f)",
                    msg->pose.position.x, msg->pose.position.y);
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("input_goal",  "Goal from blackboard"),
            BT::OutputPort<std::string>("output_goal"),
        };
    }

    BT::NodeStatus tick() override
    {
        rclcpp::spin_some(g_ros_node);
        if (goal_received_)
        {
            const std::string goal_str =
                "(" + std::to_string(latest_goal_.pose.position.x).substr(0, 6)
                + ", " + std::to_string(latest_goal_.pose.position.y).substr(0, 6) + ")";
            setOutput("output_goal", goal_str);
        }
        return child_node_->executeTick();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// ComputePathToPose (StatefulActionNode)
// Monitors /<ns>/compute_path_to_pose/_action/status.
// ─────────────────────────────────────────────────────────────────────────────
class ComputePathToPose : public BT::StatefulActionNode
{
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub_;
    action_msgs::msg::GoalStatusArray::SharedPtr last_status_;

public:
    ComputePathToPose(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config)
    {
        const std::string full_topic = ns_topic("compute_path_to_pose/_action/status");
        RCLCPP_INFO(g_ros_node->get_logger(), "ComputePathToPose subscribing to '%s'", full_topic.c_str());

        sub_ = g_ros_node->create_subscription<action_msgs::msg::GoalStatusArray>(
            full_topic, 10,
            [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
                last_status_ = msg;
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("goal"),
            BT::InputPort<std::string>("start"),
            BT::InputPort<std::string>("planner_id"),
            BT::InputPort<std::string>("server_name"),
            BT::InputPort<std::string>("server_timeout"),
            BT::OutputPort<std::string>("path"),
            BT::OutputPort<std::string>("error_code_id"),
        };
    }

    BT::NodeStatus onStart() override   { return onRunning(); }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(g_ros_node);
        auto status = actionStatusToNodeStatus(last_status_);
        if (status == BT::NodeStatus::SUCCESS)
            setOutput("path", "computed");
        return status;
    }

    void onHalted() override {}
};

// ─────────────────────────────────────────────────────────────────────────────
// TruncatePath (SyncActionNode)
// Nav2's TruncatePath is PURE COMPUTATION — no topics or action servers.
// It reads input_path from the BT blackboard (written by ComputePathToPose),
// trims poses near the goal end, and writes the result to output_path.
//
// For monitoring: returns SUCCESS when {path} blackboard key is set (meaning
// ComputePathToPose already produced a path), FAILURE while path is absent.
// ─────────────────────────────────────────────────────────────────────────────
class TruncatePath : public BT::SyncActionNode
{
public:
    TruncatePath(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("distance"),
            BT::InputPort<std::string>("input_path"),
            BT::OutputPort<std::string>("output_path"),
        };
    }

    BT::NodeStatus tick() override
    {
        // input_path is fed from {path} blackboard key set by ComputePathToPose.
        // If that key holds a non-empty value, the path exists and truncation succeeds.
        auto path = getInput<std::string>("input_path");
        if (path && !path.value().empty())
        {
            setOutput("output_path", "truncated");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

// KeepRunningUntilFailure is a built-in decorator in behaviortree_cpp_v3.
// Do NOT register it — it is already present and re-registering throws.

// ─────────────────────────────────────────────────────────────────────────────
// FollowPath (StatefulActionNode)
// Monitors /<ns>/follow_path/_action/status.
// ─────────────────────────────────────────────────────────────────────────────
class FollowPath : public BT::StatefulActionNode
{
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub_;
    action_msgs::msg::GoalStatusArray::SharedPtr last_status_;

public:
    FollowPath(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config)
    {
        const std::string full_topic = ns_topic("follow_path/_action/status");
        RCLCPP_INFO(g_ros_node->get_logger(), "FollowPath subscribing to '%s'", full_topic.c_str());

        sub_ = g_ros_node->create_subscription<action_msgs::msg::GoalStatusArray>(
            full_topic, 10,
            [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
                last_status_ = msg;
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("controller_id"),
            BT::InputPort<std::string>("path"),
            BT::InputPort<std::string>("goal_checker_id"),
            BT::InputPort<std::string>("progress_checker_id"),
            BT::InputPort<std::string>("server_name"),
            BT::InputPort<std::string>("server_timeout"),
            BT::OutputPort<std::string>("error_code_id"),
        };
    }

    BT::NodeStatus onStart() override   { return onRunning(); }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(g_ros_node);
        return actionStatusToNodeStatus(last_status_);
    }

    void onHalted() override {}
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Bootstrap node — reads all parameters before BT node constructors run.
    // (BT node constructors use g_nav2_namespace, so it must be set first.)
    auto bootstrap = rclcpp::Node::make_shared("nav2_bt_monitor");
    bootstrap->declare_parameter("nav2_namespace",   std::string(""));
    // ZMQ ports — use 1668/1669 so they don't clash with turtle_bt (1666/1667).
    bootstrap->declare_parameter("zmq_publisher_port", 1669);
    bootstrap->declare_parameter("zmq_server_port",    1670);

    g_nav2_namespace = bootstrap->get_parameter("nav2_namespace").as_string();
    const int zmq_pub_port = bootstrap->get_parameter("zmq_publisher_port").as_int();
    const int zmq_srv_port = bootstrap->get_parameter("zmq_server_port").as_int();

    // Re-use the same node for all subscriptions.
    g_ros_node = bootstrap;

    RCLCPP_INFO(g_ros_node->get_logger(),
        "Nav2 BT Monitor — namespace: '%s'",
        g_nav2_namespace.c_str());

    BehaviorTreeFactory factory;

    // Control nodes
    factory.registerNodeType<PipelineSequence>("PipelineSequence");

    // Decorator nodes
    factory.registerNodeType<RateController>("RateController");
    factory.registerNodeType<GoalUpdater>("GoalUpdater");
    // KeepRunningUntilFailure: built-in, do not register

    // Action nodes
    // NOTE: constructors run here — g_nav2_namespace must be set before this point.
    factory.registerNodeType<ControllerSelector>("ControllerSelector");
    factory.registerNodeType<PlannerSelector>("PlannerSelector");
    factory.registerNodeType<ComputePathToPose>("ComputePathToPose");
    factory.registerNodeType<TruncatePath>("TruncatePath");
    factory.registerNodeType<FollowPath>("FollowPath");

    std::string pkg_share = ament_index_cpp::get_package_share_directory("turtlesim_bt");
    std::string tree_xml  = pkg_share + "/config/nav2_monitor_tree.xml";
    auto tree = factory.createTreeFromFile(tree_xml);

    std::string log_file_path = "/home/rgt/rgt2_ws/install/turtlesim_bt/share/turtlesim_bt/config/nav2_bt_log.fbl";
    BT::FileLogger file_logger(tree, log_file_path.c_str());

    // ZMQ publisher — connect Groot in Real-time mode.
    // Uses ports 1668/1669 by default (turtle_bt already owns 1666/1667).
    BT::PublisherZMQ zmq_publisher(tree, 25, zmq_pub_port, zmq_srv_port);

    RCLCPP_INFO(g_ros_node->get_logger(),
        "Tree loaded. Open Groot → Real-time on publisher port %d / server port %d. Ticking at 10 Hz.",
        zmq_pub_port, zmq_srv_port);

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        tree.tickRoot();
        rclcpp::spin_some(g_ros_node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
