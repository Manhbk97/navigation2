// SpinMonitor: observes the Nav2 "spin" action server status topic.
// Returns RUNNING while Nav2 is executing Spin, SUCCESS on completion,
// FAILURE when no goal is active (Nav2 is not spinning).
#include <memory>
#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "action_msgs/msg/goal_status_array.hpp"
#include "nav2_monitoring_tree/nav2_monitor_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_monitoring_tree
{

class SpinMonitor : public BT::StatefulActionNode
{
public:
  SpinMonitor(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    const std::string ns = config.blackboard->get<std::string>("nav2_namespace");
    const std::string topic = ns_topic(ns, "spin/_action/status");

    RCLCPP_INFO(node_->get_logger(), "SpinMonitor subscribing to '%s'", topic.c_str());

    sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      topic, 10,
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        last_status_ = msg;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("spin_dist", 1.57, "Target yaw distance (rad)"),
      BT::OutputPort<int>("error_code_id"),
    };
  }

  BT::NodeStatus onStart() override { return onRunning(); }

  BT::NodeStatus onRunning() override
  {
    rclcpp::spin_some(node_);
    return actionStatusToNodeStatus(last_status_);
  }

  void onHalted() override { last_status_.reset(); }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub_;
  action_msgs::msg::GoalStatusArray::SharedPtr last_status_;
};

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::SpinMonitor>("Spin");
}
