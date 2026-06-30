// RecoveryOncePerLocation (BTcpp v3): suppresses repeated recovery at the same
// stuck location.  Frame parameters use the same port → ROS param → fallback
// resolution as GetCurrentPose, mirroring the BTcpp v4
// BT::deconflictPortAndParamFrame pattern.
#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_monitoring_tree/nav2_monitor_utils.hpp"

namespace nav2_monitoring_tree
{

class RecoveryOncePerLocation : public BT::DecoratorNode
{
public:
  RecoveryOncePerLocation(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config),
    has_triggered_(false),
    initialized_(false)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(
        "min_distance", 0.3,
        "Min distance (m) robot must travel before recovery fires again"),
      // Frame ports mirror GetCurrentPose: omit in XML → resolved from ROS param.
      BT::InputPort<std::string>("global_frame",        "Global/map TF frame"),
      BT::InputPort<std::string>("robot_base_frame",    "Robot base TF frame"),
      BT::InputPort<double>(     "transform_tolerance", "TF lookup timeout (s)"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (!initialized_) {
      initialize();
      initialized_ = true;
    }

    // If child is already running, keep ticking it.
    if (child_node_->status() == BT::NodeStatus::RUNNING) {
      const auto child_result = child_node_->executeTick();
      if (child_result == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
      }
      // Child finished — recovery was attempted, let navigation resume.
      return BT::NodeStatus::SUCCESS;
    }

    // Get current robot pose.
    geometry_msgs::msg::PoseStamped current_pose;
    if (!getCurrentPose(current_pose)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[RecoveryOncePerLocation] TF unavailable — allowing recovery.");
      last_trigger_pose_ = current_pose;
      has_triggered_ = true;
      const auto r = child_node_->executeTick();
      return (r == BT::NodeStatus::RUNNING) ? r : BT::NodeStatus::SUCCESS;
    }

    // First stuck event: always run recovery.
    if (!has_triggered_) {
      RCLCPP_INFO(node_->get_logger(),
        "[RecoveryOncePerLocation] First stuck event — triggering recovery.");
      last_trigger_pose_ = current_pose;
      has_triggered_ = true;
      const auto r = child_node_->executeTick();
      return (r == BT::NodeStatus::RUNNING) ? r : BT::NodeStatus::SUCCESS;
    }

    // Distance from last trigger.
    const double dx =
      current_pose.pose.position.x - last_trigger_pose_.pose.position.x;
    const double dy =
      current_pose.pose.position.y - last_trigger_pose_.pose.position.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    if (dist >= min_distance_) {
      RCLCPP_INFO(node_->get_logger(),
        "[RecoveryOncePerLocation] Moved %.2f m ≥ %.2f m — triggering at new location.",
        dist, min_distance_);
      last_trigger_pose_ = current_pose;
      const auto r = child_node_->executeTick();
      return (r == BT::NodeStatus::RUNNING) ? r : BT::NodeStatus::SUCCESS;
    }

    // Same location — skip recovery, keep RecoveryNode(999999) alive.
    RCLCPP_DEBUG(node_->get_logger(),
      "[RecoveryOncePerLocation] Same location (%.2f m < %.2f m) — skipping.",
      dist, min_distance_);
    return BT::NodeStatus::SUCCESS;
  }

private:
  void initialize()
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_   = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

    // Derive namespace-aware defaults so frame names are never hardcoded.
    const std::string ns = config().blackboard->get<std::string>("nav2_namespace");

    // Port → ROS param → namespace-derived fallback.
    global_frame_        = getFromPortOrParam<std::string>(
      node_, "global_frame",     this, defaultGlobalFrame(ns));
    robot_base_frame_    = getFromPortOrParam<std::string>(
      node_, "robot_base_frame", this, defaultRobotBaseFrame(ns));
    transform_tolerance_ = getFromPortOrParam<double>(
      node_, "transform_tolerance", this, 0.1);

    getInput("min_distance", min_distance_);

    RCLCPP_DEBUG(node_->get_logger(),
      "[RecoveryOncePerLocation] global_frame='%s', robot_base_frame='%s', "
      "min_distance=%.2f m",
      global_frame_.c_str(), robot_base_frame_.c_str(), min_distance_);
  }

  bool getCurrentPose(geometry_msgs::msg::PoseStamped & pose)
  {
    try {
      geometry_msgs::msg::TransformStamped tf_stamped =
        tf_->lookupTransform(
          global_frame_, robot_base_frame_,
          tf2::TimePointZero,
          tf2::durationFromSec(transform_tolerance_));

      pose.header              = tf_stamped.header;
      pose.pose.position.x     = tf_stamped.transform.translation.x;
      pose.pose.position.y     = tf_stamped.transform.translation.y;
      pose.pose.position.z     = tf_stamped.transform.translation.z;
      pose.pose.orientation    = tf_stamped.transform.rotation;
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_{0.1};
  double min_distance_{0.3};

  bool has_triggered_;
  geometry_msgs::msg::PoseStamped last_trigger_pose_;
  bool initialized_;
};

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::RecoveryOncePerLocation>(
    "RecoveryOncePerLocation");
}
