// GetCurrentPose: TF lookup from robot_base_frame → global_frame.
//
// Frame parameters are resolved in this order (most-specific wins):
//   1. BT InputPort  (per-node XML attribute)
//   2. ROS node parameter  (declared by the runner / launch file)
//   3. Built-in fallback: global_frame="map", robot_base_frame="base_link"
//
// This mirrors the BTcpp v4 BT::deconflictPortAndParamFrame pattern, but is
// implemented in plain BTcpp v3.
#include <memory>
#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_monitoring_tree/nav2_monitor_utils.hpp"

namespace nav2_monitoring_tree
{

class GetCurrentPose : public BT::SyncActionNode
{
public:
  GetCurrentPose(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_   = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

    // Derive namespace-aware defaults so frame names are never hardcoded.
    // When Nav2 runs as namespace "apple", TF publishes "apple/base_footprint".
    const std::string ns = config.blackboard->get<std::string>("nav2_namespace");

    // Resolution: XML port → ROS param → namespace-derived fallback.
    global_frame_        = getFromPortOrParam<std::string>(
      node_, "global_frame",     this, defaultGlobalFrame(ns));
    robot_base_frame_    = getFromPortOrParam<std::string>(
      node_, "robot_base_frame", this, defaultRobotBaseFrame(ns));
    transform_tolerance_ = getFromPortOrParam<double>(
      node_, "transform_tolerance", this, 0.1);

    RCLCPP_INFO(
      node_->get_logger(),
      "GetCurrentPose: global_frame='%s', robot_base_frame='%s', tolerance=%.2f s",
      global_frame_.c_str(), robot_base_frame_.c_str(), transform_tolerance_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      // Frame ports are optional — omit them in the XML and the ROS parameter
      // (or the built-in fallback) will be used instead.
      BT::InputPort<std::string>("global_frame",        "Global/map TF frame"),
      BT::InputPort<std::string>("robot_base_frame",    "Robot base TF frame"),
      BT::InputPort<double>(     "transform_tolerance", "TF lookup timeout (s)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        global_frame_, robot_base_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "GetCurrentPose: TF lookup failed: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header                = transform.header;
    pose.pose.position.x       = transform.transform.translation.x;
    pose.pose.position.y       = transform.transform.translation.y;
    pose.pose.position.z       = transform.transform.translation.z;
    pose.pose.orientation      = transform.transform.rotation;

    setOutput("current_pose", pose);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
};

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::GetCurrentPose>("GetCurrentPose");
}
