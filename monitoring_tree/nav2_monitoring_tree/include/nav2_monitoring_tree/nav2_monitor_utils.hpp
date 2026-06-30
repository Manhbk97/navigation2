#ifndef NAV2_MONITORING_TREE__NAV2_MONITOR_UTILS_HPP_
#define NAV2_MONITORING_TREE__NAV2_MONITOR_UTILS_HPP_

#include <string>
#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_monitoring_tree
{

// Build a fully-qualified topic name with the Nav2 namespace.
//   ns_topic("apple", "compute_path_to_pose/_action/status")
//     → "/apple/compute_path_to_pose/_action/status"
//   ns_topic("",      "compute_path_to_pose/_action/status")
//     → "/compute_path_to_pose/_action/status"
inline std::string ns_topic(const std::string & ns, const std::string & topic)
{
  const std::string t = (!topic.empty() && topic.front() == '/') ? topic.substr(1) : topic;
  return ns.empty() ? ("/" + t) : ("/" + ns + "/" + t);
}

// Translate the latest ROS2 action goal status → BT NodeStatus.
inline BT::NodeStatus actionStatusToNodeStatus(
  const action_msgs::msg::GoalStatusArray::SharedPtr & msg)
{
  if (!msg || msg->status_list.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  using S = action_msgs::msg::GoalStatus;
  const uint8_t s = msg->status_list.back().status;
  if (s == S::STATUS_EXECUTING || s == S::STATUS_ACCEPTED) {
    return BT::NodeStatus::RUNNING;
  }
  if (s == S::STATUS_SUCCEEDED) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// ── TF frame name helpers ─────────────────────────────────────────────────────
//
// When Nav2 runs under a namespace (e.g. "apple"), ROS2 prefixes every TF
// frame it publishes with that namespace:
//   global frame  → "map"              (always unprefixed — TF world frame)
//   robot frame   → "apple/base_footprint"
//
// These helpers derive correct default frame names from the namespace so that
// no hardcoded robot-frame strings are needed anywhere in the plugins.

// Default TF frame names derived from the Nav2 namespace.
//
// When Nav2 runs under a namespace (e.g. "apple"):
//   global frame  → "map"               (world frame, never namespaced)
//   robot frame   → "apple/base_footprint"  (robot frame, always namespaced)

inline std::string defaultGlobalFrame(const std::string & /*ns*/)
{
  return "map";  // TF world frame is always global — never namespace-prefixed
}

inline std::string defaultRobotBaseFrame(const std::string & ns)
{
  return ns.empty() ? "base_footprint" : (ns + "/base_footprint");
}

// ── BTcpp v3 equivalent of BT::deconflictPortAndParamFrame ───────────────────
//
// Resolution order (most-specific wins):
//   1. BT InputPort  (set as XML attribute on this node)
//   2. ROS node parameter  (declared by runner or launch file)
//   3. `fallback` supplied by the caller  (use the ns-aware helpers above)
//
// Usage inside a BT node's initialize():
//   const std::string ns = config().blackboard->get<std::string>("nav2_namespace");
//   global_frame_     = getFromPortOrParam(node_, "global_frame",     this,
//                                          defaultGlobalFrame());
//   robot_base_frame_ = getFromPortOrParam(node_, "robot_base_frame",  this,
//                                          defaultRobotBaseFrame(ns));
template<typename T>
inline T getFromPortOrParam(
  rclcpp::Node::SharedPtr node,
  const std::string & key,
  BT::TreeNode * self,
  const T & fallback)
{
  // 1. BT port (XML attribute)
  auto port_res = self->getInput<T>(key);
  if (port_res) {
    return port_res.value();
  }
  // 2. ROS node parameter (set by user via --ros-args or launch file)
  T value;
  if (node->has_parameter(key)) {
    node->get_parameter(key, value);
    return value;
  }
  // 3. Namespace-derived or hard fallback
  return fallback;
}

// String specialisation: additionally skip empty-string parameter values so
// that a mis-typed `-p global_frame:=` does not override the derived default.
template<>
inline std::string getFromPortOrParam<std::string>(
  rclcpp::Node::SharedPtr node,
  const std::string & key,
  BT::TreeNode * self,
  const std::string & fallback)
{
  auto port_res = self->getInput<std::string>(key);
  if (port_res && !port_res.value().empty()) {
    return port_res.value();
  }
  if (node->has_parameter(key)) {
    std::string value;
    node->get_parameter(key, value);
    if (!value.empty()) {
      return value;
    }
  }
  return fallback;
}

}  // namespace nav2_monitoring_tree
#endif  // NAV2_MONITORING_TREE__NAV2_MONITOR_UTILS_HPP_
