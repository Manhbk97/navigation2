#ifndef NAV2_MONITORING_TREE__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_MONITOR_HPP_
#define NAV2_MONITORING_TREE__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_MONITOR_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_monitoring_tree
{

// Monitors /<ns>/compute_path_to_pose/_action/status and reflects
// the Nav2 planner action state as a BT NodeStatus.
class ComputePathToPoseMonitor : public BT::StatefulActionNode
{
public:
  ComputePathToPoseMonitor(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub_;
  action_msgs::msg::GoalStatusArray::SharedPtr last_status_;
};

}  // namespace nav2_monitoring_tree
#endif  // NAV2_MONITORING_TREE__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_MONITOR_HPP_
