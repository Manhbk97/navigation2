#ifndef NAV2_MONITORING_TREE__PLUGINS__ACTION__FOLLOW_PATH_MONITOR_HPP_
#define NAV2_MONITORING_TREE__PLUGINS__ACTION__FOLLOW_PATH_MONITOR_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_monitoring_tree
{

// Monitors /<ns>/follow_path/_action/status and reflects
// the Nav2 controller action state as a BT NodeStatus.
class FollowPathMonitor : public BT::StatefulActionNode
{
public:
  FollowPathMonitor(const std::string & name, const BT::NodeConfiguration & config);

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
#endif  // NAV2_MONITORING_TREE__PLUGINS__ACTION__FOLLOW_PATH_MONITOR_HPP_
