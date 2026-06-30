#ifndef NAV2_MONITORING_TREE__PLUGINS__ACTION__PLANNER_SELECTOR_MONITOR_HPP_
#define NAV2_MONITORING_TREE__PLUGINS__ACTION__PLANNER_SELECTOR_MONITOR_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_monitoring_tree
{

// Subscribes to /<ns>/planner_selector and writes the result to
// the selected_planner output port.
class PlannerSelectorMonitor : public BT::SyncActionNode
{
public:
  PlannerSelectorMonitor(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string selected_;
};

}  // namespace nav2_monitoring_tree
#endif  // NAV2_MONITORING_TREE__PLUGINS__ACTION__PLANNER_SELECTOR_MONITOR_HPP_
