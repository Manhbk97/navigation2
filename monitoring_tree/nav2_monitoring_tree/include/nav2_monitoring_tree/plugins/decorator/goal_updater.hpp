#ifndef NAV2_MONITORING_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_HPP_
#define NAV2_MONITORING_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_monitoring_tree
{

// Subscribes to /<ns>/goal_update and writes the latest goal string to
// the output_goal port. Always ticks its child.
class GoalUpdater : public BT::DecoratorNode
{
public:
  GoalUpdater(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  geometry_msgs::msg::PoseStamped latest_goal_;
  bool goal_received_{false};
};

}  // namespace nav2_monitoring_tree
#endif  // NAV2_MONITORING_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_HPP_
