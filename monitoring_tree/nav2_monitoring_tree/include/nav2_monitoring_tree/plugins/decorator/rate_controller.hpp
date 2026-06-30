#ifndef NAV2_MONITORING_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_
#define NAV2_MONITORING_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_monitoring_tree
{

// Limits child ticks to `hz` Hz using rclcpp::Time from the blackboard node.
class RateController : public BT::DecoratorNode
{
public:
  RateController(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  double hz_;
  rclcpp::Time last_tick_time_;
  BT::NodeStatus last_child_status_;
};

}  // namespace nav2_monitoring_tree
#endif  // NAV2_MONITORING_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_
