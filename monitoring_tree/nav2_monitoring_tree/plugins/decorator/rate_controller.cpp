#include "nav2_monitoring_tree/plugins/decorator/rate_controller.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_monitoring_tree
{

RateController::RateController(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config),
  last_child_status_(BT::NodeStatus::RUNNING)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  hz_ = getInput<double>("hz").value_or(1.0);
  last_tick_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

BT::PortsList RateController::providedPorts()
{
  return {BT::InputPort<double>("hz", 1.0, "Tick rate in Hz")};
}

BT::NodeStatus RateController::tick()
{
  const double period_sec = 1.0 / hz_;
  if ((node_->now() - last_tick_time_).seconds() >= period_sec) {
    last_tick_time_ = node_->now();
    last_child_status_ = child_node_->executeTick();
  }
  return last_child_status_;
}

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::RateController>("RateController");
}
