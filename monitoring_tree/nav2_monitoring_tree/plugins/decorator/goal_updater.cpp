#include "nav2_monitoring_tree/plugins/decorator/goal_updater.hpp"
#include "nav2_monitoring_tree/nav2_monitor_utils.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_monitoring_tree
{

GoalUpdater::GoalUpdater(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  const std::string ns = config.blackboard->get<std::string>("nav2_namespace");
  const std::string full_topic = ns_topic(ns, "goal_update");

  RCLCPP_INFO(node_->get_logger(), "GoalUpdater subscribing to '%s'", full_topic.c_str());

  sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    full_topic, 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      latest_goal_ = *msg;
      goal_received_ = true;
      RCLCPP_INFO(node_->get_logger(), "GoalUpdater: new goal (%.2f, %.2f)",
        msg->pose.position.x, msg->pose.position.y);
    });
}

BT::PortsList GoalUpdater::providedPorts()
{
  return {
    BT::InputPort<std::string>("input_goal",  "Goal from blackboard"),
    BT::OutputPort<std::string>("output_goal"),
  };
}

BT::NodeStatus GoalUpdater::tick()
{
  rclcpp::spin_some(node_);
  if (goal_received_) {
    const std::string goal_str =
      "(" + std::to_string(latest_goal_.pose.position.x).substr(0, 6)
      + ", " + std::to_string(latest_goal_.pose.position.y).substr(0, 6) + ")";
    setOutput("output_goal", goal_str);
  }
  return child_node_->executeTick();
}

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::GoalUpdater>("GoalUpdater");
}
