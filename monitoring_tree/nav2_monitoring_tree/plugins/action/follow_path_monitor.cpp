#include "nav2_monitoring_tree/plugins/action/follow_path_monitor.hpp"
#include "nav2_monitoring_tree/nav2_monitor_utils.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_monitoring_tree
{

FollowPathMonitor::FollowPathMonitor(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  const std::string ns = config.blackboard->get<std::string>("nav2_namespace");
  const std::string full_topic = ns_topic(ns, "follow_path/_action/status");

  RCLCPP_INFO(node_->get_logger(), "FollowPath subscribing to '%s'", full_topic.c_str());

  sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
    full_topic, 10,
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      last_status_ = msg;
    });
}

BT::PortsList FollowPathMonitor::providedPorts()
{
  return {
    BT::InputPort<std::string>("controller_id"),
    BT::InputPort<std::string>("path"),
    BT::InputPort<std::string>("goal_checker_id"),
    BT::InputPort<std::string>("progress_checker_id"),
    BT::InputPort<std::string>("server_name"),
    BT::InputPort<std::string>("server_timeout"),
    BT::OutputPort<std::string>("error_code_id"),
  };
}

BT::NodeStatus FollowPathMonitor::onStart()
{
  return onRunning();
}

BT::NodeStatus FollowPathMonitor::onRunning()
{
  rclcpp::spin_some(node_);
  return actionStatusToNodeStatus(last_status_);
}

void FollowPathMonitor::onHalted()
{
  last_status_.reset();
}

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::FollowPathMonitor>("FollowPath");
}
