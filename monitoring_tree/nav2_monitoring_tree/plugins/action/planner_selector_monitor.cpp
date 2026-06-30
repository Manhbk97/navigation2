#include "nav2_monitoring_tree/plugins/action/planner_selector_monitor.hpp"
#include "nav2_monitoring_tree/nav2_monitor_utils.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_monitoring_tree
{

PlannerSelectorMonitor::PlannerSelectorMonitor(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  const std::string ns = config.blackboard->get<std::string>("nav2_namespace");
  const auto port_topic = getInput<std::string>("topic_name").value_or("planner_selector");
  const std::string full_topic = ns_topic(ns, port_topic);

  RCLCPP_INFO(node_->get_logger(), "PlannerSelector subscribing to '%s'", full_topic.c_str());

  sub_ = node_->create_subscription<std_msgs::msg::String>(
    full_topic, rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      selected_ = msg->data;
      RCLCPP_INFO(node_->get_logger(), "PlannerSelector: '%s'", selected_.c_str());
    });
}

BT::PortsList PlannerSelectorMonitor::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic_name", "planner_selector", "Relative topic name"),
    BT::InputPort<std::string>("default_planner", "GridBased", "Default planner"),
    BT::OutputPort<std::string>("selected_planner"),
  };
}

BT::NodeStatus PlannerSelectorMonitor::tick()
{
  rclcpp::spin_some(node_);
  const auto def = getInput<std::string>("default_planner").value_or("GridBased");
  setOutput("selected_planner", selected_.empty() ? def : selected_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::PlannerSelectorMonitor>("PlannerSelector");
}
