#include "behavior_plugins/narrow_path.hpp"

namespace behavior_plugins
{
NarrowPath::NarrowPath(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  status_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/narrow_path_detected_active",
    rclcpp::SystemDefaultsQoS());    
}

NarrowPath::~NarrowPath()
{
  if (initialized_) {
    callback_group_executor_.cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }
}

void NarrowPath::initialize()
{
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  narrow_path_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/narrow_path",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NarrowPath::narrowPathCallback, this, std::placeholders::_1),
    sub_option);

  executor_thread_ = std::thread([this]() {
    callback_group_executor_.spin();
  });

  initialized_ = true;
}

BT::NodeStatus NarrowPath::tick()
{
  if (!initialized_) {
    initialize();
  }
  publishStatus();
  return narrow_path_detected_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void NarrowPath::publishStatus()
{
  std_msgs::msg::Bool msg;
  msg.data = narrow_path_detected_;
  status_pub_->publish(msg);
}

void NarrowPath::narrowPathCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  narrow_path_detected_ = msg->data;
  publishStatus();
}

} // namespace behavior_plugins
