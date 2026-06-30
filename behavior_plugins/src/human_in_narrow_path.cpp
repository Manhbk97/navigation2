#include "behavior_plugins/human_in_narrow_path.hpp"

namespace behavior_plugins
{

HumanInNarrowPath::HumanInNarrowPath(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  // Publisher is created at tree-load time so the topic is always visible,
  // even when this branch is never ticked (e.g. FollowPath keeps running).
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  status_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/human_in_narrow_path_active",
    rclcpp::SystemDefaultsQoS());
}

HumanInNarrowPath::~HumanInNarrowPath()
{
  if (initialized_) {
    callback_group_executor_.cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }
}

void HumanInNarrowPath::initialize()
{
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  human_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/human_detected",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&HumanInNarrowPath::humanCallback, this, std::placeholders::_1),
    sub_option);

  narrow_path_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/narrow_path_detected",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&HumanInNarrowPath::narrowPathCallback, this, std::placeholders::_1),
    sub_option);

  // Spin the executor in a dedicated thread so callbacks fire immediately
  // when messages arrive, independent of the BT tick rate.
  executor_thread_ = std::thread([this]() {
    callback_group_executor_.spin();
  });

  initialized_ = true;
}

BT::NodeStatus HumanInNarrowPath::tick()
{
  if (!initialized_) {
    initialize();
  }

  return (human_detected_ && narrow_path_detected_)
    ? BT::NodeStatus::SUCCESS
    : BT::NodeStatus::FAILURE;
}

void HumanInNarrowPath::publishStatus()
{
  std_msgs::msg::Bool status_msg;
  status_msg.data = human_detected_ && narrow_path_detected_;
  status_pub_->publish(status_msg);
}

void HumanInNarrowPath::humanCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  human_detected_ = msg->data;
  publishStatus();
}

void HumanInNarrowPath::narrowPathCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  narrow_path_detected_ = msg->data;
  publishStatus();
}


}  // namespace behavior_plugins