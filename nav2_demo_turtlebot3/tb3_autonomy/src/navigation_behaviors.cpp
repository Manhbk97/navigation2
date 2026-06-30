#include "navigation_behaviors.h"
#include "yaml-cpp/yaml.h"
#include <string>

GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/apple/navigate_to_pose");
  done_flag_ = false;
  goal_accepted_ = false;
}

BT::PortsList GoToPose::providedPorts()
{
  return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GoToPose::onStart()
{
  // Wait for action server to be available
  if (!action_client_ptr_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Action server /apple/navigate_to_pose not available!");
    return BT::NodeStatus::FAILURE;
  }

  // Get location key from port and read YAML file
  BT::Optional<std::string> loc = getInput<std::string>("loc");
  const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

  YAML::Node locations = YAML::LoadFile(location_file);

  std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

  // setup action client with both goal_response and result callbacks
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&GoToPose::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

  // make pose
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node_ptr_->now();
  goal_msg.pose.pose.position.x = pose[0];
  goal_msg.pose.pose.position.y = pose[1];

  tf2::Quaternion q;
  q.setRPY(0, 0, pose[2]);
  q.normalize();
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  // send pose
  done_flag_ = false;
  goal_accepted_ = false;
  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2 (x=%.2f, y=%.2f, theta=%.2f)\n",
              pose[0], pose[1], pose[2]);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
  if (done_flag_)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name());
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void GoToPose::goal_response_callback(const GoalHandleNav::SharedPtr &goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Goal was rejected by Nav2 server");
    goal_accepted_ = false;
  }
  else
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Goal accepted by Nav2 server");
    goal_accepted_ = true;
  }
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_ptr_->get_logger(), "Navigation succeeded!");
      done_flag_ = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_ptr_->get_logger(), "Navigation was aborted");
      done_flag_ = true;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_ptr_->get_logger(), "Navigation was canceled");
      done_flag_ = true;
      break;
    default:
      RCLCPP_ERROR(node_ptr_->get_logger(), "Unknown navigation result code");
      done_flag_ = true;
      break;
  }
}