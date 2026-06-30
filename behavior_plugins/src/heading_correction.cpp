// Copyright (c) 2025
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behavior_plugins/heading_correction.hpp"

#include <algorithm>
#include <cmath>

namespace behavior_plugins
{

namespace
{
// Yaw of a quaternion that is assumed to encode a rotation about +Z only,
// but also correct for a general quaternion (atan2 form).
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Wrap an angle to [-pi, pi].
double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

HeadingCorrection::HeadingCorrection(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
}

void HeadingCorrection::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "global_frame", this);
  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "robot_base_frame", this);

  // Keep this a RELATIVE topic name (e.g. "cmd_vel", no leading '/'): the
  // bt_navigator node runs inside the robot namespace, so it resolves to
  // /<namespace>/cmd_vel — the same topic collision_monitor publishes on.
  // A leading '/' would make it global and bypass the namespace.
  std::string cmd_vel_topic = "cmd_vel";
  getInput("cmd_vel_topic", cmd_vel_topic);
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::SystemDefaultsQoS());

  initialized_ = true;

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[HeadingCorrection] initialized: cmd_vel_topic=%s, global_frame=%s, "
    "robot_frame=%s, transform_tolerance=%.2f",
    cmd_vel_topic.c_str(), global_frame_.c_str(),
    robot_base_frame_.c_str(), transform_tolerance_);
}

bool HeadingCorrection::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  return nav2_util::getCurrentPose(
    pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_);
}

void HeadingCorrection::stopRobot()
{
  if (vel_pub_) {
    geometry_msgs::msg::Twist twist;  // all-zero
    vel_pub_->publish(twist);
  }
}

BT::NodeStatus HeadingCorrection::onStart()
{
  if (!initialized_) {
    initialize();
  }

  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[HeadingCorrection] missing required input port 'goal'");
    return BT::NodeStatus::FAILURE;
  }

  getInput("yaw_goal_tolerance", yaw_goal_tolerance_);
  getInput("max_angular_speed", max_angular_speed_);
  getInput("min_angular_speed", min_angular_speed_);
  getInput("goal_proximity", goal_proximity_);

  goal_yaw_ = yawFromQuaternion(goal.pose.orientation);

  // Distance gate: only spin in place when we are genuinely near the goal.
  geometry_msgs::msg::PoseStamped current_pose;
  if (!getRobotPose(current_pose)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[HeadingCorrection] cannot get robot pose from TF - skipping heading correction");
    return BT::NodeStatus::FAILURE;
  }

  const double dist =
    nav2_util::geometry_utils::euclidean_distance(current_pose, goal);
  if (dist > goal_proximity_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[HeadingCorrection] robot is %.2f m from goal (> %.2f m) - "
      "skipping heading correction",
      dist, goal_proximity_);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[HeadingCorrection] obstacle near goal, controller idle - "
    "rotating in place to goal heading %.3f rad (dist %.2f m)",
    goal_yaw_, dist);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HeadingCorrection::onRunning()
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!getRobotPose(current_pose)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[HeadingCorrection] lost robot pose while correcting heading - stopping");
    stopRobot();
    return BT::NodeStatus::FAILURE;
  }

  const double current_yaw = yawFromQuaternion(current_pose.pose.orientation);
  const double yaw_error = normalizeAngle(goal_yaw_ - current_yaw);

  if (std::fabs(yaw_error) <= yaw_goal_tolerance_) {
    stopRobot();
    RCLCPP_INFO(
      node_->get_logger(),
      "[HeadingCorrection] heading aligned to goal (error %.3f rad)", yaw_error);
    return BT::NodeStatus::SUCCESS;
  }

  // Proportional control, clamped so the robot keeps turning right up to the
  // tolerance band and never exceeds the configured max.
  double cmd = 1.5 * yaw_error;
  const double sign = (cmd >= 0.0) ? 1.0 : -1.0;
  cmd = sign * std::clamp(std::fabs(cmd), min_angular_speed_, max_angular_speed_);

  geometry_msgs::msg::Twist twist;
  twist.angular.z = cmd;
  vel_pub_->publish(twist);

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[HeadingCorrection] yaw_error=%.3f rad, cmd_w=%.3f rad/s", yaw_error, cmd);
  return BT::NodeStatus::RUNNING;
}

void HeadingCorrection::onHalted()
{
  stopRobot();
  RCLCPP_DEBUG(node_->get_logger(), "[HeadingCorrection] halted - robot stopped");
}

}  // namespace behavior_plugins
