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

#ifndef NAV2_BT_NAVIGATOR__PLUGINS__HEADING_CORRECTION_HPP_
#define NAV2_BT_NAVIGATOR__PLUGINS__HEADING_CORRECTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_bt_navigator
{

/**
 * @brief Stateful BT action that rotates the robot on the spot until its
 *        heading matches the goal orientation.
 *
 * Intended use (near-goal recovery): when the robot is close to the goal
 * (< goal_proximity m) but the controller can no longer drive the final leg
 * because an obstacle occupies the goal cell, the near-goal recovery branch
 * ticks this node. It commands an in-place rotation (proportional angular
 * velocity, clamped to [min_angular_speed, max_angular_speed]) on
 * `cmd_vel_topic` until the heading error drops below `yaw_goal_tolerance`,
 * then publishes a zero Twist and returns SUCCESS.
 *
 * Returns FAILURE immediately (without commanding any motion) if the goal
 * input is missing, the robot pose is unavailable, or the robot is farther
 * than `goal_proximity` from the goal.
 *
 * XML usage:
 *   <HeadingCorrection goal="{goal}" yaw_goal_tolerance="0.1"
 *                      max_angular_speed="0.5" min_angular_speed="0.15"
 *                      goal_proximity="1.5" cmd_vel_topic="cmd_vel"/>
 */
class HeadingCorrection : public BT::StatefulActionNode
{
public:
  HeadingCorrection(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  HeadingCorrection() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Goal pose whose orientation the robot should face"),
      BT::InputPort<double>(
        "yaw_goal_tolerance", 0.1,
        "Heading error (rad) below which the action succeeds"),
      BT::InputPort<double>(
        "max_angular_speed", 0.5, "Max rotational velocity (rad/s)"),
      BT::InputPort<double>(
        "min_angular_speed", 0.15,
        "Min rotational velocity (rad/s), so the robot keeps turning near the target"),
      BT::InputPort<double>(
        "goal_proximity", 1.5,
        "Only correct heading when the robot is within this distance (m) of the goal"),
      BT::InputPort<std::string>(
        "cmd_vel_topic", "cmd_vel",
        "Velocity command topic (relative name; namespace is applied "
        "automatically, e.g. /<namespace>/cmd_vel). Do not prefix with '/'."),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void initialize();
  void stopRobot();
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_{0.1};

  // Per-run parameters (re-read from ports on every onStart()).
  double yaw_goal_tolerance_{0.1};
  double max_angular_speed_{0.5};
  double min_angular_speed_{0.15};
  double goal_proximity_{1.5};
  double goal_yaw_{0.0};

  bool initialized_{false};
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__PLUGINS__HEADING_CORRECTION_HPP_
