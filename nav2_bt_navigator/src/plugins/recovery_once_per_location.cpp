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
#include "nav2_bt_navigator/plugins/recovery_once_per_location.hpp"

namespace nav2_bt_navigator
{

RecoveryOncePerLocation::RecoveryOncePerLocation(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
}

void RecoveryOncePerLocation::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_   = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "global_frame", this);
  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "robot_base_frame", this);

  getInput("min_distance", min_distance_);

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[RecoveryOncePerLocation] initialized: min_distance=%.2f m, "
    "global_frame=%s, robot_frame=%s",
    min_distance_, global_frame_.c_str(), robot_base_frame_.c_str());
}

BT::NodeStatus RecoveryOncePerLocation::tick()
{
  // One-time parameter initialisation on the very first tick.
  if (!initialized_) {
    initialize();
    initialized_ = true;
  }

  // ── If the child is already executing, keep ticking it. ──────────────────
  // This handles multi-tick recovery actions (Spin, Wait, BackUp) correctly:
  // we must not re-evaluate the distance condition while they are in progress.
  // When the child finishes (SUCCESS or FAILURE), return SUCCESS regardless —
  // the decorator's contract is "I attempted recovery at this location once",
  // not "recovery must succeed". Returning SUCCESS keeps RecoveryNode(999999)
  // alive so navigation keeps replanning.
  if (child_node_->status() == BT::NodeStatus::RUNNING) {
    const auto child_result = child_node_->executeTick();
    if (child_result == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    // Child finished (SUCCESS or FAILURE) — recovery was attempted, move on.
    return BT::NodeStatus::SUCCESS;
  }

  // ── Get current robot pose ────────────────────────────────────────────────
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    // TF unavailable – allow recovery to run (fail-open).
    RCLCPP_WARN(
      node_->get_logger(),
      "[RecoveryOncePerLocation] Cannot get robot pose from TF, "
      "allowing recovery to run.");
    last_trigger_pose_ = current_pose;
    has_triggered_ = true;
    const auto r = child_node_->executeTick();
    return (r == BT::NodeStatus::RUNNING) ? r : BT::NodeStatus::SUCCESS;
  }

  // ── Decide whether to trigger ─────────────────────────────────────────────
  if (!has_triggered_) {
    // Very first stuck event: always run recovery.
    RCLCPP_INFO(
      node_->get_logger(),
      "[RecoveryOncePerLocation] First stuck event – triggering recovery.");
    last_trigger_pose_ = current_pose;
    has_triggered_ = true;
    const auto r = child_node_->executeTick();
    return (r == BT::NodeStatus::RUNNING) ? r : BT::NodeStatus::SUCCESS;
  }

  const double dist = nav2_util::geometry_utils::euclidean_distance(
    last_trigger_pose_, current_pose);

  if (dist >= min_distance_) {
    // Robot moved to a genuinely new location: run recovery.
    RCLCPP_INFO(
      node_->get_logger(),
      "[RecoveryOncePerLocation] Robot moved %.2f m (>= %.2f m) – "
      "triggering recovery at new location.",
      dist, min_distance_);
    last_trigger_pose_ = current_pose;
    const auto r = child_node_->executeTick();
    return (r == BT::NodeStatus::RUNNING) ? r : BT::NodeStatus::SUCCESS;
  }

  // ── Same location as last recovery: skip ─────────────────────────────────
  // Return SUCCESS so the outer RecoveryNode(999999) stays alive and keeps
  // retrying navigation without doing a useless recovery at the same spot.
  RCLCPP_DEBUG(
    node_->get_logger(),
    "[RecoveryOncePerLocation] Same location (%.2f m < %.2f m) – "
    "skipping recovery, navigation will retry replanning.",
    dist, min_distance_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_navigator

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_navigator::RecoveryOncePerLocation>(
    "RecoveryOncePerLocation");
}
