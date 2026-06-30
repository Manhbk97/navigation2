// Copyright (c) 2021 Joshua Wallace
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

#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "behavior_plugins/validate_path_action.hpp"

namespace behavior_plugins
{

ValidatePath::ValidatePath(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::IsPathValid>(service_node_name, conf, "is_path_valid")
{
}

void ValidatePath::on_tick()
{
  // Read all optional ports. Missing inputs keep the default values declared in providedPorts().
  getInput<uint16_t>("max_cost", max_cost_);
  getInput<bool>("consider_unknown_as_obstacle", consider_unknown_as_obstacle_);
  getInput<std::string>("layer_name", layer_name_);
  getInput<std::string>("footprint", footprint_);
  getInput<bool>("stop_at_first_collision", stop_at_first_collision_);
  getInput<double>("max_lookahead_distance", max_lookahead_distance_);
  getInput("path", path_);

  request_ = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
  request_->path = path_;
  // .srv field is uint8; clamp before narrowing.
  request_->max_cost = static_cast<uint8_t>(std::min<uint16_t>(max_cost_, 255));
  request_->consider_unknown_as_obstacle = consider_unknown_as_obstacle_;
  request_->layer_name = layer_name_;
  request_->footprint = footprint_;
  request_->stop_at_first_collision = stop_at_first_collision_;
  request_->max_lookahead_distance = max_lookahead_distance_;
}

BT::NodeStatus ValidatePath::on_completion(
  std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
{
  // `success` is the service-call outcome (server reached, footprint parsed, layer
  // exists, ...). When false, the validity result is undefined.
  if (!response->success) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "IsPathValid service failed to validate path");
    return BT::NodeStatus::FAILURE;
  }

  if (response->is_valid) {
    // Clear the collision_poses output when the path is valid.
    std::vector<geometry_msgs::msg::PoseStamped> collision_poses;
    setOutput("collision_poses", collision_poses);
    return BT::NodeStatus::SUCCESS;
  }

  // Map invalid pose indices back to actual poses for downstream consumers.
  std::vector<geometry_msgs::msg::PoseStamped> collision_poses;
  if (!response->invalid_pose_indices.empty()) {
    std::stringstream ss;
    ss << "Path validation failed. Invalid pose indices: [";
    for (size_t i = 0; i < response->invalid_pose_indices.size(); ++i) {
      int32_t idx = response->invalid_pose_indices[i];
      ss << idx;
      if (i + 1 < response->invalid_pose_indices.size()) {
        ss << ", ";
      }
      if (idx >= 0 && static_cast<size_t>(idx) < path_.poses.size()) {
        collision_poses.push_back(path_.poses[idx]);
      }
    }
    ss << "]";
    RCLCPP_WARN(node_->get_logger(), "%s", ss.str().c_str());
  }

  setOutput("collision_poses", collision_poses);

  return BT::NodeStatus::FAILURE;
}

}  // namespace behavior_plugins

// Node registration is centralized in plugin_registration.cpp
