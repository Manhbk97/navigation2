// Copyright (c) 2026 Jakub Chudziński
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

#ifndef BEHAVIOR_PLUGINS__IS_GOAL_NEARBY_CONDITION_HPP_
#define BEHAVIOR_PLUGINS__IS_GOAL_NEARBY_CONDITION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace behavior_plugins
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the remaining length of
 * the current planned path is less than `proximity_threshold` and FAILURE
 * otherwise.
 *
 * Use `max_robot_pose_search_dist` to limit search distance when the path
 * updates regularly to reduce computational cost. Set to a negative value to
 * disable the bound and search the full path every tick.
 *
 * Note: ported to the older nav2_behavior_tree API (no nav2_ros_common).
 *
 * Usage in XML:
 * @code
 * <IsGoalNearby path="{path}" proximity_threshold="1.0" />
 * @endcode
 */
class IsGoalNearbyCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsGoalNearbyCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsGoalNearbyCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  IsGoalNearbyCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Planned path"),
      BT::InputPort<double>(
        "proximity_threshold", 1.0,
        "Proximity length (m) of the remaining path considered as nearby"),
      BT::InputPort<double>(
        "max_robot_pose_search_dist", -1.0,
        "Maximum forward integrated distance along the path "
        "(starting from the last detected pose) to bound the search for the closest pose "
        "to the robot. When set to a negative value (default), the whole path is searched."),
      BT::InputPort<std::string>("global_frame", "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", "Robot base frame"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::Path path_;
  std::vector<geometry_msgs::msg::PoseStamped>::iterator closest_pose_detection_begin_;
  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;
};

}  // namespace behavior_plugins

#endif  // BEHAVIOR_PLUGINS__IS_GOAL_NEARBY_CONDITION_HPP_
