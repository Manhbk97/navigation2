#include "behavior_plugins/set_goal_from_location.hpp"

#include <cmath>
#include <limits>

#include "yaml-cpp/yaml.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"

namespace behavior_plugins
{

SetGoalFromLocation::SetGoalFromLocation(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Reuse bt_navigator's frame names if it declared them, otherwise fall
  // back to the nav2 defaults. This keeps the plugin free of any topic
  // or namespace assumptions.
  auto get_or_default = [this](const std::string & key, const std::string & fallback) {
    if (node_->has_parameter(key)) {
      return node_->get_parameter(key).as_string();
    }
    return fallback;
  };
  global_frame_ = get_or_default("global_frame", "map");
  robot_base_frame_ = get_or_default("robot_base_frame", "base_link");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(node_->get_logger(),
    "SetGoalFromLocation: using TF %s -> %s for closest-location lookup",
    global_frame_.c_str(), robot_base_frame_.c_str());
}

BT::NodeStatus SetGoalFromLocation::tick()
{
  std::string location_file, loc;
  if (!getInput("location_file", location_file) || !getInput("loc", loc)) {
    return BT::NodeStatus::FAILURE;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(location_file);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(),
      "SetGoalFromLocation: failed to load '%s': %s", location_file.c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  if (!root["locations"]) {
    RCLCPP_ERROR(node_->get_logger(),
      "SetGoalFromLocation: no 'locations' key in '%s'", location_file.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto makePose = [this](const YAML::Node & entry) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = entry["x"].as<double>();
    pose.pose.position.y = entry["y"].as<double>();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, entry["yaw"].as<double>());
    q.normalize();
    pose.pose.orientation = tf2::toMsg(q);
    return pose;
  };

  if (loc == "closest") {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(node_->get_logger(),
        "SetGoalFromLocation: no TF %s -> %s yet (%s), cannot pick closest location",
        global_frame_.c_str(), robot_base_frame_.c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }

    const double rx = tf.transform.translation.x;
    const double ry = tf.transform.translation.y;

    double best_dist = std::numeric_limits<double>::max();
    YAML::Node best_entry;
    std::string best_name;

    for (auto it = root["locations"].begin(); it != root["locations"].end(); ++it) {
      auto entry = it->second;
      if (!entry["x"] || !entry["y"] || !entry["yaw"]) {
        continue;
      }
      const double dx = entry["x"].as<double>() - rx;
      const double dy = entry["y"].as<double>() - ry;
      const double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < best_dist) {
        best_dist = dist;
        best_entry = entry;
        best_name = it->first.as<std::string>();
      }
    }

    if (!best_entry.IsDefined()) {
      RCLCPP_ERROR(node_->get_logger(),
        "SetGoalFromLocation: no valid locations found in '%s'", location_file.c_str());
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(),
      "SetGoalFromLocation: closest location is '%s' (dist=%.2f m)",
      best_name.c_str(), best_dist);

    setOutput("goal", makePose(best_entry));
    return BT::NodeStatus::SUCCESS;

  } else {
    if (!root["locations"][loc]) {
      RCLCPP_ERROR(node_->get_logger(),
        "SetGoalFromLocation: location '%s' not found in '%s'",
        loc.c_str(), location_file.c_str());
      return BT::NodeStatus::FAILURE;
    }
    const auto & entry = root["locations"][loc];
    if (!entry["x"] || !entry["y"] || !entry["yaw"]) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("goal", makePose(entry));
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace behavior_plugins
