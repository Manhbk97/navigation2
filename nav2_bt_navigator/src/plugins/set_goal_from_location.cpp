#include "nav2_bt_navigator/plugins/set_goal_from_location.hpp"

#include <cmath>
#include <limits>

#include "yaml-cpp/yaml.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_bt_navigator
{

SetGoalFromLocation::SetGoalFromLocation(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Build absolute topic name: /<namespace>/ekf_odom
  std::string ns = node_->get_namespace();
  if (ns == "/") {
    ns = "";
  }
  const std::string topic = ns + "/ekf_odom";

  // Use a dedicated callback group + executor so spin_some() in tick() can
  // process odom messages even though the main node runs a single-threaded
  // executor (the BT tick IS that executor's active callback).
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false /* do not auto-add to the node's main executor */);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&SetGoalFromLocation::odomCallback, this, std::placeholders::_1),
    sub_opts);

  // Warm-up spin required in Jazzy due to rclcpp regression
  callback_group_executor_.spin_some(std::chrono::nanoseconds(1));

  RCLCPP_INFO(node_->get_logger(),
    "SetGoalFromLocation: subscribed to '%s'", topic.c_str());
}

void SetGoalFromLocation::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_position_ = msg->pose.pose.position;
  pose_received_ = true;
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

  // Helper: build a PoseStamped from a YAML entry {x, y, yaw}
  auto makePose = [](const YAML::Node & entry) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = entry["x"].as<double>();
    pose.pose.position.y = entry["y"].as<double>();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, entry["yaw"].as<double>());
    q.normalize();
    pose.pose.orientation = tf2::toMsg(q);
    return pose;
  };

  if (loc == "closest") {
    // Drain any pending odom messages from our dedicated executor
    callback_group_executor_.spin_some();

    if (!pose_received_) {
      RCLCPP_WARN(node_->get_logger(),
        "SetGoalFromLocation: no odometry received yet on ekf_odom, cannot pick closest location");
      return BT::NodeStatus::FAILURE;
    }

    double best_dist = std::numeric_limits<double>::max();
    YAML::Node best_entry;
    std::string best_name;

    for (auto it = root["locations"].begin(); it != root["locations"].end(); ++it) {
      auto entry = it->second;
      if (!entry["x"] || !entry["y"] || !entry["yaw"]) {
        continue;
      }
      const double dx = entry["x"].as<double>() - robot_position_.x;
      const double dy = entry["y"].as<double>() - robot_position_.y;
      const double dist = std::sqrt(dx * dx + dy * dy);
      // RCLCPP_INFO(node_->get_logger(),
      //   "SetGoalFromLocation: location '%s' -> dist=%.3f m (x=%.2f, y=%.2f)",
      //   it->first.as<std::string>().c_str(), dist,
      //   entry["x"].as<double>(), entry["y"].as<double>());
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
    // Named location – existing behaviour
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

}  // namespace nav2_bt_navigator

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_navigator::SetGoalFromLocation>("SetGoalFromLocation");
}
