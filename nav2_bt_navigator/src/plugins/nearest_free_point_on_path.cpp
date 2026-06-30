#include "nav2_bt_navigator/plugins/nearest_free_point_on_path.hpp"

#include <cmath>
#include <algorithm>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_bt_navigator
{

NearestFreePointOnPath::NearestFreePointOnPath(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Build namespace-aware default for the costmap topic
  std::string ns = node_->get_namespace();
  if (ns == "/") {
    ns = "";
  }
  const std::string default_costmap_topic = ns + "/local_costmap/costmap";

  // Helper: declare parameter only when it does not exist yet
  auto decl = [&](const std::string & full_name, const rclcpp::ParameterValue & val) {
    if (!node_->has_parameter(full_name)) {
      node_->declare_parameter(full_name, val);
    }
  };

  const std::string P = "nearest_free_point_on_path.";

  // Inherit global_frame and robot_base_frame from the bt_navigator node
  std::string default_map_frame   = "map";
  std::string default_robot_frame = ns.empty() ? "base_footprint" : ns.substr(1) + "/base_footprint";
  if (node_->has_parameter("global_frame")) {
    default_map_frame = node_->get_parameter("global_frame").as_string();
  }
  if (node_->has_parameter("robot_base_frame")) {
    default_robot_frame = node_->get_parameter("robot_base_frame").as_string();
  }

  // Declare parameters with defaults
  decl(P + "search_radius",           rclcpp::ParameterValue(1.5));
  decl(P + "min_distance_from_goal",  rclcpp::ParameterValue(1.0));
  decl(P + "free_threshold",          rclcpp::ParameterValue(50));
  decl(P + "step_size",               rclcpp::ParameterValue(0.05));
  decl(P + "safety_distance",         rclcpp::ParameterValue(0.30));
  decl(P + "robot_frame",             rclcpp::ParameterValue(default_robot_frame));
  decl(P + "map_frame",               rclcpp::ParameterValue(default_map_frame));
  decl(P + "costmap_topic",           rclcpp::ParameterValue(default_costmap_topic));
  decl(P + "tf_timeout",              rclcpp::ParameterValue(0.5));

  // Get parameters
  search_radius_          = node_->get_parameter(P + "search_radius").as_double();
  min_distance_from_goal_ = node_->get_parameter(P + "min_distance_from_goal").as_double();
  free_threshold_         = static_cast<int>(node_->get_parameter(P + "free_threshold").as_int());
  step_size_              = node_->get_parameter(P + "step_size").as_double();
  safety_distance_        = node_->get_parameter(P + "safety_distance").as_double();
  robot_frame_            = node_->get_parameter(P + "robot_frame").as_string();
  map_frame_              = node_->get_parameter(P + "map_frame").as_string();
  costmap_topic_          = node_->get_parameter(P + "costmap_topic").as_string();
  tf_timeout_             = node_->get_parameter(P + "tf_timeout").as_double();

  // Sanity check: min must be < max, otherwise no point can ever be selected
  if (min_distance_from_goal_ >= search_radius_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "NearestFreePointOnPath: min_distance_from_goal (%.2f) >= search_radius (%.2f); "
      "no points can satisfy the range. Resetting min_distance_from_goal to 0.0.",
      min_distance_from_goal_, search_radius_);
    min_distance_from_goal_ = 0.0;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "NearestFreePointOnPath: search_radius=%.2f m, min_distance=%.2f m, "
    "step_size=%.3f m, threshold=%d, safety_distance=%.2f m, "
    "topic=%s, map_frame=%s",
    search_radius_, min_distance_from_goal_, step_size_,
    free_threshold_, safety_distance_, costmap_topic_.c_str(), map_frame_.c_str());
}

// ---------------------------------------------------------------------------
// Lazy initialisation (called on first tick)
// ---------------------------------------------------------------------------
void NearestFreePointOnPath::initialize()
{
  // Prefer the shared TF buffer placed on the BT blackboard
  try {
    tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  } catch (const std::exception &) {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, node_, false /* do not spin */);
    RCLCPP_INFO(
      node_->get_logger(),
      "NearestFreePointOnPath: 'tf_buffer' not found on blackboard, created own TF listener.");
  }

  // Dedicated callback group for costmap subscription
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false /* do not auto-add to main executor */);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_,
    rclcpp::QoS(1).transient_local(),
    std::bind(&NearestFreePointOnPath::costmapCallback, this, std::placeholders::_1),
    sub_opts);

  // Warm-up spin to handle any already-queued messages
  callback_group_executor_.spin_some(std::chrono::nanoseconds(1));

  initialized_ = true;
  RCLCPP_INFO(
    node_->get_logger(),
    "NearestFreePointOnPath: subscribed to '%s'", costmap_topic_.c_str());
}

// ---------------------------------------------------------------------------
// Costmap callback
// ---------------------------------------------------------------------------
void NearestFreePointOnPath::costmapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  costmap_ = msg;
}

// ---------------------------------------------------------------------------
// Free-cell check
// ---------------------------------------------------------------------------
bool NearestFreePointOnPath::isFreePose(double x, double y) const
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) {
    return false;
  }

  const double ox  = costmap_->info.origin.position.x;
  const double oy  = costmap_->info.origin.position.y;
  const double res = costmap_->info.resolution;
  const unsigned int W = costmap_->info.width;
  const unsigned int H = costmap_->info.height;

  const int cx = static_cast<int>((x - ox) / res);
  const int cy = static_cast<int>((y - oy) / res);

  if (cx < 0 || cy < 0 ||
      static_cast<unsigned int>(cx) >= W ||
      static_cast<unsigned int>(cy) >= H)
  {
    return false;  // out of costmap bounds
  }

  const int8_t cost = costmap_->data[cy * static_cast<int>(W) + cx];
  return cost >= 0 && cost <= static_cast<int8_t>(free_threshold_);
}

// ---------------------------------------------------------------------------
// Safety check: all cells within `safety_distance_` of (x,y) must be free
// ---------------------------------------------------------------------------
bool NearestFreePointOnPath::isSafePose(double x, double y) const
{
  if (safety_distance_ <= 0.0) {
    return isFreePose(x, y);
  }

  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) {
    return false;
  }

  const double ox  = costmap_->info.origin.position.x;
  const double oy  = costmap_->info.origin.position.y;
  const double res = costmap_->info.resolution;
  const unsigned int W = costmap_->info.width;
  const unsigned int H = costmap_->info.height;

  const int center_cx = static_cast<int>((x - ox) / res);
  const int center_cy = static_cast<int>((y - oy) / res);
  const int r_cells   = static_cast<int>(std::ceil(safety_distance_ / res));
  const double r_sq   = safety_distance_ * safety_distance_;

  // Sweep a circular footprint around the candidate point; any unknown,
  // out-of-bounds, or above-threshold cell within the circle makes it unsafe.
  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      const double dxm = dx * res;
      const double dym = dy * res;
      if (dxm * dxm + dym * dym > r_sq) {
        continue;
      }
      const int cx = center_cx + dx;
      const int cy = center_cy + dy;
      if (cx < 0 || cy < 0 ||
          static_cast<unsigned int>(cx) >= W ||
          static_cast<unsigned int>(cy) >= H)
      {
        return false;  // out of costmap → treat as unsafe
      }
      const int8_t cost = costmap_->data[cy * static_cast<int>(W) + cx];
      if (cost < 0 || cost > static_cast<int8_t>(free_threshold_)) {
        return false;
      }
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Distance calculation
// ---------------------------------------------------------------------------
double NearestFreePointOnPath::poseDistance(
  const geometry_msgs::msg::PoseStamped & p1,
  const geometry_msgs::msg::PoseStamped & p2) const
{
  const double dx = p1.pose.position.x - p2.pose.position.x;
  const double dy = p1.pose.position.y - p2.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

// ---------------------------------------------------------------------------
// Find nearest free point on path
// ---------------------------------------------------------------------------
bool NearestFreePointOnPath::findNearestFreePointOnPath(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & goal,
  geometry_msgs::msg::PoseStamped & new_goal)
{
  if (path.poses.empty()) {
    RCLCPP_INFO(node_->get_logger(), "NearestFreePointOnPath: path is empty");
    return false;
  }

  // Find the index of the goal point in the path (or closest point)
  int goal_idx = 0;
  double min_dist_to_goal = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dist = poseDistance(path.poses[i], goal);
    if (dist < min_dist_to_goal) {
      min_dist_to_goal = dist;
      goal_idx = i;
    }
  }

  // Search backwards from goal_idx towards the start, looking for free points
  // within search_radius from the original goal
  geometry_msgs::msg::PoseStamped best_point;
  double best_distance = std::numeric_limits<double>::max();
  bool found = false;

  // Check points backwards from goal
  for (int i = goal_idx; i >= 0; --i) {
    const auto & pose = path.poses[i];
    double dist_from_goal = poseDistance(pose, goal);

    // Skip points outside [min_distance_from_goal_, search_radius_]
    if (dist_from_goal < min_distance_from_goal_ || dist_from_goal > search_radius_) {
      continue;
    }

    // Check candidate stays clear of obstacles within safety_distance
    if (isSafePose(pose.pose.position.x, pose.pose.position.y)) {
      // Prefer points closest to the original goal (but ≥ min_distance_from_goal_)
      if (dist_from_goal < best_distance) {
        best_distance = dist_from_goal;
        best_point = pose;
        found = true;
      }
    }
  }

  // Also search forward from goal_idx to maximize chances of finding an alternative
  if (goal_idx + 1 < static_cast<int>(path.poses.size())) {
    for (size_t i = goal_idx + 1; i < path.poses.size(); ++i) {
      const auto & pose = path.poses[i];
      double dist_from_goal = poseDistance(pose, goal);

      // Skip points outside [min_distance_from_goal_, search_radius_]
      if (dist_from_goal < min_distance_from_goal_ || dist_from_goal > search_radius_) {
        continue;
      }

      // Check if this point is free
      if (isFreePose(pose.pose.position.x, pose.pose.position.y)) {
        // Prefer points closest to the original goal (but ≥ min_distance_from_goal_)
        if (dist_from_goal < best_distance) {
          best_distance = dist_from_goal;
          best_point = pose;
          found = true;
        }
      }
    }
  }

  if (found) {
    // Take position from path point but preserve the original goal's
    // orientation (heading). Path points have tangent-aligned orientations
    // which would make the robot face along the path; we want the robot
    // to end facing the original goal's intended heading.
    new_goal.header = best_point.header;
    new_goal.pose.position = best_point.pose.position;
    new_goal.pose.orientation = goal.pose.orientation;

    RCLCPP_INFO(
      node_->get_logger(),
      "NearestFreePointOnPath: found free point at distance %.3f m from goal "
      "(position from path, orientation from original goal)",
      best_distance);
    return true;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "NearestFreePointOnPath: no safe point found on path within "
    "[%.2f, %.2f] m of goal (safety_distance=%.2f m)",
    min_distance_from_goal_, search_radius_, safety_distance_);
  return false;
}

// ---------------------------------------------------------------------------
// tick()
// ---------------------------------------------------------------------------
BT::NodeStatus NearestFreePointOnPath::tick()
{
  if (!initialized_) {
    initialize();
  }

  // Drain any pending costmap messages before we query
  callback_group_executor_.spin_some();

  // Check if costmap is available
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "NearestFreePointOnPath: no costmap received on '%s' yet, returning FAILURE",
        costmap_topic_.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  // Get input ports
  auto path = getInput<nav_msgs::msg::Path>("path");
  if (!path) {
    RCLCPP_ERROR(node_->get_logger(), "NearestFreePointOnPath: missing 'path' input");
    return BT::NodeStatus::FAILURE;
  }

  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal) {
    RCLCPP_ERROR(node_->get_logger(), "NearestFreePointOnPath: missing 'goal' input");
    return BT::NodeStatus::FAILURE;
  }

  // Ensure path and goal are in the same frame (map_frame_)
  geometry_msgs::msg::PoseStamped goal_in_map = goal.value();
  if (goal_in_map.header.frame_id != map_frame_) {
    try {
      geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
        map_frame_, goal_in_map.header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
      tf2::doTransform(goal_in_map, goal_in_map, tf);
      goal_in_map.header.frame_id = map_frame_;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        node_->get_logger(),
        "NearestFreePointOnPath: failed to transform goal: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  // Find nearest free point on path
  geometry_msgs::msg::PoseStamped new_goal;
  if (findNearestFreePointOnPath(path.value(), goal_in_map, new_goal)) {
    setOutput("new_goal", new_goal);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_bt_navigator

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_navigator::NearestFreePointOnPath>("NearestFreePointOnPath");
}
