#include "behavior_plugins/nearest_free_point_on_path.hpp"

#include <cmath>
#include <algorithm>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace behavior_plugins
{

NearestFreePointOnPath::NearestFreePointOnPath(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Build namespace-aware default for the costmap topic.
  // Use the GLOBAL costmap: this node trims a GLOBAL path (in the map frame) to
  // the nearest safe point near the goal, so its cell lookups must be done in the
  // same frame as the path/goal. The local costmap's origin is in the odom frame,
  // which diverges from map once the robot moves — looking up map-frame path poses
  // against an odom-frame grid silently mis-indexes every cell and makes every
  // candidate read as "unsafe" (the "no safe point found" symptom). The header
  // already documents the default as the global costmap; the code now matches it.
  std::string ns = node_->get_namespace();
  if (ns == "/") {
    ns = "";
  }
  const std::string default_costmap_topic = ns + "/global_costmap/costmap";

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
  decl(P + "search_radius",           rclcpp::ParameterValue(2.1));
  decl(P + "min_distance_from_goal",  rclcpp::ParameterValue(0.05));
  decl(P + "free_threshold",          rclcpp::ParameterValue(50));
  decl(P + "step_size",               rclcpp::ParameterValue(0.05));
  decl(P + "safety_distance",         rclcpp::ParameterValue(0.1));
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

  initialized_ = true;
  RCLCPP_INFO(
    node_->get_logger(),
    "NearestFreePointOnPath: subscribed to '%s'", costmap_topic_.c_str());

  // The costmap is published TRANSIENT_LOCAL (latched), so a message IS waiting
  // for us — but it is not delivered the instant create_subscription() returns.
  // The old warm-up was spin_some(1 ns), which almost never caught it: tick()
  // then checked costmap_ in the same millisecond and returned FAILURE. Spin in a
  // bounded loop so the latched sample is actually received before the first use.
  waitForCostmap(std::chrono::milliseconds(500));
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
// Spin the dedicated callback group until a costmap arrives or `timeout` elapses.
// Returns true if a costmap is available when it returns.
// ---------------------------------------------------------------------------
bool NearestFreePointOnPath::waitForCostmap(std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    // Pump pending messages on our private executor (transient_local sample
    // included). spin_some processes whatever is ready right now.
    callback_group_executor_.spin_some();
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (costmap_) {
        return true;
      }
    }
    // Brief sleep so we don't busy-spin while waiting for delivery.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  return static_cast<bool>(costmap_);
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
  double search_radius,
  geometry_msgs::msg::PoseStamped & new_goal)
{
  if (path.poses.empty()) {
    RCLCPP_INFO(node_->get_logger(), "NearestFreePointOnPath: path is empty");
    return false;
  }

  // When search_radius_ <= 0 the upper distance bound is DISABLED: we accept the
  // closest-to-goal safe point anywhere on the path. This is what lets the robot
  // approach a BIG obstacle, where every free point is necessarily farther than
  // a small fixed radius from the (blocked) goal. With search_radius_ > 0 the old
  // behaviour is preserved (only consider points within that band of the goal).
  const bool radius_capped = (search_radius > 0.0);

  // Scan EVERY pose on the path and keep the safe one closest to the goal.
  // We no longer split into a backward/forward pass around goal_idx: the single
  // criterion "safe AND closest to goal" makes that distinction unnecessary and
  // also removes the old backward(isSafePose)/forward(isFreePose) asymmetry —
  // every candidate is now checked with the stricter isSafePose() clearance test.
  geometry_msgs::msg::PoseStamped best_point;
  double best_distance = std::numeric_limits<double>::max();
  bool found = false;

  for (const auto & pose : path.poses) {
    const double dist_from_goal = poseDistance(pose, goal);

    // Lower bound always applies (skip points basically ON the blocked goal);
    // upper bound applies only when a positive search_radius_ is configured.
    if (dist_from_goal < min_distance_from_goal_) {
      continue;
    }
    if (radius_capped && dist_from_goal > search_radius) {
      continue;
    }

    // Candidate must keep `safety_distance_` clearance from all obstacles.
    if (!isSafePose(pose.pose.position.x, pose.pose.position.y)) {
      continue;
    }

    // Prefer the point closest to the original goal (= closest to the obstacle).
    if (dist_from_goal < best_distance) {
      best_distance = dist_from_goal;
      best_point = pose;
      found = true;
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

  if (radius_capped) {
    RCLCPP_INFO(
      node_->get_logger(),
      "NearestFreePointOnPath: no safe point found on path within "
      "[%.2f, %.2f] m of goal (safety_distance=%.2f m)",
      min_distance_from_goal_, search_radius, safety_distance_);
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "NearestFreePointOnPath: no safe point found anywhere on path beyond "
      "%.2f m of goal (search_radius uncapped, safety_distance=%.2f m)",
      min_distance_from_goal_, safety_distance_);
  }
  return false;
}

// ---------------------------------------------------------------------------
// Ray-march the straight robot->goal line and return the farthest safe point
// ---------------------------------------------------------------------------
bool NearestFreePointOnPath::findFreePointOnLine(
  const geometry_msgs::msg::PoseStamped & robot,
  const geometry_msgs::msg::PoseStamped & goal,
  double search_radius,
  geometry_msgs::msg::PoseStamped & new_goal)
{
  const double rx = robot.pose.position.x;
  const double ry = robot.pose.position.y;
  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;

  const double dx = gx - rx;
  const double dy = gy - ry;
  const double line_len = std::sqrt(dx * dx + dy * dy);

  if (line_len < 1e-3) {
    // Robot is essentially on the goal already — nothing to approach.
    RCLCPP_INFO(
      node_->get_logger(),
      "NearestFreePointOnPath: robot already at goal (%.3f m) — no line to march",
      line_len);
    return false;
  }

  // Unit vector pointing from the robot toward the goal.
  const double ux = dx / line_len;
  const double uy = dy / line_len;

  const double step = (step_size_ > 1e-3) ? step_size_ : 0.05;
  const bool radius_capped = (search_radius > 0.0);

  // March from the robot toward the goal and STOP AT THE OBSTACLE'S NEAR EDGE.
  //
  // WHY not "closest safe point to the goal": when a BIG obstacle sits on the goal,
  // the sensor only sees the obstacle's NEAR surface. The cells BEHIND that surface
  // (including the goal cell) were never observed, so the costmap reports them as
  // FREE. "Closest safe point to the goal" would therefore pick a point in that
  // FALSELY-free region on the FAR side of the obstacle, and ComputePathToPose to it
  // would try to drive the robot THROUGH the obstacle. Instead we want the last safe
  // point BEFORE the first real obstacle the robot meets — i.e. the near edge.
  //
  // Three phases handle the robot possibly starting inside its OWN inflation:
  //   A. Leading unsafe band right at the robot (its footprint inflation) — skipped
  //      while entered_free is still false; this is NOT "the obstacle".
  //   B. Free stretch — record the farthest-along safe sample (closest to the goal
  //      so far). last_safe holds the candidate near-edge point.
  //   C. First unsafe sample AFTER the free stretch = the obstacle's near edge →
  //      STOP. last_safe is the point just in front of it. We do NOT keep scanning
  //      into any free space beyond the obstacle.
  //
  // If we reach the goal end without ever hitting an obstacle (whole line free), we
  // simply return the last safe sample (closest to the goal) — the goal area is
  // genuinely approachable.
  bool found = false;
  bool entered_free = false;
  bool stopped_at_edge = false;
  double leading_block_dist = -1.0;   // how far the leading unsafe band reaches
  geometry_msgs::msg::PoseStamped best_point;
  double best_dist_from_goal = std::numeric_limits<double>::max();

  // Diagnostics: of the unsafe samples, how many had a FREE center cell (so only
  // the safety_distance ring rejected them) vs. an occupied center cell. If the
  // line fails and ring_only_blocked > 0 while center_blocked == 0, lowering
  // safety_distance would recover a point; if center_blocked dominates, the
  // underlying cells are occupied and only costmap/inflation changes can help.
  int ring_only_blocked = 0;
  int center_blocked = 0;

  for (double s = step; s <= line_len - 1e-6; s += step) {
    const double px = rx + ux * s;
    const double py = ry + uy * s;

    const double dist_from_goal = line_len - s;  // distance remaining to the goal

    // Stop short of the goal itself (don't return a point basically ON the
    // blocked goal cell).
    if (dist_from_goal < min_distance_from_goal_) {
      break;
    }

    if (!isSafePose(px, py)) {
      // Classify WHY this sample is unsafe (only matters if the whole line fails).
      if (isFreePose(px, py)) {
        ++ring_only_blocked;   // center free, rejected by the clearance ring only
      } else {
        ++center_blocked;      // center cell itself occupied/unknown
      }
      if (entered_free) {
        // Phase C: first obstacle AFTER a free stretch = the near edge. Stop here;
        // best_point already holds the last safe point in front of it.
        stopped_at_edge = true;
        break;
      }
      // Phase A: still in the robot's own leading inflation. Keep advancing and
      // record how far it reached (used only for an honest failure log).
      leading_block_dist = s;
      continue;
    }

    // Phase B: this sample is safe → we have entered the free stretch.
    entered_free = true;

    // Optional cap: skip points farther than search_radius from the goal.
    if (radius_capped && dist_from_goal > search_radius) {
      continue;
    }

    // Record the farthest-along (closest-to-goal) safe sample so far. Because we
    // STOP at the first obstacle after this stretch, this converges on the obstacle
    // near edge — not the falsely-free region behind it.
    if (dist_from_goal < best_dist_from_goal) {
      best_point = robot;                       // copy header/stamp basis
      best_point.header.frame_id = map_frame_;
      best_point.pose.position.x = px;
      best_point.pose.position.y = py;
      best_point.pose.position.z = 0.0;
      best_dist_from_goal = dist_from_goal;
      found = true;
    }
  }

  if (found) {
    new_goal = best_point;
    new_goal.header.stamp = node_->now();
    // Keep the original goal's heading so the robot finishes facing the goal.
    new_goal.pose.orientation = goal.pose.orientation;

    RCLCPP_INFO(
      node_->get_logger(),
      "NearestFreePointOnPath: %s at %.3f m from goal (%.2f, %.2f), line length %.2f m",
      stopped_at_edge ? "stopped at obstacle near edge"
                      : "free point near goal (no obstacle on line)",
      best_dist_from_goal, new_goal.pose.position.x, new_goal.pose.position.y,
      line_len);
    return true;
  }

  // Honest failure log: report how far the unsafe band reached, the line length,
  // the thresholds, AND a breakdown of why samples were unsafe. The breakdown is
  // the actionable part:
  //   ring_only>0, center==0  -> cells are free; only safety_distance rejected them
  //                              => LOWER safety_distance to recover a point.
  //   center>0 dominates      -> the cells themselves are occupied/unknown
  //                              => reduce costmap inflation or the obstacle truly
  //                                 spans the whole line; no plugin tuning helps.
  RCLCPP_INFO(
    node_->get_logger(),
    "NearestFreePointOnPath: no safe point on the robot->goal line "
    "(line_len=%.2f m, entire line unsafe up to %.2f m, free_threshold=%d, "
    "safety_distance=%.2f m, step=%.3f m | unsafe breakdown: "
    "ring_only=%d, center_occupied=%d)",
    line_len, (leading_block_dist >= 0.0 ? leading_block_dist : line_len),
    free_threshold_, safety_distance_, step,
    ring_only_blocked, center_blocked);
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

  // Drain any pending costmap messages, and if none has arrived yet, wait briefly
  // for the TRANSIENT_LOCAL (latched) sample. On the very first tick the
  // subscription may have been created only microseconds earlier, so the latched
  // costmap has not been delivered yet — without this wait the node would return
  // FAILURE on a topic that is actually publishing fine.
  if (!waitForCostmap(std::chrono::milliseconds(500))) {
    RCLCPP_INFO(
      node_->get_logger(),
      "NearestFreePointOnPath: no costmap received on '%s' within timeout, "
      "returning FAILURE",
      costmap_topic_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Goal is required. The 'path' input is no longer used for selection — we
  // ray-march the STRAIGHT robot->goal line instead — but the port is kept for
  // backward compatibility with existing trees.
  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal) {
    RCLCPP_ERROR(node_->get_logger(), "NearestFreePointOnPath: missing 'goal' input");
    return BT::NodeStatus::FAILURE;
  }

  // Ensure the goal is expressed in map_frame_ so its coordinates line up with
  // the costmap and the robot pose below.
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

  // Robot pose in map_frame_ — the start of the line we march along.
  geometry_msgs::msg::PoseStamped robot_in_map;
  try {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      map_frame_, robot_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_));
    robot_in_map.header.frame_id = map_frame_;
    robot_in_map.header.stamp = node_->now();
    robot_in_map.pose.position.x = tf.transform.translation.x;
    robot_in_map.pose.position.y = tf.transform.translation.y;
    robot_in_map.pose.position.z = tf.transform.translation.z;
    robot_in_map.pose.orientation = tf.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "NearestFreePointOnPath: TF lookup [%s -> %s] failed: %s",
      map_frame_.c_str(), robot_frame_.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // Effective search radius: a BT input port overrides the ROS param when
  // provided (pass <= 0 to disable the from-goal distance cap).
  double effective_radius = search_radius_;
  if (auto r = getInput<double>("search_radius")) {
    effective_radius = r.value();
  }

  // Ray-march the straight robot->goal line and return the farthest safe point
  // BETWEEN the robot and the goal (closest to the goal without entering the
  // obstacle). This deliberately ignores the planned path shape so the result is
  // never a point that curves around to the far/occupied side of the goal.
  geometry_msgs::msg::PoseStamped new_goal;
  if (findFreePointOnLine(robot_in_map, goal_in_map, effective_radius, new_goal)) {
    setOutput("new_goal", new_goal);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace behavior_plugins
