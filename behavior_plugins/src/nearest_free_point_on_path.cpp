#include "behavior_plugins/nearest_free_point_on_path.hpp"

#include <cmath>
#include <algorithm>

#include "geometry_msgs/msg/point_stamped.hpp"
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
  // Use the LOCAL costmap. The local costmap's origin is expressed in its own
  // header frame (typically "odom"), which diverges from "map" once the robot
  // moves, so cell lookups must NOT assume the grid is in map. We therefore
  // transform every map-frame query point into the costmap's own header frame
  // (costmap_frame_) via TF before indexing — see mapPointToCell(). Because the
  // local costmap is a small rolling window, points outside it are treated as
  // OUT_OF_BOUNDS (unobserved) rather than as obstacles, so the marching loops
  // keep advancing instead of stopping at a false near-edge.
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
  decl(P + "search_radius",           rclcpp::ParameterValue(2.1));
  decl(P + "min_distance_from_goal",  rclcpp::ParameterValue(0.05));
  decl(P + "free_threshold",          rclcpp::ParameterValue(50));
  decl(P + "step_size",               rclcpp::ParameterValue(0.05));
  decl(P + "safety_distance",         rclcpp::ParameterValue(0.1));
  decl(P + "robot_frame",             rclcpp::ParameterValue(default_robot_frame));
  decl(P + "map_frame",               rclcpp::ParameterValue(default_map_frame));
  decl(P + "costmap_topic",           rclcpp::ParameterValue(default_costmap_topic));
  // Local costmap is published VOLATILE (no latched sample); the global costmap
  // is TRANSIENT_LOCAL. Default to volatile to match the new local-costmap
  // default; set true if you point costmap_topic at a global/latched costmap.
  decl(P + "costmap_qos_transient",   rclcpp::ParameterValue(false));
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
  costmap_qos_transient_  = node_->get_parameter(P + "costmap_qos_transient").as_bool();
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
    "topic=%s, map_frame=%s, qos=%s",
    search_radius_, min_distance_from_goal_, step_size_,
    free_threshold_, safety_distance_, costmap_topic_.c_str(), map_frame_.c_str(),
    costmap_qos_transient_ ? "transient_local" : "volatile");
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

  // QoS must match the publisher or NO messages are delivered. The local costmap
  // publishes VOLATILE; the global costmap publishes TRANSIENT_LOCAL (latched).
  // Select via the costmap_qos_transient_ parameter.
  rclcpp::QoS costmap_qos(1);
  if (costmap_qos_transient_) {
    costmap_qos.transient_local();
  } else {
    costmap_qos.durability_volatile();
  }

  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_,
    costmap_qos,
    std::bind(&NearestFreePointOnPath::costmapCallback, this, std::placeholders::_1),
    sub_opts);

  initialized_ = true;
  RCLCPP_INFO(
    node_->get_logger(),
    "NearestFreePointOnPath: subscribed to '%s' (durability=%s)",
    costmap_topic_.c_str(), costmap_qos_transient_ ? "transient_local" : "volatile");

  // Wait briefly for the first costmap. For a TRANSIENT_LOCAL (global) topic a
  // latched sample is already waiting and arrives within a few ms. For a VOLATILE
  // (local) topic there is NO latched sample — we must wait for the publisher's
  // next periodic update (local costmap typically ~5 Hz), so allow more time.
  const auto warmup = costmap_qos_transient_
    ? std::chrono::milliseconds(500)
    : std::chrono::milliseconds(1000);
  waitForCostmap(warmup);
}

// ---------------------------------------------------------------------------
// Costmap callback
// ---------------------------------------------------------------------------
void NearestFreePointOnPath::costmapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  costmap_ = msg;
  // Remember the grid's own frame so query points (in map_frame_) can be
  // transformed into it before indexing. For the local costmap this is "odom".
  costmap_frame_ = msg->header.frame_id;
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
// Transform a map-frame point into the costmap's frame and compute cell indices.
// Returns false if the TF lookup fails (caller treats that as OUT_OF_BOUNDS).
// Assumes costmap_mutex_ is already held by the caller (reads costmap_*).
// ---------------------------------------------------------------------------
bool NearestFreePointOnPath::mapPointToCell(
  double x, double y, int & out_cx, int & out_cy) const
{
  double gx = x;
  double gy = y;

  // Transform map-frame (x, y) into the grid's own frame (e.g. "odom") unless
  // they already match (global costmap case → cheap skip). map/odom differ by a
  // planar SE(2) transform, so a 2D apply of the lookup is sufficient here.
  if (!costmap_frame_.empty() && costmap_frame_ != map_frame_) {
    try {
      geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
        costmap_frame_, map_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
      geometry_msgs::msg::PointStamped in, out;
      in.header.frame_id = map_frame_;
      in.point.x = x;
      in.point.y = y;
      in.point.z = 0.0;
      tf2::doTransform(in, out, tf);
      gx = out.point.x;
      gy = out.point.y;
    } catch (const tf2::TransformException &) {
      return false;  // can't place the point in the grid frame → unobserved
    }
  }

  const double ox  = costmap_->info.origin.position.x;
  const double oy  = costmap_->info.origin.position.y;
  const double res = costmap_->info.resolution;

  out_cx = static_cast<int>(std::floor((gx - ox) / res));
  out_cy = static_cast<int>(std::floor((gy - oy) / res));
  return true;
}

// ---------------------------------------------------------------------------
// Classify a single cell: FREE / OCCUPIED / OUT_OF_BOUNDS
// ---------------------------------------------------------------------------
NearestFreePointOnPath::CellState
NearestFreePointOnPath::classifyPose(double x, double y) const
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) {
    return CellState::OUT_OF_BOUNDS;
  }

  int cx = 0, cy = 0;
  if (!mapPointToCell(x, y, cx, cy)) {
    return CellState::OUT_OF_BOUNDS;
  }

  const unsigned int W = costmap_->info.width;
  const unsigned int H = costmap_->info.height;
  if (cx < 0 || cy < 0 ||
      static_cast<unsigned int>(cx) >= W ||
      static_cast<unsigned int>(cy) >= H)
  {
    return CellState::OUT_OF_BOUNDS;  // outside the rolling window → unobserved
  }

  const int8_t cost = costmap_->data[cy * static_cast<int>(W) + cx];
  return (cost >= 0 && cost <= static_cast<int8_t>(free_threshold_))
    ? CellState::FREE
    : CellState::OCCUPIED;
}

// ---------------------------------------------------------------------------
// Free-cell check (center cell only). OUT_OF_BOUNDS counts as not-free.
// Used for the diagnostic "center cell" classification in the marching loops.
// ---------------------------------------------------------------------------
bool NearestFreePointOnPath::isFreePose(double x, double y) const
{
  return classifyPose(x, y) == CellState::FREE;
}

// ---------------------------------------------------------------------------
// Safety check over a circular footprint of radius safety_distance_.
//   - any observed cell occupied/unknown        -> OCCUPIED (a real obstacle)
//   - else any cell outside the window           -> OUT_OF_BOUNDS (unobserved)
//   - else                                       -> FREE
// OCCUPIED takes priority: a confirmed obstacle anywhere in the footprint makes
// the pose unsafe even if part of the footprint is off-window.
// ---------------------------------------------------------------------------
NearestFreePointOnPath::CellState
NearestFreePointOnPath::classifySafePose(double x, double y) const
{
  if (safety_distance_ <= 0.0) {
    return classifyPose(x, y);
  }

  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) {
    return CellState::OUT_OF_BOUNDS;
  }

  int center_cx = 0, center_cy = 0;
  if (!mapPointToCell(x, y, center_cx, center_cy)) {
    return CellState::OUT_OF_BOUNDS;
  }

  const double res = costmap_->info.resolution;
  const unsigned int W = costmap_->info.width;
  const unsigned int H = costmap_->info.height;
  const int r_cells   = static_cast<int>(std::ceil(safety_distance_ / res));
  const double r_sq   = safety_distance_ * safety_distance_;

  bool saw_out_of_bounds = false;

  // Sweep a circular footprint around the candidate point. An occupied/unknown
  // observed cell makes it OCCUPIED immediately; off-window cells are noted but
  // only downgrade the result to OUT_OF_BOUNDS if nothing occupied was found.
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
        saw_out_of_bounds = true;  // unobserved — defer judgement
        continue;
      }
      const int8_t cost = costmap_->data[cy * static_cast<int>(W) + cx];
      if (cost < 0 || cost > static_cast<int8_t>(free_threshold_)) {
        return CellState::OCCUPIED;
      }
    }
  }
  return saw_out_of_bounds ? CellState::OUT_OF_BOUNDS : CellState::FREE;
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
  // closest-to-goal safe point that lies BEFORE the first obstacle. This is what
  // lets the robot approach a BIG obstacle, where every free point is necessarily
  // farther than a small fixed radius from the (blocked) goal. With
  // search_radius_ > 0 only points within that band of the goal are considered.
  const bool radius_capped = (search_radius > 0.0);

  // STEP ALONG THE PATH FROM THE ROBOT and STOP AT THE OBSTACLE'S NEAR EDGE.
  //
  // path.poses are ordered from the robot end (poses[0]) toward the goal, so
  // iterating forward = marching away from the robot toward the goal — the same
  // direction as findFreePointOnLine, but following the planned path's shape
  // instead of the straight robot->goal line.
  //
  // WHY stop at the near edge instead of taking the global closest-to-goal safe
  // point: when a BIG obstacle sits on the goal, the sensor only sees the
  // obstacle's NEAR surface. Cells BEHIND it (incl. the goal cell) were never
  // observed, so the costmap reports them FREE. "Closest safe point to the goal"
  // would pick a point in that FALSELY-free region on the FAR side, and a plan to
  // it would drive the robot THROUGH the obstacle. Marching from the robot and
  // stopping at the first obstacle returns the last safe point in FRONT of it.
  //
  // Three phases (mirroring findFreePointOnLine) handle the robot possibly
  // starting inside its OWN footprint inflation:
  //   A. Leading unsafe band right at the robot — skipped while entered_free is
  //      still false; this is NOT "the obstacle".
  //   B. Free stretch — record the farthest-along safe pose (closest to the goal
  //      so far). best_point holds the candidate near-edge point.
  //   C. First unsafe pose AFTER the free stretch = the obstacle's near edge →
  //      STOP. We do NOT keep scanning into any free space beyond the obstacle.
  geometry_msgs::msg::PoseStamped best_point;
  double best_dist_from_goal = std::numeric_limits<double>::max();
  bool found = false;
  bool entered_free = false;
  bool stopped_at_edge = false;

  // Diagnostics for the failure log: of the unsafe poses, how many had a FREE
  // center cell (only the safety_distance ring rejected them) vs. an occupied
  // center cell. ring_only>0 with center==0 means lowering safety_distance would
  // recover a point; center-dominated means the cells themselves are blocked.
  int ring_only_blocked = 0;
  int center_blocked = 0;

  for (const auto & pose : path.poses) {
    const double dist_from_goal = poseDistance(pose, goal);

    // Stop short of the goal itself (don't return a point basically ON the
    // blocked goal cell). Poses near the goal are at the END of the path, so
    // once we are this close we are done marching.
    if (dist_from_goal < min_distance_from_goal_) {
      break;
    }

    const CellState state = classifySafePose(pose.pose.position.x, pose.pose.position.y);

    if (state == CellState::OUT_OF_BOUNDS) {
      // Point lies outside the local costmap window → UNOBSERVED, not an
      // obstacle. Do NOT treat it as the near edge: keep marching so a pose that
      // re-enters the window can still be evaluated. (For the global costmap,
      // whose frame == map and which covers the whole map, this rarely fires.)
      continue;
    }

    if (state == CellState::OCCUPIED) {
      // Classify WHY this pose is unsafe (only matters if the whole path fails).
      if (isFreePose(pose.pose.position.x, pose.pose.position.y)) {
        ++ring_only_blocked;   // center free, rejected by the clearance ring only
      } else {
        ++center_blocked;      // center cell itself occupied/unknown
      }
      if (entered_free) {
        // Phase C: first obstacle AFTER a free stretch = the near edge. Stop here;
        // best_point already holds the last safe pose in front of it.
        stopped_at_edge = true;
        break;
      }
      // Phase A: still in the robot's own leading inflation. Keep advancing.
      continue;
    }

    // Phase B: this pose is safe (FREE) → we have entered the free stretch.
    entered_free = true;

    // Optional cap: skip poses farther than search_radius from the goal.
    if (radius_capped && dist_from_goal > search_radius) {
      continue;
    }

    // Record the farthest-along (closest-to-goal) safe pose so far. Because we
    // STOP at the first obstacle after this stretch, this converges on the
    // obstacle near edge — not the falsely-free region behind it.
    if (dist_from_goal < best_dist_from_goal) {
      best_dist_from_goal = dist_from_goal;
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
      "NearestFreePointOnPath: %s at %.3f m from goal "
      "(position from path, orientation from original goal)",
      stopped_at_edge ? "stopped at obstacle near edge"
                      : "free point near goal (no obstacle on path)",
      best_dist_from_goal);
    return true;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "NearestFreePointOnPath: no safe point found while marching the path from the "
    "robot (radius_capped=%d, search_radius=%.2f m, min_distance=%.2f m, "
    "safety_distance=%.2f m | unsafe breakdown: ring_only=%d, center_occupied=%d)",
    radius_capped, search_radius, min_distance_from_goal_, safety_distance_,
    ring_only_blocked, center_blocked);
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

    const CellState state = classifySafePose(px, py);

    if (state == CellState::OUT_OF_BOUNDS) {
      // Sample lies outside the local costmap window → UNOBSERVED, not an
      // obstacle. Keep marching so a sample that re-enters the window can still
      // be evaluated; never treat the window edge as the obstacle near edge.
      continue;
    }

    if (state == CellState::OCCUPIED) {
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

    // Phase B: this sample is safe (FREE) → we have entered the free stretch.
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

  // Goal is required. The optional 'path' input selects the search strategy
  // below: if a non-empty path is provided we march ALONG it
  // (findNearestFreePointOnPath); otherwise we fall back to ray-marching the
  // STRAIGHT robot->goal line (findFreePointOnLine).
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

  // Choose the search strategy based on whether a path was supplied:
  //   - PATH AVAILABLE (non-empty): march ALONG the planned path from the robot
  //     toward the goal and stop at the obstacle's near edge. Following the path
  //     shape works around curves where the straight line would cut through a
  //     wall, while still returning a point on the ROBOT'S side of the obstacle.
  //   - NO PATH (port unset, or empty path — e.g. the planner failed because the
  //     goal is walled off): fall back to ray-marching the STRAIGHT robot->goal
  //     line so we still produce an approach point without any planner output.
  // Either way the result is the farthest-along safe point BETWEEN the robot and
  // the goal — never a point that curves around to the far/occupied side.
  geometry_msgs::msg::PoseStamped new_goal;
  bool found = false;

  auto path = getInput<nav_msgs::msg::Path>("path");
  if (path && !path.value().poses.empty()) {
    found = findNearestFreePointOnPath(path.value(), goal_in_map, effective_radius, new_goal);
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "NearestFreePointOnPath: no usable 'path' input — falling back to the "
      "straight robot->goal line");
    found = findFreePointOnLine(robot_in_map, goal_in_map, effective_radius, new_goal);
  }

  if (found) {
    setOutput("new_goal", new_goal);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace behavior_plugins
