#include "behavior_plugins/search_local_goal_aside.hpp"

#include <cmath>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace behavior_plugins
{

// ---------------------------------------------------------------------------
// Constructor – declare / read ROS 2 parameters
// ---------------------------------------------------------------------------
SearchLocalGoalAside::SearchLocalGoalAside(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Build namespace-aware default for the costmap topic.
  // node_->get_namespace() returns "/" for the root namespace, so we strip
  // the trailing slash before prepending it (same pattern as set_goal_from_location).
  std::string ns = node_->get_namespace();
  if (ns == "/") {
    ns = "";
  }
  const std::string default_costmap_topic = ns + "/local_costmap/costmap";

  // Helper: declare parameter only when it does not exist yet (guards
  // against multiple plugin instances re-declaring the same parameter).
  auto decl = [&](const std::string & full_name, const rclcpp::ParameterValue & val) {
    if (!node_->has_parameter(full_name)) {
      node_->declare_parameter(full_name, val);
    }
  };

  const std::string P = "search_local_goal_aside.";

  // Inherit global_frame and robot_base_frame from the bt_navigator node so
  // that namespaced setups (e.g. "apple/map") work without extra YAML config.
  std::string default_map_frame   = "map";
  std::string default_robot_frame = ns.empty() ? "base_footprint" : ns.substr(1) + "/base_footprint";
  if (node_->has_parameter("global_frame")) {
    default_map_frame = node_->get_parameter("global_frame").as_string();
  }
  if (node_->has_parameter("robot_base_frame")) {
    default_robot_frame = node_->get_parameter("robot_base_frame").as_string();
  }

  decl(P + "search_angle_deg",  rclcpp::ParameterValue(60.0));
  decl(P + "search_distance",   rclcpp::ParameterValue(0.3));
  decl(P + "free_threshold",    rclcpp::ParameterValue(50));
  decl(P + "robot_frame",       rclcpp::ParameterValue(default_robot_frame));
  decl(P + "map_frame",         rclcpp::ParameterValue(default_map_frame));
  decl(P + "costmap_topic",     rclcpp::ParameterValue(default_costmap_topic));
  decl(P + "search_sweep_deg",  rclcpp::ParameterValue(20.0));
  decl(P + "search_step_deg",   rclcpp::ParameterValue(5.0));
  decl(P + "tf_timeout",        rclcpp::ParameterValue(0.5));

  search_angle_deg_ = node_->get_parameter(P + "search_angle_deg").as_double();
  search_distance_  = node_->get_parameter(P + "search_distance").as_double();
  free_threshold_   = static_cast<int>(node_->get_parameter(P + "free_threshold").as_int());
  robot_frame_      = node_->get_parameter(P + "robot_frame").as_string();
  map_frame_        = node_->get_parameter(P + "map_frame").as_string();
  costmap_topic_    = node_->get_parameter(P + "costmap_topic").as_string();
  search_sweep_deg_ = node_->get_parameter(P + "search_sweep_deg").as_double();
  search_step_deg_  = node_->get_parameter(P + "search_step_deg").as_double();
  tf_timeout_       = node_->get_parameter(P + "tf_timeout").as_double();

  RCLCPP_INFO(
    node_->get_logger(),
    "SearchLocalGoalAside: angle=%.1f deg, dist=%.2f m, "
    "sweep=±%.1f deg (step %.1f deg), threshold=%d, topic=%s",
    search_angle_deg_, search_distance_,
    search_sweep_deg_, search_step_deg_,
    free_threshold_, costmap_topic_.c_str());
}

// ---------------------------------------------------------------------------
// Lazy initialisation (called on first tick)
// ---------------------------------------------------------------------------
void SearchLocalGoalAside::initialize()
{
  // Prefer the shared TF buffer placed on the BT blackboard by nav2_bt_navigator.
  // Fall back to creating a private one when running standalone (e.g. tests).
  try {
    tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  } catch (const std::exception &) {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, node_, false /* do not spin */);
    RCLCPP_WARN(
      node_->get_logger(),
      "SearchLocalGoalAside: 'tf' not found on blackboard, created own TF listener.");
  }

  // Dedicated callback group so spin_some() works inside a BT tick
  // (the main node executor is busy executing the tick at this point).
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
    std::bind(&SearchLocalGoalAside::costmapCallback, this, std::placeholders::_1),
    sub_opts);

  // Warm-up spin to handle any already-queued messages
  callback_group_executor_.spin_some(std::chrono::nanoseconds(1));

  initialized_ = true;
  RCLCPP_INFO(
    node_->get_logger(),
    "SearchLocalGoalAside: subscribed to '%s'", costmap_topic_.c_str());
}

// ---------------------------------------------------------------------------
// Costmap callback
// ---------------------------------------------------------------------------
void SearchLocalGoalAside::costmapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  costmap_ = msg;
}

// ---------------------------------------------------------------------------
// Free-cell check
// ---------------------------------------------------------------------------
bool SearchLocalGoalAside::isFreePose(double x, double y) const
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
  // cost == -1 means unknown → treat as occupied
  return cost >= 0 && cost <= static_cast<int8_t>(free_threshold_);
}

// ---------------------------------------------------------------------------
// tick()
// ---------------------------------------------------------------------------
BT::NodeStatus SearchLocalGoalAside::tick()
{
  if (!initialized_) {
    initialize();
  }

  // Drain any pending costmap messages before we query
  callback_group_executor_.spin_some();

  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "SearchLocalGoalAside: no costmap received on '%s' yet, returning FAILURE",
        costmap_topic_.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  // Retrieve the costmap frame_id while holding the mutex.
  std::string costmap_frame;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_frame = costmap_->header.frame_id;
  }

  // Robot pose in map frame – used for the output goal coordinates.
  geometry_msgs::msg::TransformStamped tf_map;
  try {
    tf_map = tf_buffer_->lookupTransform(
      map_frame_, robot_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SearchLocalGoalAside: TF lookup [%s -> %s] failed: %s",
      map_frame_.c_str(), robot_frame_.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double rx        = tf_map.transform.translation.x;
  const double ry        = tf_map.transform.translation.y;
  const double robot_yaw = tf2::getYaw(tf_map.transform.rotation);

  // Robot pose in costmap frame (e.g. "apple/ekf_odom") – used for cell
  // index lookups in isFreePose(), whose origin is expressed in that frame.
  geometry_msgs::msg::TransformStamped tf_costmap;
  try {
    tf_costmap = tf_buffer_->lookupTransform(
      costmap_frame, robot_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SearchLocalGoalAside: TF lookup [%s -> %s] failed: %s",
      costmap_frame.c_str(), robot_frame_.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double crx  = tf_costmap.transform.translation.x;
  const double cry  = tf_costmap.transform.translation.y;
  const double cyaw = tf2::getYaw(tf_costmap.transform.rotation);

  // Use the heading the robot had on the *previous* tick as the search
  // direction.  This ensures we search to the right of where the robot was
  // travelling before it stopped/was blocked, not its current (potentially
  // rotated) heading.  On the very first tick prev_heading_ is unset, so we
  // fall back to the current heading.
  const double search_yaw  = prev_heading_.value_or(robot_yaw);

  // Apply the same angular delta to the costmap-frame yaw so that both the
  // map-frame goal coordinates and the costmap-frame cell-check stay consistent.
  const double heading_delta = search_yaw - robot_yaw;
  const double search_cyaw   = cyaw + heading_delta;

  // Save current heading for the next tick.
  prev_heading_ = robot_yaw;

  // --- Build candidate angular offsets ---
  //
  // In ROS 2 convention, positive angles are counter-clockwise (left).
  // "60 degrees to the right" means -60° offset from the robot heading.
  // We therefore subtract the (positive) search_angle_deg from robot_yaw.
  //
  // Search order:  primary angle, then sweep outward alternately
  //   (larger rightward first, then slightly less rightward).

  const double primary_rad = search_angle_deg_ * M_PI / 180.0;
  const double step_rad    = search_step_deg_   * M_PI / 180.0;
  const double sweep_rad   = search_sweep_deg_  * M_PI / 180.0;

  std::vector<double> offsets;
  offsets.push_back(primary_rad);
  for (double delta = step_rad; delta <= sweep_rad + 1e-9; delta += step_rad) {
    offsets.push_back(primary_rad + delta);   // more to the right
    offsets.push_back(primary_rad - delta);   // slightly less to the right (back toward front)
  }

  for (const double offset : offsets) {
    // Candidate in map frame – used for the output PoseStamped.
    const double dir = search_yaw - offset;
    const double gx  = rx + search_distance_ * std::cos(dir);
    const double gy  = ry + search_distance_ * std::sin(dir);

    // Candidate in costmap frame – used for isFreePose() cell lookup.
    // The costmap origin (info.origin) is expressed in costmap_frame, so
    // the query coordinates must match that frame.
    const double cdir = search_cyaw - offset;
    const double cgx  = crx + search_distance_ * std::cos(cdir);
    const double cgy  = cry + search_distance_ * std::sin(cdir);

    if (!isFreePose(cgx, cgy)) {
      continue;
    }

    // Build output PoseStamped; keep robot's heading so it stays aligned
    // with the corridor after moving aside.
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = map_frame_;
    goal.header.stamp    = node_->now();
    goal.pose.position.x = gx;
    goal.pose.position.y = gy;
    goal.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, search_yaw);   // align to previous heading, not current
    goal.pose.orientation = tf2::toMsg(q);

    setOutput("goal", goal);

    RCLCPP_INFO(
      node_->get_logger(),
      "SearchLocalGoalAside: free goal at (%.2f, %.2f) — offset %.1f deg right",
      gx, gy, offset * 180.0 / M_PI);

    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(
    node_->get_logger(),
    "SearchLocalGoalAside: no free cell found within %.1f deg ± %.1f deg of robot right side",
    search_angle_deg_, search_sweep_deg_);
  return BT::NodeStatus::FAILURE;
}

}  // namespace behavior_plugins
