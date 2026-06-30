#ifndef BEHAVIOR_PLUGINS__NEAREST_FREE_POINT_ON_PATH_HPP_
#define BEHAVIOR_PLUGINS__NEAREST_FREE_POINT_ON_PATH_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace behavior_plugins
{

/**
 * @brief SyncActionNode that finds the nearest free point on the global path
 *        within a specified search radius from the goal.
 *
 * Given a blocked goal (and optionally a planned path), 
 * it finds the last safe point in front of the obstacle,
 * between the robot and the goal, so the robot can approach as close as possible without 
 * trying to drive through the obstacle
 *
 * All numeric thresholds are exposed as ROS 2 node parameters:
 *
 *   nearest_free_point_on_path.search_radius           (double, default  1.0  m)
 *   nearest_free_point_on_path.min_distance_from_goal  (double, default  0.0  m)
 *   nearest_free_point_on_path.free_threshold          (int,    default  50   [0-100])
 *   nearest_free_point_on_path.step_size               (double, default  0.05 m)
 *   nearest_free_point_on_path.safety_distance         (double, default  0.30 m)
 *   nearest_free_point_on_path.robot_frame             (string, default: inherited from bt_navigator "robot_base_frame")
 *   nearest_free_point_on_path.map_frame               (string, default: inherited from bt_navigator "global_frame")
 *   nearest_free_point_on_path.costmap_topic           (string, default "<ns>/local_costmap/costmap")
 *   nearest_free_point_on_path.costmap_qos_transient   (bool,   default false — local costmap is volatile)
 *   nearest_free_point_on_path.tf_timeout              (double, default   0.5  s)
 *
 * BT Ports:
 *   path  [input]  – nav_msgs/msg/Path on the global path to search
 *   goal  [input]  – geometry_msgs/msg/PoseStamped original goal
 *   new_goal [output] – geometry_msgs/msg/PoseStamped adjusted goal on the path
 */
class NearestFreePointOnPath : public BT::SyncActionNode
{
public:
  NearestFreePointOnPath(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>(
        "path", "Global path to search for free points"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Original goal pose"),
      BT::InputPort<double>(
        "search_radius",
        "Max distance (m) from goal to accept a point. <= 0 disables the cap so "
        "the closest reachable safe point is taken however far it is from the goal "
        "(needed to approach big obstacles). When unset, the ROS param default is used."),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "new_goal", "Nearest free point on path"),
    };
  }

  BT::NodeStatus tick() override;

  /// Result of a costmap cell query. The OUT_OF_BOUNDS case matters for the
  /// LOCAL costmap, which is a small rolling window: a point outside the window
  /// is "not yet observed", NOT a confirmed obstacle, so the marching loops keep
  /// advancing instead of stopping at a false near-edge.
  enum class CellState
  {
    FREE,          ///< cell(s) observed and below free_threshold_
    OCCUPIED,      ///< cell(s) observed and occupied/unknown — a real obstacle
    OUT_OF_BOUNDS  ///< point falls outside the costmap window — unobserved
  };

private:
  /// Lazy initialisation: called on the first tick to create subscriptions.
  void initialize();

  /// Transform a point given in `map_frame_` into the costmap's own header
  /// frame (`costmap_frame_`, e.g. "odom" for the local costmap), then write the
  /// costmap cell indices into `out_cx`/`out_cy`. Returns false (OUT_OF_BOUNDS
  /// caller should assume) if the TF lookup fails. When map_frame_ already equals
  /// costmap_frame_ (e.g. global costmap) this is a cheap no-op copy.
  bool mapPointToCell(double x, double y, int & out_cx, int & out_cy) const;

  /// Classify the single cell under map-frame point (x, y): FREE, OCCUPIED, or
  /// OUT_OF_BOUNDS (outside the costmap window / unobserved).
  CellState classifyPose(double x, double y) const;

  /// Returns true if the point (x, y) in map frame is FREE on the costmap.
  /// OUT_OF_BOUNDS is treated as not-free by this helper (used only for the
  /// diagnostic center-cell check).
  bool isFreePose(double x, double y) const;

  /// Classify the safety footprint around (x, y): OCCUPIED if any observed cell
  /// within `safety_distance_` is occupied/unknown; OUT_OF_BOUNDS if no cell is
  /// occupied but part of the footprint falls outside the window; otherwise FREE.
  CellState classifySafePose(double x, double y) const;

  /// Callback for costmap updates
  void costmapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /// Spin the dedicated callback group until a costmap is received or `timeout`
  /// elapses. Needed because the costmap is published TRANSIENT_LOCAL (latched):
  /// the sample is not delivered the instant the subscription is created, so the
  /// first tick must wait briefly for it. Returns true if a costmap is available.
  bool waitForCostmap(std::chrono::milliseconds timeout);

  /// Finds the point on the path closest to the goal that keeps `safety_distance_`
  /// clearance. `search_radius` caps the max distance from the goal; pass <= 0 to
  /// disable the cap (take the closest reachable safe point however far it is).
  /// Returns true if a valid point is found, false otherwise.
  /// (Legacy path-based selection; tick() uses findFreePointOnLine() instead.)
  bool findNearestFreePointOnPath(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & goal,
    double search_radius,
    geometry_msgs::msg::PoseStamped & new_goal);

  /// Ray-marches the STRAIGHT line from `robot` toward `goal` in `step_size_`
  /// increments and returns the farthest-along sample that still keeps
  /// `safety_distance_` clearance, i.e. the point BETWEEN the robot and the goal,
  /// as close to the goal as possible WITHOUT crossing into the obstacle. Marching
  /// stops at the first unsafe sample so it never jumps to free space on the far
  /// side of the obstacle. `search_radius` (> 0) optionally caps how far from the
  /// goal the returned point may be; <= 0 disables that cap.
  /// Returns true if at least one safe point was found, false otherwise.
  bool findFreePointOnLine(
    const geometry_msgs::msg::PoseStamped & robot,
    const geometry_msgs::msg::PoseStamped & goal,
    double search_radius,
    geometry_msgs::msg::PoseStamped & new_goal);

  /// Calculates Euclidean distance between two poses
  double poseDistance(
    const geometry_msgs::msg::PoseStamped & p1,
    const geometry_msgs::msg::PoseStamped & p2) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  mutable std::mutex costmap_mutex_;

  bool initialized_{false};

  // --- ROS 2 parameters ---
  double search_radius_;            ///< max search radius from goal [m]
  double min_distance_from_goal_;   ///< min distance from goal (skip points closer than this) [m]
  int    free_threshold_;           ///< costmap cost ≤ this is considered free [0-100]
  double step_size_;                ///< step size along the path [m]
  double safety_distance_;          ///< required clearance around candidate point [m]
  std::string robot_frame_;         ///< TF child frame of the robot base
  std::string map_frame_;           ///< TF parent (global) frame the goal/path live in
  std::string costmap_topic_;       ///< costmap topic (local costmap by default)
  bool        costmap_qos_transient_;  ///< subscribe TRANSIENT_LOCAL (global) vs VOLATILE (local)
  double tf_timeout_;               ///< TF lookup timeout [s]

  /// Header frame of the most recently received costmap (e.g. "odom" for the
  /// local costmap, "map" for the global). Used to transform map-frame query
  /// points into the grid's frame before indexing. Guarded by costmap_mutex_.
  std::string costmap_frame_;
};

}  // namespace behavior_plugins

#endif  // BEHAVIOR_PLUGINS__NEAREST_FREE_POINT_ON_PATH_HPP_
