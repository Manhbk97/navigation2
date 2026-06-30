#ifndef NAV2_BT_NAVIGATOR__PLUGINS__SEARCH_LOCAL_GOAL_ASIDE_HPP_
#define NAV2_BT_NAVIGATOR__PLUGINS__SEARCH_LOCAL_GOAL_ASIDE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_bt_navigator
{

/**
 * @brief SyncActionNode that searches the local costmap for a free pose to
 *        the right of the robot's current heading.
 *
 * A candidate goal is placed at `search_distance` metres from the robot,
 * at `search_angle_deg` degrees clockwise (right) of the robot's heading.
 * If that cell is occupied the node sweeps ±`search_sweep_deg` degrees in
 * `search_step_deg` increments to find the nearest free cell.
 * Returns FAILURE when no free cell is found within the sweep range.
 *
 * All numeric thresholds are exposed as ROS 2 node parameters so they can
 * be tuned from the bt_navigator parameter YAML without recompiling:
 *
 *   search_local_goal_aside.search_angle_deg   (double, default  60.0  deg)
 *   search_local_goal_aside.search_distance    (double, default   0.5  m)
 *   search_local_goal_aside.free_threshold     (int,    default  50    [0-100])
 *   search_local_goal_aside.robot_frame        (string, default: inherited from bt_navigator "robot_base_frame")
 *   search_local_goal_aside.map_frame          (string, default: inherited from bt_navigator "global_frame")
 *   search_local_goal_aside.costmap_topic      (string, default "<ns>/local_costmap/costmap")
 *   search_local_goal_aside.search_sweep_deg   (double, default  20.0  deg)
 *   search_local_goal_aside.search_step_deg    (double, default   5.0  deg)
 *   search_local_goal_aside.tf_timeout         (double, default   0.5  s)
 *
 * BT Ports:
 *   goal  [output] – geometry_msgs/PoseStamped in map frame
 */
class SearchLocalGoalAside : public BT::SyncActionNode
{
public:
  SearchLocalGoalAside(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Free goal pose to the right of the robot"),
    };
  }

  BT::NodeStatus tick() override;

private:
  /// Lazy initialisation: called on the first tick to create subscriptions.
  void initialize();

  /// Returns true if the point (x, y) in map frame is free on the costmap.
  bool isFreePose(double x, double y) const;

  void costmapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  ///< owned only when not from blackboard

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  mutable std::mutex costmap_mutex_;

  bool initialized_{false};

  // --- ROS 2 parameters ---
  double search_angle_deg_;   ///< primary right-side offset angle [deg]
  double search_distance_;    ///< search radius from robot [m]
  int    free_threshold_;     ///< costmap cost ≤ this is considered free [0-100]
  std::string robot_frame_;   ///< TF child frame of the robot base
  std::string map_frame_;     ///< TF parent (global) frame
  std::string costmap_topic_; ///< local costmap topic
  double search_sweep_deg_;   ///< ± sweep range when primary angle is blocked [deg]
  double search_step_deg_;    ///< angular step inside the sweep [deg]
  double tf_timeout_;         ///< TF lookup timeout [s]

  /// Heading (map frame, radians) saved at the end of the previous tick.
  /// Used so the search direction is based on where the robot was going
  /// *before* it stopped, not its current (possibly drifted) heading.
  /// Unset on the very first tick → falls back to current heading.
  std::optional<double> prev_heading_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__PLUGINS__SEARCH_LOCAL_GOAL_ASIDE_HPP_
