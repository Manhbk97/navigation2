#ifndef NAV2_BT_NAVIGATOR__PLUGINS__SET_GOAL_FROM_LOCATION_HPP_
#define NAV2_BT_NAVIGATOR__PLUGINS__SET_GOAL_FROM_LOCATION_HPP_

#include <string>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_navigator
{

/**
 * @brief SyncActionNode that reads a named pose from a YAML locations file
 *        and writes it to the BT blackboard.
 *
 * YAML format expected:
 *   locations:
 *     <name>:
 *       x: <double>
 *       y: <double>
 *       yaw: <double>   # radians, in map frame
 *
 * Ports:
 *   location_file  – absolute path to the YAML file
 *   loc            – key name under "locations:" (e.g. "location1"),
 *                    or "closest" to auto-select the nearest location
 *                    based on the robot's current odometry pose
 *   goal           – output PoseStamped written to the blackboard
 *
 * When loc=="closest" the node subscribes to /<namespace>/ekf_odom
 * and picks the location with the smallest Euclidean distance.
 */
class SetGoalFromLocation : public BT::SyncActionNode
{
public:
  SetGoalFromLocation(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("location_file", "Absolute path to locations YAML"),
      BT::InputPort<std::string>("loc", "Key name under locations: or 'closest'"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "Pose written to blackboard"),
    };
  }

  BT::NodeStatus tick() override;

private:
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  geometry_msgs::msg::Point robot_position_;
  bool pose_received_{false};
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__PLUGINS__SET_GOAL_FROM_LOCATION_HPP_
