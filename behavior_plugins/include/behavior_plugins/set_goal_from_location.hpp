#ifndef BEHAVIOR_PLUGINS__SET_GOAL_FROM_LOCATION_HPP_
#define BEHAVIOR_PLUGINS__SET_GOAL_FROM_LOCATION_HPP_

#include <memory>
#include <string>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace behavior_plugins
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
 *                    based on the robot's current pose in the map frame
 *   goal           – output PoseStamped written to the blackboard
 *
 * When loc=="closest" the node reads the robot pose from the TF tree
 * (map -> base_link) and picks the location with the smallest Euclidean
 * distance. No odom subscription is used so the plugin does not depend
 * on any odom topic name.
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
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};
};

}  // namespace behavior_plugins

#endif  // BEHAVIOR_PLUGINS__SET_GOAL_FROM_LOCATION_HPP_
