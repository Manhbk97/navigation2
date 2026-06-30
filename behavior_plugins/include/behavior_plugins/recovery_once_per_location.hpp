#ifndef BEHAVIOR_PLUGINS__RECOVERY_ONCE_PER_LOCATION_HPP_
#define BEHAVIOR_PLUGINS__RECOVERY_ONCE_PER_LOCATION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace behavior_plugins
{

/**
 * @brief Decorator: permits recovery child to run at most once per unique
 *        robot location.
 *
 * After recovery fires, the robot's pose is saved as `last_trigger_pose_`.
 * The child is suppressed (decorator returns SUCCESS) until the robot
 * travels >= `min_distance` metres from that saved pose.  Once it has moved
 * far enough the child is allowed to run again and a new pose is recorded.
 *
 * Returning SUCCESS when skipping keeps the outer RecoveryNode(999999) alive
 * so navigation keeps re-planning without burning a recovery cycle at the
 * same stuck spot.
 *
 * XML usage:
 *   <RecoveryOncePerLocation min_distance="0.3" name="OncePerLocation">
 *     <SequenceWithMemory name="RecoveryActions"> ... </SequenceWithMemory>
 *   </RecoveryOncePerLocation>
 *
 * BT Port:
 *   min_distance  [input]  double  metres the robot must travel before
 *                          recovery fires again at the same location
 *                          (default 0.3 m)
 */
class RecoveryOncePerLocation : public BT::DecoratorNode
{
public:
  RecoveryOncePerLocation(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(
        "min_distance", 0.3,
        "Min distance (m) robot must travel before recovery fires again"),
    };
  }

  BT::NodeStatus tick() override;

private:
  void initialize();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_{0.1};
  double min_distance_{0.3};

  // Persistent across halt() calls — reset only when this node is destroyed
  // (i.e., a new BT tree is loaded for a new navigator lifecycle).
  bool has_triggered_{false};
  geometry_msgs::msg::PoseStamped last_trigger_pose_;

  bool initialized_{false};
};

}  // namespace behavior_plugins

#endif  // BEHAVIOR_PLUGINS__RECOVERY_ONCE_PER_LOCATION_HPP_
