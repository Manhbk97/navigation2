// ArePosesNear: returns SUCCESS when the Euclidean XY distance between
// ref_pose and target_pose is <= tolerance.  Both poses are expected to be
// in the same frame (map); no TF transform is performed.
#include <cmath>
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_monitoring_tree
{

class ArePosesNear : public BT::ConditionNode
{
public:
  ArePosesNear(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("ref_pose"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
      BT::InputPort<double>("tolerance", 0.5, "Distance threshold in metres"),
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped p1, p2;
    double tol;

    if (!getInput("ref_pose", p1) || !getInput("target_pose", p2)) {
      return BT::NodeStatus::FAILURE;
    }
    getInput("tolerance", tol);

    const double dx = p1.pose.position.x - p2.pose.position.x;
    const double dy = p1.pose.position.y - p2.pose.position.y;
    const double dist_sq = dx * dx + dy * dy;

    return (dist_sq <= tol * tol) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::ArePosesNear>("ArePosesNear");
}
