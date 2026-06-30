// GoalUpdated: returns SUCCESS on the tick immediately after the "goal"
// blackboard entry changes value; returns FAILURE otherwise.
// Used inside ReactiveFallback to short-circuit recovery when a new goal arrives.
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_monitoring_tree
{

class GoalUpdated : public BT::ConditionNode
{
public:
  GoalUpdated(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config), initialized_(false) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped current_goal;

    // Gracefully handle missing blackboard key.
    try {
      config().blackboard->get("goal", current_goal);
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }

    if (!initialized_) {
      goal_ = current_goal;
      initialized_ = true;
      return BT::NodeStatus::FAILURE;
    }

    if (current_goal.pose.position.x != goal_.pose.position.x ||
      current_goal.pose.position.y != goal_.pose.position.y ||
      current_goal.pose.position.z != goal_.pose.position.z ||
      current_goal.pose.orientation.z != goal_.pose.orientation.z ||
      current_goal.pose.orientation.w != goal_.pose.orientation.w)
    {
      goal_ = current_goal;
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

private:
  geometry_msgs::msg::PoseStamped goal_;
  bool initialized_;
};

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::GoalUpdated>("GoalUpdated");
}
