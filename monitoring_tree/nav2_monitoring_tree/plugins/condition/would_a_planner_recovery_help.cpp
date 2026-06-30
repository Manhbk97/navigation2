// Monitoring stub — mirrors WouldAControllerRecoveryHelp rationale.
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_monitoring_tree
{

class WouldAPlannerRecoveryHelp : public BT::ConditionNode
{
public:
  WouldAPlannerRecoveryHelp(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("error_code", 0, "Planner error code (not inspected)")};
  }

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::WouldAPlannerRecoveryHelp>(
    "WouldAPlannerRecoveryHelp");
}
