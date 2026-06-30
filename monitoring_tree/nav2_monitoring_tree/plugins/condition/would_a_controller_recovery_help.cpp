// Monitoring stub: in the monitoring tree we cannot inspect the actual Nav2
// error code (only action *status* is observed, not the result payload).
// Returning SUCCESS means "recovery might help" — the recovery branch will
// run, but the Spin/Wait/BackUp monitors will return FAILURE immediately if
// Nav2 is not actually executing those actions, so no spurious recovery loops
// are produced.
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_monitoring_tree
{

class WouldAControllerRecoveryHelp : public BT::ConditionNode
{
public:
  WouldAControllerRecoveryHelp(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    // error_code port declared so the XML attribute is accepted without error.
    return {BT::InputPort<int>("error_code", 0, "Follow-path error code (not inspected)")};
  }

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::WouldAControllerRecoveryHelp>(
    "WouldAControllerRecoveryHelp");
}
