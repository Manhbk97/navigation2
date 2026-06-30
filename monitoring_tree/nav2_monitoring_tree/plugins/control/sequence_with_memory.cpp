// SequenceWithMemory: BTcpp v4 name for what BTcpp v3 calls SequenceStar.
//
// SequenceStar remembers which children already returned SUCCESS across ticks —
// when a child returns RUNNING the loop is NOT restarted on the next tick.
// This is the correct semantics for an "atomic recovery block" where
// Spin/Wait/BackUp must run to completion before the next action starts.
//
// The v3 SequenceStarNode only exposes a single-arg constructor, so we wrap
// it with a NodeConfiguration constructor so it can be loaded as a plugin.
#include "behaviortree_cpp_v3/controls/sequence_star_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_monitoring_tree
{

class SequenceWithMemory : public BT::SequenceStarNode
{
public:
  SequenceWithMemory(
    const std::string & name,
    const BT::NodeConfiguration & /*config*/)
  : BT::SequenceStarNode(name) {}

  static BT::PortsList providedPorts() { return {}; }
};

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::SequenceWithMemory>(
    "SequenceWithMemory");
}
