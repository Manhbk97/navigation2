#include "nav2_monitoring_tree/plugins/control/pipeline_sequence.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_monitoring_tree
{

PipelineSequence::PipelineSequence(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{}

BT::PortsList PipelineSequence::providedPorts()
{
  return {};
}

BT::NodeStatus PipelineSequence::tick()
{
  for (auto * child : children_nodes_) {
    auto status = child->executeTick();
    if (status == BT::NodeStatus::FAILURE) {
      haltChildren();
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::PipelineSequence>("PipelineSequence");
}
