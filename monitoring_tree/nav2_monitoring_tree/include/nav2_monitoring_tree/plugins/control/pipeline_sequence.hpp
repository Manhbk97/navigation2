#ifndef NAV2_MONITORING_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_
#define NAV2_MONITORING_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_

#include "behaviortree_cpp_v3/control_node.h"

namespace nav2_monitoring_tree
{

// Ticks ALL children every tick; returns FAILURE if any child fails,
// otherwise returns RUNNING. This is Nav2's standard navigation-loop control.
class PipelineSequence : public BT::ControlNode
{
public:
  PipelineSequence(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
};

}  // namespace nav2_monitoring_tree
#endif  // NAV2_MONITORING_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_
