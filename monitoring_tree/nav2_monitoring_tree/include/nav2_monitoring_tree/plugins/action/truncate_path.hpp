#ifndef NAV2_MONITORING_TREE__PLUGINS__ACTION__TRUNCATE_PATH_HPP_
#define NAV2_MONITORING_TREE__PLUGINS__ACTION__TRUNCATE_PATH_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_monitoring_tree
{

// Pure-computation monitoring stub: returns SUCCESS when the {path} blackboard
// key is non-empty (i.e. ComputePathToPose produced a path), FAILURE otherwise.
class TruncatePath : public BT::SyncActionNode
{
public:
  TruncatePath(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
};

}  // namespace nav2_monitoring_tree
#endif  // NAV2_MONITORING_TREE__PLUGINS__ACTION__TRUNCATE_PATH_HPP_
