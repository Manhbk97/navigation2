#include "nav2_monitoring_tree/plugins/action/truncate_path.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_monitoring_tree
{

TruncatePath::TruncatePath(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList TruncatePath::providedPorts()
{
  return {
    BT::InputPort<std::string>("distance"),
    BT::InputPort<std::string>("input_path"),
    BT::OutputPort<std::string>("output_path"),
  };
}

BT::NodeStatus TruncatePath::tick()
{
  // input_path is fed from the {path} blackboard key set by ComputePathToPose.
  // Return SUCCESS (path exists) so FollowPath can proceed; FAILURE while waiting.
  const auto path = getInput<std::string>("input_path");
  if (path && !path.value().empty()) {
    setOutput("output_path", "truncated");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::TruncatePath>("TruncatePath");
}
