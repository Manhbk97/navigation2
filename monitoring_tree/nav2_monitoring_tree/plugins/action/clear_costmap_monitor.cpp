// ClearEntireCostmapMonitor: monitoring stub for the ClearEntireCostmap service
// call node.  Costmap-clearing is a fire-and-forget service call; monitoring it
// via ROS service events is impractical.  This stub returns SUCCESS immediately
// so the monitoring tree mirrors the usual "clear succeeds" path in Nav2's BT.
#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_monitoring_tree
{

class ClearEntireCostmapMonitor : public BT::SyncActionNode
{
public:
  ClearEntireCostmapMonitor(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("service_name", "Costmap clear service (not called)"),
    };
  }

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::ClearEntireCostmapMonitor>(
    "ClearEntireCostmap");
}
