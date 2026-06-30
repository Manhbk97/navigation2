#ifndef NAV2_BT_NAVIGATOR__PLUGINS__RESET_PATH_HPP_
#define NAV2_BT_NAVIGATOR__PLUGINS__RESET_PATH_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include "nav_msgs/msg/path.hpp"

namespace nav2_bt_navigator
{

/**
 * @brief SyncActionNode that clears a nav_msgs::msg::Path entry on the blackboard.
 *
 * Use this instead of <SetBlackboard output_key="path" value=""/> which fails
 * because BT.CPP cannot parse an empty string into a typed Path entry.
 *
 * XML usage:
 *   <ResetPath path="{path}"/>
 *
 * After this node runs the blackboard entry holds a default-constructed
 * (empty) nav_msgs::msg::Path, which causes FollowPath to fail on the next
 * tick so that ComputePathToPose can replan.
 */
class ResetPath : public BT::SyncActionNode
{
public:
  ResetPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<nav_msgs::msg::Path>("path", "Blackboard path entry to clear"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__PLUGINS__RESET_PATH_HPP_
