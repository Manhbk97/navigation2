#include "behavior_plugins/reset_path.hpp"

namespace behavior_plugins
{

BT::NodeStatus ResetPath::tick()
{
  setOutput("path", nav_msgs::msg::Path{});
  return BT::NodeStatus::SUCCESS;
  // return BT::NodeStatus::FAILURE;
}

}  // namespace behavior_plugins
