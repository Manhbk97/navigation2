#include "nav2_bt_navigator/plugins/reset_path.hpp"

namespace nav2_bt_navigator
{

BT::NodeStatus ResetPath::tick()
{
  setOutput("path", nav_msgs::msg::Path{});
  return BT::NodeStatus::SUCCESS;
  // return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_bt_navigator

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_navigator::ResetPath>("ResetPath");
}
