#include "nav2_bt_navigator/plugins/set_costmap_inflation_radius.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <memory>

namespace nav2_bt_navigator
{

SetCostmapInflationRadius::SetCostmapInflationRadius(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<rcl_interfaces::srv::SetParameters>(
    service_node_name, conf)
{
  // Build the blackboard key from the resolved service name so that every
  // instance targeting the same costmap shares one cache entry.
  // service_name_ is set by BtServiceNode's constructor from the "service_name" port.
  bb_cache_key_ = "inflation_cache_" + service_name_;
}

BT::NodeStatus SetCostmapInflationRadius::tick()
{
  double requested_radius;
  if (!getInput("inflation_radius", requested_radius)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[SetCostmapInflationRadius] Missing required port 'inflation_radius'");
    return BT::NodeStatus::FAILURE;
  }

  // ── Shared blackboard cache check ────────────────────────────────────────
  // Read the last value that *any* instance successfully wrote for this
  // service.  Using the blackboard (not a member variable) is essential:
  // if a different BT branch changed the parameter, this instance must
  // detect that and send a restore call rather than assuming its own
  // last-written value is still current.
  double cached_radius = std::numeric_limits<double>::quiet_NaN();
  // Return value is false when the key is absent; NaN sentinel handles that case.
  [[maybe_unused]] bool found = config().blackboard->get(bb_cache_key_, cached_radius);

  if (!std::isnan(cached_radius) && cached_radius == requested_radius) {
    // Already at the requested value — skip the service call entirely.
    return BT::NodeStatus::SUCCESS;
  }

  // Stash so on_completion() can update the blackboard without re-reading the port.
  pending_radius_ = requested_radius;

  // Delegate to BtServiceNode for async service call + spin.
  return nav2_behavior_tree::BtServiceNode<rcl_interfaces::srv::SetParameters>::tick();
}

void SetCostmapInflationRadius::on_tick()
{
  std::string param_name{"inflation_layer.inflation_radius"};
  getInput("param_name", param_name);  // use default if port is not set in XML

  rcl_interfaces::msg::Parameter param;
  param.name = param_name;
  param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  param.value.double_value = pending_radius_;
  request_->parameters.push_back(param);

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[SetCostmapInflationRadius] Requesting %s = %.4f on service '%s'",
    param_name.c_str(), pending_radius_, service_name_.c_str());
}

BT::NodeStatus SetCostmapInflationRadius::on_completion(
  std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response)
{
  if (response->results.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[SetCostmapInflationRadius] SetParameters returned empty results for service '%s'",
      service_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!response->results[0].successful) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[SetCostmapInflationRadius] SetParameters failed for service '%s': %s",
      service_name_.c_str(), response->results[0].reason.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Write the confirmed value into the shared blackboard cache so that all
  // other instances targeting the same costmap see the updated ground truth.
  config().blackboard->set(bb_cache_key_, pending_radius_);

  RCLCPP_INFO(
    node_->get_logger(),
    "[SetCostmapInflationRadius] '%s' inflation_radius set to %.4f",
    service_name_.c_str(), pending_radius_);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_navigator

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_navigator::SetCostmapInflationRadius>(
    "SetCostmapInflationRadius");
}
