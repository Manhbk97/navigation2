#ifndef BEHAVIOR_PLUGINS__SET_COSTMAP_INFLATION_RADIUS_HPP_
#define BEHAVIOR_PLUGINS__SET_COSTMAP_INFLATION_RADIUS_HPP_

#include <limits>
#include <memory>
#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

namespace behavior_plugins
{

/**
 * @brief BT Action node that calls rcl_interfaces/srv/SetParameters to change
 *        the inflation_radius of a running costmap inflation layer at runtime.
 *
 * Cache strategy — blackboard, not instance member
 * ─────────────────────────────────────────────────
 * The BT XML contains multiple instances of this node (one per costmap per
 * branch). Using a plain member variable for the cache is incorrect: instance A
 * (narrow branch, local costmap) and instance B (normal branch, local costmap)
 * target the same service but have independent member variables. When A sets
 * the parameter to 0.25, B's cache still says 0.45, so B incorrectly skips
 * the restore call on the next tick and inflation is never recovered.
 *
 * The fix is to store the cache on the BT blackboard, keyed by service name:
 *   "inflation_cache_<service_name>"
 * All instances that target the same costmap share one blackboard entry, so
 * any write is immediately visible to every other instance.
 *
 * Ports
 * -----
 *   service_name     (input) – SetParameters service, e.g.
 *                              "local_costmap/local_costmap/set_parameters"
 *                              (relative name → resolved under the robot namespace)
 *   inflation_radius (input) – target inflation radius in metres
 *   param_name       (input) – ROS 2 parameter name inside the costmap node,
 *                              default: "inflation_layer.inflation_radius"
 *   server_timeout   (input) – inherited from BtServiceNode
 */
class SetCostmapInflationRadius
  : public nav2_behavior_tree::BtServiceNode<rcl_interfaces::srv::SetParameters>
{
public:
  SetCostmapInflationRadius(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  SetCostmapInflationRadius() = delete;

  /**
   * @brief Override tick() to short-circuit the service call when the
   *        requested value already equals the last successfully written value.
   *        The base class returns FAILURE when should_send_request_ is false,
   *        so we intercept the cache-hit case here and return SUCCESS directly.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Populate the SetParameters request with the desired inflation radius.
   */
  void on_tick() override;

  /**
   * @brief Validate the service response and update the cached value on success.
   */
  BT::NodeStatus on_completion(
    std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("inflation_radius", "Target inflation radius in metres"),
      BT::InputPort<std::string>(
        "param_name",
        "inflation_layer.inflation_radius",
        "ROS 2 parameter name for inflation radius inside the costmap node"),
    });
  }

private:
  // Blackboard key used to share the cache across all instances that target the
  // same costmap service.  Set once in the constructor from service_name_.
  std::string bb_cache_key_;

  // Staging slot: on_tick() stashes the requested value so on_completion() can
  // update the blackboard without re-calling getInput.
  double pending_radius_{0.0};
};

}  // namespace behavior_plugins

#endif  // BEHAVIOR_PLUGINS__SET_COSTMAP_INFLATION_RADIUS_HPP_
