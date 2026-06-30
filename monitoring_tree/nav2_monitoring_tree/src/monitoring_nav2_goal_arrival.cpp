/**
 * monitoring_nav2_goal_arrival.cpp
 *
 * Thin runner for the two-phase goal-arrival monitoring behavior tree:
 *   navigate_to_pose_w_infinite_replanning_and_near_goal_recovery.xml
 *
 * Frame configuration (global_frame, robot_base_frame, transform_tolerance)
 * is NOT hard-coded here.  TF-aware plugins resolve their frame names via:
 *   1. BT InputPort  (per-node XML attribute, highest priority)
 *   2. ROS node parameter  (set by this runner or via launch file)
 *   3. Built-in fallback inside the plugin (map / base_link / 0.1 s)
 *
 * This mirrors the BTcpp v4 BT::deconflictPortAndParamFrame pattern.
 *
 * ROS parameters
 * ──────────────
 *   nav2_namespace        (string, default "")         — Nav2 namespace prefix
 *   global_frame          (string, default "map")      — TF world frame
 *   robot_base_frame      (string, default "base_link")— TF robot frame
 *   transform_tolerance   (double, default 0.1 s)      — TF lookup timeout
 *   zmq_publisher_port    (int,    default 1669)        — Groot publisher
 *   zmq_server_port       (int,    default 1670)        — Groot server
 */

#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "plugins_list.hpp"   // CMake-generated: BT_BUILTIN_PLUGINS

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // allow_undeclared_parameters=true lets users pass optional overrides such as
  //   --ros-args -p global_frame:=odom -p robot_base_frame:=my_robot/base_link
  //              -p transform_tolerance:=0.3
  // without the runner needing to know about them in advance.
  // Plugins call node->has_parameter(key) → returns true only when the user
  // actually set the value, so the namespace-derived default is used otherwise.
  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true);
  auto node = rclcpp::Node::make_shared("nav2_goal_arrival_monitor", opts);

  // ── Declare required parameters ───────────────────────────────────────────
  // Frame names (global_frame, robot_base_frame, transform_tolerance) are
  // intentionally NOT declared here.  TF-aware plugins resolve them via:
  //   1. XML InputPort attribute  (per BT-node override)
  //   2. ROS parameter passed by the user  (picked up via has_parameter / get_parameter)
  //   3. Namespace-derived fallback inside the plugin:
  //        global_frame      → "map"
  //        robot_base_frame  → "{ns}/base_footprint"
  node->declare_parameter("nav2_namespace",     std::string(""));
  node->declare_parameter("zmq_publisher_port", 1671);
  node->declare_parameter("zmq_server_port",    1672);

  const std::string ns           = node->get_parameter("nav2_namespace").as_string();
  const int         zmq_pub_port = node->get_parameter("zmq_publisher_port").as_int();
  const int         zmq_srv_port = node->get_parameter("zmq_server_port").as_int();

  RCLCPP_INFO(
    node->get_logger(),
    "Nav2 Goal-Arrival Monitor — namespace: '%s' "
    "(TF defaults: global_frame='map', robot_base_frame='%sbase_footprint')",
    ns.c_str(), ns.empty() ? "" : (ns + "/").c_str());

  // ── TF2 setup ─────────────────────────────────────────────────────────────
  auto tf_buffer   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);

  // ── Blackboard ────────────────────────────────────────────────────────────
  // Only three keys are set here:
  //   "node"           — shared_ptr to this ROS node (plugins call its logger,
  //                      get_parameter, create_subscription, etc.)
  //   "nav2_namespace" — used to build fully-qualified topic names
  //   "tf_buffer"      — shared TF2 buffer for pose lookups
  //
  // Frame names (global_frame, robot_base_frame, transform_tolerance) are NOT
  // put on the blackboard.  TF-aware plugins resolve them via getFromPortOrParam
  // which queries the BT InputPort first, then node->get_parameter.
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node",           node);
  blackboard->set("nav2_namespace", ns);
  blackboard->set("tf_buffer",      tf_buffer);

  // ── Plugin loading ────────────────────────────────────────────────────────
  // All plugin names come from plugins_list.hpp (CMake-generated from the
  // plugin_libs variable in CMakeLists.txt) — no hardcoded list here.
  // To add a plugin: add_library(...) + list(APPEND plugin_libs ...) in
  // CMakeLists.txt, then rebuild. This file needs no changes.
  BT::BehaviorTreeFactory factory;

  const std::string pkg_prefix =
    ament_index_cpp::get_package_prefix("nav2_monitoring_tree");

  std::istringstream ss(nav2_monitoring_tree::BT_BUILTIN_PLUGINS);
  for (std::string lib; std::getline(ss, lib, ';'); ) {
    if (lib.empty()) {continue;}
    const std::string so_path = pkg_prefix + "/lib/lib" + lib + ".so";
    RCLCPP_DEBUG(node->get_logger(), "Loading plugin: %s", so_path.c_str());
    factory.registerFromPlugin(so_path);
  }

  // ── Tree creation ─────────────────────────────────────────────────────────
  const std::string pkg_share =
    ament_index_cpp::get_package_share_directory("nav2_monitoring_tree");
  const std::string tree_xml =
    pkg_share + "/config/"
    "navigate_to_pose_w_infinite_replanning_and_near_goal_recovery.xml";

  auto tree = factory.createTreeFromFile(tree_xml, blackboard);

  // ── Loggers ───────────────────────────────────────────────────────────────
  const std::string log_path = pkg_share + "/config/nav2_goal_arrival_bt_log.fbl";
  BT::FileLogger file_logger(tree, log_path.c_str());
  BT::PublisherZMQ zmq_publisher(tree, 25, zmq_pub_port, zmq_srv_port);

  RCLCPP_INFO(
    node->get_logger(),
    "Goal-arrival tree loaded. "
    "Groot → Real-time on publisher port %d / server port %d. Ticking at 10 Hz.",
    zmq_pub_port, zmq_srv_port);

  // ── Main loop ─────────────────────────────────────────────────────────────
  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
