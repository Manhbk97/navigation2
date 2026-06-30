cmake_minimum_required(VERSION 3.8)
project(nav2_monitoring_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(turtlesim REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Common dependency list shared by all monitoring plugins.
set(monitor_deps
  rclcpp
  behaviortree_cpp_v3
  action_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

# Extended deps for plugins that require TF.
set(monitor_tf_deps
  ${monitor_deps}
  tf2_ros
  tf2_geometry_msgs
)

# ── Plugin shared libraries ───────────────────────────────────────────────────
# Each plugin is its own .so loaded at runtime via BT::SharedLibrary.
# Adding a new plugin: create the header + cpp, then add one block below.

# control/
add_library(nav2_monitoring_tree_pipeline_sequence SHARED
  plugins/control/pipeline_sequence.cpp)
target_include_directories(nav2_monitoring_tree_pipeline_sequence PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_pipeline_sequence ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_pipeline_sequence PRIVATE BT_PLUGIN_EXPORT)

# decorator/
add_library(nav2_monitoring_tree_rate_controller SHARED
  plugins/decorator/rate_controller.cpp)
target_include_directories(nav2_monitoring_tree_rate_controller PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_rate_controller ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_rate_controller PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_goal_updater SHARED
  plugins/decorator/goal_updater.cpp)
target_include_directories(nav2_monitoring_tree_goal_updater PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_goal_updater ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_goal_updater PRIVATE BT_PLUGIN_EXPORT)

# action/
add_library(nav2_monitoring_tree_controller_selector_monitor SHARED
  plugins/action/controller_selector_monitor.cpp)
target_include_directories(nav2_monitoring_tree_controller_selector_monitor PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_controller_selector_monitor ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_controller_selector_monitor PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_planner_selector_monitor SHARED
  plugins/action/planner_selector_monitor.cpp)
target_include_directories(nav2_monitoring_tree_planner_selector_monitor PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_planner_selector_monitor ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_planner_selector_monitor PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_compute_path_to_pose_monitor SHARED
  plugins/action/compute_path_to_pose_monitor.cpp)
target_include_directories(nav2_monitoring_tree_compute_path_to_pose_monitor PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_compute_path_to_pose_monitor ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_compute_path_to_pose_monitor PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_truncate_path SHARED
  plugins/action/truncate_path.cpp)
target_include_directories(nav2_monitoring_tree_truncate_path PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_truncate_path ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_truncate_path PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_follow_path_monitor SHARED
  plugins/action/follow_path_monitor.cpp)
target_include_directories(nav2_monitoring_tree_follow_path_monitor PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_follow_path_monitor ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_follow_path_monitor PRIVATE BT_PLUGIN_EXPORT)

# ── control/ ─────────────────────────────────────────────────────────────────
add_library(nav2_monitoring_tree_recovery_node SHARED
  plugins/control/recovery_node.cpp)
target_include_directories(nav2_monitoring_tree_recovery_node PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_recovery_node ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_recovery_node PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_round_robin SHARED
  plugins/control/round_robin_node.cpp)
target_include_directories(nav2_monitoring_tree_round_robin PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_round_robin ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_round_robin PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_sequence_with_memory SHARED
  plugins/control/sequence_with_memory.cpp)
target_include_directories(nav2_monitoring_tree_sequence_with_memory PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_sequence_with_memory ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_sequence_with_memory PRIVATE BT_PLUGIN_EXPORT)

# ── condition/ ───────────────────────────────────────────────────────────────
add_library(nav2_monitoring_tree_would_a_controller_recovery_help SHARED
  plugins/condition/would_a_controller_recovery_help.cpp)
target_include_directories(nav2_monitoring_tree_would_a_controller_recovery_help PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_would_a_controller_recovery_help ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_would_a_controller_recovery_help PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_would_a_planner_recovery_help SHARED
  plugins/condition/would_a_planner_recovery_help.cpp)
target_include_directories(nav2_monitoring_tree_would_a_planner_recovery_help PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_would_a_planner_recovery_help ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_would_a_planner_recovery_help PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_goal_updated SHARED
  plugins/condition/goal_updated.cpp)
target_include_directories(nav2_monitoring_tree_goal_updated PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_goal_updated ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_goal_updated PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_are_poses_near SHARED
  plugins/condition/are_poses_near.cpp)
target_include_directories(nav2_monitoring_tree_are_poses_near PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_are_poses_near ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_are_poses_near PRIVATE BT_PLUGIN_EXPORT)

# ── action/ ──────────────────────────────────────────────────────────────────
add_library(nav2_monitoring_tree_get_current_pose SHARED
  plugins/action/get_current_pose.cpp)
target_include_directories(nav2_monitoring_tree_get_current_pose PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_get_current_pose ${monitor_tf_deps})
target_compile_definitions(nav2_monitoring_tree_get_current_pose PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_spin_monitor SHARED
  plugins/action/spin_monitor.cpp)
target_include_directories(nav2_monitoring_tree_spin_monitor PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_spin_monitor ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_spin_monitor PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_wait_monitor SHARED
  plugins/action/wait_monitor.cpp)
target_include_directories(nav2_monitoring_tree_wait_monitor PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_wait_monitor ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_wait_monitor PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_back_up_monitor SHARED
  plugins/action/back_up_monitor.cpp)
target_include_directories(nav2_monitoring_tree_back_up_monitor PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_back_up_monitor ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_back_up_monitor PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_monitoring_tree_clear_costmap_monitor SHARED
  plugins/action/clear_costmap_monitor.cpp)
target_include_directories(nav2_monitoring_tree_clear_costmap_monitor PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_clear_costmap_monitor ${monitor_deps})
target_compile_definitions(nav2_monitoring_tree_clear_costmap_monitor PRIVATE BT_PLUGIN_EXPORT)

# ── decorator/ ───────────────────────────────────────────────────────────────
add_library(nav2_monitoring_tree_recovery_once_per_location SHARED
  plugins/decorator/recovery_once_per_location.cpp)
target_include_directories(nav2_monitoring_tree_recovery_once_per_location PUBLIC include)
ament_target_dependencies(nav2_monitoring_tree_recovery_once_per_location ${monitor_tf_deps})
target_compile_definitions(nav2_monitoring_tree_recovery_once_per_location PRIVATE BT_PLUGIN_EXPORT)

# ── All plugin targets ────────────────────────────────────────────────────────
# This list is the single source of truth for which plugins are built and loaded.
# Adding a plugin: (1) add_library block above, (2) append here, (3) rebuild.
# CMake joins it into a semicolon-separated string for plugins_list.hpp so the
# runner can load every plugin without a hardcoded vector in the .cpp file.
set(plugin_libs
  # Original 8 (nav2_monitor_tree.xml)
  nav2_monitoring_tree_pipeline_sequence
  nav2_monitoring_tree_rate_controller
  nav2_monitoring_tree_goal_updater
  nav2_monitoring_tree_controller_selector_monitor
  nav2_monitoring_tree_planner_selector_monitor
  nav2_monitoring_tree_compute_path_to_pose_monitor
  nav2_monitoring_tree_truncate_path
  nav2_monitoring_tree_follow_path_monitor
  # New 13 (navigate_to_pose_w_infinite_replanning_and_near_goal_recovery.xml)
  nav2_monitoring_tree_recovery_node
  nav2_monitoring_tree_round_robin
  nav2_monitoring_tree_sequence_with_memory
  nav2_monitoring_tree_would_a_controller_recovery_help
  nav2_monitoring_tree_would_a_planner_recovery_help
  nav2_monitoring_tree_goal_updated
  nav2_monitoring_tree_are_poses_near
  nav2_monitoring_tree_get_current_pose
  nav2_monitoring_tree_spin_monitor
  nav2_monitoring_tree_wait_monitor
  nav2_monitoring_tree_back_up_monitor
  nav2_monitoring_tree_clear_costmap_monitor
  nav2_monitoring_tree_recovery_once_per_location
)

# ── Generate plugins_list.hpp (mirrors nav2_behavior_tree pattern) ────────────
# CMake lists are semicolon-separated internally, so @plugin_libs@ in the .in
# file becomes "lib1;lib2;..." — exactly what the runner splits on at runtime.
set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY ${GENERATED_DIR})
configure_file(plugins_list.hpp.in ${GENERATED_DIR}/plugins_list.hpp @ONLY)

# Thin runner: loads plugins at startup, no class definitions here.
add_executable(monitoring_nav2 src/monitoring_nav2_main.cpp)
ament_target_dependencies(monitoring_nav2
  rclcpp
  behaviortree_cpp_v3
  ament_index_cpp
)
target_include_directories(monitoring_nav2 PUBLIC include ${GENERATED_DIR})
target_link_libraries(monitoring_nav2 zmq)

# Goal-arrival monitor: two-phase BT with recovery (TF-aware).
add_executable(monitoring_nav2_goal_arrival src/monitoring_nav2_goal_arrival.cpp)
ament_target_dependencies(monitoring_nav2_goal_arrival
  rclcpp
  behaviortree_cpp_v3
  ament_index_cpp
  tf2_ros
  tf2_geometry_msgs
)
target_include_directories(monitoring_nav2_goal_arrival PUBLIC include ${GENERATED_DIR})
target_link_libraries(monitoring_nav2_goal_arrival zmq)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  monitoring_nav2
  monitoring_nav2_goal_arrival
  ${plugin_libs}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include/)

# Install the CMake-generated plugins list header so other packages can include it.
install(FILES ${GENERATED_DIR}/plugins_list.hpp
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install the Groot palette at the package share root (mirrors nav2_behavior_tree).
install(FILES nav2_tree_nodes.xml
  DESTINATION share/${PROJECT_NAME})

# ── Export ────────────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${monitor_deps} tf2_ros tf2_geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
