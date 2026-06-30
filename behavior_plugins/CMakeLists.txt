cmake_minimum_required(VERSION 3.8)
project(behavior_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Create the behavior plugins library
add_library(behavior_plugins SHARED
  src/heading_correction.cpp
  src/human_in_narrow_path.cpp
  src/narrow_path.cpp
  src/nearest_free_point_on_path.cpp
  src/recovery_once_per_location.cpp
  src/reset_path.cpp
  src/search_local_goal_aside.cpp
  src/set_costmap_inflation_radius.cpp
  src/set_goal_from_location.cpp
  src/is_goal_nearby_condition.cpp
  src/validate_path_action.cpp
  # src/keep_running_until_failure.cpp
  src/plugin_registration.cpp
)

target_include_directories(behavior_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Required by BehaviorTree.CPP so BT_REGISTER_NODES exports the
# BT_RegisterNodesFromPlugin symbol as extern "C" instead of static.
target_compile_definitions(behavior_plugins PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(behavior_plugins
  rclcpp
  pluginlib
  nav2_behavior_tree
  nav2_util
  behaviortree_cpp
  std_msgs
  nav_msgs
  nav2_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  rcl_interfaces
  yaml_cpp_vendor
)

target_link_libraries(behavior_plugins yaml-cpp)

# Plugin nodes are registered via BT_REGISTER_NODES macro in source files

# Install targets
install(
  TARGETS behavior_plugins
  DESTINATION lib
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install behavior tree files
install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
