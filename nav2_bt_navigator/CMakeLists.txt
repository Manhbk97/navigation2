cmake_minimum_required(VERSION 3.5)
project(nav2_bt_navigator)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

nav2_package()

include_directories(
  include
)

set(executable_name bt_navigator)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  nav2_behavior_tree
  nav_msgs
  nav2_msgs
  behaviortree_cpp
  std_srvs
  nav2_util
  nav2_core
  tf2_ros
  pluginlib
)

add_library(${library_name} SHARED
  src/bt_navigator.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(nav2_navigate_to_pose_navigator SHARED src/navigators/navigate_to_pose.cpp)
ament_target_dependencies(nav2_navigate_to_pose_navigator ${dependencies})

add_library(nav2_navigate_through_poses SHARED src/navigators/navigate_through_poses.cpp)
ament_target_dependencies(nav2_navigate_through_poses ${dependencies})

##set_goal_from_location plugin
add_library(nav2_set_goal_from_location_bt_node SHARED
  src/plugins/set_goal_from_location.cpp
)
ament_target_dependencies(nav2_set_goal_from_location_bt_node
  ${dependencies}
  tf2
  tf2_geometry_msgs
)
target_link_libraries(nav2_set_goal_from_location_bt_node yaml-cpp)
target_compile_definitions(nav2_set_goal_from_location_bt_node PRIVATE BT_PLUGIN_EXPORT)

## human_in_narrow_path plugin
add_library(nav2_human_in_narrow_path_bt_node SHARED
  src/plugins/human_in_narrow_path.cpp
)
ament_target_dependencies(nav2_human_in_narrow_path_bt_node
  ${dependencies}
)
target_compile_definitions(nav2_human_in_narrow_path_bt_node PRIVATE BT_PLUGIN_EXPORT)

## reset_path plugin
add_library(nav2_reset_path_bt_node SHARED
  src/plugins/reset_path.cpp
)
ament_target_dependencies(nav2_reset_path_bt_node
  ${dependencies}
)
target_compile_definitions(nav2_reset_path_bt_node PRIVATE BT_PLUGIN_EXPORT)

## narrow_path plugin
add_library(nav2_narrow_path_bt_node SHARED
  src/plugins/narrow_path.cpp
)
ament_target_dependencies(nav2_narrow_path_bt_node
  ${dependencies}
)
target_compile_definitions(nav2_narrow_path_bt_node PRIVATE BT_PLUGIN_EXPORT)

## set_costmap_inflation_radius plugin
add_library(nav2_set_costmap_inflation_radius_bt_node SHARED
  src/plugins/set_costmap_inflation_radius.cpp
)
ament_target_dependencies(nav2_set_costmap_inflation_radius_bt_node
  ${dependencies}
  rcl_interfaces
)
target_compile_definitions(nav2_set_costmap_inflation_radius_bt_node PRIVATE BT_PLUGIN_EXPORT)

## search_local_goal_aside plugin
add_library(nav2_search_local_goal_aside_bt_node SHARED
  src/plugins/search_local_goal_aside.cpp
)
ament_target_dependencies(nav2_search_local_goal_aside_bt_node
  ${dependencies}
  tf2
  tf2_geometry_msgs
)
target_compile_definitions(nav2_search_local_goal_aside_bt_node PRIVATE BT_PLUGIN_EXPORT)

## recovery_once_per_location plugin
add_library(nav2_recovery_once_per_location_bt_node SHARED
  src/plugins/recovery_once_per_location.cpp
)
ament_target_dependencies(nav2_recovery_once_per_location_bt_node
  ${dependencies}
  tf2
  tf2_geometry_msgs
)
target_compile_definitions(nav2_recovery_once_per_location_bt_node PRIVATE BT_PLUGIN_EXPORT)

## nearest_free_point_on_path plugin
add_library(nav2_nearest_free_point_on_path_bt_node SHARED
  src/plugins/nearest_free_point_on_path.cpp
)
ament_target_dependencies(nav2_nearest_free_point_on_path_bt_node
  ${dependencies}
  tf2
  tf2_geometry_msgs
)
target_compile_definitions(nav2_nearest_free_point_on_path_bt_node PRIVATE BT_PLUGIN_EXPORT)

## heading_correction plugin
add_library(nav2_heading_correction_bt_node SHARED
  src/plugins/heading_correction.cpp
)
ament_target_dependencies(nav2_heading_correction_bt_node
  ${dependencies}
  tf2
  tf2_geometry_msgs
)
target_compile_definitions(nav2_heading_correction_bt_node PRIVATE BT_PLUGIN_EXPORT)

pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)
rclcpp_components_register_nodes(${library_name} "nav2_bt_navigator::BtNavigator")

install(TARGETS ${library_name} nav2_navigate_to_pose_navigator nav2_navigate_through_poses nav2_set_goal_from_location_bt_node nav2_human_in_narrow_path_bt_node nav2_reset_path_bt_node nav2_narrow_path_bt_node nav2_set_costmap_inflation_radius_bt_node nav2_search_local_goal_aside_bt_node nav2_recovery_once_per_location_bt_node nav2_nearest_free_point_on_path_bt_node nav2_heading_correction_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name}
  nav2_navigate_to_pose_navigator
  nav2_navigate_through_poses
  nav2_set_goal_from_location_bt_node
  nav2_human_in_narrow_path_bt_node
  nav2_reset_path_bt_node nav2_narrow_path_bt_node
  nav2_set_costmap_inflation_radius_bt_node
  nav2_search_local_goal_aside_bt_node
  nav2_recovery_once_per_location_bt_node
  nav2_nearest_free_point_on_path_bt_node
  nav2_heading_correction_bt_node
)
ament_export_dependencies(${dependencies})
ament_package()
