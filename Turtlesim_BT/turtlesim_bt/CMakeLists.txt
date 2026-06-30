cmake_minimum_required(VERSION 3.8)
project(turtlesim_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(turtlesim REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(turtle_bt src/turtle_bt.cpp)

ament_target_dependencies(turtle_bt
  rclcpp
  geometry_msgs
  behaviortree_cpp_v3
  turtlesim
  ament_index_cpp
)

target_link_libraries(turtle_bt zmq)


add_executable(monitoring_nav2 src/monitoring_nav2.cpp)

ament_target_dependencies(monitoring_nav2
  rclcpp
  geometry_msgs
  behaviortree_cpp_v3
  ament_index_cpp
  action_msgs
  std_msgs
  nav_msgs
)

target_link_libraries(monitoring_nav2 zmq)


install(TARGETS
  turtle_bt
  monitoring_nav2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
