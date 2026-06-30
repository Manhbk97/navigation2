# TB3 Simulation Package

This package has the following
- scripts to 
  - set initial amcl pose of the robot
- launch files to 
  - launch TurtleBot3 in a gazebo world
  - launch nav2 node with all requirements, initialise amcl pose  and rviz visualisation
- infrastructure files in config, map



b3_autonomy/
├── CMakeLists.txt
├── package.xml
├── bt_xml/
│   └── tree.xml              # Behavior tree definition
├── include/
│   ├── autonomy_node.h       # Main node header
│   └── navigation_behaviors.h # BT node header
├── src/
│   ├── autonomy_node.cpp     # Main node implementation
│   └── navigation_behaviors.cpp # GoToPose BT action
└── launch/                   # Launch files (not shown)
Components
File	Purpose
autonomy_node.cpp	Main ROS2 node that creates and ticks the behavior tree every 500ms
navigation_behaviors.cpp	GoToPose BT action node that sends goals to Nav2
tree.xml	Behavior tree: sequentially visits location1 → location2 → location3 → location4
Inputs
Input	Type	Description
location_file	ROS2 Parameter	Path to YAML file containing named poses (x, y, theta)
loc	BT Port	Location key name (e.g., "location1") from the YAML file
Outputs (Actions/Topics)
Interface	Type	Description
/apple/navigate_to_pose	Action Client	Sends nav2_msgs/action/NavigateToPose goals to Nav2
Dependencies
rclcpp, rclcpp_action - ROS2 core
nav2_msgs - Nav2 action definitions
behaviortree_cpp_v3 - Behavior tree library
yaml-cpp - YAML parsing for location file
tf2, tf2_geometry_msgs - Quaternion conversion
Flow Diagram

┌─────────────────┐      ┌──────────────┐      ┌─────────────────────────┐
│  location.yaml  │ ──►  │  GoToPose    │ ──►  │ /apple/navigate_to_pose │
│  (x, y, theta)  │      │  (BT Node)   │      │  (Nav2 Action Server)   │
└─────────────────┘      └──────────────┘      └─────────────────────────┘
                               ▲
                               │ ticks every 500ms
                         ┌─────┴─────┐
                         │ tree.xml  │
                         │ Sequence  │
                         └───────────┘


 ros2 launch tb3_autonomy autonomy.launch.py  