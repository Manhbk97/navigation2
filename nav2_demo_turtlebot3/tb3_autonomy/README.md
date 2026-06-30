# TB3 Autonomy Package

This core autonomy package has the following
- autonomy node - ros 2 node for core robot autonomy logic
- navigation behaviors - behavior tree nodes

It has a launch file to launch autonomy node

Additional components
- behavior tree design in bt_xml




### Data Flow Diagram

┌─────────────────────────────────────────────────────────────────┐
│                        AutonomyNode                             │
│  (500ms timer ticks the behavior tree)                          │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│              Behavior Tree (tree.xml)                           │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ Sequence                                                   │ │
│  │  ├─ GoToPose (location1) ──┐                               │ │
│  │  ├─ GoToPose (location2) ──┤                               │ │
│  │  ├─ GoToPose (location3) ──┼── Each waits for success      │ │
│  │  └─ GoToPose (location4) ──┘   before next executes        │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────┬───────────────────────────────────────────┘
                      │ Reads waypoints
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│  YAML Config (sim_house_locations.yaml)                         │
│  location1: [x, y, theta]                                       │
│  location2: [x, y, theta]                                       │
│  ...                                                            │
└─────────────────────────────────────────────────────────────────┘
                      │
                      │ Sends NavigateToPose action
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│          Nav2 Action Server                                     │
│  /apple/navigate_to_pose                                        │
│  - Plans path                                                   │
│  - Executes navigation                                          │
│  - Returns SUCCESS/ABORTED/CANCELED                             │
└─────────────────────────────────────────────────────────────────┘



### Package Structure

tb3_autonomy/
├── bt_xml/
│   └── tree.xml                    # Behavior tree definition (4 waypoint sequence)
├── include/
│   ├── autonomy_node.h             # Main node header
│   └── navigation_behaviors.h      # GoToPose behavior header
├── src/
│   ├── autonomy_node.cpp           # Main node implementation
│   └── navigation_behaviors.cpp    # GoToPose behavior implementation
├── launch/
│   └── autonomy.launch.py          # Launch configuration
├── CMakeLists.txt
└── package.xml