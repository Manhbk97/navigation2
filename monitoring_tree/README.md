# nav2_monitoring_tree

Passive behavior-tree monitor for Nav2.  
It mirrors Nav2's own BT execution by **subscribing to action-status topics and TF** — no commands are ever sent to the robot.  
Two executables are provided:

| Executable | Tree XML | Purpose |
|---|---|---|
| `monitoring_nav2` | `nav2_monitor_tree.xml` | Simple follow-point monitor |
| `monitoring_nav2_goal_arrival` | `navigate_to_pose_w_infinite_replanning_and_near_goal_recovery.xml` | Two-phase goal-arrival monitor with recovery tracking |

---

## Prerequisites

```bash
# Source your ROS 2 Jazzy workspace
source /opt/ros/jazzy/setup.bash

# Build the package (first time or after source changes)
cd ~/rgt2_ws
colcon build --packages-select nav2_monitoring_tree --symlink-install
source install/setup.bash
```

---

## 1. Simple follow-point monitor (`monitoring_nav2`)

Monitors the `ControllerSelector → PlannerSelector → ComputePathToPose → FollowPath` pipeline via the provided launch file.

### Basic launch (no Nav2 namespace)

```bash
ros2 launch nav2_monitoring_tree launch_nav2_bt_launch.py
```

### With a Nav2 namespace

```bash
ros2 launch nav2_monitoring_tree launch_nav2_bt_launch.py nav2_namespace:=apple
```

### With custom Groot ports

```bash
ros2 launch nav2_monitoring_tree launch_nav2_bt_launch.py \
  nav2_namespace:=apple \
  zmq_publisher_port:=1668 \
  zmq_server_port:=1669
```

### Direct `ros2 run` (no launch file)

```bash
ros2 run nav2_monitoring_tree monitoring_nav2 \
  --ros-args \
  -p nav2_namespace:=apple \
  -p zmq_publisher_port:=1668 \
  -p zmq_server_port:=1669
```

---

## 2. Two-phase goal-arrival monitor (`monitoring_nav2_goal_arrival`)

Monitors the full two-phase navigation + recovery tree.  
Uses TF to track the robot's distance from the goal and detect stuck/recovery states.

### Basic launch (no namespace, default frames)

```bash
ros2 run nav2_monitoring_tree monitoring_nav2_goal_arrival
```

Defaults used when no parameters are given:

| Parameter | Default | Description |
|---|---|---|
| `nav2_namespace` | `""` | Nav2 node namespace prefix |
| `global_frame` | `map` | TF world frame |
| `robot_base_frame` | `base_link` | TF robot base frame |
| `transform_tolerance` | `0.1` | TF lookup timeout (seconds) |
| `zmq_publisher_port` | `1669` | Groot Real-time publisher port |
| `zmq_server_port` | `1670` | Groot Real-time server port |

### With a Nav2 namespace

```bash
ros2 run nav2_monitoring_tree monitoring_nav2_goal_arrival \
  --ros-args \
  -p nav2_namespace:=apple
```

### Custom robot frames (e.g. `base_footprint` in `odom`)

```bash
ros2 run nav2_monitoring_tree monitoring_nav2_goal_arrival \
  --ros-args \
  -p nav2_namespace:=apple \
  -p global_frame:=odom \
  -p robot_base_frame:=base_footprint \
  -p transform_tolerance:=0.3
```

### Per-node XML frame override

Frames can also be set per BT-node directly in the XML (highest priority,
overrides any ROS parameter):

```xml
<GetCurrentPose current_pose="{current_pose}"
                global_frame="map"
                robot_base_frame="base_footprint"
                transform_tolerance="0.2"/>
```

### Priority of frame configuration

```
XML InputPort attribute   (highest — per BT node)
        ↓
ROS node parameter        (set via --ros-args or launch file)
        ↓
Plugin built-in fallback  (lowest — "map" / "base_link" / 0.1 s)
```

---

## 3. Visualise with Groot

[Groot](https://github.com/BehaviorTree/Groot) lets you watch the tree execute in real time.

1. Install Groot (if not already installed):
   ```bash
   sudo apt install ros-jazzy-groot2   # or build from source
   ```
2. Open Groot → click **Real-time Monitor**
3. Connect with the ports matching what the node prints on startup:

   | Monitor | Publisher port | Server port |
   |---|---|---|
   | `monitoring_nav2` (launch default) | 1668 | 1669 |
   | `monitoring_nav2_goal_arrival` (run default) | 1669 | 1670 |

> **Port conflict tip:** if you run both monitors at the same time, assign different ports with `-p zmq_publisher_port:=...` and `-p zmq_server_port:=...`.

---

## 4. Topic reference

The monitors subscribe to the following topics (prefixed with `/<nav2_namespace>` when a namespace is set):

| Topic | Type | Used by |
|---|---|---|
| `controller_selector` | `std_msgs/String` | ControllerSelector |
| `planner_selector` | `std_msgs/String` | PlannerSelector |
| `compute_path_to_pose/_action/status` | `action_msgs/GoalStatusArray` | ComputePathToPose |
| `follow_path/_action/status` | `action_msgs/GoalStatusArray` | FollowPath |
| `spin/_action/status` | `action_msgs/GoalStatusArray` | Spin (goal-arrival monitor) |
| `wait/_action/status` | `action_msgs/GoalStatusArray` | Wait (goal-arrival monitor) |
| `backup/_action/status` | `action_msgs/GoalStatusArray` | BackUp (goal-arrival monitor) |

TF is used by `GetCurrentPose` and `RecoveryOncePerLocation` to read the robot pose (`robot_base_frame` → `global_frame`).

---

## 5. Package structure

```
nav2_monitoring_tree/
├── config/
│   ├── nav2_monitor_tree.xml                                    ← simple follow-point tree
│   └── navigate_to_pose_w_infinite_replanning_and_near_goal_recovery.xml  ← two-phase tree
├── include/nav2_monitoring_tree/
│   ├── nav2_monitor_utils.hpp        ← ns_topic(), actionStatusToNodeStatus(), getFromPortOrParam()
│   └── plugins/…                    ← per-plugin headers (original 8)
├── plugins/
│   ├── action/   ← ComputePathToPose, FollowPath, ControllerSelector, PlannerSelector,
│   │              TruncatePath, GetCurrentPose, Spin, Wait, BackUp, ClearEntireCostmap
│   ├── condition/ ← ArePosesNear, GoalUpdated, WouldAControllerRecoveryHelp,
│   │               WouldAPlannerRecoveryHelp
│   ├── control/  ← PipelineSequence, RecoveryNode, RoundRobin
│   └── decorator/ ← RateController, GoalUpdater, RecoveryOncePerLocation
├── src/
│   ├── monitoring_nav2_main.cpp          ← runner for nav2_monitor_tree.xml
│   └── monitoring_nav2_goal_arrival.cpp  ← runner for two-phase tree (TF-aware)
└── launch/
    └── launch_nav2_bt_launch.py          ← launch for monitoring_nav2
```
