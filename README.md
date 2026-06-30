# navigation2

## Packages

| Package | Description |
|---------|-------------|
| `costmap_converter` | Plugins and nodes to convert occupied costmap2d cells to primitive types (includes `costmap_converter` and `costmap_converter_msgs`) |
| `model` | Robot model files (SDF, URDF/xacro, config) |
| `nav2_behavior_tree` | Nav2 behavior tree engine and BT node plugins (actions, conditions, decorators, controls) |
| `nav2_bt_navigator` | BT Navigator — executes behavior trees for NavigateToPose and NavigateThroughPoses; includes custom BT plugins |
| `nav2_controller` | Controller action interface for Nav2 |
| `nav2_demo_turtlebot3` | Demo and simulation launch files for TurtleBot3 using BT tree and navigation various goals |
| `nav2_mppi_controller` | MPPI (Model Predictive Path Integral) controller plugin for Nav2 |
| `nav2_msgs` | Messages and service files for the Nav2 stack |
| `path_projector` | Path projection utilities |
| `people_msgs` | Message definitions used by nodes in the people stack |
| `teb_local_planner` | TEB (Timed Elastic Band) local planner plugin for the 2D navigation stack |
| `teb_msgs` | Message definitions for teb_local_planner |
| `Turtlesim_BT` | BT-controlled turtlesim demo — battery-aware behavior tree with goal navigation, visual pen/background cues, and Groot real-time monitoring and log replay |
| `velocity_calculator` | Calculates robot velocities from encoder data | 

---

## Custom BT Plugins

### nav2_behavior_tree — Decorator plugins

| Plugin | Type | Description |
|--------|------|-------------|
| `KeepRunningUntilFailure` | Decorator | Returns `RUNNING` while child returns `SUCCESS`; returns `SUCCESS` once child returns `FAILURE`. Used to hold the robot at a wait place until a blocking condition clears. |

### nav2_bt_navigator — Custom plugins

| Plugin | Type | Description |
|--------|------|-------------|
| `HumanInNarrowPath` | Condition | Returns `SUCCESS` when **both** `/human_detected` and `/narrow_path_detected` topics are `true` simultaneously. Publishes current state to `/human_in_narrow_path_active`. Uses a dedicated callback group and executor thread for real-time topic updates independent of the BT tick rate. |
| `NarrowPath` | Condition | Returns `SUCCESS` when `/narrow_path_detected` is `true`. Publishes current state to `/narrow_path_detected_active` on every tick. Uses a dedicated callback group and executor thread for real-time updates. |
| `SetGoalFromLocation` | Action | Reads a named location (or the closest location) from a YAML file and writes it to a blackboard key as `geometry_msgs/PoseStamped`. Supports `loc="closest"` to select the nearest location based on odometry from `/<namespace>/ekf_odom`. |
| `ResetPath` | Action | Clears a `nav_msgs/Path` blackboard entry to an empty path, forcing `ComputePathToPose` to replan on the next tick. Returns `FAILURE` to trigger pipeline retry. |
| `SetCostmapInflationRadius` | Action | Calls `rcl_interfaces/srv/SetParameters` to change a costmap inflation layer's `inflation_radius` at runtime. Caches the last confirmed value on the BT blackboard (keyed by service name) so all instances targeting the same costmap share one cache and skip redundant service calls. Ports: `service_name`, `inflation_radius`, `param_name` (default: `inflation_layer.inflation_radius`), `server_timeout`. |

---

## Custom Behavior Trees

### HumanInNarrowPath Auto Recovery

**File:** `navigation2/nav2_bt_navigator/behavior_trees/HumanInNarrowPath_auto_recovery.xml`

Human-aware navigation with automatic recovery for narrow corridors:

1. Robot navigates to goal normally.
2. If a human is detected blocking a narrow path → robot navigates to a predefined **wait place**.
3. Robot waits at the wait place, re-checking `HumanInNarrowPath` every tick.
4. Once the human clears → robot replans from the wait place to the original goal and resumes.

See [nav2_bt_navigator/README.md](navigation2/nav2_bt_navigator/README.md) for full documentation.

---

## Setup

- Install dependencies:
  ```bash
  sudo apt install ros-jazzy-libg2o
  ```

- Build all packages:
  ```bash
  colcon build --symlink-install
  ```

- Or build specific packages:
  ```bash
  colcon build --symlink-install --packages-select \
    teb_local_planner costmap_converter_msgs costmap_converter teb_msgs
  ```

- Build Nav2 BT packages (after adding custom plugins):
  ```bash
  colcon build --symlink-install --packages-select \
    nav2_behavior_tree nav2_bt_navigator
  ```

---

## Play

1. Launch Gazebo:
   ```bash
   ros2 launch rgt_nav3_launch gazebo.launch.py
   ```

2. Click "play" in Gazebo.

3. Launch the demo:
   ```bash
   ros2 launch rgt_nav3_launch demo.launch.py \
     use_localization:=True use_sim_time:=True namespace:=apple
   ```

### Testing HumanInNarrowPath

Simulate a human blocking the narrow path (Scenario B — both topics must be `true`):

```bash
# Trigger HumanInNarrowPath condition (both required)
ros2 topic pub -r 1 /human_detected std_msgs/msg/Bool "{data: true}"
ros2 topic pub -r 1 /narrow_path_detected std_msgs/msg/Bool "{data: true}"

# Clear the human → robot replans from wait place to original goal (Scenario A)
ros2 topic pub --once /human_detected std_msgs/msg/Bool "{data: false}"

# Monitor active state
ros2 topic echo /human_in_narrow_path_active
```

### Testing NarrowPath only

```bash
# Trigger narrow path detection
ros2 topic pub -r 1 /narrow_path_detected std_msgs/msg/Bool "{data: true}"

# Clear narrow path
ros2 topic pub --once /narrow_path_detected std_msgs/msg/Bool "{data: false}"

# Monitor active state
ros2 topic echo /narrow_path_detected_active
```


## Update log

|    Version    |      Timestamp      |   Description |
| ------------- |:-------------------:|:-------------:|
| ![version](https://badgen.net/badge/version/1.0.0/blue)      | 2026-03-26 17:26:46 | remove speedcontroller plugin; initial bt_navigator README ; plugin search local goal aside; outline give way in very narrow passage ; update GoalCritic, CostCritic, ObstaclesCritic, planner_server; set always_send_full_costmap: True; bt navigation setup with plugins |
| ![version](https://badgen.net/badge/version/1.2.0/blue)      | 2026-03-30 13:12:51 | update HumanInNarrowPath_auto_recovery |
| ![version](https://badgen.net/badge/version/1.2.0/blue)      | 2026-03-30 13:12:51 | update package bcr_bot |

