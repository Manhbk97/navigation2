# nav2_behavior_tree

This module is used by the nav2_bt_navigator to implement a ROS2 node that executes navigation Behavior Trees for either navigation or autonomy systems. The nav2_behavior_tree module uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) for the core Behavior Tree processing.

The nav2_behavior_tree module provides:
* A C++ template class for easily integrating ROS2 actions and services into Behavior Trees,
* Navigation-specific behavior tree nodes, and
* a generic BehaviorTreeEngine class that simplifies the integration of BT processing into ROS2 nodes for navigation or higher-level autonomy applications.

See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-bt-xml.html) for additional parameter descriptions and a list of XML nodes made available in this package. Also review the [Nav2 Behavior Tree Explanation](https://docs.nav2.org/behavior_trees/index.html) pages explaining more context on the default behavior trees and examples provided in this package. A [tutorial](https://docs.nav2.org/plugin_tutorials/docs/writing_new_bt_plugin.html) is also provided to explain how to create a simple BT plugin.

See the [Navigation Plugin list](https://docs.nav2.org/plugins/index.html) for a list of the currently known and available planner plugins. 



## Plugin Reference

All plugins are registered via `nav2_tree_nodes.xml` and implemented under `nav2_behavior_tree/plugins/` and `nav2_bt_navigator/plugins/`. They fall into four categories: **Action**, **Condition**, **Control**, and **Decorator**.

---

### Action Nodes

Action nodes call an action server or service and return `SUCCESS`, `FAILURE`, or `RUNNING`.

| Plugin ID | Source | Description |
|-----------|--------|-------------|
| `BackUp` | nav2_behavior_tree | Commands the robot to reverse a specified distance at a given speed within a time allowance. |
| `DriveOnHeading` | nav2_behavior_tree | Drives the robot forward along its current heading for a set distance at a given speed. |
| `CancelControl` | nav2_behavior_tree | Sends a cancellation request to the running controller server action. |
| `CancelBackUp` | nav2_behavior_tree | Cancels an in-progress `BackUp` behavior. |
| `CancelDriveOnHeading` | nav2_behavior_tree | Cancels an in-progress `DriveOnHeading` behavior. |
| `CancelSpin` | nav2_behavior_tree | Cancels an in-progress `Spin` behavior. |
| `CancelAssistedTeleop` | nav2_behavior_tree | Cancels an in-progress `AssistedTeleop` behavior. |
| `CancelWait` | nav2_behavior_tree | Cancels an in-progress `Wait` behavior. |
| `ClearEntireCostmap` | nav2_behavior_tree | Clears **all** obstacles from an entire costmap via service call. |
| `ClearCostmapExceptRegion` | nav2_behavior_tree | Clears costmap obstacles that are **outside** a given radius around the robot (keeps the nearby region intact). |
| `ClearCostmapAroundRobot` | nav2_behavior_tree | Clears costmap obstacles **within** a given radius around the robot. |
| `ClearCostmapAroundPose` | nav2_behavior_tree | Clears costmap obstacles within a given radius around a **specific pose** (not the robot). |
| `ComputePathToPose` | nav2_behavior_tree | Calls the planner server to compute a global path to a single goal pose; outputs the path to the blackboard. |
| `ComputePathThroughPoses` | nav2_behavior_tree | Calls the planner server to compute a global path that passes through an ordered list of waypoints. |
| `ComputeRoute` | nav2_behavior_tree | Calls the route server to compute a high-level route between graph node IDs or poses; outputs the route and a corresponding path. Supports specifying start/goal by ID or by pose. |
| `ComputeAndTrackRoute` | nav2_behavior_tree | Calls the route server to compute **and continuously track** a route with live feedback (current edge, next node). Outputs rerouting status and execution duration. Remains `RUNNING` until the goal is reached or an error occurs. |
| `CancelComputeAndTrackRoute` | nav2_behavior_tree | Cancels an in-progress `ComputeAndTrackRoute` action. |
| `RemovePassedGoals` | nav2_behavior_tree | Filters an input list of goal poses and removes any that the robot has already passed within a given radius tolerance. |
| `SmoothPath` | nav2_behavior_tree | Calls the smoother server to smooth a raw planned path; outputs the smoothed path and whether smoothing completed before the time limit. |
| `FollowPath` | nav2_behavior_tree | Commands the controller server to follow a given path using a specified controller and goal/progress checkers. |
| `NavigateToPose` | nav2_behavior_tree | Top-level action: navigate to a single goal pose. Can invoke a sub-behavior tree. |
| `NavigateThroughPoses` | nav2_behavior_tree | Top-level action: navigate through an ordered list of poses. Can invoke a sub-behavior tree. |
| `ReinitializeGlobalLocalization` | nav2_behavior_tree | Triggers global re-localization (e.g. spreads AMCL particles globally) via service. |
| `TruncatePath` | nav2_behavior_tree | Shortens a path by removing poses within a fixed distance of the goal end. |
| `TruncatePathLocal` | nav2_behavior_tree | Extracts a local window of a path around the robot's current position using forward and backward distance limits. Supports weighted angular distance for curved paths. |
| `PlannerSelector` | nav2_behavior_tree | Subscribes to a topic and writes the selected planner plugin name to the blackboard, allowing runtime planner switching. |
| `ControllerSelector` | nav2_behavior_tree | Subscribes to a topic and writes the selected controller plugin name to the blackboard, allowing runtime controller switching. |
| `SmootherSelector` | nav2_behavior_tree | Subscribes to a topic and writes the selected smoother plugin name to the blackboard. |
| `GoalCheckerSelector` | nav2_behavior_tree | Subscribes to a topic and writes the selected goal checker plugin name to the blackboard. |
| `ProgressCheckerSelector` | nav2_behavior_tree | Subscribes to a topic and writes the selected progress checker plugin name to the blackboard. |
| `Spin` | nav2_behavior_tree | Commands the robot to rotate in place by a specified angular distance within a time allowance. |
| `Wait` | nav2_behavior_tree | Pauses BT execution for a specified duration via the wait behavior server. |
| `AssistedTeleop` | nav2_behavior_tree | Activates assisted teleoperation mode: the operator drives, but the behavior server applies safety velocity overrides to prevent collisions. |
| `ConcatenatePaths` | nav2_behavior_tree | Joins two input paths (`input_path1` + `input_path2`) into a single output path. Synchronous, no server call needed. |
| `GetCurrentPose` | nav2_behavior_tree | Looks up the robot's current pose from TF and writes it as a `PoseStamped` to the blackboard. |
| `GetPoseFromPath` | nav2_behavior_tree | Extracts a single pose from a path by index (supports negative index `-1` for the last pose) and writes it to the blackboard. |
| `SetGoalFromLocation` | nav2_bt_navigator | Reads a named location from a YAML file (or selects the closest location) and writes its pose to the blackboard as the navigation goal. |
| `ResetPath` | nav2_bt_navigator | Clears the current path entry on the blackboard (sets it to an empty path). |
| `SetCostmapInflationRadius` | nav2_bt_navigator | Dynamically updates the `inflation_radius` parameter of a costmap node at runtime via the `set_parameters` ROS 2 service. |
| `SearchLocalGoalAside` | nav2_bt_navigator | Searches the local costmap for a free pose to the **right** of the robot's heading at a configurable distance and angle. Sweeps ±`search_sweep_deg` if the primary angle is occupied. Returns `FAILURE` if no free cell is found. |

---

### Condition Nodes

Condition nodes check a runtime state and return `SUCCESS` if the condition holds or `FAILURE` otherwise.

| Plugin ID | Source | Description |
|-----------|--------|-------------|
| `GoalReached` | nav2_behavior_tree | Returns `SUCCESS` when the robot's current pose is within tolerance of the specified goal pose. |
| `IsStuck` | nav2_behavior_tree | Returns `SUCCESS` when the robot appears to be stuck (velocity commands are sent but no real motion is detected). |
| `TransformAvailable` | nav2_behavior_tree | Returns `SUCCESS` if the TF transform between the specified parent and child frames is currently available. |
| `GoalUpdated` | nav2_behavior_tree | Returns `SUCCESS` if the goal on the blackboard has changed since the last tick (triggers reactive re-planning). |
| `GlobalUpdatedGoal` | nav2_behavior_tree | Similar to `GoalUpdated` but tracks changes to the globally-stored navigation goal independently of per-tick blackboard state. |
| `IsBatteryLow` | nav2_behavior_tree | Returns `SUCCESS` when the battery level (by percentage or voltage) drops below a configured minimum threshold. |
| `IsBatteryCharging` | nav2_behavior_tree | Returns `SUCCESS` if the battery is currently in a charging state. |
| `DistanceTraveled` | nav2_behavior_tree | Returns `SUCCESS` once the robot has traveled at least a specified distance from the position recorded at the last trigger. |
| `TimeExpired` | nav2_behavior_tree | Returns `SUCCESS` once a specified number of seconds has elapsed since the timer was last reset. |
| `PathExpiringTimer` | nav2_behavior_tree | Returns `SUCCESS` when a timer expires; the timer resets automatically whenever the path on the blackboard is updated. |
| `InitialPoseReceived` | nav2_behavior_tree | Returns `SUCCESS` if an initial pose estimate has been received (used to gate navigation startup until localization is ready). |
| `IsPathValid` | nav2_behavior_tree | Returns `SUCCESS` if the current path remains collision-free and valid according to the costmap. |
| `WouldAControllerRecoveryHelp` | nav2_behavior_tree | Returns `SUCCESS` if the controller's error code is one that a recovery behavior is likely to resolve. |
| `WouldAPlannerRecoveryHelp` | nav2_behavior_tree | Returns `SUCCESS` if the planner's error code is one that a recovery behavior is likely to resolve. |
| `WouldASmootherRecoveryHelp` | nav2_behavior_tree | Returns `SUCCESS` if the smoother's error code is one that a recovery behavior is likely to resolve. |
| `AreErrorCodesPresent` | nav2_behavior_tree | Returns `SUCCESS` if the current error code matches any code in a user-supplied list of error codes to watch for. |
| `ArePosesNear` | nav2_behavior_tree | Returns `SUCCESS` if two given poses (`ref_pose` and `target_pose`) are within a specified Euclidean distance tolerance of each other. |
| `HumanInNarrowPath` | nav2_bt_navigator | Returns `SUCCESS` if a human is detected occupying the narrow passage ahead (used for social navigation behaviors). |
| `NarrowPath` | nav2_bt_navigator | Returns `SUCCESS` if the current planned path passes through a narrow corridor, enabling the tree to switch to a narrow-path behavior strategy. |

---

### Control Nodes

Control nodes manage the **execution order** of their children.

| Plugin ID | Source | Description |
|-----------|--------|-------------|
| `PipelineSequence` | nav2_behavior_tree | A reactive `Sequence` variant that re-ticks all previously `RUNNING` children on each cycle, useful for pipelines where earlier steps must remain active while later steps are running. |
| `RecoveryNode` | nav2_behavior_tree | Runs child 1 (the main task); if it fails, runs child 2 (the recovery action). Repeats this cycle up to `number_of_retries` times before returning `FAILURE`. |
| `RoundRobin` | nav2_behavior_tree | Ticks children in a circular round-robin order, advancing to the next child when the current one succeeds. Returns `FAILURE` only after all children have failed. |

---

### Decorator Nodes

Decorator nodes wrap a **single child** and modify when or how often it is ticked.

| Plugin ID | Source | Description |
|-----------|--------|-------------|
| `RateController` | nav2_behavior_tree | Throttles child ticks to a maximum frequency (`hz`). The child is only ticked once per period; between ticks the last result is returned. |
| `DistanceController` | nav2_behavior_tree | Ticks its child only after the robot has moved at least `distance` meters since the last tick. Useful for distance-based replanning. |
| `SingleTrigger` | nav2_behavior_tree | Ticks its child **exactly once**. All subsequent ticks return the original result without re-running the child. |
| `KeepRunningUntilFailure` | nav2_behavior_tree | Always returns `RUNNING` regardless of the child's result, unless the child returns `FAILURE`, which is propagated upward. |
| `GoalUpdater` | nav2_behavior_tree | Subscribes to a goal topic and updates the goal on the blackboard before each tick of its child, enabling dynamic goal injection. |
| `SpeedController` | nav2_behavior_tree | Dynamically scales the child's tick rate between `min_rate` and `max_rate` based on the robot's current speed relative to `min_speed`/`max_speed`. |
| `PathLongerOnApproach` | nav2_behavior_tree | Only ticks its child when the computed path is significantly longer (by `length_factor`) than expected within `prox_len` meters of the goal — a signal that an obstacle is blocking the direct approach. |
| `GoalUpdatedController` | nav2_behavior_tree | Acts as a gate: ticks its child only when the goal on the blackboard has been updated, preventing redundant re-planning. |

---

### Plugin Count Summary

| Category | Count |
|----------|-------|
| Action nodes | 40 |
| Condition nodes | 19 |
| Control nodes | 3 |
| Decorator nodes | 8 |
| **Total** | **70** |


### Install

```bash
colcon build --packages-select nav2_behavior_tree \
  --cmake-args \
    -DCMAKE_BUILD_PARALLEL_LEVEL=1 \
    -DCMAKE_CXX_FLAGS="-O1"   # lower optimization = less RAM usage
    --allow-overriding 
  
```