# Nav2 Controller



The Nav2 Controller is a Task Server in Nav2 that implements the `nav2_msgs::action::FollowPath` action server.

An execution module implementing the `nav2_msgs::action::FollowPath` action server is responsible for generating command velocities for the robot, given the computed path from the planner module in `nav2_planner`. The nav2_controller package is designed to be loaded with multiple plugins for path execution. The plugins need to implement functions in the virtual base class defined in the `controller` header file in `nav2_core` package. It also contains progress checkers and goal checker plugins to abstract out that logic from specific controller implementations.

See the [Navigation Plugin list](https://docs.nav2.org/plugins/index.html) for a list of the currently known and available controller plugins. 

See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-controller-server.html) for additional parameter descriptions and a [tutorial about writing controller plugins](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html).

The `ControllerServer` makes use of a [nav2_util::TwistPublisher](../nav2_util/README.md#twist-publisher-and-twist-subscriber-for-commanded-velocities).


1. Includes & Namespace (lines 1-35)
Standard headers, ROS 2 messages, Nav2 utilities, and the main header.

2. Constructor & Destructor (lines 37-82)

ControllerServer() → Declares parameters, initializes plugin loaders, creates costmap_ros_
~ControllerServer() → Clears plugin containers and costmap thread
3. Lifecycle Callbacks (lines 84-362)
Method	Lines	Purpose
on_configure	84-257	Loads all plugins (progress checkers, goal checkers, controllers), creates action server, sets up subscribers
on_activate	259-284	Activates costmap, controllers, publishers, creates bond
on_deactivate	286-327	Deactivates components, publishes zero velocity, destroys bond
on_cleanup	329-355	Cleans up controllers and releases resources
on_shutdown	357-362	Logs shutdown message
4. Plugin Lookup Helpers (lines 364-440)
findControllerId() - Resolves controller plugin by name
findGoalCheckerId() - Resolves goal checker plugin by name
findProgressCheckerId() - Resolves progress checker plugin by name
5. Main Control Loop (lines 442-601)
computeControl() - The action server callback:

Validates goal and resolves plugins
Sets path and resets progress checker
Control loop: waits for costmap → updates path → computes velocity → checks goal
Exception handling for various failure modes (TF errors, no valid control, timeout, etc.)
6. Helper Methods (lines 603-859)
Method	Lines	Purpose
setPlannerPath	603-622	Passes path to controller, stores end pose
computeAndPublishVelocity	624-698	Gets pose, checks progress, computes cmd_vel, publishes feedback
updateGlobalPath	700-743	Handles preemption with new path/plugins
publishVelocity	745-751	Publishes TwistStamped to cmd_vel
publishZeroVelocity	753-773	Stops robot and resets controllers
isGoalReached	775-795	Checks if goal checker reports success
getRobotPose	797-805	Gets robot pose from costmap
speedLimitCallback	807-813	Applies speed limits to all controllers
dynamicParametersCallback	815-859	Handles runtime parameter changes
7. Component Registration (lines 863-868)

RCLCPP_COMPONENTS_REGISTER_NODE(nav2_controller::ControllerServer)
Registers as a composable node for component containers.

Data Flow Summary

FollowPath Action Goal
        ↓
   computeControl()
        ↓
   ┌────────────────┐
   │  Control Loop  │ ← controller_frequency_ Hz
   │                │
   │ 1. Wait costmap│
   │ 2. Update path │
   │ 3. Compute vel │ → Controller plugin
   │ 4. Publish vel │ → cmd_vel topic
   │ 5. Check goal  │ → GoalChecker plugin
   └────────────────┘
        ↓
   Goal Reached / Error


### architecture 
# nav2_controller Architecture & Data Flow

## Context
This document explains the architecture of the `nav2_controller` package in ROS2 Navigation2 (Nav2). It is the **tracking controller layer** — sits between the global planner (which outputs a path) and the robot's motor drivers (which accept velocity commands). Its job: follow a given path safely and efficiently.

---

## 1. System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        nav2_controller                               │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │              ControllerServer (LifecycleNode)                │   │
│  │                                                             │   │
│  │   ┌──────────────────┐  ┌──────────────────┐               │   │
│  │   │  ActionServer    │  │  ProgressChecker  │               │   │
│  │   │  (FollowPath)    │  │  Plugin           │               │   │
│  │   └──────────────────┘  └──────────────────┘               │   │
│  │                                                             │   │
│  │   ┌──────────────────┐  ┌──────────────────┐               │   │
│  │   │  Controller       │  │  GoalChecker     │               │   │
│  │   │  Plugin           │  │  Plugin          │               │   │
│  │   └──────────────────┘  └──────────────────┘               │   │
│  │                                                             │   │
│  │   ┌──────────────────┐  ┌──────────────────┐               │   │
│  │   │  Costmap2DROS    │  │  OdomSubscriber  │               │   │
│  │   │  (local costmap) │  │  (current vel)   │               │   │
│  │   └──────────────────┘  └──────────────────┘               │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. Node Lifecycle Transitions

```
  [Unconfigured]
       │
       │ configure()
       ▼
  [Inactive] ◄──────────────────────────────────────┐
       │                                             │
       │ activate()          deactivate()            │
       ▼                         │                   │
  [Active] ─────────────────────┘                   │
       │                                             │
       │ cleanup()                                   │
       ▼                                             │
  [Unconfigured] ──────────────────────────────────-┘

on_configure():
  1. Load & init ProgressChecker plugins
  2. Load & init GoalChecker plugins
  3. Load & init Controller plugins (configure with costmap + tf)
  4. Configure Costmap2DROS, launch in separate thread
  5. Create OdomSubscriber, TwistPublisher
  6. Create FollowPath ActionServer
  7. Subscribe to /speed_limit

on_activate():
  1. Activate costmap, controllers, vel_publisher
  2. Activate action_server (begins accepting goals)
  3. Register dynamic params callback

on_deactivate():
  1. Deactivate action_server (stops accepting)
  2. Deactivate controllers, costmap, vel_publisher
  3. Publish ZERO velocity (safe stop)
```

---

## 3. End-to-End Data Flow

```
┌─────────────────────────┐
│  nav2_bt_navigator       │   (or any node sending goals)
│  (Behavior Tree)         │
└────────────┬────────────┘
             │  FollowPath Goal
             │  ├── path (nav_msgs/Path)
             │  ├── controller_id (string)
             │  ├── goal_checker_id (string)
             │  └── progress_checker_id (string)
             │
             ▼ /follow_path (Action)
┌─────────────────────────────────────────────────────────────────┐
│  ControllerServer::computeControl()                              │
│                                                                 │
│  1. findControllerId()      ─── validate plugin exists          │
│  2. findGoalCheckerId()     ─── validate plugin exists          │
│  3. findProgressCheckerId() ─── validate plugin exists          │
│                                                                 │
│  4. setPlannerPath(path)                                        │
│     ├── controller->setPlan(path)                               │
│     ├── goal_checker->reset()                                   │
│     └── end_pose_ = path.poses.back()                           │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  CONTROL LOOP  (@ controller_frequency_ Hz, default 20) │   │
│  │                                                         │   │
│  │  ┌──────────────────────────────────────────────────┐   │   │
│  │  │  computeAndPublishVelocity()                     │   │   │
│  │  │                                                  │   │   │
│  │  │  a) getRobotPose()  ◄── Costmap2DROS TF lookup  │   │   │
│  │  │                                                  │   │   │
│  │  │  b) progress_checker->check(pose)               │   │   │
│  │  │     └── Did robot move enough? (else abort)     │   │   │
│  │  │                                                  │   │   │
│  │  │  c) odom_sub_->getTwist()  ◄── /odom topic      │   │   │
│  │  │                                                  │   │   │
│  │  │  d) controller->computeVelocityCommands(         │   │   │
│  │  │       pose, velocity, goal_checker)              │   │   │
│  │  │     └── Returns TwistStamped                     │   │   │
│  │  │                                                  │   │   │
│  │  │  e) publishVelocity(cmd)  ──► /cmd_vel topic     │   │   │
│  │  │                                                  │   │   │
│  │  │  f) publish_feedback(speed, distance_to_goal)    │   │   │
│  │  └──────────────────────────────────────────────────┘   │   │
│  │                                                         │   │
│  │  isGoalReached()?                                       │   │
│  │  └── goal_checker->isGoalReached(pose, end_pose, vel)   │   │
│  │      ── YES ──► break loop                              │   │
│  │                                                         │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  publishZeroVelocity()  ──► /cmd_vel = {0,0,0}                 │
│  action_server->succeeded_current()                             │
└─────────────────────────────────────────────────────────────────┘
             │
             ▼  FollowPath Result
┌─────────────────────────┐
│  nav2_bt_navigator       │
└─────────────────────────┘
```

---

## 4. Plugin Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                      Plugin System                             │
│                                                               │
│  nav2_core::Controller  (abstract base)                       │
│  ├── configure(node, name, tf, costmap)                       │
│  ├── setPlan(path)                                            │
│  ├── computeVelocityCommands(pose, vel, goal_checker) ──────► TwistStamped
│  ├── setSpeedLimit(limit, percentage)                         │
│  └── cancel()                                                 │
│      [loaded via: DWBLocalPlanner, RPPController, etc.]       │
│                                                               │
│  nav2_core::GoalChecker  (abstract base)                      │
│  ├── SimpleGoalChecker    ── checks XY + yaw within tolerance │
│  ├── StoppedGoalChecker   ── XY + yaw + must be stopped       │
│  └── PositionGoalChecker  ── checks XY only (ignores yaw)     │
│                                                               │
│  nav2_core::ProgressChecker  (abstract base)                  │
│  ├── SimpleProgressChecker ── moved > radius within time?     │
│  └── PoseProgressChecker   ── moved > radius OR angle?        │
│                                                               │
│  pluginlib::ClassLoader loads all plugins by type string      │
│  Multiple instances can be loaded simultaneously              │
│  Selection per-goal via controller_id / goal_checker_id       │
└───────────────────────────────────────────────────────────────┘
```

---

## 5. Error Handling & Safety

```
During control loop, exceptions mapped to error codes:

  InvalidController     ──► error_code::INVALID_CONTROLLER
  ControllerTFError     ──► error_code::TF_ERROR
  NoValidControl        ──► error_code::NO_VALID_CONTROL
                             (with failure_tolerance patience window)
  FailedToMakeProgress  ──► error_code::FAILED_TO_MAKE_PROGRESS
  InvalidPath           ──► error_code::INVALID_PATH
  ControllerTimedOut    ──► error_code::CONTROLLER_TIMED_OUT
  PatienceExceeded      ──► error_code::PATIENCE_EXCEEDED
  ControllerException   ──► error_code::UNKNOWN

  ANY exception ──► publishZeroVelocity() first (safe stop)
                ──► then set action result with error code
```

---

## 6. External Interfaces Summary

```
INPUTS:
  /follow_path          (Action)      ← Goal: path + plugin selections
  /odom                 (Topic, Sub)  ← Robot current velocity
  /speed_limit          (Topic, Sub)  ← Dynamic speed cap
  /tf, /tf_static       (TF frames)   ← Robot pose transforms

OUTPUTS:
  /cmd_vel              (Topic, Pub)  → Velocity to robot base
  /follow_path feedback (Action)      → speed, distance_to_goal
  /follow_path result   (Action)      → success or error_code
  /local_costmap/*      (Costmap)     → Visualization/debugging
```

---
## Setup multi controllers

Step 1: Select one of controller_plugin  

ros2 topic pub --once /apple/controller_selector std_msgs/msg/String   "{data: 'RPP'}"   --qos-durability transient_local

ros2 topic pub --once /apple/controller_selector std_msgs/msg/String   "{data: 'MPPI'}"   --qos-durability transient_local

Step 2: navigate to the goal by CLI aor Rviz 

ros2 action send_goal /apple/navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {position: {x: 7.5, y: 11.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  }
}"





## 7. Key Files

| File | Role |
|------|------|
| [src/controller_server.cpp](src/controller_server.cpp) | Core logic: lifecycle, action server, control loop |
| [include/nav2_controller/controller_server.hpp](include/nav2_controller/controller_server.hpp) | Class definition |
| [plugins/simple_goal_checker.cpp](plugins/simple_goal_checker.cpp) | XY+yaw goal checking |
| [plugins/stopped_goal_checker.cpp](plugins/stopped_goal_checker.cpp) | Goal reached + velocity stopped |
| [plugins/position_goal_checker.cpp](plugins/position_goal_checker.cpp) | XY-only goal checking |
| [plugins/simple_progress_checker.cpp](plugins/simple_progress_checker.cpp) | Linear progress monitoring |
| [plugins/pose_progress_checker.cpp](plugins/pose_progress_checker.cpp) | Linear + angular progress |
| [plugins.xml](plugins.xml) | Plugin type registration |
