# Architecture: navigate_to_pose_w_replanning_and_recovery

Reference file: `behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`

---

## 1. System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS 2 Node                               │
│                    BtNavigator (LifecycleNode)                  │
│                                                                 │
│  ┌────────────────────┐    ┌──────────────────────────────────┐ │
│  │  Plugin Loader     │    │  Navigator Plugins               │ │
│  │  (pluginlib)       │───▶│  ┌──────────────────────────┐   │ │
│  │                    │    │  │ NavigateToPoseNavigator   │   │ │
│  └────────────────────┘    │  │ (action server)           │   │ │
│                            │  └────────────┬─────────────┘   │ │
│  ┌────────────────────┐    │               │                  │ │
│  │  TF2 Buffer        │    │  ┌────────────▼─────────────┐   │ │
│  │  + Listener        │    │  │ NavigateThroughPoses      │   │ │
│  └────────────────────┘    │  │ (action server)           │   │ │
│                            │  └──────────────────────────┘   │ │
│  ┌────────────────────┐    └──────────────────────────────────┘ │
│  │  OdomSmoother      │                                         │
│  │  (0.3s window)     │                                         │
│  └────────────────────┘                                         │
└─────────────────────────────────────────────────────────────────┘
                              │
                    ┌─────────▼──────────┐
                    │  BehaviorTree.CPP  │
                    │  Engine            │
                    │  (Blackboard)      │
                    └─────────┬──────────┘
                              │ ticks BT nodes
          ┌───────────────────┼────────────────────┐
          ▼                   ▼                    ▼
  ┌───────────────┐  ┌────────────────┐  ┌─────────────────┐
  │ nav2_planner  │  │ nav2_controller│  │ nav2_costmap_2d  │
  │ (action srv)  │  │ (action srv)   │  │ (service)        │
  └───────────────┘  └────────────────┘  └─────────────────┘
```

---

## 2. Behavior Tree Structure (XML)

```
RecoveryNode (max 6 retries)
├── [Child 1] PipelineSequence "NavigateWithReplanning"
│   ├── ControllerSelector       ← writes {selected_controller} to blackboard
│   ├── PlannerSelector          ← writes {selected_planner} to blackboard
│   ├── RateController (1 Hz)    ← throttles replanning frequency
│   │   └── RecoveryNode (1 retry) "ComputePathToPose"
│   │       ├── ComputePathToPose   ← reads {goal}, writes {path}
│   │       └── Sequence (costmap recovery)
│   │           ├── WouldAPlannerRecoveryHelp
│   │           └── ClearEntireCostmap (global)
│   └── RecoveryNode (1 retry) "FollowPath"
│       ├── FollowPath              ← reads {path}, writes error codes
│       └── Sequence (costmap recovery)
│           ├── WouldAControllerRecoveryHelp
│           └── ClearEntireCostmap (local)
│
└── [Child 2] Sequence (system recovery, runs when Child 1 FAILS)
    ├── Fallback
    │   ├── WouldAControllerRecoveryHelp
    │   └── WouldAPlannerRecoveryHelp
    └── ReactiveFallback "RecoveryFallback"
        ├── GoalUpdated              ← exits recovery if new goal arrives
        └── RoundRobin "RecoveryActions"  ← cycles through behaviors
            ├── Sequence "ClearingActions"
            │   ├── ClearEntireCostmap (local)
            │   └── ClearEntireCostmap (global)
            ├── Spin (1.57 rad ≈ 90°)
            ├── Wait (3.0 s)
            └── BackUp (0.20 m @ 0.15 m/s)
```

### Node Semantics

| BT Node | Type | Action/Service | Description |
|---|---|---|---|
| `RecoveryNode` | Control | — | Runs child 1; if fails, runs child 2 then retries child 1 |
| `PipelineSequence` | Control | — | Like Sequence but child N re-ticks child N-1 on each tick |
| `RateController` | Decorator | — | Throttles child ticks to configured Hz |
| `ControllerSelector` | Action | `/controller_selector` topic | Sets controller plugin at runtime |
| `PlannerSelector` | Action | `/planner_selector` topic | Sets planner plugin at runtime |
| `ComputePathToPose` | Action | `compute_path_to_pose` (nav2_planner) | Calls global planner |
| `FollowPath` | Action | `follow_path` (nav2_controller) | Calls local controller |
| `ClearEntireCostmap` | Action | `clear_entirely_*` service | Clears specified costmap |
| `WouldAPlannerRecoveryHelp` | Condition | — | Checks error code for planner-fixable error |
| `WouldAControllerRecoveryHelp` | Condition | — | Checks error code for controller-fixable error |
| `GoalUpdated` | Condition | — | Returns SUCCESS if new goal in blackboard |
| `Spin` | Action | `spin` (nav2_behaviors) | In-place rotation |
| `Wait` | Action | `wait` (nav2_behaviors) | Timed pause |
| `BackUp` | Action | `back_up` (nav2_behaviors) | Backward motion |

---

## 3. Data Flow

### Blackboard Variables

```
┌──────────────────────────────────────────────────────────────┐
│                      BT Blackboard                           │
│                                                              │
│  {goal}                  ← NavigateToPoseNavigator sets      │
│                            consumed by ComputePathToPose     │
│                                                              │
│  {path}                  ← ComputePathToPose writes          │
│                            consumed by FollowPath            │
│                            read by onLoop() for feedback     │
│                                                              │
│  {selected_controller}   ← ControllerSelector writes         │
│                            consumed by FollowPath            │
│                                                              │
│  {selected_planner}      ← PlannerSelector writes            │
│                            consumed by ComputePathToPose     │
│                                                              │
│  {compute_path_error_code} ← ComputePathToPose writes        │
│                              read by WouldAPlannerRecovery   │
│                                                              │
│  {follow_path_error_code}  ← FollowPath writes               │
│                              read by WouldAControllerRecovery│
│                                                              │
│  {spin_error_code}         ← Spin writes                     │
│  {backup_code_id}          ← BackUp writes                   │
└──────────────────────────────────────────────────────────────┘
```

### Request/Response Flow

```
Client (e.g. RVIZ /goal_pose or Nav2 stack)
        │
        │  NavigateToPose action goal
        ▼
NavigateToPoseNavigator::goalReceived()
        │
        ├── transform goal → global_frame (TF2)
        ├── write {goal} to blackboard
        ├── reset number_recoveries = 0
        └── load BT XML file
                │
                ▼
        BT Engine tick loop (each ~100ms)
                │
                ├── RateController gates ComputePathToPose → 1 Hz
                │       │
                │       ▼
                │   nav2_planner::ComputePathToPose (action)
                │       └── returns nav_msgs/Path → {path}
                │
                ├── FollowPath reads {path} (every tick)
                │       │
                │       ▼
                │   nav2_controller::FollowPath (action)
                │       └── publishes cmd_vel → robot moves
                │
                └── onLoop() publishes feedback
                        ├── distance_remaining (path length)
                        ├── estimated_time_remaining (dist/speed)
                        ├── number_of_recoveries
                        ├── current_pose (from TF2)
                        └── navigation_time (elapsed)
```

### Recovery Flow

```
FollowPath FAILS (obstacle, stuck, etc.)
        │
        └── RecoveryNode (FollowPath) retries once:
                ├── WouldAControllerRecoveryHelp? → ClearLocalCostmap
                └── Still fails → propagates up to outer RecoveryNode
                        │
                        └── System Recovery Sequence:
                                ├── error classifiable? (controller or planner)
                                └── RoundRobin (cycles each failure):
                                        [1] ClearLocal + ClearGlobal
                                        [2] Spin 90°
                                        [3] Wait 3s
                                        [4] BackUp 20cm
```

---

## 4. File Structure

```
nav2_bt_navigator/
├── CMakeLists.txt
├── package.xml
├── navigator_plugins.xml                          # pluginlib exports
│
├── include/nav2_bt_navigator/
│   ├── bt_navigator.hpp                           # BtNavigator class
│   └── navigators/
│       ├── navigate_to_pose.hpp                   # NavigateToPoseNavigator
│       └── navigate_through_poses.hpp             # NavigateThroughPosesNavigator
│
├── src/
│   ├── main.cpp                                   # rclcpp::spin entry point
│   ├── bt_navigator.cpp                           # LifecycleNode, plugin loading
│   └── navigators/
│       ├── navigate_to_pose.cpp                   # single pose navigator
│       └── navigate_through_poses.cpp             # multi-pose navigator
│
└── behavior_trees/
    ├── navigate_to_pose_w_replanning_and_recovery.xml          ← this file
    ├── navigate_through_poses_w_replanning_and_recovery.xml
    ├── navigate_to_pose_w_replanning_goal_patience_and_recovery.xml
    ├── navigate_w_replanning_distance.xml
    ├── navigate_w_replanning_time.xml
    ├── navigate_w_replanning_speed.xml
    ├── navigate_w_replanning_only_if_goal_is_updated.xml
    ├── navigate_w_replanning_only_if_path_becomes_invalid.xml
    ├── navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml
    ├── nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml
    ├── follow_point.xml
    └── odometry_calibration.xml
```

---

## 5. Libraries and Dependencies

### Runtime Libraries

| Library | Package | Purpose |
|---|---|---|
| `behaviortree_cpp` | `behaviortree_cpp` | BT engine, nodes, blackboard, XML parser |
| `rclcpp` | `rclcpp` | ROS 2 C++ client |
| `rclcpp_action` | `rclcpp_action` | Action server / client |
| `rclcpp_lifecycle` | `rclcpp_lifecycle` | Lifecycle node state machine |
| `pluginlib` | `pluginlib` | Dynamic plugin loading |
| `tf2_ros` | `tf2_ros` | Transform lookup and broadcasting |
| `nav2_behavior_tree` | `nav2_behavior_tree` | All built-in BT node plugins |
| `nav2_core` | `nav2_core` | `NavigatorBase` interface |
| `nav2_util` | `nav2_util` | `LifecycleNode`, `OdomSmoother`, geometry utils |
| `nav2_msgs` | `nav2_msgs` | `NavigateToPose`, `NavigateThroughPoses` actions |
| `nav_msgs` | `nav_msgs` | `nav_msgs/Path` |
| `geometry_msgs` | `geometry_msgs` | `PoseStamped`, `Twist` |

### Built-in BT Node Plugins (from `nav2_behavior_tree`)

The following C++ plugins are loaded at startup from `nav2_behavior_tree::plugins_list.hpp`:

**Action Nodes** (call ROS 2 actions/services):
- `ComputePathToPose` → `nav2_planner::PlannerServer`
- `FollowPath` → `nav2_controller::ControllerServer`
- `ClearEntireCostmap` → `nav2_costmap_2d` service
- `Spin`, `Wait`, `BackUp` → `nav2_behaviors::BehaviorServer`

**Condition Nodes**:
- `GoalUpdated`, `WouldAPlannerRecoveryHelp`, `WouldAControllerRecoveryHelp`

**Control Nodes**:
- `RecoveryNode`, `PipelineSequence`, `RoundRobin`, `ReactiveFallback`

**Decorator Nodes**:
- `RateController`, `ControllerSelector`, `PlannerSelector`

---

## 6. How to Create a Custom Navigator

### Step 1: Create the BT XML file

```xml
<!-- my_behavior_trees/my_navigation.xml -->
<root BTCPP_format="6" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <ControllerSelector selected_controller="{selected_controller}"
                            default_controller="FollowPath"
                            topic_name="controller_selector"/>
        <PlannerSelector selected_planner="{selected_planner}"
                         default_planner="GridBased"
                         topic_name="planner_selector"/>
        <!-- Tune Hz for replanning frequency -->
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}"
                               planner_id="{selected_planner}"
                               error_code_id="{compute_path_error_code}"/>
            <Sequence>
              <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context"
                service_name="global_costmap/clear_entirely_global_costmap"/>
            </Sequence>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="{selected_controller}"
                      error_code_id="{follow_path_error_code}"/>
          <Sequence>
            <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context"
              service_name="local_costmap/clear_entirely_local_costmap"/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      <!-- System-level recovery -->
      <Sequence>
        <Fallback>
          <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
          <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
        </Fallback>
        <ReactiveFallback name="RecoveryFallback">
          <GoalUpdated/>
          <RoundRobin name="RecoveryActions">
            <Sequence name="ClearingActions">
              <ClearEntireCostmap name="ClearLocalCostmap-Subtree"
                service_name="local_costmap/clear_entirely_local_costmap"/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Subtree"
                service_name="global_costmap/clear_entirely_global_costmap"/>
            </Sequence>
            <Spin spin_dist="1.57" error_code_id="{spin_error_code}"/>
            <Wait wait_duration="3.0"/>
            <BackUp backup_dist="0.20" backup_speed="0.15" error_code_id="{backup_code_id}"/>
          </RoundRobin>
        </ReactiveFallback>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### Step 2: Create a Custom BT Action Node (optional)

If you need a new BT node not available in `nav2_behavior_tree`:

```cpp
// include/my_package/bt_nodes/my_action.hpp
#pragma once
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "my_msgs/action/my_action.hpp"

namespace my_package
{

class MyActionNode : public nav2_behavior_tree::BtActionNode<my_msgs::action::MyAction>
{
public:
  MyActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<my_msgs::action::MyAction>(xml_tag_name, action_name, conf)
  {}

  void on_tick() override
  {
    // populate goal_ fields from blackboard
    goal_.some_field = getInput<double>("some_param").value();
  }

  BT::NodeStatus on_success() override
  {
    // write results to blackboard
    setOutput("result_key", result_.result->some_result);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("some_param", "Description"),
      BT::OutputPort<SomeType>("result_key", "Description"),
    });
  }
};

}  // namespace my_package
```

### Step 3: Register the custom node via plugin_lib_names

```yaml
# nav2_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    transform_tolerance: 0.1
    default_nav_to_pose_bt_xml: ""   # empty = use installed default
    # OR specify absolute path:
    # default_nav_to_pose_bt_xml: /path/to/my_navigation.xml
    # OR pass per-goal via action goal's behavior_tree field
    navigators:
      - navigate_to_pose
      - navigate_through_poses
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_rate_controller_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_wait_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_controller_selector_bt_node
      - nav2_planner_selector_bt_node
      - nav2_would_a_planner_recovery_help_condition_bt_node
      - nav2_would_a_controller_recovery_help_condition_bt_node
      - my_package_my_action_bt_node    # your custom node
```

### Step 4: CMakeLists.txt additions

```cmake
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)

# Custom BT node plugin
add_library(my_package_my_action_bt_node SHARED
  src/bt_nodes/my_action.cpp
)
target_include_directories(my_package_my_action_bt_node PUBLIC
  include
)
ament_target_dependencies(my_package_my_action_bt_node
  behaviortree_cpp
  nav2_behavior_tree
  rclcpp
  my_msgs
)
install(TARGETS my_package_my_action_bt_node
  DESTINATION lib
)

# Install BT XML files
install(DIRECTORY my_behavior_trees
  DESTINATION share/${PROJECT_NAME}
)
```

### Step 5: package.xml additions

```xml
<depend>behaviortree_cpp</depend>
<depend>nav2_behavior_tree</depend>
<depend>nav2_core</depend>
<depend>nav2_util</depend>
<depend>nav2_msgs</depend>
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>tf2_ros</depend>
```

---

## 7. Key Design Patterns

### PipelineSequence vs Sequence

```
Sequence:  A → B → C        (stops ticking A once B starts)
PipelineSequence:
  tick 1: A
  tick 2: A, B              (A is re-ticked every time B is ticked)
  tick 3: A, B, C           (all run concurrently each tick)
```
This ensures `ComputePathToPose` and `FollowPath` are running in "parallel"
while `FollowPath` is executing — the path is continuously replanned.

### RecoveryNode Pattern

```
RecoveryNode(N retries):
  ├── Child 1 (main task)
  └── Child 2 (recovery)

  Loop:
    run Child 1
    if FAILURE:
      run Child 2   ← recovery attempt
      retry count++
      if count > N: return FAILURE
      else: try Child 1 again
    if SUCCESS: return SUCCESS
```

### RateController: 1 Hz Replanning

The `RateController` at 1 Hz means:
- `ComputePathToPose` is called at most once per second
- `FollowPath` is ticked every BT tick (~10-100 Hz) and continuously executes
- This prevents expensive global planner calls on every tick

### WouldA*RecoveryHelp: Smart Recovery Gating

Before attempting costmap clearing:
```
WouldAPlannerRecoveryHelp checks if error_code is in:
  [UNKNOWN, NONE, TIMEOUT, FAILED_TO_MAKE_PLAN,
   NO_VALID_CONTROL, PATIENCE_EXCEEDED, ...]

If YES → recovery might help → proceed with ClearEntireCostmap
If NO  → error is not costmap related → skip recovery
```

---

## 8. Common Customization Points

| What to change | Where |
|---|---|
| Replanning frequency | `RateController hz="X"` in XML |
| Max recovery retries | `RecoveryNode number_of_retries="N"` in XML |
| Recovery behaviors | Replace/add nodes in `RoundRobin` in XML |
| Planner/controller plugins | `nav2_params.yaml` → `planner_server` / `controller_server` |
| Default BT file | `default_nav_to_pose_bt_xml` param or action goal's `behavior_tree` field |
| Spin angle | `Spin spin_dist="X"` (radians) |
| Backup distance/speed | `BackUp backup_dist="X" backup_speed="Y"` |
| Wait duration | `Wait wait_duration="X"` |
| Add custom BT node | Implement `BtActionNode<>`, register via `plugin_lib_names` |
