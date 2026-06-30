# Tutorial: Navigation 2 + Behavior Trees with TurtleBot3

This tutorial walks you through `nav2_demo_turtlebot3` end to end: what each package
does, how to build and run the demo, how the pieces fit together, and how to extend it
with your own waypoints and behaviors.

The demo makes a simulated TurtleBot3 **patrol a sequence of named locations** in a
Gazebo world using the Nav2 stack, with the patrol logic expressed as a
[BehaviorTree.CPP](https://www.behaviortree.dev/) behavior tree.

> Target platform: **ROS 2 Humble** + Gazebo Classic. (A Galactic variant lives on a
> separate branch upstream.)

---

## 1. Repository layout

```
nav2_demo_turtlebot3/
├── turtlebot3.repos          # vcs manifest: pulls in upstream TurtleBot3 packages
├── tb3_sim/                  # Python package: simulation + Nav2 bring-up
│   ├── launch/
│   │   ├── turtlebot3_world.launch.py   # Gazebo world + robot
│   │   └── nav2.launch.py               # Nav2 bring-up + RViz + AMCL init pose
│   ├── tb3_sim/
│   │   └── set_init_amcl_pose.py        # node: publishes /initialpose for AMCL
│   ├── config/sim_house_locations.yaml  # named waypoints: name -> [x, y, theta]
│   └── maps/map.yaml, map.pgm           # occupancy grid used by Nav2
└── tb3_autonomy/             # C++ package: the patrol autonomy
    ├── bt_xml/tree.xml                  # the behavior tree (a Sequence of GoToPose)
    ├── include/ , src/
    │   ├── autonomy_node.*              # standalone node that ticks the BT
    │   ├── navigation_behaviors.*       # GoToPose BT action (used by autonomy_node)
    │   └── go_to_pose_bt_node.*         # GoToPose as a Nav2 BT plugin (alternative)
    ├── go_to_pose_bt_node_plugin.xml    # pluginlib manifest for the Nav2 plugin
    └── launch/autonomy.launch.py        # runs autonomy_node with the location file
```

Two ROS 2 package styles are demonstrated on purpose:

- **`tb3_sim`** — an `ament_python` package (launch files, a Python node, config/maps).
- **`tb3_autonomy`** — an `ament_cmake` package (a C++ executable and a C++ plugin
  library).

---

## 2. Prerequisites

1. ROS 2 Humble installed and sourced (`source /opt/ros/humble/setup.bash`).
2. The Nav2 stack: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`.
3. Gazebo Classic + `gazebo_ros`: `sudo apt install ros-humble-gazebo-ros-pkgs`.
4. `vcstool`: `sudo apt install python3-vcstool`.
5. The Python deps used by the AMCL-init node: `pip install transforms3d`
   (or `sudo apt install python3-transforms3d`).
6. You can run a basic TurtleBot3 Gazebo simulation. If not, work through a
   [TurtleBot3 ROS 2 getting-started tutorial](https://medium.com/@thehummingbird/ros-2-mobile-robotics-series-part-1-8b9d1b74216)
   first.

---

## 3. Workspace setup

```bash
# 1. Create a workspace and clone this repo into src/
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone <this-repo-url> nav2_demo_turtlebot3

# 2. Pull in the upstream TurtleBot3 packages listed in turtlebot3.repos
cd ~/turtlebot3_ws/src
vcs import . < nav2_demo_turtlebot3/turtlebot3.repos

# 3. Resolve dependencies
cd ~/turtlebot3_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --symlink-install
# (A few stderr warnings during the build are expected — they are not errors.)

# 5. Source the overlay
source install/setup.bash
```

Export the TurtleBot3 model and the Gazebo model path (add these to `~/.bashrc` so you
don't have to repeat them):

```bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
```

---

## 4. Running the demo

Use **three terminals**. Source `~/turtlebot3_ws/install/setup.bash` in each one.

### Terminal 1 — simulation

```bash
ros2 launch tb3_sim turtlebot3_world.launch.py
```

This launches `gzserver` + `gzclient` with the `turtlebot3_world.world`, the
`robot_state_publisher`, and spawns the robot at `x=-2.0, y=-0.5`.

### Terminal 2 — Nav2 + RViz

```bash
ros2 launch tb3_sim nav2.launch.py
```

This brings up:

- the Nav2 stack via `nav2_bringup/bringup_launch.py` with `map.yaml` and
  `use_sim_time:=true`, `autostart:=true`;
- a `bt_navigator` node configured with the custom `GoToPose` BT plugin
  (`plugin_lib_names: ['nav2_go_to_pose_bt_node']`) and the `location_file` parameter;
- `amcl_init_pose_publisher` (the Python node) which publishes the robot's starting
  pose to `/initialpose` so AMCL converges immediately — no manual "2D Pose Estimate"
  click needed;
- RViz with the Nav2 default view.

Wait until RViz shows the map, the robot's laser scan, and a stable localization.

### Terminal 3 — autonomy (the patrol)

```bash
ros2 launch tb3_autonomy autonomy.launch.py
```

This starts `autonomy_node`, which loads `bt_xml/tree.xml`, builds the behavior tree,
and ticks it every 500 ms. The robot visits each location in the tree's `Sequence` in
order. You'll see log lines like `Sent Goal to Nav2 (x=..., y=..., theta=...)` and
`Goal reached`, and finally `Finished Navigation` when the whole sequence succeeds.

---

## 5. How it works

### 5.1 The behavior tree (`tb3_autonomy/bt_xml/tree.xml`)

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="sequence">
      <GoToPose name="go_to_location1" loc="location1" />
      <GoToPose name="go_to_location2" loc="location2" />
      <!-- <GoToPose name="go_to_location3" loc="location3" /> -->
      <!-- <GoToPose name="go_to_location4" loc="location4" /> -->
    </Sequence>
  </BehaviorTree>
</root>
```

- `Sequence` runs its children left to right; it moves on only when a child returns
  `SUCCESS`. If any child returns `FAILURE`, the whole sequence fails.
- `GoToPose` is a **custom action node** (registered in C++). Its `loc` input port is a
  key into the locations YAML file.
- Want a 4-location patrol? Uncomment the extra `GoToPose` lines and add matching keys
  to the YAML (see §6).

### 5.2 `GoToPose` — the custom BT action

The repo contains **two implementations** of `GoToPose`, used by two different runtimes:

| File | Used by | BT.CPP version | Base class | Action name |
|------|---------|----------------|------------|-------------|
| `src/navigation_behaviors.cpp` | `autonomy_node` (standalone) | v3 (`behaviortree_cpp_v3`) | `BT::StatefulActionNode` | `/apple/navigate_to_pose` |
| `src/go_to_pose_bt_node.cpp` | `nav2_bt_navigator` (as a plugin) | v4 (`behaviortree_cpp`) | `nav2_behavior_tree::BtActionNode<NavigateToPose>` | `navigate_to_pose` |

Both do the same thing:

1. Read the `loc` input port (e.g. `"location1"`).
2. Read the `location_file` ROS parameter (a path to the YAML).
3. Look up `[x, y, theta]` for that key.
4. Build a `nav2_msgs/action/NavigateToPose` goal in the `map` frame, converting
   `theta` (yaw) to a quaternion via `tf2::Quaternion::setRPY(0, 0, theta)`.
5. Send the goal to the Nav2 action server and report `RUNNING` until Nav2 returns
   `SUCCEEDED` / `ABORTED` / `CANCELED`.

The standalone version (`navigation_behaviors.cpp`) is a clean, minimal example of
writing an async BT action that wraps a ROS 2 action client: `onStart()` sends the
goal and returns `RUNNING`; `onRunning()` polls a `done_flag_` set by the result
callback; `goal_response_callback` / `nav_to_pose_callback` are the action-client
callbacks.

> ⚠️ **Note on the action name `/apple/navigate_to_pose`.** `navigation_behaviors.cpp`
> connects to `/apple/navigate_to_pose`, but a default Nav2 bring-up exposes
> `/navigate_to_pose`. If `autonomy_node` logs *"Action server /apple/navigate_to_pose
> not available!"*, either (a) change the string in `navigation_behaviors.cpp` to
> `/navigate_to_pose` and rebuild, or (b) launch Nav2 under the `apple` namespace /
> remap the action. This is the most common "the robot doesn't move" gotcha.

### 5.3 `autonomy_node` (`tb3_autonomy/src/autonomy_node.cpp`)

A `rclcpp::Node` that:

- declares the `location_file` parameter (set by `autonomy.launch.py` to
  `tb3_sim/config/sim_house_locations.yaml`);
- in `create_behavior_tree()`, registers a `NodeBuilder` for `GoToPose` with the
  `BT::BehaviorTreeFactory`, then builds the tree from
  `<install>/share/tb3_autonomy/bt_xml/tree.xml`;
- starts a 500 ms wall timer that calls `tree_.tickRoot()`;
- on `SUCCESS` logs *"Finished Navigation"*; on `FAILURE` logs *"Navigation Failed"*
  and cancels the timer.

### 5.4 The Nav2 BT plugin path (`go_to_pose_bt_node.cpp`)

This is the "production-style" alternative: instead of running a separate node, the
`GoToPose` action is compiled into a shared library (`nav2_go_to_pose_bt_node`),
exported via pluginlib (`go_to_pose_bt_node_plugin.xml` →
`pluginlib_export_plugin_description_file(nav2_behavior_tree ...)` in `CMakeLists.txt`),
and loaded by `nav2_bt_navigator` because `nav2.launch.py` passes
`plugin_lib_names: ['nav2_go_to_pose_bt_node']` and `location_file` to the
`bt_navigator` node. You could then reference `<GoToPose loc="..."/>` directly inside a
Nav2 behavior-tree XML.

### 5.5 `set_init_amcl_pose.py` (`tb3_sim`)

A small Python node, registered as the `amcl_init_pose_publisher` console script. It
declares `x`, `y`, `theta`, `cov` parameters, waits for a subscriber on `/initialpose`,
then publishes one `PoseWithCovarianceStamped` (using `transforms3d` for the
euler→quaternion conversion). `nav2.launch.py` runs it with `x: -2.0, y: -0.5` — the
same spot the robot is spawned — so AMCL localizes immediately.

### 5.6 Data flow at a glance

```
sim_house_locations.yaml ──► GoToPose (BT action) ──► nav2_msgs/NavigateToPose ──► Nav2
   (name → x,y,theta)              ▲                       (plans + drives the robot)
                                   │ ticked every 500 ms
                            ┌──────┴──────┐
                            │  tree.xml   │  Sequence: location1 → location2 → ...
                            └─────────────┘
```

---

## 6. Customizing the demo

### Change or add waypoints

Edit `tb3_sim/config/sim_house_locations.yaml`. Each entry is `name: [x, y, theta]` in
the `map` frame (theta in radians):

```yaml
location1: [7.5, 5.0, 0.0]
location2: [6.48, 1.77, 0.0]
location3: [8.4, 2.15, 0.0]
location4: [8.2, 12.7, 0.785]
```

Pick valid coordinates by clicking **"Publish Point"** / **"2D Goal Pose"** in RViz and
reading the coordinates off `/clicked_point` or the terminal, or by inspecting the map.
Then reference the new keys from `tree.xml`. Rebuild only the YAML's package isn't
needed if you built with `--symlink-install`; otherwise re-run `colcon build`.

### Change the patrol order / structure

Edit `tb3_autonomy/bt_xml/tree.xml`. For an endless patrol, wrap the `Sequence` in a
`<Repeat num_cycles="-1">` (BT.CPP v3) decorator. For "keep going even if one waypoint
fails," swap `Sequence` for `<Fallback>` semantics per child, or use a `RetryUntil...`
decorator. Rebuild `tb3_autonomy` (or rely on `--symlink-install` since the XML is just
installed, not compiled).

### Add a new behavior

Add a new BT action class alongside `GoToPose` in `tb3_autonomy/src/`, register it with
the `BehaviorTreeFactory` in `autonomy_node.cpp` (`create_behavior_tree()`), add the
source to `BEHAVIOR_SOURCES` in `CMakeLists.txt`, reference it in `tree.xml`, and
rebuild.

### Change the world or map

`turtlebot3_world.launch.py` hard-codes `turtlebot3_world.world` and the spawn pose;
`nav2.launch.py` hard-codes `tb3_sim/maps/map.yaml` and the AMCL init pose. To use a
different world, swap the world file and regenerate the map (e.g. with SLAM Toolbox),
update `map.yaml`/`map.pgm`, and keep the spawn pose and AMCL init pose consistent.

---

## 7. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Robot never moves; `autonomy_node` logs *"Action server /apple/navigate_to_pose not available!"* | Action-name mismatch — see the note in §5.2. Use `/navigate_to_pose` or a namespace/remap. |
| `colcon build` prints stderr lines | Expected — those are warnings from upstream packages, not failures. Check the final summary says all packages succeeded. |
| RViz shows the map but the robot/scan is in the wrong place; navigation aborts immediately | AMCL not localized. Confirm `amcl_init_pose_publisher` ran, or set "2D Pose Estimate" manually in RViz. Make sure the init pose matches the Gazebo spawn pose. |
| `Gazebo` can't find the TurtleBot3 model | `TURTLEBOT3_MODEL` and `GAZEBO_MODEL_PATH` not exported in that terminal. |
| `bt_navigator` fails to load `nav2_go_to_pose_bt_node` | The `tb3_autonomy` overlay isn't sourced, or it wasn't built. Re-`colcon build` and re-`source install/setup.bash`. |
| `ModuleNotFoundError: transforms3d` | `pip install transforms3d` (or `apt install python3-transforms3d`). |
| Everything launches but `use_sim_time` issues (TFs "extrapolation into the future") | Make sure all three launches use sim time; the provided launch files already set `use_sim_time:=true`. Don't mix with a real-time node. |

---

## 8. Where to go next

- Read [Nav2's behavior-tree docs](https://navigation.ros.org/behavior_trees/index.html)
  to see how the plugin path in §5.4 plugs into the larger Nav2 BT.
- Read the [BehaviorTree.CPP](https://www.behaviortree.dev/) docs for decorators,
  blackboard, subtrees, and reactive sequences.
- Compare with Sebastian Castro's
  [`turtlebot3_behavior_demos`](https://github.com/sea-bass/turtlebot3_behavior_demos),
  which this repo is a stripped-down, beginner-oriented adaptation of.
</content>
</invoke>
