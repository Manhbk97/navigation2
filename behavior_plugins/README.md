# behavior_plugins

 The package provides condition / action / decorator nodes that extend the default rgt navigator with narrow-path handling, location-based goal selection, runtime costmap tuning, and smarter recovery logic, together with a collection of ready-to-use behavior tree XML files.

---

## Plugins Overview

All plugin nodes are declared in the `behavior_plugins` namespace and registered through a single `BT_REGISTER_NODES` factory in [src/plugin_registration.cpp](src/plugin_registration.cpp).

| BT XML name | Node kind | Purpose |
| --- | --- | --- |
| `NarrowPath` | Condition | Returns `SUCCESS` when `/narrow_path_detected` is `true`. Used as a gate so the behavior tree can swap the planner (e.g. to `NarrowPlanner`) whenever the robot enters a narrow corridor. |
| `HumanInNarrowPath` | Condition | Returns `SUCCESS` only when both `/human_detected` **and** `/narrow_path_detected` are `true`. Used to trigger a "step aside and wait" sub-tree when a human is blocking a narrow corridor. |
| `RecoveryOncePerLocation` | Decorator | Allows its recovery child to run **at most once per unique robot location**. After it triggers, the child is skipped (returns `SUCCESS`) until the robot moves at least `min_distance` metres from where recovery last fired. Prevents the robot from burning recovery cycles repeatedly at the same stuck spot, while keeping the outer `RecoveryNode` alive so replanning continues. |
| `ResetPath` | SyncAction | Clears a `nav_msgs/Path` entry on the blackboard. Replacement for `<SetBlackboard output_key="path" value=""/>`, which BT.CPP cannot parse into a typed `Path`. After this runs, `FollowPath` fails on the next tick so `ComputePathToPose` is forced to replan. |
| `SearchLocalGoalAside` | SyncAction | Searches the local costmap for a free pose placed `search_distance` metres from the robot, `search_angle_deg` degrees clockwise (to the right) of its previous heading. Sweeps ±`search_sweep_deg` degrees to find the nearest free cell. Writes the chosen pose to the `goal` output port. Useful for "step aside" behaviors. |
| `SetCostmapInflationRadius` | Action (BtServiceNode) | Calls `rcl_interfaces/srv/SetParameters` to change `inflation_layer.inflation_radius` of a running costmap at runtime. Uses a blackboard-backed cache keyed by service name, so multiple instances that target the same costmap share one cache and restores work correctly when the BT has narrow / normal branches. |
| `SetGoalFromLocation` | SyncAction | Reads a named pose from a locations YAML file and writes it to the BT blackboard as a `PoseStamped`. The special key `loc="closest"` subscribes to `/<namespace>/ekf_odom` and auto-selects the nearest location to the robot's current pose. Typical use: pick a "wait place" near the robot when a human blocks the path. |

### Topics, ports, and parameters (quick reference)

- **NarrowPath** — Subscribes: `/narrow_path_detected` (`std_msgs/Bool`). No BT ports.
- **HumanInNarrowPath** — Subscribes: `/human_detected`, `/narrow_path_detected` (both `std_msgs/Bool`). No BT ports.
- **RecoveryOncePerLocation** — Input port: `min_distance` (double, default `0.3` m).
- **ResetPath** — Output port: `path` (`nav_msgs/Path`).
- **SearchLocalGoalAside** — Output port: `goal` (`PoseStamped`). ROS 2 parameters (read from the `bt_navigator` node):
  - `search_local_goal_aside.search_angle_deg`  (default `60.0`)
  - `search_local_goal_aside.search_distance`   (default `0.5` m)
  - `search_local_goal_aside.free_threshold`    (default `50`, range 0–100)
  - `search_local_goal_aside.search_sweep_deg`  (default `20.0`)
  - `search_local_goal_aside.search_step_deg`   (default `5.0`)
  - `search_local_goal_aside.robot_frame`, `map_frame`, `costmap_topic`, `tf_timeout`
- **SetCostmapInflationRadius** — Input ports: `service_name`, `inflation_radius`, `param_name` (default `inflation_layer.inflation_radius`), `server_timeout`.
- **SetGoalFromLocation** — Input ports: `location_file` (absolute path), `loc` (key name or `"closest"`). Output port: `goal` (`PoseStamped`). Expected YAML format:
  ```yaml
  locations:
    location1:
      x: 1.23
      y: 4.56
      yaw: 0.0   # radians, map frame
  ```

---

## Build

```bash
cd ~/ros2_robot_ws
colcon build --packages-select behavior_plugins --symlink-install
source install/setup.bash
```

The build produces `libbehavior_plugins.so`, installed under `install/behavior_plugins/lib/`. Behavior tree XML files are installed under `install/behavior_plugins/share/behavior_plugins/behavior_trees/`.

Dependencies (all declared in [package.xml](package.xml)): `rclcpp`, `pluginlib`, `nav2_behavior_tree`, `nav2_util`, `behaviortree_cpp`, `std_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `rcl_interfaces`, `yaml_cpp_vendor`.

---

## Setup — Loading the Plugins in `bt_navigator`

Nav2's `bt_navigator` loads BT nodes via the `plugin_lib_names` parameter. Add `behavior_plugins` to this list in your `nav2_params.yaml`:
```
    default_nav_to_pose_bt_xml: /src/behavior_plugins/behavior_trees/HumanInNarrowPath.xml
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    location_file: /src/bcr_bot/config/location.yaml
    plugin_lib_names:
      - behavior_plugins
```
Launch Nav2 as usual; `bt_navigator` will `dlopen` `libbehavior_plugins.so` and all seven node types will be available to any behavior tree XML.

---


## Debugging Tips

- Make sure `behavior_plugins` appears in `bt_navigator`'s `plugin_lib_names`. If a node type is missing, `bt_navigator` logs `Error: ... not found in the factory`.
- Topics the plugins subscribe to (`/human_detected`, `/narrow_path_detected`, `/<ns>/ekf_odom`, local costmap) should be remapped per robot namespace in your launch file.
- For `SetGoalFromLocation`, use an **absolute path** for `location_file` — the node does not resolve package-relative paths.
- `SearchLocalGoalAside` uses the robot's heading *from the previous tick* as the search reference, so the first call after a fresh tree load falls back to the current heading.

---


## Update log

| Version | Timestamp | Description |
| --- | :---: | :--- |
| ![version](https://badgen.net/badge/version/1.0.0/blue) | 2026-04-15 | Initial release: 7 Nav2 BT plugins — `NarrowPath`, `HumanInNarrowPath`, `RecoveryOncePerLocation`, `ResetPath`, `SearchLocalGoalAside`, `SetCostmapInflationRadius`, `SetGoalFromLocation` — plus ready-to-use BT XML files under `behavior_trees/`. |


---


## License

Apache License 2.0 — see [package.xml](package.xml).
