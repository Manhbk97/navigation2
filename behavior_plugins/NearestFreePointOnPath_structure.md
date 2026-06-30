# NearestFreePointOnPath — Structure

A Nav2 Behavior Tree **SyncActionNode** that, when the robot is stuck near its
goal (e.g. a large obstacle is sitting on the goal cell), produces an
**alternative, reachable goal** that sits just in front of the obstacle's near
edge — as close to the original goal as possible without driving into the
obstacle.

- **Header:** [include/behavior_plugins/nearest_free_point_on_path.hpp](include/behavior_plugins/nearest_free_point_on_path.hpp)
- **Source:** [src/nearest_free_point_on_path.cpp](src/nearest_free_point_on_path.cpp)
- **BT class name:** `NearestFreePointOnPath`
- **Registered in:** [src/plugin_registration.cpp:25](src/plugin_registration.cpp#L25)
- **Declared in tree-nodes manifest:** [nav2_tree_nodes.xml:399](nav2_tree_nodes.xml#L399)
- **Built from:** [CMakeLists.txt:29](CMakeLists.txt#L29)

---

## 1. Behavior Tree Ports

| Port | Direction | Type | Required | Description |
|------|-----------|------|----------|-------------|
| `goal` | input | `geometry_msgs/PoseStamped` | **yes** | The original goal pose. Transformed into `map_frame` internally. |
| `path` | input | `nav_msgs/Path` | no | **Kept for backward compatibility only.** The current `tick()` ray-marches the straight robot→goal line and does **not** use this port. |
| `search_radius` | input | `double` | no | Overrides the `search_radius` ROS param for this tick. `<= 0` disables the from-goal distance cap (needed to approach large obstacles). When unset, the ROS param default is used. |
| `new_goal` | **output** | `geometry_msgs/PoseStamped` | — | The adjusted goal: position is the last safe point on the line; orientation is copied from the **original goal**. |

Defined in `providedPorts()` at
[nearest_free_point_on_path.hpp:55-70](include/behavior_plugins/nearest_free_point_on_path.hpp#L55-L70).

---

## 2. ROS Topics

| Topic | Direction | Type | QoS | Notes |
|-------|-----------|------|-----|-------|
| `<ns>/global_costmap/costmap` | **subscribe** | `nav_msgs/OccupancyGrid` | `QoS(1).transient_local()` (latched) | The costmap used for all free/safety lookups. Topic name is the `costmap_topic` param (default is namespace-aware). |

> **Must be the GLOBAL costmap, not the local costmap.**
> All lookups convert a `map`-frame `(x,y)` into a grid cell using the costmap's
> `info.origin`. The global costmap's origin is in the **`map`** frame, matching
> the marched points. The local costmap's origin is in the **`odom`** frame,
> which drifts away from `map` as the robot moves — feeding `map`-frame points
> into an `odom`-frame grid mis-indexes every cell, so every candidate reads as
> "unsafe" and the node always returns `FAILURE`. See the rationale comment at
> [nearest_free_point_on_path.cpp:20-27](src/nearest_free_point_on_path.cpp#L20-L27).

Subscription is created lazily on the **first tick** in `initialize()`
([cpp:121-125](src/nearest_free_point_on_path.cpp#L121-L125)) on a **dedicated
callback group** with its own `SingleThreadedExecutor`, so it does not depend on
the bt_navigator's main executor.

---

## 3. TF Frames

| Lookup | Used for |
|--------|----------|
| `map_frame ← goal.frame_id` | Transform the goal into the map frame ([cpp:549-563](src/nearest_free_point_on_path.cpp#L549-L563)) |
| `map_frame ← robot_frame` | Get the robot pose (start of the marched line) ([cpp:567-577](src/nearest_free_point_on_path.cpp#L567-L577)) |

The TF buffer is **shared** from the BT blackboard key `tf_buffer` when present;
otherwise the node creates its own listener
([cpp:100-109](src/nearest_free_point_on_path.cpp#L100-L109)).

---

## 4. ROS Parameters

All declared under the `nearest_free_point_on_path.` prefix. Declared only if
not already present, so launch-file overrides win
([cpp:35-73](src/nearest_free_point_on_path.cpp#L35-L73)).

| Parameter | Type | Default | Meaning |
|-----------|------|---------|---------|
| `search_radius` | double | `2.1` m | Max distance from goal a returned point may be. `<= 0` disables the cap. Overridable per-tick by the BT `search_radius` port. |
| `min_distance_from_goal` | double | `0.05` m | Skip points closer than this to the goal (don't return a point basically on the blocked goal cell). |
| `free_threshold` | int | `50` (0–100) | Costmap cost `≤` this is considered free. |
| `step_size` | double | `0.05` m | Ray-march step along the robot→goal line. |
| `safety_distance` | double | `0.1` m | Required clearance ring around a candidate point. `<= 0` falls back to a single-cell free check. |
| `robot_frame` | string | inherits `robot_base_frame`, else `<ns>/base_footprint` | TF child frame of the robot base. |
| `map_frame` | string | inherits `global_frame`, else `map` | TF parent / global frame. Must match the costmap's frame. |
| `costmap_topic` | string | `<ns>/global_costmap/costmap` | Costmap topic to subscribe to. |
| `tf_timeout` | double | `0.5` s | TF lookup timeout. |

**Sanity guard:** if `min_distance_from_goal >= search_radius`, no point could
ever qualify, so `min_distance_from_goal` is reset to `0.0`
([cpp:76-83](src/nearest_free_point_on_path.cpp#L76-L83)).

> Note: the header doc-comment lists older defaults (e.g. `search_radius` 1.0);
> the authoritative values are the ones declared in the `.cpp` and tabulated
> above.

---

## 5. Core Logic

### `tick()` — [cpp:517-604](src/nearest_free_point_on_path.cpp#L517-L604)
1. Lazy `initialize()` on first call (TF buffer, callback group, subscription).
2. `waitForCostmap(500 ms)` — drain pending messages and wait for the latched
   costmap sample. If none arrives → `FAILURE`.
3. Read the required `goal` port → transform into `map_frame`.
4. Look up the robot pose in `map_frame`.
5. Resolve the effective radius (BT port overrides the param).
6. Call `findFreePointOnLine(robot, goal, radius, new_goal)`.
   - On success → `setOutput("new_goal", …)` and return **SUCCESS**.
   - Otherwise → **FAILURE**.

### `findFreePointOnLine()` — [cpp:357-512](src/nearest_free_point_on_path.cpp#L357-L512)
The heart of the node. Marches the **straight line** from robot to goal in
`step_size` increments and returns the farthest-along safe sample **before the
first real obstacle** — the obstacle's near edge.

Three phases handle the robot possibly starting inside its own inflation:

- **Phase A — leading unsafe band:** unsafe samples right at the robot (its own
  footprint inflation) are skipped while `entered_free == false`. This is *not*
  treated as "the obstacle".
- **Phase B — free stretch:** record the farthest-along (closest-to-goal) safe
  sample. `best_point` tracks the candidate near-edge point.
- **Phase C — first obstacle after free stretch:** the first unsafe sample once
  we've entered free space is the obstacle's near edge → **STOP**. We never scan
  into free space *beyond* the obstacle.

> **Why a straight line and not the planned `path`?** When a big obstacle sits
> on the goal, the sensor only sees the obstacle's near surface. Cells *behind*
> it (including the goal cell) were never observed, so the costmap reports them
> as FREE. Selecting the "closest safe point to the goal" along a path could pick
> a point in that falsely-free region on the **far** side of the obstacle, and
> planning to it would try to drive the robot *through* the obstacle. Marching
> the straight line and stopping at the near edge avoids this
> ([cpp:388-409](src/nearest_free_point_on_path.cpp#L388-L409)).

On failure it logs an **actionable breakdown** of why samples were unsafe:
- `ring_only > 0, center == 0` → cells are free; only `safety_distance` rejected
  them ⇒ lower `safety_distance` to recover a point.
- `center_occupied` dominates → underlying cells are occupied/unknown ⇒ reduce
  costmap inflation, or the obstacle genuinely spans the whole line.

### Cell-level checks
- **`isFreePose(x, y)`** — [cpp:176-201](src/nearest_free_point_on_path.cpp#L176-L201):
  convert `(x,y)` → cell via `origin`/`resolution`; in-bounds and
  `0 ≤ cost ≤ free_threshold`.
- **`isSafePose(x, y)`** — [cpp:206-252](src/nearest_free_point_on_path.cpp#L206-L252):
  sweep a circular footprint of radius `safety_distance`; **any** out-of-bounds,
  unknown, or above-threshold cell in the circle makes the point unsafe. Needed
  because inflation cost decays with distance and can read below `free_threshold`
  even close to an obstacle.

### `findNearestFreePointOnPath()` — [cpp:269-352](src/nearest_free_point_on_path.cpp#L269-L352)
**Legacy / unused** path-based selector (scan path poses, keep the safe one
closest to the goal). Retained for reference; `tick()` uses the line-march
instead.

---

## 6. Concurrency & Lifecycle

- The costmap subscription runs on a **private callback group + executor**
  ([cpp:111-119](src/nearest_free_point_on_path.cpp#L111-L119)), pumped manually
  by `waitForCostmap()` ([cpp:153-171](src/nearest_free_point_on_path.cpp#L153-L171)).
- `costmap_` is guarded by `costmap_mutex_`; the callback just stores the latest
  message ([cpp:143-147](src/nearest_free_point_on_path.cpp#L143-L147)).
- Because the costmap is `TRANSIENT_LOCAL` (latched), the sample is *not*
  delivered the instant the subscription is created — hence the bounded
  `waitForCostmap` spin loop on both init and every tick.

---

## 7. Data Flow (summary)

```
goal (BT) ──transform──► goal_in_map ─┐
robot (TF) ─lookup────► robot_in_map ─┼─► findFreePointOnLine()
                                      │      │ march line in step_size_ steps
/global_costmap/costmap ──► costmap_ ─┘      │ each sample → isSafePose() [map-frame cells]
                                             │ Phase A skip own inflation
                                             │ Phase B track closest-to-goal safe pt
                                             │ Phase C stop at obstacle near edge
                                             ▼
                                   new_goal (output port) ──► SUCCESS
                                   (else FAILURE)
```

Output `new_goal` is typically consumed downstream by `ComputePathToPose` /
`FollowPath` to drive the robot up to the obstacle edge instead of failing on
the blocked original goal.
