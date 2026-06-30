# `state_machine.cpp` — System Design / Data Flow & Logic (ROS 2)

`TaskManager` is a ROS 2 lifecycle component that accepts navigation tasks (serving,
cruising, docking, etc.), drives them through a state machine on a 100 ms timer, and
commands the Nav2 action servers (`navigate_to_pose`, `follow_waypoints`, `dock_robot`,
`undock_robot`). It is the central "brain" that turns high-level task requests into
low-level navigation goals while publishing status/progress back to the system.

---

## 1. Top-Level Block Diagram (Inputs → Core → Outputs)

```
                         ┌──────────────────────────────────────────────┐
   INPUTS                │                 TaskManager                  │   OUTPUTS
                         │                                              │
 Service                 │  ┌────────────┐   ┌─────────────────────┐    │  Topics (pub)
  /task_request ─────────┼─►│ handleTask │──►│  tasks_  (deque)    │    │   /task_progress
  (TaskServe)            │  │  Request   │   │  TaskInfo queue     │    │   /navi_status
                         │  └────────────┘   └─────────┬───────────┘    │   /speed_limit
 Topics (sub)            │                             │                │   /tray_det_enable
  /positions ───────────►│  positionsCallback ──► positions_, docks_    │   /service_log
  /core_status ─────────►│  coreStatusCallback ──► core_status_         │
  /tray_status ─────────►│  trayStatusCallback ──► tray_status_         │  Action goals (send)
  /return_button/status ►│  doneCallback ──────► cmdDone()              │   navigate_to_pose
                         │                             │                │   follow_waypoints
 Config files (poll 2s)  │                             ▼                │   dock_robot
  target_speeds.yaml ───►│  watchConfigFiles ──► target_speeds_         │   undock_robot
  start_pose.yaml ──────►│                       start_pose_id_         │   (+ async_cancel)
                         │                             │                │
 TF tree                 │                             ▼                │
  map→base_footprint ───►│  getRobotPose() ─────► robot pose            │
                         │                             │                │
 Action feedback/result  │            ┌────────────────▼─────────────┐  │
  (4 nav actions) ──────►│            │  runCoordinator() @ 100 ms   │  │
                         │            │  (state machine, see §3)     │  │
                         │            └──────────────────────────────┘  │
                         └──────────────────────────────────────────────┘
```

---

## 2. Inputs & Outputs Reference

### Inputs

| Kind | Name | Type | Handler → State updated |
|------|------|------|--------------------------|
| Service | `task_request` | `TaskServe` | `handleTaskRequest` → `tasks_` |
| Sub | `positions` | `Positions` | `positionsCallback` → `positions_`, `docks_` |
| Sub | `core_status` | `CoreStatus` | `coreStatusCallback` → `core_status_`, resets timeout |
| Sub | `tray_status` | `Int8MultiArray` | `trayStatusCallback` → `tray_status_` |
| Sub | `return_button/status` | `Bool` | `doneCallback` → issues `COMMAND_DONE` |
| Action FB/Result | `navigate_to_pose` | `NavigateToPose` | `general_feedback_` / `general_result_` |
| Action FB/Result | `follow_waypoints` | `FollowWaypoints` | `cruise_feedback_` / `cruise_result_` |
| Action FB/Result | `dock_robot` | `DockRobot` | `dock_feedback_` / `dock_result_` |
| Action FB/Result | `undock_robot` | `UndockRobot` | `undock_feedback_` / `undock_result_` |
| File (poll 2 s) | `target_speeds.yaml` | YAML | `loadTargetSpeeds` → `target_speeds_` |
| File (poll 2 s) | `start_pose.yaml` | YAML | `loadStartPose` → `start_pose_id_` |
| TF | `map → base_footprint` | transform | `getRobotPose()` |

### Outputs

| Kind | Topic / Action | Type | Produced by |
|------|----------------|------|-------------|
| Pub (10 Hz) | `task_progress` | `Progress` | `publishProgress` |
| Pub (10 Hz) | `navi_status` | `NaviStatus` | `publishNaviStatus` |
| Pub (latched) | `speed_limit` | `SpeedLimit` | `setSpeedLimitForCurrentGoal` |
| Pub (latched) | `tray_det_enable` | `Bool` | `publishNaviStatus` (proximity gated) |
| Pub (latched) | `service_log` | `ServiceLog` | `logTaskData` / `logCruiseData` |
| Action goal | `navigate_to_pose` | goal | `sendGeneralGoal` |
| Action goal | `follow_waypoints` | goal | `sendCruiseGoal` |
| Action goal | `dock_robot` | goal | `sendDockGoal` |
| Action goal | `undock_robot` | goal | `sendUndockGoal` |
| Service resp | `task_request` | `TaskServe.Response` | `cmd*` handlers |

---

## 3. State Machine (`runCoordinator`, 100 ms)

`navi_status_.navi_state` is the active state. Each tick first applies global guards
(estop / teleop / charging) then dispatches to the matching `handle*()`.

```
                         ┌─────────────────────────────────────────────┐
   GLOBAL GUARDS (every tick, before switch):                          │
     estop OR teleop_active  ──► cancelActiveGoals(), hold_for_core_    │
     cleared again           ──► force NAV_IDLE                         │
                                                                        │
                        ┌──────────┐                                    │
              ┌────────►│ NAV_IDLE │◄──────────────────────────────────┤
              │         └────┬─────┘   dispatchNextTask():              │
              │              │           - pop finished tasks (+log)    │
              │              │           - maybe enqueue return task    │
              │              │           - undock if docked ───────┐    │
              │              │           - set speed limit          │    │
              │              │           - send Cruise/General goal │    │
              │              ▼                                      ▼    │
              │        ┌──────────┐                          ┌───────────┐
              │        │ NAV_PLAN │                          │NAV_UNDOCK │
              │        └────┬─────┘ goal accepted            └─────┬─────┘
              │             │  (handle*: success→IDLE,             │ success→IDLE
              │             ▼                  abort→ABORT)        │ abort→ABORT
              │        ┌──────────┐                                │
              │        │ NAV_NAVI │                                │
              │        └────┬─────┘                                │
              │   ┌─────────┼──────────┐                           │
              │   │ general │ cruise   │ dock-target               │
              │   ▼         ▼          ▼                           │
              │ success   success   ┌──────────┐                   │
              │   │         │       │ NAV_DOCK │                   │
              │   ▼         ▼       └────┬─────┘                   │
              │ ┌────────────┐           │ success                 │
              │ │ NAV_GPOINT │◄──────────┘                         │
              │ └────┬───────┘  (wait_time elapsed                 │
              │      │ → mark FINISHED) ──► back to NAV_IDLE        │
              │      │                                             │
   from any   │  ┌─────────┐  ┌──────────┐  ┌──────────┐  ┌─────────┐
   state ─────┴─►│NAV_PAUSE│  │NAV_CANCEL│  │NAV_ABORT │  │NAV_HALT │
                 └─────────┘  └──────────┘  └──────────┘  └─────────┘
                  resume→IDLE  done→IDLE     timeout→IDLE  core OK→IDLE
                               (cancel goals) (retry)      (core fault)
```

State responsibilities:

- **NAV_IDLE** (`handleIdle` → `dispatchNextTask`): advances the task queue, logs
  finished sub-goals, may inject a return-to-wait/start task, performs undocking if the
  robot sits on a dock, sets the speed limit, then sends the first nav goal.
- **NAV_PLAN** (`handlePlanning`): waits for the action server to accept the goal, then
  promotes to NAV_NAVI and records `start_time`.
- **NAV_NAVI** (`handleNavigating`): per-tick — checks cancel/pause, checks core health
  (→ NAV_HALT), accumulates travel distance, and processes navigation result+feedback
  (cruise vs. general path).
- **NAV_GPOINT** (`handleGoalPoint`): goal reached; holds for `wait_time` (ack-required
  task types stay ARRIVED until acknowledged), then marks FINISHED → NAV_IDLE.
- **NAV_DOCK / NAV_UNDOCK** (`handleDocking` / `handleUndocking`): drive the dock/undock
  actions; success → NAV_GPOINT / NAV_IDLE, abort → NAV_ABORT.
- **NAV_PAUSE** (`handlePaused`): cancels active goals, holds; resume → NAV_IDLE.
- **NAV_CANCEL** (`handleCanceling`): cancels active goals, marks sub-goals CANCELED,
  waits for terminal result → NAV_IDLE.
- **NAV_ABORT** (`handleAborted`): waits `abort_timeout_`, resets result codes, retries
  via NAV_IDLE.
- **NAV_HALT** (`handleHalt`): entered on core fault / core-status timeout; cancels goals
  until core nominal, then → NAV_IDLE.

---

## 4. Request Command Flow (`handleTaskRequest`)

```
/task_request ──► requestValidityCheck ──► switch(command)
                                            0 cmdAdd     → push TaskInfo to tasks_
                                            1 cmdPause   → task->pause()
                                            2 cmdResume  → task->resume()
                                            3 cmdCancel  → task->cancel()
                                            4 cmdSkip    → task->skip()
                                            5 cmdDone    → task->done()
                                            6 cmdStatus  → read sub_goals_status
```

`cmdAdd` also expands a `*demo*` goal into one goal per `positions_->table` entry
(serving task, 20 loops). Command results feed `response->success`, `task_id`,
`tasks_pending`, and `task_status`.

---

## 5. Task → Goal Dispatch Detail (`dispatchNextTask`)

```
tasks_.front() ──► logTaskData ──► set_sub_goal_index()
   │  (no active sub-goal) → last_task_ = task; pop; returnTask()
   ▼
active task exists
   │
   ├─ sendUndockGoal()  ── if isDocked() → undock action, state=NAV_UNDOCK, return
   │
   ├─ getTravelDistance() reset + restore goal's traveled_distance
   ├─ setSpeedLimitForCurrentGoal() ── publishes /speed_limit
   │      (SPEED_DEFAULT→target_speeds_[type]; SLOW/NORMAL/FAST; smooth_mode caps 0.3)
   │
   └─ task_type == CRUISING ? sendCruiseGoal()   (follow_waypoints, closest-WP first)
                            : sendGeneralGoal()   (navigate_to_pose, with dock offset calc)
```

`returnTask()`: after a SERVING/CALLBELL/INSTANT task finishes (and queue empty), it
auto-creates a `_return` task toward a `wait` or `start` position.

---

## 6. Pose / Dock Geometry Helpers

- `getRobotPose()` — TF lookup `map → [ns/]base_footprint`.
- `getTravelDistance()` — integrates pose deltas into `travelDistance`.
- `isDocked()` — projects each dock pose by `±dock_distance_/2`, uses
  `distanceToSegment` + `yawDifference` against tolerances to decide if docked.
- `sendGeneralGoal()` — offsets the goal by `dock_distance_` along yaw for
  FRONT/BACK/CHARGE docks (and rotates 180° for back docks) before approach.
- `distanceToPose`, `distanceToSegment`, `yawDifference` — pure geometry utilities.

---

## 7. Lifecycle & Timers

| Lifecycle | Effect |
|-----------|--------|
| Constructor | Creates clients, service, publishers, timers (cancelled), TF, loads params |
| `activate()` | Creates subscriptions, resets the 3 timers (progress / navi_status / coordinator) |
| `deactivate()` | Cancels timers, drops subs, cancels active goals, state→IDLE |
| `cleanup()` | Clears tasks, resets all handles/interfaces/clients |

| Timer | Period | Callback |
|-------|--------|----------|
| `coordinator_timer_` | 100 ms | `runCoordinator` (state machine) |
| `progress_timer_` | 100 ms | `publishProgress` |
| `navi_status_timer_` | 100 ms | `publishNaviStatus` (+ increments core timeout) |
| `config_watch_timer_` | 2 s | `watchConfigFiles` (YAML hot-reload) |
