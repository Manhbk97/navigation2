# Tutorial: Linking BehaviorTree.CPP with Groot in ROS 2

This tutorial explains how to connect the **Groot** visualization tool to the `turtlesim_bt` package for real-time behavior tree monitoring and log replay.

---

## Prerequisites

- ROS 2 (Humble or later) installed and sourced
- Workspace at `~/rgt2_ws` with the `turtlesim_bt` package
- `libzmq3-dev` installed (required for live monitoring)

Verify ZMQ is available:

```bash
sudo apt install libzmq3-dev
```

---

## Understanding the Integration

The `turtlesim_bt` package uses **BehaviorTree.CPP v3** (`behaviortree_cpp_v3`). Groot has two versions — you must use the correct one:

| Groot Version | Compatible With |
|---|---|
| **Groot1** (master branch) | BehaviorTree.CPP **v3** — your case |
| **Groot2** | BehaviorTree.CPP v4 only |

The package already has both loggers configured in [`turtlesim_bt/src/turtle_bt.cpp`](turtlesim_bt/src/turtle_bt.cpp) (lines 350–352):

```cpp
// Writes a .fbl file for post-run replay
BT::FileLogger file_logger(tree, log_file_path.c_str());

// Opens ZMQ ports for live monitoring in Groot
BT::PublisherZMQ publisher(tree);
```

The `PublisherZMQ` broadcasts the tree state over two ZMQ ports:

| Port | Purpose |
|---|---|
| **1666** | Live node status updates (publisher) |
| **1667** | Tree structure (server) |

No code changes are needed — the integration is already in place.

---

## Step 1 — Install Groot1

### Option A: AppImage (Recommended)

Download and make executable:

```bash
wget https://github.com/BehaviorTree/Groot/releases/download/1.0.0/Groot-1.0.0-x86_64.AppImage
chmod +x Groot-1.0.0-x86_64.AppImage
```

Run Groot:

```bash
./Groot-1.0.0-x86_64.AppImage
```

### Option B: Build from Source

Install dependencies:

```bash
sudo apt install qtbase5-dev libzmq3-dev libdw-dev
```

Clone and build:

```bash
git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git
cd Groot
git checkout master   # Groot1 is on the master branch
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

Run Groot:

```bash
./build/Groot
```

---

## Step 2 — Build and Launch the Package

Open a terminal, build and source the workspace:

```bash
cd ~/rgt2_ws
colcon build --packages-select turtlesim_bt
source install/setup.bash
```

Launch the simulation and behavior tree:

```bash
ros2 launch turtlesim_bt launch_turtlesim_bt.launch.py
```

This starts two nodes:
1. `turtlesim_node` — the turtle simulator window
2. `turtle_bt` — executes the behavior tree and opens ZMQ ports 1666/1667

> **Important:** Keep this terminal running. The ZMQ ports are only available while `turtle_bt` is active.

---

## Step 3 — Real-Time Monitoring with Groot

1. Launch Groot (in a second terminal or separate window).

2. In the Groot window, click **"Monitor"**.

3. Enter the connection parameters:

   | Field | Value |
   |---|---|
   | Publisher IP | `localhost` |
   | Publisher Port | `1666` |
   | Server Port | `1667` |

4. Click **"Connect"**.

The behavior tree will appear and update live as the turtle moves. Node colors indicate their current state:

| Color | State |
|---|---|
| Green | SUCCESS |
| Yellow | RUNNING |
| Red | FAILURE |
| Grey | IDLE (not yet ticked) |

You can observe in real time how the tree navigates between the high-battery path (moving to goals) and the low-battery path (returning to charge).

---

## Step 4 — Log Replay (Post-Run)

Every run writes a binary log file. Its location after building:

```
~/rgt2_ws/install/turtlesim_bt/share/turtlesim_bt/config/bt_log.fbl
```

The source copy is at:

```
~/rgt2_ws/src/Turtlesim_BT/turtlesim_bt/config/bt_log.fbl
```

To replay a recorded run:

1. Open Groot → click **"Log Replay"**.
2. Click the folder icon and select the `.fbl` file.
3. Use the playback controls to step forward/backward through each tick.

This is useful for debugging — you can inspect the exact state of every node at every tick without needing the simulation running.

---

## Step 5 — Edit the Tree Visually (Optional)

Groot can also be used as a visual editor for the XML tree file located at [`turtlesim_bt/config/turtle_tree.xml`](turtlesim_bt/config/turtle_tree.xml).

1. Open Groot → click **"Editor"**.
2. Click **"Load"** and select `turtle_tree.xml`.
3. Edit the tree visually (drag nodes, change ports, add/remove nodes).
4. Click **"File → Save"** to overwrite the XML.
5. Rebuild and relaunch to run the updated tree.

> **Tip:** If you connect Groot in Monitor mode while the tree is running, it auto-discovers all custom node models (ports, types). This makes it easier to edit nodes correctly in the Editor afterward.

---

## File Reference

| File | Description |
|---|---|
| [`turtlesim_bt/src/turtle_bt.cpp`](turtlesim_bt/src/turtle_bt.cpp) | Main node — contains `PublisherZMQ` and `FileLogger` setup |
| [`turtlesim_bt/config/turtle_tree.xml`](turtlesim_bt/config/turtle_tree.xml) | Behavior tree XML loaded at runtime |
| `turtlesim_bt/config/bt_log.fbl` | Generated log file for Groot replay |
| [`turtlesim_bt/CMakeLists.txt`](turtlesim_bt/CMakeLists.txt) | Links `zmq` library required by `PublisherZMQ` |
| [`turtlesim_bt/launch/launch_turtlesim_bt.launch.py`](turtlesim_bt/launch/launch_turtlesim_bt.launch.py) | Launch file that starts both nodes |

---

## Troubleshooting

| Problem | Cause | Fix |
|---|---|---|
| Groot shows "Connection refused" | `turtle_bt` is not running | Launch the package first (Step 2), then connect |
| Groot shows a blank/empty tree | Wrong Groot version | You need **Groot1** — Groot2 is incompatible with BT.CPP v3 |
| Build fails with ZMQ error | Missing ZMQ library | `sudo apt install libzmq3-dev` then rebuild |
| Tree stops updating in Groot | Node execution ended | The tree only ticks while `rclcpp::ok()` is true — check the terminal for errors |
| `.fbl` file is empty | Package not rebuilt after changes | `colcon build --packages-select turtlesim_bt` and relaunch |
