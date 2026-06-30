# path_projector

ROS 2 (Jazzy) package that projects a Nav2 navigation plan onto a live camera image using TF and the pinhole camera model. The output is an annotated image with the planned path drawn as dots and connecting lines, suitable for visualization in RViz2 or any image viewer.

---

## Architecture

```
                        ┌─────────────────────────────────────────────┐
                        │           PathProjectorNode                 │
                        │                                             │
  /apple/plan  ────────►│ _plan_callback()                            │
  (nav_msgs/Path)       │   • Sample every path_step-th pose          │
  1 Hz                  │   • Cache (x,y,z) list → latest_plan        │
                        │                                             │
  /apple/camera1/ ─────►│ _info_callback()  [once]                    │
  camera_info           │   • Extract K matrix (3×3 intrinsics)       │
  (CameraInfo)          │   • Extract D vector (distortion)           │
                        │   • Store cam_frame (frame_id string)       │
                        │                                             │
  /apple/camera1/ ─────►│ _image_callback()  [30 Hz]                  │
  image_raw             │   1. Guard: K and latest_plan must exist     │
  (Image)               │   2. TF lookup: map → cam_frame @ Time(0)   │──► /tf
                        │   3. Apply body→optical rotation fix        │◄── /tf
                        │   4. For each cached plan point:            │
                        │      a. Transform map→camera frame          │
                        │      b. Filter: Z > 0 (in front)           │
                        │      c. Filter: depth_min ≤ dist ≤ depth_max│
                        │      d. Project with cv2.projectPoints()    │
                        │      e. Filter: u,v within image bounds     │
                        │   5. Draw lines + dots on image (OpenCV)    │
                        │   6. Publish annotated image                │
                        │                                        │    │
                        └────────────────────────────────────────┼────┘
                                                                 │
                                                                 ▼
                                                   /path_projection/image
                                                   (sensor_msgs/Image)
```

### Key pipeline stages

| Stage | What happens | Failure symptom |
|---|---|---|
| `_info_callback` | Stores K, D, cam_frame from first CameraInfo | `No camera_info received yet` warn |
| `_plan_callback` | Samples every Nth pose into `latest_plan` | No overlay drawn (republishes raw image) |
| TF lookup | Gets 4×4 transform map→camera at `Time(0)` | `TF lookup failed` warn |
| Body→optical rotation | Corrects Gazebo's body-frame TF to ROS optical convention | All points `behind_cam` (Z < 0) |
| Distance filter | Keeps only points within `[depth_min, depth_max]` metres | `too_close` or `too_far` in diag log |
| Projection + bounds | `cv2.projectPoints` → pixel (u,v), check within image | `out_of_bounds` in diag log |

---

## Topics

### Subscribed

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/apple/plan` | `nav_msgs/Path` | RELIABLE / VOLATILE | Nav2 navigation plan |
| `/apple/camera1/image_raw` | `sensor_msgs/Image` | BEST_EFFORT / VOLATILE | Raw RGB camera feed |
| `/apple/camera1/camera_info` | `sensor_msgs/CameraInfo` | RELIABLE / VOLATILE | Camera intrinsics and distortion |

### Published

| Topic | Type | Description |
|---|---|---|
| `/path_projection/image` | `sensor_msgs/Image` | Annotated image with projected path overlay |

---

## Parameters

All parameters can be set via the YAML config or `--ros-args -p name:=value`.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `plan_topic` | string | `/apple/plan` | Nav2 plan topic |
| `image_topic` | string | `/apple/camera1/image_raw` | Input camera image topic |
| `camera_info_topic` | string | `/apple/camera1/camera_info` | Camera intrinsics topic |
| `output_topic` | string | `/path_projection/image` | Output annotated image topic |
| `map_frame` | string | `map` | Frame that the plan is expressed in — **must match `plan.header.frame_id`** |
| `path_step` | int | `5` | Sample every Nth pose from the plan (lower = denser overlay) |
| `dot_radius` | int | `4` | Pixel radius of each projected dot |
| `line_thickness` | int | `2` | Pixel thickness of connecting lines |
| `dot_color_bgr` | int[3] | `[0, 255, 0]` | Dot color in BGR |
| `line_color_bgr` | int[3] | `[0, 200, 0]` | Line color in BGR |
| `tf_timeout_sec` | float | `0.1` | Max wait for TF lookup (seconds) |
| `queue_size` | int | `5` | Subscription queue depth for image |
| `use_latest_tf` | bool | `true` | Use `Time(0)` (latest TF) instead of image stamp — required for Gazebo simulation due to sim-bridge timestamp lag |
| `depth_min_m` | float | `2.0` | Minimum distance (metres) from camera to show a path point |
| `depth_max_m` | float | `6.0` | Maximum distance (metres) from camera to show a path point |

---

## Camera Configuration (model.sdf)

This node is tuned for **camera1** in the `apple`-namespaced robot:

| Property | Value |
|---|---|
| Link | `apple/camera1_link` |
| Pose | `x=0.1795 m, z=0.548 m`, no rotation (looking straight ahead) |
| TF frame (gz_frame_id) | `apple/camera1_depth_optical_frame` |
| Resolution | 640 × 480 px |
| Horizontal FOV | 1.211 rad (≈ 69°) |
| Depth clip | near=0.1 m, far=10 m |

### Gazebo TF Convention Fix

> **Important:** Gazebo publishes `apple/camera1_depth_optical_frame` using **camera body-frame axes** (X forward, Y left, Z up), not the ROS optical convention (Z forward, X right, Y down). The node applies a correction rotation automatically:
>
> ```
> opt_X = -body_Y    (right  = −left)
> opt_Y = -body_Z    (down   = −up)
> opt_Z = +body_X    (forward)
> ```
>
> Without this correction, all plan points appear `behind_cam` (Z < 0) and nothing is drawn.

---

## QoS Notes

| Topic | Issue | Solution applied |
|---|---|---|
| `camera_info` | Gazebo bridge publishes VOLATILE; default ROS subscriber uses TRANSIENT_LOCAL → connection silently dropped | Subscriber uses `RELIABLE + VOLATILE` |
| `image_raw` | Sensor data — publisher uses BEST_EFFORT | Subscriber uses `BEST_EFFORT + VOLATILE` |
| `plan` | Nav2 publishes RELIABLE | Subscriber uses `RELIABLE` |

---

## Things to Watch Carefully

### 1. `map_frame` must match the plan's `frame_id`
The plan's `header.frame_id` must equal the `map_frame` parameter. The node logs a warning if they differ:
```
Plan updated: frame=apple/map ...   ← if you see "apple/map", set map_frame:=apple/map
```

### 2. `use_latest_tf` must stay `true` for Gazebo
Gazebo image timestamps can run **~1 second ahead** of the TF buffer. Using the image stamp for TF lookup causes:
```
TF lookup failed: Lookup would require extrapolation into the future.
```
`use_latest_tf: true` bypasses this by always using the most recent available transform.

### 3. Camera pitch = 0 — ground visibility window
Camera1 has **no downward tilt** (pitch = 0). Ground-plane path points become visible only beyond a minimum forward distance. At the default camera height (0.548 m) and FOV (69°):

| Distance ahead | Pixel row (v) | Visible? |
|---|---|---|
| < 1.0 m | > 480 (below image) | No |
| 1.3 m | ≈ 479 (bottom edge) | Barely |
| 2.0 m | ≈ 378 | Yes |
| 4.0 m | ≈ 299 | Yes |
| 6.0 m | ≈ 269 | Yes |

This is why `depth_min_m: 2.0` is the recommended minimum.

### 4. Diagnostic log
The node emits a `[diag]` log every 3 seconds showing point rejection breakdown:
```
[diag] frame=apple/camera1_depth_optical_frame total=67
       behind_cam=0 too_close(<2.0m)=5 too_far(>6.0m)=50
       out_of_bounds=0 visible=12
```
Use this to tune `depth_min_m` / `depth_max_m` for your scenario.

### 5. `path_step` vs path density
Nav2 publishes plans at ~0.05 m resolution. With `path_step=5`, dots are spaced ~0.25 m apart in the world. Decrease `path_step` for denser projection (higher CPU cost); increase it to reduce clutter.

---

## Run

```bash
# With YAML config
ros2 run path_projector path_projector_node \
  --ros-args --params-file src/path_projector/config/path_projector.yaml

# Override depth range on the fly
ros2 run path_projector path_projector_node \
  --ros-args --params-file src/path_projector/config/path_projector.yaml \
  -p depth_min_m:=1.0 -p depth_max_m:=10.0

# View output
ros2 run image_view image_view --ros-args -r image:=/path_projection/image
```

---

## Dependencies

| Library | Purpose |
|---|---|
| `rclpy` | ROS 2 Python client |
| `cv_bridge` | ROS Image ↔ OpenCV conversion |
| `opencv-python` (`cv2`) | `projectPoints`, drawing |
| `numpy` | Matrix math, 4×4 transforms |
| `scipy` | Quaternion → rotation matrix (`Rotation.from_quat`) |
| `tf2_ros` | TF buffer and listener |
| `tf2_geometry_msgs` | Registers PoseStamped with tf2 |
