#!/usr/bin/env python3
"""
path_projector_node.py  –  ROS 2 Jazzy
=======================================
Projects nav2 /plan (nav_msgs/Path) onto a camera image using TF and
the pinhole camera model.

Topic names are relative so they resolve against the node's namespace,
which should be set in the launch file (e.g. namespace='apple').

Required topics (relative to node namespace)
---------------------------------------------
  plan                     nav_msgs/Path          (published by nav2)
  camera1/image_raw        sensor_msgs/Image      (raw RGB feed)
  camera1/camera_info      sensor_msgs/CameraInfo (intrinsics + distortion)

Published topics
----------------
  path_projection/image    sensor_msgs/Image      (annotated output)

Parameters (set via YAML or CLI --ros-args -p)
-------------------------------------------------
  plan_topic         (str)   default: plan
  image_topic        (str)   default: camera1/image_raw
  camera_info_topic  (str)   default: camera1/camera_info
  output_topic       (str)   default: path_projection/image
  map_frame          (str)   default: map
  path_step          (int)   default: 5   – sample every Nth pose
  dot_radius         (int)   default: 4   – pixel radius of each dot
  line_thickness     (int)   default: 2   – thickness of connecting lines
  dot_color_bgr      (list)  default: [0,255,0]  – BGR color for dots
  line_color_bgr     (list)  default: [0,200,0]  – BGR color for lines
  tf_timeout_sec     (float) default: 0.1
  queue_size         (int)   default: 5

Usage example (launch file or terminal)
-----------------------------------------
  ros2 run <your_pkg> path_projector_node \\
      --ros-args -r __ns:=/apple \\
      -p path_step:=3
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Path
import message_filters
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped with tf2

from geometry_msgs.msg import PoseStamped, PointStamped


# ---------------------------------------------------------------------------
# Helper: 4x4 transform from TransformStamped
# ---------------------------------------------------------------------------
def transform_to_matrix(t):
    """Convert a geometry_msgs/TransformStamped to a 4×4 numpy array."""
    from scipy.spatial.transform import Rotation
    tr = t.transform.translation
    ro = t.transform.rotation
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_quat([ro.x, ro.y, ro.z, ro.w]).as_matrix()
    mat[:3, 3] = [tr.x, tr.y, tr.z]
    return mat


class PathProjectorNode(Node):

    def __init__(self):
        super().__init__('path_projector_node')

        # ── Declare parameters ──────────────────────────────────────────────
        self.declare_parameter('plan_topic',        'plan')
        self.declare_parameter('image_topic',       'camera1/image_raw')
        self.declare_parameter('camera_info_topic', 'camera1/camera_info')
        self.declare_parameter('output_topic',      'path_projection/image')
        self.declare_parameter('map_frame',         'map')
        self.declare_parameter('path_step',         5)
        self.declare_parameter('dot_radius',        4)
        self.declare_parameter('line_thickness',    2)
        self.declare_parameter('dot_color_bgr',     [0, 255, 0])
        self.declare_parameter('line_color_bgr',    [0, 200, 0])
        self.declare_parameter('tf_timeout_sec',    0.1)
        self.declare_parameter('queue_size',        5)
        self.declare_parameter('use_latest_tf',     True)
        self.declare_parameter('depth_min_m',       2.0)
        self.declare_parameter('depth_max_m',       6.0)

        p = self.get_parameters_by_prefix('')  # convenient shorthand
        def g(name):
            return self.get_parameter(name).value

        self.plan_topic        = g('plan_topic')
        self.image_topic       = g('image_topic')
        self.camera_info_topic = g('camera_info_topic')
        self.output_topic      = g('output_topic')
        self.map_frame         = g('map_frame')
        self.path_step         = int(g('path_step'))
        self.dot_radius        = int(g('dot_radius'))
        self.line_thickness    = int(g('line_thickness'))
        self.dot_color         = tuple(int(x) for x in g('dot_color_bgr'))
        self.line_color        = tuple(int(x) for x in g('line_color_bgr'))
        self.tf_timeout        = g('tf_timeout_sec')
        self.queue_size        = int(g('queue_size'))
        self.use_latest_tf     = bool(g('use_latest_tf'))
        self.depth_min         = float(g('depth_min_m'))
        self.depth_max         = float(g('depth_max_m'))

        # ── Internal state ──────────────────────────────────────────────────
        self.bridge        = CvBridge()
        self.camera_info   = None          # filled once on first CameraInfo msg
        self.K             = None          # 3×3 numpy intrinsic matrix
        self.D             = None          # distortion coefficients (numpy)
        self.cam_frame     = None          # optical frame id from camera_info
        self.latest_plan   = None          # list of sampled (x,y,z) in map frame

        # ── TF ──────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── QoS profiles ────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=self.queue_size,
        )
        camera_info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # ── Subscriptions ────────────────────────────────────────────────────
        # CameraInfo – volatile (Gazebo bridge publishes VOLATILE, not latched)
        self.info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._info_callback,
            camera_info_qos,
        )

        # Plan – use RELIABLE since nav2 publishes it reliably
        self.plan_sub = self.create_subscription(
            Path,
            self.plan_topic,
            self._plan_callback,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1),
        )

        # Image – synchronize with a simple subscription; we project the
        # latest plan onto every arriving frame
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            sensor_qos,
        )

        # ── Publisher ────────────────────────────────────────────────────────
        self.image_pub = self.create_publisher(Image, self.output_topic, 5)

        self.get_logger().info(
            f'PathProjectorNode started.\n'
            f'  Listening plan   : {self.plan_topic}\n'
            f'  Listening image  : {self.image_topic}\n'
            f'  Listening info   : {self.camera_info_topic}\n'
            f'  Publishing to    : {self.output_topic}\n'
            f'  Map frame        : {self.map_frame}\n'
            f'  Path step        : every {self.path_step} pose(s)\n'
            f'  TF mode          : {"latest (Time(0))" if self.use_latest_tf else "image stamp"}'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _info_callback(self, msg: CameraInfo):
        """Store camera intrinsics. Called once (latched topic)."""
        if self.K is not None:
            return  # already initialised
        self.camera_info = msg
        self.cam_frame   = msg.header.frame_id
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)
        self.get_logger().info(
            f'Camera intrinsics received. Optical frame: {self.cam_frame}\n'
            f'  K = {self.K}'
        )

    def _plan_callback(self, msg: Path):
        """Cache the sampled plan points in the plan frame."""
        plan_frame = msg.header.frame_id
        # Warn if the plan frame doesn't match map_frame — TF lookup will use
        # map_frame as the source, so a mismatch means wrong coordinates.
        if plan_frame and plan_frame != self.map_frame:
            self.get_logger().warn(
                f'Plan frame_id="{plan_frame}" != map_frame="{self.map_frame}". '
                f'Set map_frame to "{plan_frame}" via parameter.',
                throttle_duration_sec=10.0,
            )
        pts = []
        for i, pose_stamped in enumerate(msg.poses):
            if i % self.path_step != 0:
                continue
            p = pose_stamped.pose.position
            pts.append((p.x, p.y, p.z))
        self.latest_plan = pts
        self.get_logger().info(
            f'Plan updated: frame={plan_frame} {len(msg.poses)} poses → {len(pts)} sampled',
            throttle_duration_sec=5.0,
        )

    def _image_callback(self, msg: Image):
        """Project the cached plan onto the incoming image and publish."""
        # Guard: need both intrinsics and a plan
        if self.K is None:
            self.get_logger().warn(
                'No camera_info received yet – skipping frame.',
                throttle_duration_sec=5.0,
            )
            return

        if not self.latest_plan:
            # Nothing to draw – republish original
            self.image_pub.publish(msg)
            return

        # Convert ROS image → OpenCV BGR
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # Look up TF: map → camera_optical
        # use_latest_tf=True  → Time(0) = most recent TF available (avoids
        #                        "extrapolation into future" from sim bridge lag)
        # use_latest_tf=False → exact image stamp (requires TF to be in sync)
        tf_time = Time() if self.use_latest_tf else msg.header.stamp
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.cam_frame,           # target frame  (camera optical)
                self.map_frame,           # source frame  (map)
                tf_time,                  # time
                timeout=rclpy.duration.Duration(
                    seconds=self.tf_timeout
                ),
            )
        except TransformException as e:
            self.get_logger().warn(
                f'TF lookup failed ({self.map_frame} → {self.cam_frame}): {e}',
                throttle_duration_sec=2.0,
            )
            self.image_pub.publish(msg)
            return

        T_cam_from_map = transform_to_matrix(tf_stamped)  # 4×4

        # Gazebo publishes camera1_depth_optical_frame with body-frame axes
        # (X fwd, Y left, Z up) instead of ROS optical axes (Z fwd, X right, Y down).
        # Apply the standard body → optical rotation to correct this:
        #   opt_X = -body_Y  (right = -left)
        #   opt_Y = -body_Z  (down  = -up)
        #   opt_Z = +body_X  (forward)
        R_body_to_optical = np.array([
            [ 0, -1,  0, 0],
            [ 0,  0, -1, 0],
            [ 1,  0,  0, 0],
            [ 0,  0,  0, 1],
        ], dtype=np.float64)
        T_cam_from_map = R_body_to_optical @ T_cam_from_map

        # Project each sampled point – count rejection reasons for diagnostics
        pixel_pts = []
        n_behind = n_near = n_far = n_oob = 0
        samples = []

        for (mx, my, mz) in self.latest_plan:
            p_cam = T_cam_from_map @ np.array([mx, my, mz, 1.0])
            X, Y, Z = float(p_cam[0]), float(p_cam[1]), float(p_cam[2])
            dist = float(np.sqrt(X*X + Y*Y + Z*Z))

            if len(samples) < 3:
                samples.append(f'map({mx:.1f},{my:.1f},{mz:.1f})'
                                f'→cam(Z={Z:.2f},dist={dist:.2f}m)')

            if Z <= 0.01:
                n_behind += 1
                continue
            if dist < self.depth_min:
                n_near += 1
                continue
            if dist > self.depth_max:
                n_far += 1
                continue

            obj_pt = np.array([[[X, Y, Z]]], dtype=np.float64)
            img_pt, _ = cv2.projectPoints(
                obj_pt, np.zeros(3), np.zeros(3), self.K, self.D)
            u = int(round(img_pt[0, 0, 0]))
            v = int(round(img_pt[0, 0, 1]))
            h, w = self.camera_info.height, self.camera_info.width
            if not (0 <= u < w and 0 <= v < h):
                n_oob += 1
                if len(samples) < 5:
                    samples.append(f'  oob u={u},v={v} img={w}x{h}')
                continue
            pixel_pts.append((u, v))

        self.get_logger().info(
            f'[diag] frame={self.cam_frame} '
            f'total={len(self.latest_plan)} '
            f'behind_cam={n_behind} '
            f'too_close(<{self.depth_min}m)={n_near} '
            f'too_far(>{self.depth_max}m)={n_far} '
            f'out_of_bounds={n_oob} '
            f'visible={len(pixel_pts)} | '
            + ' | '.join(samples),
            throttle_duration_sec=3.0,
        )

        self._draw_path(cv_img, pixel_pts)

        # Publish
        out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)

    # ── Core projection ────────────────────────────────────────────────────

    def _project_point(self, mx, my, mz, T_cam_from_map):
        """
        Transform a 3D point from map frame to camera frame, then project
        to pixel coordinates using the pinhole + distortion model.

        Returns (u, v) as integers, or None if the point is behind the camera
        or outside the image bounds.
        """
        # 1. Transform to camera frame
        p_map = np.array([mx, my, mz, 1.0])
        p_cam = T_cam_from_map @ p_map   # [X, Y, Z, 1]

        X, Y, Z = p_cam[0], p_cam[1], p_cam[2]

        # Only project points in FRONT of the camera (Z > 0)
        if Z <= 0.01:
            return None

        # Filter by Euclidean distance from the camera (2–6 m range)
        dist = float(np.sqrt(X*X + Y*Y + Z*Z))
        if not (self.depth_min <= dist <= self.depth_max):
            return None

        # 2. Project using OpenCV (handles distortion correctly)
        # objectPoints shape: (1, 1, 3)
        obj_pt = np.array([[[X, Y, Z]]], dtype=np.float64)
        img_pt, _ = cv2.projectPoints(
            obj_pt,
            np.zeros(3),       # rvec = identity (already in camera frame)
            np.zeros(3),       # tvec = zero
            self.K,
            self.D,
        )
        u = int(round(img_pt[0, 0, 0]))
        v = int(round(img_pt[0, 0, 1]))

        # 3. Bounds check
        h, w = self.camera_info.height, self.camera_info.width
        if 0 <= u < w and 0 <= v < h:
            return (u, v)
        return None

    def _draw_path(self, img, pixel_pts):
        """Draw connecting lines and dots on img (in-place)."""
        if not pixel_pts:
            return

        # Draw lines between consecutive projected points
        if len(pixel_pts) > 1:
            for i in range(len(pixel_pts) - 1):
                cv2.line(
                    img,
                    pixel_pts[i],
                    pixel_pts[i + 1],
                    self.line_color,
                    self.line_thickness,
                    lineType=cv2.LINE_AA,
                )

        # Draw dots on top
        for pt in pixel_pts:
            cv2.circle(img, pt, self.dot_radius, self.dot_color, -1,
                       lineType=cv2.LINE_AA)


# ── Entry point ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PathProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()