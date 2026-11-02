"""Depth-only terrain hazard node: ROS 2 port of the team's terrain_detect.py idea.

Each depth frame: fit the ground plane (IMU gravity as the "up" prior), build a 5 cm height map
in front of the camera, classify bumps / dips / steep / hidden cells, and report features
(size, height, volume, position).

Subscribes (launch the camera with profile:=terrain, emitter on):
    <camera_ns>/depth/image_rect_raw, <camera_ns>/depth/camera_info, <camera_ns>/imu (optional)
Publishes:
    /terrain/grid      nav_msgs/OccupancyGrid  (-1 unknown, 0 free, 100 hazard) in frame `terrain_ground`
    /terrain/features  std_msgs/String         JSON list of features
    /terrain/image     sensor_msgs/Image       top-down map: red bumps, blue dips, yellow hidden (debug_image:=true)
    TF  <depth optical frame> -> terrain_ground (ground below the camera, x forward, y left, z up)

This is a single-frame map. Phase 3 of docs/ROADMAP.md accumulates frames into a rolling grid in
`odom` once odometry exists, and feeds a Nav2 costmap layer.
"""
import json
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from rover_perception.depth_utils import DEPTH_SCALE_M
from rover_perception.terrain_core import TerrainParams, analyze_depth, render_map, to_occupancy


def quat_from_matrix(m):
    """Rotation matrix -> (x, y, z, w)."""
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    q[3] = (m[k, j] - m[j, k]) / s
    return tuple(q)


class TerrainNode(Node):
    def __init__(self):
        super().__init__("terrain_node")
        d = TerrainParams()
        self.declare_parameter("camera_ns", "/camera/camera")
        self.declare_parameter("ground_frame", "terrain_ground")
        self.declare_parameter("use_imu", True)
        self.declare_parameter("debug_image", False)
        self.declare_parameter("max_rate_hz", 5.0)     # terrain changes slowly; save CPU
        for name in ("resolution", "max_range", "min_range", "width", "step_up", "step_down", "max_slope_deg"):
            self.declare_parameter(name, float(getattr(d, name)))
        self.declare_parameter("pixel_step", d.pixel_step)
        self.declare_parameter("min_hits", d.min_hits)

        gp = self.get_parameter
        self.params = TerrainParams(
            resolution=gp("resolution").value, max_range=gp("max_range").value, min_range=gp("min_range").value,
            width=gp("width").value, step_up=gp("step_up").value, step_down=gp("step_down").value,
            max_slope_deg=gp("max_slope_deg").value, pixel_step=int(gp("pixel_step").value),
            min_hits=int(gp("min_hits").value))
        ns = gp("camera_ns").value.rstrip("/")
        self.ground_frame = gp("ground_frame").value
        self.debug = bool(gp("debug_image").value)
        self.min_period = 1.0 / max(float(gp("max_rate_hz").value), 0.1)

        self.bridge = CvBridge()
        self.K = None
        self.accel = None          # low-passed accelerometer = "up" in the IMU optical frame
        self.last_run = 0.0
        self.create_subscription(CameraInfo, f"{ns}/depth/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, f"{ns}/depth/image_rect_raw", self.on_depth, qos_profile_sensor_data)
        if gp("use_imu").value:
            self.create_subscription(Imu, f"{ns}/imu", self.on_imu, qos_profile_sensor_data)

        self.grid_pub = self.create_publisher(OccupancyGrid, "terrain/grid", 1)
        self.feat_pub = self.create_publisher(String, "terrain/features", 10)
        self.img_pub = self.create_publisher(Image, "terrain/image", 1) if self.debug else None
        self.tf = TransformBroadcaster(self)
        self.warned = set()

    def on_info(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_imu(self, msg):
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        if not np.all(np.isfinite(a)) or np.linalg.norm(a) < 1.0:
            return
        self.accel = a if self.accel is None else 0.95 * self.accel + 0.05 * a

    def on_depth(self, msg):
        now = time.monotonic()
        if self.K is None or now - self.last_run < self.min_period:
            return
        self.last_run = now
        depth = self.bridge.imgmsg_to_cv2(msg, "passthrough").astype(np.float32) * DEPTH_SCALE_M
        # The D435i IMU optical frame has the same axes as the depth optical frame (x right, y down,
        # z forward), and at rest the accelerometer reads +g pointing up.
        up = None if self.accel is None else self.accel / np.linalg.norm(self.accel)
        t0 = time.perf_counter()
        res = analyze_depth(depth, *self.K, up_prior=up, params=self.params)
        ms = (time.perf_counter() - t0) * 1000
        if not res.ok:
            self._warn_once(res.message)
            return
        if res.message:
            self._warn_once(res.message)

        self._publish_tf(msg, res)
        self._publish_grid(msg, res)
        feats = [f.as_dict() for f in res.features]
        self.feat_pub.publish(String(data=json.dumps({
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "frame": self.ground_frame, "camera_height": round(res.camera_height, 3),
            "process_ms": round(ms, 1), "features": feats})))
        if self.img_pub is not None:
            img = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(render_map(res.classes)), "bgr8")
            img.header = msg.header
            self.img_pub.publish(img)
        self.get_logger().info(
            f"{ms:.0f} ms | cam {res.camera_height:.2f} m | "
            + (", ".join(f"{f['kind']} {f['x']:.2f}m/{f['y']:+.2f}m h{f['height']:.2f}" for f in feats) or "clear"),
            throttle_duration_sec=2.0)

    def _publish_tf(self, msg, res):
        t = TransformStamped()
        t.header = msg.header                       # parent: depth optical frame
        t.child_frame_id = self.ground_frame
        origin = -res.camera_height * res.ground_normal_opt     # point on the ground below the camera
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in origin)
        qx, qy, qz, qw = quat_from_matrix(res.R_ground_from_opt.T)   # columns = ground axes in optical
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
        self.tf.sendTransform(t)

    def _publish_grid(self, msg, res):
        p = self.params
        occ = to_occupancy(res.classes)             # (nx along x, ny along y)
        g = OccupancyGrid()
        g.header.stamp = msg.header.stamp
        g.header.frame_id = self.ground_frame
        g.info.resolution = float(p.resolution)
        g.info.width, g.info.height = occ.shape[0], occ.shape[1]
        g.info.origin.position.x, g.info.origin.position.y = res.grid_origin(p)
        g.info.origin.orientation.w = 1.0
        g.data = occ.T.flatten().tolist()           # row-major with x fastest
        self.grid_pub.publish(g)

    def _warn_once(self, text):
        if text not in self.warned:
            self.warned.add(text)
            self.get_logger().warn(text)


def main():
    rclpy.init()
    node = TerrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
