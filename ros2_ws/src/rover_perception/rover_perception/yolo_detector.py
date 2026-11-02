"""YOLO detector node: subscribes to the realsense2_camera color + aligned depth topics
and publishes vision_msgs/Detection3DArray on /detections.

It never opens the camera, so the IR + IMU streams for VIO and AprilTags keep sharing
the single camera node. Launch the camera with profile:=detect (align_depth.enable: true).

Run (inside the YOLO venv, workspace sourced):
    ros2 run rover_perception yolo_detector --ros-args -p engine_path:=$HOME/Robotics-cam/models/yolo26n.engine
"""
import time

import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from rover_perception.depth_utils import DEPTH_SCALE_M, box_distance, pixel_to_point


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        self.declare_parameter("engine_path", "yolo26n.engine")   # .engine on the Jetson; .pt also works (slow)
        self.declare_parameter("conf", 0.4)
        self.declare_parameter("imgsz", 640)                     # must match the size the engine was exported at
        self.declare_parameter("camera_ns", "/camera/camera")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("classes", [""])                  # e.g. ["person"]; empty = all classes

        engine = self.get_parameter("engine_path").value
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        ns = self.get_parameter("camera_ns").value.rstrip("/")
        self.debug = bool(self.get_parameter("publish_debug_image").value)
        self.keep = {c for c in self.get_parameter("classes").value if c}

        from ultralytics import YOLO  # imported here so a missing venv gives a clear error at start-up
        self.get_logger().info(f"loading {engine}")
        self.model = YOLO(engine, task="detect")

        self.bridge = CvBridge()
        self.K = None
        self.create_subscription(CameraInfo, f"{ns}/color/camera_info", self.on_info, qos_profile_sensor_data)
        color = Subscriber(self, Image, f"{ns}/color/image_raw", qos_profile=qos_profile_sensor_data)
        depth = Subscriber(self, Image, f"{ns}/aligned_depth_to_color/image_raw", qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([color, depth], queue_size=2, slop=0.03)
        self.sync.registerCallback(self.on_frames)

        self.pub = self.create_publisher(Detection3DArray, "detections", 10)
        self.debug_pub = self.create_publisher(Image, "detections/image", 1) if self.debug else None
        self.frames, self.t_last_log, self.infer_ms = 0, time.monotonic(), 0.0

    def on_info(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy

    def on_frames(self, color_msg, depth_msg):
        if self.K is None:
            return
        fx, fy, cx, cy = self.K
        img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")  # uint16, millimetres

        t0 = time.perf_counter()
        result = self.model(img, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        self.infer_ms = (time.perf_counter() - t0) * 1000

        out = Detection3DArray(header=color_msg.header)
        for box in result.boxes:
            name = result.names[int(box.cls[0])]
            if self.keep and name not in self.keep:
                continue
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
            z = box_distance(depth, x1, y1, x2, y2, DEPTH_SCALE_M)
            if z is None:
                continue
            x, y, z = pixel_to_point((x1 + x2) / 2, (y1 + y2) / 2, z, fx, fy, cx, cy)
            det = Detection3D(header=color_msg.header)
            det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.center.position.z = x, y, z
            det.bbox.size.x = (x2 - x1) * z / fx   # rough metric width/height at that depth
            det.bbox.size.y = (y2 - y1) * z / fy
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = float(box.conf[0])
            det.results.append(hyp)
            out.detections.append(det)
        self.pub.publish(out)

        if self.debug_pub is not None:
            dbg = self.bridge.cv2_to_imgmsg(result.plot(), "bgr8")
            dbg.header = color_msg.header
            self.debug_pub.publish(dbg)

        self.frames += 1
        now = time.monotonic()
        if now - self.t_last_log >= 5.0:
            fps = self.frames / (now - self.t_last_log)
            self.get_logger().info(f"{fps:.1f} FPS | inference {self.infer_ms:.1f} ms | {len(out.detections)} detections")
            self.frames, self.t_last_log = 0, now


def main():
    rclpy.init()
    node = YoloDetector()
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
