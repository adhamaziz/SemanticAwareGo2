#!/usr/bin/env python3
"""
Phase A YOLO detector: pretrained YOLOv8n (COCO classes, no training),
throttled for CPU-only inference. Watches for a configurable target class
(default: 'fire hydrant') on the D435i RGB feed, uses the aligned depth
image to back-project the detection's center pixel into a 3D point, and
publishes both the raw 2D detections and an estimated map-frame pose for
anything matching the target class.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/d435i/color/image_raw')
        self.declare_parameter('depth_topic', '/d435i/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/d435i/color/camera_info')
        self.declare_parameter('target_class', 'fire hydrant')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('inference_hz', 2.0)
        self.declare_parameter('inference_width', 416)
        self.declare_parameter('target_frame', 'map')

        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.target_class = self.get_parameter('target_class').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.inference_period = 1.0 / self.get_parameter('inference_hz').value
        self.inference_width = self.get_parameter('inference_width').value
        self.target_frame = self.get_parameter('target_frame').value

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # auto-downloads COCO-pretrained weights on first run
        self.get_logger().info(
            f'YOLOv8n loaded. Watching for "{self.target_class}" at {1.0/self.inference_period:.1f} Hz'
        )

        self.latest_depth = None
        self.camera_info = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.det_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/yolo/target_pose', 10)

        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.caminfo_cb, 10)

        self.last_inference_time = self.get_clock().now()
        self.create_subscription(Image, self.image_topic, self.image_cb, 1)

    def depth_cb(self, msg: Image):
        self.latest_depth = msg

    def caminfo_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def image_cb(self, msg: Image):
        now = self.get_clock().now()
        if (now - self.last_inference_time) < Duration(seconds=self.inference_period):
            return  # throttle -- CPU inference is the bottleneck, skip frames between ticks
        self.last_inference_time = now

        if self.camera_info is None:
            return  # need intrinsics before we can back-project anything

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model.predict(
            cv_image, imgsz=self.inference_width, conf=self.conf_thresh, verbose=False
        )
        result = results[0]

        det_array = Detection2DArray()
        det_array.header = msg.header

        best_target = None
        best_conf = 0.0

        for box in result.boxes:
            cls_id = int(box.cls[0])
            class_name = self.model.names[cls_id]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

            det = Detection2D()
            det.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_name
            hyp.hypothesis.score = conf
            det.results.append(hyp)
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            det_array.detections.append(det)

            if class_name == self.target_class and conf > best_conf:
                best_conf = conf
                best_target = (cx, cy)

        self.det_pub.publish(det_array)

        if best_target is not None:
            self.publish_target_pose(best_target, msg.header)

    def publish_target_pose(self, pixel, header):
        if self.latest_depth is None:
            return

        cx, cy = pixel
        depth_img = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
        px, py = int(cx), int(cy)
        if not (0 <= py < depth_img.shape[0] and 0 <= px < depth_img.shape[1]):
            return

        depth_val = float(depth_img[py, px])
        if depth_val <= 0.0 or math.isnan(depth_val) or math.isinf(depth_val):
            return  # invalid depth at that pixel, skip this detection rather than publish garbage

        # Sim depth is typically metres already (float32); adjust /1000.0 here
        # if your depth topic turns out to be raw mm (uint16) instead.
        depth_m = depth_val

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        ppx = self.camera_info.k[2]
        ppy = self.camera_info.k[5]

        x = (px - ppx) * depth_m / fx
        y = (py - ppy) * depth_m / fy
        z = depth_m

        camera_pose = PoseStamped()
        camera_pose.header = header
        camera_pose.pose.position.x = x
        camera_pose.pose.position.y = y
        camera_pose.pose.position.z = z
        camera_pose.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, header.frame_id, header.stamp,
                timeout=Duration(seconds=0.2)
            )
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed for YOLO target pose: {e}')
            return

        from tf2_geometry_msgs import do_transform_pose
        world_pose_geom = do_transform_pose(camera_pose.pose, transform)

        out = PoseStamped()
        out.header.frame_id = self.target_frame
        out.header.stamp = header.stamp
        out.pose = world_pose_geom
        self.pose_pub.publish(out)


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
