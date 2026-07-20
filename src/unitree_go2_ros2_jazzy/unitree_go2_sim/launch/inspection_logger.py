#!/usr/bin/env python3
"""
C4 -- Inspection Logger.

Subscribes to /aruco_detections and writes one JSON record per uniquely
inspected marker to an inspection report file. A marker already logged is
only re-logged if a new detection improves on the best-seen reprojection
confidence significantly (avoids spamming one record per frame while the
robot stands in front of a marker for several seconds).

Output: a single JSON file, list of records:
{
  "marker_id": int,
  "position": {"x": .., "y": .., "z": ..},
  "orientation": {"x": .., "y": .., "z": .., "w": ..},
  "num_observations": int,
  "first_seen_sec": float,
  "last_updated_sec": float
}

Usage:
    python3 inspection_logger.py --ros-args -p output_json:=/home/ros2_ws/src/inspection_report.json
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from aruco_opencv_msgs.msg import ArucoDetection
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose


class InspectionLogger(Node):
    def __init__(self):
        super().__init__('inspection_logger')

        self.declare_parameter('output_json', 'inspection_report.json')
        self.declare_parameter('reinspect_position_threshold_m', 0.05)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter(
            'camera_frame', 'd435i_color_optical_frame'
        )  # NOT msg.header.frame_id -- that reports "go2/base_link/d435i_rgbd",
           # an orphaned name Gazebo substitutes when a sensor's gz_frame_id is
           # invalid (see the "not defined in SDF" warnings in every launch log).
           # It was never wired to a real broadcast TF frame. This one is.

        self.output_path = self.get_parameter('output_json').value
        self.reinspect_thresh = self.get_parameter('reinspect_position_threshold_m').value
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # marker_id -> record dict
        self.records = {}
        self._load_existing()

        self.sub = self.create_subscription(
            ArucoDetection, '/aruco_detections', self.detection_cb, 10
        )
        self.get_logger().info(
            f'Inspection logger writing to {self.output_path}, poses in "{self.target_frame}" frame'
        )

    def _load_existing(self):
        if os.path.exists(self.output_path):
            try:
                with open(self.output_path, 'r') as f:
                    existing = json.load(f)
                for rec in existing:
                    self.records[rec['marker_id']] = rec
                self.get_logger().info(f'Loaded {len(self.records)} existing records')
            except (json.JSONDecodeError, KeyError):
                self.get_logger().warn('Existing inspection report unreadable, starting fresh')

    def detection_cb(self, msg: ArucoDetection):
        now = time.time()

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_frame, msg.header.stamp,
                timeout=Duration(seconds=0.5)
            )
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'Could not transform {self.camera_frame} -> {self.target_frame}: {e}. '
                'Skipping this detection rather than logging a meaningless camera-relative pose.'
            )
            return

        for marker in msg.markers:
            mid = marker.marker_id
            world_pose = do_transform_pose(marker.pose, transform)
            pos = world_pose.position
            orient = world_pose.orientation

            if mid not in self.records:
                self.records[mid] = {
                    'marker_id': mid,
                    'frame': self.target_frame,
                    'position': {'x': pos.x, 'y': pos.y, 'z': pos.z},
                    'orientation': {'x': orient.x, 'y': orient.y, 'z': orient.z, 'w': orient.w},
                    'num_observations': 1,
                    'first_seen_sec': now,
                    'last_updated_sec': now,
                }
                self.get_logger().info(f'NEW marker {mid} logged at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})')
                self._write()
            else:
                rec = self.records[mid]
                dx = pos.x - rec['position']['x']
                dy = pos.y - rec['position']['y']
                dz = pos.z - rec['position']['z']
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)

                rec['num_observations'] += 1
                rec['last_updated_sec'] = now

                if dist > self.reinspect_thresh:
                    self.get_logger().warn(
                        f'Marker {mid} re-observed {dist:.3f}m from previous estimate -- '
                        'averaging in, but check for detection/TF noise if this recurs.'
                    )
                    rec['position'] = {'x': pos.x, 'y': pos.y, 'z': pos.z}
                    rec['orientation'] = {
                        'x': orient.x, 'y': orient.y, 'z': orient.z, 'w': orient.w
                    }
                self._write()

    def _write(self):
        with open(self.output_path, 'w') as f:
            json.dump(list(self.records.values()), f, indent=2)


def main():
    rclpy.init()
    node = InspectionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()