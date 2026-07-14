#!/usr/bin/env python3
"""
C1 -- SWAP State Machine (ArUco-triggered variant, no YOLO gate yet).

States: B1_EXPLORE <-> B2_APPROACH_INSPECT

On a newly-seen ArUco marker: pause explore_lite (/explore/resume: False),
compute a simple standoff viewpoint (0.8m along the marker's outward
normal -- matches the thesis's R(v,m) range-score Gaussian center),
send that as a NavigateToPose goal (preempts whatever explore_lite had
in flight), hold briefly for a few clean detections, mark the marker
seen, resume exploration.

Publishes /current_behaviour (std_msgs/String) for RViz/logging visibility.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from aruco_opencv_msgs.msg import ArucoDetection
from explore_lite_msgs.msg import ExploreStatus
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose


def rotate_vector_by_quaternion(vx, vy, vz, qx, qy, qz, qw):
    """Rotate vector v by quaternion q (standard v' = q * v * q^-1, expanded)."""
    # Using the standard formula: v' = v + 2*qw*(qv x v) + 2*(qv x (qv x v))
    qvx, qvy, qvz = qx, qy, qz
    # qv x v
    cx = qvy * vz - qvz * vy
    cy = qvz * vx - qvx * vz
    cz = qvx * vy - qvy * vx
    # qv x (qv x v)
    ccx = qvy * cz - qvz * cy
    ccy = qvz * cx - qvx * cz
    ccz = qvx * cy - qvy * cx
    rx = vx + 2 * qw * cx + 2 * ccx
    ry = vy + 2 * qw * cy + 2 * ccy
    rz = vz + 2 * qw * cz + 2 * ccz
    return rx, ry, rz


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SwapStateMachine(Node):
    def __init__(self):
        super().__init__('swap_state_machine')

        self.declare_parameter('standoff_distance_m', 0.8)
        self.declare_parameter('inspect_dwell_sec', 3.0)
        self.declare_parameter('target_frame', 'map')

        self.standoff = self.get_parameter('standoff_distance_m').value
        self.dwell_sec = self.get_parameter('inspect_dwell_sec').value
        self.target_frame = self.get_parameter('target_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = 'B1_EXPLORE'
        self.seen_marker_ids = set()
        self.explore_status = None
        self.busy = False  # true while a B2/B3 cycle is in progress

        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.behaviour_pub = self.create_publisher(String, '/current_behaviour', 10)

        self.create_subscription(ArucoDetection, '/aruco_detections', self.aruco_cb, 10)
        self.create_subscription(ExploreStatus, '/explore/status', self.explore_status_cb, 10)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.publish_state_timer = self.create_timer(1.0, self.publish_state)

        self.get_logger().info(
            f'SWAP state machine started. standoff={self.standoff}m, dwell={self.dwell_sec}s'
        )

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.behaviour_pub.publish(msg)

    def explore_status_cb(self, msg: ExploreStatus):
        self.explore_status = msg.status

    def aruco_cb(self, msg: ArucoDetection):
        if self.busy or self.state != 'B1_EXPLORE':
            return  # already mid-inspection, ignore new detections until done

        for marker in msg.markers:
            if marker.marker_id in self.seen_marker_ids:
                continue

            # Found a genuinely new marker -- trigger B2/B3.
            self.busy = True
            self.get_logger().info(f'New marker {marker.marker_id} detected -- starting inspection')
            self.start_inspection(marker, msg.header)
            return  # handle one new marker per callback; rest will be picked up next cycle

    def start_inspection(self, marker, header):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, header.frame_id, header.stamp,
                timeout=Duration(seconds=0.2)
            )
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed, aborting this inspection attempt: {e}')
            self.busy = False
            return

        world_pose = do_transform_pose(marker.pose, transform)
        mx, my, mz = world_pose.position.x, world_pose.position.y, world_pose.position.z
        qx = world_pose.orientation.x
        qy = world_pose.orientation.y
        qz = world_pose.orientation.z
        qw = world_pose.orientation.w

        # Marker's local +Z axis rotated into world frame = outward normal
        nx, ny, nz = rotate_vector_by_quaternion(0.0, 0.0, 1.0, qx, qy, qz, qw)
        norm = math.sqrt(nx * nx + ny * ny) or 1.0  # guard against a purely-vertical normal
        nx, ny = nx / norm, ny / norm  # flatten to ground plane for a drivable goal

        goal_x = mx + nx * self.standoff
        goal_y = my + ny * self.standoff
        # Face back toward the marker (opposite of the outward normal)
        goal_yaw = math.atan2(-ny, -nx)

        self.state = 'B2_APPROACH_INSPECT'
        self.publish_state()

        resume_msg = Bool()
        resume_msg.data = False
        self.resume_pub.publish(resume_msg)

        goal = PoseStamped()
        goal.header.frame_id = self.target_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        qx2, qy2, qz2, qw2 = yaw_to_quaternion(goal_yaw)
        goal.pose.orientation.x = qx2
        goal.pose.orientation.y = qy2
        goal.pose.orientation.z = qz2
        goal.pose.orientation.w = qw2

        self.get_logger().info(
            f'Marker {marker.marker_id}: standoff goal ({goal_x:.2f}, {goal_y:.2f}), yaw={goal_yaw:.2f}'
        )

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available, aborting inspection')
            self.finish_inspection(marker.marker_id)
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        future = self.nav_client.send_goal_async(nav_goal)
        future.add_done_callback(lambda f: self.on_goal_response(f, marker.marker_id))

    def on_goal_response(self, future, marker_id):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Inspection goal for marker {marker_id} rejected by Nav2')
            self.finish_inspection(marker_id)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.on_goal_result(f, marker_id))

    def on_goal_result(self, future, marker_id):
        # Regardless of exact success/failure: hold briefly so
        # aruco_opencv + the inspection logger get a few clean
        # observations, then mark inspected and resume exploring.
        self.get_logger().info(
            f'Nav2 goal for marker {marker_id} finished, dwelling {self.dwell_sec}s to inspect'
        )

        def _fire_once():
            timer.cancel()
            self.finish_inspection(marker_id)

        timer = self.create_timer(self.dwell_sec, _fire_once)

    def finish_inspection(self, marker_id):
        self.seen_marker_ids.add(marker_id)
        self.state = 'B1_EXPLORE'
        self.publish_state()

        resume_msg = Bool()
        resume_msg.data = True
        self.resume_pub.publish(resume_msg)

        self.busy = False
        self.get_logger().info(f'Marker {marker_id} inspection complete, resuming exploration')


def main():
    rclpy.init()
    node = SwapStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
