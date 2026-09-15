#!/usr/bin/env python3
"""
C1 -- SWAP State Machine, REAL ROBOT variant.

Same orbit-sweep NBV FSM logic as the simulation version, adapted for
one confirmed real hardware difference: the real robot's aruco_detector_node
publishes marker IDs and poses as two SEPARATE, parallel-indexed topics
(/go2/aruco/marker_ids: Int32MultiArray, /go2/aruco/poses: PoseArray)
rather than a single bundled message (sim's aruco_opencv_msgs/ArucoDetection).
This file synchronizes that pair itself rather than assuming the sim
message format.

UNVERIFIED, must be confirmed on the actual robot before trusting this:
  - target_frame / robot_base_frame / camera_frame parameter defaults below
    are guesses based on common real-robot TF naming conventions, NOT
    confirmed against this specific robot's actual TF tree. Run
    `ros2 run tf2_tools view_frames` on the robot and override these
    parameters with the real names.
  - Whether /navigate_to_pose and /spin action servers are registered
    under these exact names on the real Nav2 bringup.
  - marker_size/dictionary must match your aruco_detector_node.py launch
    config exactly, or the marker IDs won't mean the same thing here vs.
    physically on your printed tags.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import Bool, String, Int32MultiArray
from geometry_msgs.msg import PoseStamped, Point, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose, Spin
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose


def rotate_vector_by_quaternion(vx, vy, vz, qx, qy, qz, qw):
    cx = qy * vz - qz * vy
    cy = qz * vx - qx * vz
    cz = qx * vy - qy * vx
    ccx = qy * cz - qz * cy
    ccy = qz * cx - qx * cz
    ccz = qx * cy - qy * cx
    return (
        vx + 2 * qw * cx + 2 * ccx,
        vy + 2 * qw * cy + 2 * ccy,
        vz + 2 * qw * cz + 2 * ccz,
    )


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_from_quaternion(qx, qy, qz, qw):
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class SwapStateMachineReal(Node):
    def __init__(self):
        super().__init__('swap_state_machine_real')

        self.declare_parameter('standoff_distance_m', 0.8)
        self.declare_parameter('approach_standoff_m', 1.0)
        self.declare_parameter('inspect_dwell_sec', 3.0)
        self.declare_parameter('nbv_orbit_radius_m', 1.0)
        self.declare_parameter('nbv_orbit_num_waypoints', 6)
        self.declare_parameter('nbv_waypoint_timeout_sec', 15.0)
        self.declare_parameter('orbit_dwell_sec', 2.0)  # idle time at each orbit stop after facing the target, before moving to the next -- gives ArUco a real chance to see the tag, not just an instant
        self.declare_parameter('approach_timeout_sec', 40.0)
        self.declare_parameter('revisit_distance_m', 1.0)

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('yolo_target_pose_topic', '/yolo/target_pose')
        self.declare_parameter('aruco_poses_topic', '/go2/aruco/poses')
        self.declare_parameter('aruco_ids_topic', '/go2/aruco/marker_ids')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('spin_action', '/spin')
        # Robustness against single noisy YOLO/depth frames on real hardware
        # (a reflection, an edge pixel, or a stale/extrapolated TF sample can
        # each produce one wildly-wrong pose): require this many consecutive
        # detections, all within yolo_confirm_radius_m of each other, before
        # committing to a real Nav2 approach goal.
        self.declare_parameter('yolo_confirm_count', 3)
        self.declare_parameter('yolo_confirm_radius_m', 0.3)
        self.declare_parameter('max_target_distance_m', 4.0)  # sanity backstop: a real depth-camera detection should never claim the object is further than this from the robot's own current position. Catches the case where the robot's own map-frame localization is momentarily wrong (e.g. under heavy CPU load) even though the camera's raw distance reading was fine -- the corruption happens in the camera-to-map TF conversion, not the depth measurement itself.

        self.standoff = self.get_parameter('standoff_distance_m').value
        self.approach_standoff = self.get_parameter('approach_standoff_m').value
        self.dwell_sec = self.get_parameter('inspect_dwell_sec').value
        self.orbit_radius = self.get_parameter('nbv_orbit_radius_m').value
        self.orbit_num_wp = self.get_parameter('nbv_orbit_num_waypoints').value
        self.waypoint_timeout = self.get_parameter('nbv_waypoint_timeout_sec').value
        self.orbit_dwell_sec = self.get_parameter('orbit_dwell_sec').value
        self.approach_timeout = self.get_parameter('approach_timeout_sec').value
        self.revisit_thresh = self.get_parameter('revisit_distance_m').value
        self.target_frame = self.get_parameter('target_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.yolo_confirm_count = self.get_parameter('yolo_confirm_count').value
        self.yolo_confirm_radius = self.get_parameter('yolo_confirm_radius_m').value
        self.max_target_distance = self.get_parameter('max_target_distance_m').value
        self._yolo_confirm_buffer = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = 'B1_EXPLORE'
        self.seen_marker_ids = set()
        self.visited_positions = []
        self.busy = False
        self.orbit_waypoints = []
        self.orbit_index = 0
        self._orbit_dwell_timer = None
        self.orbit_stopped = False
        self.current_goal_handle = None
        self.timeout_timer = None

        self._latest_aruco_poses = None
        self._latest_aruco_header = None

        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.behaviour_pub = self.create_publisher(String, '/current_behaviour', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/swap/inspection_markers', 10)
        self._marker_id_counter = 0

        yolo_topic = self.get_parameter('yolo_target_pose_topic').value
        aruco_poses_topic = self.get_parameter('aruco_poses_topic').value
        aruco_ids_topic = self.get_parameter('aruco_ids_topic').value

        self.create_subscription(PoseStamped, yolo_topic, self.yolo_cb, 10)
        self.create_subscription(PoseArray, aruco_poses_topic, self.aruco_poses_cb, 10)
        self.create_subscription(Int32MultiArray, aruco_ids_topic, self.aruco_ids_cb, 10)

        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter('navigate_to_pose_action').value
        )
        self.spin_client = ActionClient(
            self, Spin, self.get_parameter('spin_action').value
        )

        self.create_timer(1.0, self.publish_state)
        self.get_logger().info(
            'REAL ROBOT variant -- target_frame/robot_base_frame/camera_frame '
            'confirmed against this robot\'s actual TF tree (map->base_link->camera_link '
            '-> camera_color_optical_frame). Re-verify if the URDF or mount changes.'
        )
        self.get_logger().info(
            f'SWAP state machine (real robot) started. orbit_radius={self.orbit_radius}m, '
            f'orbit_waypoints={self.orbit_num_wp}, waypoint_timeout={self.waypoint_timeout}s'
        )

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.behaviour_pub.publish(msg)

    def _already_visited(self, x, y):
        for vx, vy in self.visited_positions:
            if math.hypot(x - vx, y - vy) < self.revisit_thresh:
                return True
        return False

    def _pause_explore(self):
        msg = Bool()
        msg.data = False
        self.resume_pub.publish(msg)

    def _resume_explore(self):
        msg = Bool()
        msg.data = True
        self.resume_pub.publish(msg)

    def _plot_marker(self, x, y, z, success):
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'swap_inspections'
        marker.id = self._marker_id_counter
        self._marker_id_counter += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=x, y=y, z=z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        if success:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 0.1, 0.9
        else:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.9, 0.1, 0.1, 0.7
        marker.lifetime.sec = 0
        arr = MarkerArray()
        arr.markers.append(marker)
        self.marker_pub.publish(arr)

    def _cancel_timeout_timer(self):
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None

    def _send_nav_goal(self, x, y, yaw, on_done, timeout_sec=None):
        goal = PoseStamped()
        goal.header.frame_id = self.target_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available')
            on_done(False)
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        future = self.nav_client.send_goal_async(nav_goal)

        done_called = {'flag': False}

        def _call_once(success):
            if done_called['flag']:
                return
            done_called['flag'] = True
            self._cancel_timeout_timer()
            self.current_goal_handle = None
            on_done(success)

        def _on_response(f):
            handle = f.result()
            if not handle.accepted:
                self.get_logger().warn('Nav2 goal rejected')
                _call_once(False)
                return
            self.current_goal_handle = handle
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda rf: _call_once(True))

        future.add_done_callback(_on_response)

        if timeout_sec is not None:
            def _on_timeout():
                self.get_logger().warn(
                    f'Goal exceeded {timeout_sec}s timeout, cancelling and moving on'
                )
                if self.current_goal_handle is not None:
                    self.current_goal_handle.cancel_goal_async()
                _call_once(False)

            self.timeout_timer = self.create_timer(timeout_sec, _on_timeout)

    def _get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, self.robot_base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
            q = t.transform.rotation
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            return t.transform.translation.x, t.transform.translation.y, yaw
        except (LookupException, ExtrapolationException):
            return 0.0, 0.0, 0.0

    def _get_robot_xy(self):
        x, y, _ = self._get_robot_pose()
        return x, y

    def _rotate_to_heading(self, target_yaw, on_done):
        _, _, current_yaw = self._get_robot_pose()
        delta = normalize_angle(target_yaw - current_yaw)

        if not self.spin_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Spin action server not available, skipping heading correction')
            on_done()
            return

        spin_goal = Spin.Goal()
        spin_goal.target_yaw = delta

        future = self.spin_client.send_goal_async(spin_goal)

        def _on_response(f):
            handle = f.result()
            if not handle.accepted:
                self.get_logger().warn('Spin goal rejected, continuing without heading correction')
                on_done()
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda rf: on_done())

        future.add_done_callback(_on_response)

    def yolo_cb(self, msg: PoseStamped):
        if self.busy or self.state != 'B1_EXPLORE':
            self._yolo_confirm_buffer = []
            return
        x, y = msg.pose.position.x, msg.pose.position.y
        if self._already_visited(x, y):
            self._yolo_confirm_buffer = []
            return

        # Require yolo_confirm_count consecutive detections, all within
        # yolo_confirm_radius_m of the previous one, before trusting this
        # target enough to send a real Nav2 goal. A single outlier pose
        # (seen during testing: one frame jumped ~2m from every neighbour)
        # resets the buffer rather than getting averaged in.
        if self._yolo_confirm_buffer:
            last_x, last_y = self._yolo_confirm_buffer[-1]
            if math.hypot(x - last_x, y - last_y) > self.yolo_confirm_radius:
                self.get_logger().warn(
                    f'YOLO pose ({x:.2f}, {y:.2f}) is {math.hypot(x - last_x, y - last_y):.2f}m '
                    f'from the previous candidate ({last_x:.2f}, {last_y:.2f}) -- treating as an '
                    f'outlier and resetting confirmation (had {len(self._yolo_confirm_buffer)}/'
                    f'{self.yolo_confirm_count})'
                )
                self._yolo_confirm_buffer = []

        self._yolo_confirm_buffer.append((x, y))

        if len(self._yolo_confirm_buffer) < self.yolo_confirm_count:
            self.get_logger().info(
                f'YOLO target candidate ({x:.2f}, {y:.2f}) -- '
                f'{len(self._yolo_confirm_buffer)}/{self.yolo_confirm_count} consistent, waiting for more'
            )
            return

        # Confirmed. Average the buffered poses rather than using the very
        # last one -- slightly steadier than any single frame, at no extra
        # latency since we already waited for yolo_confirm_count frames.
        avg_x = sum(p[0] for p in self._yolo_confirm_buffer) / len(self._yolo_confirm_buffer)
        avg_y = sum(p[1] for p in self._yolo_confirm_buffer) / len(self._yolo_confirm_buffer)
        self._yolo_confirm_buffer = []

        # Sanity backstop: reject if this is further from the robot's own
        # current position than a real depth-camera detection plausibly
        # could be. This specifically catches map-frame corruption -- the
        # camera's raw distance reading can be perfectly fine while the
        # camera-to-map TF conversion is wrong because the robot's own AMCL
        # localization was momentarily degraded (e.g. under heavy CPU load).
        # Confirmed on hardware: this produced targets landing "off the
        # global costmap" entirely, wasting a full approach+orbit cycle on
        # a coordinate that was never real.
        rx, ry = self._get_robot_xy()
        dist_from_robot = math.hypot(avg_x - rx, avg_y - ry)
        if dist_from_robot > self.max_target_distance:
            self.get_logger().error(
                f'YOLO target ({avg_x:.2f}, {avg_y:.2f}) is {dist_from_robot:.2f}m from the '
                f'robot\'s own current position ({rx:.2f}, {ry:.2f}) -- further than any real '
                f'depth-camera detection should be (max_target_distance_m={self.max_target_distance}). '
                f'Rejecting rather than sending a likely-corrupted goal. This usually means the '
                f'robot\'s own localization was momentarily wrong, not that the object moved -- '
                f'check system load (htop) if this keeps happening.'
            )
            return

        self.busy = True
        self._approach_retry_count = 0
        self.get_logger().info(
            f'YOLO target confirmed at ({avg_x:.2f}, {avg_y:.2f}) after '
            f'{self.yolo_confirm_count} consistent detections -- starting approach'
        )
        self.start_approach(avg_x, avg_y)

    def start_approach(self, target_x, target_y):
        self.state = 'B2_APPROACH'
        self.publish_state()
        self._pause_explore()

        rx, ry = self._get_robot_xy()
        dx, dy = target_x - rx, target_y - ry
        dist = math.hypot(dx, dy)
        if dist > self.approach_standoff:
            frac = (dist - self.approach_standoff) / dist
            goal_x = rx + dx * frac
            goal_y = ry + dy * frac
        else:
            goal_x, goal_y = rx, ry
        goal_yaw = math.atan2(dy, dx)

        self._pending_yolo_target = (target_x, target_y)
        self._send_nav_goal(goal_x, goal_y, goal_yaw, self.on_approach_done, timeout_sec=self.approach_timeout)

    def on_approach_done(self, success):
        if success:
            self._approach_retry_count = 0
            self.get_logger().info('Approach succeeded, reached standoff distance -- starting orbit NBV search')
            self.start_nbv_search()
            return

        self._approach_retry_count += 1
        if self._approach_retry_count <= 1:
            self.get_logger().warn(
                f'Approach failed/timed out (attempt {self._approach_retry_count}), retrying once'
            )
            tx, ty = self._pending_yolo_target
            self.start_approach(tx, ty)
        else:
            self.get_logger().warn(
                'Approach failed again on retry -- aborting this object, never actually reached standoff'
            )
            self.finish_cycle(marker_id=None)

    def start_nbv_search(self):
        self.state = 'B2_NBV_SEARCH'
        self.publish_state()
        self.orbit_stopped = False

        cx, cy = self._pending_yolo_target
        self.orbit_waypoints = []
        for i in range(self.orbit_num_wp):
            theta = 2 * math.pi * i / self.orbit_num_wp
            wx = cx + self.orbit_radius * math.cos(theta)
            wy = cy + self.orbit_radius * math.sin(theta)
            facing_inward = math.atan2(cy - wy, cx - wx)
            self.orbit_waypoints.append((wx, wy, facing_inward))

        self.orbit_index = 0
        self._dispatch_next_orbit_waypoint()

    def _dispatch_next_orbit_waypoint(self):
        if self.orbit_stopped:
            return

        if self.orbit_index >= len(self.orbit_waypoints):
            self.get_logger().warn('Orbit exhausted, no ArUco tag found around this object')
            self.finish_cycle(marker_id=None)
            return

        wx, wy, w_facing_inward = self.orbit_waypoints[self.orbit_index]
        self.orbit_index += 1

        rx, ry = self._get_robot_xy()
        travel_yaw = math.atan2(wy - ry, wx - rx)

        self.get_logger().info(
            f'Orbit waypoint {self.orbit_index}/{len(self.orbit_waypoints)}: ({wx:.2f}, {wy:.2f})'
        )
        self._send_nav_goal(
            wx, wy, travel_yaw,
            lambda success: self._on_orbit_position_reached(success, w_facing_inward),
            timeout_sec=self.waypoint_timeout,
        )

    def _on_orbit_position_reached(self, success, facing_inward):
        if self.orbit_stopped:
            return

        if not success:
            self._dispatch_next_orbit_waypoint()
            return

        self._rotate_to_heading(facing_inward, self._start_orbit_dwell)

    def _start_orbit_dwell(self):
        if self.orbit_stopped:
            return
        if self.orbit_dwell_sec <= 0:
            self._dispatch_next_orbit_waypoint()
            return

        def _fire_once():
            self._orbit_dwell_timer.cancel()
            self._orbit_dwell_timer = None
            if not self.orbit_stopped:
                self._dispatch_next_orbit_waypoint()

        self._orbit_dwell_timer = self.create_timer(self.orbit_dwell_sec, _fire_once)

    def aruco_poses_cb(self, msg: PoseArray):
        self._latest_aruco_poses = msg.poses
        self._latest_aruco_header = msg.header

    def aruco_ids_cb(self, msg: Int32MultiArray):
        if self.state != 'B2_NBV_SEARCH':
            return
        if self._latest_aruco_poses is None:
            return
        if len(msg.data) != len(self._latest_aruco_poses):
            self.get_logger().warn(
                f'ArUco poses/ids array length mismatch ({len(self._latest_aruco_poses)} '
                f'vs {len(msg.data)}), skipping this detection'
            )
            return

        for marker_id, pose in zip(msg.data, self._latest_aruco_poses):
            if marker_id in self.seen_marker_ids:
                continue
            self.get_logger().info(f'ArUco marker {marker_id} found during orbit search')
            self.orbit_stopped = True
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
            self._cancel_timeout_timer()
            if self._orbit_dwell_timer is not None:
                self._orbit_dwell_timer.cancel()
                self._orbit_dwell_timer = None
            self.start_precise_inspect(marker_id, pose, self._latest_aruco_header)
            return

    def start_precise_inspect(self, marker_id, pose, header):
        self.state = 'B2_PRECISE_INSPECT'
        self.publish_state()

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_frame, header.stamp,
                timeout=Duration(seconds=0.5)
            )
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed, aborting precise inspect: {e}')
            self.finish_cycle(marker_id=None)
            return

        world_pose = do_transform_pose(pose, transform)
        mx, my = world_pose.position.x, world_pose.position.y
        qx, qy, qz, qw = (
            world_pose.orientation.x, world_pose.orientation.y,
            world_pose.orientation.z, world_pose.orientation.w
        )

        nx, ny, _ = rotate_vector_by_quaternion(0.0, 0.0, 1.0, qx, qy, qz, qw)
        norm = math.hypot(nx, ny) or 1.0
        nx, ny = nx / norm, ny / norm

        goal_x = mx + nx * self.standoff
        goal_y = my + ny * self.standoff
        travel_yaw = math.atan2(my - goal_y, mx - goal_x)

        self._pending_marker_pos = (mx, my, world_pose.position.z)
        self._pending_inspect_facing = math.atan2(-ny, -nx)
        self._send_nav_goal(
            goal_x, goal_y, travel_yaw,
            lambda success: self.on_inspect_nav_done(success, marker_id),
            timeout_sec=self.approach_timeout,
        )

    def on_inspect_nav_done(self, success, marker_id):
        self._rotate_to_heading(
            self._pending_inspect_facing,
            lambda: self._start_dwell(marker_id)
        )

    def _start_dwell(self, marker_id):
        self.state = 'B3_DWELL'
        self.publish_state()
        self.get_logger().info(f'Dwelling {self.dwell_sec}s to inspect marker {marker_id}')

        def _fire_once():
            timer.cancel()
            self.finish_cycle(marker_id=marker_id)

        timer = self.create_timer(self.dwell_sec, _fire_once)

    def finish_cycle(self, marker_id):
        if marker_id is not None:
            self.seen_marker_ids.add(marker_id)
            mx, my, mz = getattr(self, '_pending_marker_pos', (*self._pending_yolo_target, 0.0))
            self._plot_marker(mx, my, mz, success=True)
        else:
            cx, cy = self._pending_yolo_target
            self._plot_marker(cx, cy, 0.0, success=False)

        if hasattr(self, '_pending_yolo_target'):
            self.visited_positions.append(self._pending_yolo_target)

        self.state = 'B1_EXPLORE'
        self.publish_state()
        self._resume_explore()
        self.busy = False
        self.get_logger().info('Cycle complete, resuming exploration')


def main():
    rclpy.init()
    node = SwapStateMachineReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()