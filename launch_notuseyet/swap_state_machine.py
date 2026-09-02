#!/usr/bin/env python3
"""
C1 -- SWAP State Machine, YOLO-gated version (matches the flowchart).

B1_EXPLORE -> (YOLO target_pose) -> B2_APPROACH -> B2_NBV_SEARCH
  -> (ArUco found) -> B2_PRECISE_INSPECT -> B3_DWELL -> back to B1_EXPLORE
  -> (ArUco not found within timeout) -> back to B1_EXPLORE

B2_NBV_SEARCH is a simplified stand-in for the full NBV viewpoint-scoring
(C3, Q(v,m) from the thesis roadmap) -- it rotates in place scanning for
the tag rather than sampling/scoring candidate viewpoints. Swap this
state's implementation for real NBV later without touching the rest of
the FSM.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from aruco_opencv_msgs.msg import ArucoDetection
from explore_lite_msgs.msg import ExploreStatus
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose


def rotate_vector_by_quaternion(vx, vy, vz, qx, qy, qz, qw):
    qvx, qvy, qvz = qx, qy, qz
    cx = qvy * vz - qvz * vy
    cy = qvz * vx - qvx * vz
    cz = qvx * vy - qvy * vx
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

        self.declare_parameter('standoff_distance_m', 1.0)
        self.declare_parameter('approach_standoff_m', 1.3)  # YOLO-triggered approach: stop short of the raw target
        self.declare_parameter('inspect_dwell_sec', 3.0)
        self.declare_parameter('nbv_search_timeout_sec', 25.0)  # was 15.0 -- see fix note below
        self.declare_parameter('nbv_search_angular_vel', 0.3)
        self.declare_parameter('revisit_distance_m', 1.0)  # ignore new YOLO targets this close to an already-visited one
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('camera_frame', 'd435i_color_optical_frame')

        self.standoff = self.get_parameter('standoff_distance_m').value
        self.approach_standoff = self.get_parameter('approach_standoff_m').value
        self.dwell_sec = self.get_parameter('inspect_dwell_sec').value
        self.nbv_timeout = self.get_parameter('nbv_search_timeout_sec').value
        self.nbv_angular_vel = self.get_parameter('nbv_search_angular_vel').value
        self.revisit_thresh = self.get_parameter('revisit_distance_m').value
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = 'B1_EXPLORE'
        self.seen_marker_ids = set()
        self.visited_positions = []  # list of (x, y) already approached via YOLO
        self.busy = False
        self.nbv_timer = None
        self.nbv_search_timer = None
        self.current_robot_pose = None

        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.behaviour_pub = self.create_publisher(String, '/current_behaviour', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(PoseStamped, '/yolo/target_pose', self.yolo_cb, 10)
        self.create_subscription(ArucoDetection, '/aruco_detections', self.aruco_cb, 10)
        self.create_subscription(ExploreStatus, '/explore/status', self.explore_status_cb, 10)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.create_timer(1.0, self.publish_state)
        self.get_logger().info(
            f'SWAP state machine (YOLO-gated) started. approach_standoff={self.approach_standoff}m, '
            f'nbv_timeout={self.nbv_timeout}s'
        )

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.behaviour_pub.publish(msg)

    def explore_status_cb(self, msg: ExploreStatus):
        pass  # available if needed later; not required for the current transitions

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

    def _send_nav_goal(self, x, y, yaw, on_done):
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

        def _on_response(f):
            handle = f.result()
            if not handle.accepted:
                self.get_logger().warn('Nav2 goal rejected')
                on_done(False)
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda rf: on_done(True))

        future.add_done_callback(_on_response)

    # ---------------- B1: waiting for a YOLO trigger ----------------

    def yolo_cb(self, msg: PoseStamped):
        if self.busy or self.state != 'B1_EXPLORE':
            return

        x, y = msg.pose.position.x, msg.pose.position.y
        if self._already_visited(x, y):
            return

        self.busy = True
        self.get_logger().info(f'YOLO target at ({x:.2f}, {y:.2f}) -- starting approach')
        self.start_approach(x, y)

    # ---------------- B2a: approach the general YOLO target ----------------

    def start_approach(self, target_x, target_y):
        self.state = 'B2_APPROACH'
        self.publish_state()
        self._pause_explore()

        # Stop short of the raw YOLO position by approach_standoff, along
        # the line from the robot's current pose toward the target -- a
        # generic standoff since we don't yet know a specific marker's
        # facing direction at this stage.
        rx, ry = self._get_robot_xy()
        dx, dy = target_x - rx, target_y - ry
        dist = math.hypot(dx, dy)
        if dist > self.approach_standoff:
            frac = (dist - self.approach_standoff) / dist
            goal_x = rx + dx * frac
            goal_y = ry + dy * frac
        else:
            goal_x, goal_y = rx, ry  # already close enough, don't overshoot into the object
        goal_yaw = math.atan2(dy, dx)

        self._pending_yolo_target = (target_x, target_y)
        self._send_nav_goal(goal_x, goal_y, goal_yaw, self.on_approach_done)

    def _get_robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, 'base_footprint', rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
            return t.transform.translation.x, t.transform.translation.y
        except (LookupException, ExtrapolationException):
            return 0.0, 0.0  # fallback; should be rare once TF is settled

    def on_approach_done(self, success):
        self.get_logger().info(f'Approach goal finished (success={success}), starting NBV search')
        self.start_nbv_search()

    # ---------------- B2b: simplified NBV -- rotate-search for ArUco ----------------

    def start_nbv_search(self):
        self.state = 'B2_NBV_SEARCH'
        self.publish_state()

        twist = Twist()
        twist.angular.z = self.nbv_angular_vel
        self.nbv_timer = self.create_timer(0.2, lambda: self.cmd_vel_pub.publish(twist))

        def _timeout():
            self.get_logger().warn('NBV search timed out, no ArUco tag found near this target')
            self._stop_nbv_search()
            self.finish_cycle(marker_id=None)

        self.nbv_search_timer = self.create_timer(self.nbv_timeout, _timeout)

    def _stop_nbv_search(self):
        if self.nbv_timer is not None:
            self.nbv_timer.cancel()
            self.nbv_timer = None
        if self.nbv_search_timer is not None:
            self.nbv_search_timer.cancel()
            self.nbv_search_timer = None
        self.cmd_vel_pub.publish(Twist())  # zero velocity, stop rotating

    def aruco_cb(self, msg: ArucoDetection):
        if self.state != 'B2_NBV_SEARCH':
            return  # inspection_logger.py handles passive/opportunistic logging separately

        for marker in msg.markers:
            if marker.marker_id in self.seen_marker_ids:
                continue
            self.get_logger().info(f'ArUco marker {marker.marker_id} found during NBV search')
            self._stop_nbv_search()
            self.start_precise_inspect(marker, msg.header)
            return

    # ---------------- B3: standoff + inspect the confirmed marker ----------------

    def start_precise_inspect(self, marker, header):
        self.state = 'B2_PRECISE_INSPECT'
        self.publish_state()

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_frame, header.stamp,
                timeout=Duration(seconds=0.2)
            )
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed, aborting precise inspect: {e}')
            self.finish_cycle(marker_id=None)
            return

        world_pose = do_transform_pose(marker.pose, transform)
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
        goal_yaw = math.atan2(-ny, -nx)

        marker_id = marker.marker_id
        self._send_nav_goal(
            goal_x, goal_y, goal_yaw,
            lambda success: self.on_inspect_nav_done(success, marker_id)
        )

    def on_inspect_nav_done(self, success, marker_id):
        self.state = 'B3_DWELL'
        self.publish_state()
        self.get_logger().info(f'Dwelling {self.dwell_sec}s to inspect marker {marker_id}')

        def _fire_once():
            timer.cancel()
            self.finish_cycle(marker_id=marker_id)

        timer = self.create_timer(self.dwell_sec, _fire_once)

    # ---------------- back to B1 ----------------

    def finish_cycle(self, marker_id):
        if marker_id is not None:
            self.seen_marker_ids.add(marker_id)
        if hasattr(self, '_pending_yolo_target'):
            self.visited_positions.append(self._pending_yolo_target)

        self.state = 'B1_EXPLORE'
        self.publish_state()
        self._resume_explore()
        self.busy = False
        self.get_logger().info('Cycle complete, resuming exploration')


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
