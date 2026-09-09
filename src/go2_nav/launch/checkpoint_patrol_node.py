#!/usr/bin/env python3
"""
Phase 2 replacement for frontier exploration. Once you have a saved,
static map (Phase 1), there's no more "unknown space" left for a frontier
explorer to chase -- it would just sit idle. This node instead cycles
through a fixed list of checkpoints on the known map via NavigateToPose,
so YOLO/ArUco/the SWAP FSM have something driving the robot around to scan
for the target.

Honors the same /explore/resume Bool convention swap_state_machine_real.py
already publishes (False=pause, True=resume) -- so when the FSM finds a
target and pauses exploration, this patrol actually stops sending new
goals and cancels whatever's in flight, instead of fighting the FSM's own
approach goal for control of the robot.
"""
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

from swap_state_machine_real import yaw_to_quaternion  # reuse existing helper


class CheckpointPatrolNode(Node):
    def __init__(self):
        super().__init__('checkpoint_patrol_node')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('random_order', True)
        self.declare_parameter('goal_timeout_sec', 60.0)
        # Flattened x,y,yaw_deg triples, e.g. "1.0,2.0,0.0,3.5,-1.0,90.0".
        # Easiest way to get real coordinates: use RViz's "Publish Point"
        # tool on the saved map and read them off `ros2 topic echo
        # /clicked_point`, or just note down 2D Pose Estimate clicks.
        self.declare_parameter('checkpoints', '')

        self.target_frame = self.get_parameter('target_frame').value
        self.random_order = self.get_parameter('random_order').value
        self.goal_timeout = self.get_parameter('goal_timeout_sec').value

        raw = self.get_parameter('checkpoints').value
        self.checkpoints = self._parse_checkpoints(raw)
        if not self.checkpoints:
            self.get_logger().error(
                'No checkpoints configured -- set the "checkpoints" parameter '
                '(flattened x,y,yaw_deg triples). Patrol will not start.'
            )

        self.paused = False
        self.current_goal_handle = None
        self.checkpoint_index = 0
        self.timeout_timer = None

        self.create_subscription(Bool, '/explore/resume', self._resume_cb, 10)

        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter('navigate_to_pose_action').value
        )

        self.get_logger().info(
            f'Checkpoint patrol started. {len(self.checkpoints)} checkpoints, '
            f'random_order={self.random_order}'
        )

        # Kick off the first goal once the action server is ready, rather
        # than blocking __init__ on wait_for_server (which would delay
        # this node coming up and publishing anything at all).
        self._startup_timer_handle = self.create_timer(1.0, self._startup_check)

    def _startup_check(self):
        if self.nav_client.server_is_ready():
            self._startup_timer_handle.cancel()
            self._send_next_checkpoint()
        # else: keep polling on the 1s timer until the server is up

    def _parse_checkpoints(self, raw: str):
        if not raw.strip():
            return []
        vals = [float(v) for v in raw.split(',')]
        if len(vals) % 3 != 0:
            self.get_logger().error(
                f'checkpoints param has {len(vals)} values, not a multiple of 3 '
                '(x,y,yaw_deg per checkpoint) -- ignoring all of them.'
            )
            return []
        return [tuple(vals[i:i + 3]) for i in range(0, len(vals), 3)]

    def _resume_cb(self, msg: Bool):
        if msg.data and self.paused:
            self.paused = False
            self.get_logger().info('Patrol resumed')
            self._send_next_checkpoint()
        elif not msg.data and not self.paused:
            self.paused = True
            self.get_logger().info('Patrol paused (FSM took over)')
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
            self._cancel_timeout_timer()

    def _cancel_timeout_timer(self):
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None

    def _next_checkpoint(self):
        if not self.checkpoints:
            return None
        if self.random_order:
            return random.choice(self.checkpoints)
        cp = self.checkpoints[self.checkpoint_index % len(self.checkpoints)]
        self.checkpoint_index += 1
        return cp

    def _send_next_checkpoint(self):
        if self.paused or not self.checkpoints:
            return

        cp = self._next_checkpoint()
        if cp is None:
            return
        x, y, yaw_deg = cp

        if not self.nav_client.server_is_ready():
            self.get_logger().warn('NavigateToPose server not ready, retrying shortly')
            self.create_timer(2.0, self._send_next_checkpoint)
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.target_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(math.radians(yaw_deg))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f'Patrol heading to checkpoint ({x:.2f}, {y:.2f}, {yaw_deg:.0f}deg)')

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Patrol goal rejected, trying another checkpoint')
            self._send_next_checkpoint()
            return

        self.current_goal_handle = handle
        self._cancel_timeout_timer()
        self.timeout_timer = self.create_timer(self.goal_timeout, self._on_goal_timeout)

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_timeout(self):
        self.get_logger().warn('Patrol goal timed out, cancelling and moving on')
        self._cancel_timeout_timer()
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

    def _on_goal_result(self, future):
        self._cancel_timeout_timer()
        self.current_goal_handle = None
        if not self.paused:
            self._send_next_checkpoint()


def main():
    rclpy.init()
    node = CheckpointPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
