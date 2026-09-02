#!/usr/bin/env python3
"""
C4 -- Inspection Logger. DIAGNOSTIC BUILD -- extra logging added to
track down a confirmed Z-axis bug (logged marker height stays pinned
near the camera's own height instead of converging to the marker's
real mounted height, even after 146 averaged observations). Remove the
DEBUG log line once the root cause is found and fixed.
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
        self.declare_parameter('camera_frame', 'd435i_color_optical_frame')
        # Known, fixed mounting heights per marker ID, from the world file --
        # overrides the TF-derived Z (currently unreliable, see the odom Z
        # investigation) with the actual design-time constant. X/Y stay
        # TF-derived since those were confirmed accurate (~12cm noise,
        # converges with more observations). Parallel arrays since ROS2
        # params don't support dicts directly.
        self.declare_parameter('known_marker_ids', [0])
        self.declare_parameter('known_marker_heights', [0.3])

        self.output_path = self.get_parameter('output_json').value
        self.reinspect_thresh = self.get_parameter('reinspect_position_threshold_m').value
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        known_ids = self.get_parameter('known_marker_ids').value
        known_heights = self.get_parameter('known_marker_heights').value
        self.known_marker_heights = dict(zip(known_ids, known_heights))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
                'Skipping this detection.'
            )
            return

        for marker in msg.markers:
            mid = marker.marker_id
            world_pose = do_transform_pose(marker.pose, transform)
            pos = world_pose.position
            orient = world_pose.orientation

            # Z override: TF-derived height is currently unreliable (see
            # odom-frame investigation); X/Y are TF-derived and confirmed
            # accurate. Fall back to the TF value if this marker has no
            # known height configured.
            z_value = self.known_marker_heights.get(mid, pos.z)

            if mid not in self.records:
                self.records[mid] = {
                    'marker_id': mid,
                    'frame': self.target_frame,
                    'position': {'x': pos.x, 'y': pos.y, 'z': z_value},
                    'orientation': {'x': orient.x, 'y': orient.y, 'z': orient.z, 'w': orient.w},
                    'num_observations': 1,
                    'first_seen_sec': now,
                    'last_updated_sec': now,
                }
                self.get_logger().info(f'NEW marker {mid} logged at ({pos.x:.2f}, {pos.y:.2f}, {z_value:.2f})')
                self._write()
            else:
                rec = self.records[mid]
                dx = pos.x - rec['position']['x']
                dy = pos.y - rec['position']['y']
                dz = z_value - rec['position']['z']
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)

                rec['num_observations'] += 1
                rec['last_updated_sec'] = now

                if dist > self.reinspect_thresh:
                    rec['position'] = {'x': pos.x, 'y': pos.y, 'z': z_value}
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
