#!/usr/bin/env python3
"""
Gauge reader -- steps 6+7 of the professor's pipeline (capture + digitize).

Design: rather than training a second detector to find "the gauge," this
uses the ArUco marker's already-solved pose directly. The marker-to-gauge
offset is a fixed, known physical relationship (we placed them together
in the world file), so the gauge's image-space location can be computed
analytically: rotate the known local offset by the marker's detected
orientation, add to the marker's detected position (both already given
in the camera's own frame by /aruco_detections -- no TF lookup needed
here, unlike the FSM/inspection logger, since this only needs a
camera-relative projection, not a map-relative navigation goal),
project through the camera intrinsics.

Needle angle extraction: classic CV (color threshold for the needle ->
Hough line transform -> angle from the known dial center), per the
professor's suggested approach, not a trained model.

CALIBRATION IS UNVERIFIED against the actual rendered gauge -- run
against the known 30-degree test angle in the world file first and
adjust GAUGE_ANGLE_OFFSET_DEG / GAUGE_ANGLE_SIGN until the reported
percentage matches what you placed, before trusting it on anything else.
"""
import json
import math
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from aruco_opencv_msgs.msg import ArucoDetection
from cv_bridge import CvBridge


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


class GaugeReader(Node):
    def __init__(self):
        super().__init__('gauge_reader')

        self.declare_parameter('image_topic', '/d435i/color/image_raw')
        self.declare_parameter('camera_info_topic', '/d435i/color/camera_info')
        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('gauge_offset_local', [0.0, 0.15, 0.0])  # was [0,-0.15,0] --
        # empirically wrong direction: that moved the crop DOWN (away from
        # the dial), confirming +Y (not -Y) is "up" in this library's
        # actual convention. Determined by testing, not re-derived from
        # the OpenCV convention I assumed before -- that assumption was
        # also wrong, or this specific library differs from it.
        # Real physical half-size of the gauge dial (matches the 0.12m
        # box in the world file, plus a margin factor) -- used to compute
        # the crop size via proper pinhole projection (apparent_px =
        # fx * real_size_m / distance_m) rather than an empirically
        # guessed reference-distance/reference-pixel pair, which was
        # still coming out too large (swallowing the marker) even after
        # two rounds of tuning the cap.
        self.declare_parameter('gauge_physical_half_size_m', 0.08)
        self.declare_parameter('output_json', 'gauge_readings.json')
        self.declare_parameter('gauge_calib_slope', 1.0547)      # CALIBRATED: from two real
        self.declare_parameter('gauge_calib_intercept', 46.0148)  # measured (angle, true%) pairs

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.target_marker_id = self.get_parameter('target_marker_id').value
        self.gauge_offset_local = self.get_parameter('gauge_offset_local').value
        self.gauge_half_size_m = self.get_parameter('gauge_physical_half_size_m').value
        self.output_path = self.get_parameter('output_json').value
        self.calib_slope = self.get_parameter('gauge_calib_slope').value
        self.calib_intercept = self.get_parameter('gauge_calib_intercept').value

        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_image = None
        self.recent_readings = []  # rolling window for temporal smoothing
        self.declare_parameter('smoothing_window', 5)
        self.smoothing_window = self.get_parameter('smoothing_window').value

        self.create_subscription(CameraInfo, self.camera_info_topic, self.caminfo_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 5)
        self.create_subscription(ArucoDetection, '/aruco_detections', self.aruco_cb, 10)

        self.debug_pub = self.create_publisher(Image, '/gauge/debug_image', 10)

        self.get_logger().info('Gauge reader started, watching marker id %d' % self.target_marker_id)

    def caminfo_cb(self, msg):
        self.camera_info = msg

    def image_cb(self, msg):
        self.latest_image = msg

    def aruco_cb(self, msg: ArucoDetection):
        if self.camera_info is None or self.latest_image is None:
            return
        for marker in msg.markers:
            if marker.marker_id != self.target_marker_id:
                continue
            self.process_gauge(marker)
            return

    def process_gauge(self, marker):
        p = marker.pose.position
        q = marker.pose.orientation
        ox, oy, oz = self.gauge_offset_local

        rx, ry, rz = rotate_vector_by_quaternion(ox, oy, oz, q.x, q.y, q.z, q.w)
        gx, gy, gz = p.x + rx, p.y + ry, p.z + rz

        if gz <= 0.05:
            self.get_logger().warn('Gauge projected behind/at the camera plane, skipping')
            return

        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]
        u = int(fx * gx / gz + cx)
        v = int(fy * gy / gz + cy)

        # Proper pinhole projection using the gauge's known real physical
        # size, instead of an empirically-tuned reference-distance guess
        # (which was still coming out too large after two rounds of
        # tuning). apparent_px = focal_length * real_size_m / distance_m
        # is exact given accurate intrinsics and gz -- no guessing.
        scaled_half = int(fx * self.gauge_half_size_m / gz)
        scaled_half = max(15, min(150, scaled_half))  # sane bounds only, not a fudge factor

        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        h, w = cv_image.shape[:2]
        x0, x1 = max(0, u - scaled_half), min(w, u + scaled_half)
        y0, y1 = max(0, v - scaled_half), min(h, v + scaled_half)
        if x1 - x0 < 20 or y1 - y0 < 20:
            self.get_logger().warn(f'Gauge projected near/off image edge ({u},{v}), skipping')
            return

        crop = cv_image[y0:y1, x0:x1]
        reading = self.extract_needle_angle(crop)

        if reading is not None:
            angle_deg, pct = reading
            self.recent_readings.append(pct)
            if len(self.recent_readings) > self.smoothing_window:
                self.recent_readings.pop(0)
            smoothed_pct = float(np.median(self.recent_readings))

            self.get_logger().info(
                f'Marker {marker.marker_id}: needle angle {angle_deg:.1f} deg -> '
                f'{pct:.1f}% (raw), {smoothed_pct:.1f}% (smoothed over {len(self.recent_readings)} readings)'
            )
            self._log_reading(marker.marker_id, angle_deg, smoothed_pct)
        else:
            self.get_logger().warn('Needle not found in gauge crop')

        debug_img = cv_image.copy()
        cv2.rectangle(debug_img, (x0, y0), (x1, y1), (0, 255, 0), 2)
        debug_msg = self._to_image_msg(debug_img)
        debug_msg.header = self.latest_image.header
        self.debug_pub.publish(debug_msg)

    def extract_needle_angle(self, crop):
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 80])
        upper_red2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        ys, xs = np.nonzero(mask)
        if len(xs) < 10:
            return None  # not enough red pixels to be a real detection

        # Centroid of the red mask, relative to the known pivot (crop
        # center, by construction -- we projected the gauge center to
        # build this crop). With the needle now asymmetric (long pointer
        # arm + short tail, see the world file update), the centroid is
        # naturally pulled toward the pointer side -- this resolves
        # direction unambiguously, unlike a Hough line's endpoint order
        # on the old symmetric shape, which could flip 180 degrees
        # randomly between frames.
        cx_crop, cy_crop = crop.shape[1] / 2.0, crop.shape[0] / 2.0
        centroid_x, centroid_y = float(np.mean(xs)), float(np.mean(ys))

        dx = centroid_x - cx_crop
        dy = centroid_y - cy_crop
        if math.hypot(dx, dy) < 3:
            return None  # centroid too close to pivot to get a reliable direction

        angle_deg = math.degrees(math.atan2(-dy, dx))  # -dy: image y is flipped vs. math convention

        # Directly-fitted linear calibration from two real measured points:
        # (angle=20.84 deg, true=68%) and (angle=-16.13 deg, true=29%).
        # Replaces the earlier offset/sign/270-degree-sweep formula, which
        # was based on an unverified assumption about the dial's angular
        # range rather than an actual measurement.
        pct = self.calib_slope * angle_deg + self.calib_intercept
        pct = max(0.0, min(100.0, pct))

        return angle_deg, pct

    def _to_image_msg(self, cv_image):
        cv_image = np.ascontiguousarray(cv_image, dtype=np.uint8)
        msg = Image()
        msg.height, msg.width = cv_image.shape[0], cv_image.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = cv_image.shape[1] * cv_image.shape[2]
        msg.data = cv_image.tobytes()
        return msg

    def _log_reading(self, marker_id, angle_deg, pct):
        records = []
        if os.path.exists(self.output_path):
            try:
                with open(self.output_path) as f:
                    records = json.load(f)
            except json.JSONDecodeError:
                pass
        records.append({
            'marker_id': marker_id,
            'needle_angle_deg': angle_deg,
            'reading_percent': pct,
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
        })
        with open(self.output_path, 'w') as f:
            json.dump(records, f, indent=2)


def main():
    rclpy.init()
    node = GaugeReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
