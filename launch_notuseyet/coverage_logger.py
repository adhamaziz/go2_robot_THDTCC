#!/usr/bin/env python3
"""
Logs map coverage % over time during an exploration trial.
Subscribes to an OccupancyGrid (default /projected_map) and writes a CSV
row every time a new map arrives: elapsed_seconds, percent_known, free_cells,
occupied_cells, unknown_cells, total_cells.

Usage:
    ros2 run <your_pkg> coverage_logger.py --ros-args -p output_csv:=/home/ros2_ws/src/trial_1.csv
or just run directly:
    python3 coverage_logger.py  (writes to ./coverage_log.csv by default)

Stop with Ctrl+C when the trial is done (frontiers exhausted / time cap
reached) -- the CSV is flushed after every row, so partial trials are safe.
"""
import csv
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid


class CoverageLogger(Node):
    def __init__(self):
        super().__init__('coverage_logger')

        self.declare_parameter('map_topic', '/projected_map')
        self.declare_parameter('output_csv', 'coverage_log.csv')

        map_topic = self.get_parameter('map_topic').value
        self.csv_path = self.get_parameter('output_csv').value

        self.start_time = None
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'elapsed_seconds', 'percent_known', 'percent_free',
            'percent_occupied', 'free_cells', 'occupied_cells',
            'unknown_cells', 'total_cells'
        ])

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(OccupancyGrid, map_topic, self.map_cb, qos)
        self.get_logger().info(f'Logging coverage from {map_topic} to {self.csv_path}')

    def map_cb(self, msg: OccupancyGrid):
        if self.start_time is None:
            self.start_time = time.time()

        data = msg.data
        total = len(data)
        if total == 0:
            return

        unknown = sum(1 for v in data if v == -1)
        occupied = sum(1 for v in data if v >= 65)   # matches your occupied_thresh convention
        free = total - unknown - occupied

        elapsed = time.time() - self.start_time
        pct_known = 100.0 * (total - unknown) / total
        pct_free = 100.0 * free / total
        pct_occupied = 100.0 * occupied / total

        self.writer.writerow([
            f'{elapsed:.1f}', f'{pct_known:.2f}', f'{pct_free:.2f}',
            f'{pct_occupied:.2f}', free, occupied, unknown, total
        ])
        self.csv_file.flush()

        self.get_logger().info(
            f't={elapsed:6.1f}s  known={pct_known:5.1f}%  '
            f'free={free}  occupied={occupied}  unknown={unknown}'
        )

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CoverageLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
