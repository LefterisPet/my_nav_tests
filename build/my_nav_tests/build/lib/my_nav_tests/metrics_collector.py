#!/usr/bin/env python3

import csv
import json
import math
import os
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')

        # --- Μεταβλητές Μετρικών ---
        self.start_time = None
        self.end_time = None
        self.total_distance = 0.0
        self.last_pose = None
        self.current_pose = None
        self.min_obstacle_clearance = float('inf')
        self.navigation_success = False

        # --- Subscriptions ---
        # Παρακολούθηση θέσης (Odometry)
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Παρακολούθηση αποστάσεων από εμπόδια (LiDAR)
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('Metrics Collector Node iniciado y listo.')

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pose = (x, y)

        # Έναρξη καταγραφής χρόνου στο πρώτο ληφθέν σημείο
        if self.start_time is None:
            self.start_time = time.time()

        # Υπολογισμός συνολικής διανυθείσας απόστασης
        if self.last_pose is not None:
            dist = math.sqrt((x - self.last_pose[0])**2 + (y - self.last_pose[1])**2)
            self.total_distance += dist

        self.last_pose = (x, y)

    def scan_callback(self, msg: LaserScan):
        # Υπολογισμός ελάχιστης απόστασης από εμπόδια
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            current_min = min(valid_ranges)
            if current_min < self.min_obstacle_clearance:
                self.min_obstacle_clearance = current_min

    def stop_collection(self, success: bool = True):
        self.end_time = time.time()
        self.navigation_success = success

    def save_reports(self, start_pose=(-2.0, -0.5), goal_pose=(0.5, 0.5)):
        # --- ΑΠΟΘΗΚΕΥΣΗ ΣΤΟ src/my_nav_tests/my_nav_tests ---
        script_dir = os.path.dirname(os.path.abspath(__file__))

        json_path = os.path.join(script_dir, "navigation_regression_report.json")
        csv_path = os.path.join(script_dir, "navigation_metrics.csv")

        # --- Υπολογισμοί Μετρικών ---
        total_time = (self.end_time - self.start_time) if (self.start_time and self.end_time) else 0.0
        straight_dist = math.sqrt((goal_pose[0] - start_pose[0])**2 + (goal_pose[1] - start_pose[1])**2)
        
        efficiency = (straight_dist / self.total_distance) if self.total_distance > 0 else 0.0

        final_error = 0.0
        if self.current_pose:
            final_error = math.sqrt((goal_pose[0] - self.current_pose[0])**2 + (goal_pose[1] - self.current_pose[1])**2)

        clearance = self.min_obstacle_clearance if self.min_obstacle_clearance != float('inf') else 0.0

        metrics = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "success": self.navigation_success,
            "total_time_sec": round(total_time, 2),
            "total_distance_m": round(self.total_distance, 2),
            "straight_line_distance_m": round(straight_dist, 2),
            "path_efficiency": round(efficiency, 3),
            "final_positional_error_m": round(final_error, 3),
            "min_obstacle_clearance_m": round(clearance, 3),
            "average_speed_m_s": round(self.total_distance / total_time, 2) if total_time > 0 else 0.0
        }

        # 1. Αποθήκευση JSON Report
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(metrics, f, indent=4)

        # 2. Αποθήκευση CSV Report
        file_exists = os.path.isfile(csv_path)
        with open(csv_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=metrics.keys())
            if not file_exists:
                writer.writeheader()
            writer.writerow(metrics)

        self.get_logger().info(f"[REPORT] Τα αποτελέσματα αποθηκεύτηκαν επιτυχώς στο: {script_dir}")


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_collection(success=True)
        node.save_reports()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
