#!/usr/bin/env python3

"""Nav2 regression testing execution and metrics collection module."""

import csv
import glob
import json
import math
import os
import shutil
import subprocess
import time
import unittest

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def clean_environment():
    """Clean FastDDS locks and terminate specific ROS 2/Gazebo processes."""
    print('[SETUP] Καθαρισμός διεργασιών και FastDDS Shared Memory...')
    targets = [
        'gz', 'ruby', 'rviz2', 'nav2_amcl', 'nav2_planner',
        'nav2_controller', 'nav2_bt_navigator', 'robot_state_publisher'
    ]
    for target in targets:
        subprocess.run(['pkill', '-9', '-f', target], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    
    shm_patterns = ['/dev/shm/fastrtps_*', '/dev/shm/sem.fastrtps_*', '/tmp/fastrtps_*']
    for pattern in shm_patterns:
        for filepath in glob.glob(pattern):
            try:
                if os.path.isfile(filepath) or os.path.islink(filepath):
                    os.unlink(filepath)
                elif os.path.isdir(filepath):
                    shutil.rmtree(filepath)
            except Exception:
                pass
    print('[SETUP] Ο καθαρισμός ολοκληρώθηκε επιτυχώς.')


def launch_headless_simulation():
    """Launch Gazebo Sim in headless mode with Nav2 autostart."""
    print('[SETUP] Εκκίνηση Gazebo Sim & Nav2 σε Headless Mode...')
    env = os.environ.copy()
    env['TURTLEBOT3_MODEL'] = 'waffle'
    env['GZ_SIM_RESOURCE_PATH'] = '/opt/ros/jazzy/share/turtlebot3_gazebo/models:/opt/ros/jazzy/share'
    env['GAZEBO_MODEL_PATH'] = '/opt/ros/jazzy/share/turtlebot3_gazebo/models'
    
    cmd = [
        'ros2', 'launch', 'nav2_bringup', 'tb3_simulation_launch.py',
        'use_sim_time:=True', 'headless:=True', 'use_rviz:=False', 'autostart:=True'
    ]
    process = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print('[SETUP] Αναμονή 12s για την πλήρη αρχικοποίηση του simulation...')
    time.sleep(12)
    return process


class Nav2MetricsCollector(Node):
    """Node for collecting navigation metrics and executing goal requests."""

    def __init__(self):
        super().__init__('nav2_metrics_collector')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.current_pose = None
        self.last_pose = None
        self.total_distance = 0.0
        self.start_time = None
        self.end_time = None
        self.navigation_completed = False
        self.navigation_success = False
        self.min_obstacle_clearance = float('inf')

    def scan_callback(self, msg):
        """Record minimum obstacle clearance from LiDAR scan."""
        if self.start_time is not None and not self.navigation_completed:
            valid_ranges = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
            if valid_ranges:
                current_min = min(valid_ranges)
                if current_min < self.min_obstacle_clearance:
                    self.min_obstacle_clearance = current_min

    def odom_callback(self, msg):
        """Record current pose and compute traveled distance from odometry."""
        pos = msg.pose.pose.position
        self.current_pose = (pos.x, pos.y)
        if self.start_time is not None and not self.navigation_completed:
            if self.last_pose is not None:
                dx = pos.x - self.last_pose[0]
                dy = pos.y - self.last_pose[1]
                dist = math.sqrt(dx*dx + dy*dy)
                if dist > 0.001:
                    self.total_distance += dist
            self.last_pose = self.current_pose

    def set_initial_pose(self, x=-2.0, y=-0.5):
        """Publish initial pose for localization initialization."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        print(f'[TEST] Αποστολή Initial Pose στο ({x}, {y})...')
        for _ in range(5):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.4)

    def send_goal(self, goal_x=0.5, goal_y=0.5):
        """Send navigation goal to Nav2 action server."""
        print('[TEST] Αναμονή για τον Action Server Nav2...')
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=15.0):
            print('[ERROR] O Nav2 Action Server δεν είναι διαθέσιμος!')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        print(f'[TEST] Αποστολή Navigation Goal στο ({goal_x}, {goal_y})...')
        self.start_time = time.time()
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            print('[ERROR] Το Goal απορρίφθηκε από το Nav2!')
            return False

        print('[TEST] Το Goal έγινε αποδεκτό. Πλοήγηση σε εξέλιξη...')
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)

        self.end_time = time.time()
        self.navigation_completed = True
        status = result_future.result().status
        self.navigation_success = (status == 4)
        return self.navigation_success

    def save_reports(self, start_pose=(-2.0, -0.5), goal_pose=(0.5, 0.5)):
        """Save collected metrics to JSON and CSV reports."""
        target_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(target_dir, 'navigation_regression_report.json')
        csv_path = os.path.join(target_dir, 'navigation_metrics.csv')
        
        total_time = (self.end_time - self.start_time) if (self.start_time and self.end_time) else 0.0
        straight_dist = math.sqrt((goal_pose[0] - start_pose[0])**2 + (goal_pose[1] - start_pose[1])**2)
        efficiency = (straight_dist / self.total_distance) if self.total_distance > 0 else 0.0
        final_error = math.sqrt((goal_pose[0] - self.current_pose[0])**2 + (goal_pose[1] - self.current_pose[1])**2) if self.current_pose else 0.0
        clearance = self.min_obstacle_clearance if self.min_obstacle_clearance != float('inf') else 0.0

        metrics = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'success': self.navigation_success,
            'total_time_sec': round(total_time, 2),
            'total_distance_m': round(self.total_distance, 2),
            'straight_line_distance_m': round(straight_dist, 2),
            'path_efficiency': round(efficiency, 3),
            'final_positional_error_m': round(final_error, 3),
            'min_obstacle_clearance_m': round(clearance, 3),
            'average_speed_m_s': round(self.total_distance / total_time, 2) if total_time > 0 else 0.0
        }

        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(metrics, f, indent=4)

        file_exists = os.path.isfile(csv_path)
        with open(csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=metrics.keys())
            if not file_exists:
                writer.writeheader()
            writer.writerow(metrics)
        print(f'[REPORT] Τα αποτελέσματα αποθηκεύτηκαν επιτυχώς στη διαδρομή: {target_dir}')


class TestNavigationRegression(unittest.TestCase):
    """Test case for running Nav2 regression and validating results."""

    def test_navigation_regression_execution(self):
        """Execute simulation and check navigation success."""
        clean_environment()
        sim_process = launch_headless_simulation()
        try:
            if not rclpy.ok():
                rclpy.init()
            collector = Nav2MetricsCollector()
            collector.set_initial_pose(-2.0, -0.5)
            time.sleep(2.0)
            success = collector.send_goal(0.5, 0.5)
            collector.save_reports((-2.0, -0.5), (0.5, 0.5))
            self.assertTrue(success, 'Navigation goal failed or was rejected by Nav2.')
        except Exception as e:
            self.fail(f'Regression test raised an exception: {e}')
        finally:
            print('[CLEANUP] Τερματισμός διεργασιών...')
            if rclpy.ok():
                rclpy.shutdown()
            if sim_process:
                sim_process.terminate()
                sim_process.wait()
            clean_environment()


if __name__ == '__main__':
    unittest.main()
