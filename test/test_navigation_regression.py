#!/usr/bin/env python3

"""Nav2 regression testing execution module in standard (non-headless) mode with full visualization."""

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
from nav_msgs.msg import Odometry, OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def clean_environment():
    """Clean FastDDS locks and terminate specific ROS 2/Gazebo processes."""
    print('[SETUP] Clean environment and FastDDS Shared Memory...')
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
    print('[SETUP] The cleanup has finished successfully.')


class Nav2MetricsCollector(Node):
    """Node for collecting navigation metrics, executing goal requests, and validating costmaps."""

    def __init__(self):
        super().__init__('nav2_metrics_collector')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, 
            '/global_costmap/costmap', 
            self.global_costmap_callback, 
            10
        )
        self.latest_costmap = None

        self.current_pose = None
        self.last_pose = None
        self.total_distance = 0.0
        self.start_time = None
        self.end_time = None
        self.navigation_completed = False
        self.navigation_success = False
        self.min_obstacle_clearance = float('inf')

    def global_costmap_callback(self, msg):
        self.latest_costmap = msg

    def check_goal_in_costmap(self, goal_x, goal_y):
        """Ελέγχει αν ο στόχος πέφτει πάνω σε εμπόδιο ή εκτός ορίων στο costmap."""
        if self.latest_costmap is None:
            self.get_logger().warn('[WARNING] Costmap not received yet! Skipping local costmap validation.')
            return True

        info = self.latest_costmap.info
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height

        mx = int(round((goal_x - origin_x) / resolution))
        my = int(round((goal_y - origin_y) / resolution))

        if 0 <= mx < width and 0 <= my < height:
            index = my * width + mx
            cost = self.latest_costmap.data[index]
            
            self.get_logger().info(f'[COSTMAP CHECK] Target ({goal_x}, {goal_y}) -> Grid ({mx}, {my}) has Cost: {cost}')
            
            if cost >= 100 or cost == -1:
                self.get_logger().error(f'[ERROR] ❌ Goal is on a lethal obstacle or unknown space! (Cost: {cost})')
                return False
            else:
                self.get_logger().info('[SUCCESS] ✅ Goal position is acceptable.')
                return True
        else:
            self.get_logger().error('[ERROR] ❌ Goal is OUT OF BOUNDS of the costmap!')
            return False

    def scan_callback(self, msg):
        if self.start_time is not None and not self.navigation_completed:
            valid_ranges = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
            if valid_ranges:
                current_min = min(valid_ranges)
                if current_min < self.min_obstacle_clearance:
                    self.min_obstacle_clearance = current_min

    def odom_callback(self, msg):
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
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'[TEST] Send Initial Pose in ({x}, {y})...')
        for _ in range(3):
            self.initial_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.3)
            time.sleep(0.3)
        self.get_logger().info('[TEST] The Initial Pose is sent successfully.')

    def send_goal(self, goal_x=0.5673, goal_y=-1.6966):
        self.get_logger().info('[DEBUG] Waiting for Action Server Nav2...')
        
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=45.0):
            self.get_logger().error('[ERROR] The Nav2 Action Server is not available!')
            return False

        self.get_logger().info('[DEBUG] Waiting for global costmap data...')
        wait_start = time.time()
        while self.latest_costmap is None and (time.time() - wait_start) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.3)
            
        if self.latest_costmap is None:
            self.get_logger().warn('[WARNING] Costmap timeout! Proceeding without costmap validation.')
        else:
            self.get_logger().info('[SUCCESS] ✅ Global costmap received successfully!')

        if not self.check_goal_in_costmap(goal_x, goal_y):
            self.get_logger().error('[ABORT] Goal rejected locally due to obstacle collision in costmap.')
            return False

        self.get_logger().info('[DEBUG] Nav2 Action Server found! Preparing goal message...')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'[DEBUG] Sending Navigation Goal to ({goal_x}, {goal_y})...')
        self.start_time = time.time()
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        
        while rclpy.ok() and not send_goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('[ERROR] ❌ Goal was REJECTED by Nav2 Action Server!')
            return False

        self.get_logger().info('[DEBUG] ✅ Goal was ACCEPTED by Nav2! Navigation in progress...')
        
        result_future = goal_handle.get_result_async()
        
        start_nav_wait = time.time()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            if (time.time() - start_nav_wait) > 120.0:
                self.get_logger().error('[ERROR] ❌ Navigation timeout exceeded!')
                break

        self.end_time = time.time()
        self.navigation_completed = True
        
        if not result_future.done():
            self.get_logger().error('[ERROR] Result future timed out!')
            return False

        result = result_future.result()
        if not result:
            self.get_logger().error('[ERROR] Result future returned None!')
            return False
            
        status = result.status
        self.get_logger().info(f'[DEBUG] Navigation finished with Action Status Code: {status}')
        
        if self.current_pose:
            final_dist = math.sqrt((goal_x - self.current_pose[0])**2 + (goal_y - self.current_pose[1])**2)
            self.get_logger().info(f'[METRICS] Final distance to target: {final_dist:.3f} meters')
            if status == 4 or final_dist < 0.35:
                self.get_logger().info('[SUCCESS] 🎉 Navigation completed successfully!')
                self.navigation_success = True
            else:
                self.get_logger().error(f'[ERROR] Navigation failed with status code: {status}')
                self.navigation_success = False
        else:
            if status == 4:
                self.get_logger().info('[SUCCESS] Navigation completed successfully!')
                self.navigation_success = True
            else:
                self.get_logger().error(f'[ERROR] Navigation failed with status code: {status}')
                self.navigation_success = False
            
        return self.navigation_success

    def save_reports(self, start_pose=(-2.0, -0.5), goal_pose=(0.5673, -1.6966)):
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
        print(f'[REPORT] The results have been saved to the route: {target_dir}')


class TestNavigationRegression(unittest.TestCase):
    """Test case for auto-launching simulation in standard GUI mode."""

    def test_navigation_regression_execution(self):
        target_goal_x = 0.5673
        target_goal_y = -1.6966
        start_x = -2.0
        start_y = -0.5

        sim_process = None
        try:
            clean_environment()

            if not rclpy.ok():
                rclpy.init()

            print('[LAUNCH] Starting TurtleBot3 simulation and Nav2 stack with GUI...')
            env = os.environ.copy()
            env['TURTLEBOT3_MODEL'] = 'waffle'
            
            # --- ΕΠΙΣΤΡΟΦΗ ΣΕ GUI MODE (headless:=False) ---
            sim_process = subprocess.Popen(
                ['ros2', 'launch', 'nav2_bringup', 'tb3_simulation_launch.py', 'use_sim_time:=True', 'headless:=False'],
                env=env
            )

            print('[LAUNCH] Waiting 15 seconds for simulation and GUI startup...')
            time.sleep(15.0)

            collector = Nav2MetricsCollector()
            collector.set_initial_pose(start_x, start_y)

            print('[LAUNCH] Waiting for TF tree (map -> base_link) to become active via AMCL...')
            temp_node = rclpy.create_node('tf_waiter_node')
            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, temp_node)
            
            tf_ready = False
            start_wait = time.time()
            
            while (time.time() - start_wait) < 30.0:
                rclpy.spin_once(temp_node, timeout_sec=0.5)
                try:
                    if tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                        print('[LAUNCH] ✅ TF tree is fully ready! Map frame detected.')
                        tf_ready = True
                        break
                except Exception:
                    pass
            
            temp_node.destroy_node()

            if not tf_ready:
                self.fail('Timeout waiting for TF transform from map to base_link!')

            print('[LAUNCH] Giving AMCL 2 seconds to stabilize before sending goal...')
            time.sleep(2.0)

            success = collector.send_goal(target_goal_x, target_goal_y)
            collector.save_reports((start_x, start_y), (target_goal_x, target_goal_y))
            
            print('\n' + '='*60)
            print('[INFO] ✅ Test execution completed successfully!')
            print('[INFO] Gazebo and RViz are open for your visual inspection.')
            print('[INFO] Press [ENTER] in this terminal when you are done to close everything.')
            print('='*60 + '\n')
            
            input()

            self.assertTrue(success, 'Navigation goal failed or was rejected by Nav2.')

        except Exception as e:
            self.fail(f'Regression test raised an exception: {e}')
        finally:
            print('[CLEANUP] Terminating simulation process and cleaning up...')
            if sim_process:
                sim_process.terminate()
                sim_process.wait()
            clean_environment()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()