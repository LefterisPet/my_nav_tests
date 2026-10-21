import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    package_dir = get_package_share_directory('my_nav_tests')

    use_rviz = LaunchConfiguration('use_rviz', default='False')
    headless = LaunchConfiguration('headless', default='True')
    autostart = LaunchConfiguration('autostart', default='False')
    normalized_headless = PythonExpression([
        "'True' if '", headless, "'.lower() == 'true' else 'False'"
    ])
    world = LaunchConfiguration(
        'world',
        default=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
    )
    robot_sdf = LaunchConfiguration(
        'robot_sdf',
        default=os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro'),
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')
            ),
            launch_arguments={
                'map': os.path.join(nav2_bringup_dir, 'maps', 'tb3_sandbox.yaml'),
                'params_file': os.path.join(package_dir, 'config', 'nav2_params.yaml'),
                'rviz_config_file': os.path.join(
                    nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'
                ),
                'use_sim_time': 'true',
                'autostart': autostart,
                'slam': 'False',
                'world': world,
                'robot_sdf': robot_sdf,
                'headless': normalized_headless,
                'use_rviz': use_rviz,
                'x_pose': '-0.9853',
                'y_pose': '0.5273',
                'z_pose': '0.01',
                'yaw': '0.0',
            }.items(),
        )
    ])
