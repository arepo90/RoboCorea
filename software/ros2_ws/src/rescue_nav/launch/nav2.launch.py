"""Nav2 navigation stack (controller, planner, behaviors, BT, waypoint follower).

Wraps nav2_bringup's navigation_launch.py with our nav2_params.yaml. Run on top
of localization.launch.py (AMCL on a saved map) or slam.launch.py (live map).
Send goals from RViz ("Nav2 Goal") or `ros2 run rescue_nav waypoint_runner`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('rescue_nav')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        # Default FALSE: the real robot runs on the wall clock. The sim entry
        # points (demo.launch.py) pass use_sim_time:=true explicitly.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items()),
    ])
