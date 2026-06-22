"""
world.launch.py — Start Gazebo Fortress with the Stage A world.

Usage (standalone):
    ros2 launch tracked_robot_gz world.launch.py
    ros2 launch tracked_robot_gz world.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_gz = get_package_share_directory('tracked_robot_gz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_gz, 'worlds', 'stage_a.sdf')

    headless = LaunchConfiguration('headless')

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without GUI (server only).',
    )

    # gz_args: -r = run immediately, -s = server only (headless)
    # We build the args string conditionally via a PythonExpression
    from launch.conditions import IfCondition, UnlessCondition
    from launch_ros.actions import Node

    gz_sim_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items(),
        condition=UnlessCondition(headless),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s {world_file}',
        }.items(),
        condition=IfCondition(headless),
    )

    return LaunchDescription([
        declare_headless,
        gz_sim_with_gui,
        gz_sim_headless,
    ])
