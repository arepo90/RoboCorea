"""
nav2.launch.py — Launch Nav2 stack with the Dicerox Stage A map and params.

Usage (standalone — requires robot already spawned and odom running):
    ros2 launch tracked_robot_nav2 nav2.launch.py
    ros2 launch tracked_robot_nav2 nav2.launch.py use_rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2      = get_package_share_directory('tracked_robot_nav2')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    map_yaml   = os.path.join(pkg_nav2, 'maps', 'stage_a.yaml')
    params_file = os.path.join(pkg_nav2, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_nav2, 'rviz', 'stage_a.rviz')

    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with the Stage A config.',
    )

    # Nav2 bringup (lifecycle-managed stack)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':            map_yaml,
            'params_file':    params_file,
            'use_sim_time':   'true',
            'autostart':      'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_rviz,
        nav2,
        rviz,
    ])
