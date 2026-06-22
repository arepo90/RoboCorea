"""
odom.launch.py — Standalone launch for the tracked odometry node.

Usage:
    ros2 launch tracked_robot_odom odom.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    track_sep = LaunchConfiguration('track_separation')
    rate      = LaunchConfiguration('publish_rate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'track_separation',
            default_value='0.530',
            description='Distance between left and right track centrelines (m)',
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Odometry publish rate (Hz)',
        ),

        Node(
            package='tracked_robot_odom',
            executable='tracked_odom_node',
            name='tracked_odom_node',
            output='screen',
            parameters=[{
                'track_separation': track_sep,
                'publish_rate':     rate,
                'odom_frame':       'odom',
                'base_frame':       'base_footprint',
                'use_sim_time':     True,
            }],
        ),
    ])
