"""3-D volumetric mapping (OctoMap) — runs ON THE JETSON.

Builds an octree from the ZED2 cloud + the SLAM/EKF TF tree locally and publishes
only the compressed binary octree on /robot/map3d (latched, low-rate). The raw
cloud never leaves the robot. Requires the sensors + 2-D mapping stacks already
running (the ZED cloud and the map->odom->base TF tree).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('rescue_mapping3d'), 'config', 'octomap.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=cfg),
        Node(
            package='rescue_mapping3d',
            executable='octomap_node',
            name='octomap_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
