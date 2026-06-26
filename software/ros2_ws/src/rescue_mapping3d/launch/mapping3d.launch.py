"""3-D volumetric mapping (OctoMap) — runs ON THE JETSON.

Builds an octree from the ZED2 cloud + the SLAM/EKF TF tree locally and publishes
only the compressed binary octree on /robot/map3d (latched, low-rate). The raw
cloud never leaves the robot. Requires the sensors + 2-D mapping stacks already
running (the ZED cloud and the map->odom->base_footprint TF tree).

TF note: the mapping_ekf tree ends at base_footprint (no base_link), but the ZED
roots its camera frames at base_link, so map<-zed_*_frame can't resolve and the
octree stays empty. We publish a static base_footprint->base_link here to join
the two. Set base_link_z to the base_link height above the ground (base_footprint)
for correct absolute height; 0.0 still connects the tree (self-consistent with the
URDF, which is also placed at base_footprint). Disable with publish_base_link:=false
if your stack already publishes base_footprint->base_link.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('rescue_mapping3d'), 'config', 'octomap.yaml')
    base_link_z = LaunchConfiguration('base_link_z')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=cfg),
        DeclareLaunchArgument('publish_base_link', default_value='true',
                              description='Publish a static base_footprint->base_link '
                                          '(joins the ZED frames to the map tree).'),
        DeclareLaunchArgument('base_link_z', default_value='0.0',
                              description='base_link height above base_footprint (m).'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_base_link')),
            arguments=['--x', '0', '--y', '0', '--z', base_link_z,
                       '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
        ),

        Node(
            package='rescue_mapping3d',
            executable='octomap_node',
            name='octomap_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
