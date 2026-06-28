"""3-D volumetric mapping (OctoMap) — runs ON THE JETSON.

Builds an octree from the ZED2 cloud + the SLAM/EKF TF tree locally and publishes
only the compressed binary octree on /robot/map3d (latched, low-rate). The raw
cloud never leaves the robot. Requires the sensors + 2-D mapping stacks already
running (the ZED cloud and the map->odom->base_footprint TF tree).

TF note: the 2-D mapping tree's odom edge ends at base_footprint. The ZED's camera frames are
their own subtree rooted at zed_camera_link (published by the ZED state publisher
on the Jetson; the robot URDF that would mount them lives on the laptop). To let
octomap_node transform the cloud into `map`, we graft the ZED subtree onto the map
tree with a single static base_footprint->zed_camera_link mount.

IMPORTANT — do NOT route this through `base_link`: the digital-twin URDF
(robot_state_publisher on the workstation) already publishes world_link->base_link
continuously, so a frame can only have one parent and any base_footprint->base_link
static we publish loses that conflict — stranding the ZED subtree under world_link,
unreachable from `map` (the octree then never grows). Parenting zed_camera_link
directly under base_footprint sidesteps the twin entirely. Set zed_mount_z to the
camera height above base_footprint (ground) for correct absolute height.
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
    zed_frame = LaunchConfiguration('zed_mount_frame')
    zed_parent = LaunchConfiguration('zed_parent_frame')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=cfg),
        # The ZED camera frames are their own subtree on the Jetson (root =
        # zed_camera_link). Graft it onto the MAP tree at base_footprint so
        # octomap_node can transform the cloud into `map`. base_footprint (NOT
        # base_link) is deliberate — base_link is owned by the twin URDF on the
        # laptop (world_link->base_link) and would win the parent conflict.
        DeclareLaunchArgument('publish_zed_mount', default_value='true',
                              description='Publish a static <zed_parent_frame>->ZED-camera mount.'),
        DeclareLaunchArgument('zed_parent_frame', default_value='base_footprint',
                              description='Map-tree frame to parent the ZED subtree under.'),
        DeclareLaunchArgument('zed_mount_frame', default_value='zed_camera_link',
                              description='ZED subtree root frame to parent under zed_parent_frame.'),
        DeclareLaunchArgument('zed_mount_x', default_value='0.20'),
        DeclareLaunchArgument('zed_mount_y', default_value='0.0'),
        DeclareLaunchArgument('zed_mount_z', default_value='0.30',
                              description='Camera height above base_footprint (ground), m.'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_zed',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_zed_mount')),
            arguments=['--x', LaunchConfiguration('zed_mount_x'),
                       '--y', LaunchConfiguration('zed_mount_y'),
                       '--z', LaunchConfiguration('zed_mount_z'),
                       '--frame-id', zed_parent, '--child-frame-id', zed_frame],
        ),

        Node(
            package='rescue_mapping3d',
            executable='octomap_node',
            name='octomap_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
