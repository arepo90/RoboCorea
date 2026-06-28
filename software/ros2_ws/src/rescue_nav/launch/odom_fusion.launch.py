"""
odom_fusion.launch.py — the odometry fusion layer.

Brings up:
  * adaptive_odom_covariance  — dynamic covariance front-end + QoS/frame bridge
  * ekf_filter_node (robot_localization) — fuses the *_adaptive topics into a
    single continuous odom -> base_footprint estimate (and the TF).

This is the ONLY publisher of odom -> base_footprint. When you run it alongside
the SLAM/localization front-end, keep the planar ZED node's TF off
(publish_odom_tf:=false) and run the ZED driver with publish_tf:=false so nothing
fights the EKF for that TF. real_mapping.launch.py / real_navigation.launch.py
wire that up for you (the use_ekf arg selects EKF vs ZED-direct).

Outputs:
  /odometry/filtered  (nav_msgs/Odometry)  — the fused estimate
  TF: odom -> base_footprint
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('rescue_nav')
    default_ekf = os.path.join(pkg, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_params = LaunchConfiguration('ekf_params_file')
    wheel_odom_topic = LaunchConfiguration('wheel_odom_topic')
    zed_odom_topic = LaunchConfiguration('zed_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    base_frame = LaunchConfiguration('base_frame')
    imu_yaw_sign = LaunchConfiguration('imu_yaw_sign')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('ekf_params_file', default_value=default_ekf),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/odom/wheel'),
        DeclareLaunchArgument('zed_odom_topic', default_value='/filtered_odom',
                              description='Planar ZED odometry (zed_planar_odom output).'),
        DeclareLaunchArgument('imu_topic', default_value='/zed/zed_node/imu/data'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('imu_yaw_sign', default_value='1.0',
                              description='Set to -1.0 if the ZED yaw-gyro sign is inverted.'),

        Node(
            package='rescue_nav',
            executable='adaptive_odom_covariance',
            name='adaptive_odom_covariance',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'wheel_odom_topic': wheel_odom_topic,
                'zed_odom_topic': zed_odom_topic,
                'imu_topic': imu_topic,
                'base_frame': base_frame,
                'imu_yaw_sign': ParameterValue(imu_yaw_sign, value_type=float),
            }],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_params,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
    ])
