"""
real_mapping.launch.py — SCENARIO A: map an unknown competition arena (REAL robot).

Production mapping entry point. Brings up the full mapping pipeline ON FUSED
ODOMETRY (not ZED-only), so SLAM gets the most robust odom the robot can make:

    sensors (started separately)            this launch
    ─────────────────────────────────────  ──────────────────────────────────────
    ZED wrapper  -> /zed/zed_node/odom  ─┐
    Slamtec A1   -> /scan               ─┼─ sensor_frontend  (/filtered_odom, /scan_flat,
    esp32_bridge -> /odom/wheel         ─┘                     static base_link/base_laser TFs)
                                            odom_fusion (EKF)  -> /odometry/filtered + odom->base_footprint TF
                                            slam_toolbox(map)  -> map->odom TF + /map

TF ownership (exactly one publisher per edge):
    map -> odom              : slam_toolbox        (this launch)
    odom -> base_footprint   : EKF (odom_tf_owner=ekf) OR ZED planar (odom_tf_owner=zed)
    base_footprint->base_link->base_laser : static (sensor_frontend)

After driving the arena, save with:
    ros2 launch rescue_nav save_competition_map.launch.py map_name:=<path>

Sensor prerequisites (start these FIRST, in their own terminals):
    ZED wrapper with publish_tf:=false       (it must NOT own odom->base)
    Slamtec A1 driver publishing /scan
    ros2 launch esp32_bridge esp32_bridge.launch.py   (gives the EKF /odom/wheel)

Args:
    use_sim_time   (false)  — real robot uses wall clock
    use_rviz       (true)
    namespace      ('')     — single-robot default; reserved for multi-robot
    slam_params_file        — slam_toolbox mapping config (dicerox_mapping default)
    map_save_path           — informational; printed as the save command hint
    odom_tf_owner  (ekf)    — 'ekf' (fused, default) or 'zed' (ZED-only, no EKF)
    imu_topic / imu_yaw_sign / zed_odom_topic — forwarded to the fusion layer
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    nav_share = get_package_share_directory('rescue_nav')
    mapping_share = get_package_share_directory('dicerox_mapping')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_save_path = LaunchConfiguration('map_save_path')
    odom_tf_owner = LaunchConfiguration('odom_tf_owner')
    zed_odom_topic = LaunchConfiguration('zed_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    imu_yaw_sign = LaunchConfiguration('imu_yaw_sign')

    frontend_launch = os.path.join(nav_share, 'launch', 'sensor_frontend.launch.py')
    fusion_launch = os.path.join(nav_share, 'launch', 'odom_fusion.launch.py')

    # The ZED planar node (inside sensor_frontend) publishes odom->base_footprint
    # ONLY in the ZED-only fallback. In the default 'ekf' path the EKF owns it.
    zed_publishes_tf = PythonExpression(["'true' if '", odom_tf_owner, "' == 'zed' else 'false'"])
    use_ekf = LaunchConfigurationEquals('odom_tf_owner', 'ekf')

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(mapping_share, 'config', 'slam_toolbox_params.yaml'),
            description='slam_toolbox MAPPING config.'),
        DeclareLaunchArgument(
            'map_save_path',
            default_value=os.path.join(os.path.expanduser('~'), 'maps', 'competition_map'),
            description='Where you intend to save the map (no extension). Informational.'),
        DeclareLaunchArgument(
            'odom_tf_owner', default_value='ekf',
            description="'ekf' = fused odom owns odom->base_footprint (default); "
                        "'zed' = ZED-only planar odom owns it (no fusion)."),
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/zed/zed_node/imu/data'),
        DeclareLaunchArgument('imu_yaw_sign', default_value='1.0'),
    ]

    body = GroupAction([
        PushRosNamespace(namespace),

        LogInfo(msg=['[real_mapping] After driving the arena, save the map with:\n',
                     '  ros2 launch rescue_nav save_competition_map.launch.py map_name:=',
                     map_save_path]),

        # 1) Perception/odometry front-end. The ZED node owns the odom TF only in
        #    the 'zed' fallback; otherwise it is a pure /filtered_odom source.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(frontend_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'zed_odom_topic': zed_odom_topic,
                'publish_zed_odom_tf': zed_publishes_tf,
            }.items(),
        ),

        # 2) Fused odometry (EKF) — owns odom->base_footprint in the default path.
        GroupAction(
            condition=use_ekf,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fusion_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'wheel_odom_topic': '/odom/wheel',
                    'zed_odom_topic': '/filtered_odom',
                    'imu_topic': imu_topic,
                    'imu_yaw_sign': imu_yaw_sign,
                }.items(),
            )],
        ),

        # 3) SLAM Toolbox in mapping mode — owns map->odom + publishes /map.
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time, 'scan_topic': '/scan_flat'},
            ],
        ),

        # 4) RViz (optional).
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(mapping_share, 'rviz', 'mapping.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])

    return LaunchDescription(declared + [body])
