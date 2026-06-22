"""
sensor_frontend.launch.py — the real-robot perception/odometry FRONT-END.

This is the shared first stage for BOTH competition workflows (real_mapping and
real_navigation). It brings up everything between the raw sensors and the SLAM/
AMCL/Nav2 consumers, but it does NOT start SLAM, AMCL, Nav2 or the EKF. Compose
it with odom_fusion.launch.py (EKF) + a localization layer downstream.

What it starts
--------------
  1. zed_planar_odom (from dicerox_mapping) — projects ZED VIO into a planar
     /filtered_odom. publish_odom_tf defaults to FALSE here: in production the
     robot_localization EKF owns odom -> base_footprint, so this node is a pure
     odometry *source*. (Set publish_odom_tf:=true only for a ZED-only fallback
     with no EKF.)
  2. scan_frame_republisher (from dicerox_mapping) — /scan -> /scan_flat in the
     base_laser frame, gated until /filtered_odom exists so SLAM/AMCL never see
     a scan before the TF path is up.
  3. static TF base_footprint -> base_link  (identity by default; the planar
     rescue base treats them coincident — refine z on hardware if needed).
  4. static TF base_link -> base_laser      (LiDAR extrinsics; A1 mounted
     reversed by default -> yaw = pi, matching dicerox_mapping's known-good
     mounting).

Standardized TF contract (shared with the sim stack):
    map -> odom -> base_footprint -> base_link -> base_laser
              ^         ^
              |         └─ EKF (odom_fusion.launch.py), NOT this front-end
              └─ SLAM (real_mapping) or AMCL/slam_toolbox (real_navigation)

NOTE: the raw inputs (ZED wrapper -> /zed/zed_node/odom, the Slamtec A1 driver
-> /scan, and esp32_bridge -> /odom/wheel) are started SEPARATELY as the robot's
sensor bringup. Run the ZED wrapper with publish_tf:=false so nothing else
fights the EKF for odom -> base_footprint.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    zed_odom_topic = LaunchConfiguration('zed_odom_topic')
    filtered_odom_topic = LaunchConfiguration('filtered_odom_topic')
    publish_zed_odom_tf = LaunchConfiguration('publish_zed_odom_tf')
    zed_x = LaunchConfiguration('zed_x')
    zed_y = LaunchConfiguration('zed_y')
    zed_yaw = LaunchConfiguration('zed_yaw')
    pose_alpha = LaunchConfiguration('pose_alpha')
    yaw_alpha = LaunchConfiguration('yaw_alpha')
    yaw_deadband = LaunchConfiguration('yaw_deadband')
    max_yaw_rate = LaunchConfiguration('max_yaw_rate')

    input_scan_topic = LaunchConfiguration('input_scan_topic')
    scan_flat_topic = LaunchConfiguration('scan_flat_topic')
    laser_frame = LaunchConfiguration('laser_frame')
    require_odom_before_scan = LaunchConfiguration('require_odom_before_scan')

    # base_footprint -> base_link
    bl_x = LaunchConfiguration('base_link_x')
    bl_y = LaunchConfiguration('base_link_y')
    bl_z = LaunchConfiguration('base_link_z')
    # base_link -> base_laser
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # ── ZED planar odom source ───────────────────────────────────────
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('filtered_odom_topic', default_value='/filtered_odom'),
        DeclareLaunchArgument(
            'publish_zed_odom_tf', default_value='false',
            description='Keep FALSE in production (EKF owns odom->base_footprint). '
                        'Set true only for a ZED-only no-EKF fallback.'),
        DeclareLaunchArgument('zed_x', default_value='0.0'),
        DeclareLaunchArgument('zed_y', default_value='0.0'),
        DeclareLaunchArgument('zed_yaw', default_value='0.0'),
        DeclareLaunchArgument('pose_alpha', default_value='0.35'),
        DeclareLaunchArgument('yaw_alpha', default_value='0.12'),
        DeclareLaunchArgument('yaw_deadband', default_value='0.015'),
        DeclareLaunchArgument('max_yaw_rate', default_value='0.6'),

        # ── Scan reframing ───────────────────────────────────────────────
        DeclareLaunchArgument('input_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_flat_topic', default_value='/scan_flat'),
        DeclareLaunchArgument('laser_frame', default_value='base_laser'),
        DeclareLaunchArgument(
            'require_odom_before_scan', default_value='true',
            description='Drop scans until /filtered_odom is available so SLAM/AMCL '
                        'never receive a scan before the odom TF path exists.'),

        # ── base_footprint -> base_link (planar base: identity by default) ─
        DeclareLaunchArgument('base_link_x', default_value='0.0'),
        DeclareLaunchArgument('base_link_y', default_value='0.0'),
        DeclareLaunchArgument('base_link_z', default_value='0.0'),

        # ── base_link -> base_laser (A1 LiDAR extrinsics) ────────────────
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.05'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='3.14159'),

        Node(
            package='dicerox_mapping',
            executable='zed_planar_odom_node.py',
            name='zed_planar_odom',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'input_odom_topic': zed_odom_topic,
                'output_odom_topic': filtered_odom_topic,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'zed_x': ParameterValue(zed_x, value_type=float),
                'zed_y': ParameterValue(zed_y, value_type=float),
                'zed_yaw': ParameterValue(zed_yaw, value_type=float),
                'pose_alpha': ParameterValue(pose_alpha, value_type=float),
                'yaw_alpha': ParameterValue(yaw_alpha, value_type=float),
                'yaw_deadband': ParameterValue(yaw_deadband, value_type=float),
                'max_yaw_rate': ParameterValue(max_yaw_rate, value_type=float),
                # publish_tf is the node's TF switch; keep it off in fusion mode.
                'publish_tf': ParameterValue(publish_zed_odom_tf, value_type=bool),
            }],
        ),

        Node(
            package='dicerox_mapping',
            executable='scan_frame_republisher.py',
            name='scan_frame_republisher',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'input_scan_topic': input_scan_topic,
                'output_scan_topic': scan_flat_topic,
                'output_frame': laser_frame,
                'filtered_odom_topic': filtered_odom_topic,
                'require_odom_before_publish': ParameterValue(
                    require_odom_before_scan, value_type=bool),
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_tf_publisher',
            arguments=[
                '--x', bl_x, '--y', bl_y, '--z', bl_z,
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'base_link',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_laser_tf_publisher',
            arguments=[
                '--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                '--roll', lidar_roll, '--pitch', lidar_pitch, '--yaw', lidar_yaw,
                '--frame-id', 'base_link',
                '--child-frame-id', laser_frame,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
