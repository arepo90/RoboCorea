"""
real_navigation.launch.py — SCENARIO B: navigate a KNOWN map (REAL robot).

Loads a map saved in Scenario A, localizes on it, and runs Nav2 to drive the
tracked base to a requested 2D goal — all on FUSED odometry.

Pipeline:
    sensor_frontend     -> /filtered_odom, /scan_flat, static base_link/base_laser TFs
    odom_fusion (EKF)   -> /odometry/filtered + odom->base_footprint TF
    localization        -> map->odom TF
        localization:=amcl          (DEFAULT) nav2 map_server + AMCL
        localization:=slam_toolbox            slam_toolbox in localization mode
    Nav2 (navigation)   -> /cmd_vel  (consumed by esp32_bridge -> VESC traction)

TF ownership (exactly one publisher per edge):
    map -> odom              : AMCL  (or slam_toolbox)        — selected localization
    odom -> base_footprint   : EKF   (or ZED planar)          — odom_tf_owner
    base_footprint->base_link->base_laser : static            — sensor_frontend

SAFETY: launching this does NOT move the robot. Nav2 only publishes /cmd_vel;
the wheels move only once you (a) run esp32_bridge with
enable_cmd_vel_drive:=true AND (b) send a goal. Until then esp32_bridge stays in
RC mode. See the "where /cmd_vel reaches the base" note in COMPETITION_WORKFLOWS.

Sensor prerequisites (start FIRST, own terminals):
    ZED wrapper with publish_tf:=false
    Slamtec A1 driver publishing /scan
    ros2 launch esp32_bridge esp32_bridge.launch.py   (EKF /odom/wheel; add
        enable_cmd_vel_drive:=true ONLY when you want Nav2 to actually drive)

Key args:
    map                       (REQUIRED for localization:=amcl) path to <map>.yaml
    localization   (amcl)     'amcl' | 'slam_toolbox'
    nav            (true)     include Nav2; false = localization-only (no Nav2)
    params_file               nav2_real_params.yaml (AMCL + map_server + Nav2)
    slam_map_file             posegraph basename (no ext) for slam_toolbox path
    use_sim_time   (false)
    use_rviz       (true)
    namespace      ('')
    odom_tf_owner  (ekf)      'ekf' | 'zed'
    set_initial_pose (true)   publish initial_pose_* to /initialpose once (AMCL)
    initial_pose_x/y/yaw (0)  initial pose in the map frame
    imu_topic / imu_yaw_sign / zed_odom_topic — forwarded to the fusion layer
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _setup(context, *args, **kwargs):
    nav_share = get_package_share_directory('rescue_nav')
    mapping_share = get_package_share_directory('dicerox_mapping')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    imu_topic = LaunchConfiguration('imu_topic')
    imu_yaw_sign = LaunchConfiguration('imu_yaw_sign')
    zed_odom_topic = LaunchConfiguration('zed_odom_topic')

    # Resolved (string) values we need to branch on at launch time.
    owner = LaunchConfiguration('odom_tf_owner').perform(context)
    localization = LaunchConfiguration('localization').perform(context)
    slam_map_file = LaunchConfiguration('slam_map_file').perform(context)
    use_ekf = owner == 'ekf'
    zed_publishes_tf = 'true' if owner == 'zed' else 'false'

    frontend_launch = os.path.join(nav_share, 'launch', 'sensor_frontend.launch.py')
    fusion_launch = os.path.join(nav_share, 'launch', 'odom_fusion.launch.py')

    actions = [PushRosNamespace(namespace)]

    # 1) Perception/odometry front-end.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(frontend_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'zed_odom_topic': zed_odom_topic,
            'publish_zed_odom_tf': zed_publishes_tf,
        }.items(),
    ))

    # 2) Fused odometry (EKF) — owns odom->base_footprint in the default path.
    if use_ekf:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fusion_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'wheel_odom_topic': '/odom/wheel',
                'zed_odom_topic': '/filtered_odom',
                'imu_topic': imu_topic,
                'imu_yaw_sign': imu_yaw_sign,
            }.items(),
        ))

    # 3) Localization — owns map->odom.
    if localization == 'slam_toolbox':
        loc_params = os.path.join(
            mapping_share, 'config', 'slam_toolbox_localization.yaml')
        slam_params = {'use_sim_time': use_sim_time, 'scan_topic': '/scan_flat'}
        if slam_map_file:
            slam_params['map_file_name'] = slam_map_file
        actions.append(Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[loc_params, slam_params],
        ))
    else:  # 'amcl' (default): nav2 map_server + AMCL
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'localization_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
            }.items(),
        ))

    # 4) Nav2 navigation stack — controller/planner/BT/smoother -> /cmd_vel.
    #    Gated by 'nav' so this launch can run LOCALIZATION-ONLY (nav:=false):
    #    rescue-localization.service uses that to localize on a loaded map without
    #    bringing Nav2 up; rescue-navigation.service starts Nav2 separately on top
    #    (nav2.launch.py) when the operator explicitly starts the Navigation stack.
    include_nav = LaunchConfiguration('nav').perform(context).lower() == 'true'
    if include_nav:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
            }.items(),
        ))

    # 5) One-shot initial pose for AMCL (after it activates). Skipped for
    #    slam_toolbox (which relocalizes from map_start_pose / scan matching).
    if localization == 'amcl' and \
            LaunchConfiguration('set_initial_pose').perform(context).lower() == 'true':
        ix = float(LaunchConfiguration('initial_pose_x').perform(context))
        iy = float(LaunchConfiguration('initial_pose_y').perform(context))
        iyaw = float(LaunchConfiguration('initial_pose_yaw').perform(context))
        qz = math.sin(iyaw / 2.0)
        qw = math.cos(iyaw / 2.0)
        pose_msg = (
            "{header: {frame_id: 'map'}, pose: {pose: {"
            f"position: {{x: {ix}, y: {iy}, z: 0.0}}, "
            f"orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}, "
            "covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, "
            "0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
        )
        # Published once, delayed so AMCL has activated and is listening.
        actions.append(TimerAction(period=6.0, actions=[_initialpose_proc(pose_msg)]))

    # 6) Pre-flight validator. Runs with localization (nav:=false) too, so the GUI
    #    has a readiness signal the moment a map is loaded — BEFORE Nav2 is started.
    #    Publishes /nav/preflight (latched) + serves /nav/preflight_check (Trigger).
    if LaunchConfiguration('preflight').perform(context).lower() == 'true':
        actions.append(Node(
            package='rescue_nav', executable='nav_preflight', name='nav_preflight',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'scan_topic': '/scan_flat',
                'odom_topic': '/odometry/filtered',
                'cmd_vel_topic': '/cmd_vel',
                'map_topic': '/map',
                'global_frame': 'map',
                'laser_frame': 'base_laser',
            }],
        ))

    # 7) RViz (optional).
    actions.append(Node(
        condition=IfCondition(use_rviz),
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(nav_share, 'rviz', 'nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    ))

    return [GroupAction(actions)]


def _initialpose_proc(pose_msg):
    from launch.actions import ExecuteProcess
    return ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/initialpose',
             'geometry_msgs/msg/PoseWithCovarianceStamped', pose_msg],
        output='screen',
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Path to <map>.yaml (REQUIRED for localization:=amcl).'),
        DeclareLaunchArgument(
            'localization', default_value='amcl',
            description="'amcl' (nav2 map_server+AMCL, default) | 'slam_toolbox'."),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('rescue_nav'),
                'config', 'nav2_real_params.yaml'),
            description='Nav2 real params (contains amcl + map_server + nav stack).'),
        DeclareLaunchArgument(
            'slam_map_file', default_value='',
            description='Posegraph basename (no extension) for localization:=slam_toolbox.'),
        DeclareLaunchArgument(
            'odom_tf_owner', default_value='ekf',
            description="'ekf' (fused, default) | 'zed' (ZED-only, no EKF)."),
        DeclareLaunchArgument(
            'nav', default_value='true',
            description="Include the Nav2 navigation stack. Set false for "
                        "LOCALIZATION-ONLY (map_server+AMCL+EKF, no Nav2) — used by "
                        "rescue-localization.service; rescue-navigation.service then "
                        "runs Nav2 on top via nav2.launch.py."),
        DeclareLaunchArgument(
            'preflight', default_value='true',
            description='Run the nav_preflight readiness validator '
                        '(/nav/preflight + /nav/preflight_check).'),
        DeclareLaunchArgument('set_initial_pose', default_value='true'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/zed/zed_node/imu/data'),
        DeclareLaunchArgument('imu_yaw_sign', default_value='1.0'),
        OpaqueFunction(function=_setup),
    ])
