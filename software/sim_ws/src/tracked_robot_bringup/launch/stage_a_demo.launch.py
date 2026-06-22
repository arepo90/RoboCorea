"""
stage_a_demo.launch.py — Single-command full Stage A demo launch.

Starts in order:
  1. Gazebo Fortress  (world: stage_a.sdf)
  2. robot_state_publisher + joint_state_publisher  (TF tree)
  3. ros_gz_bridge  (/clock, /scan_raw, /cmd_vel)
  4. Robot spawn  (dicerox model → Gazebo, delayed 3 s)
  5. scan_sanitizer_node  (/scan_raw → filtered /scan)
  6. tracked_odom_node  (cmd_vel → /odom + TF odom→base_footprint)
  7. Nav2 stack  (map_server, AMCL, planners, controllers, BT)
  8. RViz2  (optional, default: on)

Usage:
    ros2 launch tracked_robot_bringup stage_a_demo.launch.py
    ros2 launch tracked_robot_bringup stage_a_demo.launch.py headless:=true use_rviz:=false
    ros2 launch tracked_robot_bringup stage_a_demo.launch.py x:=1.0 y:=0.5 yaw:=1.57

Arguments:
    headless   (bool, default false)  — run Gazebo without GUI
    use_rviz   (bool, default true)   — launch RViz
    x          (float, default 0.0)   — robot spawn X
    y          (float, default 0.0)   — robot spawn Y
    yaw        (float, default 0.0)   — robot spawn yaw (rad)
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def get_required_share_directory(package_name):
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError as exc:
        instructions = (
            f"Required package '{package_name}' was not found. Stage A uses "
            "Dicerox model packages from ros2_moveit_ws. In every new terminal, "
            "source both workspaces in this order:\n"
            "  cd ~/Projects/PhD/Dicerox_PhD_Core\n"
            "  source /opt/ros/humble/setup.bash\n"
            "  source ros2_moveit_ws/install/setup.bash\n"
            "  source ros2_ws/install/setup.bash\n"
            "Then launch Stage A again."
        )
        raise RuntimeError(instructions) from exc


def generate_launch_description():
    # ── Package share directories ─────────────────────────────────────────
    pkg_desc    = get_package_share_directory('tracked_robot_description')
    pkg_gz      = get_package_share_directory('tracked_robot_gz')
    pkg_nav2    = get_package_share_directory('tracked_robot_nav2')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_ros_gz  = get_package_share_directory('ros_gz_sim')

    # ── Gazebo resource paths — lets Gazebo resolve package:// and model:// meshes ──
    # ros_gz_sim can rewrite package:// URIs to model:// URIs during URDF -> SDF
    # conversion. Add the share parents for the Dicerox chassis and arm packages
    # imported from ros2_moveit_ws so Ignition/Gazebo can find their STL meshes.
    gazebo_resource_paths = [pkg_gz, os.path.join(pkg_gz, 'models')]
    for package_name in ('dicerox_urdf_v1', 'dicerox_arm_urdf'):
        package_share = get_required_share_directory(package_name)
        gazebo_resource_paths.extend([package_share, os.path.dirname(package_share)])

    existing_ign = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    existing_gz = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ign_resource_val = ':'.join(filter(None, gazebo_resource_paths + [existing_ign]))
    gz_resource_val = ':'.join(filter(None, gazebo_resource_paths + [existing_gz]))
    set_ign_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', ign_resource_val
    )
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', gz_resource_val
    )

    # ── File paths ────────────────────────────────────────────────────────
    xacro_file   = os.path.join(pkg_desc,  'urdf',   'dicerox_stage_a.urdf.xacro')
    world_file   = os.path.join(pkg_gz,    'worlds', 'stage_a.sdf')
    bridge_yaml  = os.path.join(pkg_gz,    'config', 'bridge.yaml')
    map_yaml     = os.path.join(pkg_nav2,  'maps',   'stage_a.yaml')
    params_file  = os.path.join(pkg_nav2,  'config', 'nav2_params.yaml')
    rviz_config  = os.path.join(pkg_nav2,  'rviz',   'stage_a.rviz')

    # ── Launch arguments ──────────────────────────────────────────────────
    headless  = LaunchConfiguration('headless')
    use_rviz  = LaunchConfiguration('use_rviz')
    spawn_x   = LaunchConfiguration('x')
    spawn_y   = LaunchConfiguration('y')
    spawn_yaw = LaunchConfiguration('yaw')

    declare_args = [
        DeclareLaunchArgument('headless',  default_value='false',
                              description='Run Gazebo server only (no GUI).'),
        DeclareLaunchArgument('use_rviz',  default_value='true',
                              description='Launch RViz.'),
        DeclareLaunchArgument('x',         default_value='0.0',
                              description='Robot spawn X (m).'),
        DeclareLaunchArgument('y',         default_value='0.0',
                              description='Robot spawn Y (m).'),
        DeclareLaunchArgument('yaw',       default_value='0.0',
                              description='Robot spawn yaw (rad).'),
    ]

    # ── 1. Robot description ─────────────────────────────────────────────
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    # ── 2. Gazebo Fortress (with GUI) ─────────────────────────────────────
    gz_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        condition=UnlessCondition(headless),
    )

    # ── 2b. Gazebo Fortress (headless) ────────────────────────────────────
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -s {world_file}'}.items(),
        condition=IfCondition(headless),
    )

    # ── 3. robot_state_publisher ─────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ── 4. joint_state_publisher (Stage A: all joints at zero) ───────────
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'rate': 50,
        }],
    )

    # ── 5. ros_gz_bridge (/clock, /scan_raw, /cmd_vel) ────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_yaml,
            'use_sim_time': True,
        }],
    )

    # ── 6. Scan sanitizer ────────────────────────────────────────────────
    # Gazebo GPU lidar can emit finite max-range endpoints and self-hits.
    # Nav2 and RViz consume /scan after this cleanup step.
    scan_sanitizer = Node(
        package='tracked_robot_odom',
        executable='scan_sanitizer_node',
        name='scan_sanitizer_node',
        output='screen',
        parameters=[{
            'input_topic': '/scan_raw',
            'output_topic': '/scan',
            'self_clearance_range': 0.65,
            'max_range_epsilon': 0.05,
            'drop_max_range_returns': True,
            'use_sim_time': True,
        }],
    )

    # ── 6. Spawn robot (delayed for Gazebo GUI/world/resource loading) ────
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_dicerox',
                output='screen',
                arguments=[
                    '-name',  'dicerox',
                    '-allow_renaming', 'false',
                    '-topic', '/robot_description',
                    '-x',     spawn_x,
                    '-y',     spawn_y,
                    '-z',     '0.08',    # spawn just above ground to reduce settling pitch
                    '-Y',     spawn_yaw,
                ],
            ),
        ],
    )

    # ── 7. Tracked odometry node ──────────────────────────────────────────
    # Delayed 2 s so the ros_gz_bridge /clock is flowing before this node
    # creates its timer. Without this delay rclpy's ROSClock falls back to
    # wall time before the first /clock message, the timer fires at wall
    # time, and the initial odom→base_footprint TF is stamped at the Unix
    # epoch (~1.78e9 s). Because wall time >> sim time, TF2 treats the
    # wall-clock entry as the "newest" and permanently rejects every
    # subsequent sim-time entry as TF_OLD_DATA.
    tracked_odom = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='tracked_robot_odom',
                executable='tracked_odom_node',
                name='tracked_odom_node',
                output='screen',
                parameters=[{
                    'track_separation': 0.530,
                    'publish_rate':     50.0,
                    'odom_frame':       'odom',
                    'base_frame':       'base_footprint',
                    'use_sim_time':     True,
                }],
            ),
        ],
    )

    # ── 8. Nav2 stack ─────────────────────────────────────────────────────
    # Delayed 8 s — after the robot spawns (5 s) and /clock has been steadily
    # flowing — so AMCL's lifecycle activation publishes its first map→odom
    # transform with valid sim time. If AMCL activates before /clock has
    # synced in its (freshly started) process, it stamps that transform at
    # wall time (~1.78e9 s), which permanently poisons the tf2 buffer and
    # makes every later sim-time lookup fail as TF_OLD_DATA / extrapolation.
    # Gazebo under software rendering is slow to boot, so this margin matters.
    nav2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map':          map_yaml,
                    'params_file':  params_file,
                    'use_sim_time': 'true',
                    'autostart':    'true',
                }.items(),
            ),
        ],
    )


    # ── 9. RViz ──────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declare_args + [
            set_ign_resource_path,   # must come before Gazebo starts
            set_gz_resource_path,
            gz_with_gui,
            gz_headless,
            robot_state_publisher,
            joint_state_publisher,
            bridge,
            scan_sanitizer,
            spawn_robot,
            tracked_odom,
            nav2,
            rviz,
        ]
    )
