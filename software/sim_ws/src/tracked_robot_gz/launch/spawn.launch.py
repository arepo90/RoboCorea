"""
spawn.launch.py — Spawn the Dicerox robot into a running Gazebo Fortress world.

Responsibilities:
  1. Load and publish robot_description (xacro → URDF → topic).
  2. Spawn the robot model in Gazebo via ros_gz_sim create.
  3. Start robot_state_publisher (TF for fixed joints and flipper frames).
  4. Start joint_state_publisher  (publishes zero joint states — Stage A).
  5. Start ros_gz_bridge          (clock, /scan_raw, /cmd_vel).
  6. Start scan_sanitizer_node   (/scan_raw -> /scan).

Usage (standalone — requires Gazebo already running):
    ros2 launch tracked_robot_gz spawn.launch.py
    ros2 launch tracked_robot_gz spawn.launch.py x:=1.0 y:=0.0 yaw:=0.0
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
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
    pkg_desc = get_package_share_directory('tracked_robot_description')
    pkg_gz   = get_package_share_directory('tracked_robot_gz')

    dicerox_model_share_parents = []
    for package_name in ('dicerox_urdf_v1', 'dicerox_arm_urdf'):
        package_share = get_required_share_directory(package_name)
        dicerox_model_share_parents.append(os.path.dirname(package_share))
    existing_ign = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ign_resource_val = ':'.join(filter(None, dicerox_model_share_parents + [existing_ign]))
    set_ign_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', ign_resource_val
    )

    xacro_file   = os.path.join(pkg_desc, 'urdf', 'dicerox_stage_a.urdf.xacro')
    bridge_yaml  = os.path.join(pkg_gz,   'config', 'bridge.yaml')

    # ── Spawn pose arguments ────────────────────────────────────────────────
    x   = LaunchConfiguration('x')
    y   = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    declare_x   = DeclareLaunchArgument('x',   default_value='0.0',  description='Spawn X position')
    declare_y   = DeclareLaunchArgument('y',   default_value='0.0',  description='Spawn Y position')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0',  description='Spawn yaw (radians)')

    # ── Robot description (xacro → URDF string) ─────────────────────────────
    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    # ── robot_state_publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    # ── joint_state_publisher (Stage A: publishes zero for all joints) ──────
    # In Stage B, replace with a node that reads actual Gazebo joint states.
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

    # ── Spawn robot into Gazebo (delayed 3 s to let the world fully load) ───
    # ros_gz_sim create reads /robot_description topic published by RSP above.
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_dicerox',
                output='screen',
                arguments=[
                    '-name',  'dicerox',
                    '-topic', '/robot_description',
                    '-x',     x,
                    '-y',     y,
                    # z: set slightly above ground to avoid penetration at spawn
                    '-z',     '0.08',
                    '-Y',     yaw,
                ],
            ),
        ],
    )

    # ── ros_gz_bridge (clock, scan_raw, cmd_vel) ────────────────────────────
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


    return LaunchDescription([
        set_ign_resource_path,
        declare_x,
        declare_y,
        declare_yaw,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        bridge,
        scan_sanitizer,
    ])
