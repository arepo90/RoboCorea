"""mapping_ekf.launch.py — SLAM with switchable odometry source.

PROD (use_ekf:=true, the default): the robot_localization EKF (wheel + ZED VIO +
ZED IMU) drives odom -> base_footprint. This is what you want on the real robot,
driving on its tracks with the bridge running.

BENCH (use_ekf:=false): skip the EKF/adaptive layer and let zed_planar_odom own
odom -> base_footprint directly (planar ZED VIO). Use this when there are no
tracks/bridge (so wheel odom is absent) or you're moving the ZED by hand — the
planar ground-robot EKF mis-models that and adds jank. The ZED odom is already
visual-INERTIAL (it folds the IMU in itself), so you don't lose the IMU here.

It:
  * includes dicerox_mapping/mapping.launch.py. In PROD it runs with
    publish_odom_tf:=false (zed_planar_odom is a pure /filtered_odom *source*, no
    TF); in BENCH it runs with publish_odom_tf:=true (zed_planar_odom owns the TF).
  * includes odom_fusion.launch.py (adaptive node + EKF) ONLY in PROD; that node
    then becomes the sole owner of odom -> base_footprint.

Resulting TF tree (unchanged for everyone downstream):
    map -> odom -> base_footprint -> base_laser
            ^         ^
            |         └─ EKF (prod)  OR  zed_planar_odom (bench)
            └─ slam_toolbox

The systemd unit (rescue-mapping.service) reads use_ekf from the
ROBOCOREA_USE_EKF env var (default true), so the GUI's Start SLAM button can drive
either mode without editing the unit — see rescue_bringup/systemd/README.md.

IMPORTANT: run the ZED driver with publish_tf:=false too (it must not publish
odom->base either). See architecture §18 item 12.

Args of note:
  use_ekf          : true (EKF, prod) | false (ZED-direct, bench).
  slam_params_file : config/slam_toolbox_localization.yaml to LOCALIZE against a
                     saved map instead of building one (mapping default).
  use_rviz         : forwarded to mapping.launch.py.
  imu_yaw_sign     : forwarded to the fusion layer (flip if gyro sign inverted).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    mapping_share = get_package_share_directory('dicerox_mapping')
    nav_share = get_package_share_directory('rescue_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_ekf = LaunchConfiguration('use_ekf')
    slam_params_file = LaunchConfiguration('slam_params_file')
    zed_odom_topic = LaunchConfiguration('zed_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    imu_yaw_sign = LaunchConfiguration('imu_yaw_sign')

    mapping_launch = os.path.join(mapping_share, 'launch', 'mapping.launch.py')
    fusion_launch = os.path.join(nav_share, 'launch', 'odom_fusion.launch.py')

    # zed_planar_odom owns the odom TF only in BENCH (use_ekf=false); in PROD the
    # EKF owns it, so the planar node must NOT publish TF.
    publish_odom_tf = PythonExpression(
        ["'false' if '", use_ekf, "'.lower() in ('true', '1', 'yes') else 'true'"])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description='true = EKF fusion owns odom (prod); '
                        'false = zed_planar_odom owns odom directly (bench/ZED-only).'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(mapping_share, 'config', 'slam_toolbox_params.yaml'),
            description='Use slam_toolbox_localization.yaml to localize against a saved map.'),
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/zed/zed_node/imu/data'),
        DeclareLaunchArgument('imu_yaw_sign', default_value='1.0'),

        # Mapping/localization + planar ZED source + scan reframe + slam_toolbox.
        # publish_odom_tf is the inverse of use_ekf (see above).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_rviz': use_rviz,
                'slam_params_file': slam_params_file,
                'zed_odom_topic': zed_odom_topic,
                'publish_odom_tf': publish_odom_tf,
            }.items(),
        ),

        # The fusion layer: adaptive covariance + EKF (owns odom -> base_footprint).
        # PROD only — skipped entirely in bench mode (zed_planar_odom owns the TF).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fusion_launch),
            condition=IfCondition(use_ekf),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'wheel_odom_topic': '/odom/wheel',
                'zed_odom_topic': '/filtered_odom',
                'imu_topic': imu_topic,
                'imu_yaw_sign': imu_yaw_sign,
            }.items(),
        ),
    ])
