"""
save_competition_map.launch.py — save the map built in Scenario A.

Saves BOTH artifacts that the two navigation back-ends need:

  * <map_name>.pgm + <map_name>.yaml   — occupancy grid (nav2 map_server / AMCL,
                                          via nav2_map_server map_saver_cli on /map)
  * <map_name>.posegraph + .data       — slam_toolbox serialized graph (for the
                                          localization:=slam_toolbox back-end)

Run this WHILE real_mapping.launch.py is still running (slam_toolbox must be up
to answer the serialize service and to be publishing /map).

Usage:
    ros2 launch rescue_nav save_competition_map.launch.py
    ros2 launch rescue_nav save_competition_map.launch.py map_name:=$HOME/maps/arena1

Then confirm:
    ls -l <map_name>.pgm <map_name>.yaml <map_name>.posegraph <map_name>.data
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_name = LaunchConfiguration('map_name')
    map_topic = LaunchConfiguration('map_topic')

    default_map = os.path.join(os.path.expanduser('~'), 'maps', 'competition_map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name', default_value=default_map,
            description='Output basename (no extension) for all map artifacts.'),
        DeclareLaunchArgument(
            'map_topic', default_value='/map',
            description='Occupancy grid topic the map_saver subscribes to.'),

        LogInfo(msg=['[save_competition_map] Saving occupancy grid + posegraph to: ',
                     map_name]),

        # 1) Occupancy grid (.pgm/.yaml) for nav2 map_server / AMCL.
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', map_name, '-t', map_topic,
                 '--ros-args', '-p', 'save_map_timeout:=10.0'],
            output='screen',
        ),

        # 2) slam_toolbox serialized posegraph (.posegraph/.data) for the
        #    localization:=slam_toolbox back-end. Harmless if slam_toolbox is not
        #    running (the service call simply fails and is logged).
        ExecuteProcess(
            cmd=['ros2', 'service', 'call',
                 '/slam_toolbox/serialize_map',
                 'slam_toolbox/srv/SerializePoseGraph',
                 ['{filename: "', map_name, '"}']],
            output='screen',
        ),
    ])
