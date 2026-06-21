import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mlx90640_ros2'), 'config', 'mlx90640.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_dummy', default_value='false'),
        Node(
            package='mlx90640_ros2',
            executable='mlx90640_node',
            name='mlx90640_node',
            output='screen',
            parameters=[config, {'use_dummy': LaunchConfiguration('use_dummy')}],
        ),
        Node(
            package='mlx90640_ros2',
            executable='thermal_visualizer_node',
            name='thermal_visualizer_node',
            output='screen',
            parameters=[config],
        ),
    ])
