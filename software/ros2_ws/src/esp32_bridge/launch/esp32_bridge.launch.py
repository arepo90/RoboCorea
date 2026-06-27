from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='',
            description='Optional explicit ESP32 serial device; empty means scan serial_candidates.'),
        DeclareLaunchArgument(
            'serial_candidates', default_value='/dev/ttyCH341USB*,/dev/serial/by-id/*,/dev/serial/by-path/*',
            description='Comma-separated allowlist of serial device paths/globs to probe for ESP32 boards. '
                        'ttyCH341USB* is the Jetson WCH CH341 driver name for the ESP32 CH340 UARTs.'),
        DeclareLaunchArgument(
            'baud_rate', default_value='921600',
            description='UART baud rate (must match MINIPC_BAUD in config.h)'),
        DeclareLaunchArgument(
            'gear_ratio', default_value='23.333',
            description='Traction motor-to-track output reduction for VESC tachometer odometry.'),
        Node(
            package='esp32_bridge',
            executable='esp32_bridge',
            name='esp32_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_candidates': LaunchConfiguration('serial_candidates'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'gear_ratio': ParameterValue(LaunchConfiguration('gear_ratio'), value_type=float),
            }],
        ),
    ])
