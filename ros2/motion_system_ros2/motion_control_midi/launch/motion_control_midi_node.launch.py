import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motion_control_bridge_pkg_share = get_package_share_directory('motion_control_bridge')
    motor_config_file = os.path.join(
        motion_control_bridge_pkg_share,
        'config',
        'example_socketcan_cubemars.yaml',
    )

    return LaunchDescription([
        Node(
            package='motion_control_bridge',
            executable='motor_manager_node',
            name='motor_manager_node',
            output='screen',
            parameters=[{
                'config_file': motor_config_file,
            }],
        ),
        Node(
            package='xtouch_midi',
            executable='xtouch_node',
            name='xtouch_node',
            output='screen',
            parameters=[{
                'btn3_requires_fader_update': True,
            }],
        ),
        Node(
            package='motion_control_midi',
            executable='motion_control_midi_node',
            name='motion_control_midi_node',
            output='screen',
        ),
    ])
