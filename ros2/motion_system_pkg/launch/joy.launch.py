from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Publishes sensor_msgs/Joy from a Linux joystick (/dev/input/js*)."""

    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='Index passed to joy_node (matches /dev/input/js{N}).',
    )
    deadzone_arg = DeclareLaunchArgument(
        'deadzone',
        default_value='0.05',
        description='Axis deadzone for joy_node.',
    )
    autorepeat_arg = DeclareLaunchArgument(
        'autorepeat_rate',
        default_value='20.0',
        description='Autorepeat rate (Hz); 0.0 disables.',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('device_id'),
            'deadzone': LaunchConfiguration('deadzone'),
            'autorepeat_rate': LaunchConfiguration('autorepeat_rate'),
        }],
    )

    return LaunchDescription([
        device_id_arg,
        deadzone_arg,
        autorepeat_arg,
        joy_node,
    ])
