import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """joy_node + robot_manager_node (expects motor_manager_node already publishing motor_state)."""

    pkg_share = get_package_share_directory('motion_system_pkg')

    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='Joystick index for joy_node (/dev/input/js{N}).',
    )
    deadzone_arg = DeclareLaunchArgument(
        'deadzone',
        default_value='0.05',
        description='Axis deadzone for joy_node.',
    )
    autorepeat_arg = DeclareLaunchArgument(
        'autorepeat_rate',
        default_value='20.0',
        description='Autorepeat rate (Hz) for joy_node.',
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'silver_lain.yaml'),
        description='YAML for robot_manager_node (keys: robot, dt, stride_length, …).',
    )
    stride_length_arg = DeclareLaunchArgument(
        'stride_length',
        default_value='0.1',
        description='Nominal stride length (m) for walk duration = stick speed / stride_length.',
    )

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'joy.launch.py'),
        ),
        launch_arguments={
            'device_id': LaunchConfiguration('device_id'),
            'deadzone': LaunchConfiguration('deadzone'),
            'autorepeat_rate': LaunchConfiguration('autorepeat_rate'),
        }.items(),
    )

    robot_manager = Node(
        package='motion_system_pkg',
        executable='robot_manager_node.py',
        name='robot_manager_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'stride_length': LaunchConfiguration('stride_length'),
        }],
    )

    return LaunchDescription([
        device_id_arg,
        deadzone_arg,
        autorepeat_arg,
        config_file_arg,
        stride_length_arg,
        joy_launch,
        robot_manager,
    ])
