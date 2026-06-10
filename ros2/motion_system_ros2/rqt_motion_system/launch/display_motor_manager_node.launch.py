from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    ros2_motor_manager_pkg_share = get_package_share_directory('ros2_motor_manager')
    default_motor_config = os.path.join(ros2_motor_manager_pkg_share, 'config', 'example_ethercat_zeroerr.yaml')

    motor_config_file_arg = DeclareLaunchArgument(
        'motor_config_file',
        default_value=default_motor_config,
        description='Absolute path to motor_manager YAML (masters / drivers).',
    )

    motor_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros2_motor_manager_pkg_share, 'launch', 'motor_manager_node.launch.py'),
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('motor_config_file'),
        }.items(),
    )

    motion_system_controller = Node(
        package='rqt_motion_system',
        executable='run_rqt',
        name='motion_system_controller',
        output='screen',
        arguments=['--force-discover'],
        parameters=[{
            'config_file': LaunchConfiguration('motor_config_file'),
        }],
    )

    return LaunchDescription([
        motor_config_file_arg,
        motor_manager_launch,
        motion_system_controller,
    ])
