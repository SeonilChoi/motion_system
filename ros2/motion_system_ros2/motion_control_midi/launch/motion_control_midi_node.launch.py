from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('motion_control_bridge'),
        'config',
        'example_socketcan_cubemars.yaml',
    ])

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Absolute path to motion_control_bridge YAML config.',
    )
    midi_topic_arg = DeclareLaunchArgument(
        'midi_topic',
        default_value='/xtouch/midi',
        description='Full X-Touch MIDI state topic.',
    )
    command_topic_arg = DeclareLaunchArgument(
        'command_topic',
        default_value='/motor_command',
        description='MotorStatus command topic consumed by motion_control_bridge.',
    )
    publish_period_ms_arg = DeclareLaunchArgument(
        'publish_period_ms',
        default_value='5',
        description='Motor command publish period in milliseconds.',
    )
    max_smoothing_time_ms_arg = DeclareLaunchArgument(
        'max_smoothing_time_ms',
        default_value='3000.0',
        description='Dial=127 smoothing time constant in milliseconds.',
    )
    smoothing_curve_power_arg = DeclareLaunchArgument(
        'smoothing_curve_power',
        default_value='2.0',
        description='Nonlinear dial curve; larger values make high dial values smoother.',
    )

    return LaunchDescription([
        config_file_arg,
        midi_topic_arg,
        command_topic_arg,
        publish_period_ms_arg,
        max_smoothing_time_ms_arg,
        smoothing_curve_power_arg,
        Node(
            package='motion_control_midi',
            executable='motion_control_midi_node',
            name='motion_control_midi_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'midi_topic': LaunchConfiguration('midi_topic'),
                'command_topic': LaunchConfiguration('command_topic'),
                'publish_period_ms': ParameterValue(
                    LaunchConfiguration('publish_period_ms'),
                    value_type=int,
                ),
                'max_smoothing_time_ms': ParameterValue(
                    LaunchConfiguration('max_smoothing_time_ms'),
                    value_type=float,
                ),
                'smoothing_curve_power': ParameterValue(
                    LaunchConfiguration('smoothing_curve_power'),
                    value_type=float,
                ),
            }],
        ),
    ])
