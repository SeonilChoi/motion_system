import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Int8MultiArray
from motion_control_msgs.msg import MotorStatus

from common_robot_interface.joint_frame import joint_frame_t
from common_robot_interface.state_frame import State, state_frame_t
from common_robot_interface.action_frame import Action, action_frame_t

from robot_manager.robot_manager import RobotManager

JOY_BUTTON_MAX = 10

JOY_BUTTON_SQUARE = 0
JOY_BUTTON_CIRCLE = 1
JOY_BUTTON_CROSS = 2

JOY_BUTTON_DPAD_LEFT = 4
JOY_BUTTON_DPAD_RIGHT = 5


class RobotManagerNode(Node):
    def __init__(self):
        super().__init__('robot_manager_node')

        self.config_file = (
            self.declare_parameter(
                'config_file',
                'src/ros2/motion_system_ros2/motion_control_robot/config/rocking_chair.yaml',
            )
            .get_parameter_value()
            .string_value
        )
        self.robot_manager = RobotManager(self.config_file)
        self.number_of_robots = self.robot_manager.number_of_robots
        self.dt = self.robot_manager.dt

        self.motor_status_subscriber = self.create_subscription(
            MotorStatus,
            'motion_control/motor_status',
            self.motor_status_callback,
            10,
        )
        self.motor_command_publisher = self.create_publisher(
            MotorStatus,
            'motion_control/motor_command',
            10,
        )

        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10,
        )
        self.timer = self.create_wall_timer(
            self.dt,
            self.timer_callback,
        )

        self.joy_buttons: list[bool] = [False] * JOY_BUTTON_MAX
        self.joy_buttons_prev: list[bool] = [False] * JOY_BUTTON_MAX
        self.joy_button_action: dict[int, int] = {
            JOY_BUTTON_SQUARE: Action.HOME,
            JOY_BUTTON_CIRCLE: Action.MOVE,
            JOY_BUTTON_CROSS: Action.STOP,
        }

        self.selected_robot_index: int = 0
        self.robot_actions: list[action_frame_t] = [
            action_frame_t(robot_index=i, action=Action.STOP)
            for i in range(self.number_of_robots)
        ]

    def motor_status_callback(self, msg: MotorStatus):
        joint_status = joint_frame_t(
            controller_index=np.asarray(msg.controller_index, dtype=np.uint8),
            controlword=np.asarray(msg.controlword, dtype=np.uint16),
            position=np.asarray(msg.position, dtype=np.float64),
            velocity=np.asarray(msg.velocity, dtype=np.float64),
            effort=np.asarray(msg.effort, dtype=np.float64),
        )
        self.robot_manager.updateJointStatus(joint_status)

    def publish_motor_command(self, commands: joint_frame_t):
        msg = MotorStatus()
        msg.number_of_target_interfaces = [
            int(count)
            for count in self.robot_manager.number_of_target_interfaces()
        ]
        msg.target_interface_id = [
            Int8MultiArray(data=[int(interface_id) for interface_id in target_interface_ids])
            for target_interface_ids in self.robot_manager.target_interface_ids()
        ]
        msg.controller_index = [int(index) for index in commands.controller_index]
        msg.controlword = [int(controlword) for controlword in commands.controlword]
        msg.position = [float(position) for position in commands.position]
        msg.velocity = [float(velocity) for velocity in commands.velocity]
        msg.effort = [float(effort) for effort in commands.effort]
        self.motor_command_publisher.publish(msg)

    def joy_callback(self, msg: Joy):
        for btn in range(JOY_BUTTON_MAX):
            self.joy_buttons[btn] = bool(msg.buttons[btn])

    def timer_callback(self):
        # Check if the robot is stopped because of the homing is completed
        state_frames = self.robot_manager.get_state_frames()
        for state_frame in state_frames:
            if state_frame.state == State.STOPPED and self.robot_actions[state_frame.robot_index].action != Action.STOP:
                self.robot_actions[state_frame.robot_index].action = Action.STOP

        # Select the robot by the DPAD
        if self.joy_buttons[JOY_BUTTON_DPAD_LEFT] and not self.joy_buttons_prev[JOY_BUTTON_DPAD_LEFT]:
            self.selected_robot_index = self.selected_robot_index - 1 if self.selected_robot_index > 0 else self.number_of_robots - 1
        elif self.joy_buttons[JOY_BUTTON_DPAD_RIGHT] and not self.joy_buttons_prev[JOY_BUTTON_DPAD_RIGHT]:
            self.selected_robot_index = (self.selected_robot_index + 1) % self.number_of_robots

        # Check if action button is pressed
        for btn, action in self.joy_button_action.items():
            if self.joy_buttons[btn] and not self.joy_buttons_prev[btn]:
                self.robot_actions[self.selected_robot_index].action = action
                break

        # Send the action to the robot
        commands: joint_frame_t = self.robot_manager.set_action_frames(self.robot_actions)
        self.publish_motor_command(commands)

        self.joy_buttons_prev = self.joy_buttons.copy()


def main(args=None):
    rclpy.init(args=args)
    node = RobotManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
