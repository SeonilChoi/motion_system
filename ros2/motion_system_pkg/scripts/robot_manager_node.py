#!/usr/bin/env python3

from __future__ import annotations

import copy
from typing import Dict, List
from enum import Enum

import numpy as np

from common_robot_interface import Action, ActionFrame, State, JointState
from robot_manager import RobotManager

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from motion_system_msgs.msg import MotorFrameMultiArray, MotorFrame


class JoyAxes(Enum):
    LEFT_HORIZONTAL       = 0
    LEFT_VERTICAL         = 1
    LT                    = 2
    RIGHT_HORIZONTAL      = 3
    RIGHT_VERTICAL        = 4
    RT                    = 5
    LEFT_RIGHT_DIRECTION  = 6
    UP_DOWN_DIRECTION     = 7

class JoyButton(Enum):
    A          = 0
    B          = 1
    X          = 2
    Y          = 3
    LB         = 4
    RB         = 5
    BACK       = 6
    START      = 7
    HOME       = 8
    LEFT_AXES  = 9
    RIGHT_AXES = 10


def _motor_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
    )

def _joy_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
    )


class RobotManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('robot_manager_node')
        self.declare_parameter('config_file', '')
        self._config_file = str(self.get_parameter('config_file').value)

        # Robot Manager Variables
        self._robot_manager: RobotManager = RobotManager(self._config_file)
        self._selected_robot_id: int = 0
        self._number_of_robots: int = self._robot_manager.number_of_robots
        self._number_of_motors: int = self._robot_manager.number_of_motors

        # JoyStick Variables
        self._is_valid_joy_stick: bool = False
        self._joy_axes: Dict[JoyAxes, float] = {axes: 0.0 for axes in JoyAxes}
        self._joy_buttons: Dict[JoyButton, bool] = {btn: False for btn in JoyButton}
        self._prev_joy_axes: Dict[JoyAxes, float] = {axes: 0.0 for axes in JoyAxes}
        self._prev_joy_buttons: Dict[JoyButton, bool] = {btn: False for btn in JoyButton}
        self._joy_button_action: dict[JoyButton, Action] = {
            JoyButton.A: Action.HOME,
            JoyButton.B: Action.MOVE,
            JoyButton.X: Action.WALK,
            JoyButton.Y: Action.STOP,
        }

        # Joint State Variables
        self._joint_states: JointState = JointState(
            motor_id=np.zeros(self._number_of_motors, dtype=np.int32),
            position=np.zeros(self._number_of_motors, dtype=np.float64),
            velocity=np.zeros(self._number_of_motors, dtype=np.float64),
            torque=np.zeros(self._number_of_motors, dtype=np.float64),
        )

        # Scheduler Variables
        self._curr_action: List[ActionFrame] = [
            ActionFrame(action=Action.STOP) for _ in range(self._number_of_robots)
        ]

        self._joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            _joy_qos(),
        )      

        self._motor_state_sub = self.create_subscription(
            MotorFrameMultiArray,
            'motor_state',
            self.motor_state_callback,
            _motor_qos(),
        )

        self._motor_cmd_pub = self.create_publisher(
            MotorFrameMultiArray,
            'motor_command',
            _motor_qos(),
        )

        self._timer = self.create_timer(
            self._robot_manager.dt,
            self.timer_callback,
        )


    def _check_joy_stick_mode(self, mode: float) -> None:
        if mode == 1:
            self._is_valid_joy_stick = True
            self.get_logger().info("Joy stick mode is valid")

    def _select_robot(self, up_down: float) -> None:
        self._selected_robot_id = (self._selected_robot_id + int(up_down)) % self._number_of_robots
        if self._selected_robot_id < 0:
            self._selected_robot_id = self._number_of_robots - 1
        self.get_logger().info(f"Selected robot ID: {self._selected_robot_id}")

    def _publish_joint_commands(self, joint_commands: JointState) -> None:
        msg = MotorFrameMultiArray()
        msg.data = [MotorFrame(
            controller_index=int(joint_commands.motor_id[i]),
            position=float(joint_commands.position[i]),
            velocity=float(joint_commands.velocity[i]),
            torque=float(joint_commands.torque[i]),
        ) for i in range(self._number_of_motors)]
        self._motor_cmd_pub.publish(msg)


    def motor_state_callback(self, msg: MotorFrameMultiArray) -> None:
        for i in range(self._number_of_motors):
            self._joint_states.motor_id[i] = msg.data[i].controller_index
            self._joint_states.position[i] = msg.data[i].position
            self._joint_states.velocity[i] = msg.data[i].velocity
            self._joint_states.torque[i] = msg.data[i].torque

        self._robot_manager.set_joint_states(self._joint_states)

    def joy_callback(self, msg: Joy) -> None:
        if self._is_valid_joy_stick is False:
            self._check_joy_stick_mode(msg.axes[JoyAxes.LEFT_RIGHT_DIRECTION.value])
            return
        
        for axes in JoyAxes:
            self._joy_axes[axes] = msg.axes[axes.value]

        for btn in JoyButton:
            self._joy_buttons[btn] = msg.buttons[btn.value]

    def timer_callback(self) -> None:
        if self._is_valid_joy_stick is False:
            return
        
        if self._joy_axes[JoyAxes.UP_DOWN_DIRECTION] != 0.0 and self._prev_joy_axes[JoyAxes.UP_DOWN_DIRECTION] == 0.0:
            self._select_robot(self._joy_axes[JoyAxes.UP_DOWN_DIRECTION])

        if self._robot_manager.get_state(self._selected_robot_id).state == State.STOPPED:
            self._curr_action[self._selected_robot_id] = ActionFrame(action=Action.STOP)

        for btn, action in self._joy_button_action.items():
            if self._joy_buttons[btn] and not self._prev_joy_buttons[btn]:
                self._curr_action[self._selected_robot_id] = ActionFrame(action=action)
                break

        if self._curr_action[self._selected_robot_id].action == Action.WALK:
            vx = self._joy_axes[JoyAxes.LEFT_VERTICAL]
            vy = self._joy_axes[JoyAxes.LEFT_HORIZONTAL]
            wz = self._joy_axes[JoyAxes.RIGHT_HORIZONTAL]
            
            self._curr_action[self._selected_robot_id] = ActionFrame(
                action=Action.WALK,
                goal=np.array([vx, vy, wz])
            )

        joint_commands = self._robot_manager.set_action(self._curr_action)
        self._publish_joint_commands(joint_commands)
        
        self._prev_joy_axes = copy.deepcopy(self._joy_axes)
        self._prev_joy_buttons = copy.deepcopy(self._joy_buttons)


def main() -> None:
    rclpy.init()
    node = RobotManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
