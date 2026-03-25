#!/usr/bin/env python3

from __future__ import annotations

import copy
from typing import Dict

from common_robot_interface import JoyAxes, JoyButton
from robot_manager import RobotManager

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from motion_system_msgs.msg import MotorFrameMultiArray

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

        self.declare_parameter('stride_length', 0.5)
        self._stride_length = float(self.get_parameter('stride_length').value)

        self._robot_manager = RobotManager(
            self._config_file,
            stride_length=self._stride_length,
        )

        self._joy_sub = self.create_subscription(
            Joy,
            'joy',
            self._joy_callback,
            _joy_qos(),
        )      

        self._motor_state_sub = self.create_subscription(
            MotorFrameMultiArray,
            'motor_state',
            self._motor_state_callback,
            _motor_qos(),
        )

        self._motor_cmd_pub = self.create_publisher(
            MotorFrameMultiArray,
            'motor_command',
            _motor_qos(),
        )

        self._timer = self.create_timer(
            self._robot_manager.dt,
            self._timer_callback,
        )

        self._is_valid_joy_stick = False

        self._joy_axes: Dict[JoyAxes, float] = {axes: 0.0 for axes in JoyAxes}

        self._joy_buttons: Dict[JoyButton, bool] = {btn: False for btn in JoyButton}

        self._prev_joy_buttons: Dict[JoyButton, bool] = {btn: False for btn in JoyButton}

    def _check_joy_stick_mode(self, mode: float) -> None:
        if mode == 1:
            self._is_valid_joy_stick = True
            self.get_logger().info("Joy stick mode is valid")

    def _motor_state_callback(self, msg: MotorFrameMultiArray) -> None:
        pass

    def _joy_callback(self, msg: Joy) -> None:
        if self._is_valid_joy_stick is False:
            self._check_joy_stick_mode(msg.axes[JoyAxes.LEFT_RIGHT_DIRECTION.value])
            return
        
        for axes in JoyAxes:
            self._joy_axes[axes] = msg.axes[axes.value]

        for btn in JoyButton:
            self._joy_buttons[btn] = msg.buttons[btn.value]

    def _timer_callback(self) -> None:
        if self._is_valid_joy_stick is False:
            return

        state = self._robot_manager.get_state()
        self.get_logger().info(f"State: {state.kind}, {state.progress}")

        self._robot_manager.joy_stick_command(self._joy_axes, self._joy_buttons, self._prev_joy_buttons)

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
