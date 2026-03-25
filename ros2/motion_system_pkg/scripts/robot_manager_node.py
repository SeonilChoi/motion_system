#!/usr/bin/env python3

from __future__ import annotations

import copy
from enum import Enum
from typing import Dict

from common_robot_interface import Action, ActionKind, StateKind
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


class RobotManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('robot_manager_node')
        self.declare_parameter('velocity_scale', 1.0)
        self._vel_scale = float(self.get_parameter('velocity_scale').value)

        self.declare_parameter('config_file', '')
        self._config_file = str(self.get_parameter('config_file').value)

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
            0.01,
            self._timer_callback,
        )

        self._robot = RobotManager(self._config_file)

        self._is_valid_joy_stick = False

        self._joy_axes: Dict[JoyAxes, float] = {axes: 0.0 for axes in JoyAxes}

        self._joy_buttons: Dict[JoyButton, bool] = {btn: False for btn in JoyButton}

        self._prev_joy_buttons: Dict[JoyButton, bool] = {btn: False for btn in JoyButton}

        self._current_action_kind = ActionKind.STOP

        self._joy_button_action_kind: Dict[JoyButton, ActionKind] = {
            JoyButton.A: ActionKind.HOME,
            JoyButton.B: ActionKind.MOVE,
            JoyButton.X: ActionKind.WALK,
            JoyButton.Y: ActionKind.STOP,
        }

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

        if self._robot.current_state_kind == StateKind.STOPPED:
            self._current_action_kind = ActionKind.STOP

        self._current_action_kind = self._action_kind_from_button_edges()

        self._robot.submit_action(Action(kind=self._current_action_kind))

        self._prev_joy_buttons = copy.deepcopy(self._joy_buttons)

        self.get_logger().info(f"Current state kind: {self._robot.current_state_kind}")

    def _action_kind_from_button_edges(self) -> ActionKind:
        for btn, kind in self._joy_button_action_kind.items():
            if self._joy_buttons[btn] and not self._prev_joy_buttons[btn]:
                return kind
        return self._current_action_kind


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
