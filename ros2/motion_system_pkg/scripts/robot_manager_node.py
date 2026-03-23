#!/usr/bin/env python3
"""Bridges gamepad input to motor commands using the latest motor state as a template."""

from __future__ import annotations

import copy
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from motion_system_msgs.msg import MotorFrameMultiArray
from sensor_msgs.msg import Joy


def _motor_qos() -> QoSProfile:
    return QoSProfile(
        depth=10,
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
        self.declare_parameter('velocity_scale', 1.0)
        self._vel_scale = float(self.get_parameter('velocity_scale').value)

        self._motor_state: Optional[MotorFrameMultiArray] = None

        self._motor_state_sub = self.create_subscription(
            MotorFrameMultiArray,
            'motor_state',
            self._motor_state_callback,
            _motor_qos(),
        )
        self._joy_sub = self.create_subscription(
            Joy,
            'joy',
            self._joy_callback,
            _joy_qos(),
        )
        self._motor_cmd_pub = self.create_publisher(
            MotorFrameMultiArray,
            'motor_command',
            _motor_qos(),
        )

    def _motor_state_callback(self, msg: MotorFrameMultiArray) -> None:
        self._motor_state = msg

    def _joy_callback(self, msg: Joy) -> None:
        if self._motor_state is None or not self._motor_state.data:
            return

        axes = msg.axes
        if len(axes) < 2:
            return

        cmd = MotorFrameMultiArray()
        cmd.data = [copy.deepcopy(f) for f in self._motor_state.data]

        scale = self._vel_scale
        
        cmd.data[0].velocity = float(-axes[1]) * scale
        if len(cmd.data) > 1 and len(axes) > 0:
            cmd.data[1].velocity = float(axes[0]) * scale

        self._motor_cmd_pub.publish(cmd)


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
