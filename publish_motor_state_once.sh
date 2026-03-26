#!/usr/bin/env bash
set -euo pipefail

# Publish /motor_state at ~1 kHz from one process.
# Position ramps from -pi/2 to +pi/2 and repeats.
# Args: [COUNT] [STEP_RAD]
COUNT="${1:-36}"
STEP="${2:-0.01}"

COUNT="$COUNT" STEP="$STEP" python3 - <<'PY'
import os
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from motion_system_msgs.msg import MotorFrame, MotorFrameMultiArray


class MotorStatePublisher(Node):
    def __init__(self, count: int, step: float) -> None:
        super().__init__("motor_state_ramp_pub")
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(MotorFrameMultiArray, "/motor_state", qos)
        self._count = count
        self._step = step
        self._min_pos = -math.pi / 2.0
        self._max_pos = math.pi / 2.0
        self._pos = self._min_pos
        self._timer = self.create_timer(0.001, self._on_timer)

    def _on_timer(self) -> None:
        msg = MotorFrameMultiArray()
        for i in range(self._count):
            frame = MotorFrame()
            frame.number_of_target_interfaces = 0
            frame.target_interface_id = []
            frame.controller_index = i
            frame.controlword = 0
            frame.statusword = 0
            frame.errorcode = 0
            frame.position = self._pos
            frame.velocity = 0.0
            frame.torque = 0.0
            msg.data.append(frame)
        self._pub.publish(msg)

        self._pos += self._step
        if self._pos > self._max_pos:
            self._pos = self._min_pos


def main() -> None:
    count = int(os.environ.get("COUNT", "36"))
    step = float(os.environ.get("STEP", "0.01"))

    rclpy.init()
    node = MotorStatePublisher(count, step)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
