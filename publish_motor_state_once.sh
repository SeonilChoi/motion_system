#!/usr/bin/env bash
set -euo pipefail

# Publish /motor_state every 0.001 s with fixed initial positions.
# Needs: source install/setup.bash (motion_system_msgs on PYTHONPATH).
# Arg: motor count (default 36).
COUNT="${1:-36}"

export COUNT
python3 - <<'PY'
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from motion_system_msgs.msg import MotorFrame, MotorFrameMultiArray

# Rad; same pattern as silver_lain.yaml home_joint_positions (6 legs × 3), one robot.
R40, R140 = 0.6981317007977318, 2.443460952792061
_ONE_ROBOT = [
    0.0, R40, R140,
    0.0, -R40, -R140,
    0.0, R40, R140,
    0.0, -R40, -R140,
    0.0, R40, R140,
    0.0, -R40, -R140,
]
# Default: two robots (0–17, 18–35)
_DEFAULT = _ONE_ROBOT + _ONE_ROBOT


def positions(count: int) -> list[float]:
    if count <= len(_DEFAULT):
        return _DEFAULT[:count]
    return _DEFAULT + [0.0] * (count - len(_DEFAULT))


class Pub(Node):
    def __init__(self, pos: list[float]) -> None:
        super().__init__("motor_state_initial_pub")
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(MotorFrameMultiArray, "/motor_state", qos)
        self._pos = pos
        self.create_timer(0.001, self._tick)

    def _tick(self) -> None:
        msg = MotorFrameMultiArray()
        for i, p in enumerate(self._pos):
            f = MotorFrame()
            f.number_of_target_interfaces = 0
            f.target_interface_id = []
            f.controller_index = i
            f.controlword = 0
            f.statusword = 0
            f.errorcode = 0
            f.position = float(p)
            f.velocity = 0.0
            f.torque = 0.0
            msg.data.append(f)
        self._pub.publish(msg)


def main() -> None:
    n = int(os.environ["COUNT"])
    rclpy.init()
    node = Pub(positions(n))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


main()
PY
