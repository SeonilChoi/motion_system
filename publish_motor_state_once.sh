#!/usr/bin/env bash
set -euo pipefail

# Publish /motor_state once with fixed home-like positions (SilverLain pattern).
# Needs: source install/setup.bash (motion_system_msgs on PYTHONPATH).
# Arg: motor count (default 36).
COUNT="${1:-36}"

export COUNT
python3 - <<'PY'
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Int8MultiArray

from motion_system_msgs.msg import MotorStatus

# Rad; same pattern as silver_lain home (6 legs × 3), one robot.
R40, R140 = 0.6981317007977318, 2.443460952792061
_ONE_ROBOT = [
    0.0, R40, R140,
    0.0, -R40, -R140,
    0.0, R40, R140,
    0.0, -R40, -R140,
    0.0, R40, R140,
    0.0, -R40, -R140,
]
_DEFAULT = _ONE_ROBOT + _ONE_ROBOT


def positions(count: int) -> list[float]:
    if count <= len(_DEFAULT):
        return _DEFAULT[:count]
    return _DEFAULT + [0.0] * (count - len(_DEFAULT))


def main() -> None:
    n = int(os.environ["COUNT"])
    pos = positions(n)

    rclpy.init()
    node = Node("motor_state_once_pub")
    qos = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
    )
    pub = node.create_publisher(MotorStatus, "/motor_state", qos)

    msg = MotorStatus()
    msg.number_of_target_interfaces = [0] * n
    msg.target_interface_id = [Int8MultiArray() for _ in range(n)]
    msg.controller_index = list(range(n))
    msg.controlword = [0] * n
    msg.statusword = [0] * n
    msg.errorcode = [0] * n
    msg.position = [float(p) for p in pos]
    msg.velocity = [0.0] * n
    msg.torque = [0.0] * n

    # Brief wait so subscribers can match the publisher (one-shot otherwise often misses).
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.05)

    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()


main()
PY
