# Copyright 2026 csi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Publish MotorStatus sine-wave position commands on /motor_command."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int8MultiArray
from motion_system_msgs.msg import MotorStatus

N_MOTORS = 5

CW_NEW_SET_POINT_MINAS = 0x003F
CW_NEW_SET_POINT_ZEROERR = 0x103F


def _motor_command_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
    )


class SineMotorCommandPublisher(Node):
    def __init__(self) -> None:
        super().__init__('sine_motor_command_publisher')
        self._n = N_MOTORS
        self._pub = self.create_publisher(
            MotorStatus, 'motor_command', _motor_command_qos()
        )
        self._t = 0.0
        self._timer = self.create_timer(0.01, self._on_timer)
        self.get_logger().info('Publishing sine motor_command')

    def _on_timer(self) -> None:
        msg = MotorStatus()
        n = self._n
        msg.number_of_target_interfaces = [2] * n
        msg.target_interface_id = [Int8MultiArray(data=[0, 1]) for _ in range(n)]
        msg.controller_index = list(range(n))
        z = [0] * n
        msg.controlword = [int(CW_NEW_SET_POINT_MINAS) if i < 2 else int(CW_NEW_SET_POINT_ZEROERR) for i in range(n)]
        msg.statusword = z
        msg.errorcode = z
        msg.velocity = [0.0] * n
        msg.torque = [0.0] * n

        w = 2.0 * math.pi * 0.1
        msg.position = [math.sin(w * self._t) for _ in range(n)]
        #msg.position = [1.0 for _ in range(n)]
        self._pub.publish(msg)
        self._t += 0.01


def main() -> None:
    rclpy.init()
    node = SineMotorCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
