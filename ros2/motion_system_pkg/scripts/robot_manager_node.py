#!/usr/bin/env python3

from __future__ import annotations

import copy
import numpy as np
from typing import Dict, List
from enum import Enum

from robot_manager import RobotManager
from scheduler.gait_scheduler import Event
from common_robot_interface import Action, ActionFrame, State, StateFrame
from common_robot_interface import JointStatus, RobotStatus

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int8MultiArray, Float64MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, Vector3
from motion_system_msgs.msg import MotorStatus


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


def _joy_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
    )

def _motor_status_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
    )

def _pose_qos() -> QoSProfile:
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
        self._number_of_motors: int = self._robot_manager.number_of_motors
        self._number_of_robots: int = self._robot_manager.number_of_robots
        
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
        self._joint_status: JointStatus = JointStatus(
            motor_id=np.zeros(self._number_of_motors, dtype=np.int32),
            position=np.zeros(self._number_of_motors, dtype=np.float64),
            velocity=np.zeros(self._number_of_motors, dtype=np.float64),
            torque=np.zeros(self._number_of_motors, dtype=np.float64)
        )

        # Robot Status Variables
        self._robot_status: RobotStatus = RobotStatus(
            robot_id = self._selected_robot_id,
            pose=np.zeros(6, dtype=np.float64),
            point=np.zeros((6, 3), dtype=np.float64),
            twist=np.zeros(6, dtype=np.float64),
            wrench=np.zeros(6, dtype=np.float64)
        )

        # Scheduler Variables
        self._curr_action: List[ActionFrame] = [
            ActionFrame(action=Action.STOP) for _ in range(self._number_of_robots)
        ]

        # ROS2 Variables
        self._joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            _joy_qos(),
        )      

        self._motor_state_sub = self.create_subscription(
            MotorStatus,
            'motor_state',
            self.motor_state_callback,
            _motor_status_qos(),
        )

        self._motor_command_pub = self.create_publisher(
            MotorStatus,
            'motor_command',
            _motor_status_qos(),
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

    def _round_direction(self, direction: np.ndarray) -> np.ndarray:
        angle = np.arctan2(direction[1], direction[0])
        if np.abs(angle) <= np.pi / 8:
            return np.array([1, 0])
        elif angle > np.pi / 8 and angle <= 3 * np.pi / 8:
            return np.array([np.sqrt(2) / 2, np.sqrt(2) / 2])
        elif angle > 3 * np.pi / 8 and angle <= 5 * np.pi / 8:
            return np.array([0, 1])
        elif angle > 5 * np.pi / 8 and angle <= 7 * np.pi / 8:
            return np.array([-np.sqrt(2) / 2, np.sqrt(2) / 2])
        elif angle >= 7 * np.pi / 8 or angle <= -7 * np.pi / 8:
            return np.array([-1, 0])
        elif angle > -7 * np.pi / 8 and angle <= -5 * np.pi / 8:
            return np.array([-np.sqrt(2) / 2, -np.sqrt(2) / 2])
        elif angle > -5 * np.pi / 8 and angle <= -3 * np.pi / 8:
            return np.array([0, -1])
        elif angle > -3 * np.pi / 8 and angle <= -np.pi / 8:
            return np.array([np.sqrt(2) / 2, -np.sqrt(2) / 2])
        else:
            return np.array([1, 0])

    def _normalize_joy_command(self, joy_axes: Dict[JoyAxes, float]) -> np.ndarray:
        vx = joy_axes[JoyAxes.LEFT_VERTICAL]
        vy = joy_axes[JoyAxes.LEFT_HORIZONTAL]
        wz = joy_axes[JoyAxes.RIGHT_HORIZONTAL]
        
        direction = np.zeros(2)
        
        stride_length = self._robot_manager.stride_length(self._selected_robot_id)
        duration = self._robot_manager.duration(self._selected_robot_id)
        
        linear_velocity = np.array([vx, vy])
        linear_speed = np.linalg.norm(linear_velocity)
        
        if linear_speed > 0.0:
            direction = linear_velocity / linear_speed
            direction = self._round_direction(direction)
            
        return ActionFrame(
            action=Action.WALK,
            goal=np.array([direction[0], direction[1], wz]),
            duration=duration,
        )


    def _publish_joint_command(self, joint_command: JointStatus) -> None:
        msg = MotorStatus()
        msg.number_of_target_interfaces = np.ones(self._number_of_motors, dtype=np.uint8)
        msg.target_interface_id = [
            Int8MultiArray(data=[int(joint_command.interface_id[i])])
            for i in range(self._number_of_motors)
        ]
        msg.controller_index = joint_command.motor_id
        msg.position = joint_command.position
        msg.velocity = joint_command.velocity
        msg.torque = joint_command.torque
        self._motor_command_pub.publish(msg)


    def motor_state_callback(self, msg: MotorStatus) -> None:
        for i in range(self._number_of_motors):
            self._joint_status.motor_id[i] = msg.controller_index[i]
            self._joint_status.position[i] = msg.position[i]
            self._joint_status.velocity[i] = msg.velocity[i]
            self._joint_status.torque[i] = msg.torque[i]

        self._robot_manager.update_joint_status(self._joint_status)

    def joy_callback(self, msg: Joy) -> None:
        if self._is_valid_joy_stick is False:
            self._check_joy_stick_mode(msg.axes[JoyAxes.LEFT_RIGHT_DIRECTION.value])
            return
        
        for axes in JoyAxes:
            self._joy_axes[axes] = msg.axes[axes.value]

        for btn in JoyButton:
            self._joy_buttons[btn] = msg.buttons[btn.value]

    def timer_callback(self) -> None:
        # Read robot state and publish it
        self._robot_status = self._robot_manager.get_robot_status(self._selected_robot_id)
        
        prev_pose = self._robot_status.pose.copy()
        prev_joint_position = self._robot_manager._robots[self._selected_robot_id]._curr_joint_status.position.copy()

        # If joy stick is not valid, return
        if self._is_valid_joy_stick is False:
            return

        # Select robot if up/down direction is changed
        if self._joy_axes[JoyAxes.UP_DOWN_DIRECTION] != 0.0 and self._prev_joy_axes[JoyAxes.UP_DOWN_DIRECTION] == 0.0:
            self._select_robot(self._joy_axes[JoyAxes.UP_DOWN_DIRECTION])
        
        # If robot is stopped, set action to STOP
        for robot_id in range(self._number_of_robots):
            if self._robot_manager.get_state_frame(robot_id).state == State.STOPPED:
                self._curr_action[robot_id] = ActionFrame(action=Action.STOP)

        # Set action based on button press
        for btn, action in self._joy_button_action.items():
            if self._joy_buttons[btn] and not self._prev_joy_buttons[btn]:
                self._curr_action[self._selected_robot_id] = ActionFrame(action=action)
                break

        # Normalize joy command if action is WALK
        if self._curr_action[self._selected_robot_id].action == Action.WALK:
            if self._robot_manager._robots[self._selected_robot_id]._scheduler.current_state.progress == 1.0:
                if self._count_count >= 1:
                    self._count = (np.random.randint(0, 5) - 2 + self._count + 8) % 8
                    self._count_count = 0
                else:
                    self._count_count += 1


            #self._curr_action[self._selected_robot_id] = self._normalize_joy_command(self._joy_axes)
            self._curr_action[self._selected_robot_id] = self._normalize_joy_command(self._for_data_collection_joy_axes())

        # Set action and get joint commands and publish it
        joint_command = self._robot_manager.set_action_frame(self._curr_action)
        self._publish_joint_command(joint_command)

        # Update previous joy axes and buttons
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
