from __future__ import annotations

from typing import Optional

import numpy as np

from common_robot_interface import ActionFrame, JointStatus, RobotStatus, StateFrame

from robot_interface.robot import Robot, RobotConfig

from scheduler.fsm_scheduler import FsmScheduler


class LittleReader(Robot):
    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config)
        self._scheduler = FsmScheduler(config.dt)
        self._curr_joint_status: Optional[JointStatus] = None

    def get_state_frame(self) -> StateFrame:
        return self._scheduler.current_state

    def set_action_frame(self, frame: ActionFrame) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        self._scheduler.tick(frame)
        n = self._number_of_motors
        z = np.zeros(n, dtype=np.float64)
        if self._curr_joint_status is None:
            return z, z, z
        p, v, t = (
            self._curr_joint_status.position,
            self._curr_joint_status.velocity,
            self._curr_joint_status.torque,
        )
        return (
            z if p is None else p.copy(),
            z if v is None else v.copy(),
            z if t is None else t.copy(),
        )

    def get_robot_status(self) -> RobotStatus:
        return RobotStatus(
            robot_id=self._robot_id,
            pose=np.zeros(6, dtype=np.float64),
            point=np.zeros((6, 3), dtype=np.float64),
            twist=np.zeros(6, dtype=np.float64),
            wrench=np.zeros(6, dtype=np.float64),
        )

    def update_joint_status(self, joint_status: JointStatus) -> None:
        self._curr_joint_status = JointStatus(
            motor_id=None if joint_status.motor_id is None else joint_status.motor_id.copy(),
            interface_id=(
                None
                if joint_status.interface_id is None
                else joint_status.interface_id.copy()
            ),
            position=(
                None
                if joint_status.position is None
                else joint_status.position.copy()
            ),
            velocity=(
                None
                if joint_status.velocity is None
                else joint_status.velocity.copy()
            ),
            torque=None if joint_status.torque is None else joint_status.torque.copy(),
        )
