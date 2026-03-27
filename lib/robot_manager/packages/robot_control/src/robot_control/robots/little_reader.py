from __future__ import annotations

from typing import Any, Optional

from common_robot_interface import ActionFrame, StateFrame
from common_robot_interface.joint import JointState
from common_robot_interface.robot import RobotState

from robot_interface.robot import Robot

from robot_control.scheduler.fsm_scheduler import FsmScheduler


class LittleReader(Robot):
    def __init__(
        self,
        robot_id: int = 0,
        dt: float = 0.01,
        stride_length: float = 0.0,
        clearance: float = 0.05,
        controller_indexes: Optional[list[int]] = None,
        home_joint_positions: Any = None,
    ) -> None:
        super().__init__(robot_id, dt, stride_length, clearance, controller_indexes)
        self._home_joint_positions = home_joint_positions
        self._scheduler = FsmScheduler(dt)
        self._curr_joint_state: Optional[JointState] = None

    def get_robot_state(self) -> Optional[RobotState]:
        return None

    def set_joint_state(self, joint_states: JointState) -> None:
        self._curr_joint_state = JointState(
            motor_id=joint_states.motor_id.copy(),
            position=None if joint_states.position is None else joint_states.position.copy(),
            velocity=None if joint_states.velocity is None else joint_states.velocity.copy(),
            torque=None if joint_states.torque is None else joint_states.torque.copy(),
        )

    def get_state(self) -> StateFrame:
        return self._scheduler.current_state

    def set_action(self, frame: ActionFrame) -> None:
        self._scheduler.tick(frame)
