from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from common_robot_interface import ActionFrame, StateFrame
from common_robot_interface.joint import JointState
from common_robot_interface.robot import RobotState


class Robot(ABC):
    def __init__(
        self,
        robot_id: int,
        dt: float = 0.01,
        stride_length: float = 0.0,
        controller_indexes: Optional[list[int]] = None,
    ) -> None:
        self._robot_id: int = robot_id
        self._dt: float = dt
        self._stride_length: float = stride_length
        self._controller_indexes: Optional[list[int]] = (
            list(controller_indexes) if controller_indexes is not None else None
        )


    @property
    def robot_id(self) -> int:
        return self._robot_id

    @property
    def stride_length(self) -> float:
        return self._stride_length

    @property
    def controller_indexes(self) -> Optional[list[int]]:
        return self._controller_indexes


    @abstractmethod
    def get_robot_state(self) -> Optional[RobotState]:
        ...

    @abstractmethod
    def set_joint_state(self, joint_states: JointState) -> None:
        ...

    @abstractmethod
    def get_state(self) -> StateFrame:
        ...

    @abstractmethod
    def set_action(self, frame: ActionFrame) -> None:
        ...
