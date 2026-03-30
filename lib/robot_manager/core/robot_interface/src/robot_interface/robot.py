from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from common_robot_interface import ActionFrame, StateFrame
from common_robot_interface import JointStatus, RobotStatus


class Robot(ABC):
    def __init__(
        self,
        robot_id: int,
        dt: float = 0.01,
        stride_length: float = 0.0,
        clearance: float = 0.05,
        controller_indexes: Optional[list[int]] = None,
        interface_ids: Optional[list[int]] = None,
        home_joint_positions: np.ndarray = None,
    ) -> None:
        self._robot_id: int = robot_id
        
        self._dt: float = dt
        self._stride_length: float = stride_length
        self._clearance: float = clearance
        
        self._controller_indexes: Optional[list[int]] = (
            list(controller_indexes) if controller_indexes is not None else None
        )
        self._interface_ids: Optional[list[int]] = (
            list(interface_ids) if interface_ids is not None else None
        )
        self._number_of_motors: int = (
            len(controller_indexes) if controller_indexes is not None else 0
        )

        self._home_joint_positions: np.ndarray = home_joint_positions

    @property
    def robot_id(self) -> int:
        return self._robot_id

    @property
    def stride_length(self) -> float:
        return self._stride_length

    @property
    def clearance(self) -> float:
        return self._clearance

    @property
    def controller_indexes(self) -> Optional[list[int]]:
        return self._controller_indexes

    @property
    def interface_ids(self) -> Optional[list[int]]:
        return self._interface_ids


    @abstractmethod
    def get_state_frame(self) -> StateFrame:
        ...

    @abstractmethod
    def set_action_frame(self, frame: ActionFrame) -> np.ndarray:
        ...

    @abstractmethod
    def get_robot_status(self) -> RobotStatus:
        ...

    @abstractmethod
    def update_joint_status(self, joint_status: JointStatus) -> None:
        ...
