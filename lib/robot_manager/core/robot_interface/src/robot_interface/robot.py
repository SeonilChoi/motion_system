from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from common_robot_interface import ActionFrame, StateFrame
from common_robot_interface import JointStatus, RobotStatus


@dataclass
class RobotConfig:
    """Shared construction parameters for all `Robot` implementations (e.g. from YAML)."""

    robot_id: int = 0
    dt: float = 0.01
    stride_length: float = 0.0
    clearance: float = 0.05
    duration: float = 5.0
    controller_indexes: Any = field(default_factory=lambda: np.zeros(0, dtype=np.int32))
    interface_ids: Any = field(default_factory=lambda: np.zeros(0, dtype=np.int32))
    home_joint_positions: Any = field(default_factory=lambda: np.zeros(0, dtype=np.float64))
    home_pose: Any = field(default_factory=lambda: np.zeros(6, dtype=np.float64))


class Robot(ABC):
    def __init__(self, config: RobotConfig) -> None:
        self._robot_config: RobotConfig = config
        c = config

        self._robot_id: int = c.robot_id
        self._dt: float = c.dt
        self._stride_length: float = c.stride_length
        self._clearance: float = c.clearance
        self._duration: float = c.duration

        self._controller_indexes = np.asarray(c.controller_indexes, dtype=np.int32).reshape(-1).copy()
        self._interface_ids = np.asarray(c.interface_ids, dtype=np.int32).reshape(-1).copy()
        self._number_of_motors: int = int(self._controller_indexes.size)

        self._home_joint_positions = np.asarray(c.home_joint_positions, dtype=np.float64).reshape(-1).copy()
        self._home_pose = np.asarray(c.home_pose, dtype=np.float64).reshape(-1).copy()


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
    def duration(self) -> float:
        return self._duration

    @property
    def controller_indexes(self) -> np.ndarray:
        return self._controller_indexes

    @property
    def interface_ids(self) -> np.ndarray:
        return self._interface_ids

    @property
    def number_of_motors(self) -> int:
        return self._number_of_motors

    @property
    def home_joint_positions(self) -> Any:
        return self._home_joint_positions
    
    @property
    def home_pose(self) -> Any:
        return self._home_pose
    

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
