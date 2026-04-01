from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np


class TrajectoryPlanner(ABC):
    def __init__(self) -> None:
        self._init_state: np.ndarray = None
        self._goal_state: np.ndarray = None


    @property
    def initial_state(self) -> np.ndarray | None:
        return None if self._init_state is None else self._init_state.copy()


    def set_initial_state(self, initial_state: np.ndarray) -> None:
        self._init_state = initial_state.copy()


    @abstractmethod
    def update_goal_state(self, goal_state: np.ndarray) -> None:
        self._goal_state = goal_state.copy()

    @abstractmethod
    def eval(self, s : float or List[float]) -> np.ndarray:
        ...


    @staticmethod
    def _quintic_time_scaling(s : float) -> float:
        s = float(np.clip(s, 0.0, 1.0))
        s = (10.0 * s**3) - (15.0 * s**4) + (6.0 * s**5)
        return s

    @staticmethod
    def _parabolic_time_scaling(s : float) -> float:
        s = float(np.clip(s, 0.0, 1.0))
        s = s * (1.0 - s) * 4.0
        return s