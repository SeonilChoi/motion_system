from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np


def _quintic_time_scaling(s : float) -> float:
    s = float(np.clip(s, 0.0, 1.0))
    s = (10.0 * s**3) - (15.0 * s**4) + (6.0 * s**5)
    return s

def _parabolic_time_scaling(s : float) -> float:
    s = float(np.clip(s, 0.0, 1.0))
    s = s * (1.0 - s) * 4.0
    return s


class TrajectoryPlanner(ABC):
    def __init__(self) -> None:
        self._curr_position: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._goal_position: np.ndarray = np.array([0.0, 0.0, 0.0])
        
        self._delta: np.ndarray = np.array([0.0, 0.0, 0.0])


    @abstractmethod
    def eval(self, s : float) -> np.ndarray:
        ...

    def set_parameters(self, curr_position: np.ndarray, goal_position: np.ndarray) -> None:
        self._curr_position = curr_position.copy()
        self._goal_position = goal_position.copy()

        self._delta = goal_position - curr_position
