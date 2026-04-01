from __future__ import annotations

import numpy as np

from robot_interface.planner import TrajectoryPlanner


class GaitTrajectoryPlanner(TrajectoryPlanner):
    def __init__(self, clearance: float, duration: float) -> None:
        super().__init__()
        self._clearance : float = clearance
        self._duration : float = duration
    

    def update_goal_state(self, goal_state: np.ndarray, clearance: float, duration: float) -> None:
        super().update_goal_state(goal_state)
        self._clearance = clearance
        self._duration = duration

    def eval(self, s : float) -> np.ndarray:
        u = self._quintic_time_scaling(s)
        v = self._parabolic_time_scaling(s)
        
        p = self._init_state + u * (self._goal_state - self._init_state)
        p[2] += (v * self._clearance)

        return p
