from __future__ import annotations

import numpy as np

from robot_interface.planner import TrajectoryPlanner


class GaitTrajectoryPlanner(TrajectoryPlanner):
    def __init__(self, clearance: float) -> None:
        super().__init__()
        self._clearance : float = clearance
        self._duration : float = None
        

    def set_initial_state(self, initial_state: np.ndarray) -> None:
        super().set_initial_state(initial_state)

    def update_goal_state(self, goal_state: np.ndarray, duration: float) -> None:
        super().update_goal_state(goal_state)
        self._duration = duration

    def eval(self, s : float or List[float]) -> np.ndarray:
        u = self._quintic_time_scaling(s)
        v = self._parabolic_time_scaling(s)
        
        if isinstance(s, float):
            p = self._init_state + u * (self._goal_state - self._init_state)
            p[2] += (v * self._clearance)
        else:
            p = self._init_state + u.reshape(-1, 1) * (self._goal_state - self._init_state)
            p[:, 2] += (v * self._clearance)

        return p
