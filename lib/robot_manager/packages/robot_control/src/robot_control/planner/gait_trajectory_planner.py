from __future__ import annotations

import numpy as np

from robot_interface.planner import TrajectoryPlanner, _quintic_time_scaling, _parabolic_time_scaling


class GaitTrajectoryPlanner(TrajectoryPlanner):
    def __init__(self) -> None:
        super().__init__()
        self._duration : float = None
        self._clearance : float = None
        

    def set_parameters(
        self,
        curr_position: np.ndarray,
        goal_position: np.ndarray,
        duration: float,
        clearance: float,
    ) -> None:
        super().set_parameters(curr_position, goal_position)
        self._duration = duration
        self._clearance = clearance

    def eval(self, s : float) -> np.ndarray:
        u = _quintic_time_scaling(s)
        v = _parabolic_time_scaling(s)
        
        p = self._curr_position + u * self._delta
        p[2] += (v * self._clearance)

        return p
