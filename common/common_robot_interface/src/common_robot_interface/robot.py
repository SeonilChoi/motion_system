from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(slots=True)
class RobotStatus:
    robot_id: int
    pose: np.ndarray
    point: np.ndarray
    twist: np.ndarray
    wrench: np.ndarray
