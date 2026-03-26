from __future__ import annotations

from dataclasses import dataclass

import numpy as np

@dataclass(frozen=True, slots=True)
class JointState:
    motor_id: np.ndarray
    position: np.ndarray
    velocity: np.ndarray     
    torque: np.ndarray
