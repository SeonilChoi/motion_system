from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(slots=True)
class JointStatus:
    motor_id: Optional[np.ndarray] = None
    interface_id: Optional[np.ndarray] = None
    position: Optional[np.ndarray] = None
    velocity: Optional[np.ndarray] = None
    torque: Optional[np.ndarray] = None
