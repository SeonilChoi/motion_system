from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

@dataclass(frozen=True, slots=True)
class Vector3:
    x: float
    y: float
    z: float

@dataclass(frozen=True, slots=True)
class Pose:
    position: Vector3
    orientation: Vector3
    points: List[Vector3]

@dataclass(frozen=True, slots=True)
class Twist:
    linear: Vector3
    angular: Vector3

@dataclass(frozen=True, slots=True)
class Wrench:
    force: Vector3
    torque: Vector3


@dataclass(frozen=True, slots=True)
class RobotState:
    robot_id: int
    pose: Pose
    twist: Twist    
    wrench: wrench
