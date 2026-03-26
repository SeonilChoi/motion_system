from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np


class Action(Enum):
    HOME = auto()
    MOVE = auto()
    WALK = auto()
    STOP = auto()


@dataclass(frozen=True, slots=True)
class ActionFrame:
    action: Action
    duration: float = 0.0
    goal: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
