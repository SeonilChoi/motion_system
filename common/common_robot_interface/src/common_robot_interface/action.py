from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np

class ActionKind(Enum):
    """Which robot action is requested or running."""

    HOME = auto()
    MOVE = auto()
    WALK = auto()
    STOP = auto()


@dataclass(frozen=True, slots=True)
class Action:
    """Plain data record for the current action (struct-like)."""

    kind: ActionKind
    duration: float = 0.0
    goal: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
