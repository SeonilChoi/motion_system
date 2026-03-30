from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

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
    goal: Optional[np.ndarray] = None
