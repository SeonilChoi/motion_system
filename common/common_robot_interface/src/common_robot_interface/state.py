from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


class State(Enum):
    HOMMING = auto()
    OPERATING = auto()
    WALKING = auto()
    STOPPED = auto()

@dataclass(frozen=True, slots=True)
class StateFrame:
    state: State
    progress: float = 0.0
