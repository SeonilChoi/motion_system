from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


class StateKind(Enum):
    """Which robot state is requested or running."""

    HOMMING = auto()
    OPERATING = auto()
    WALKING = auto()
    STOPPED = auto()


@dataclass(frozen=True, slots=True)
class State:
    """Plain data record for the current state (struct-like)."""

    kind: StateKind
    progress: float = 0.0
