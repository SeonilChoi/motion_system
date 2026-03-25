from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


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
