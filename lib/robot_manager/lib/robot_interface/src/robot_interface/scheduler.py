from __future__ import annotations

from abc import ABC, abstractmethod

from common_robot_interface import Action, State, StateKind

class Scheduler(ABC):
    def __init__(self, dt: float) -> None:
        self._dt = dt

        self._t = 0.0

        self._current_state = State(kind=StateKind.STOPPED)

    @property
    def current_state(self) -> State:
        return self._current_state

    def reset(self) -> None:
        self._t = 0.0

        self._current_state = State(kind=StateKind.STOPPED)

    def step(self) -> None:
        self._t += self._dt

    @abstractmethod
    def tick(self, action: Action) -> State:
        ...