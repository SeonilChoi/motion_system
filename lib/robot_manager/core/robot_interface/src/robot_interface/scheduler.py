from __future__ import annotations

from abc import ABC, abstractmethod

from common_robot_interface import ActionFrame, State, StateFrame


class Scheduler(ABC):
    def __init__(self, dt: float) -> None:
        self._dt: float = dt
        self._t: float = 0.0

        self._current_state = StateFrame(state=State.STOPPED, progress=0.0)


    @property
    def current_state(self) -> StateFrame:
        return self._current_state

    def reset(self) -> None:
        self._t = 0.0

        self._current_state = StateFrame(state=State.STOPPED, progress=0.0)


    @abstractmethod
    def tick(self, frame: ActionFrame) -> bool:
        ...
