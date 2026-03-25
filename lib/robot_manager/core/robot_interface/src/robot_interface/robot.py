from __future__ import annotations

from abc import ABC, abstractmethod

from common_robot_interface import Action, State


class Robot(ABC):
    def __init__(self, dt: float = 0.01) -> None:
        self._dt = dt

    @abstractmethod
    def get_state(self) -> State:
        ...

    @abstractmethod
    def set_action(self, action: Action) -> None:
        ...
