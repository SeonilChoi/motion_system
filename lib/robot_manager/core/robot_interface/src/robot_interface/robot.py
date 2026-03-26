from __future__ import annotations

from abc import ABC, abstractmethod

from common_robot_interface import ActionFrame, StateFrame


class Robot(ABC):
    def __init__(self, dt: float = 0.01, stride_length: float = 0.0) -> None:
        self._dt = dt
        self._stride_length = stride_length


    @property
    def stride_length(self) -> float:
        return self._stride_length


    @abstractmethod
    def get_state(self) -> StateFrame:
        ...

    @abstractmethod
    def set_action(self, frame: ActionFrame) -> None:
        ...
