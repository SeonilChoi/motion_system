from __future__ import annotations

from common_robot_interface import Action, ActionKind, StateKind
from robot_interface.robot import Robot

class RobotManager:
    def __init__(self, config_file: str) -> None:
        self._loadConfigurations(config_file)

        self._active_action: Action | None = None

        self._current_state_kind = StateKind.STOPPED

        self._robot = Robot()

    @property
    def current_state_kind(self) -> StateKind:
        return self._current_state_kind

    def submit_action(self, action: Action | None) -> None:
        self._active_action = action

        if self._active_action.kind == ActionKind.HOME:
            pass
        elif self._active_action.kind == ActionKind.MOVE:
            self._current_state_kind = StateKind.OPERATING
        elif self._active_action.kind == ActionKind.WALK:
            self._current_state_kind = StateKind.WALKING
        elif self._active_action.kind == ActionKind.STOP:
            self._current_state_kind = StateKind.STOPPED

    def _loadConfigurations(self, config_file: str) -> None:
        pass
