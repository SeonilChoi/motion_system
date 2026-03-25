from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from common_robot_interface import Action, ActionKind, StateKind, JoyAxes, JoyButton
from robot_interface.robot import Robot

from robot_control.robots.little_reader import LittleReader
from robot_control.robots.silver_lain import SilverLain

_ROBOT_BY_KEY: dict[str, type[Robot]] = {
    'little_reader': LittleReader,
    'silver_lain': SilverLain,
}


class RobotManager:
    def __init__(
        self,
        config_file: str,
        *,
        stride_length: float | None = None,
    ) -> None:
        self._config: dict[str, Any] = {}
        self._loadConfigurations(config_file)
        if stride_length is not None:
            self._config['stride_length'] = float(stride_length)

        robot_cls = self._robot_class_from_config()
        dt = self.dt
        self._robot: Robot = robot_cls(dt=dt)
        self._current_action_kind = ActionKind.STOP

        self._joy_button_action_kind: dict[JoyButton, ActionKind] = {
            JoyButton.A: ActionKind.HOME,
            JoyButton.B: ActionKind.MOVE,
            JoyButton.X: ActionKind.WALK,
            JoyButton.Y: ActionKind.STOP,
        }

    @property
    def dt(self) -> float:
        return float(self._config.get('dt', 0.01))

    @property
    def stride_length(self) -> float:
        v = float(self._config.get('stride_length', 0.5))
        return v if v > 0.0 else 0.5

    def get_state(self) -> State:
        return self._robot.get_state()

    def joy_stick_command(
        self,
        axes: dict[JoyAxes, float],
        button: dict[JoyButton, bool],
        prev_button: dict[JoyButton, bool],
    ) -> None:
        if self._robot.get_state().kind == StateKind.STOPPED:
            self._current_action_kind = ActionKind.STOP

        for btn, kind in self._joy_button_action_kind.items():
            if button[btn] and not prev_button[btn]:
                self._current_action_kind = kind
                break
        
        if self._current_action_kind != ActionKind.WALK:
            self._robot.set_action(Action(kind=self._current_action_kind))
        else:
            vx = axes[JoyAxes.LEFT_VERTICAL]
            vy = axes[JoyAxes.RIGHT_HORIZONTAL]

            norm = np.sqrt(vx**2 + vy**2)
            if norm == 0.0:
                duration = 0.0
            else:
                speed = (norm / np.sqrt(2)) * 0.05 # 0.05 is the maximum speed
                duration = self.stride_length / speed

            self._robot.set_action(Action(kind=ActionKind.WALK, duration=duration, goal=np.array([vx, vy, 0.0])))

    def _loadConfigurations(self, config_file: str) -> None:
        self._config = {}
        path = Path((config_file or '').strip()).expanduser()
        if not path.is_file():
            return
        with path.open('r', encoding='utf-8') as f:
            loaded = yaml.safe_load(f)
        if loaded is None:
            return
        if not isinstance(loaded, Mapping):
            return
        self._config = dict(loaded)

    def _robot_class_from_config(self) -> type[Robot]:
        key = str(self._config.get('robot', 'little_reader')).lower().replace(' ', '_')
        return _ROBOT_BY_KEY.get(key, LittleReader)
