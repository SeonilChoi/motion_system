from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml

from common_robot_interface import Action, StateKind
from robot_interface.robot import Robot

from robot_control.robots.little_reader import LittleReader
from robot_control.robots.silver_lain import SilverLain

_ROBOT_BY_KEY: dict[str, type[Robot]] = {
    'little_reader': LittleReader,
    'silver_lain': SilverLain,
}


class RobotManager:
    def __init__(self, config_file: str) -> None:
        self._config: dict[str, Any] = {}
        self._loadConfigurations(config_file)

        robot_cls = self._robot_class_from_config()
        dt = float(self._config.get('dt', 0.01))
        self._robot: Robot = robot_cls(dt=dt)

    def get_current_state_kind(self) -> StateKind:
        return self._robot.get_state().kind

    def submit_action(self, action: Action | None) -> None:
        if action is None:
            return
        self._robot.set_action(action)

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
