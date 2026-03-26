from __future__ import annotations

from pathlib import Path
from typing import Any, List, Mapping

import yaml

from common_robot_interface import ActionFrame, StateFrame
from robot_interface.robot import Robot

from robot_control.robots.little_reader import LittleReader
from robot_control.robots.silver_lain import SilverLain


_ROBOT_BY_KEY: dict[str, type[Robot]] = {
    'little_reader': LittleReader,
    'silver_lain': SilverLain,
}


def _robot_class_for_key(key: str) -> type[Robot]:
    return _ROBOT_BY_KEY.get(key.lower().replace(' ', '_'), LittleReader)


class RobotManager:
    def __init__(self, config_file: str) -> None:
        self._loadConfigurations(config_file)

        # Robot Manager Variables
        self._dt = float(self._config.get('dt', 0.01))
        self._number_of_robots = int(self._config.get('number_of_robots', 1))
        self._robots: List[Robot] = []

        self._build_robots()


    @property
    def dt(self) -> float:
        return self._dt

    @property
    def number_of_robots(self) -> int:
        return self._number_of_robots


    def _loadConfigurations(self, config_file: str) -> None:
        self._config: dict[str, Any] = {}
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

    def _build_robots(self) -> None:
        c, dt = self._config, self._dt
        rows = c.get('robots')
        if isinstance(rows, list) and rows:
            self._robots = [
                _robot_class_for_key(str(dict(r).get('robot', 'little_reader')))(
                    dt=dt,
                    stride_length=dict(r).get('stride_length', 0.0)
                )
                for r in rows
            ]
            self._number_of_robots = len(self._robots)
            return


    def stride_length(self, robot_id: int) -> float:
        return self._robots[robot_id].stride_length

    def get_state(self, robot_id: int) -> StateFrame:
        return self._robots[robot_id].get_state()

    def set_action(self, action_frame_list: List[ActionFrame]) -> None:
        for robot_id, frame in enumerate(action_frame_list):
            if robot_id >= len(self._robots):
                break
            self._robots[robot_id].set_action(frame)
