from __future__ import annotations

from pathlib import Path
from typing import Any, List, Mapping, Optional

import numpy as np
import yaml

from common_robot_interface import ActionFrame, StateFrame
from common_robot_interface.joint import JointState
from robot_interface.robot import Robot

from robot_control.robots.little_reader import LittleReader
from robot_control.robots.silver_lain import SilverLain


_ROBOT_BY_KEY: dict[str, type[Robot]] = {
    'little_reader': LittleReader,
    'silver_lain': SilverLain,
}


def _robot_class_for_key(key: str) -> type[Robot]:
    return _ROBOT_BY_KEY.get(key.lower().replace(' ', '_'), LittleReader)

def _as_controller_indexes(value: Any) -> Optional[list[int]]:
    if value is None:
        return None
    if isinstance(value, (list, tuple)):
        return [int(x) for x in value]
    return None


class RobotManager:
    def __init__(self, config_file: str) -> None:
        self._loadConfigurations(config_file)

        self._number_of_motors: int = 0
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

    @property
    def number_of_motors(self) -> int:
        return self._number_of_motors


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
            self._robots = []
            for r in rows:
                row = dict(r)
                cls = _robot_class_for_key(str(row.get('robot', 'little_reader')))
                rid = int(row.get('id', 0))
                stride = float(row.get('stride_length', 0.0))
                ctrl = _as_controller_indexes(row.get('controller_indexes'))
                self._robots.append(
                    cls(
                        robot_id=rid,
                        dt=dt,
                        stride_length=stride,
                        clearance=float(row.get('clearance', 0.05)),
                        controller_indexes=ctrl,
                        home_joint_positions=row.get('home_joint_positions'),
                    )
                )
                self._number_of_motors += len(ctrl) if ctrl else 0
            self._number_of_robots = len(self._robots)
            return


    def get_state(self, robot_id: int) -> StateFrame:
        return self._robots[robot_id].get_state()

    def set_action(self, action_frame_list: List[ActionFrame]) -> JointState:
        positions = np.zeros(self._number_of_motors)
        for robot_id, frame in enumerate(action_frame_list):
            if robot_id >= len(self._robots):
                break
            positions[self._robots[robot_id].controller_indexes] = self._robots[robot_id].set_action(frame)

        commands = JointState(
            motor_id=np.arange(self._number_of_motors),
            position=positions,
            velocity=np.zeros(self._number_of_motors),
            torque=np.zeros(self._number_of_motors),
        )
        return commands

    def get_robot_states(self, robot_id: int) -> RobotState:
        return self._robots[robot_id].get_robot_state()

    def set_joint_states(self, joint_states: JointState) -> None:
        for robot in self._robots:
            idx = robot.controller_indexes
            if not idx:
                continue
            sel = np.asarray(idx, dtype=int)
            sub = JointState(
                motor_id=joint_states.motor_id[sel],
                position=joint_states.position[sel],
                velocity=joint_states.velocity[sel],
                torque=joint_states.torque[sel],
            )
            robot.set_joint_state(sub)
