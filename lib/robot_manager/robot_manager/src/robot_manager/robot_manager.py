from __future__ import annotations

import yaml
import numpy as np
from pathlib import Path
from typing import Any, List, Mapping, Optional

from common_robot_interface import ActionFrame, StateFrame
from common_robot_interface import JointStatus, RobotStatus

from robot_interface.robot import Robot
from robot_control.robots.little_reader import LittleReader
from robot_control.robots.silver_lain import SilverLain


_ROBOT_BY_KEY: dict[str, type[Robot]] = {
    'little_reader': LittleReader,
    'silver_lain': SilverLain,
}


def _robot_class_for_key(key: str) -> type[Robot]:
    return _ROBOT_BY_KEY.get(key.lower().replace(' ', '_'), LittleReader)

def _as_array(value: Any) -> Optional[np.ndarray]:
    if value is None:
        return None
    if isinstance(value, (list, tuple)):
        return [int(x) for x in value]
    return None

def _home_joint_vector(home_joint_positions: Any) -> np.ndarray:
    table = home_joint_positions if home_joint_positions is not None else _DEFAULT_HOME_JOINT_RAD
    arr = np.asarray(table, dtype=np.float64)
    if arr.shape != (6, 3):
        raise ValueError(
            f"home_joint_positions must be 6×3 (six legs, three joints each), got {arr.shape}"
        )
    return arr.reshape(-1)


class RobotManager:
    def __init__(self, config_file: str) -> None:
        self._loadConfigurations(config_file)

        self._dt = float(self._config.get('dt', 0.01))
        self._number_of_motors: int = 0
        self._number_of_robots = int(self._config.get('number_of_robots', 1))
        
        self._robots: List[Robot] = []
        self._build_robots()


    @property
    def dt(self) -> float:
        return self._dt

    @property
    def number_of_motors(self) -> int:
        return self._number_of_motors

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
            self._robots = []
            for r in rows:
                row = dict(r)
                
                cls = _robot_class_for_key(str(row.get('robot', 'little_reader')))
                
                rid = int(row.get('id', 0))
                stride = float(row.get('stride_length', 0.0))
                ctrl = _as_array(row.get('controller_indexes'))
                interface_ids = _as_array(row.get('interface_ids'))
                home_joint_positions = _home_joint_vector(row.get('home_joint_positions'))
                
                self._robots.append(
                    cls(
                        robot_id=rid,
                        dt=dt,
                        stride_length=stride,
                        clearance=float(row.get('clearance', 0.05)),
                        controller_indexes=ctrl,
                        interface_ids=interface_ids,
                        home_joint_positions=home_joint_positions,
                    )
                )
                self._number_of_motors += len(ctrl) if ctrl else 0
            self._number_of_robots = len(self._robots)
            return


    def stride_length(self, robot_id: int) -> float:
        return self._robots[robot_id].stride_length

    def get_state_frame(self, robot_id: int) -> StateFrame:
        return self._robots[robot_id].get_state_frame()

    def set_action_frame(self, action_frame_list: List[ActionFrame]) -> JointStatus:
        joint_command = JointStatus(
            motor_id=np.arange(self._number_of_motors),
            interface_id=np.zeros(self._number_of_motors, dtype=np.int32),
            position=np.zeros(self._number_of_motors, dtype=np.float64),
            velocity=np.zeros(self._number_of_motors, dtype=np.float64),
            torque=np.zeros(self._number_of_motors, dtype=np.float64),
        )
        
        for robot_id, frame in enumerate(action_frame_list):
            commands = self._robots[robot_id].set_action_frame(frame)

            sel = np.asarray(self._robots[robot_id].controller_indexes, dtype=int)
            
            joint_command.motor_id[sel] = self._robots[robot_id].controller_indexes
            joint_command.interface_id[sel] = np.asarray(self._robots[robot_id].interface_ids, dtype=int)
            joint_command.position[sel] = commands

        return joint_command

    def get_robot_status(self, robot_id: int) -> RobotStatus:
        return self._robots[robot_id].get_robot_status()

    def update_joint_status(self, joint_status: JointStatus) -> None:
        for robot in self._robots:
            idx = robot.controller_indexes
            sel = np.asarray(idx, dtype=int)
            sub = JointStatus(
                motor_id=joint_status.motor_id[sel],
                position=joint_status.position[sel],
                velocity=joint_status.velocity[sel],
                torque=joint_status.torque[sel],
            )
            robot.update_joint_status(sub)
