from __future__ import annotations

import copy
import yaml
import numpy as np
from pathlib import Path
from typing import Any, List

from common_robot_interface import ActionFrame, StateFrame
from common_robot_interface import JointStatus, RobotStatus

from robot_interface.robot import Robot, RobotConfig
from robots.little_reader import LittleReader
from robots.silver_lain import SilverLain


_ROBOT_BY_KEY: dict[str, type[Robot]] = {
    'little_reader': LittleReader,
    'silver_lain': SilverLain,
}


class RobotManager:
    def __init__(self, config_file: str) -> None:
        self._config: dict[str, Any] = {}
        self._dt = 0.01
        self._number_of_motors = 0
        self._number_of_robots = 0
        self._robots: List[Robot] = []
        self._loadConfigurations(config_file)
    
        self._home_joint_status = JointStatus(
            motor_id=np.arange(self._number_of_motors),
            interface_id=np.concatenate([robot.interface_ids for robot in self._robots]),
            position=np.concatenate(
                [np.asarray(robot.home_joint_positions, dtype=np.float64).reshape(-1) for robot in self._robots]
            ),
            velocity=np.zeros(self._number_of_motors, dtype=np.float64),
            torque=np.zeros(self._number_of_motors, dtype=np.float64),
        )
        

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
        path = Path((config_file or '').strip()).expanduser()
        if not path.is_file():
            return

        raw = yaml.safe_load(path.read_text(encoding='utf-8'))
        if not isinstance(raw, dict):
            return

        self._config = raw
        self._dt = float(raw.get('dt', 0.01))
        dt = self._dt
        rows = raw.get('robots')
        if not isinstance(rows, list):
            return

        robots: List[Robot] = []
        motors = 0

        for row in rows:
            if not isinstance(row, dict):
                continue

            ctrl = np.asarray(row['controller_indexes'], dtype=np.int32).reshape(-1).copy()
            iface = np.asarray(row['interface_ids'], dtype=np.int32).reshape(-1).copy()

            hj = np.asarray(row['home_joint_positions'], dtype=np.float64).reshape(-1).copy()
            home_pose = np.asarray(row['home_pose'], dtype=np.float64).reshape(-1).copy()

            cfg = RobotConfig(
                robot_id=int(row.get('id', 0)),
                dt=dt,
                stride_length=float(row.get('stride_length', 0.0)),
                clearance=float(row.get('clearance', 0.05)),
                controller_indexes=ctrl,
                interface_ids=iface,
                home_joint_positions=hj,
                home_pose=home_pose,
                duration=float(row.get('duration', 5.0)),
            )
            key = str(row.get('robot', 'little_reader')).lower().replace(' ', '_')
            impl = _ROBOT_BY_KEY.get(key, LittleReader)
            robots.append(impl(cfg))
            motors += int(ctrl.size)

        self._robots = robots
        self._number_of_motors = motors
        self._number_of_robots = len(robots)


    def stride_length(self, robot_id: int) -> float:
        return self._robots[robot_id].stride_length

    def duration(self, robot_id: int) -> float:
        return self._robots[robot_id].duration


    def get_state_frame(self, robot_id: int) -> StateFrame:
        return self._robots[robot_id].get_state_frame()

    def set_action_frame(self, action_frame_list: List[ActionFrame]) -> JointStatus:
        joint_commands = copy.deepcopy(self._home_joint_status)

        for robot_id, frame in enumerate(action_frame_list):
            commands = self._robots[robot_id].set_action_frame(frame)

            sel = np.asarray(self._robots[robot_id].controller_indexes, dtype=int)
            joint_commands.position[sel] = commands[:, 0]
            joint_commands.velocity[sel] = commands[:, 1]
            joint_commands.torque[sel] = commands[:, 2]

        return joint_commands

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

    def reset(self) -> None:
        for robot in self._robots:
            robot.reset()