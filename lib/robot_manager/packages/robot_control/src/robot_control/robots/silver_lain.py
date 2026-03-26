from __future__ import annotations

from typing import Optional

import numpy as np

from common_robot_interface import Action, ActionFrame, StateFrame
from common_robot_interface.joint import JointState
from common_robot_interface.robot import Pose, RobotState, Twist, Vector3, Wrench

from robot_interface.robot import Robot

from robot_control.scheduler.gait_scheduler import GaitScheduler
from robot_control.planner.gait_trajectory_planner import GaitTrajectoryPlanner
from robot_control.kinematics.silver_lain_solver import SilverLainSolver


LEG_GROUP_A = [0, 2, 4]
LEG_GROUP_B = [1, 3, 5]


LINK_LIST = [0.32, 0.0, 0.615, 1.25]


class SilverLain(Robot):
    def __init__(
        self,
        robot_id: int = 0,
        dt: float = 0.01,
        stride_length: float = 0.0,
        controller_indexes: Optional[list[int]] = None,
    ) -> None:
        super().__init__(robot_id, dt, stride_length, controller_indexes)

        self._scheduler = GaitScheduler(self._dt, LEG_GROUP_A, LEG_GROUP_B)

        self._trajectory_planner = GaitTrajectoryPlanner()

        self._motor_id = np.asarray(self._controller_indexes or [], dtype=np.int32)
        
        self._curr_joint_states: JointState = JointState(
            motor_id=np.asarray(self._controller_indexes or [], dtype=np.int32),
            position=np.zeros(0, dtype=np.float64),
            velocity=np.zeros(0, dtype=np.float64),
            torque=np.zeros(0, dtype=np.float64),
        )

        self._curr_robot_state: RobotState = RobotState(
            robot_id=self._robot_id,
            pose=Pose(
                position=Vector3(0.0, 0.0, 0.0),
                orientation=Vector3(0.0, 0.0, 0.0),
                points=[],
            ),
            twist=Twist(
                linear=Vector3(0.0, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, 0.0),
            ),
            wrench=Wrench(
                force=Vector3(0.0, 0.0, 0.0),
                torque=Vector3(0.0, 0.0, 0.0),
            ),
        )
        
        self._kinematic_solver = SilverLainSolver(LINK_LIST)


    def _compute_goal_position(self, goal: np.ndarray) -> np.ndarray:
        pass


    def get_state(self) -> StateFrame:
        return self._scheduler.current_state

    def set_action(self, frame: ActionFrame) -> None:
        event: bool = self._scheduler.tick(frame)

        if event:
            if frame.action == Action.HOME:
                pass
            elif frame.action == Action.MOVE:
                pass

        if frame.action == Action.WALK:
            if self._scheduler.events:
                pass

        if frame.action == Action.STOP:
            pass

        if frame.action == Action.WALK and frame.duration != 0.0:
            self._scheduler.step()

    def get_robot_state(self) -> RobotState:
        return self._curr_robot_state

    def set_joint_state(self, joint_states: JointState) -> None:
        self._curr_joint_states = JointState(
            motor_id=joint_states.motor_id.copy(),
            position=joint_states.position.copy(),
            velocity=joint_states.velocity.copy(),
            torque=joint_states.torque.copy(),
        )
        
        points = self._kinematic_solver.forward(self._curr_joint_states.position)[:, -1, :]
        point_list = [Vector3(float(x), float(y), float(z)) for x, y, z in points]
        self._curr_robot_state = RobotState(
            robot_id=self._robot_id,
            pose=Pose(
                position=Vector3(0.0, 0.0, 0.0),
                orientation=Vector3(0.0, 0.0, 0.0),
                points=point_list,
            ),
            twist=Twist(
                linear=Vector3(0.0, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, 0.0),
            ),
            wrench=Wrench(
                force=Vector3(0.0, 0.0, 0.0),
                torque=Vector3(0.0, 0.0, 0.0),
            ),
        )
