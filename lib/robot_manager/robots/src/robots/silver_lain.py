from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

from common_robot_interface import Action, ActionFrame, StateFrame
from common_robot_interface import JointStatus, RobotStatus

from robot_interface.robot import Robot, RobotConfig
from scheduler.gait_scheduler import Event, EventKind, GaitScheduler
from planner.gait_trajectory_planner import GaitTrajectoryPlanner
from kinematics.silver_lain_solver import SilverLainSolver


LEG_GROUP_A = [0, 2, 4]
LEG_GROUP_B = [1, 3, 5]


LINK_LIST = [0.32, 0.0, 0.615, 1.25]


class SilverLain(Robot):
    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config)

        # Gait Scheduler Variables
        self._events : Optional[List[Event]] = None
        self._scheduler = GaitScheduler(self._dt, LEG_GROUP_A, LEG_GROUP_B)
        
        # Gait Trajectory Planner Variables
        self._trajectory_planner = [GaitTrajectoryPlanner(self._clearance, self._duration) for _ in range(6)]

        # Kinematic Solver Variables
        self._kinematic_solver = SilverLainSolver(LINK_LIST)

        # Robot Variables
        self._curr_joint_status: JointStatus = JointStatus(
            motor_id=np.asarray(self._controller_indexes, dtype=np.int8),
            position=self._home_joint_positions.copy(),
            velocity=np.zeros(self._number_of_motors, dtype=np.float64),
            torque=np.zeros(self._number_of_motors, dtype=np.float64),
        )

        home_point = self._kinematic_solver.forward_with_pose(self._home_pose.copy(), self._home_joint_positions.copy())
        self._curr_robot_state: RobotStatus = RobotStatus(
            robot_id=self._robot_id,
            pose=self._home_pose.copy(),
            point=home_point.copy(),
            twist=np.zeros(6, dtype=np.float64),
            wrench=np.zeros(6, dtype=np.float64),
        )


    def _compute_next_target(self, duration: float, goal: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        stride_length = self._stride_length * 0.5 if self._scheduler.first_step else self._stride_length

        direction = np.array([goal[0], goal[1]])
        linear_velocity = stride_length / (duration * 0.5) * direction

        vx, vy, wz = linear_velocity[0], linear_velocity[1], goal[2]

        theta = self._curr_robot_state.pose[5] # orientation.z
        dx = (vx * np.cos(theta) - vy * np.sin(theta)) * self._dt
        dy = (vx * np.sin(theta) + vy * np.cos(theta)) * self._dt
        dw = wz * self._dt

        if self._scheduler.current_state.progress == 0.5:
            subprogress = 1.0
        elif self._scheduler.current_state.progress == 1.0:
            subprogress = 1.0
        elif self._scheduler.current_state.progress < 0.5:
            subprogress = self._scheduler.current_state.progress / 0.5
        else:
            subprogress = (self._scheduler.current_state.progress - 0.5) / 0.5

        remaining_time = (1.0 - subprogress) * (duration * 0.5)
        gx = (vx * np.cos(theta) - vy * np.sin(theta)) * remaining_time
        gy = (vx * np.sin(theta) + vy * np.cos(theta)) * remaining_time
        gw = 0

        goal_pose = self._curr_robot_state.pose.copy()
        goal_pose[0] += gx
        goal_pose[1] += gy
        goal_pose[5] += gw

        goal_point = self._kinematic_solver.forward_with_pose(goal_pose.copy(), self._home_joint_positions.copy())
        forward_position = (self.stride_length * 0.5) * direction
        goal_point[:, :2] += forward_position
        goal_point[:, -1] = 0.0

        target_pose = self._curr_robot_state.pose.copy()
        target_pose[0] += dx
        target_pose[1] += dy
        target_pose[5] += dw

        target_points = np.zeros((6, 3), dtype=np.float64)
        for e in self._events:
            if e.event == EventKind.TOUCH_DOWN:
                self._trajectory_planner[e.leg].update_goal_state(self._trajectory_planner[e.leg].initial_state, 0.0, duration * 0.5)
            else:
                self._trajectory_planner[e.leg].update_goal_state(goal_point[e.leg], self._clearance, duration * 0.5)

            target_points[e.leg] = self._trajectory_planner[e.leg].eval(subprogress)

        return target_pose, target_points
    

    def get_state_frame(self) -> StateFrame:
        return self._scheduler.current_state

    def set_action_frame(self, frame: ActionFrame) -> np.ndarray:
        commands = np.zeros((self._number_of_motors, 3))
        commands[:, 0] = self._curr_joint_status.position.copy()
        
        if frame.action == Action.HOME:
            self._scheduler.set_first_step(True)

        event = self._scheduler.tick(frame)

        if frame.action == Action.WALK and event and len(self._scheduler._events) > 0:
            self._events = list(self._scheduler._events)
            initial_point = self._kinematic_solver.forward_with_pose(self._curr_robot_state.pose.copy(), self._curr_joint_status.position.copy())
            for i in range(6):
                self._trajectory_planner[i].set_initial_state(initial_point[i])
        
        if frame.action == Action.WALK and self._events is not None:
            target_pose, target_point = self._compute_next_target(frame.duration, frame.goal)

            target_position = self._kinematic_solver.inverse_with_pose(target_pose.copy(), target_point.copy())
            commands[:, 0] = target_position.copy()

        if frame.action == Action.WALK and not np.all(frame.goal == 0.0):
            self._scheduler.step()

        return commands

    def get_robot_status(self) -> RobotStatus:
        return self._curr_robot_state

    def update_joint_status(self, joint_status: JointStatus) -> None:
        self._curr_joint_status.position = joint_status.position
        self._curr_joint_status.velocity = joint_status.velocity
        self._curr_joint_status.torque = joint_status.torque

    def reset(self) -> None:
        # Gait Scheduler Variables
        self._events = None
        self._scheduler.reset()
        
        # Robot Variables
        self._curr_joint_status = JointStatus(
            motor_id=np.asarray(self._controller_indexes, dtype=np.int8),
            position=self._home_joint_positions.copy(),
            velocity=np.zeros(self._number_of_motors, dtype=np.float64),
            torque=np.zeros(self._number_of_motors, dtype=np.float64),
        )

        home_point = self._kinematic_solver.forward_with_pose(self._home_pose.copy(), self._home_joint_positions.copy())
        self._curr_robot_state = RobotStatus(
            robot_id=self._robot_id,
            pose=self._home_pose.copy(),
            point=home_point.copy(),
            twist=np.zeros(6, dtype=np.float64),
            wrench=np.zeros(6, dtype=np.float64),
        )