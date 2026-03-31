from __future__ import annotations

from typing import Any, List, Optional, Tuple

import numpy as np

from common_robot_interface import Action, ActionFrame, State, StateFrame
from common_robot_interface import JointStatus, RobotStatus

from robot_interface.robot import Robot
from robot_control.scheduler.gait_scheduler import Event, EventKind, GaitScheduler
from robot_control.planner.gait_trajectory_planner import GaitTrajectoryPlanner
from robot_control.kinematics.silver_lain_solver import SilverLainSolver


LEG_GROUP_A = [0, 2, 4]
LEG_GROUP_B = [1, 3, 5]


LINK_LIST = [0.32, 0.0, 0.615, 1.25]

# Default home pose (rad): legs 0,2,4 → 0, 40°, 140°; legs 1,3,5 → 0, -40°, -140°.
_R40 = 0.6981317007977318
_R140 = 2.443460952792061
_DEFAULT_HOME_JOINT_RAD = np.array(
    [
        [0.0, _R40, _R140],
        [0.0, -_R40, -_R140],
        [0.0, _R40, _R140],
        [0.0, -_R40, -_R140],
        [0.0, _R40, _R140],
        [0.0, -_R40, -_R140],
    ],
    dtype=np.float64,
)


class SilverLain(Robot):
    def __init__(
        self,
        robot_id: int = 0,
        dt: float = 0.01,
        stride_length: float = 0.0,
        clearance: float = 0.05,
        controller_indexes: Optional[list[int]] = None,
        interface_ids: Optional[list[int]] = None,
        home_joint_positions: Any = None,
    ) -> None:
        super().__init__(robot_id, dt, stride_length, clearance, controller_indexes, interface_ids, home_joint_positions)

        # Gait Scheduler Variables
        self._events : List[Event] = []
        self._scheduler = GaitScheduler(self._dt, LEG_GROUP_A, LEG_GROUP_B)
        
        # Gait Trajectory Planner Variables
        self._first_step = True
        self._trajectory_planner = GaitTrajectoryPlanner(self._clearance)

        # Kinematic Solver Variables
        self._kinematic_solver = SilverLainSolver(LINK_LIST)

        # Robot Variables
        self._curr_joint_status: JointStatus = JointStatus(
            motor_id=np.asarray(self._controller_indexes, dtype=np.int8),
            position=np.zeros(self._number_of_motors, dtype=np.float64),
            velocity=np.zeros(self._number_of_motors, dtype=np.float64),
            torque=np.zeros(self._number_of_motors, dtype=np.float64),
        )

        self._curr_robot_state: RobotStatus = RobotStatus(
            robot_id=self._robot_id,
            pose=np.zeros(6, dtype=np.float64),
            point=np.zeros((6, 3), dtype=np.float64),
            twist=np.zeros(6, dtype=np.float64),
            wrench=np.zeros(6, dtype=np.float64),
        )


    def _compute_next_target(self, duration: float, goal: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        duration = duration * 0.5 if self._first_step else duration
        stride_length = self._stride_length * 0.5 if self._first_step else self._stride_length
        
        vx, vy, wz = goal[0], goal[1], goal[2]

        theta = self._curr_robot_state.pose[5] # orientation.z
        dx = (vx * np.cos(theta) - vy * np.sin(theta)) * self._dt
        dy = (vx * np.sin(theta) + vy * np.cos(theta)) * self._dt
        dw = wz * self._dt

        remaining_time = (1.0 - self._scheduler.current_state.progress) * duration
        gx = (vx * np.cos(theta) - vy * np.sin(theta)) * remaining_time
        gy = (vx * np.sin(theta) + vy * np.cos(theta)) * remaining_time
        gw = 0

        p = self._curr_robot_state.pose
        goal_pose = np.array(
            [
                p[0] + gx,
                p[1] + gy,
                p[2],
                p[3],
                p[4],
                p[5] + gw,
            ],
            dtype=np.float64,
        )

        goal_point = self._kinematic_solver.forward_with_pose(goal_pose, self._home_joint_positions)
        goal_point[:, 0] += gx
        goal_point[:, 1] += gy
        goal_point[:, 2] = 0.0

        for e in self._events:
            if e.event == EventKind.TOUCH_DOWN:
                goal_point[e.leg] = self._trajectory_planner.initial_state[e.leg]

        self._trajectory_planner.update_goal_state(goal_point, duration)

        p = self._curr_robot_state.pose
        target_pose = np.array(
            [
                p[0] + dx,
                p[1] + dy,
                p[2],
                p[3],
                p[4],
                p[5] + dw,
            ],
            dtype=np.float64,
        )

        target_points = self._trajectory_planner.eval(self._scheduler.current_state.progress)
        return target_pose, target_points
    
    def get_state_frame(self) -> StateFrame:
        return self._scheduler.current_state

    def set_action_frame(self, frame: ActionFrame) -> np.ndarray:
        target_position = self._curr_joint_status.position.copy()
        
        if frame.action == Action.HOME:
            self._first_step = True

        event = self._scheduler.tick(frame)
        
        if frame.action == Action.WALK:
            if event:
                self._events = self._scheduler.events
                current_point = self._kinematic_solver.forward_with_pose(self._curr_robot_state.pose, self._curr_joint_status.position)
                self._trajectory_planner.set_initial_state(current_point)
    
            target_pose, target_point = self._compute_next_target(frame.duration, frame.goal)

            target_position = self._kinematic_solver.inverse_with_pose(target_pose, target_point)

            # For test
            self._curr_robot_state.pose = target_pose
            self._curr_robot_state.point = target_point
            self._curr_joint_status.position = target_position

        if frame.action == Action.WALK and not np.all(frame.goal == 0.0):
            self._scheduler.step()

        return target_position

    def get_robot_status(self) -> RobotStatus:
        return self._curr_robot_state

    def update_joint_status(self, joint_status: JointStatus) -> None:
        self._curr_joint_status.position = joint_status.position
        self._curr_joint_status.velocity = joint_status.velocity
        self._curr_joint_status.torque = joint_status.torque
