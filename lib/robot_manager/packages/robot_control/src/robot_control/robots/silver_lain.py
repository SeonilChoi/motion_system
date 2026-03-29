from __future__ import annotations

from typing import Any, Optional

import numpy as np

from common_robot_interface import Action, ActionFrame, StateFrame
from common_robot_interface.joint import JointState
from common_robot_interface.robot import Pose, RobotState, Twist, Vector3, Wrench

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


def _home_joint_vector(home_joint_positions: Any) -> np.ndarray:
    table = home_joint_positions if home_joint_positions is not None else _DEFAULT_HOME_JOINT_RAD
    arr = np.asarray(table, dtype=np.float64)
    if arr.shape != (6, 3):
        raise ValueError(
            f"home_joint_positions must be 6×3 (six legs, three joints each), got {arr.shape}"
        )
    return arr.reshape(-1)


class SilverLain(Robot):
    def __init__(
        self,
        robot_id: int = 0,
        dt: float = 0.01,
        stride_length: float = 0.0,
        clearance: float = 0.05,
        controller_indexes: Optional[list[int]] = None,
        home_joint_positions: Any = None,
    ) -> None:
        super().__init__(robot_id, dt, stride_length, clearance, controller_indexes)

        self._home_positions: np.ndarray = _home_joint_vector(home_joint_positions)

        self._scheduler = GaitScheduler(self._dt, LEG_GROUP_A, LEG_GROUP_B)
        
        self._events : List[Event] = []

        self._trajectory_planner = GaitTrajectoryPlanner(self._clearance)

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

        self._first_step = True

        self._target_pose: np.ndarray = None

        self._target_positions: np.ndarray = None
        
        self._kinematic_solver = SilverLainSolver(LINK_LIST)


    def _compute_target_state(self, event: bool, progress: float, duration: float, goal: np.ndarray, linear_direction: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        vx, vy, wz = goal[0], goal[1], goal[2]

        theta = self._curr_robot_state.pose.orientation.z
        dx = (vx * np.cos(theta) - vy * np.sin(theta)) * self._dt
        dy = (vx * np.sin(theta) + vy * np.cos(theta)) * self._dt
        dw = wz * self._dt

        remaining_time = (1.0 - progress) * duration
        gx = (vx * np.cos(theta) - vy * np.sin(theta)) * remaining_time
        gy = (vx * np.sin(theta) + vy * np.cos(theta)) * remaining_time
        gw = 0

        goal_pose = np.array([
            self._curr_robot_state.pose.position.x + gx,
            self._curr_robot_state.pose.position.y + gy,
            0.0,
            self._curr_robot_state.pose.orientation.z + gw
        ])

        goal_points = self._kinematic_solver.forward_with_pose(goal_pose, self._home_positions)[:, -1, :]
        goal_points[:, :2] += self._stride_length * 0.5 * linear_direction
        goal_points[:, -1] = 0.0

        for e in self._events:
            if e.event == EventKind.TOUCH_DOWN:
                goal_points[e.leg] = self._trajectory_planner.initial_state[e.leg]

        self._trajectory_planner.update_goal_state(goal_points, duration)

        target_pose = np.array([
            self._curr_robot_state.pose.position.x + dx,
            self._curr_robot_state.pose.position.y + dy,
            0.0,
            self._curr_robot_state.pose.orientation.z + dw
        ])

        target_points = self._trajectory_planner.eval(self._scheduler.current_state.progress)
        return target_pose, target_points

    
    def get_state(self) -> StateFrame:
        return self._scheduler.current_state

    def set_action(self, frame: ActionFrame) -> None:
        duration = 0.0

        if frame.action == Action.HOME:
            self._first_step = True
            self._scheduler.tick(frame, duration)
        elif frame.action == Action.MOVE:
            self._scheduler.tick(frame, duration)
        elif frame.action == Action.STOP:
            self._scheduler.tick(frame, duration)
        else:
            stride_length = self._stride_length * 0.5 if self._first_step else self._stride_length

            linear_velocity = np.array([frame.goal[0], frame.goal[1]])
            linear_speed = np.linalg.norm(linear_velocity)
            if linear_speed == 0.0:
                duration = 10.0
            else:
                linear_direction = linear_velocity / linear_speed    
                normalized_linear_speed = np.clip(linear_speed, 0.0, 1.0) * 0.1 # 0.1 m/s is the maximum speed
                duration = stride_length / normalized_linear_speed
                linear_velocity = normalized_linear_speed * linear_direction

            event = self._scheduler.tick(frame, duration)

            if self._scheduler.events:
                self._events = self._scheduler.events

                init_points = np.array([
                    [v.x, v.y, v.z] for v in self._curr_robot_state.pose.points 
                ])
                self._trajectory_planner.set_initial_state(init_points)
            
            if self._events:
                self._target_pose, target_points = self._compute_target_state(event, self._scheduler.current_state.progress, duration, new_frame.goal, linear_direction)
                self._target_positions = self._kinematic_solver.inverse_with_pose(self._target_pose, target_points)
                return self._target_positions

        if frame.action == Action.WALK and not np.all(frame.goal == 0.0):
            self._scheduler.step()

        return self._curr_joint_states.position

    def get_robot_state(self) -> RobotState:
        return self._curr_robot_state

    def set_joint_state(self, joint_states: JointState) -> None:
        self._curr_joint_states = JointState(
            motor_id=joint_states.motor_id.copy(),
            position=joint_states.position.copy(),
            velocity=joint_states.velocity.copy(),
            torque=joint_states.torque.copy(),
        )
        
        curr_pose = np.array([
            self._curr_robot_state.pose.position.x,
            self._curr_robot_state.pose.position.y,
            0.0,
            self._curr_robot_state.pose.orientation.z
        ])
        points = self._kinematic_solver.forward_with_pose(curr_pose, self._curr_joint_states.position)
        point_list = [Vector3(float(p[0]), float(p[1]), float(p[2])) for p in points]
        self._curr_robot_state = RobotState(
            robot_id=self._robot_id,
            pose=Pose(
                position=self._curr_robot_state.pose.position,
                orientation=self._curr_robot_state.pose.orientation,
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

        if self._target_pose is not None and self._target_positions is not None:
            points = self._kinematic_solver.forward_with_pose(self._target_pose, self._target_positions)[:, -1, :]
            point_list = [Vector3(float(p[0]), float(p[1]), float(p[2])) for p in points]

            self._curr_joint_states = JointState(
                motor_id=self._curr_joint_states.motor_id.copy(),
                position=self._target_positions.copy(),
                velocity=self._curr_joint_states.velocity.copy(),
                torque=self._curr_joint_states.torque.copy(),
            )

            self._curr_robot_state = RobotState(
                robot_id=self._robot_id,
                pose=Pose(
                    position=Vector3(self._target_pose[0], self._target_pose[1], 0.0),
                    orientation=Vector3(0.0, 0.0, self._target_pose[3]),
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
