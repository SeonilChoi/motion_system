from __future__ import annotations

from common_robot_interface import Action, ActionFrame, StateFrame

from robot_interface.robot import Robot

from robot_control.scheduler.gait_scheduler import GaitScheduler
#from robot_control.planner.gait_trajectory_planner import GaitTrajectoryPlanner


LEG_GROUP_A = [0, 2, 4]
LEG_GROUP_B = [1, 3, 5]


class SilverLain(Robot):
    def __init__(self, dt: float = 0.01, stride_length: float = 0.0) -> None:
        super().__init__(dt, stride_length)

        self._scheduler = GaitScheduler(dt, LEG_GROUP_A, LEG_GROUP_B)

        #self._trajectory_planner = GaitTrajectoryPlanner()

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
