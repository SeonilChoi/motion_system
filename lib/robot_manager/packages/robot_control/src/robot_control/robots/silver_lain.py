from __future__ import annotations

from common_robot_interface import Action, ActionKind, State

from robot_interface.robot import Robot

from robot_control.scheduler.gait_scheduler import GaitScheduler


LEG_GROUP_A = [0, 2, 4]
LEG_GROUP_B = [1, 3, 5]

class SilverLain(Robot):
    def __init__(self, dt: float = 0.01) -> None:
        super().__init__(dt)
        
        self._scheduler = GaitScheduler(dt, LEG_GROUP_A, LEG_GROUP_B)

    def get_state(self) -> State:
        return self._scheduler.current_state

    def set_action(self, action: Action) -> None:
        event = self._scheduler.tick(action)

        if event:
            if action.kind == ActionKind.HOME:
                pass
            elif action.kind == ActionKind.MOVE:
                pass

        if self._scheduler.events is not None:
            pass

        if action.kind == ActionKind.STOP:
            pass
        
        if action.kind == ActionKind.WALK and action.duration != 0.0:
            self._scheduler.step()