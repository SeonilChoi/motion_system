from __future__ import annotations

from common_robot_interface import Action, State

from robot_interface.robot import Robot

from robot_control.scheduler.fsm_scheduler import FsmScheduler


class LittleReader(Robot):
    def __init__(self, dt: float = 0.01) -> None:
        super().__init__(dt)
        self._scheduler = FsmScheduler(dt)

    def get_state(self) -> State:
        return self._scheduler.current_state

    def set_action(self, action: Action) -> None:
        self._scheduler.tick(action)
