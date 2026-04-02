from __future__ import annotations

from typing import Dict, Tuple

from common_robot_interface import Action, ActionFrame, State, StateFrame

from robot_interface.scheduler import Scheduler

transition_table: Dict[Tuple[State, Action], State] = {
    (State.STOPPED, Action.HOME): State.HOMMING,
    (State.STOPPED, Action.MOVE): State.OPERATING,
    (State.STOPPED, Action.STOP): State.STOPPED,
    (State.HOMMING, Action.HOME): State.HOMMING,
    (State.HOMMING, Action.STOP): State.STOPPED,
    (State.OPERATING, Action.MOVE): State.OPERATING,
    (State.OPERATING, Action.STOP): State.STOPPED,
}


class FsmScheduler(Scheduler):
    def __init__(self, dt: float) -> None:
        super().__init__(dt)

    def tick(self, frame: ActionFrame) -> bool:
        is_event = False

        key = (self._current_state.state, frame.action)
        next_kind = transition_table.get(key, self._current_state.state)

        if next_kind != self._current_state.state:
            is_event = True

        self._current_state = StateFrame(state=next_kind, progress=0.0)

        return is_event
