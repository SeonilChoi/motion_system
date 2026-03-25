from __future__ import annotations

from typing import Dict, Tuple

from common_robot_interface import Action, ActionKind, State, StateKind

from robot_interface.scheduler import Scheduler

transition_table: Dict[Tuple[StateKind, ActionKind], StateKind] = {
    (StateKind.STOPPED, ActionKind.HOME): StateKind.HOMMING,
    (StateKind.STOPPED, ActionKind.MOVE): StateKind.OPERATING,
    (StateKind.STOPPED, ActionKind.STOP): StateKind.STOPPED,
    (StateKind.HOMMING, ActionKind.HOME): StateKind.HOMMING,
    (StateKind.HOMMING, ActionKind.STOP): StateKind.STOPPED,
    (StateKind.OPERATING, ActionKind.MOVE): StateKind.OPERATING,
    (StateKind.OPERATING, ActionKind.STOP): StateKind.STOPPED,
}


class FsmScheduler(Scheduler):
    def __init__(self, dt: float) -> None:
        super().__init__(dt)

    def tick(self, action: Action) -> bool:
        is_event = False
        
        key = (self._current_state.kind, action.kind)
        next_kind = transition_table.get(key, self._current_state.kind)
        
        if next_kind != self._current_state.kind:
            is_event = True

        self._current_state = State(kind=next_kind)

        return is_event