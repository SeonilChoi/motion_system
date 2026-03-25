from __future__ import annotations

from typing import Dict, Tuple

from common_robot_interface import Action, ActionKind, State, StateKind

transition_table: Dict[Tuple[StateKind, ActionKind], StateKind] = {
    (StateKind.STOPPED, ActionKind.HOME): StateKind.HOMMING,
    (StateKind.STOPPED, ActionKind.MOVE): StateKind.OPERATING,
    (StateKind.STOPPED, ActionKind.STOP): StateKind.STOPPED,
    (StateKind.STOPPED, ActionKind.WALK): StateKind.WALKING,
    (StateKind.HOMMING, ActionKind.HOME): StateKind.HOMMING,
    (StateKind.HOMMING, ActionKind.STOP): StateKind.STOPPED,
    (StateKind.OPERATING, ActionKind.MOVE): StateKind.OPERATING,
    (StateKind.OPERATING, ActionKind.STOP): StateKind.STOPPED,
    (StateKind.WALKING, ActionKind.WALK): StateKind.WALKING,
    (StateKind.WALKING, ActionKind.STOP): StateKind.STOPPED,
}


class GaitScheduler(FsmScheduler):
    def tick(self, action: Action) -> State:
        key = (self._current_state.kind, action.kind)
        next_kind = _gait_table.get(key, self._current_state.kind)
        self._current_state = State(kind=next_kind)
        return self._current_state
