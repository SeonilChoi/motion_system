from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, List, Tuple

from common_robot_interface import Action, ActionKind, State, StateKind

from robot_interface.scheduler import Scheduler

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


class Phase(Enum):
    STANCE = auto()
    SWING = auto()


class EventKind(Enum):
    TOUCH_DOWN = auto()
    LIFT_OFF = auto()


@dataclass
class Event:
    leg: int
    event: EventKind


class GaitScheduler(Scheduler):
    def __init__(self, dt: float, leg_group_a: list[int], leg_group_b: list[int]) -> None:
        super().__init__(dt)

        self._prev_progress = 0.0

        self._offset : Dict[int, float] = {}
        for leg in leg_group_a:
            self._offset[leg] = 0.0
        for leg in leg_group_b:
            self._offset[leg] = 0.5

        self._prev_phase : Dict[int, Phase] = {leg: Phase.STANCE for leg in leg_group_a + leg_group_b}
    
        self._events : List[Event] = []

    @property
    def events(self) -> List[Event]:
        return self._events

    def step(self) -> None:
        self._t += self._dt

    def tick(self, action: Action) -> bool:
        is_event = False
        self._events.clear()
        
        key = (self._current_state.kind, action.kind)
        next_kind = transition_table.get(key, self._current_state.kind)
        if next_kind != self._current_state.kind:
            is_event = True

        if action.kind != ActionKind.WALK:
            self._current_state = State(kind=next_kind)
        else:
            if action.duration == 0.0:
                self._current_state = State(kind=next_kind, progress=self._prev_progress)
            else:
                self._T = action.duration
                self._t = self._T * self._prev_progress
                t = self._t + self._dt

                progress = self._progress_raw(t)
                self._current_state = State(kind=next_kind, progress=progress)

                self._prev_progress = progress

                for leg in self._prev_phase.keys():
                    curr_progress = self._progress_leg(self._t, leg)

                    prev_phase = self._prev_phase[leg]
                    curr_phase = Phase.STANCE if curr_progress < 0.5 else Phase.SWING

                    if prev_phase != curr_phase:
                        if prev_phase == Phase.STANCE and curr_phase == Phase.SWING:
                            self._events.append(Event(leg=leg, event=EventKind.LIFT_OFF))
                        elif prev_phase == Phase.SWING and curr_phase == Phase.STANCE:
                            self._events.append(Event(leg=leg, event=EventKind.TOUCH_DOWN))
                    
                    self._prev_phase[leg] = curr_phase

                if progress >= 1.0:
                    self._t = 0.0
                    self._prev_progress = 0.0

        return is_event

    def _progress_raw(self, t: float) -> float:
        if self._T == 0.0:
            return 0.0
        if self._T == t:
            return 1.0
        return round(t / self._T, 3)

    def _progress_leg(self, t: float, leg: int) -> float:
        if self._T == 0.0:
            return 0.0
        if self._T == t:
            return 1.0
        return round((t / self._T + self._offset[leg]) % 1.0, 3)
