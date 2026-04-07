from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, List, Tuple

import numpy as np

from common_robot_interface import Action, ActionFrame, State, StateFrame

from robot_interface.scheduler import Scheduler


transition_table: Dict[Tuple[State, Action], State] = {
    (State.STOPPED, Action.HOME): State.HOMMING,
    (State.STOPPED, Action.MOVE): State.OPERATING,
    (State.STOPPED, Action.STOP): State.STOPPED,
    (State.STOPPED, Action.WALK): State.WALKING,
    (State.HOMMING, Action.HOME): State.HOMMING,
    (State.HOMMING, Action.STOP): State.STOPPED,
    (State.OPERATING, Action.MOVE): State.OPERATING,
    (State.OPERATING, Action.STOP): State.STOPPED,
    (State.WALKING, Action.WALK): State.WALKING,
    (State.WALKING, Action.STOP): State.STOPPED,
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

        self._T: float = 0.0

        self._prev_progress: float = 0.0
        
        self._offset: Dict[int, float] = {}
        for leg in leg_group_a:
            self._offset[leg] = 0.0
        for leg in leg_group_b:
            self._offset[leg] = 0.5 # 0.5 is the offset for the leg group B

        self._prev_phase: Dict[int, Phase] = {leg: Phase.STANCE for leg in range(len(leg_group_a + leg_group_b))}
        for leg in self._prev_phase.keys():
            if self._offset[leg] < 0.5:
                self._prev_phase[leg] = Phase.SWING
            else:
                self._prev_phase[leg] = Phase.STANCE
        
        self._events: List[Event] = []

        self._first_step = True


    @property
    def events(self) -> List[Event]:
        return self._events

    @property
    def first_step(self) -> bool:
        return self._first_step


    def _progress_raw(self, t: float) -> float:
        if self._T == 0.0:
            return 0.0
        if self._T == t:
            return 1.0
        return round(t / self._T, 3)

    def _progress_leg(self, t: float, leg: int) -> float:
        if self._T == 0.0:
            return 0.0
        
        res = round((t / self._T + self._offset[leg]) % 1.0, 3)
        if res == 0.0:
            return 1.0
        else:
            return res

    def tick(self, frame: ActionFrame) -> bool:
        is_event = False
        self._events = []

        key = (self._current_state.state, frame.action)
        next_kind = transition_table.get(key, self._current_state.state)
        if next_kind != self._current_state.state:
            is_event = True

        if frame.action != Action.WALK:
            self._current_state = StateFrame(state=next_kind, progress=0.0)
        else:
            if np.all(frame.goal == 0.0):
                self._current_state = StateFrame(state=next_kind, progress=self._prev_progress)
            else:
                self._T = frame.duration
                self._t = self._T * self._prev_progress
                t = self._t + self._dt

                progress = self._progress_raw(t)
                self._current_state = StateFrame(state=next_kind, progress=progress)

                self._prev_progress = progress

                for leg in self._prev_phase.keys():
                    curr_progress = self._progress_leg(t, leg)

                    prev_phase = self._prev_phase[leg]
                    curr_phase = Phase.STANCE if curr_progress <= 0.5 else Phase.SWING

                    if prev_phase != curr_phase:
                        if prev_phase == Phase.STANCE and curr_phase == Phase.SWING:
                            self._events.append(Event(leg=leg, event=EventKind.LIFT_OFF))
                        elif prev_phase == Phase.SWING and curr_phase == Phase.STANCE:
                            self._events.append(Event(leg=leg, event=EventKind.TOUCH_DOWN))

                    self._prev_phase[leg] = curr_phase

                if len(self._events) > 0:
                    is_event = True

                if progress > 0.5:
                    self._first_step = False

                if progress >= 1.0:
                    self._t = 0.0
                    self._prev_progress = 0.0

        return is_event

    def step(self) -> None:
        self._t += self._dt

    def set_first_step(self, first_step: bool) -> None:
        self._first_step = first_step

    def reset(self) -> None:
        self._t = 0.0
        
        self._prev_progress = 0.0
        
        self._events = []
        
        self._first_step = True
