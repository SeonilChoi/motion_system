from __future__ import annotations

from enum import Enum

class JoyAxes(Enum):
    LEFT_HORIZONTAL       = 0
    LEFT_VERTICAL         = 1
    LT                    = 2
    RIGHT_HORIZONTAL      = 3
    RIGHT_VERTICAL        = 4
    RT                    = 5
    LEFT_RIGHT_DIRECTION  = 6
    UP_DOWN_DIRECTION     = 7


class JoyButton(Enum):
    A          = 0
    B          = 1
    X          = 2
    Y          = 3
    LB         = 4
    RB         = 5
    BACK       = 6
    START      = 7
    HOME       = 8
    LEFT_AXES  = 9
    RIGHT_AXES = 10