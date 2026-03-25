# common_robot_interface

Python package: `common_robot_interface`. Source: `src/common_robot_interface/`.

## Module `common_robot_interface` (`__init__.py`)

**Exports (`__all__`)**  
`Action`, `ActionKind`, `JoyAxes`, `JoyButton`, `State`, `StateKind`

**Usage**

```python
from common_robot_interface import Action, ActionKind, State, StateKind, JoyAxes, JoyButton
```

---

## `action.py`

### `class ActionKind(Enum)`

| Member | Meaning |
|--------|---------|
| `HOME` | Request or run homing-related action. |
| `MOVE` | Request or run generic move / operate action. |
| `WALK` | Request or run walking gait action. |
| `STOP` | Request or run stop action. |

### `@dataclass(frozen=True, slots=True) class Action`

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `kind` | `ActionKind` | (required) | Active action kind. |
| `duration` | `float` | `0.0` | Walk segment duration (seconds); used when `kind` is `WALK`. |
| `goal` | `np.ndarray` | `zeros(3)` | Walk direction / goal vector; indices `0,1` used as `vx, vy` from joystick in `RobotManager`. |

**Construction**

```python
Action(kind=ActionKind.STOP)
Action(kind=ActionKind.WALK, duration=2.0, goal=np.array([1.0, 0.0, 0.0]))
```

---

## `state.py`

### `class StateKind(Enum)`

| Member | Meaning |
|--------|---------|
| `HOMMING` | Robot in homing. |
| `OPERATING` | Robot in non-walk operating mode. |
| `WALKING` | Robot in walk gait. |
| `STOPPED` | Robot stopped / idle. |

### `@dataclass(frozen=True, slots=True) class State`

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `kind` | `StateKind` | (required) | Current high-level state. |
| `progress` | `float` | `0.0` | Gait or motion progress in `[0, 1]` when applicable (`GaitScheduler`). |

---

## `joy.py`

### `class JoyAxes(Enum)`

Maps logical axes to indices into `sensor_msgs/msg/Joy.axes`.

| Member | Value |
|--------|-------|
| `LEFT_HORIZONTAL` | 0 |
| `LEFT_VERTICAL` | 1 |
| `LT` | 2 |
| `RIGHT_HORIZONTAL` | 3 |
| `RIGHT_VERTICAL` | 4 |
| `RT` | 5 |
| `LEFT_RIGHT_DIRECTION` | 6 |
| `UP_DOWN_DIRECTION` | 7 |

**Usage:** `msg.axes[JoyAxes.LEFT_VERTICAL.value]`

### `class JoyButton(Enum)`

Maps logical buttons to indices into `sensor_msgs/msg/Joy.buttons`.

| Member | Value |
|--------|-------|
| `A` … `RIGHT_AXES` | 0 … 10 |

**Usage:** `msg.buttons[JoyButton.A.value]`

---

## `setup.py`

Package metadata and install configuration for the setuptools/ament Python package (not a runtime API).
