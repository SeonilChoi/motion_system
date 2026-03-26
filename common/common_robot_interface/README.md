# common_robot_interface

Python package: `common_robot_interface`. Source: `src/common_robot_interface/`.

## Module `common_robot_interface` (`__init__.py`)

**Exports (`__all__`)**  
`Action`, `ActionFrame`, `JoyAxes`, `JoyButton`, `State`, `StateFrame`

**Usage**

```python
from common_robot_interface import Action, ActionFrame, State, StateFrame, JoyAxes, JoyButton
```

---

## `action.py`

### `class Action(Enum)`

High-level command **kind** (not a dataclass). Used inside `ActionFrame.action` and in FSM transition tables together with `State`.

| Member | Meaning |
|--------|---------|
| `HOME` | Homing-related command. |
| `MOVE` | Generic operate / move command. |
| `WALK` | Walking gait command (`duration` / `goal` on the frame matter for `GaitScheduler`). |
| `STOP` | Stop / idle command. |

### `@dataclass(frozen=True, slots=True) class ActionFrame`

One logical command sample: **kind** plus optional timing and goal vector (ROS / teleop boundary type).

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `action` | `Action` | (required) | Command kind. |
| `duration` | `float` | `0.0` | Walk segment duration (seconds) when `action` is `WALK`. |
| `goal` | `np.ndarray` | `zeros(3)` | Direction / goal; e.g. `vx, vy` from joystick in `robot_manager_node`. |

**Construction**

```python
ActionFrame(action=Action.STOP)
ActionFrame(action=Action.WALK, duration=2.0, goal=np.array([1.0, 0.0, 0.0]))
```

`Robot.set_action` and `Scheduler.tick` take an `ActionFrame` end-to-end.

---

## `state.py`

### `class State(Enum)`

High-level **motion / mode** state. Used inside `StateFrame.state` and in scheduler transition tables.

| Member | Meaning |
|--------|---------|
| `HOMMING` | Homing. |
| `OPERATING` | Non-walk operating. |
| `WALKING` | Walk gait active. |
| `STOPPED` | Stopped / idle. |

### `@dataclass(frozen=True, slots=True) class StateFrame`

Published scheduler state: enum **state** plus scalar **progress**.

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `state` | `State` | (required) | Current high-level state. |
| `progress` | `float` | `0.0` | Gait progress in `[0, 1]` when applicable (`GaitScheduler`). |

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

> **Note:** `scripts/robot_manager_node.py` currently defines **local** `JoyAxes` / `JoyButton` enums with the same axis/button indices so the node stays self-contained; you may switch it to import these from `common_robot_interface` if you prefer a single definition.

### `class JoyButton(Enum)`

Maps logical buttons to indices into `sensor_msgs/msg/Joy.buttons`.

| Member | Value |
|--------|-------|
| `A` … `RIGHT_AXES` | 0 … 10 |

**Usage:** `msg.buttons[JoyButton.A.value]`

---

## `setup.py`

Package metadata and install configuration for the setuptools/ament Python package (not a runtime API).
