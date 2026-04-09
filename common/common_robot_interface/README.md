# common_robot_interface

Python package: `common_robot_interface`. Source: `src/common_robot_interface/`.

## Package exports (`__all__`)

`Action`, `ActionFrame`, `JointStatus`, `RobotStatus`, `State`, `StateFrame`

```python
from common_robot_interface import (
    Action,
    ActionFrame,
    JointStatus,
    RobotStatus,
    State,
    StateFrame,
)
```

---

## `action.py`

### `class Action(Enum)`

| Member |
|--------|
| `HOME` |
| `MOVE` |
| `WALK` |
| `STOP` |

### `@dataclass(frozen=True, slots=True) class ActionFrame`

| Field | Type | Default |
|-------|------|---------|
| `action` | `Action` | (required) |
| `duration` | `float` | `0.0` |
| `goal` | `Optional[np.ndarray]` | `None` |

---

## `state.py`

### `class State(Enum)`

| Member |
|--------|
| `HOMMING` |
| `OPERATING` |
| `WALKING` |
| `STOPPED` |

### `@dataclass(frozen=True, slots=True) class StateFrame`

| Field | Type | Default |
|-------|------|---------|
| `state` | `State` | (required) |
| `progress` | `float` | `0.0` |

---

## `joint.py`

### `@dataclass(slots=True) class JointStatus`

Aggregated joint / motor feedback or command buffers (global or per-robot slices).

| Field | Type | Default |
|-------|------|---------|
| `motor_id` | `Optional[np.ndarray]` | `None` |
| `interface_id` | `Optional[np.ndarray]` | `None` |
| `position` | `Optional[np.ndarray]` | `None` |
| `velocity` | `Optional[np.ndarray]` | `None` |
| `torque` | `Optional[np.ndarray]` | `None` |

---

## `robot.py`

### `@dataclass(slots=True) class RobotStatus`

High-level robot snapshot (pose, feet, wrench, etc.).

| Field | Type |
|-------|------|
| `robot_id` | `int` |
| `pose` | `np.ndarray` |
| `point` | `np.ndarray` |
| `twist` | `np.ndarray` |
| `wrench` | `np.ndarray` |
