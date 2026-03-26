# common_robot_interface

Python package: `common_robot_interface`. Source: `src/common_robot_interface/`.

## Module `common_robot_interface` (`__init__.py`)

**Exports (`__all__`)**  
`Action`, `ActionFrame`, `JointState`, `State`, `StateFrame`

**Usage**

```python
from common_robot_interface import Action, ActionFrame, JointState, State, StateFrame
```

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
| `goal` | `np.ndarray` | `zeros(3)` |

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

## `joint.py`

### `@dataclass(frozen=True, slots=True) class JointState`

| Field | Type | Default |
|-------|------|---------|
| `motor_id` | `np.ndarray` | (required) |
| `position` | `Optional[np.ndarray]` | `None` |
| `velocity` | `Optional[np.ndarray]` | `None` |
| `torque` | `Optional[np.ndarray]` | `None` |
