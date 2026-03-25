# robot_interface

Python package: `robot_interface`. Source: `src/robot_interface/`.

## `__init__.py`

**`__all__`:** `Robot` — re-exported from `robot.py`.

## `robot.py`

### `class Robot(ABC)`

Abstract robot facade: holds timestep, exposes state and action injection.

#### Constructor

`__init__(self, dt: float = 0.01) -> None`

| Attribute | Meaning |
|-----------|---------|
| `_dt` | Scheduler / simulation step period (seconds). |

#### Methods

| Method | Signature | Meaning |
|--------|-----------|---------|
| `get_state` | `() -> State` | Return current `State` from the concrete robot. |
| `set_action` | `(action: Action) -> None` | Apply a new `Action` (drives scheduler tick in implementations). |

**Subclassing:** implement `get_state` and `set_action`.

---

## `scheduler.py`

### `class Scheduler(ABC)`

Abstract scheduler: owns simulated time, current `State`, and reset behavior.

#### Constructor

`__init__(self, dt: float) -> None`

| Attribute | Initial value | Meaning |
|-----------|---------------|---------|
| `_dt` | `dt` | Tick duration (seconds). |
| `_t` | `0.0` | Internal time accumulator (used by `GaitScheduler`). |
| `_current_state` | `State(kind=StateKind.STOPPED)` | Last computed state. |

#### Properties

| Property | Type | Meaning |
|----------|------|---------|
| `current_state` | `State` | Read-only view of `_current_state`. |

#### Methods

| Method | Signature | Meaning |
|--------|-----------|---------|
| `reset` | `() -> None` | Set `_t` to `0.0` and `_current_state` to stopped. |
| `tick` | `(action: Action) -> State` | Abstract: advance logic for one step given `action`. *(Concrete schedulers in `robot_control` return `bool` from `tick`; see their README.)* |

**Subclassing:** implement `tick`.
