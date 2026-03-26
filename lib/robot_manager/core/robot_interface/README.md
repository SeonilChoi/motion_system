# robot_interface

Python package: `robot_interface`. Source: `src/robot_interface/`.

## `__init__.py`

**`__all__`:** `Robot` — re-exported from `robot.py`.

## `robot.py`

### `class Robot(ABC)`

Abstract robot facade: timestep, optional per-robot `stride_length`, `StateFrame` readout, and `ActionFrame` injection.

#### Constructor

`__init__(self, dt: float = 0.01, stride_length: float = 0.0) -> None`

| Attribute | Meaning |
|-----------|---------|
| `_dt` | Scheduler step period (seconds). |
| `_stride_length` | Nominal stride (meters) for teleop walk timing; exposed via property. |

#### Properties

| Property | Type | Meaning |
|----------|------|---------|
| `stride_length` | `float` | Value passed at construction (from YAML per robot in `RobotManager`). |

#### Methods

| Method | Signature | Meaning |
|--------|-----------|---------|
| `get_state` | `() -> StateFrame` | Current `StateFrame` from the concrete robot (usually `scheduler.current_state`). |
| `set_action` | `(frame: ActionFrame) -> None` | Apply one command frame (drives `Scheduler.tick` in implementations). |

**Subclassing:** implement `get_state` and `set_action`.

---

## `scheduler.py`

### `class Scheduler(ABC)`

Abstract scheduler: owns simulated time, current `StateFrame`, and reset behavior.

#### Constructor

`__init__(self, dt: float) -> None`

| Attribute | Initial value | Meaning |
|-----------|---------------|---------|
| `_dt` | `dt` | Tick duration (seconds). |
| `_t` | `0.0` | Internal time accumulator (`GaitScheduler`). |
| `_current_state` | `StateFrame(state=State.STOPPED, progress=0.0)` | Last computed state. |

#### Properties

| Property | Type | Meaning |
|----------|------|---------|
| `current_state` | `StateFrame` | Read-only view of `_current_state`. |

#### Methods

| Method | Signature | Meaning |
|--------|-----------|---------|
| `reset` | `() -> None` | Set `_t` to `0.0` and `_current_state` to stopped. |
| `tick` | `(frame: ActionFrame) -> bool` | Abstract: advance one step. Implementations return whether the **state enum** changed this tick (`FsmScheduler`, `GaitScheduler`). |

**Subclassing:** implement `tick`.
