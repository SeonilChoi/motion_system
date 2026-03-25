# robot_control

Python package: `robot_control`. Schedulers and concrete robots.

## `__init__.py`

**`__all__`:** `FsmScheduler`, `GaitScheduler`, `LittleReader`, `Scheduler`, `SilverLain`  
(`Scheduler` is re-exported from `robot_interface` via `scheduler/__init__.py`.)

## `scheduler/__init__.py`

**`__all__`:** `FsmScheduler`, `GaitScheduler`, `Scheduler` — pulls `Scheduler` from `robot_interface.scheduler`.

## `robots/__init__.py`

**`__all__`:** `LittleReader`, `SilverLain`.

---

## `scheduler/fsm_scheduler.py`

### `transition_table: Dict[Tuple[StateKind, ActionKind], StateKind]`

Finite-state transition map: current state kind + action kind → next state kind. Used by `FsmScheduler.tick`.

### `class FsmScheduler(Scheduler)`

FSM-only scheduler (no walk progression).

#### Constructor

`__init__(self, dt: float) -> None` — calls `Scheduler.__init__(dt)`.

#### Methods

| Method | Signature | Returns | Meaning |
|--------|-----------|---------|---------|
| `tick` | `(action: Action) -> bool` | `True` if `StateKind` changed this tick, else `False`. | Looks up `(self._current_state.kind, action.kind)` in `transition_table`; updates `_current_state` to next kind (or unchanged if missing). |

---

## `scheduler/gait_scheduler.py`

### `transition_table: Dict[Tuple[StateKind, ActionKind], StateKind]`

Same pattern as FSM table but includes `WALK` transitions into `WALKING` and self-loop on `WALK`.

### `class Phase(Enum)`

| Member | Meaning |
|--------|---------|
| `STANCE` | Leg in stance phase (`progress < 0.5` in leg-local frame). |
| `SWING` | Leg in swing phase (progress ≥ 0.5). |

### `class EventKind(Enum)`

| Member | Meaning |
|--------|---------|
| `TOUCH_DOWN` | Transition swing → stance for a leg. |
| `LIFT_OFF` | Transition stance → swing for a leg. |

### `@dataclass class Event`

| Field | Type | Meaning |
|-------|------|---------|
| `leg` | `int` | Leg index. |
| `event` | `EventKind` | Event type. |

### `class GaitScheduler(Scheduler)`

Walk-capable scheduler with per-leg phase offsets and contact events.

#### Constructor

`__init__(self, dt: float, leg_group_a: list[int], leg_group_b: list[int]) -> None`

| Private attribute | Meaning |
|-------------------|---------|
| `_prev_progress` | Previous gait progress scalar. |
| `_offset` | Per-leg phase offset (`0.0` for group A, `0.5` for group B). |
| `_prev_phase` | Last `Phase` per leg. |
| `_events` | List of `Event` produced in the last `tick` (cleared at start of each `tick`). |
| `_T` | Walk period in seconds; set when handling `WALK` with non-zero `duration`. |

#### Properties

| Property | Type | Meaning |
|----------|------|---------|
| `events` | `List[Event]` | Events from the most recent `tick` (empty if none). |

#### Methods

| Method | Signature | Meaning |
|--------|-----------|---------|
| `step` | `() -> None` | Increments `_t` by `_dt` (used by `SilverLain` after walk ticks). |
| `tick` | `(action: Action) -> bool` | Clears `_events`. Updates FSM state via `transition_table`. If action is not `WALK`, sets state to next kind only. If `WALK` with `duration == 0`, keeps progress at `_prev_progress`. If `WALK` with non-zero `duration`, advances time, computes `progress`, updates leg phases, appends `TOUCH_DOWN` / `LIFT_OFF` events on phase edges, resets time at `progress >= 1.0`. Returns `True` if state kind changed. |
| `_progress_raw` | `(t: float) -> float` | Normalized progress `t / _T` (rounded to 3 decimals); handles `_T == 0`. |
| `_progress_leg` | `(t: float, leg: int) -> float` | Leg-local progress: `(t / _T + offset[leg]) % 1.0` (rounded). |

---

## `robots/little_reader.py`

### `class LittleReader(Robot)`

| Attribute | Meaning |
|-----------|---------|
| `_scheduler` | `FsmScheduler` instance. |

| Method | Meaning |
|--------|---------|
| `get_state` | Returns `_scheduler.current_state`. |
| `set_action` | Calls `_scheduler.tick(action)`. |

`__init__(self, dt: float = 0.01)` builds `FsmScheduler(dt)`.

---

## `robots/silver_lain.py`

### Module constants

| Name | Value | Meaning |
|------|-------|---------|
| `LEG_GROUP_A` | `[0, 2, 4]` | Leg indices with 0.0 gait offset. |
| `LEG_GROUP_B` | `[1, 3, 5]` | Leg indices with 0.5 gait offset. |

### `class SilverLain(Robot)`

| Attribute | Meaning |
|-----------|---------|
| `_scheduler` | `GaitScheduler(dt, LEG_GROUP_A, LEG_GROUP_B)`. |

| Method | Meaning |
|--------|---------|
| `get_state` | Returns `_scheduler.current_state`. |
| `set_action` | Calls `_scheduler.tick(action)`; on `WALK` with `duration != 0`, calls `_scheduler.step()`. Contains placeholder `pass` branches for events and other kinds (extend as needed). |

`__init__(self, dt: float = 0.01)` constructs the gait scheduler.
