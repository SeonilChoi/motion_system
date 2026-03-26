# robot_control

Python package: `robot_control`. Schedulers and concrete robots.

## `__init__.py`

**`__all__`:** `FsmScheduler`, `GaitScheduler`, `LittleReader`, `Scheduler`, `SilverLain`  
(`Scheduler` is re-exported from `robot_interface` via `scheduler/__init__.py`.)

## `scheduler/__init__.py`

**`__all__`:** `FsmScheduler`, `GaitScheduler`, `Scheduler` — pulls `Scheduler` from `robot_interface.scheduler`.

## `robots/__init__.py`

**`__all__`:** `LittleReader`, `SilverLain`.

## `planner/__init__.py`

Exports `GaitTrajectoryPlanner`.

## `kinematics/__init__.py`

Exports `SilverLainSolver`.

---

## `scheduler/fsm_scheduler.py`

### `transition_table: Dict[Tuple[State, Action], State]`

Finite-state map: **current** `State` enum + incoming `Action` enum → **next** `State` enum. Keys use the shared `Action` / `State` types from `common_robot_interface`.

### `class FsmScheduler(Scheduler)`

FSM-only scheduler (no walk phase progression).

#### Constructor

`__init__(self, dt: float) -> None` — calls `Scheduler.__init__(dt)`.

#### Methods

| Method | Signature | Returns | Meaning |
|--------|-----------|---------|---------|
| `tick` | `(frame: ActionFrame) -> bool` | `True` if `State` **enum** changed this tick. | Looks up `(self._current_state.state, frame.action)`; sets `_current_state` to `StateFrame(state=next_kind, progress=0.0)`. |

---

## `scheduler/gait_scheduler.py`

### `transition_table: Dict[Tuple[State, Action], State]`

Same pattern as the FSM table, plus `WALK` transitions into `WALKING` and self-loop on `WALK`.

### `class Phase(Enum)`

| Member | Meaning |
|--------|---------|
| `STANCE` | Leg in stance (`progress < 0.5` in leg-local normalized time). |
| `SWING` | Leg in swing (`progress ≥ 0.5`). |

### `class EventKind(Enum)`

| Member | Meaning |
|--------|---------|
| `TOUCH_DOWN` | Swing → stance edge for a leg. |
| `LIFT_OFF` | Stance → swing edge for a leg. |

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
| `_T` | Walk period (s); set when handling `WALK` with non-zero `frame.duration`. |
| `_prev_progress` | Previous gait progress scalar. |
| `_offset` | Per-leg phase offset (`0.0` group A, `0.5` group B). |
| `_prev_phase` | Last `Phase` per leg. |
| `_events` | `Event` list produced in the last `tick` (cleared at start of each `tick`). |

#### Properties

| Property | Type | Meaning |
|----------|------|---------|
| `events` | `List[Event]` | Events from the most recent `tick`. |

#### Methods

| Method | Signature | Meaning |
|--------|-----------|---------|
| `step` | `() -> None` | `_t += _dt` (used by `SilverLain` after walk ticks). |
| `tick` | `(frame: ActionFrame) -> bool` | Clears `_events`. Uses `transition_table` with `(self._current_state.state, frame.action)`. If `frame.action != WALK`, sets `StateFrame(state=next_kind, progress=0.0)`. If `WALK`, advances time from `frame.duration`, updates `progress` and leg phases, emits `TOUCH_DOWN` / `LIFT_OFF`, resets at `progress >= 1.0`. Returns whether the **state enum** changed. |
| `_progress_raw` | `(t: float) -> float` | `t / _T` (3-decimal rounding); handles `_T == 0`. |
| `_progress_leg` | `(t: float, leg: int) -> float` | `(t / _T + offset[leg]) % 1.0` (rounded). |

---

## `robots/little_reader.py`

### `class LittleReader(Robot)`

| Attribute | Meaning |
|-----------|---------|
| `_scheduler` | `FsmScheduler` instance. |

| Method | Meaning |
|--------|---------|
| `get_state` | Returns `_scheduler.current_state` (`StateFrame`). |
| `set_action` | `_scheduler.tick(frame)`. |

`__init__(self, robot_id=0, dt=0.01, stride_length=0.0, controller_indexes=None)`.

Additional methods:

- `get_robot_state() -> Optional[RobotState]` returns `None` (no geometric state yet)
- `set_joint_state(joint_states: JointState)` stores latest feedback

---

## `robots/silver_lain.py`

### Module constants

| Name | Value | Meaning |
|------|-------|---------|
| `LEG_GROUP_A` | `[0, 2, 4]` | Leg indices, offset `0.0`. |
| `LEG_GROUP_B` | `[1, 3, 5]` | Leg indices, offset `0.5`. |

### `class SilverLain(Robot)`

| Attribute | Meaning |
|-----------|---------|
| `_scheduler` | `GaitScheduler(dt, LEG_GROUP_A, LEG_GROUP_B)`. |

| Method | Meaning |
|--------|---------|
| `get_state` | Returns `_scheduler.current_state`. |
| `set_action` | `tick(frame)`; placeholder branches for `HOME` / `MOVE` / `STOP` / `WALK` events; if `WALK` and `frame.duration != 0`, calls `_scheduler.step()`. |

`__init__(self, robot_id=0, dt=0.01, stride_length=0.0, controller_indexes=None)`:

- builds `GaitScheduler`, `GaitTrajectoryPlanner`, `SilverLainSolver`
- stores latest `JointState`
- computes and updates `RobotState.pose.points` on `set_joint_state`

Extra methods:

- `get_robot_state() -> RobotState`
- `set_joint_state(joint_states: JointState) -> None`
