# scheduler

`ament_python` high-level **`robot_interface.Scheduler`** implementations: discrete FSM and gait phasing. Consume **`ActionFrame`** / **`State`** / **`StateFrame`** from **`common_robot_interface`**.

---

## `src/scheduler/fsm_scheduler.py`

### Variables (module)

| Name | Type | Description |
|------|------|-------------|
| `transition_table` | `Dict[Tuple[State, Action], State]` | Maps `(current_state, action)` to next `State`. Covers `STOPPED`, `HOMMING`, `OPERATING` with `HOME`, `MOVE`, `STOP`. **No `WALK`** row (use `GaitScheduler` for walking). |

### Classes

#### `FsmScheduler` (`Scheduler`)

Minimal state machine: each `tick` replaces `current_state` with the table lookup (or unchanged state) and resets **`progress`** to `0.0`.

| Method | Description |
|--------|-------------|
| `__init__(dt)` | Passes `dt` to `Scheduler`. |
| `tick(frame)` | Looks up `(self._current_state.state, frame.action)` in `transition_table`; if the next state differs, sets `is_event = True`. Updates `_current_state` to `StateFrame(next_state, progress=0.0)`. Returns `is_event`. |

### Functions

None.

---

## `src/scheduler/gait_scheduler.py`

### Variables (module)

| Name | Type | Description |
|------|------|-------------|
| `transition_table` | `Dict[Tuple[State, Action], State]` | Like the FSM table plus **`WALK`**: e.g. `STOPPED+WALK → WALKING`, `WALKING+WALK → WALKING`, `WALKING+STOP → STOPPED`. |

### Classes

#### `Phase` (enum)

| Member | Meaning |
|--------|---------|
| `STANCE` | Leg in support (normalized phase ≤ 0.5). |
| `SWING` | Leg in swing (> 0.5). |

#### `EventKind` (enum)

| Member | Meaning |
|--------|---------|
| `TOUCH_DOWN` | Swing → stance transition. |
| `LIFT_OFF` | Stance → swing transition. |

#### `Event` (dataclass)

| Field | Type | Meaning |
|-------|------|---------|
| `leg` | `int` | Leg index `0…5`. |
| `event` | `EventKind` | `TOUCH_DOWN` or `LIFT_OFF`. |

#### `GaitScheduler` (`Scheduler`)

Tripod-style gait clock: two leg groups with **0 / 0.5** phase offsets, per-leg stance/swing detection, and optional `Event` list each cycle when `Action.WALK` advances time.

| Property | Description |
|----------|-------------|
| `events` | Copy of `_events` produced in the last `tick` (may be empty). |
| `first_step` | `True` until global progress exceeds `0.5` during a walk (then cleared). |

| Method | Description |
|--------|-------------|
| `__init__(dt, leg_group_a, leg_group_b)` | Stores period `dt`, builds `_offset[leg]` (`0.0` for group A, `0.5` for group B), initializes `_prev_phase` from offsets (group A starts in `SWING`, B in `STANCE`), clears `_events`, sets `_first_step = True`. Initializes `_T = 0.0`, `_prev_progress = 0.0`. `_t` (time along the gait period) is set when processing `Action.WALK` with a non-zero goal. |
| `_progress_raw(t)` | If `_T == 0` return `0`; if `_T == t` return `1`; else `round(t / _T, 3)`. |
| `_progress_leg(t, leg)` | Normalized leg phase: `(t / _T + _offset[leg]) % 1`, rounded; `0` mapped to `1.0`. |
| `tick(frame)` | Applies FSM transition from `transition_table` for non-`WALK` actions (resets `progress` to `0`). For **`WALK`**: if `goal` is all zeros, keeps `progress` at `_prev_progress`; else integrates `_t`, updates global `progress`, detects stance/swing edges per leg, appends `Event`s, updates `_first_step` / wraps at `progress >= 1.0`. Returns `True` if FSM state changed or any gait event fired. |
| `step()` | Increments `_t` by `_dt` (used when the robot layer advances time without a full gait tick update). |
| `set_first_step(first_step)` | Sets `_first_step`. |
| `reset()` | Clears `_t`, `_prev_progress`, `_events`, and sets `_first_step = True` (does not rebuild offsets / phases). |

### Functions

None.

---

## `src/scheduler/__init__.py`

### Variables / classes / functions

Re-exports **`Scheduler`** (from `robot_interface`), **`FsmScheduler`**, **`GaitScheduler`** (`__all__ = ['FsmScheduler', 'GaitScheduler', 'Scheduler']`).
