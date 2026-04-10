# planner

`ament_python` trajectory planners built on **`robot_interface.TrajectoryPlanner`**. Used with **`scheduler`** and concrete robots in **`robots/`**.

---

## `src/planner/gait_trajectory_planner.py`

### Variables (module)

None.

### Classes

#### `GaitTrajectoryPlanner` (`TrajectoryPlanner`)

Foot swing / stance style trajectory: linear interpolation in x/y (and other state dims) plus a **vertical bump** from parabolic time scaling.

| Member | Description |
|--------|-------------|
| `_clearance` | Peak foot clearance (m) added along index `2` of the state vector in `eval`. |
| `_duration` | Nominal segment duration (s); updated with `update_goal_state`. |

| Method | Description |
|--------|-------------|
| `__init__(clearance, duration)` | Calls `TrajectoryPlanner.__init__`, stores `_clearance` and `_duration`. |
| `update_goal_state(goal_state, clearance, duration)` | Calls `super().update_goal_state(goal_state)` (stores `_goal_state`), then overwrites `_clearance` and `_duration` with the passed values. |
| `eval(s)` | `s` in \([0,1]\): blends `_init_state` → `_goal_state` with `_quintic_time_scaling(s)` for horizontal/planar dims, adds `_parabolic_time_scaling(s) * _clearance` to component `[2]` (vertical). Returns a 1D `np.ndarray` same shape as init/goal. |

### Functions

Module-level free functions: none.

---

## `src/planner/__init__.py`

### Variables / classes / functions

Re-exports **`GaitTrajectoryPlanner`** (`__all__ = ['GaitTrajectoryPlanner']`).
