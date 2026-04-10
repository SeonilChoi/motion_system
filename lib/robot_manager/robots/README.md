# robots

`ament_python` concrete **`robot_interface.Robot`** implementations. Selected from YAML in **`robot_manager`**. Depends on **`common_robot_interface`**, **`robot_interface`**, **`scheduler`**, **`planner`**, **`kinematics`** (for `SilverLain`).

---

## `src/robots/silver_lain.py`

### Variables (module)

| Name | Description |
|------|-------------|
| `LEG_GROUP_A` | `[0, 2, 4]` — tripod group A for `GaitScheduler` offsets. |
| `LEG_GROUP_B` | `[1, 3, 5]` — tripod group B. |
| `LINK_LIST` | `[0.32, 0.0, 0.615, 1.25]` — link lengths passed to **`SilverLainSolver`**. |

### Classes

#### `SilverLain` (`Robot`)

Hexapod with **`GaitScheduler`**, six **`GaitTrajectoryPlanner`** instances, and **`SilverLainSolver`** IK/FK. Caches **`JointStatus`** and **`RobotStatus`**; walk commands drive stance/swing foot targets from gait events.

| Member | Description |
|--------|-------------|
| `_events` | Last copied list of gait `Event`s from `GaitScheduler` (or `None`). |
| `_scheduler` | `GaitScheduler(dt, LEG_GROUP_A, LEG_GROUP_B)`. |
| `_trajectory_planner` | List of six `GaitTrajectoryPlanner(clearance, duration)`. |
| `_kinematic_solver` | `SilverLainSolver(LINK_LIST)`. |
| `_curr_joint_status` | Current feedback mirror (`position` starts at home). |
| `_curr_robot_state` | `RobotStatus` with pose, foot `point` from FK at home, zero twist/wrench. |

| Method | Description |
|--------|-------------|
| `__init__(config)` | Builds scheduler, planners, solver, and initial joint/robot status from `RobotConfig` / home FK. |
| `_compute_next_target(duration, goal)` | From `goal` \([dx, dy, yaw\_rate]\) and scheduler `progress`, computes body `target_pose` step and per-leg `target_points` using stride (`_stride_length`, halved on `first_step`), integrates velocity in world frame, sets planner goals on `TOUCH_DOWN` (hold) vs swing (foot target + clearance), returns `(target_pose, target_points)`. |
| `get_state_frame()` | Returns `_scheduler.current_state`. |
| `set_action_frame(frame)` | Initializes `commands` to current positions; on `Action.HOME` sets scheduler `first_step`. Calls `_scheduler.tick`. On `WALK` with events, snapshots foot positions into each planner’s `initial_state`. On `WALK` with `_events`, runs `_compute_next_target`, IK `inverse_with_pose`, writes position column. If `WALK` and goal not all zeros, calls `_scheduler.step()`. Returns `commands` `(n_motors, 3)` (position column filled; others left zero). |
| `get_robot_status()` | Returns `_curr_robot_state`. |
| `update_joint_status(joint_status)` | Copies position/velocity/torque into `_curr_joint_status`. |
| `reset()` | Clears `_events`, resets scheduler, rebuilds joint/robot state to home FK snapshot. |

### Functions

None.

---

## `src/robots/little_reader.py`

### Variables (module)

None.

### Classes

#### `LittleReader` (`Robot`)

Placeholder / passive robot: **`FsmScheduler`** only; **`set_action_frame`** echoes the last joint feedback as three vectors (no kinematics).

| Member | Description |
|--------|-------------|
| `_scheduler` | `FsmScheduler(config.dt)`. |
| `_curr_joint_status` | Last `JointStatus` from `update_joint_status`, or `None`. |

| Method | Description |
|--------|-------------|
| `__init__(config)` | Stores scheduler and `_curr_joint_status = None`. |
| `get_state_frame()` | `_scheduler.current_state`. |
| `set_action_frame(frame)` | `tick(frame)` on scheduler. Returns `(position, velocity, torque)` each length `n_motors`: zeros if no joint status yet, else copies of current p/v/t (zeros for any `None` field). **Note:** return type is a **3-tuple** of arrays, not a single `(n, 3)` matrix (differs from `SilverLain`). |
| `get_robot_status()` | Synthetic `RobotStatus` with zero pose, `(6, 3)` points, twist, wrench. |
| `update_joint_status(joint_status)` | Deep-copies fields into `_curr_joint_status`. |

### Functions

None.

---

## `src/robots/__init__.py`

### Variables / classes / functions

Re-exports **`LittleReader`**, **`SilverLain`**, and (from **`scheduler`**) **`FsmScheduler`**, **`GaitScheduler`**, **`Scheduler`**:  
`__all__ = ['FsmScheduler', 'GaitScheduler', 'LittleReader', 'Scheduler', 'SilverLain']`.
