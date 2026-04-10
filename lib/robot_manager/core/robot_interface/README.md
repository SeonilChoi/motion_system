# robot_interface

Python robot abstractions (`ament_python`). Namespace: package `robot_interface` under `src/robot_interface/`. Types such as `ActionFrame`, `StateFrame`, `JointStatus`, and `RobotStatus` come from **`common_robot_interface`**.

---

## `src/robot_interface/robot.py`

### Classes

#### `RobotConfig` (dataclass)

Construction parameters shared by concrete `Robot` implementations (e.g. from YAML).

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `robot_id` | `int` | `0` | Logical robot id. |
| `dt` | `float` | `0.01` | Control period (s). |
| `stride_length` | `float` | `0.0` | Nominal stride (m-class); walk timing / scaling. |
| `clearance` | `float` | `0.05` | Foot / body clearance for planners. |
| `duration` | `float` | `5.0` | Default action duration (s). |
| `controller_indexes` | `Any` | empty `int32` 1D | Indices into the **global** motor vector (`motor_state` ordering). |
| `interface_ids` | `Any` | empty `int32` 1D | Per-motor interface ids for command messages (same length as `controller_indexes`). |
| `home_joint_positions` | `Any` | empty `float64` 1D | Nominal joint vector at home. |
| `home_pose` | `Any` | `zeros(6)` | Nominal base/body pose at home (`x, y, z, R, P, Y`). |

#### `Robot` (ABC)

Base robot: stores a copy of config arrays (`controller_indexes`, `interface_ids`, `home_*`) and exposes read-only properties.

| Property | Description |
|----------|-------------|
| `robot_id`, `stride_length`, `clearance`, `duration` | From `RobotConfig`. |
| `controller_indexes`, `interface_ids` | 1D `int32` views of motor indexing / interface ids. |
| `number_of_motors` | `len(controller_indexes)`. |
| `home_joint_positions`, `home_pose` | 1D `float64` home posture. |

| Method | Description |
|--------|-------------|
| `__init__(config)` | Copies and reshapes config fields into internal arrays. |
| `get_state_frame()` | Scheduler / FSM snapshot as `StateFrame` (subclass implements). |
| `set_action_frame(frame)` | Consumes `ActionFrame`; returns per-motor command array for `RobotManager` (e.g. shape `(n_motors, 3)` for p/v/t). |
| `get_robot_status()` | Kinematics / pose snapshot as `RobotStatus` (subclass implements). |
| `update_joint_status(joint_status)` | Injects feedback `JointStatus` for this robot (subclass implements). |

### Functions

Module-level free functions: none.

---

## `src/robot_interface/scheduler.py`

### Classes

#### `Scheduler` (ABC)

High-level state machine stepper driven by `ActionFrame`.

| Property | Description |
|----------|-------------|
| `current_state` | `StateFrame`; starts as `State.STOPPED`, `progress=0.0`. |

| Method | Description |
|--------|-------------|
| `__init__(dt)` | Stores control period `dt` and internal time `t` (starts at `0.0`). |
| `tick(frame)` | Advance one step from `ActionFrame`; return value is scheduler-specific `bool` (subclass implements). |
| `reset()` | Reset internal time and state (subclass implements). |

### Functions

None.

---

## `src/robot_interface/kinematics_solver.py`

### Classes

#### `KinematicsSolver` (ABC)

Kinematics base class: holds `link_list` (link lengths); subclasses implement leg/body FK/IK.

| Method | Description |
|--------|-------------|
| `__init__(link_list)` | Stores `link_list`. |
| `_forward(positions)` | Joint positions → workspace (abstract). |
| `_inverse(points)` | Workspace → joint positions (abstract). |
| `forward_with_pose(pose, positions)` | FK with base `pose` (`x,y,z,R,P,Y`) and joint `positions` (abstract). |
| `inverse_with_pose(pose, points)` | IK with base `pose` and foot/feature `points` (abstract). |

### Functions

Static helpers on `KinematicsSolver` (used by subclasses and transforms):

| Function | Description |
|----------|-------------|
| `_get_pose_transformation_matrix(pose)` | Builds \(4 \times 4\) homogeneous transform from `pose` \([x,y,z,R,P,Y]\) (ZYX Euler: `Rz @ Ry @ Rx` and translation). |
| `_invert_pose_transformation_matrix(pose)` | Inverse of the world–body transform from `pose`. |
| `_get_transformation_matrix(param)` | Single Denavit–Hartenberg row `param = [a, al, d, th]` → \(4 \times 4\) link transform. |
| `_forward_kinematics(dh_params)` | Product of DH matrices; returns the \(3 \times 1\) position (last column, first three rows). |

---

## `src/robot_interface/planner.py`

### Classes

#### `TrajectoryPlanner` (ABC)

Stores optional initial/goal states and evaluates a trajectory parameter `s`.

| Property | Description |
|----------|-------------|
| `initial_state` | Copy of `_init_state`, or `None` if unset. |

| Method | Description |
|--------|-------------|
| `__init__()` | Sets `_init_state` / `_goal_state` to `None`. |
| `set_initial_state(initial_state)` | Copies `initial_state` into `_init_state`. |
| `update_goal_state(goal_state)` | Abstract; default body in base assigns `_goal_state = goal_state.copy()` (subclasses override via `@abstractmethod`). |
| `eval(s)` | Evaluate trajectory at normalized parameter `s` (scalar or sequence per type hint); returns `np.ndarray` (abstract). |

### Functions

| Function | Description |
|----------|-------------|
| `_quintic_time_scaling(s)` | Clips `s` to \([0,1]\), applies quintic blend \(10s^3 - 15s^4 + 6s^5\) (smooth start/stop). |
| `_parabolic_time_scaling(s)` | Clips `s` to \([0,1]\), returns \(4s(1-s)\) (parabolic bump, zero at endpoints). |

---

## `src/robot_interface/__init__.py`

### Classes / functions

Re-exports only **`Robot`** and **`RobotConfig`**. `Scheduler`, `KinematicsSolver`, and `TrajectoryPlanner` are imported from their modules in dependent packages (`scheduler`, `kinematics`, `planner`).
