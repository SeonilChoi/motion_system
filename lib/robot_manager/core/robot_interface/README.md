# robot_interface

Python **robot abstraction** (`ament_python`). Source: `src/robot_interface/`.

## `RobotConfig` (dataclass)

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `robot_id` | `int` | `0` | Logical robot id. |
| `dt` | `float` | `0.01` | Control period (s). |
| `stride_length` | `float` | `0.0` | Nominal stride (m-class); used for walk timing / scaling. |
| `clearance` | `float` | `0.05` | Foot / body clearance for planners. |
| `duration` | `float` | `5.0` | Default action duration (s). |
| `controller_indexes` | `Any` | empty `int32` array | Indices into the **global** motor vector (matches `motor_state` ordering). |
| `interface_ids` | `Any` | empty `int32` array | Per-motor interface ids for command messages (same length as `controller_indexes`). |
| `home_joint_positions` | `Any` | empty `float64` array | Nominal joint vector at home. |
| `home_pose` | `Any` | 6-vector | Nominal base / body pose at home. |

## `Robot` (abstract)

Constructed with `RobotConfig`. Exposes properties: `robot_id`, `stride_length`, `clearance`, `duration`, `controller_indexes`, `interface_ids`, `number_of_motors`, `home_joint_positions`, `home_pose`.

| Method | Return | Meaning |
|--------|--------|---------|
| `get_state_frame()` | `StateFrame` | Scheduler / FSM state snapshot. |
| `set_action_frame(frame)` | `np.ndarray` | Apply `ActionFrame`; return per-motor command array shape expected by `RobotManager` (typically `(n_motors, 3)` for p/v/t columns). |
| `get_robot_status()` | `RobotStatus` | Kinematics / pose snapshot. |
| `update_joint_status(joint_status)` | `None` | Inject feedback `JointStatus` slice for this robot. |

## `Scheduler` (abstract)

| Field / property | Type | Meaning |
|------------------|------|---------|
| `current_state` | `StateFrame` | Latest scheduler state (initial: `STOPPED`, `progress=0`). |

| Method | Meaning |
|--------|---------|
| `tick(frame)` | Advance one step from `ActionFrame`; returns scheduler-specific bool. |
| `reset()` | Reset internal time and state. |
