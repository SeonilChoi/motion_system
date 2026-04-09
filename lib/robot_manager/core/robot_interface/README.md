# robot_interface

Python package: `robot_interface`. Source: `src/robot_interface/`.

## `robot.py`

### `@dataclass class RobotConfig`

Construction parameters shared by all `Robot` implementations (typically from YAML via `RobotManager`).

| Field | Type | Default | Role |
|-------|------|---------|------|
| `robot_id` | `int` | `0` | Logical robot id. |
| `dt` | `float` | `0.01` | Control period (s). |
| `stride_length` | `float` | `0.0` | Walk scaling (m-class units; robot-specific). |
| `clearance` | `float` | `0.05` | Planner clearance. |
| `duration` | `float` | `5.0` | Default action duration. |
| `controller_indexes` | `Any` | empty `int32` array | Global motor indices for this robot. |
| `interface_ids` | `Any` | empty `int32` array | Per-motor interface ids (ROS / drive mapping). |
| `home_joint_positions` | `Any` | empty `float64` array | Nominal joint pose at home. |
| `home_pose` | `Any` | 6-vector | Base / body home pose. |

### `class Robot(ABC)`

#### Constructor

`__init__(self, config: RobotConfig)`

Exposes `robot_id`, `stride_length`, `clearance`, `duration`, `controller_indexes`, `interface_ids`, `number_of_motors`, `home_joint_positions`, and `home_pose` as properties.

#### Abstract methods

| Method | Signature |
|--------|-----------|
| `get_state_frame` | `() -> StateFrame` |
| `set_action_frame` | `(frame: ActionFrame) -> np.ndarray` |
| `get_robot_status` | `() -> RobotStatus` |
| `update_joint_status` | `(joint_status: JointStatus) -> None` |

Implementations used with `RobotManager` should return a 2D `np.ndarray` whose columns correspond to position, velocity, and torque rows merged in `RobotManager.set_action_frame` (see `robot_manager/robot_manager.py`).

---

## `scheduler.py`

### `class Scheduler(ABC)`

Base scheduler holding high-level `StateFrame`.

- Initial `_current_state`: `StateFrame(state=State.STOPPED, progress=0.0)`.
- `reset()` is abstract; implementations restore timing and state.

#### Property

| Name | Type |
|------|------|
| `current_state` | `StateFrame` |

#### Abstract methods

| Method | Signature |
|--------|-----------|
| `tick` | `(frame: ActionFrame) -> bool` |
| `reset` | `() -> None` |
