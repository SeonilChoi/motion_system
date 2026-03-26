# robot_interface

Python package: `robot_interface`. Source: `src/robot_interface/`.

## `robot.py`

### `class Robot(ABC)`

Base abstraction for each robot instance.

#### Constructor

`__init__(self, robot_id: int, dt: float = 0.01, stride_length: float = 0.0, controller_indexes: Optional[list[int]] = None)`

#### Properties

| Property | Type |
|----------|------|
| `robot_id` | `int` |
| `stride_length` | `float` |
| `controller_indexes` | `Optional[list[int]]` |

#### Abstract methods

| Method | Signature |
|--------|-----------|
| `get_robot_state` | `() -> Optional[RobotState]` |
| `set_joint_state` | `(joint_states: JointState) -> None` |
| `get_state` | `() -> StateFrame` |
| `set_action` | `(frame: ActionFrame) -> None` |

## `scheduler.py`

### `class Scheduler(ABC)`

Base scheduler with current high-level state.

- `_current_state` initial value: `StateFrame(state=State.STOPPED, progress=0.0)`
- `reset()` restores `_t = 0.0` and stopped state.

#### Abstract method

| Method | Signature |
|--------|-----------|
| `tick` | `(frame: ActionFrame) -> bool` |
