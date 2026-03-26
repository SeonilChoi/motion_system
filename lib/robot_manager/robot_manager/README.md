# robot_manager (package)

Python package: `robot_manager`. Source: `src/robot_manager/robot_manager.py`.

## `class RobotManager`

Loads robot YAML and creates one robot instance per `robots` row.

### Properties

| Property | Type |
|----------|------|
| `dt` | `float` |
| `number_of_robots` | `int` |
| `number_of_motors` | `int` |

### Public methods

| Method | Signature | Notes |
|--------|-----------|-------|
| `stride_length` | `(robot_id: int) -> float` | For walk duration scaling. |
| `get_state` | `(robot_id: int) -> StateFrame` | High-level scheduler state. |
| `set_action` | `(action_frame_list: list[ActionFrame]) -> None` | Per-robot action dispatch. |
| `get_robot_states` | `(robot_id: int) -> RobotState` | Kinematics-derived robot state. |
| `set_joint_states` | `(joint_states: JointState) -> None` | Splits global arrays by each robot's `controller_indexes`. |

### YAML keys used

```yaml
dt: 0.01
number_of_robots: 2
robots:
  - id: 0
    robot: silver_lain
    stride_length: 0.1
    controller_indexes: [0, ..., 17]
  - id: 1
    robot: silver_lain
    stride_length: 0.15
    controller_indexes: [18, ..., 35]
```

`controller_indexes` is used to map `/motor_state` frames into per-robot `JointState`.
