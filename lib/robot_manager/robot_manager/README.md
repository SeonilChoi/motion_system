# robot_manager (package)

Python package: `robot_manager`. Implementation: `src/robot_manager/robot_manager.py`.

## `class RobotManager`

Loads a robot YAML file and constructs one `Robot` instance per `robots` list entry (`silver_lain`, `little_reader`, …).

### Properties

| Property | Type |
|----------|------|
| `dt` | `float` |
| `number_of_robots` | `int` |
| `number_of_motors` | `int` (sum of `controller_indexes` lengths) |

### Public methods

| Method | Signature | Role |
|--------|-----------|------|
| `stride_length` | `(robot_id: int) -> float` | Per-robot stride. |
| `duration` | `(robot_id: int) -> float` | Per-robot default duration. |
| `get_state_frame` | `(robot_id: int) -> StateFrame` | Scheduler state. |
| `set_action_frame` | `(action_frame_list: list[ActionFrame]) -> JointStatus` | Dispatches each frame, merges joint commands into a global `JointStatus` (starting from home pose baseline). |
| `get_robot_status` | `(robot_id: int) -> RobotStatus` | Kinematics / pose snapshot. |
| `update_joint_status` | `(joint_status: JointStatus) -> None` | Splits global arrays by each robot’s `controller_indexes` and calls `robot.update_joint_status`. |
| `reset` | `() -> None` | Calls `reset()` on every robot. |

If the config path is missing or invalid, the manager keeps defaults (`dt = 0.01`, zero robots/motors).

### YAML consumed by `_loadConfigurations`

Top-level keys:

| Key | Role |
|-----|------|
| `dt` | Control period (float). |
| `robots` | List of per-robot dicts (see below). |

Each **robot row**:

| Key | Role |
|-----|------|
| `id` | Robot id (int, default `0`). |
| `robot` | Implementation key: `silver_lain`, `little_reader`, … |
| `stride_length` | Float. |
| `clearance` | Float (default `0.05`). |
| `duration` | Float (default `5.0`). |
| `home_joint_positions` | List / array of joint positions. |
| `home_pose` | Length-6 pose vector. |
| `controller_indexes` | int32 array — indices into the global motor vector / `motor_state` ordering. |
| `interface_ids` | int32 array — same length as `controller_indexes`; passed through to command messages. |

Example (abbreviated):

```yaml
dt: 0.01
robots:
  - id: 0
    robot: silver_lain
    duration: 5.0
    stride_length: 0.2
    clearance: 0.15
    home_joint_positions: [0.0, ...]
    home_pose: [0.0, 0.0, 0.7788, 0.0, 0.0, 0.0]
    controller_indexes: [0, 1, 2, ...]
    interface_ids: [1, 1, 1, ...]
```

See `ros2/motion_system_pkg/config/silver_lain.yaml` for a full sample.
