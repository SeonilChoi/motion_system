# common_robot_interface

Shared Python types for high-level robot logic (`ament_python`). Public API is re-exported from `src/common_robot_interface/__init__.py`: `State`, `StateFrame`, `Action`, `ActionFrame`, `JointStatus`, `RobotStatus`.

---

## `src/common_robot_interface/state.py`

### Enums

#### `State`

| Value | Meaning |
|-------|---------|
| `HOMMING` | Homing in progress. |
| `OPERATING` | Non-walk operation. |
| `WALKING` | Walk / gait active. |
| `STOPPED` | Idle / stopped. |

### Dataclasses

#### `StateFrame`

`@dataclass(frozen=True, slots=True)`

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `state` | `State` | (required) | Current high-level state. |
| `progress` | `float` | `0.0` | Sub-phase progress in \([0, 1]\) (scheduler-defined). |

---

## `src/common_robot_interface/action.py`

### Enums

#### `Action`

| Value | Meaning |
|-------|---------|
| `HOME` | Home / homing intent. |
| `MOVE` | Move / operate intent. |
| `WALK` | Gait / walk intent. |
| `STOP` | Stop intent. |

### Dataclasses

#### `ActionFrame`

`@dataclass(frozen=True, slots=True)`

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `action` | `Action` | (required) | Which action to run. |
| `duration` | `float` | `0.0` | Action horizon (seconds); gait / planner use. |
| `goal` | `Optional[np.ndarray]` | `None` | Goal vector (e.g. 3D: direction + yaw rate); semantics depend on scheduler / robot. |

---

## `src/common_robot_interface/joint.py`

### Dataclasses

#### `JointStatus`

`@dataclass(slots=True)`

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `motor_id` | `Optional[np.ndarray]` | `None` | Global or local motor / controller indices. |
| `interface_id` | `Optional[np.ndarray]` | `None` | Drive interface indices (maps to `MotorStatus.target_interface_id`). |
| `position` | `Optional[np.ndarray]` | `None` | Joint positions (rad or drive units after conversion). |
| `velocity` | `Optional[np.ndarray]` | `None` | Joint velocities. |
| `torque` | `Optional[np.ndarray]` | `None` | Joint torques. |

---

## `src/common_robot_interface/robot.py`

### Dataclasses

#### `RobotStatus`

`@dataclass(slots=True)`

| Field | Type | Meaning |
|-------|------|---------|
| `robot_id` | `int` | Robot index in a multi-robot setup. |
| `pose` | `np.ndarray` | Base / body pose (6-DOF layout used by the robot implementation). |
| `point` | `np.ndarray` | Foot / feature points (e.g. shape `(6, 3)`). |
| `twist` | `np.ndarray` | Twist (6-DOF). |
| `wrench` | `np.ndarray` | Wrench (6-DOF). |

---

Used by `motor_manager::MotorManager::write` / `read` and bridged to `motion_system_msgs/msg/MotorStatus` in `motor_manager_node`.
