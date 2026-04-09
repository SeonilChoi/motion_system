# common_robot_interface

Shared Python types for high-level robot logic (`ament_python`). Exports: `Action`, `ActionFrame`, `JointStatus`, `RobotStatus`, `State`, `StateFrame`.

## `Action` (enum)

| Value | Meaning |
|-------|---------|
| `HOME` | Home / homing intent. |
| `MOVE` | Move / operate intent. |
| `WALK` | Gait / walk intent. |
| `STOP` | Stop intent. |

## `ActionFrame` (dataclass)

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `action` | `Action` | (required) | Which action to run. |
| `duration` | `float` | `0.0` | Action horizon (seconds); gait / planner use. |
| `goal` | `Optional[np.ndarray]` | `None` | Goal vector (e.g. 3D: direction + yaw rate); semantics depend on scheduler / robot. |

## `State` (enum)

| Value | Meaning |
|-------|---------|
| `HOMMING` | Homing in progress. |
| `OPERATING` | Non-walk operation. |
| `WALKING` | Walk / gait active. |
| `STOPPED` | Idle / stopped. |

## `StateFrame` (dataclass)

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `state` | `State` | (required) | Current high-level state. |
| `progress` | `float` | `0.0` | Sub-phase progress in \([0, 1]\) (scheduler-defined). |

## `JointStatus` (dataclass)

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `motor_id` | `Optional[np.ndarray]` | `None` | Global or local motor / controller indices. |
| `interface_id` | `Optional[np.ndarray]` | `None` | Drive interface indices (maps to `MotorStatus.target_interface_id`). |
| `position` | `Optional[np.ndarray]` | `None` | Joint positions (rad or drive units after conversion). |
| `velocity` | `Optional[np.ndarray]` | `None` | Joint velocities. |
| `torque` | `Optional[np.ndarray]` | `None` | Joint torques. |

## `RobotStatus` (dataclass)

| Field | Type | Meaning |
|-------|------|---------|
| `robot_id` | `int` | Robot index in a multi-robot setup. |
| `pose` | `np.ndarray` | Base / body pose (6-DOF layout used by the robot implementation). |
| `point` | `np.ndarray` | Foot / feature points (e.g. shape `(6, 3)`). |
| `twist` | `np.ndarray` | Twist (6-DOF). |
| `wrench` | `np.ndarray` | Wrench (6-DOF). |
