# motion_system_msgs

ROS 2 message definitions for motor topics (`ament_cmake` + `rosidl`).

## `MotorStatus` (`msg/MotorStatus.msg`)

All array fields are **parallel by controller index** `i` (one logical axis / drive per `i`).

| Field | Type | Meaning |
|-------|------|---------|
| `number_of_target_interfaces[i]` | `uint8` | Number of valid entries in `target_interface_id[i].data`. |
| `target_interface_id[i]` | `std_msgs/Int8MultiArray` | List of interface indices for controller `i` (copied into `motor_frame_t.target_interface_id`, length capped by `motor_interface::MAX_INTERFACE_SIZE`). |
| `controller_index[i]` | `uint8` | Controller index; matches `motor_frame_t.controller_index`. |
| `controlword[i]` | `uint16` | Command controlword. |
| `statusword[i]` | `uint16` | Feedback statusword. |
| `errorcode[i]` | `uint16` | Drive error code. |
| `position[i]` | `float64` | Position (physical units). |
| `velocity[i]` | `float64` | Velocity (physical units). |
| `torque[i]` | `float64` | Torque (physical units). |

**Topics:** `motor_command` (subscribed by `motor_manager_node`), `motor_state` (published by `motor_manager_node`; `robot_manager_node` subscribes and may publish commands).
