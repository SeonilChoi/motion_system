# common_motor_interface

C++ header: `include/common_motor_interface/motor_frame.hpp`  
Types live in the `motor_interface` namespace (see the header).

## `MAX_INTERFACE_SIZE`

| Name | Type | Value | Meaning |
|------|------|-------|---------|
| `MAX_INTERFACE_SIZE` | `uint8_t` (`inline constexpr`) | `16` | Maximum length of `target_interface_id` and related per-frame interface arrays. |

## `struct motor_frame_t`

One logical motor/controller command or status snapshot used by `motor_manager::MotorManager::write` / `read`.

| Field | Type | Meaning |
|-------|------|---------|
| `number_of_target_interfaces` | `uint8_t` | Valid entries in `target_interface_id`. |
| `target_interface_id` | `uint8_t[MAX_INTERFACE_SIZE]` | Interface indices (driver / PDO mapping). |
| `controller_index` | `uint8_t` | Index in the manager’s controller array. |
| `controlword` | `uint16_t` | CiA402-style controlword (command). |
| `statusword` | `uint16_t` | CiA402-style statusword (feedback). |
| `errorcode` | `uint16_t` | Drive error code. |
| `position` | `double` | Position in physical units. |
| `velocity` | `double` | Velocity in physical units. |
| `torque` | `double` | Torque in physical units. |

**Usage**

- Fill the struct (or arrays of structs) and pass them to `MotorManager::write` / `read`.
- The ROS node `motor_manager_node` maps between `motor_frame_t` and `motion_system_msgs/msg/MotorStatus` on `motor_command` / `motor_state`.
