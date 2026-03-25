# common_motor_interface

C++ header: `include/common_motor_interface/motor_frame.hpp`  
Namespace in this repository: `motor_interface` (see header).

## `MAX_INTERFACE_SIZE`

| Name | Type | Value | Meaning |
|------|------|-------|---------|
| `MAX_INTERFACE_SIZE` | `uint8_t` (inline constexpr) | `16` | Maximum number of entries in `target_interface_id` and related interface arrays. |

## `struct motor_frame_t`

Aggregates one logical motor/controller command or status snapshot.

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `number_of_target_interfaces` | `uint8_t` | `0` | Count of valid entries in `target_interface_id`. |
| `target_interface_id` | `uint8_t[MAX_INTERFACE_SIZE]` | zero | Interface IDs selecting PDO/mapping targets (driver-specific). |
| `controller_index` | `uint8_t` | `{}` | Index of the controller/slave in the multi-axis layout. |
| `controlword` | `uint16_t` | `{}` | CiA402-style controlword (command side). |
| `statusword` | `uint16_t` | `{}` | CiA402-style statusword (feedback side). |
| `errorcode` | `uint16_t` | `{}` | Drive error code. |
| `position` | `double` | `{}` | Position in physical units after driver conversion. |
| `velocity` | `double` | `{}` | Velocity in physical units. |
| `torque` | `double` | `{}` | Torque in physical units. |

**Usage**

- Fill arrays and scalars, then pass pointers to `motor_manager::MotorManager::write` / `read`.
- ROS bridging copies these fields to/from `motion_system_msgs/MotorFrame` (see `motion_system_msgs` README).
