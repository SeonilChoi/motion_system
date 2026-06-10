# common_motor_interface

Shared C++ motor **data** types (`ament_cmake`). Header: `include/common_motor_interface/motor_frame.hpp`. Namespace: `motor_interface`.

## Constants (namespace scope)

| Name | Type | Value | Meaning |
|------|------|-------|---------|
| `MAX_INTERFACE_SIZE` | `uint8_t` (`inline constexpr`) | `16` | Max length of `target_interface_id[]` per `motor_frame_t`. |

## Structs

### `motor_frame_t`

| Field | Type | Default | Meaning |
|-------|------|---------|---------|
| `number_of_target_interfaces` | `uint8_t` | `0` | How many entries in `target_interface_id` are valid. |
| `target_interface_id` | `uint8_t[MAX_INTERFACE_SIZE]` | zeros | Sub-interface / PDO slice indices for this controller. |
| `controller_index` | `uint8_t` | `0` | Index in `MotorManager`’s controller table. |
| `controlword` | `uint16_t` | `0` | CiA402-style controlword (command). |
| `statusword` | `uint16_t` | `0` | CiA402-style statusword (feedback). |
| `errorcode` | `uint16_t` | `0` | Drive error code. |
| `position` | `double` | `0` | Position (physical units after scaling). |
| `velocity` | `double` | `0` | Velocity (physical units). |
| `torque` | `double` | `0` | Torque (physical units). |

Used by `motor_manager::MotorManager::write` / `read` and bridged to `motion_system_msgs/msg/MotorStatus` in `motor_manager_node`.
