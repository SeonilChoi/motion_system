# motion_system_msgs

## `msg/MotorStatus.msg`

| Field | Type | Description |
|-------|------|-------------|
| `number_of_target_interfaces` | `uint8[]` | Per controller `i`: count of valid entries in `target_interface_id[i].data`. |
| `target_interface_id` | `std_msgs/Int8MultiArray[]` | Per controller `i`: interface indices written to `motor_frame_t.target_interface_id` (length bounded by `MAX_INTERFACE_SIZE` in `common_motor_interface`). |
| `controller_index` | `uint8[]` | Per controller `i`: dense controller index; matches `motor_frame_t.controller_index`. |
| `controlword` | `uint16[]` | Per controller `i`: command controlword (CiA402-style). |
| `statusword` | `uint16[]` | Per controller `i`: feedback statusword. |
| `errorcode` | `uint16[]` | Per controller `i`: drive error code. |
| `position` | `float64[]` | Per controller `i`: position (physical units). |
| `velocity` | `float64[]` | Per controller `i`: velocity (physical units). |
| `torque` | `float64[]` | Per controller `i`: torque (physical units). |

All array fields are **parallel by index** `i` (one drive / axis per index).
