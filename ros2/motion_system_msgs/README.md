# motion_system_msgs

ROS 2 interface package. `motion_system_pkg` uses these messages on `motor_command` and `motor_state`.

## `msg/MotorStatus.msg`

Single message carrying **arrays in parallel**: index `i` is one controller / axis. Each `target_interface_id[i]` is a `std_msgs/Int8MultiArray` whose `data` holds interface indices for that controller (length should match `number_of_target_interfaces[i]`, capped in the C++ bridge by `motor_interface::MAX_INTERFACE_SIZE`).

| Field | Type | Meaning |
|-------|------|---------|
| `number_of_target_interfaces` | `uint8[]` | Per-controller count of valid interface IDs. |
| `target_interface_id` | `std_msgs/Int8MultiArray[]` | Per-controller list of interface indices. |
| `controller_index` | `uint8[]` | Controller index (aligned with `motor_frame_t.controller_index`). |
| `controlword` | `uint16[]` | CiA402 controlword (command). |
| `statusword` | `uint16[]` | CiA402 statusword (feedback). |
| `errorcode` | `uint16[]` | Drive error code. |
| `position` | `float64[]` | Position (physical units). |
| `velocity` | `float64[]` | Velocity (physical units). |
| `torque` | `float64[]` | Torque (physical units). |

**Consumers**

- `motor_manager_node` (C++): `motor_command_callback` fills `motor_frame_t[]` from these arrays; the 1 ms timer publishes fresh feedback with the parallel arrays sized to `number_of_controllers()`.
- `robot_manager_node` (Python): reads `motor_state` into `JointStatus`, and builds `MotorStatus` for `motor_command` from `JointStatus` (including `Int8MultiArray` interface IDs).

**Build**

Depend on `motion_system_msgs` in `package.xml` and `CMakeLists.txt` (`find_package`, `ament_target_dependencies`, or `rosidl` usage) in any package that includes generated headers or uses the Python modules.
