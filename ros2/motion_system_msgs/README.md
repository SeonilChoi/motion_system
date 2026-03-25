# motion_system_msgs

ROS 2 message package. Messages are used on topics `motor_command` and `motor_state` by `motion_system_pkg`.

## `msg/MotorFrame.msg`

| Field | Type | Meaning |
|-------|------|---------|
| `number_of_target_interfaces` | `uint8` | Count of valid `target_interface_id` elements. |
| `target_interface_id` | `uint8[]` | Interface indices (max length capped in C++ bridge to `motor_interface::MAX_INTERFACE_SIZE`). |
| `controller_index` | `uint8` | Controller index in the multi-frame array. |
| `controlword` | `uint16` | CiA402 controlword (command). |
| `statusword` | `uint16` | CiA402 statusword (feedback). |
| `errorcode` | `uint16` | Drive error code. |
| `position` | `float64` | Position (physical units). |
| `velocity` | `float64` | Velocity (physical units). |
| `torque` | `float64` | Torque (physical units). |

**Usage:** one message per axis/controller; populate `data` array in `MotorFrameMultiArray`.

## `msg/MotorFrameMultiArray.msg`

| Field | Type | Meaning |
|-------|------|---------|
| `data` | `MotorFrame[]` | Ordered list of frames; index matches `controller_index` after bridging in `motor_manager_node`. |

**Build:** add `motion_system_msgs` to `package.xml` / `CMakeLists.txt` `find_package` and message dependencies before packages that include generated headers.
