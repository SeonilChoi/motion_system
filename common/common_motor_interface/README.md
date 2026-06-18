# common_motor_interface

Shared C++ motor frame type package for ROS 2 / `ament_cmake`.

This package is header-only and exports `include/common_motor_interface/motor_frame.hpp`. The types live in namespace `motor_interface` and are used by the standalone `motor_manager` library and the ROS 2 bridge packages.

## Build

From the root of a colcon workspace:

```bash
colcon build --packages-select common_motor_interface
source install/setup.bash
```

## API

### `MAX_INTERFACE_SIZE`

| Name | Type | Value | Meaning |
| --- | --- | --- | --- |
| `MAX_INTERFACE_SIZE` | `uint8_t` | `16` | Maximum number of target interface IDs carried by one `motor_frame_t`. |

### `motor_frame_t`

`motor_frame_t` is the command/status payload exchanged between higher-level control code and each motor controller.

| Field | Type | Default | Meaning |
| --- | --- | --- | --- |
| `number_of_target_interfaces` | `uint8_t` | `0` | Number of valid entries in `target_interface_id`. |
| `target_interface_id` | `uint8_t[MAX_INTERFACE_SIZE]` | zeros | Interface IDs to write, such as controlword, target position, target velocity, or target torque. |
| `controller_index` | `uint8_t` | `0` | Index of the controller inside `MotorManager`. |
| `controlword` | `uint16_t` | `0` | CiA402-style command word. |
| `statusword` | `uint16_t` | `0` | CiA402-style status word. |
| `errorcode` | `uint16_t` | `0` | Drive error code. |
| `position` | `double` | `0` | Position in physical units after driver scaling. |
| `velocity` | `double` | `0` | Velocity in physical units after driver scaling. |
| `torque` | `double` | `0` | Torque in physical units after driver scaling. |

## Usage

```cpp
#include "common_motor_interface/motor_frame.hpp"
#include "motor_interface/motor_driver.hpp"

motor_interface::motor_frame_t frame{};
frame.controller_index = 0;
frame.number_of_target_interfaces = 2;
frame.target_interface_id[0] = motor_interface::ID_CONTROLWORD;
frame.target_interface_id[1] = motor_interface::ID_TARGET_POSITION;
frame.controlword = 0x000F;
frame.position = 1.0;
```

`motor_frame_t` itself only requires `common_motor_interface/motor_frame.hpp`. The semantic interface ID constants in this example are declared by `motor_interface/motor_driver.hpp` in the `motor_manager` package.

# common_motor_interface

ROS2의 `motor_status_msg`의 데이터를 `motor_manager`에게 공유할 때 사용하는 인터페이스.

한 개의 모터의 상태 혹은 명령을 담는다.

## Build

```bash
colcon build --packages-select common_motor_interface
```

## API

### `MAX_INTERFACE_SIZE`

| Name | Type | Meaning | Default
| --- | --- | --- | --- |
| `MAX_INTERFACE_SIZE` | `uint8_t` | 현재 제어 입력에서 사용할 인터페이스 수의 최대값 | `16` |

### `motor_frame_t`

`motor_frame_t`는 모터의 상태나 명령을 담을 수 있는 페이로드이다.
| Name | Type | Meaning | Default
| --- | --- | --- | --- |
| `number_of_target_interfaces` | `uint8_t` | 현재 제어 입력에서 사용할 인터페이스 수 | `0` |
| `target_ingerface_id` | `uint8_t[]` | 현재 제어 입력에서 사용할 인터페이스 ID | 'zeros' |
| `controller_index` | `uint8_t` | `motor_controller`의 고유한 인덱스 | `0` |
| `controlword` | `uint16_t` | CiA-402 control word | `0` |
| `statusword` | `uint16_t` | CiA-402 status word | `0` |
| `errorcode` | `uint16_t` | 드라이버의 에러 코드 | `0` |
| `position` | `double` | 위치 값 (e.g degree, mm) | `0` |
| `velocity` | `double` | 속도 값 (e.g degree/s, mm/s) | `0` |
| `effort` | `double` | 힘 값 (e.g Nm, N) | `0` |

## Usage

```c++
#include "common_motor_interface/motor_frame.hpp"
#include "motor_interface/motor_driver.hpp"

motor_interface::motor_frame_t frame{};
frame.controller_index = 0;
frame.number_of_target_interfaces = 2;
frame.target_interface_id[0] = motor_interface::ID_CONTROLWORD;
frame.target_interface_id[1] = motor_interface::ID_TARGET_POSITION;
frame.controlword = 0x000F;
frame.position = 1.0;
```
