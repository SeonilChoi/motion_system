# motion_system_ros2

## English Version

`motion_system_ros2` is a ROS 2 package group for motor command/status exchange and operator control.

It provides shared motion control messages, a ROS 2 bridge for `motor_manager`, an RQt control UI, and a MIDI control launch flow for X-Touch.

### Packages

| Name | Purpose |
| --- | --- |
| `motion_control_msgs` | Defines `MotorStatus.msg` for motor command and status data. |
| `motion_control_bridge` | Runs `motor_manager_node`, which connects ROS 2 topics with the `motor_manager` runtime. |
| `motion_control_rqt` | Provides an RQt UI for monitoring motor status and sending motor commands. |
| `motion_control_midi` | Runs the MIDI control flow for X-Touch input and motor commands. |

### Requirements

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

`motion_control_midi` launches `xtouch_midi`, so `src/ros2/xtouch_midi_ros2` must also exist in the workspace when using MIDI control.

### Build

```bash
cd ~/colcon_ws

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run

Run only the motor manager bridge node.

```bash
ros2 launch motion_control_bridge motor_manager_node.launch.py
```

Run the RQt UI with the motor manager bridge.

```bash
ros2 launch motion_control_rqt display_motor_manager_node.launch.py
```

Run the MIDI control flow.

```bash
ros2 launch motion_control_midi motion_control_midi_node.launch.py
```

### Topic

| Name | Type | Payload |
| --- | --- | --- |
| `/motion_control/motor_command` | `motion_control_msgs/msg/MotorStatus` | Motor command consumed by `motion_control_bridge` |
| `/motion_control/motor_status` | `motion_control_msgs/msg/MotorStatus` | Motor status published by `motion_control_bridge` |
| `/motion_control/request_stop` | `std_msgs/msg/Empty` | Stop request consumed by `motion_control_bridge` |
| `/xtouch/midi` | `midi_msgs/msg/Midi` | X-Touch MIDI state published by `xtouch_midi` and consumed by `motion_control_midi` |

### `MotorStatus.msg`

| Name | Type |
| --- | --- |
| `number_of_target_interfaces` | `uint8[]` |
| `target_interface_id` | `std_msgs/Int8MultiArray[]` |
| `controller_index` | `uint8[]` |
| `controlword` | `uint16[]` |
| `statusword` | `uint16[]` |
| `errorcode` | `uint16[]` |
| `position` | `float64[]` |
| `velocity` | `float64[]` |
| `effort` | `float64[]` |

## Korean Version

`motion_system_ros2`는 모터 명령/상태 교환과 operator control을 위한 ROS 2 패키지 그룹이다.

shared motion control message, `motor_manager`를 위한 ROS 2 bridge, RQt control UI, X-Touch를 위한 MIDI control launch flow를 제공한다.

### Packages

| Name | Purpose |
| --- | --- |
| `motion_control_msgs` | 모터 명령과 상태 데이터를 위한 `MotorStatus.msg`를 정의한다. |
| `motion_control_bridge` | ROS 2 topic과 `motor_manager` runtime을 연결하는 `motor_manager_node`를 실행한다. |
| `motion_control_rqt` | 모터 상태 확인과 모터 명령 전송을 위한 RQt UI를 제공한다. |
| `motion_control_midi` | X-Touch 입력과 모터 명령을 위한 MIDI control flow를 실행한다. |

### Requirements

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

`motion_control_midi`는 `xtouch_midi`를 함께 실행하므로, MIDI 제어를 사용할 때는 `src/ros2/xtouch_midi_ros2`도 workspace에 있어야 한다.

### Build

```bash
cd ~/colcon_ws

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run

motor manager bridge node만 실행한다.

```bash
ros2 launch motion_control_bridge motor_manager_node.launch.py
```

motor manager bridge와 함께 RQt UI를 실행한다.

```bash
ros2 launch motion_control_rqt display_motor_manager_node.launch.py
```

MIDI control flow를 실행한다.

```bash
ros2 launch motion_control_midi motion_control_midi_node.launch.py
```

### Topic

| Name | Type | Payload |
| --- | --- | --- |
| `/motion_control/motor_command` | `motion_control_msgs/msg/MotorStatus` | `motion_control_bridge`가 사용하는 motor command |
| `/motion_control/motor_status` | `motion_control_msgs/msg/MotorStatus` | `motion_control_bridge`가 publish하는 motor status |
| `/motion_control/request_stop` | `std_msgs/msg/Empty` | `motion_control_bridge`가 사용하는 stop request |
| `/xtouch/midi` | `midi_msgs/msg/Midi` | `xtouch_midi`가 publish하고 `motion_control_midi`가 사용하는 X-Touch MIDI state |

### `MotorStatus.msg`

| Name | Type |
| --- | --- |
| `number_of_target_interfaces` | `uint8[]` |
| `target_interface_id` | `std_msgs/Int8MultiArray[]` |
| `controller_index` | `uint8[]` |
| `controlword` | `uint16[]` |
| `statusword` | `uint16[]` |
| `errorcode` | `uint16[]` |
| `position` | `float64[]` |
| `velocity` | `float64[]` |
| `effort` | `float64[]` |
