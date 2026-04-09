# Motion System

ROS 2 workspace for robot motion: a **real-time motor stack** (EtherCAT with MINAS / ZeroErr drivers, and related plumbing) and a **Python robot / scheduler / kinematics** layer. The `motion_system_pkg` nodes connect joystick and high-level robot state to the low-level `MotorManager`.

Use this repository directory as the **colcon workspace root**; `colcon` discovers packages recursively via `package.xml` files under `common/`, `lib/`, and `ros2/`.

---

## Requirements

- **ROS 2** (point `setup.bash` at your distro, e.g. Humble or Jazzy)
- **Build**: `colcon`, CMake, C++17 toolchain
- **Libraries**: `yaml-cpp` (motor YAML loading in `motor_manager`)
- **Real EtherCAT**: IgH EtherCAT Master and matching kernel/driver setup (see `lib/motor_manager/communications/ethercat`)

Python stacks need `numpy`, `rclpy`, and other dependencies declared in the relevant `package.xml` / `setup.py` files.

---

## Build

```bash
source /opt/ros/<distro>/setup.bash
cd /path/to/motion_system
colcon build --symlink-install
source install/setup.bash
```

---

## Running

1. **Motor layer** — `motor_manager_node` loads `MotorManager` from YAML and exchanges `motion_system_msgs/msg/MotorStatus` on `motor_command` / `motor_state`.
2. **Robot / joystick layer** — `robot_manager_node` subscribes to `joy` and `motor_state`, runs `RobotManager`, and publishes `motor_command`.

```bash
# Terminal 1 — motor stack (default: ethercat_integrated.yaml)
ros2 launch motion_system_pkg motor_manager_node.launch.py

# Terminal 2 — joystick + robot manager (default: silver_lain.yaml)
ros2 launch motion_system_pkg robot_manager_node.launch.py
```

---

## Repository layout

```
motion_system/                 # colcon workspace root
├── common/
│   ├── common_robot_interface/   # ament_python: Action, ActionFrame, JointStatus, RobotStatus, …
│   └── common_motor_interface/   # ament_cmake: shared C++ headers (e.g. motor_frame_t)
├── lib/
│   ├── robot_manager/            # ament_cmake wrapper + nested Python packages
│   │   ├── core/robot_interface/
│   │   ├── kinematics/ planner/ scheduler/ robots/
│   │   └── robot_manager/        # RobotManager, robot YAML
│   └── motor_manager/            # ament_cmake: EtherCAT, MINAS, ZeroErr, MotorManager
│       ├── core/motor_interface/
│       ├── communications/ethercat/
│       ├── hardware/minas/ hardware/zeroerr/
│       └── motor_manager/
└── ros2/
    ├── motion_system_msgs/       # MotorStatus.msg
    ├── motion_system_pkg/        # motor_manager_node (C++), robot_manager_node (Python), launch/config/param
    └── utils_pkg/                # ament_python utilities
```

---

## Package roles


| Area                      | Role                                                                 |
| ------------------------- | -------------------------------------------------------------------- |
| `common/*`                | Shared types and headers only; no ROS nodes                          |
| `lib/robot_manager/*`     | Robot behavior, schedulers, kinematics, `RobotManager`               |
| `lib/motor_manager/*`     | Real-time motor stack (YAML-defined masters / controllers / drivers) |
| `ros2/motion_system_msgs` | Message definitions for motor topics                                 |
| `ros2/motion_system_pkg`  | ROS bridge to `MotorManager`, joystick integration, launch files     |
| `ros2/utils_pkg`          | Small test / utility nodes                                           |


---

## Further reading


| Path                                                                               | Contents                  |
| ---------------------------------------------------------------------------------- | ------------------------- |
| [common/common_robot_interface/README.md](common/common_robot_interface/README.md) | Python shared robot types |
| [common/common_motor_interface/README.md](common/common_motor_interface/README.md) | C++ `motor_frame_t`       |
| [lib/robot_manager/README.md](lib/robot_manager/README.md)                         | Robot manager stack index |
| [lib/motor_manager/README.md](lib/motor_manager/README.md)                         | Motor runtime stack index |
| [ros2/motion_system_msgs/README.md](ros2/motion_system_msgs/README.md)             | `MotorStatus` message     |
| [ros2/motion_system_pkg/README.md](ros2/motion_system_pkg/README.md)               | Nodes, launches, scripts  |


---

## License

Per-package licenses are given in each `package.xml` and any `LICENSE` files (for example MIT for `motion_system_msgs` / `motion_system_pkg`, Apache-2.0 for `utils_pkg`).