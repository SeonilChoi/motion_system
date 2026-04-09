# motion_system — system overview

ROS 2 workspace for legged / multi-axis motion: a **C++ motor layer** (EtherCAT, CiA402-style drives, YAML-configured `MotorManager`) and a **Python robot layer** (kinematics, planners, schedulers, `RobotManager`). **`motion_system_pkg`** provides `motor_manager_node` and `robot_manager_node`, which exchange **`motion_system_msgs/msg/MotorStatus`** on `motor_command` and `motor_state`.

This repository root is a **colcon workspace**; packages are discovered under `common/`, `lib/`, and `ros2/`.

---

## Requirements

- **ROS 2** (e.g. Humble or Jazzy): `setup.bash` for your distro
- **Build tools**: `colcon`, CMake, **C++17** compiler
- **Libraries**: `yaml-cpp` (motor configuration)
- **Python**: `numpy`, `rclpy`, and dependencies declared in each package’s `package.xml` / `setup.py`
- **Real EtherCAT** (optional): IgH EtherCAT Master and matching kernel / driver setup

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

Typical two-process setup:

```bash
ros2 launch motion_system_pkg motor_manager_node.launch.py
ros2 launch motion_system_pkg robot_manager_node.launch.py
```

Defaults use `config/ethercat_integrated.yaml` (motor) and `config/silver_lain.yaml` (robot), with `joy` included from `robot_manager_node.launch.py`.

---

## Repository layout

```
motion_system/
├── README.md
├── common/
│   ├── common_robot_interface/
│   │   ├── package.xml
│   │   ├── setup.py, setup.cfg
│   │   ├── resource/
│   │   └── src/common_robot_interface/
│   └── common_motor_interface/
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── include/common_motor_interface/
├── lib/
│   ├── robot_manager/
│   │   ├── README.md
│   │   ├── core/
│   │   │   └── robot_interface/
│   │   │       ├── package.xml
│   │   │       ├── setup.py, setup.cfg
│   │   │       ├── resource/
│   │   │       └── src/robot_interface/
│   │   ├── kinematics/
│   │   │   ├── package.xml
│   │   │   ├── setup.py, setup.cfg
│   │   │   ├── resource/
│   │   │   └── src/kinematics/
│   │   ├── planner/
│   │   │   ├── package.xml
│   │   │   ├── setup.py, setup.cfg
│   │   │   ├── resource/
│   │   │   └── src/planner/
│   │   ├── scheduler/
│   │   │   ├── package.xml
│   │   │   ├── setup.py, setup.cfg
│   │   │   ├── resource/
│   │   │   └── src/scheduler/
│   │   ├── robots/
│   │   │   ├── package.xml
│   │   │   ├── setup.py, setup.cfg
│   │   │   ├── resource/
│   │   │   └── src/robots/
│   │   └── robot_manager/
│   │       ├── package.xml
│   │       ├── setup.py, setup.cfg
│   │       ├── resource/
│   │       └── src/robot_manager/
│   └── motor_manager/
│       ├── README.md
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── core/
│       │   └── motor_interface/
│       │       ├── CMakeLists.txt
│       │       └── include/motor_interface/
│       ├── communications/
│       │   └── ethercat/
│       │       ├── CMakeLists.txt
│       │       └── include/ethercat/, src/
│       ├── hardware/
│       │   ├── minas/
│       │   │   ├── CMakeLists.txt
│       │   │   └── include/minas/, src/
│       │   └── zeroerr/
│       │       ├── CMakeLists.txt
│       │       └── include/zeroerr/, src/
│       └── motor_manager/
│           ├── CMakeLists.txt
│           └── include/motor_manager/, src/
└── ros2/
    ├── motion_system_msgs/
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   └── msg/
    ├── motion_system_pkg/
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── include/motion_system_pkg/
    │   ├── src/
    │   ├── scripts/
    │   ├── launch/
    │   ├── config/
    │   └── param/
    └── utils_pkg/
        ├── package.xml
        ├── setup.py, setup.cfg
        ├── resource/
        ├── utils_pkg/
        └── test/
```

Build artifacts (`build/`, `install/`, `log/`) and `__pycache__` are local only and not shown above.

---

## Package README links

| Package (`package.xml` name) | README |
|------------------------------|--------|
| `common_robot_interface` | [common/common_robot_interface/README.md](common/common_robot_interface/README.md) |
| `common_motor_interface` | [common/common_motor_interface/README.md](common/common_motor_interface/README.md) |
| `motor_manager` | [lib/motor_manager/README.md](lib/motor_manager/README.md) |
| `robot_interface` | [lib/robot_manager/core/robot_interface/README.md](lib/robot_manager/core/robot_interface/README.md) |
| `kinematics` | [lib/robot_manager/kinematics/README.md](lib/robot_manager/kinematics/README.md) |
| `planner` | [lib/robot_manager/planner/README.md](lib/robot_manager/planner/README.md) |
| `scheduler` | [lib/robot_manager/scheduler/README.md](lib/robot_manager/scheduler/README.md) |
| `robots` | [lib/robot_manager/robots/README.md](lib/robot_manager/robots/README.md) |
| `robot_manager` | [lib/robot_manager/robot_manager/README.md](lib/robot_manager/robot_manager/README.md) |
| `motion_system_msgs` | [ros2/motion_system_msgs/README.md](ros2/motion_system_msgs/README.md) |
| `motion_system_pkg` | [ros2/motion_system_pkg/README.md](ros2/motion_system_pkg/README.md) |
| `utils_pkg` | [ros2/utils_pkg/README.md](ros2/utils_pkg/README.md) |

**Python robot stack index** (not a separate colcon package): [lib/robot_manager/README.md](lib/robot_manager/README.md)

**Inside `motor_manager` (same colcon package):**

| Component | README |
|-----------|--------|
| `motor_interface` (core) | [lib/motor_manager/core/motor_interface/README.md](lib/motor_manager/core/motor_interface/README.md) |
| `motor_manager` (library) | [lib/motor_manager/motor_manager/README.md](lib/motor_manager/motor_manager/README.md) |
| EtherCAT | [lib/motor_manager/communications/ethercat/README.md](lib/motor_manager/communications/ethercat/README.md) |
| MINAS | [lib/motor_manager/hardware/minas/README.md](lib/motor_manager/hardware/minas/README.md) |
| ZeroErr | [lib/motor_manager/hardware/zeroerr/README.md](lib/motor_manager/hardware/zeroerr/README.md) |
