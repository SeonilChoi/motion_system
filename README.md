# motion_system workspace

## Repository layout

Top-level directories under this `src` tree (colcon packages live here):

```
src/
├── README.md
├── common/
│   ├── common_robot_interface/     # ament_python: Action, ActionFrame, State, StateFrame (+ joint.py)
│   └── common_motor_interface/     # ament_cmake: shared C++ motor_frame_t header
├── lib/
│   ├── robot_manager/              # ament_cmake wrapper + nested Python packages
│   │   ├── core/robot_interface/   # abstract Robot + Scheduler
│   │   ├── packages/robot_control/ # FSM/gait schedulers, concrete robots
│   │   └── robot_manager/          # RobotManager, YAML + joystick mapping
│   └── motor_manager/              # ament_cmake: EtherCAT + MINAS + MotorManager
│       ├── core/motor_interface/   # MotorMaster / MotorController / MotorDriver
│       ├── communications/ethercat/ # IgH EtherCAT master + controller
│       ├── hardware/minas/         # MinasDriver
│       └── motor_manager/          # MotorManager orchestrator
└── ros2/
    ├── motion_system_msgs/         # MotorFrame.msg, MotorFrameMultiArray.msg
    └── motion_system_pkg/          # motor_manager_node (C++), robot_manager_node (Python)
        ├── config/                 # robot + EtherCAT YAML defaults for launch
        ├── param/                  # drive parameter YAML (e.g. MINAS)
        ├── include/, src/           # C++ node
        ├── scripts/                 # Python node entry scripts
        └── launch/                  # joy, motor_manager_node, robot_manager_node
```

| Area | Role |
|------|------|
| `common/*` | Language-specific shared types only; no ROS nodes. |
| `lib/robot_manager/*` | Python robot behavior: config, schedulers, robot models, `RobotManager`. |
| `lib/motor_manager/*` | C++ real-time motor stack: YAML-loaded masters/controllers/drivers. |
| `ros2/motion_system_msgs` | Message schema for `motor_command` / `motor_state` topics. |
| `ros2/motion_system_pkg` | Bridges ROS topics to `MotorManager` and runs teleop-side logic. |

Generated or local artifacts (`build/`, `install/`, `log/` under `lab_ws`, and `__pycache__`) are not shown; they are produced by the build, not source.

---

## API reference index

Package-level API references (classes, functions, constants, message fields):

| Path | Reference |
|------|-----------|
| [common/common_robot_interface/README.md](common/common_robot_interface/README.md) | Python `Action`, `ActionFrame`, `JointState`, `State`, `StateFrame` |
| [common/common_motor_interface/README.md](common/common_motor_interface/README.md) | C++ `motor_frame_t`, `MAX_INTERFACE_SIZE` |
| [lib/robot_manager/README.md](lib/robot_manager/README.md) | Robot manager stack entry |
| [lib/motor_manager/README.md](lib/motor_manager/README.md) | Motor runtime stack entry |
| [ros2/motion_system_msgs/README.md](ros2/motion_system_msgs/README.md) | ROS message field reference |
| [ros2/motion_system_pkg/README.md](ros2/motion_system_pkg/README.md) | ROS nodes, launch arguments, scripts |

Build (from `lab_ws`):

```bash
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```
