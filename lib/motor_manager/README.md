# motor_manager (library tree)

This tree contains the C++ motor runtime stack for communication, hardware driver handling, and real-time orchestration.

## Role

- Implements deterministic motor control loop execution for configured masters and slaves.
- Bridges YAML configuration into concrete communication and driver objects.
- Exposes a unified command/state buffer model through motor frame types.

## Submodules

- `core/motor_interface`: abstract interfaces and shared contracts
- `communications/ethercat`: IgH EtherCAT master/controller implementation
- `hardware/minas`: Minas drive-specific logic and conversion
- `motor_manager`: top-level orchestrator library

## Build And Dependency Notes

- Built as `ament_cmake` C++17 libraries.
- Requires `common_motor_interface`, `yaml-cpp`, and system `libethercat`.
- This tree itself does not provide ROS nodes; ROS integration is done in `ros2/motion_system_pkg`.
