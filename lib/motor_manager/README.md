# motor_manager (C++ stack)

This tree implements the low-level motor runtime that talks to hardware and executes periodic control cycles.

## Start Here

Read in this order:

1. `motor_manager/README.md`
2. `core/motor_interface/README.md`
3. `communications/ethercat/README.md`
4. `hardware/minas/README.md`

## Architecture

- `core/motor_interface`: abstract contracts and shared low-level types.
- `communications/ethercat`: bus communication backend.
- `hardware/minas`: drive-specific behavior and conversion.
- `motor_manager`: orchestrates object construction and cyclic loop.

## Runtime Flow

1. Load YAML configuration.
2. Build masters, controllers, and drivers.
3. Execute periodic receive/check/update/transmit loop.
4. Expose thread-safe command and status frame buffers.

## Dependencies

- `common_motor_interface`
- `yaml-cpp`
- IgH EtherCAT `libethercat`

ROS node wrappers that use this library live in `ros2/motion_system_pkg`.
