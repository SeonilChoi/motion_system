# motor_manager library

This module is the top-level orchestrator that builds configured masters, controllers, and drivers, then runs the periodic motor control loop.

## Role

- Parses system YAML and creates runtime objects.
- Owns cyclic execution timing and synchronization sequence.
- Exposes thread-safe command/state exchange APIs for external callers.

## Main Class

- `motor_manager::MotorManager`

## Responsibilities

- Load `period`, masters, slaves, and driver definitions from YAML
- Initialize communication and driver instances
- Execute receive, enable/check, update, and transmit phases per cycle
- Provide `write` and `read` APIs around `motor_frame_t` buffers

## Notes

- Designed for deterministic periodic execution.
- Used by ROS-side node wrappers in `ros2/motion_system_pkg`.
