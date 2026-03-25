# common_motor_interface

This package provides shared C++ frame structures used by motor runtime modules.

## Why It Exists

- Keeps command and feedback memory layout consistent across all motor libraries.
- Centralizes shared constants such as frame size limits.

## Core File

- `include/common_motor_interface/motor_frame.hpp`

## Main Types

- `motor_frame_t`
- `MAX_INTERFACE_SIZE`

## Scope

This package is interface-only. It has no ROS node and no runtime loop.
