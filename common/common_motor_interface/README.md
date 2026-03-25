# common_motor_interface

`common_motor_interface` is a shared C++ interface package that defines motor frame data structures used by the low-level motor stack.

## Role

- Provides canonical motor I/O types so communication, driver, and manager layers use the same memory layout.
- Centralizes constants such as interface size limits to keep binary contracts consistent.

## Main Contents

- `include/common_motor_interface/motor_frame.hpp`
- `motor_frame_t` structure for command/state exchange
- `MAX_INTERFACE_SIZE` and related shared definitions

## Usage

This package is consumed by motor-side C++ libraries as a dependency and does not expose ROS nodes or runtime parameters by itself.
