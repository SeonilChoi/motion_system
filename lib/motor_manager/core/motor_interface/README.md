# motor_interface

`motor_interface` defines abstract C++ interfaces used by motor communication and hardware driver implementations.

## Role

- Provides base contracts for master, controller, and driver components.
- Defines configuration structures and entry metadata shared across implementations.
- Standardizes read/write/check behavior over `motor_frame_t`.

## Main Interfaces

- `MotorMaster`: bus-level lifecycle and cyclic transport hooks
- `MotorController`: per-slave read/write/enable/check logic
- `MotorDriver`: device-specific conversion, PDO/SDO mapping, and state checks

## Key Shared Types

- `master_config_t`, `slave_config_t`, `driver_config_t`
- `entry_table_t`, `DataType`, `DriverState`
- `motor_frame_t` from `common_motor_interface`

## Scope

This package is interface-focused and does not execute control loops directly.
