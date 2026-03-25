# motor_interface

`motor_interface` defines the common C++ contracts that all motor communication and driver code must follow.

## What To Understand First

- `MotorMaster`: cycle-level communication control.
- `MotorController`: per-slave read/write/check behavior.
- `MotorDriver`: device-specific mapping and conversion logic.

## Why This Package Is Important

Almost all low-level motor code depends on these abstractions. If behavior differs between implementations, this is the contract they should still satisfy.

## Key Shared Types

- `master_config_t`, `slave_config_t`, `driver_config_t`
- `entry_table_t`, `DataType`, `DriverState`
- `motor_frame_t` from `common_motor_interface`

## Practical Use

Use this package as a reference when adding a new communication backend or a new drive type.
