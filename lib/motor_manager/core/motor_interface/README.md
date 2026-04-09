# motor_interface

C++ **motor abstraction** headers (`ament_cmake`). Namespace: `motor_interface`. Process command/status payloads use `motor_frame_t` from `common_motor_interface/motor_frame.hpp`.

## `master_config_t` (`motor_master.hpp`)

| Field | Type | Meaning |
|-------|------|---------|
| `id` | `uint8_t` | Master instance id (YAML `masters[].id`). |
| `number_of_slaves` | `uint8_t` | Slave count on this master. |
| `master_index` | `unsigned int` | IgH EtherCAT master index (EtherCAT only). |

## `slave_config_t` (`motor_controller.hpp`)

| Field | Type | Meaning |
|-------|------|---------|
| `controller_index` | `uint8_t` | Dense index in `MotorManager` controller array. |
| `master_id` | `uint8_t` | Owning `MotorMaster` id. |
| `driver_id` | `uint8_t` | `MotorDriver` id for PDO mapping / scaling. |
| `alias` | `uint16_t` | EtherCAT alias. |
| `position` | `uint16_t` | EtherCAT ring position. |
| `vendor_id` | `uint32_t` | Slave vendor id. |
| `product_id` | `uint32_t` | Slave product code. |

## `driver_config_t` (`motor_driver.hpp`)

| Field | Type | Meaning |
|-------|------|---------|
| `id` | `uint8_t` | Driver instance id. |
| `pulse_per_revolution` | `uint32_t` | Encoder pulses per revolution (scaling). |
| `rated_torque` | `double` | Rated torque (Nm, drive datasheet). |
| `unit_torque` | `double` | Raw unit ↔ Nm scale. |
| `lower` | `double` | Position or command lower limit (drive units / physical). |
| `upper` | `double` | Upper limit. |
| `speed` | `double` | Speed limit / profile parameter. |
| `acceleration` | `double` | Acceleration limit. |
| `deceleration` | `double` | Deceleration limit. |
| `profile_velocity` | `double` | Profile velocity. |
| `profile_acceleration` | `double` | Profile acceleration. |
| `profile_deceleration` | `double` | Profile deceleration. |

## `entry_table_t` (`motor_driver.hpp`)

| Field | Type | Meaning |
|-------|------|---------|
| `id` | `uint8_t` | Semantic id (see table below). |
| `index` | `uint16_t` | CANopen / object dictionary index. |
| `subindex` | `uint8_t` | Subindex. |
| `type` | `DataType` | `U8` … `S32`. |
| `size` | `uint8_t` | Payload size in bytes (≤ `MAX_DATA_SIZE`). |
| `data` | `uint8_t[MAX_DATA_SIZE]` | Little-endian raw value buffer. |

## Semantic `id` values (`motor_driver.hpp`)

| Constant | Value | Typical role |
|----------|-------|----------------|
| `ID_CONTROLWORD` | 0 | CiA402 controlword |
| `ID_TARGET_POSITION` | 1 | Target position |
| `ID_TARGET_VELOCITY` | 2 | Target velocity |
| `ID_TARGET_TORQUE` | 3 | Target torque |
| `ID_STATUSWORD` | 4 | Statusword |
| `ID_ERRORCODE` | 5 | Error code |
| `ID_CURRENT_POSITION` | 6 | Actual position |
| `ID_CURRENT_VELOCITY` | 7 | Actual velocity |
| `ID_CURRENT_TORQUE` | 8 | Actual torque |

## `DataType` (enum)

`U8`, `U16`, `U32`, `U64`, `S8`, `S16`, `S32` — used in `entry_table_t` and YAML parameter files.

## `DriverState` (enum)

`SwitchOnDisabled`, `ReadyToSwitchOn`, `SwitchedOn`, `OperationEnabled` — CiA402-style state for enable sequencing.

## Class roles (no field tables)

- **`MotorMaster`** — Cyclic bus I/O (`initialize`, `activate`, `transmit`, `receive`, DC time APIs).
- **`MotorController`** — One slave: maps `motor_frame_t` ↔ process data via driver `entry_table_t` / interfaces.
- **`MotorDriver`** — Vendor PDO map, scaling, `loadParameters`, enable/disable checks.

Constants: `MAX_DATA_SIZE` = 4, `MAX_ITEM_SIZE` = 32 (items vs interfaces; interface count capped by `MAX_INTERFACE_SIZE` in `common_motor_interface`).
