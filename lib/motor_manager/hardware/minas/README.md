# minas

Header: `include/minas/minas_driver.hpp`  
Source: `src/minas_driver.cpp`  
Namespace: `minas`

## Extra semantic IDs (`ID_MAX_TORQUE` … `ID_TXPDO`)

YAML-driven item ids for MINAS-specific objects (torque/position limits, profile parameters, RX/PDO group ids). See header for numeric values (50–58, 98–99).

## `class MinasDriver : motor_interface::MotorDriver`

Panasonic MINAS CiA402-style driver: parses parameter file, implements enable/disable/state checks, unit conversion.

### Constructor

`explicit MinasDriver(const motor_interface::driver_config_t& config)`

### Overrides

| Method | Meaning |
|--------|---------|
| `loadParameters(const std::string& param_file)` | Populate `items_` / `interfaces_` and counts from YAML. |
| `isEnabled` / `isDisabled` | Statusword/controlword state machine; update `DriverState` and output buffer. |
| `isReceived` | Target-reached style check (raw buffer). |
| `position` / `velocity` / `torque` | int16/32 ↔ double using `driver_config_t` scaling. |

**Usage:** constructed by `MotorManager` when `drivers[].type` is `"minas"`; `param_file` is resolved relative to the main motor YAML directory before `loadParameters`.
