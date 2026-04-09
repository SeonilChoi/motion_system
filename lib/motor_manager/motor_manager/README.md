# motor_manager library

Header: `include/motor_manager/motor_manager.hpp`  
Implementation: `src/motor_manager.cpp`  
Namespace: `motor_manager`

## Constants

| Name | Type | Value | Meaning |
|------|------|-------|---------|
| `NSEC_PER_SEC` | `uint64_t` | `1000000000` | Nanoseconds per second (loop timing). |
| `MAX_MASTER_SIZE` | `uint8_t` | `8` | Cap on masters in `masters_`. |
| `MAX_DRIVER_SIZE` | `uint8_t` | `8` | Cap on drivers in `drivers_`. |
| `MAX_CONTROLLER_SIZE` | `uint8_t` | `16` | Max controllers; size of `command_` / `status_` arrays and `write`/`read` bounds. |

## `enum class CommunicationType`

| Enumerator | String in YAML (`masters[].type`) |
|------------|-----------------------------------|
| `Ethercat` | `"ethercat"` |
| `Canopen` | `"canopen"` |
| `Dynamixel` | `"dynamixel"` |

## `enum class DriverType`

| Enumerator | String in YAML (`drivers[].type`) |
|------------|-------------------------------------|
| `Minas` | `"minas"` |
| `Zeroerr` | `"zeroerr"` |
| `Dynamixel` | `"dynamixel"` |

## Free functions

| Function | Signature | Meaning |
|----------|-----------|---------|
| `toCommunicationType` | `(const std::string& type) -> CommunicationType` | Parses YAML type string; throws `std::runtime_error` if invalid. |
| `toDriverType` | `(const std::string& type) -> DriverType` | Parses driver type string; throws if invalid. |

## `class MotorManager`

### Constructor / destructor

| | |
|---|---|
| `MotorManager(const std::string& config_file)` | Loads YAML, builds masters/controllers/drivers, calls `initialize()`. |
| `virtual ~MotorManager() = default` | Default. |

### Public methods

| Method | Signature | Meaning |
|--------|-----------|---------|
| `run` | `void()` | Sets `running_`, activates masters, applies RT scheduling (`SCHED_FIFO`), `mlockall`, periodic loop: `apply_application_time` → `receive` → `enable`/`update` → `save_clock` → `transmit` until `request_stop()`. Then unlock memory and `stop()`. |
| `request_stop` | `void()` | Clears `running_`; loop exits on next iteration. |
| `write` | `(const motor_interface::motor_frame_t* command, uint8_t size)` | Mutex copy of up to `min(size, MAX_CONTROLLER_SIZE)` frames into `command_`; sets `is_command_changed_`. |
| `read` | `(motor_interface::motor_frame_t* status)` | Mutex copy of `number_of_controllers_` frames from `status_` into `status`. |
| `period` | `uint32_t() const` | Cycle period from YAML `period` (nanoseconds). |
| `number_of_controllers` | `uint8_t() const` | Controllers instantiated from slave entries. |

### Private methods (call graph / extension reference)

| Method | Meaning |
|--------|---------|
| `loadConfigurations` | Parses YAML: `period`, `masters` (EtherCAT builds `EthercatMaster` + `EthercatController` per slave), `drivers` (`"minas"` → `MinasDriver`, `"zeroerr"` → `ZeroerrDriver`; resolves `param_file` relative to the config directory and calls `loadParameters`). |
| `initialize` | `master->initialize()` for all; each controller `initialize(master, driver)`. |
| `start` / `stop` | All masters `activate` / `deactivate`. |
| `enable` | Calls `enable()` on each controller until all succeed (`is_enable_`). |
| `disable` | Calls `disable()` on each controller (`is_disabled_`). |
| `update` | Per controller: `read(status_)`, `check(status_)`; if command changed, `write(command_)` to controllers and clear flag. |

### Private data members (reference)

| Member | Meaning |
|--------|---------|
| `masters_` | `id` → `MotorMaster` (e.g. `EthercatMaster`). |
| `drivers_` | `id` → `MotorDriver` (e.g. `MinasDriver`). |
| `controllers_` | Dense array indexed by `controller_index` up to `number_of_controllers_`. |
| `period_`, `frequency_` | Timing. |
| `is_enable_`, `is_disabled_` | Enable state flags. |
| `running_` | Atomic run flag. |
| `frame_mutex_` | Protects `command_`/`status_` and command-changed flag. |
| `command_`, `status_` | `motor_frame_t[MAX_CONTROLLER_SIZE]`. |

### YAML keys consumed by `loadConfigurations`

| Key | Role |
|-----|------|
| `period` | `uint32_t` nanoseconds per cycle. |
| `masters` | Sequence: `id`, `type`, `number_of_slaves`, EtherCAT `master_index`, `slaves[]` with `controller_index`, `driver_id`, `alias`, `position`, `vendor_id`, `product_id`. |
| `drivers` | Sequence: `id`, `type`, limits/torque/pulse fields in `driver_config_t`, `param_file`. |
