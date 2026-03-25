# motor_interface

Headers under `include/motor_interface/`. Namespace: `motor_interface` unless noted.  
Uses `motor_frame_t` and `MAX_INTERFACE_SIZE` from `common_motor_interface/motor_frame.hpp`.

## `motor_master.hpp`

### `struct master_config_t`

| Field | Type | Meaning |
|-------|------|---------|
| `id` | `uint8_t` | Master instance id. |
| `number_of_slaves` | `uint8_t` | Slave count on this master. |
| `master_index` | `unsigned int` | IgH master index (EtherCAT). |

### `class MotorMaster` (abstract)

#### Constructor

`explicit MotorMaster(const master_config_t& config)` — stores `id_`, `number_of_slaves_`.

#### Public virtual methods

| Method | Meaning |
|--------|---------|
| `initialize` | One-time setup. |
| `activate` | Bring master online for cyclic IO. |
| `deactivate` | Tear down / stop IO. |
| `transmit` | Send process data. |
| `receive` | Receive process data. |
| `apply_application_time` | Pass application time to stack (DC sync). |
| `save_clock` | Clock sync housekeeping. |

#### Accessors

`id()`, `number_of_slaves()`

#### Protected

`id_`, `number_of_slaves_`

---

## `motor_controller.hpp`

### `struct slave_config_t`

| Field | Meaning |
|-------|---------|
| `controller_index` | Index in manager’s controller array. |
| `master_id` | Which `MotorMaster` owns this slave. |
| `driver_id` | Which `MotorDriver` provides mapping. |
| `alias`, `position` | EtherCAT topology addressing. |
| `vendor_id`, `product_id` | Slave identity. |

### `class MotorController` (abstract)

#### Constructor

Binds `index_`, `master_id_`, `driver_id_`.

#### Public virtual methods

| Method | Meaning |
|--------|---------|
| `initialize(MotorMaster&, MotorDriver&)` | Wire PDOs / SDOs to master and driver. |
| `enable` | `bool` success per slave. |
| `disable` | `bool` success per slave. |
| `check(const motor_frame_t& status)` | Validate/transition drive state from status. |
| `write(const motor_frame_t& command)` | Apply command to process data. |
| `read(motor_frame_t& status)` | Fill `motor_frame_t` from process data. |

#### Accessors

`master_id()`, `driver_id()`

#### Protected virtual

`registerEntries()`, `writeData(...)`, `readData(...)`

#### Protected fields

`driver_`, `current_driver_state_`, `index_`, `master_id_`, `driver_id_`

---

## `motor_driver.hpp`

### Constants

| Name | Value | Typical PDO meaning |
|------|-------|---------------------|
| `MAX_DATA_SIZE` | 4 | Bytes per `entry_table_t::data`. |
| `MAX_ITEM_SIZE` | 32 | Max SDO/item entries. |
| `ID_CONTROLWORD` … `ID_CURRENT_TORQUE` | 0–8 | Semantic ids for items/interfaces. |

### `enum class DataType`

`U8`, `U16`, `U32`, `U64`, `S8`, `S16`, `S32`

### `enum class DriverState`

`SwitchOnDisabled`, `ReadyToSwitchOn`, `SwitchedOn`, `OperationEnabled`

### `struct driver_config_t`

Numeric limits and profile parameters: `id`, `pulse_per_revolution`, `rated_torque`, `unit_torque`, `lower`, `upper`, `speed`, `acceleration`, `deceleration`, `profile_velocity`, `profile_acceleration`, `profile_deceleration`.

### `struct entry_table_t`

| Field | Meaning |
|-------|---------|
| `id` | Semantic id (e.g. `ID_CONTROLWORD`). |
| `index`, `subindex` | Object dictionary address. |
| `type` | `DataType`. |
| `size` | Payload size in bytes. |
| `data` | Raw little-endian buffer (`MAX_DATA_SIZE`). |

### `toDataType(const std::string&) -> DataType`

Parses `"u8"`, `"u16"`, … throws on invalid.

### `template value<T>(const uint8_t* data) -> T`

Little-endian decode from `data`.

### `template fill<T>(const T& value, uint8_t* data)`

Little-endian encode into `data`.

### `class MotorDriver` (abstract)

#### Constructor

`explicit MotorDriver(const driver_config_t& config)` — stores `config_`.

#### Public virtual methods

| Method | Meaning |
|--------|---------|
| `loadParameters(const std::string& param_file)` | Load YAML/parameter file for PDO map and scaling. |
| `isEnabled` / `isDisabled` | Interpret raw `data` with `DriverState` machine; write `out` controlword bytes if needed. |
| `isReceived` | Check setpoint acknowledgment. |
| `position` / `velocity` / `torque` | Overloads: raw ↔ physical double conversions (drive units). |

#### Const accessors

`items()`, `interfaces()`, `number_of_items()`, `number_of_interfaces()`, `number_of_rx_interfaces()`, `number_of_tx_interfaces()`

#### Protected members

`items_[MAX_ITEM_SIZE]`, `interfaces_[MAX_INTERFACE_SIZE]`, counters, `config_`

**Note:** `interfaces_` capacity is `MAX_INTERFACE_SIZE` from `common_motor_interface` (16), not `MAX_ITEM_SIZE`.
