# motor_interface

C++ motor abstraction headers (`ament_cmake`). Namespace: `motor_interface`. Command/status payloads use `motor_frame_t` from `common_motor_interface/motor_frame.hpp`.

---

## `include/motor_interface/motor_master.hpp`

### Classes

#### `MotorMaster`

Abstract bus master: lifecycle, cyclic `transmit` / `receive`, and DC clock hooks.

| Kind | Name | Notes |
|------|------|--------|
| ctor | `MotorMaster(const master_config_t& config)` | Stores `id` and slave count from config. |
| dtor | `virtual ~MotorMaster() = default` | |
| method | `virtual void initialize() = 0` | |
| method | `virtual void activate() = 0` | |
| method | `virtual void deactivate() = 0` | |
| method | `virtual void transmit() = 0` | Send process data. |
| method | `virtual void receive() = 0` | Receive process data. |
| method | `virtual void apply_application_time(const timespec& time) = 0` | Distributed clock / app time. |
| method | `virtual void save_clock() = 0` | |
| method | `uint8_t id() const` | Master instance id. |
| method | `uint8_t number_of_slaves() const` | Slave count on this master. |

Protected members: `id_` (`uint8_t`), `number_of_slaves_` (`uint8_t`).

### Structs

#### `master_config_t`

| Field | Type | Meaning |
|-------|------|---------|
| `id` | `uint8_t` | Master instance id (YAML `masters[].id`). |
| `number_of_slaves` | `uint8_t` | Slave count on this master. |
| `master_index` | `unsigned int` | IgH EtherCAT master index (EtherCAT implementations). |

---

## `include/motor_interface/motor_controller.hpp`

Depends on `motor_master.hpp`, `motor_driver.hpp`, and `common_motor_interface/motor_frame.hpp`.

### Classes

#### `MotorController`

Abstract per-slave controller: maps `motor_frame_t` ↔ process data via the driver’s `entry_table_t` / interface layout.

| Kind | Name | Notes |
|------|------|--------|
| ctor | `MotorController(const slave_config_t& config)` | Stores controller index, `master_id`, `driver_id`. |
| dtor | `virtual ~MotorController() = default` | |
| method | `virtual void initialize(MotorMaster& master, MotorDriver& driver) = 0` | |
| method | `virtual bool enable() = 0` | |
| method | `virtual bool disable() = 0` | |
| method | `virtual void check(const motor_frame_t& status) = 0` | |
| method | `virtual void write(const motor_frame_t& command) = 0` | |
| method | `virtual void read(motor_frame_t& status) = 0` | |
| method | `uint8_t master_id() const` | |
| method | `uint8_t driver_id() const` | |

Protected: `virtual void registerEntries() = 0`; `virtual void writeData(const entry_table_t* rx_interfaces, uint8_t number_of_rx_interfaces) = 0`; `virtual void readData(entry_table_t* tx_interfaces, uint8_t number_of_tx_interfaces) = 0`; `MotorDriver* driver_{nullptr}`; `DriverState current_driver_state_{DriverState::SwitchOnDisabled}`; `index_`, `master_id_`, `driver_id_` (`const uint8_t`).

### Structs

#### `slave_config_t`

| Field | Type | Meaning |
|-------|------|---------|
| `controller_index` | `uint8_t` | Dense index in `MotorManager` controller array. |
| `master_id` | `uint8_t` | Owning `MotorMaster` id. |
| `driver_id` | `uint8_t` | `MotorDriver` id for PDO mapping / scaling. |
| `alias` | `uint16_t` | EtherCAT alias. |
| `position` | `uint16_t` | EtherCAT ring position. |
| `vendor_id` | `uint32_t` | Slave vendor id. |
| `product_id` | `uint32_t` | Slave product code. |

---

## `include/motor_interface/motor_driver.hpp`

### Classes

#### `MotorDriver`

Abstract vendor driver: YAML parameter load, CiA402-style enable/disable checks, raw ↔ physical scaling, and PDO `entry_table_t` layout (`items_`, `interfaces_`).

| Kind | Name | Notes |
|------|------|--------|
| ctor | `MotorDriver(const driver_config_t& config)` | |
| dtor | `virtual ~MotorDriver() = default` | |
| method | `virtual void loadParameters(const std::string& param_file) = 0` | |
| method | `virtual bool isEnabled(const uint8_t* data, DriverState& driver_state, uint8_t* out) = 0` | |
| method | `virtual bool isDisabled(const uint8_t* data, DriverState& driver_state, uint8_t* out) = 0` | |
| method | `virtual bool isReceived(const uint8_t* data, uint8_t* out) = 0` | |
| method | `virtual double position(const int32_t value) = 0` | Raw → physical position. |
| method | `virtual double velocity(const int32_t value) = 0` | Raw → physical velocity. |
| method | `virtual double torque(const int16_t value) = 0` | Raw → physical torque. |
| method | `virtual int32_t position(const double value) = 0` | Physical → raw position. |
| method | `virtual int32_t velocity(const double value) = 0` | Physical → raw velocity. |
| method | `virtual int16_t torque(const double value) = 0` | Physical → raw torque. |
| method | `const entry_table_t* items() const` | PDO / object map entries. |
| method | `const entry_table_t* interfaces() const` | RX/TX interface slices. |
| method | `uint8_t number_of_items() const` | |
| method | `uint8_t number_of_interfaces() const` | Capped by `MAX_INTERFACE_SIZE` (`common_motor_interface`). |
| method | `uint8_t number_of_rx_interfaces() const` | |
| method | `uint8_t number_of_tx_interfaces() const` | |

Protected members: `items_[MAX_ITEM_SIZE]`, `interfaces_[MAX_INTERFACE_SIZE]`, `number_of_items_`, `number_of_interfaces_`, `number_of_rx_interfaces_`, `number_of_tx_interfaces_`, `config_` (`const driver_config_t`).

### Structs

#### `driver_config_t`

| Field | Type | Meaning |
|-------|------|---------|
| `id` | `uint8_t` | Driver instance id. |
| `pulse_per_revolution` | `uint32_t` | Encoder pulses per revolution (scaling). |
| `rated_torque` | `double` | Rated torque (Nm, datasheet). |
| `unit_torque` | `double` | Raw unit ↔ Nm scale. |
| `lower` | `double` | Position/command lower limit. |
| `upper` | `double` | Upper limit. |
| `speed` | `double` | Speed limit / profile parameter. |
| `acceleration` | `double` | Acceleration limit. |
| `deceleration` | `double` | Deceleration limit. |
| `profile_velocity` | `double` | Profile velocity. |
| `profile_acceleration` | `double` | Profile acceleration. |
| `profile_deceleration` | `double` | Profile deceleration. |

#### `entry_table_t`

| Field | Type | Meaning |
|-------|------|---------|
| `id` | `uint8_t` | Semantic id (see **Variables** below). |
| `index` | `uint16_t` | CANopen / object dictionary index. |
| `subindex` | `uint8_t` | Subindex. |
| `type` | `DataType` | `U8` … `S32`. |
| `size` | `uint8_t` | Payload size in bytes (≤ `MAX_DATA_SIZE`). |
| `data` | `uint8_t[MAX_DATA_SIZE]` | Little-endian raw buffer. |

### Enums

#### `DataType`

`U8`, `U16`, `U32`, `U64`, `S8`, `S16`, `S32` — used in `entry_table_t` and YAML parameter files.

#### `DriverState`

`SwitchOnDisabled`, `ReadyToSwitchOn`, `SwitchedOn`, `OperationEnabled` — CiA402-style enable sequencing (`MotorController` tracks `current_driver_state_`).

### Functions

| Name | Notes |
|------|--------|
| `DataType toDataType(const std::string& type)` | Parses `"u8"` … `"s32"`; throws `std::runtime_error` if invalid. |
| `template <typename T> T value(const uint8_t* data)` | Little-endian decode from `data` (size `sizeof(T)`). |
| `template <typename T> void fill(const T& value, uint8_t* data)` | Little-endian encode into `data`. |

### Variables

`inline constexpr` limits:

| Name | Value | Meaning |
|------|-------|---------|
| `MAX_DATA_SIZE` | `4` | Max bytes in `entry_table_t::data`. |
| `MAX_ITEM_SIZE` | `32` | Max `items_` entries. |

Semantic entry `id` constants (for `entry_table_t::id`):

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
