# ethercat

Headers: `include/ethercat/`. Namespace: `ethercat`.

## `ethercat_master.hpp`

### `class EthercatMaster : motor_interface::MotorMaster`

IgH EtherCAT master wrapper.

#### Constructor

`explicit EthercatMaster(const motor_interface::master_config_t& config)` — stores `master_index_` from `config.master_index`.

#### Public overrides

Same as `MotorMaster`: `initialize`, `activate`, `deactivate`, `transmit`, `receive`, `apply_application_time`, `save_clock`.

#### Accessors

| Method | Return | Meaning |
|--------|--------|---------|
| `master` | `ec_master_t*` | IgH master handle. |
| `domain` | `ec_domain_t*` | Process-data domain. |
| `domain_pd` | `uint8_t*` | Domain process-data image. |
| `master_index` | `unsigned int` | Configured master index. |

#### Private

`master_`, `domain_`, `domain_pd_`, `master_index_`

---

## `ethercat_controller.hpp`

### ID constants

Duplicates semantic ids (`ID_CONTROLWORD` … `ID_CURRENT_TORQUE`) aligned with `motor_interface::MotorDriver` for PDO mapping.

### `class EthercatController : motor_interface::MotorController`

Per-slave EtherCAT controller: registers PDO entries, maps `motor_frame_t` ↔ process data.

#### Constructor

`explicit EthercatController(const motor_interface::slave_config_t& config)` — stores `alias_`, `position_`, `vendor_id_`, `product_id_`.

#### Public overrides

| Method | Meaning |
|--------|---------|
| `initialize` | Casts master to `EthercatMaster*`, configures slave SDOs/PDOs, registers entries, builds offsets. |
| `registerEntries` | (override) PDO entry registration. |
| `enable` / `disable` | Drive state machine via driver. |
| `check` | Status/controlword checks using `motor_frame_t`. |
| `write` / `read` | Copy between `motor_frame_t` and domain image via `writeData` / `readData`. |

#### Private

`writeData`, `readData`, `addSlaveConfigSdos`, `addSlaveConfigPdos`, `master_`, `slave_config_`, `offset_[]`, `tx_interfaces_[]`, topology ids.
