# motor_manager (library)

**`MotorManager`**: loads motor YAML (`period`, `masters`, `drivers`), owns cyclic **`write` / `read`** on `motor_frame_t[]`, and runs **`run()`** in a realtime loop (EtherCAT + MINAS / ZeroErr drivers). Constants include `MAX_CONTROLLER_SIZE` (16). See header `include/motor_manager/motor_manager.hpp`.

## `MotorManager` functions (public)

| Function | Description |
|----------|-------------|
| `MotorManager(config_file)` | Calls `loadConfigurations` (YAML: `period`, `masters`, `drivers`), then `initialize` (masters `initialize`, each controller `initialize` with its master and driver). Resolves relative `param_file` paths against the config file’s directory. |
| `run()` | Sets `running`, activates all masters (`start`), locks memory (`mlockall`), switches to `SCHED_FIFO` at max priority, prefaults stack, then loops until `request_stop`: absolute `clock_nanosleep` for `period_` ns, `apply_application_time` on each master, `receive`, `enable` until all axes report enabled then `update` each cycle, `save_clock`, `transmit`. On exit: unlock memory, `stop` (deactivate masters). Throws on scheduler / sleep / page-size errors. |
| `request_stop()` | Clears the `running` flag; the next loop iteration exits `run()` and runs `stop()`. |
| `write(command, size)` | Under `frame_mutex_`, copies up to `min(size, MAX_CONTROLLER_SIZE)` frames into `command_` and sets `is_command_changed_`. |
| `read(status)` | Under `frame_mutex_`, copies `status_[0..number_of_controllers_)` into `status`. |
| `period()` | Control-loop period in nanoseconds from YAML `period`. |
| `number_of_controllers()` | Count of instantiated controllers (slaves across masters). |

## Namespace functions (`motor_manager`)

| Function | Description |
|----------|-------------|
| `toCommunicationType(type)` | Maps `"ethercat"` / `"canopen"` / `"dynamixel"` to `CommunicationType`; throws on unknown string. Only EtherCAT is implemented in `loadConfigurations`. |
| `toDriverType(type)` | Maps `"minas"` / `"zeroerr"` / `"dynamixel"` to `DriverType`; throws on unknown string. Only Minas and Zeroerr are constructed. |

## Internal (`motor_manager.cpp`)

Not part of the public API:

| Symbol | Description |
|--------|-------------|
| `loadConfigurations` | Parses YAML into `masters_`, `controllers_[]`, `drivers_`, `number_of_controllers_`, `period_`. |
| `initialize` | Sets `frequency_` from `period_`, initializes all masters, wires each controller to its master and driver. |
| `start` / `stop` | `activate` / `deactivate` every master. |
| `enable` | Calls `enable()` on each controller until all return true; sets `is_enable_`. |
| `update` | Under mutex: `read` then `check` on each controller; if commands changed, `write` each `command_[i]`. |
| `unlock_memory`, `stack_prefault` | Anonymous namespace helpers for `mlockall` / stack prefault in `run()`. |
