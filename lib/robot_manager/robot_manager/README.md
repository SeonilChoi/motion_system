# robot_manager (package)

Python package: `robot_manager`. Source: `src/robot_manager/robot_manager.py`.

## `__init__.py`

**`__all__`:** `RobotManager` — imported from `robot_manager.py`.

## Module-level

### `_ROBOT_BY_KEY: dict[str, type[Robot]]`

Maps YAML/config string `robot` (normalized) to a concrete `Robot` subclass.

| Key | Class |
|-----|-------|
| `'little_reader'` | `LittleReader` |
| `'silver_lain'` | `SilverLain` |

Unknown keys fall back to `LittleReader`.

---

## `class RobotManager`

Loads YAML config, constructs the selected `Robot`, maps joystick edges to `Action`, and forwards walk vectors.

### Constructor

`__init__(self, config_file: str, *, stride_length: float | None = None) -> None`

| Parameter | Meaning |
|-----------|---------|
| `config_file` | Path to YAML; if missing or invalid, internal config stays empty and defaults apply. |
| `stride_length` | If not `None`, overrides `stride_length` from file after load. |

| Private attribute | Meaning |
|-------------------|---------|
| `_config` | Parsed mapping from YAML. |
| `_robot` | Instance of selected `Robot`. |
| `_current_action_kind` | Last latched `ActionKind` from buttons or stop logic. |
| `_joy_button_action_kind` | Maps `JoyButton` → `ActionKind` (A→HOME, B→MOVE, X→WALK, Y→STOP). |

### Properties

| Property | Type | Meaning |
|----------|------|---------|
| `dt` | `float` | From `_config['dt']`, default `0.01`. |
| `stride_length` | `float` | From `_config['stride_length']`, default `0.5`; clamped to `0.5` if `<= 0`. |

### Methods

| Method | Signature | Meaning |
|--------|-----------|---------|
| `get_state` | `() -> State` | Delegates to `self._robot.get_state()`. |
| `joy_stick_command` | `(axes: dict[JoyAxes, float], button: dict[JoyButton, bool], prev_button: dict[JoyButton, bool]) -> None` | If robot state is `STOPPED`, forces `_current_action_kind` to `STOP`. On rising edge of a mapped button, sets action kind. Non-`WALK`: `set_action(Action(kind=...))`. `WALK`: builds `goal` from `LEFT_VERTICAL` and `RIGHT_HORIZONTAL`, computes `duration` from stick magnitude and `stride_length`, then `set_action` with `ActionKind.WALK`. |
| `_loadConfigurations` | `(config_file: str) -> None` | Loads YAML into `_config` if path exists and root is a `Mapping`. |
| `_robot_class_from_config` | `() -> type[Robot]` | Reads `_config['robot']`, normalizes string, returns class from `_ROBOT_BY_KEY` or `LittleReader`. |

### YAML keys (read by this class)

| Key | Type | Role |
|-----|------|------|
| `robot` | `str` | Robot type key (`little_reader`, `silver_lain`, …). |
| `dt` | number | Tick period for robot construction. |
| `stride_length` | number | Nominal stride for walk duration computation. |
