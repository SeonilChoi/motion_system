# robot_manager (package)

Python package: `robot_manager`. Source: `src/robot_manager/robot_manager.py`.

## `__init__.py`

**`__all__`:** `RobotManager` — imported from `robot_manager.py`.

## Module-level

### `_ROBOT_BY_KEY: dict[str, type[Robot]]`

Maps YAML string `robot` (normalized: lower case, spaces → `_`) to a concrete `Robot` subclass.

| Key | Class |
|-----|-------|
| `'little_reader'` | `LittleReader` |
| `'silver_lain'` | `SilverLain` |

Unknown keys fall back to `LittleReader`.

### `_robot_class_for_key(key: str) -> type[Robot]`

Looks up `_ROBOT_BY_KEY` with normalized `key`.

---

## `class RobotManager`

Loads YAML, builds **one `Robot` per `robots` list entry**, and exposes multi-robot `get_state` / `set_action` / `stride_length`.

### Constructor

`__init__(self, config_file: str) -> None`

| Parameter | Meaning |
|-----------|---------|
| `config_file` | Path to YAML. If missing or not a file, `_config` stays `{}` and defaults apply where coded. |

| Attribute | Meaning |
|-----------|---------|
| `_config` | Parsed root mapping from YAML. |
| `_dt` | `float(self._config.get('dt', 0.01))`. |
| `_number_of_robots` | From `number_of_robots` key until `_build_robots` runs; **overwritten** when a `robots` list is used (see below). |
| `_robots` | `list[Robot]` instances. |

### Properties

| Property | Type | Meaning |
|----------|------|---------|
| `dt` | `float` | Control period for timers (`robot_manager_node`). |
| `number_of_robots` | `int` | `len(_robots)` after build. |

### Methods

| Method | Signature | Meaning |
|--------|-----------|---------|
| `get_state` | `(robot_id: int) -> StateFrame` | `self._robots[robot_id].get_state()`. |
| `set_action` | `(action_frame_list: list[ActionFrame]) -> None` | For each index `i`, `self._robots[i].set_action(frame)` while `i < len(_robots)`. |
| `stride_length` | `(robot_id: int) -> float` | `self._robots[robot_id].stride_length`. |
| `_loadConfigurations` | `(config_file: str) -> None` | `yaml.safe_load` into `_config` if path is a file and root is a `Mapping`. |
| `_build_robots` | `() -> None` | If `robots` is a non-empty `list`, instantiates one robot per element (see YAML). Otherwise `_robots` is left empty in the current implementation—prefer a non-empty `robots` list in config. |

### YAML layout (supported)

**Multi-robot (recommended)** — list order is `robot_id` **0, 1, …** (the optional `id` field is documentation only; not used for ordering).

```yaml
dt: 0.01
number_of_robots: 2   # informational; count follows len(robots) when `robots` is set

robots:
  - id: 0
    robot: silver_lain
    stride_length: 0.1
  - id: 1
    robot: little_reader
    stride_length: 0.0
```

| Key (root) | Type | Role |
|------------|------|------|
| `dt` | number | Passed as `dt=` to each `Robot`. |
| `number_of_robots` | int | Initial value before build; when `robots` exists, `_number_of_robots` becomes `len(robots)`. |
| `robots` | list | Each item: at least `robot` (string); optional `stride_length` (number) passed to `Robot(..., stride_length=...)`. |

Joystick → `ActionFrame` mapping lives in **`motion_system_pkg` / `robot_manager_node`**, not in `RobotManager`.
