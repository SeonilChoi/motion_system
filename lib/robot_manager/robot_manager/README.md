# robot_manager (package)

---

## `src/robot_manager/robot_manager.py`

### Variables (module)

| Name | Type | Description |
|------|------|-------------|
| `_ROBOT_BY_KEY` | `dict[str, type[Robot]]` | Maps YAML `robot` string (lowercase, spaces → `_`) to a concrete class: `'little_reader'` → `LittleReader`, `'silver_lain'` → `SilverLain`. Unknown keys fall back to `LittleReader` in `_loadConfigurations`. |

### Classes

#### `RobotManager`

| Instance attribute | Description |
|--------------------|-------------|
| `_config` | Parsed YAML `dict` after successful load; `{}` if load skipped. |
| `_dt` | Global control period (s) from YAML `dt`, default `0.01`. |
| `_number_of_motors` | Sum of all robots’ `controller_indexes` sizes. |
| `_number_of_robots` | Length of `robots[]` in YAML (constructed instances). |
| `_robots` | `List[Robot]` in YAML order. |
| `_home_joint_status` | Aggregated `JointStatus`: `motor_id` is `np.arange(n_motors)`, `interface_id` is concatenated per-robot `interface_ids`, `position` is concatenated home joints, v/t zero. Template for `set_action_frame` output. |

| Property | Description |
|----------|-------------|
| `dt` | `_dt`. |
| `number_of_motors` | Total motor count. |
| `number_of_robots` | Robot count. |

| Method | Description |
|--------|-------------|
| `__init__(config_file)` | Clears state, calls `_loadConfigurations`, then builds `_home_joint_status` from loaded robots. |
| `_loadConfigurations(config_file)` | Resolves path; if missing or not a file, returns without changing robots. Loads YAML with `safe_load`; if not a `dict`, returns. Sets `_config`, `_dt`, iterates `robots` list: builds `RobotConfig` per row (`id`, `stride_length`, `clearance`, `duration`, `controller_indexes`, `interface_ids`, `home_joint_positions`, `home_pose`), picks impl from `_ROBOT_BY_KEY` (default `little_reader`), appends instance, sums motors. Updates `_robots`, counts. |
| `stride_length(robot_id)` | Delegates to `_robots[robot_id].stride_length`. |
| `duration(robot_id)` | Delegates to `_robots[robot_id].duration`. |
| `get_state_frame(robot_id)` | `_robots[robot_id].get_state_frame()`. |
| `set_action_frame(action_frame_list)` | Deep-copies `_home_joint_status` into `joint_commands`. For each `(robot_id, frame)`, calls `set_action_frame` on that robot. **Expects** a return value indexable as `commands[:, 0]`, `[:, 1]`, `[:, 2]` (shape `(n_motors_on_robot, 3)`) and scatters rows into global `position` / `velocity` / `torque` at `controller_indexes`. Returns the filled `JointStatus`. |
| `get_robot_status(robot_id)` | `_robots[robot_id].get_robot_status()`. |
| `update_joint_status(joint_status)` | For each robot, slices `joint_status` by that robot’s `controller_indexes` into a `JointStatus` sub-view and calls `update_joint_status`. |
| `reset()` | Calls `reset()` on every robot in `_robots`. |

### Functions

Module-level free functions: none.

---

## `src/robot_manager/__init__.py`

### Variables / classes / functions

Re-exports **`RobotManager`** only (`__all__ = ['RobotManager']`).
