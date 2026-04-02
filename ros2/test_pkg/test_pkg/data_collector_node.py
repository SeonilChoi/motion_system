#!/usr/bin/env python3

"""
Saved obs row layout (float64 1-D), from ``std_msgs/Float64MultiArray`` on ``obs`` topic:

| slice   | meaning                  |
|---------|--------------------------|
| [0:3]   | position (incoming); saved ``position.npy`` adds ``POSE_Z_OFFSET`` to z |
| [3:6]   | orientation              |
| [6:8]   | direction                |
| [8]     | progress (one element)   |
| [9:15]  | target_contact           |
| [15:18] | target_position (incoming); saved ``target_position.npy`` adds ``POSE_Z_OFFSET`` to z |
| [18:21] | → ``target_orientaiton.npy`` (filename as published) |
| [21:24] | target_linear_velocity   |
| [24:27] | target_angular_velocity    |
| [27:45] | target_joint_position (incoming); saved ``target_joint_position.npy`` applies offset, reorder, then direction |
| [45:]   | target_joint_velocity (incoming); saved ``target_joint_velocity.npy`` applies reorder, then direction (no offset) |

Joint count 18 for [27:45] and for velocities from index 45 onward.

Output layout: ``{output_dir}/{idx}/stem.npy`` (e.g. ``data/1/position.npy``). With non-empty
``output_tag``: ``{output_dir}/{tag}_{idx}/stem.npy``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64MultiArray


def _qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
    )


# Same as ros2/motion_system_pkg/config/silver_lain.yaml → robots[0].home_joint_positions
_SILVER_LAIN_HOME_JOINT_POSITIONS: list[float] = [
    0.0,
    -0.6981317007977318,
    2.443460952792061,
    0.0,
    -0.6981317007977318,
    2.443460952792061,
    0.0,
    -0.6981317007977318,
    2.443460952792061,
    0.0,
    0.6981317007977318,
    -2.443460952792061,
    0.0,
    0.6981317007977318,
    -2.443460952792061,
    0.0,
    0.6981317007977318,
    -2.443460952792061,
]

_OBS_JOINT_POS = slice(27, 45)
_OBS_JOINT_VEL_START = 45
_OBS_MIN_LEN = 45

POSE_Z_OFFSET = np.float32(-0.7788)

JOINT_POS_OFFSET = np.array(
    [
        0.0,
        0.6981317007977318,
        -2.443460952792061,
        0.0,
        0.6981317007977318,
        -2.443460952792061,
        0.0,
        0.6981317007977318,
        -2.443460952792061,
        0.0,
        -0.6981317007977318,
        2.443460952792061,
        0.0,
        -0.6981317007977318,
        2.443460952792061,
        0.0,
        -0.6981317007977318,
        2.443460952792061,
    ],
    dtype=np.float64,
)

JOINT_REORDER = np.array(
    [15, 12, 9, 6, 3, 0, 16, 13, 10, 7, 4, 1, 17, 14, 11, 8, 5, 2],
    dtype=np.intp,
)

JOINT_DIRECTION = np.array(
    [-1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, 1],
    dtype=np.float64,
)


def _joint_positions_for_storage(q: np.ndarray) -> np.ndarray:
    """obs[:, 27:45] → offset → column reorder → per-joint sign (for ``target_joint_position.npy``)."""
    x = np.asarray(q, dtype=np.float64) + JOINT_POS_OFFSET
    x = x[:, JOINT_REORDER]
    x = x * JOINT_DIRECTION
    return x


def _translation_for_storage(t: np.ndarray) -> np.ndarray:
    """Add ``POSE_Z_OFFSET`` to z for ``position.npy`` / ``target_position.npy``."""
    x = np.asarray(t, dtype=np.float64).copy()
    x[:, 2] = x[:, 2] + float(POSE_Z_OFFSET)
    return x


def _joint_velocities_for_storage(qdot: np.ndarray) -> np.ndarray:
    """obs joint velocity slice → column reorder → per-joint sign (for ``target_joint_velocity.npy``)."""
    x = np.asarray(qdot, dtype=np.float64)
    x = x[:, JOINT_REORDER]
    x = x * JOINT_DIRECTION
    return x


def _save_obs_split(obs: np.ndarray, path_for_stem: Callable[[str], Path]) -> None:
    """Save (N, D) rows into separate .npy files; ``path_for_stem(stem) -> Path``."""
    np.save(path_for_stem("position"), _translation_for_storage(obs[:, 0:3]))
    np.save(path_for_stem("orientation"), obs[:, 3:6])
    np.save(path_for_stem("direction"), obs[:, 6:8])
    np.save(path_for_stem("progress"), obs[:, 8:9])
    np.save(path_for_stem("target_contact"), obs[:, 9:15])
    np.save(path_for_stem("target_position"), _translation_for_storage(obs[:, 15:18]))
    np.save(path_for_stem("target_orientation"), obs[:, 18:21])
    np.save(path_for_stem("target_linear_velocity"), obs[:, 21:24])
    np.save(path_for_stem("target_angular_velocity"), obs[:, 24:27])
    np.save(
        path_for_stem("target_joint_position"),
        _joint_positions_for_storage(obs[:, 27:45]),
    )
    np.save(
        path_for_stem("target_joint_velocity"),
        _joint_velocities_for_storage(obs[:, 45:]),
    )


class DataCollectorNode(Node):
    def __init__(self) -> None:
        super().__init__("data_collector_node")
        self.declare_parameter("max_samples", 5000)
        self.declare_parameter("chunk_size", 1000)
        self.declare_parameter("file_index_start", 1)
        source_data_dir = Path("/home/csi/Documents/lab_ws/src/motion_system/ros2/test_pkg/data")
        default_output_dir = source_data_dir
        self.declare_parameter("output_dir", str(default_output_dir))
        self.declare_parameter(
            "output_tag",
            "",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("initial_joint_positions", _SILVER_LAIN_HOME_JOINT_POSITIONS)
        self.declare_parameter("joy_cmd_topic", "joy_cmd")
        self.declare_parameter("event_topic", "event")
        self.declare_parameter("contact_dim", 6)

        self._max_samples = int(self.get_parameter("max_samples").value)
        self._chunk_size = max(0, int(self.get_parameter("chunk_size").value))
        self._file_index = int(self.get_parameter("file_index_start").value)
        self._output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser().resolve()
        self._output_tag = str(self.get_parameter("output_tag").value).strip()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        _ = self.get_parameter("initial_joint_positions").value
        _ = str(self.get_parameter("joy_cmd_topic").value)
        _ = str(self.get_parameter("event_topic").value)
        _ = int(self.get_parameter("contact_dim").value)

        self._obs_buffer: list[np.ndarray] = []
        self._last_obs: Optional[np.ndarray] = None
        self._rows_recorded = 0
        self._saved = False
        self._warned_layout = False

        self.create_subscription(
            Float64MultiArray,
            "obs",
            self.obs_callback,
            _qos(),
        )

        self.get_logger().info(
            "Obs-only mode: subscribe 'obs' (std_msgs/Float64MultiArray); "
            "on change, save split .npy: position, orientation, direction, progress, "
            "target_contact, target_position, target_orientaiton, target_linear_velocity, "
            "target_angular_velocity, target_joint_position, target_joint_velocity."
        )
        self.get_logger().info(
            f"Target {self._max_samples} rows; same layout as incoming obs vector (min {_OBS_MIN_LEN} floats/row)."
        )
        if self._chunk_size > 0:
            n_files = (self._max_samples + self._chunk_size - 1) // self._chunk_size
            self.get_logger().info(
                f"Chunked save: {self._chunk_size} rows/file, indices {self._file_index}…"
                f"{self._file_index + n_files - 1} (~{n_files} files)"
            )
        else:
            self.get_logger().info("Single-file mode (chunk_size=0): split *.npy at end")
        if self._output_tag:
            self.get_logger().info(
                f"output_tag={self._output_tag!r} → e.g. {self._output_dir.name}/{self._output_tag}_<idx>/position.npy, …"
            )
        else:
            self.get_logger().info(
                f"Each chunk → subdirectory {self._output_dir.name}/<idx>/ (e.g. …/1/position.npy)"
            )

    def _run_subdir(self, file_idx: int) -> Path:
        """``output_dir / {tag}_{idx} /`` or ``output_dir / {idx} /``."""
        name = f"{self._output_tag}_{file_idx}" if self._output_tag else str(file_idx)
        return self._output_dir / name

    def _npy_path_in_run(self, stem: str, file_idx: int) -> Path:
        return self._run_subdir(file_idx) / f"{stem}.npy"

    def _check_layout(self, packed: np.ndarray) -> None:
        if self._warned_layout or packed.size < _OBS_MIN_LEN:
            return
        n_pos = packed[_OBS_JOINT_POS].size
        n_vel = packed.size - _OBS_JOINT_VEL_START
        if n_vel != n_pos:
            self.get_logger().warning(
                f"obs length {packed.size}: target_joint_position has {n_pos} elements "
                f"but target_joint_velocity has {n_vel}; expected equal."
            )
            self._warned_layout = True

    def obs_callback(self, msg: Float64MultiArray) -> None:
        if self._saved:
            return

        raw = np.asarray(msg.data, dtype=np.float64).reshape(-1)
        if raw.size < _OBS_MIN_LEN:
            self.get_logger().warning(
                f"Ignoring obs: need at least {_OBS_MIN_LEN} elements, got {raw.size}"
            )
            return

        row = raw.copy()
        self._check_layout(row)

        if self._last_obs is not None and row.shape == self._last_obs.shape:
            if np.allclose(row, self._last_obs, rtol=1e-12, atol=1e-12):
                return

        self._last_obs = row.copy()
        self._obs_buffer.append(row)
        self._rows_recorded += 1

        if self._chunk_size > 0:
            while len(self._obs_buffer) >= self._chunk_size:
                self._flush_chunk_to_disk(self._chunk_size)

        if self._rows_recorded % 50 == 0 or self._rows_recorded == self._max_samples:
            pending = len(self._obs_buffer)
            self.get_logger().info(
                f"Recorded {self._rows_recorded}/{self._max_samples} rows (buffer {pending})"
            )

        if self._rows_recorded >= self._max_samples:
            if self._chunk_size > 0:
                if self._obs_buffer:
                    self._flush_chunk_to_disk(len(self._obs_buffer))
            else:
                self._save_single_file_and_shutdown()
                return
            self._finalize_shutdown()

    def _flush_chunk_to_disk(self, n: int) -> None:
        assert n > 0 and n <= len(self._obs_buffer)
        obs = np.stack(self._obs_buffer[:n], axis=0)
        idx = self._file_index
        run_dir = self._run_subdir(idx)
        run_dir.mkdir(parents=True, exist_ok=True)

        def path_chunk(stem: str) -> Path:
            return self._npy_path_in_run(stem, idx)

        _save_obs_split(obs, path_chunk)
        del self._obs_buffer[:n]
        self._file_index += 1
        self.get_logger().info(
            f"Saved chunk {idx} ({n} rows) → {run_dir / 'position.npy'}, … (11 arrays in {run_dir.name}/)"
        )

    def _save_single_file_and_shutdown(self) -> None:
        obs = np.stack(self._obs_buffer, axis=0)
        idx = self._file_index
        run_dir = self._run_subdir(idx)
        run_dir.mkdir(parents=True, exist_ok=True)

        def path_run(stem: str) -> Path:
            return self._npy_path_in_run(stem, idx)

        _save_obs_split(obs, path_run)
        self._saved = True
        self.get_logger().info(f"Saved split arrays under {run_dir} (11 .npy files)")
        self.get_logger().info("Data collection complete. Shutting down node.")
        self.destroy_node()
        rclpy.shutdown()

    def _finalize_shutdown(self) -> None:
        self._saved = True
        self.get_logger().info("Data collection complete. Shutting down node.")
        self.destroy_node()
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = DataCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
