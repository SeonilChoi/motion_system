#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Vector3
from motion_system_msgs.msg import MotorStatus
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


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


class DataCollectorNode(Node):
    def __init__(self) -> None:
        super().__init__("data_collector_node")
        # Number of saved (obs, action) rows; requires max_samples+1 command changes after init.
        self.declare_parameter("max_samples", 5000)
        # Rows per file when chunking (e.g. 1000 → …_1_obs.npy …_5_obs.npy for 5000 total). 0 = single file at end.
        self.declare_parameter("chunk_size", 1000)
        # First index in filenames: {start}_obs.npy, {start+1}_obs.npy, … (or {tag}_{start}_obs.npy).
        self.declare_parameter("file_index_start", 1)
        source_data_dir = Path("/home/csi/Documents/lab_ws/src/motion_system/ros2/test_pkg/data")
        default_output_dir = source_data_dir
        self.declare_parameter("output_dir", str(default_output_dir))
        # Optional prefix: names become {tag}_{idx}_obs.npy; empty → {idx}_obs.npy only.
        self.declare_parameter(
            "output_tag",
            "",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("initial_joint_positions", _SILVER_LAIN_HOME_JOINT_POSITIONS)
        self.declare_parameter("joy_cmd_topic", "joy_cmd")

        self._max_samples = int(self.get_parameter("max_samples").value)
        self._chunk_size = max(0, int(self.get_parameter("chunk_size").value))
        self._file_index = int(self.get_parameter("file_index_start").value)
        self._output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser().resolve()
        self._output_tag = str(self.get_parameter("output_tag").value).strip()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        init_joints = np.asarray(
            self.get_parameter("initial_joint_positions").value,
            dtype=np.float64,
        ).reshape(-1)
        if init_joints.size == 0:
            init_joints = np.asarray(_SILVER_LAIN_HOME_JOINT_POSITIONS, dtype=np.float64)
        self._latest_pose: Optional[np.ndarray] = None
        self._prev_motor_command: np.ndarray = init_joints.copy()
        # Command before _prev_motor_command (for finite difference of motor targets).
        self._prev_prev_motor_command: np.ndarray = init_joints.copy()
        # Pose at previous command change; None until first post-init change (warm-up for velocity).
        self._pose_at_last_cmd: Optional[np.ndarray] = None
        self._obs_buffer: list[np.ndarray] = []
        self._action_buffer: list[np.ndarray] = []
        self._condition_buffer: list[np.ndarray] = []
        # Latest joy_cmd (Vector3: goal x,y + gait progress z); snapshotted per recorded row.
        self._latest_condition = np.zeros(3, dtype=np.float64)
        self._rows_recorded = 0
        self._saved = False

        joy_topic = str(self.get_parameter("joy_cmd_topic").value)
        self._pose_sub = self.create_subscription(Pose, "pose", self.pose_callback, _qos())
        self._motor_command_sub = self.create_subscription(
            MotorStatus, "motor_command", self.motor_command_callback, _qos()
        )
        self._joy_cmd_sub = self.create_subscription(
            Vector3,
            joy_topic,
            self.joy_cmd_callback,
            _qos(),
        )

        self.get_logger().info(
            f"Target {self._max_samples} obs rows "
            f"(needs {self._max_samples + 1} motor command changes after init; "
            "first change only anchors pose for velocity). "
            "obs=[pose(6)+pose_vel(6)+prev_motor(M)+prev_motor_vel(M)], action=[current_motor_command], "
            "condition=[joy_cmd.x, joy_cmd.y, joy_cmd.z] per row"
        )
        if self._chunk_size > 0:
            n_files = (self._max_samples + self._chunk_size - 1) // self._chunk_size
            self.get_logger().info(
                f"Chunked save: {self._chunk_size} rows/file, indices {self._file_index}…"
                f"{self._file_index + n_files - 1} (~{n_files} files)"
            )
        else:
            self.get_logger().info("Single-file mode (chunk_size=0): one obs/action/condition at end")
        self.get_logger().info(
            f"prev motor targets initialized from Silver Lain home ({init_joints.size} joints); "
            "override with parameter initial_joint_positions if needed."
        )
        if self._output_tag:
            self.get_logger().info(
                f"output_tag={self._output_tag!r} → filenames like {self._output_tag}_<idx>_obs.npy"
            )
        self.get_logger().info(f"Subscribing to joy_cmd on {joy_topic!r} (geometry_msgs/Vector3)")

    def _npy_path_legacy(self, stem: str) -> Path:
        if self._output_tag:
            return self._output_dir / f"{self._output_tag}_{stem}.npy"
        return self._output_dir / f"{stem}.npy"

    def _npy_path_chunk(self, stem: str, file_idx: int) -> Path:
        if self._output_tag:
            return self._output_dir / f"{self._output_tag}_{file_idx}_{stem}.npy"
        return self._output_dir / f"{file_idx}_{stem}.npy"

    @staticmethod
    def _pose_to_numpy(msg: Pose) -> np.ndarray:
        return np.array(
            [
                msg.position.x,
                msg.position.y,
                msg.position.z,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _motor_command_to_numpy(msg: MotorStatus) -> np.ndarray:
        return np.array(msg.position, dtype=np.float64)

    def pose_callback(self, msg: Pose) -> None:
        self._latest_pose = self._pose_to_numpy(msg)

    def joy_cmd_callback(self, msg: Vector3) -> None:
        self._latest_condition[0] = msg.x
        self._latest_condition[1] = msg.y
        self._latest_condition[2] = msg.z

    def motor_command_callback(self, msg: MotorStatus) -> None:
        if self._saved:
            return

        current_motor_command = self._motor_command_to_numpy(msg)

        if self._latest_pose is None:
            return

        # Save only when command actually changes.
        if np.allclose(current_motor_command, self._prev_motor_command):
            return

        # One extra command step: anchor pose so the next step has pose_velocity = pose_now - pose_prev.
        if self._pose_at_last_cmd is None:
            self._pose_at_last_cmd = self._latest_pose.copy()
            self._prev_prev_motor_command = self._prev_motor_command.copy()
            self._prev_motor_command = current_motor_command.copy()
            self.get_logger().info(
                "Pose anchor set for velocity; next command change will start recording obs rows."
            )
            return

        pose_velocity = self._latest_pose - self._pose_at_last_cmd
        prev_motor_velocity = self._prev_motor_command - self._prev_prev_motor_command
        obs = np.concatenate(
            [
                self._latest_pose,
                pose_velocity,
                self._prev_motor_command,
                prev_motor_velocity,
            ],
            axis=0,
        )
        action = current_motor_command.copy()

        self._obs_buffer.append(obs)
        self._action_buffer.append(action)
        self._condition_buffer.append(self._latest_condition.copy())
        self._pose_at_last_cmd = self._latest_pose.copy()
        self._prev_prev_motor_command = self._prev_motor_command.copy()
        self._prev_motor_command = current_motor_command.copy()

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
        action = np.stack(self._action_buffer[:n], axis=0)
        condition = np.stack(self._condition_buffer[:n], axis=0)
        idx = self._file_index
        obs_path = self._npy_path_chunk("obs", idx)
        action_path = self._npy_path_chunk("action", idx)
        condition_path = self._npy_path_chunk("condition", idx)
        np.save(obs_path, obs)
        np.save(action_path, action)
        np.save(condition_path, condition)
        del self._obs_buffer[:n]
        del self._action_buffer[:n]
        del self._condition_buffer[:n]
        self._file_index += 1
        self.get_logger().info(f"Saved chunk {idx} ({n} rows) → {obs_path.name}, …")

    def _save_single_file_and_shutdown(self) -> None:
        obs = np.stack(self._obs_buffer, axis=0)
        action = np.stack(self._action_buffer, axis=0)
        condition = np.stack(self._condition_buffer, axis=0)
        obs_path = self._npy_path_legacy("obs")
        action_path = self._npy_path_legacy("action")
        condition_path = self._npy_path_legacy("condition")
        np.save(obs_path, obs)
        np.save(action_path, action)
        np.save(condition_path, condition)
        self._saved = True
        self.get_logger().info(f"Saved obs to {obs_path}")
        self.get_logger().info(f"Saved action to {action_path}")
        self.get_logger().info(f"Saved condition to {condition_path}")
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

