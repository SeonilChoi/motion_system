#!/usr/bin/env python3

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

PI = np.pi
R30 = PI / 6
R60 = PI / 3
R90 = PI / 2


class KinematicsSolver:
    def __init__(self, link_list: list[float]) -> None:
        self._link_list = link_list

    @staticmethod
    def _get_pose_transformation_matrix(pose: np.ndarray) -> np.ndarray:
        x, y, z, roll, pitch, yaw = pose

        rz = np.array(
            [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]],
            dtype=np.float64,
        )
        ry = np.array(
            [[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]],
            dtype=np.float64,
        )
        rx = np.array(
            [[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]],
            dtype=np.float64,
        )

        rotation = rz @ ry @ rx
        position = np.array([x, y, z], dtype=np.float64).reshape(3, 1)
        return np.r_[np.c_[rotation, position], [[0, 0, 0, 1]]]

    @staticmethod
    def _invert_pose_transformation_matrix(pose: np.ndarray) -> np.ndarray:
        transform = KinematicsSolver._get_pose_transformation_matrix(pose)
        rotation = transform[:3, :3]
        position = transform[:3, -1]
        return np.r_[np.c_[rotation.T, -rotation.T @ position], [[0, 0, 0, 1]]]

    @staticmethod
    def _get_transformation_matrix(param: np.ndarray) -> np.ndarray:
        a, alpha, d, theta = param
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array(
            [[ct, -st, 0, a], [st * ca, ct * ca, -sa, -sa * d], [st * sa, ct * sa, ca, ca * d], [0, 0, 0, 1]],
            dtype=np.float64,
        )

    @staticmethod
    def _forward_kinematics(dh_params: np.ndarray) -> np.ndarray:
        transform = np.eye(4, dtype=np.float64)
        for param in dh_params:
            transform = transform @ KinematicsSolver._get_transformation_matrix(param)
        return transform[:3, -1].reshape(3)


class SilverLainSolver(KinematicsSolver):
    def _update_dh_params(self, index: int, positions: np.ndarray) -> np.ndarray:
        l0, l1, l2, l3 = self._link_list
        q1, q2, q3 = positions[index * 3 : index * 3 + 3]

        if index < 3:
            dh_params = np.array(
                [
                    [0, 0, 0, -R30 - R60 * index],
                    [l0, 0, 0, q1],
                    [0, 0, l1, 0],
                    [0, R90, 0, R90 + q2],
                    [l2, PI, 0, q3],
                    [l3, 0, 0, 0],
                ],
                dtype=np.float64,
            )
        else:
            dh_params = np.array(
                [
                    [0, 0, 0, R30 + R60 * (5 - index)],
                    [l0, 0, 0, q1],
                    [0, 0, l1, 0],
                    [0, -R90, 0, -R90 + q2],
                    [l2, -PI, 0, q3],
                    [l3, 0, 0, 0],
                ],
                dtype=np.float64,
            )
        return dh_params

    def forward_with_pose(self, pose: np.ndarray, positions: np.ndarray) -> np.ndarray:
        points_body = np.zeros((6, 3), dtype=np.float64)
        for i in range(6):
            points_body[i] = self._forward_kinematics(self._update_dh_params(i, positions))

        transform = self._get_pose_transformation_matrix(pose)
        points_h = np.c_[points_body, np.ones((6, 1))]
        points_world = (transform @ points_h.T).T
        return points_world[:, :3]

    def _to_leg_frame(self, index: int, point: np.ndarray) -> np.ndarray:
        transform = self._get_transformation_matrix(self._update_dh_params(index, np.zeros(18))[0])
        rotation = transform[:3, :3]
        position = transform[:3, -1].reshape(3, 1)
        inv = np.r_[np.c_[rotation.T, -rotation.T @ position], [[0, 0, 0, 1]]]
        point_h = np.r_[point, 1.0].reshape(4, 1)
        return (inv @ point_h).reshape(-1)[:3]

    def _inverse_kinematics(self, index: int, point: np.ndarray) -> np.ndarray:
        l0, l1, l2, l3 = self._link_list
        px, py, pz = point
        px -= l0

        th1 = np.arctan2(py, px)
        qx = px / np.cos(th1)
        qz = pz - l1

        c3 = np.clip((qx**2 + qz**2 - l2**2 - l3**2) / (2 * l2 * l3), -1.0, 1.0)
        s3 = np.sqrt(1.0 - c3**2)

        th3 = np.arctan2(s3, c3)
        th3 = -th3 if index < 3 else th3

        m = l2 + l3 * c3
        n = l3 * np.sin(th3) if index < 3 else l3 * np.sin(-th3)
        th2 = np.arctan2(qz, qx) - np.arctan2(n, m)
        th2 = th2 - R90 if index < 3 else -th2 + R90

        return np.array([th1, th2, -th3], dtype=np.float64)

    def inverse_with_pose(self, pose: np.ndarray, points_world: np.ndarray) -> np.ndarray:
        transform_inv = self._invert_pose_transformation_matrix(pose)
        points_h = np.c_[points_world, np.ones((6, 1))]
        points_body = (transform_inv @ points_h.T).T[:, :3]

        positions = np.zeros(18, dtype=np.float64)
        for i in range(6):
            point_leg = self._to_leg_frame(i, points_body[i])
            positions[i * 3 : i * 3 + 3] = self._inverse_kinematics(i, point_leg)
        return positions


def leg_chain_world(solver: SilverLainSolver, pose: np.ndarray, positions: np.ndarray, leg_idx: int) -> np.ndarray:
    dh_params = solver._update_dh_params(leg_idx, positions)
    transform = solver._get_pose_transformation_matrix(pose)

    points = [transform[:3, -1].copy()]
    for param in dh_params:
        transform = transform @ solver._get_transformation_matrix(param)
        points.append(transform[:3, -1].copy())

    return np.vstack([points[0], points[2], points[4], points[5], points[6]])


def main() -> None:
    # From silver_lain.yaml lines 12-17 (flattened to 18 joints)
    home_joint_positions = np.array(
        [
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
        ],
        dtype=np.float64,
    )
    pose = np.zeros(6, dtype=np.float64)
    link_list = [0.32, 0.0, 0.615, 1.25]

    solver = SilverLainSolver(link_list)
    feet = solver.forward_with_pose(pose, home_joint_positions)
    solved_joint_positions = solver.inverse_with_pose(pose, feet)
    feet_reconstructed = solver.forward_with_pose(pose, solved_joint_positions)

    joint_error = np.max(np.abs(home_joint_positions - solved_joint_positions))
    foot_error = np.max(np.abs(feet - feet_reconstructed))
    print(f"max joint error (rad): {joint_error:.6e}")
    print(f"max foot point error (m): {foot_error:.6e}")

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("FK -> IK -> FK Visualization")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.grid(True)
    ax.set_box_aspect([1, 1, 0.6])

    body_vertices = []
    for i in range(3):
        body_vertices.append([link_list[0] * np.cos(-R30 - R60 * i), link_list[0] * np.sin(-R30 - R60 * i), 0.0])
    for i in range(3, 6):
        body_vertices.append(
            [link_list[0] * np.cos(R30 + R60 * (5 - i)), link_list[0] * np.sin(R30 + R60 * (5 - i)), 0.0]
        )
    body_vertices = np.array(body_vertices, dtype=np.float64)
    body_loop = np.vstack([body_vertices, body_vertices[0]])
    ax.plot(body_loop[:, 0], body_loop[:, 1], body_loop[:, 2], "k-", linewidth=2, label="body")

    for leg_idx in range(6):
        chain_fk = leg_chain_world(solver, pose, home_joint_positions, leg_idx)
        chain_ik = leg_chain_world(solver, pose, solved_joint_positions, leg_idx)
        ax.plot(chain_fk[:, 0], chain_fk[:, 1], chain_fk[:, 2], linewidth=2, color="tab:blue")
        ax.plot(chain_ik[:, 0], chain_ik[:, 1], chain_ik[:, 2], "--", linewidth=1.8, color="tab:orange")
    ax.scatter(feet[:, 0], feet[:, 1], feet[:, 2], c="tab:blue", s=30, label="feet (from initial joints)")
    ax.scatter(
        feet_reconstructed[:, 0],
        feet_reconstructed[:, 1],
        feet_reconstructed[:, 2],
        c="tab:orange",
        s=22,
        label="feet (after IK)",
    )

    points = np.vstack([body_vertices, feet, feet_reconstructed])
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) * 0.5
    radius = max((maxs - mins).max() * 0.6, 0.5)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(min(mins[2] - 0.2, -0.2), maxs[2] + 0.2)

    ax.legend(loc="upper right")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
