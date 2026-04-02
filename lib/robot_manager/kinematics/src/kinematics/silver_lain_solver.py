from __future__ import annotations

from robot_interface.kinematics_solver import KinematicsSolver

import numpy as np

PI = np.pi
R30 = PI / 6
R60 = PI / 3
R90 = PI / 2

class SilverLainSolver(KinematicsSolver):
    def __init__(self, link_list: list[float]) -> None:
        super().__init__(link_list)


    def _update_dh_params(self, index: int, positions: np.ndarray) -> np.ndarray:
        l0, l1, l2, l3 = self._link_list
        q1, q2, q3 = positions[index*3:index*3+3]
        
        if index < 3:
            dh_params = np.array([
                [ 0,   0,  0, -R30 - R60 * index],
                [l0,   0,  0,                 q1],
                [ 0,   0, l1,                  0],
                [ 0, R90,  0,           R90 + q2],
                [l2,  PI,  0,                 q3],
                [l3,   0,  0,                  0]
            ])
        else:
            dh_params = np.array([
                [ 0,    0,  0, R30 + R60 * (5 - index)],
                [l0,    0,  0,                      q1],
                [ 0,    0, l1,                       0],
                [ 0, -R90,  0,               -R90 + q2],
                [l2,  -PI,  0,                      q3],
                [l3,    0,  0,                       0]
            ])
        
        return dh_params

    def _to_leg_frame(self, index: int, point: np.ndarray) -> np.ndarray:
        T01 = self._get_transformation_matrix(
            self._update_dh_params(index, np.zeros(18))[0]
        )
        R01 = T01[:3, :3]
        p01 = T01[:3, -1].reshape(3, 1)

        T10 = np.r_[np.c_[R01.T, -R01.T @ p01], [[0, 0, 0, 1]]]
        
        point = np.r_[point, 1].reshape(4, 1)

        return (T10 @ point).reshape(-1)[:3]

    def _inverse_kinematics(self, index: int, point: np.ndarray) -> np.ndarray:
        l0, l1, l2, l3 = self._link_list

        px, py, pz = point
        px -= l0

        th1 = np.arctan2(py, px)

        qx = px / np.cos(th1)
        qz = pz - l1

        c3 = np.clip((qx**2 + qz**2 - l2**2 - l3**2) / (2 * l2 * l3), -1.0, 1.0)
        s3 = np.sqrt(1 - c3**2)

        th3 = np.arctan2(s3, c3)
        th3 = -th3 if index < 3 else th3

        m = l2 + l3 * c3
        n = l3 * np.sin(th3) if index < 3 else l3 * np.sin(-th3)

        th2 = np.arctan2(qz, qx) - np.arctan2(n, m)
        th2 = th2 - R90 if index < 3 else -th2 + R90

        return np.array([th1, th2, -th3], dtype=np.float64)

    def _forward(self, positions: np.ndarray) -> np.ndarray:
        points = np.zeros((6, 3))

        for i in range(6):
            dh_params = self._update_dh_params(i, positions)
            points[i] = self._forward_kinematics(dh_params)

        return points

    def _inverse(self, points: np.ndarray) -> np.ndarray:
        positions = np.zeros(18) # Joint positions

        for i in range(6):
            p = self._to_leg_frame(i, points[i])
            q = self._inverse_kinematics(i, p)

            positions[i*3:i*3+3] = q
        
        return positions


    def forward_with_pose(self, pose: np.ndarray, positions: np.ndarray) -> np.ndarray:
        p_b = self._forward(positions)
        
        p_w = np.dot(
            self._get_pose_transformation_matrix(pose),
            np.c_[p_b, np.ones((6, 1))].T
        ).T
        
        return p_w[:, :3]

    def inverse_with_pose(self, pose: np.ndarray, points: np.ndarray) -> np.ndarray:
        p_b = np.dot(
            self._invert_pose_transformation_matrix(pose),
            np.c_[points, np.ones((6, 1))].T
        ).T
        
        return self._inverse(p_b[:, :3])
        
