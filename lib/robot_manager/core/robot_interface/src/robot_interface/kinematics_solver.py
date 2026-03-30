from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class KinematicsSolver(ABC):
    def __init__(self, link_list: list[float]) -> None:
        self._link_list = link_list

    @staticmethod
    def _get_pose_transformation_matrix(pose: np.ndarray) -> np.ndarray:
        x, y, z, R, P, Y = pose

        Rz = np.array([
            [np.cos(Y), -np.sin(Y), 0],
            [np.sin(Y),  np.cos(Y), 0],
            [        0,          0, 1]
        ])

        Ry = np.array([
            [ np.cos(P), 0, np.sin(P)],
            [         0, 1,         0],
            [-np.sin(P), 0, np.cos(P)]
        ])

        Rx = np.array([
            [1,         0,          0],
            [0, np.cos(R), -np.sin(R)],
            [0, np.sin(R),  np.cos(R)]
        ])

        R = Rz @ Ry @ Rx
        p = np.array([x, y, z]).reshape(3, 1)
        return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]

    @staticmethod
    def _invert_pose_transformation_matrix(pose: np.ndarray) -> np.ndarray:
        T_wb = KinematicsSolver._get_pose_transformation_matrix(pose)
        R = T_wb[:3, :3]
        p = T_wb[:3, -1]
        return np.r_[np.c_[R.T, -R.T @ p], [[0, 0, 0, 1]]]

    @staticmethod
    def _get_transformation_matrix(param: np.ndarray) -> np.ndarray:
        a, al, d, th = param
        ct, st = np.cos(th), np.sin(th)
        ca, sa = np.cos(al), np.sin(al)
        return np.array([
            [   ct,   -st,   0,     a],
            [st*ca, ct*ca, -sa, -sa*d],
            [st*sa, ct*sa,  ca,  ca*d],
            [    0,     0,   0,     1]
        ], dtype=np.float64)

    @staticmethod
    def _forward_kinematics(dh_params: np.ndarray) -> np.ndarray:
        T = np.eye(4)
        for param in dh_params:
            T = T @ KinematicsSolver._get_transformation_matrix(param)

        return T[:3, -1].reshape(3)


    @abstractmethod
    def _forward(self, positions: np.ndarray) -> np.ndarray:
        ...

    @abstractmethod
    def _inverse(self, points: np.ndarray) -> np.ndarray:
        ...

    
    @abstractmethod
    def forward_with_pose(self, pose: np.ndarray, positions: np.ndarray) -> np.ndarray:
        ...

    @abstractmethod
    def inverse_with_pose(self, pose: np.ndarray, points: np.ndarray) -> np.ndarray:
        ...