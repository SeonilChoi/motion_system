from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class KinematicsSolver(ABC):
    def __init__(self, link_list: list[float]) -> None:
        self._link_list = link_list


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
        T_list = np.zeros((dh_params.shape[0], 4, 4))
        for i, param in enumerate(dh_params):
            T = T @ KinematicsSolver._get_transformation_matrix(param)
            T_list[i] = T

        return T_list


    @abstractmethod
    def forward(self, positions: np.ndarray) -> np.ndarray:
        ...

    @abstractmethod
    def inverse(self, points: np.ndarray) -> np.ndarray:
        ...