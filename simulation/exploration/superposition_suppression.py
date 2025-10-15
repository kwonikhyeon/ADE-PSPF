"""
Superposition Suppression for Multi-source Radiation Gain (Eq. 13-15)

이 모듈은 논문 Section 4.2.1의 superposition suppression을 구현합니다.
목적: 가까운 여러 소스들의 common neighborhood에서 abnormal gain 제거

논문 Fig. 4 (abnormal gain) → Fig. 5 (suppressed gain)
"""
from __future__ import annotations

import numpy as np
from typing import List
from dataclasses import dataclass


@dataclass
class SuppressionParams:
    """
    Parameters for superposition suppression (Eq. 13-14).

    논문 Section 4.2.1에서 사용되는 파라미터들.
    """
    # Suppression Factor (Eq. 13)
    T_dist_sup: float = 50.0  # Threshold distance for suppression (pixels, ~2m)
    S_dist_sup: float = 10.0  # Scale parameter for suppression

    # Suppression Range Coefficient (Eq. 14)
    sigma_dist_r: float = 0.1  # Background value
    T_dist_r: float = 100.0  # Distance threshold between sources (pixels, ~4m)
    S_dist_r: float = 20.0  # Scale parameter for range


class SuperpositionSuppression:
    """
    Implements superposition suppression for multi-source radiation gain.

    논문 Section 4.2.1:
    - Eq. 13: Suppression factor for each source pair
    - Eq. 14: Suppression range coefficient
    - Eq. 15: Final multi-source gain with suppression

    목적: 가까운 소스들의 common neighborhood에서 abnormal gain 제거
    """

    def __init__(self, params: SuppressionParams):
        """
        Initialize superposition suppression.

        Args:
            params: Suppression parameters
        """
        self.params = params

    def suppression_range_coefficient(self, d_r_kj: float) -> float:
        """
        Suppression Range Coefficient (Eq. 14).

        소스 간 거리가 가까우면 억제 강도 증가
        소스 간 거리가 멀면 억제 불필요

        Args:
            d_r_kj: Distance between kth and jth sources (pixels)

        Returns:
            C_dist_sup: Suppression range coefficient
        """
        # Eq. 14: Sigmoid function
        C_dist_sup = (
            self.params.sigma_dist_r +
            (1.0 - self.params.sigma_dist_r) /
            (1.0 + np.exp((d_r_kj - self.params.T_dist_r) / self.params.S_dist_r))
        )
        return C_dist_sup

    def suppression_factor(
        self,
        node_pos: np.ndarray,
        source_k: np.ndarray,
        sources: List[np.ndarray],
        k: int
    ) -> float:
        """
        Suppression Factor for kth source (Eq. 13).

        두 소스가 가까우면 중간 지점 근처의 gain 억제

        Args:
            node_pos: Node position [x, y]
            source_k: kth source [x, y, intensity]
            sources: All sources
            k: Index of current source

        Returns:
            F_k: Suppression factor for kth source
        """
        if len(sources) <= 1:
            # Single source: no suppression needed
            return 1.0

        F_k = 0.0

        for j, source_j in enumerate(sources):
            if j == k:
                continue

            # Distance from node to midpoint of two sources
            midpoint = (source_k[:2] + source_j[:2]) / 2.0
            d_kj_t = np.linalg.norm(node_pos - midpoint)

            # Distance between two sources
            d_r_kj = np.linalg.norm(source_k[:2] - source_j[:2])

            # Suppression range coefficient (Eq. 14)
            C_dist_sup = self.suppression_range_coefficient(d_r_kj)

            # Suppression factor (Eq. 13)
            # 두 소스의 중간 지점 근처: F_k 작음 → gain 억제
            # 중간 지점에서 멀리: F_k 큼 → gain 유지
            F_k += (
                1.0 - C_dist_sup +
                C_dist_sup / (1.0 + np.exp(
                    (self.params.T_dist_sup - d_kj_t) / self.params.S_dist_sup
                ))
            )

        return F_k

    def compute_gain_with_suppression(
        self,
        node_pos: np.ndarray,
        sources: List[np.ndarray],
        single_source_gain_func
    ) -> float:
        """
        Multi-source Radiation Gain with Suppression (Eq. 15).

        G_src(n_t) = Σ G̃_src^(t,k)(n_t) × F_src^(t,k)(n_t)

        Args:
            node_pos: Node position [x, y]
            sources: List of sources [x, y, intensity]
            single_source_gain_func: Function to compute single source gain
                                     signature: func(node_pos, source_pos) -> float

        Returns:
            Total gain with suppression
        """
        if len(sources) == 0:
            return 0.0

        if len(sources) == 1:
            # Single source: no suppression needed
            return single_source_gain_func(node_pos, sources[0][:2])

        total_gain = 0.0

        for k, source_k in enumerate(sources):
            # Single source gain (Eq. 11)
            gain_k = single_source_gain_func(node_pos, source_k[:2])

            # Suppression factor (Eq. 13)
            F_k = self.suppression_factor(node_pos, source_k, sources, k)

            # Eq. 15
            total_gain += gain_k * F_k

        return total_gain


def create_default_suppression() -> SuperpositionSuppression:
    """
    Create suppression with default parameters from paper.

    Returns:
        SuperpositionSuppression instance with paper parameters
    """
    params = SuppressionParams()
    return SuperpositionSuppression(params)
