"""
Gain Corrections for Radiation-based Exploration (Eq. 19-22)

이 모듈은 논문 Section 4.2.3의 세 가지 gain correction을 구현합니다:
1. Observation Intensity Correction (OIC) - Eq. 19
2. Redundant Sampling Correction (RSC) - Eq. 20
3. Repeat Exploring Correction (REC) - Eq. 21

이 correction들은 exploration과 exploitation의 균형을 맞추는 핵심 메커니즘입니다.
"""
from __future__ import annotations

import numpy as np
from typing import List, Tuple
from dataclasses import dataclass

from .rrt_planner import RRTNode


@dataclass
class GainCorrectionParams:
    """
    Parameters for gain corrections (Eq. 19-21).

    논문 Section 4.2.3에서 사용되는 파라미터들.
    """
    # Observation Intensity Correction (OIC) - Eq. 19
    sigma_gain_src: float = 0.1  # Background value of OIC
    T_gain_src: float = 2.0  # Threshold for intensity ratio
    S_gain_src: float = 0.5  # Scale parameter of OIC

    # Redundant Sampling Correction (RSC) - Eq. 20
    sigma_rs: float = 0.2  # Slight offset for RSC
    S_rs: float = 0.1  # Scale parameter of RSC
    sampling_radius: float = 50.0  # Radius to count observations (pixels, ~2m)

    # Repeat Exploring Correction (REC) - Eq. 21
    S_rex: float = 50.0  # Scale parameter of REC (50 pixels = 2.0m)
    sigma_rex: float = 0.1  # Offset for REC


class GainCorrections:
    """
    Implements gain corrections for radiation-based exploration.

    논문 Section 4.2.3의 세 가지 correction:
    - OIC: Pseudo-source 탐색 방지
    - RSC: 특정 소스 과도 샘플링 방지
    - REC: 미탐색 영역 탐색 촉진
    """

    def __init__(self, params: GainCorrectionParams):
        """
        Initialize gain corrections.

        Args:
            params: Correction parameters
        """
        self.params = params

    def observation_intensity_correction(
        self,
        node: RRTNode,
        observations: List[Tuple[np.ndarray, float]],
        I_back: float
    ) -> float:
        """
        Observation Intensity Correction (Eq. 19).

        목적: Pseudo-source 탐색 방지
        - 예측된 소스 근처에 낮은 radiation → C_obs 낮음 → Gain 감소
        - 예측된 소스 근처에 높은 radiation → C_obs 높음 → Gain 증가

        Args:
            node: RRT node to evaluate
            observations: List of (position, intensity) tuples
            I_back: Background radiation intensity

        Returns:
            C_obs: Observation intensity correction factor (0 < C_obs ≤ 1)
        """
        if not observations:
            return 1.0

        node_pos = np.array([node.pose.x, node.pose.y])

        # Find nearest observation to this node
        distances = [np.linalg.norm(node_pos - obs_pos) for obs_pos, _ in observations]
        nearest_idx = np.argmin(distances)
        _, I_near = observations[nearest_idx]

        # Calculate intensity ratio (observed / background)
        # High ratio → real source nearby → high C_obs
        # Low ratio → pseudo-source or noise → low C_obs
        ratio = I_near / (I_back + 1e-10)

        # Sigmoid correction (Eq. 19)
        C_obs = (
            self.params.sigma_gain_src +
            (1.0 - self.params.sigma_gain_src) /
            (1.0 + np.exp((self.params.T_gain_src - ratio) / self.params.S_gain_src))
        )

        return C_obs

    def redundant_sampling_correction(
        self,
        node: RRTNode,
        sources: List[np.ndarray],
        observations: List[Tuple[np.ndarray, float]]
    ) -> float:
        """
        Redundant Sampling Correction (Eq. 20).

        목적: 특정 소스 과도 샘플링 방지
        - 특정 소스 주변에 관측 많음 → C_rs 낮음 → Gain 감소
        - 미탐색 소스 → C_rs 높음 → Gain 유지

        Args:
            node: RRT node to evaluate
            sources: List of source parameters [x, y, intensity]
            observations: List of (position, intensity) tuples

        Returns:
            C_rs: Redundant sampling correction factor
        """
        if not sources or not observations:
            return 1.0

        node_pos = np.array([node.pose.x, node.pose.y])

        # Find nearest source to this node
        distances = [np.linalg.norm(node_pos - src[:2]) for src in sources]
        d_near = min(distances)
        nearest_src_idx = np.argmin(distances)
        nearest_src = sources[nearest_src_idx]

        # Count observations near this source
        N_sam = 0
        for obs_pos, _ in observations:
            dist_to_src = np.linalg.norm(obs_pos - nearest_src[:2])
            if dist_to_src < self.params.sampling_radius:
                N_sam += 1

        # Correction factor (Eq. 20)
        # Many samples near source → low C_rs → reduce gain
        # Few samples → high C_rs → maintain gain
        exp_pos = np.exp(N_sam - self.params.S_rs * d_near)
        exp_neg = np.exp(self.params.S_rs * d_near - N_sam)

        C_rs = 1.0 + (self.params.sigma_rs - 1.0) * exp_pos / (exp_neg + exp_pos + 1e-10)

        return C_rs

    def repeat_exploring_correction(
        self,
        node: RRTNode,
        observations: List[Tuple[np.ndarray, float]]
    ) -> float:
        """
        Repeat Exploring Correction (Eq. 21).

        목적: 미탐색 영역 탐색 촉진
        - 방문한 영역 근처 → C_rex 낮음 → Gain 감소
        - 미방문 영역 → C_rex 높음 → Gain 유지

        Args:
            node: RRT node to evaluate
            observations: List of (position, intensity) tuples

        Returns:
            C_rex: Repeat exploring correction factor
        """
        if not observations:
            return 1.0

        node_pos = np.array([node.pose.x, node.pose.y])

        # Product of corrections from all observations (Eq. 21)
        # Close to visited area → small C_rex
        # Far from visited area → C_rex ≈ 1.0
        C_rex = 1.0
        for obs_pos, _ in observations:
            d_tb = np.linalg.norm(node_pos - obs_pos)

            # Each observation contributes a correction factor
            C_rex *= np.exp(-1.0 / ((d_tb / self.params.S_rex) + self.params.sigma_rex))

        return C_rex

    def apply_all_corrections(
        self,
        node: RRTNode,
        sources: List[np.ndarray],
        observations: List[Tuple[np.ndarray, float]],
        I_back: float
    ) -> Tuple[float, float, float]:
        """
        Apply all three corrections and return individual factors.

        이 메서드는 디버깅과 분석을 위해 각 correction을 개별적으로 반환합니다.

        Args:
            node: RRT node to evaluate
            sources: List of source parameters
            observations: List of observations
            I_back: Background radiation

        Returns:
            (C_obs, C_rs, C_rex): Individual correction factors
        """
        C_obs = self.observation_intensity_correction(node, observations, I_back)
        C_rs = self.redundant_sampling_correction(node, sources, observations)
        C_rex = self.repeat_exploring_correction(node, observations)

        return C_obs, C_rs, C_rex


def compute_corrected_gain(
    base_gain: float,
    C_obs: float,
    C_rs: float,
    C_rex: float
) -> float:
    """
    Compute final corrected gain (part of Eq. 22).

    Gain_corrected = base_gain × C_obs × C_rs × C_rex

    Args:
        base_gain: Base radiation gain (Gain_src × C_dist × C_rot)
        C_obs: Observation intensity correction
        C_rs: Redundant sampling correction
        C_rex: Repeat exploring correction

    Returns:
        Corrected gain value
    """
    return base_gain * C_obs * C_rs * C_rex
