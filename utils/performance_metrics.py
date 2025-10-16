"""
Performance Metrics for Integration Tests.

이 모듈은 논문 Table 6의 지표(SR, RFC, LE, IE)를 재현하기 위해 사용한다.
테스트 코드와 동일한 구현을 utils/로 옮겨 재사용성을 확보했다.
"""
from __future__ import annotations

import numpy as np
from typing import List, Tuple
from scipy.optimize import linear_sum_assignment

# 단위 변환 상수 (256px = 10m)
PIXELS_PER_METER = 256.0 / 10.0
METERS_PER_PIXEL = 10.0 / 256.0


def pixels_to_meters(pixels: float) -> float:
    """Convert pixels to meters."""
    return pixels * METERS_PER_PIXEL


def meters_to_pixels(meters: float) -> float:
    """Convert meters to pixels."""
    return meters * PIXELS_PER_METER


def match_sources(
    true_sources: List[np.ndarray],
    estimated_sources: List[np.ndarray],
    distance_threshold_meters: float = 0.5
) -> List[Tuple[int, int]]:
    """
    Match estimated sources to ground truth sources (Hungarian algorithm).

    Args:
        true_sources: [[x, y, intensity], ...] ground truth
        estimated_sources: [[x, y, intensity], ...] predictions
        distance_threshold_meters: 허용 오차 (논문 0.5m)

    Returns:
        매칭된 (true_idx, est_idx) 리스트
    """
    if len(true_sources) == 0 or len(estimated_sources) == 0:
        return []

    n_true = len(true_sources)
    n_est = len(estimated_sources)
    distance_matrix = np.zeros((n_true, n_est))

    for i, true_src in enumerate(true_sources):
        for j, est_src in enumerate(estimated_sources):
            dist_px = np.linalg.norm(true_src[:2] - est_src[:2])
            distance_matrix[i, j] = pixels_to_meters(dist_px)

    row_indices, col_indices = linear_sum_assignment(distance_matrix)

    matches: List[Tuple[int, int]] = []
    for true_idx, est_idx in zip(row_indices, col_indices, strict=False):
        if distance_matrix[true_idx, est_idx] <= distance_threshold_meters:
            matches.append((int(true_idx), int(est_idx)))
    return matches


def compute_success_rate(
    true_sources: List[np.ndarray],
    estimated_sources: List[np.ndarray],
    distance_threshold_meters: float = 0.5
) -> float:
    """Success Rate = 감지된 소스 / 전체 소스 × 100[%]."""
    if len(true_sources) == 0:
        return 100.0 if len(estimated_sources) == 0 else 0.0

    matched = match_sources(true_sources, estimated_sources, distance_threshold_meters)
    return (len(matched) / len(true_sources)) * 100.0


def compute_localization_error(
    true_sources: List[np.ndarray],
    estimated_sources: List[np.ndarray],
    matched_pairs: List[Tuple[int, int]]
) -> Tuple[float, float]:
    """Localization Error (평균, 표준편차) [m]."""
    if len(matched_pairs) == 0:
        return float("inf"), 0.0

    errors = []
    for true_idx, est_idx in matched_pairs:
        dist_px = np.linalg.norm(
            true_sources[true_idx][:2] - estimated_sources[est_idx][:2]
        )
        errors.append(pixels_to_meters(dist_px))

    return float(np.mean(errors)), float(np.std(errors))


def compute_intensity_error(
    true_sources: List[np.ndarray],
    estimated_sources: List[np.ndarray],
    matched_pairs: List[Tuple[int, int]]
) -> Tuple[float, float]:
    """Intensity Error (평균, 표준편차)."""
    if len(matched_pairs) == 0:
        return float("inf"), 0.0

    errors = [
        abs(true_sources[t][2] - estimated_sources[e][2])
        for t, e in matched_pairs
    ]
    return float(np.mean(errors)), float(np.std(errors))


def compute_all_metrics(
    true_sources: List[np.ndarray],
    estimated_sources: List[np.ndarray],
    rfc: float,
    distance_threshold_meters: float = 0.5
) -> dict:
    """논문 Table 6에 맞춘 종합 지표 계산."""
    matched = match_sources(true_sources, estimated_sources, distance_threshold_meters)
    success_rate = compute_success_rate(true_sources, estimated_sources, distance_threshold_meters)
    le_mean, le_std = compute_localization_error(true_sources, estimated_sources, matched)
    ie_mean, ie_std = compute_intensity_error(true_sources, estimated_sources, matched)

    return {
        "n_true_sources": len(true_sources),
        "n_estimated_sources": len(estimated_sources),
        "success_rate": success_rate,
        "rfc": rfc,
        "localization_error_mean": le_mean,
        "localization_error_std": le_std,
        "intensity_error_mean": ie_mean,
        "intensity_error_std": ie_std,
        "matched_pairs": matched,
    }


def format_metrics(metrics: dict) -> str:
    """사전 포맷."""
    lines = [
        "Performance Metrics",
        f"  • SR: {metrics['success_rate']:.1f}%",
        f"  • RFC: {metrics['rfc']:.4f}",
    ]
    if np.isfinite(metrics["localization_error_mean"]):
        lines.append(
            f"  • LE: {metrics['localization_error_mean']:.3f} ± "
            f"{metrics['localization_error_std']:.3f} m"
        )
    if np.isfinite(metrics["intensity_error_mean"]):
        lines.append(
            f"  • IE: {metrics['intensity_error_mean']:.2f} ± "
            f"{metrics['intensity_error_std']:.2f}"
        )
    return "\n".join(lines)


__all__ = [
    "PIXELS_PER_METER",
    "METERS_PER_PIXEL",
    "pixels_to_meters",
    "meters_to_pixels",
    "match_sources",
    "compute_success_rate",
    "compute_localization_error",
    "compute_intensity_error",
    "compute_all_metrics",
    "format_metrics",
]
