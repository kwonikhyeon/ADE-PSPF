"""
Performance Metrics for Integration Tests

논문 Table 6 성능 지표 계산:
- SR (Success Rate): 소스 탐지 성공률
- RFC (Radiation Field Confidence): 방사선 필드 신뢰도
- LE (Localization Error): 위치 추정 오차
- IE (Intensity Error): 강도 추정 오차

환경 설정:
- Grid: 256px × 256px = 10m × 10m
- 1 pixel = 0.0390625m ≈ 3.9cm
"""
import numpy as np
from typing import List, Tuple, Optional
from scipy.optimize import linear_sum_assignment


# Unit conversion constants
PIXELS_PER_METER = 256.0 / 10.0  # 25.6 pixels/meter
METERS_PER_PIXEL = 10.0 / 256.0  # 0.0390625 m/pixel


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
    Match estimated sources to ground truth sources using Hungarian algorithm.

    Args:
        true_sources: List of [x, y, intensity] in pixels
        estimated_sources: List of [x, y, intensity] in pixels
        distance_threshold_meters: Maximum distance for valid match (meters)

    Returns:
        List of (true_idx, estimated_idx) pairs
    """
    if len(true_sources) == 0 or len(estimated_sources) == 0:
        return []

    # Compute distance matrix (in meters)
    n_true = len(true_sources)
    n_est = len(estimated_sources)
    distance_matrix = np.zeros((n_true, n_est))

    for i, true_src in enumerate(true_sources):
        for j, est_src in enumerate(estimated_sources):
            # Distance in pixels
            dist_px = np.linalg.norm(true_src[:2] - est_src[:2])
            # Convert to meters
            distance_matrix[i, j] = pixels_to_meters(dist_px)

    # Hungarian algorithm for optimal assignment
    row_indices, col_indices = linear_sum_assignment(distance_matrix)

    # Filter by distance threshold
    matched_pairs = []
    for true_idx, est_idx in zip(row_indices, col_indices):
        if distance_matrix[true_idx, est_idx] <= distance_threshold_meters:
            matched_pairs.append((int(true_idx), int(est_idx)))

    return matched_pairs


def compute_success_rate(
    true_sources: List[np.ndarray],
    estimated_sources: List[np.ndarray],
    distance_threshold_meters: float = 0.5
) -> float:
    """
    Compute Success Rate (SR).

    SR = (Number of detected sources) / (Total number of sources) × 100%

    A source is detected if an estimated source is within distance_threshold.

    Args:
        true_sources: List of [x, y, intensity] in pixels
        estimated_sources: List of [x, y, intensity] in pixels
        distance_threshold_meters: Detection threshold (meters)

    Returns:
        Success rate as percentage (0-100)
    """
    if len(true_sources) == 0:
        return 100.0 if len(estimated_sources) == 0 else 0.0

    matched_pairs = match_sources(true_sources, estimated_sources, distance_threshold_meters)
    n_detected = len(matched_pairs)
    n_total = len(true_sources)

    return (n_detected / n_total) * 100.0


def compute_localization_error(
    true_sources: List[np.ndarray],
    estimated_sources: List[np.ndarray],
    matched_pairs: List[Tuple[int, int]]
) -> Tuple[float, float]:
    """
    Compute Localization Error (LE).

    LE = mean and std of Euclidean distances between matched pairs.

    Args:
        true_sources: List of [x, y, intensity] in pixels
        estimated_sources: List of [x, y, intensity] in pixels
        matched_pairs: List of (true_idx, est_idx) from match_sources()

    Returns:
        (mean_error, std_error) in meters
    """
    if len(matched_pairs) == 0:
        return float('inf'), 0.0

    errors_meters = []
    for true_idx, est_idx in matched_pairs:
        true_pos = true_sources[true_idx][:2]
        est_pos = estimated_sources[est_idx][:2]

        # Distance in pixels
        dist_px = np.linalg.norm(true_pos - est_pos)
        # Convert to meters
        dist_m = pixels_to_meters(dist_px)
        errors_meters.append(dist_m)

    mean_error = float(np.mean(errors_meters))
    std_error = float(np.std(errors_meters))

    return mean_error, std_error


def compute_intensity_error(
    true_sources: List[np.ndarray],
    estimated_sources: List[np.ndarray],
    matched_pairs: List[Tuple[int, int]]
) -> Tuple[float, float]:
    """
    Compute Intensity Error (IE).

    IE = mean and std of absolute intensity differences.

    Args:
        true_sources: List of [x, y, intensity] in pixels
        estimated_sources: List of [x, y, intensity] in pixels
        matched_pairs: List of (true_idx, est_idx) from match_sources()

    Returns:
        (mean_error, std_error) in intensity units
    """
    if len(matched_pairs) == 0:
        return float('inf'), 0.0

    errors = []
    for true_idx, est_idx in matched_pairs:
        true_intensity = true_sources[true_idx][2]
        est_intensity = estimated_sources[est_idx][2]

        error = abs(true_intensity - est_intensity)
        errors.append(error)

    mean_error = float(np.mean(errors))
    std_error = float(np.std(errors))

    return mean_error, std_error


def compute_all_metrics(
    true_sources: List[np.ndarray],
    estimated_sources: List[np.ndarray],
    rfc: float,
    distance_threshold_meters: float = 0.5
) -> dict:
    """
    Compute all performance metrics.

    Args:
        true_sources: List of [x, y, intensity] in pixels
        estimated_sources: List of [x, y, intensity] in pixels
        rfc: Radiation Field Confidence
        distance_threshold_meters: Detection threshold

    Returns:
        Dictionary with all metrics
    """
    # Match sources
    matched_pairs = match_sources(true_sources, estimated_sources, distance_threshold_meters)

    # Compute metrics
    sr = compute_success_rate(true_sources, estimated_sources, distance_threshold_meters)
    le_mean, le_std = compute_localization_error(true_sources, estimated_sources, matched_pairs)
    ie_mean, ie_std = compute_intensity_error(true_sources, estimated_sources, matched_pairs)

    return {
        'success_rate': sr,
        'rfc': rfc,
        'localization_error_mean': le_mean,
        'localization_error_std': le_std,
        'intensity_error_mean': ie_mean,
        'intensity_error_std': ie_std,
        'n_true_sources': len(true_sources),
        'n_estimated_sources': len(estimated_sources),
        'n_matched': len(matched_pairs),
        'matched_pairs': matched_pairs
    }


def format_metrics(metrics: dict) -> str:
    """
    Format metrics for display.

    Args:
        metrics: Dictionary from compute_all_metrics()

    Returns:
        Formatted string
    """
    lines = [
        "="*70,
        "Performance Metrics",
        "="*70,
        f"Success Rate (SR):          {metrics['success_rate']:.1f}%",
        f"RFC:                        {metrics['rfc']:.4f}",
        f"Localization Error (LE):    {metrics['localization_error_mean']:.3f} ± {metrics['localization_error_std']:.3f} m",
        f"Intensity Error (IE):       {metrics['intensity_error_mean']:.2f} ± {metrics['intensity_error_std']:.2f}",
        "",
        f"Sources (true/estimated):   {metrics['n_true_sources']}/{metrics['n_estimated_sources']}",
        f"Matched sources:            {metrics['n_matched']}",
        "="*70
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    print("Testing performance metrics...\n")

    # Test data (in pixels)
    true_sources = [
        np.array([100.0, 100.0, 50.0]),
        np.array([150.0, 150.0, 60.0]),
        np.array([200.0, 100.0, 55.0])
    ]

    estimated_sources = [
        np.array([102.0, 101.0, 48.0]),  # Close to source 1
        np.array([148.0, 152.0, 62.0]),  # Close to source 2
        np.array([180.0, 100.0, 53.0])   # Somewhat close to source 3
    ]

    print("True sources (pixels):")
    for i, src in enumerate(true_sources):
        print(f"  Source {i+1}: x={src[0]:.1f}, y={src[1]:.1f}, I={src[2]:.1f}")

    print("\nEstimated sources (pixels):")
    for i, src in enumerate(estimated_sources):
        print(f"  Source {i+1}: x={src[0]:.1f}, y={src[1]:.1f}, I={src[2]:.1f}")

    # Compute metrics
    rfc = 0.92
    metrics = compute_all_metrics(true_sources, estimated_sources, rfc)

    print("\n" + format_metrics(metrics))

    # Test unit conversion
    print("\n" + "="*70)
    print("Unit Conversion Test")
    print("="*70)
    print(f"1 pixel = {METERS_PER_PIXEL:.6f} meters")
    print(f"1 meter = {PIXELS_PER_METER:.2f} pixels")
    print(f"10 pixels = {pixels_to_meters(10):.3f} meters")
    print(f"1 meter = {meters_to_pixels(1.0):.2f} pixels")
    print(f"256 pixels = {pixels_to_meters(256):.1f} meters")

    print("\n✅ Performance metrics test complete!")
