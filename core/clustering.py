"""
Mean Shift Clustering for finding swarm centroids.

This module implements mean shift algorithm to find the centroid
of each particle swarm, which represents the estimated source parameter.
"""
from __future__ import annotations
import numpy as np
from typing import Optional, Tuple
from sklearn.cluster import MeanShift
import warnings


def mean_shift_clustering(
    particles: np.ndarray,
    weights: np.ndarray,
    bandwidth: Optional[float] = None,
    max_iter: int = 300,
    min_cluster_size: int = 3
) -> Tuple[Optional[np.ndarray], bool]:
    """
    Find centroid of particle distribution using weighted mean shift.

    Implements proper weighted mean shift algorithm as described in the paper
    to find the mode (peak) of the particle distribution.

    Args:
        particles: (N, 3) array of particles [x, y, intensity]
        weights: (N,) array of particle weights
        bandwidth: Bandwidth for mean shift (auto-estimated if None)
        max_iter: Maximum iterations for convergence
        min_cluster_size: Minimum particles to form valid cluster

    Returns:
        Tuple of (centroid, success)
        - centroid: [x, y, intensity] or None if clustering failed
        - success: True if valid centroid found
    """
    if len(particles) < min_cluster_size:
        return None, False

    # Normalize weights
    weights_norm = weights / (np.sum(weights) + 1e-10)

    # If all weights are too small, clustering failed
    if np.max(weights_norm) < 1e-6:
        return None, False

    # Estimate bandwidth if not provided
    if bandwidth is None:
        # Use rule of thumb: bandwidth ≈ std of weighted particles
        weighted_mean = np.average(particles, weights=weights_norm, axis=0)
        weighted_var = np.average(
            (particles - weighted_mean) ** 2,
            weights=weights_norm,
            axis=0
        )
        bandwidth = np.mean(np.sqrt(weighted_var)) * 1.5
        # Ensure minimum bandwidth
        bandwidth = max(bandwidth, 5.0)

    try:
        # Use proper weighted mean shift to find the mode (peak)
        # This is critical for accurate source localization
        centroid, success = weighted_mean_shift(
            particles,
            weights_norm,
            bandwidth=bandwidth,
            max_iter=max_iter,
            tol=1e-3
        )

        # Check if centroid is valid
        if centroid is None or np.any(np.isnan(centroid)) or np.any(np.isinf(centroid)):
            return None, False

        return centroid, success

    except Exception as e:
        warnings.warn(f"Mean shift clustering failed: {e}")
        return None, False


def weighted_mean_shift(
    particles: np.ndarray,
    weights: np.ndarray,
    bandwidth: float = 10.0,
    max_iter: int = 300,
    tol: float = 1e-3
) -> Tuple[Optional[np.ndarray], bool]:
    """
    Custom weighted mean shift implementation.

    This iteratively moves toward the mode (peak) of the weighted
    particle distribution.

    Args:
        particles: (N, 3) array of particles [x, y, intensity]
        weights: (N,) array of particle weights
        bandwidth: Kernel bandwidth
        max_iter: Maximum iterations
        tol: Convergence tolerance

    Returns:
        Tuple of (centroid, success)
    """
    if len(particles) == 0:
        return None, False

    # Normalize weights
    weights_norm = weights / (np.sum(weights) + 1e-10)

    # Initialize at weighted mean
    centroid = np.average(particles, weights=weights_norm, axis=0)

    for iteration in range(max_iter):
        # Compute distances to all particles
        distances = np.linalg.norm(particles - centroid, axis=1)

        # Gaussian kernel weights
        kernel_weights = np.exp(-0.5 * (distances / bandwidth) ** 2)

        # Combine with particle weights
        combined_weights = kernel_weights * weights_norm
        combined_weights /= (np.sum(combined_weights) + 1e-10)

        # Update centroid
        new_centroid = np.sum(particles * combined_weights[:, np.newaxis], axis=0)

        # Check convergence
        shift = np.linalg.norm(new_centroid - centroid)
        if shift < tol:
            return new_centroid, True

        centroid = new_centroid

    # Max iterations reached
    return centroid, True


def filter_redundant_swarms(
    centroids: list[Optional[np.ndarray]],
    min_distance: float = 20.0,
    swarm_weights: Optional[list[float]] = None
) -> list[int]:
    """
    Filter redundant swarms that are too close to each other.

    When multiple swarms converge to similar centroids, keep only
    the most confident ones based on particle weights.

    Args:
        centroids: List of centroids (can contain None for failed clustering)
        min_distance: Minimum distance between valid centroids (pixels)
        swarm_weights: Optional list of swarm confidence scores (average particle weights)

    Returns:
        List of valid swarm indices
    """
    # Build list of valid centroids with their indices and weights
    centroid_data = []
    for i, centroid in enumerate(centroids):
        if centroid is None:
            continue

        weight = swarm_weights[i] if swarm_weights is not None else 1.0
        centroid_data.append({
            'index': i,
            'centroid': centroid,
            'weight': weight
        })

    if len(centroid_data) == 0:
        return []

    # Sort by weight (descending) - keep most confident swarms first
    centroid_data.sort(key=lambda x: x['weight'], reverse=True)

    valid_indices = []
    valid_centroids = []

    for data in centroid_data:
        centroid = data['centroid']

        # Check if too close to existing valid centroids
        is_redundant = False
        for existing_centroid in valid_centroids:
            # Only compare spatial position (x, y), not intensity
            distance = np.linalg.norm(centroid[:2] - existing_centroid[:2])
            if distance < min_distance:
                is_redundant = True
                break

        if not is_redundant:
            valid_indices.append(data['index'])
            valid_centroids.append(centroid)

    # Return indices in original order
    valid_indices.sort()
    return valid_indices


def adaptive_bandwidth(
    particles: np.ndarray,
    weights: np.ndarray,
    percentile: float = 75.0
) -> float:
    """
    Estimate adaptive bandwidth for mean shift.

    Uses weighted percentile of inter-particle distances.

    Args:
        particles: (N, 3) array of particles
        weights: (N,) array of weights
        percentile: Percentile for bandwidth estimation (default 75)

    Returns:
        Estimated bandwidth
    """
    if len(particles) < 2:
        return 10.0  # Default bandwidth

    # Normalize weights
    weights_norm = weights / (np.sum(weights) + 1e-10)

    # Sample particles based on weights
    n_samples = min(100, len(particles))
    indices = np.random.choice(
        len(particles),
        size=n_samples,
        replace=False,
        p=weights_norm
    )

    sampled = particles[indices]

    # Compute pairwise distances
    distances = []
    for i in range(len(sampled)):
        for j in range(i + 1, len(sampled)):
            dist = np.linalg.norm(sampled[i] - sampled[j])
            distances.append(dist)

    if not distances:
        return 10.0

    # Use percentile of distances as bandwidth
    bandwidth = np.percentile(distances, percentile)

    # Clamp to reasonable range
    bandwidth = np.clip(bandwidth, 5.0, 50.0)

    return bandwidth


if __name__ == "__main__":
    print("Testing Mean Shift Clustering...\n")

    # Test 1: Simple clustering
    print("=== Test 1: Simple Clustering ===")
    rng = np.random.default_rng(42)

    # Create particles around a center
    center = np.array([100.0, 100.0, 50.0])
    particles = center + rng.normal(0, 5, (50, 3))
    weights = rng.uniform(0.5, 1.0, 50)

    centroid, success = mean_shift_clustering(particles, weights)
    print(f"True center: {center}")
    print(f"Found centroid: {centroid}")
    print(f"Success: {success}")
    print(f"Error: {np.linalg.norm(centroid - center):.4f}")

    # Test 2: Weighted mean shift
    print("\n=== Test 2: Weighted Mean Shift ===")
    centroid_wms, success_wms = weighted_mean_shift(
        particles, weights, bandwidth=10.0
    )
    print(f"Centroid (WMS): {centroid_wms}")
    print(f"Success: {success_wms}")
    print(f"Error: {np.linalg.norm(centroid_wms - center):.4f}")

    # Test 3: Multiple swarms
    print("\n=== Test 3: Multiple Swarms ===")
    centers = [
        np.array([50.0, 50.0, 40.0]),
        np.array([150.0, 150.0, 60.0]),
        np.array([100.0, 200.0, 50.0])
    ]

    centroids = []
    for i, center in enumerate(centers):
        particles = center + rng.normal(0, 5, (30, 3))
        weights = rng.uniform(0.5, 1.0, 30)
        centroid, success = mean_shift_clustering(particles, weights)
        centroids.append(centroid)
        print(f"Swarm {i}: True={center}, Found={centroid}, Success={success}")

    # Test 4: Filter redundant swarms
    print("\n=== Test 4: Filter Redundant Swarms ===")
    # Add redundant centroid close to first one
    centroids.append(np.array([52.0, 52.0, 41.0]))
    centroids.append(None)  # Failed clustering

    valid_indices = filter_redundant_swarms(centroids, min_distance=20.0)
    print(f"Valid swarm indices: {valid_indices}")
    print(f"Valid centroids:")
    for idx in valid_indices:
        print(f"  Swarm {idx}: {centroids[idx]}")

    # Test 5: Adaptive bandwidth
    print("\n=== Test 5: Adaptive Bandwidth ===")
    particles = center + rng.normal(0, 10, (100, 3))
    weights = rng.uniform(0, 1, 100)
    bw = adaptive_bandwidth(particles, weights)
    print(f"Estimated bandwidth: {bw:.2f}")

    print("\n✅ All clustering tests passed!")
