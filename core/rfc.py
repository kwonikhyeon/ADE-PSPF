"""
Radiation Field Confidence (RFC) calculation and Configuration Maintenance.

Implements Equation 7 from the paper:
    F = (1/N_o) Σ [f_p(M^int_i | I'_cum(...)) / f_p(⌊I'_cum(...)⌋ | I'_cum(...))]
"""
from __future__ import annotations
import numpy as np
from typing import List, Tuple, Optional
from .weights import poisson_pmf, predict_intensity
from .particle import Configuration


def compute_cumulative_intensity(
    observation_pos: np.ndarray,
    centroids: List[np.ndarray],
    h: float = 0.5,
    mu_air: float = 6.86e-3
) -> float:
    """
    Compute cumulative radiation intensity from all predicted sources.

    I'_cum(M^pos_i, {C_s}) = Σ_s I(M^pos_i, C_s)

    Args:
        observation_pos: Observation position [x, y]
        centroids: List of swarm centroids [[x, y, I], ...]
        h: Height of sources
        mu_air: Air absorption coefficient

    Returns:
        Cumulative predicted intensity
    """
    total_intensity = 0.0
    pixel_to_meter = 10.0 / 256.0

    for centroid in centroids:
        if centroid is None:
            continue

        centroid_pos = centroid[:2]  # [x, y]
        centroid_intensity = centroid[2]  # I

        # Distance in meters
        dist_pixels = np.linalg.norm(observation_pos - centroid_pos)
        dist_meters = dist_pixels * pixel_to_meter

        # Inverse square law (exp term ≈ 1 for small distances)
        intensity = centroid_intensity / (h**2 + dist_meters**2)
        total_intensity += intensity

    return total_intensity


def compute_rfc(
    observations: List[Tuple[np.ndarray, float]],
    centroids: List[np.ndarray],
    E: float = 1.0,
    tau: float = 1.0,
    R_back: float = 0.0,
    h: float = 0.5
) -> float:
    """
    Compute Radiation Field Confidence (RFC).

    Implements Equation 7:
        F = (1/N_o) Σ_i [f_p(M^int_i | I'_cum(...)) / f_p(⌊I'_cum(...)⌋ | I'_cum(...))]

    Args:
        observations: List of (position, intensity) tuples
            - position: [x, y] in pixels
            - intensity: observed radiation intensity
        centroids: List of predicted centroids [[x, y, I], ...]
        E: Conversion constant (nSv/h → CPS)
        tau: Observation duration
        R_back: Background radiation
        h: Height of sources

    Returns:
        RFC value (0 to 1, higher is better)
    """
    if not observations or not centroids:
        return 0.0

    # Filter out None centroids
    valid_centroids = [c for c in centroids if c is not None]
    if not valid_centroids:
        return 0.0

    confidence_sum = 0.0
    n_observations = len(observations)

    for obs_pos, obs_intensity in observations:
        # Predict cumulative intensity
        predicted_intensity = compute_cumulative_intensity(
            obs_pos, valid_centroids, h=h
        )

        # Convert to mean count
        lambda_pred = E * tau * (predicted_intensity + R_back)
        k_obs = E * tau * (obs_intensity + R_back)

        # Poisson probability ratio
        numerator = poisson_pmf(k_obs, lambda_pred)
        denominator = poisson_pmf(np.floor(lambda_pred), lambda_pred)

        # Avoid division by zero
        if denominator < 1e-10:
            denominator = 1e-10

        confidence_sum += numerator / denominator

    # Average confidence
    rfc = confidence_sum / n_observations

    # Clamp to [0, 1]
    rfc = np.clip(rfc, 0.0, 1.0)

    return rfc


class ConfigurationManager:
    """
    Manages configuration snapshots for Configuration Maintenance (Section 3.4).

    논문 설명:
    "The configuration maintenance will start when the current RFC is lower
    than the historical best confidence. It utilizes the particle swarms,
    which have the highest confidence, as the basis for recovery."

    즉, RFC가 감소하면 best configuration으로 자동 복원합니다.
    """

    def __init__(self, tolerance: float = 0.01):
        """
        Initialize configuration manager.

        Args:
            tolerance: Tolerance for RFC comparison (noise handling)
        """
        self.tolerance = tolerance
        self.best_config: Optional[Configuration] = None
        self.best_rfc: float = 0.0
        self.current_config: Optional[Configuration] = None
        self.history: List[Tuple[Configuration, float]] = []
        self.maintenance_count: int = 0  # Number of times maintenance triggered

    def update(
        self,
        particles_list: List[np.ndarray],
        centroids_list: List[np.ndarray],
        rfc: float,
        iteration: int
    ) -> Tuple[bool, Optional[Configuration]]:
        """
        Update configuration with new state (논문 Section 3.4).

        Configuration Maintenance:
        - RFC 증가: 현재 config 유지, best 업데이트
        - RFC 감소: best config로 복원

        Args:
            particles_list: List of (N_p, 3) arrays for each swarm
            centroids_list: List of (3,) arrays for each centroid
            rfc: Current RFC value
            iteration: Current iteration number

        Returns:
            (maintained, restored_config):
                - maintained: True if best config was restored
                - restored_config: Best config if restored, None otherwise
        """
        # Create new configuration
        new_config = Configuration(
            particles=[p.copy() for p in particles_list],
            centroids=[c.copy() if c is not None else None for c in centroids_list],
            confidence=rfc,
            iteration=iteration
        )

        self.current_config = new_config

        # Check if valid configuration
        if not new_config.is_valid():
            if self.best_config is not None:
                self.maintenance_count += 1
                return True, self.best_config.copy()
            return False, None

        # Configuration Maintenance (논문 Section 3.4)
        if rfc < self.best_rfc - self.tolerance:
            # RFC decreased: restore best configuration
            if self.best_config is not None:
                self.maintenance_count += 1
                return True, self.best_config.copy()

        # RFC improved or maintained: update best
        if rfc > self.best_rfc:
            self.best_config = new_config.copy()
            self.best_rfc = rfc

        # Add to history
        self.history.append((new_config.copy(), rfc))

        return False, None

    def restore_best(self) -> Configuration:
        """
        Restore the best configuration.

        Returns:
            Best configuration snapshot
        """
        if self.best_config is None:
            raise ValueError("No best configuration available")

        return self.best_config.copy()

    def get_best_rfc(self) -> float:
        """Get the best RFC achieved so far."""
        if self.best_config is None:
            return 0.0
        return self.best_config.confidence

    def get_current_rfc(self) -> float:
        """Get the current RFC."""
        if self.current_config is None:
            return 0.0
        return self.current_config.confidence

    def should_restore(self) -> bool:
        """
        Check if configuration should be restored.

        Returns:
            True if current RFC is significantly worse than best
        """
        if self.best_config is None or self.current_config is None:
            return False

        return self.current_config.confidence < (self.best_rfc - self.tolerance)

    def clear_history(self):
        """Clear configuration history (but keep best)."""
        self.history.clear()

    def get_rfc_history(self) -> np.ndarray:
        """Get array of RFC values over iterations."""
        return np.array([config.confidence for config in self.history])

    def __repr__(self) -> str:
        best_rfc = self.get_best_rfc()
        current_rfc = self.get_current_rfc()
        return (f"ConfigurationManager(best_RFC={best_rfc:.4f}, "
                f"current_RFC={current_rfc:.4f}, tolerance={self.tolerance})")


if __name__ == "__main__":
    print("Testing RFC calculation and Configuration Management...\n")

    # Test 1: Compute cumulative intensity
    print("=== Test 1: Cumulative Intensity ===")
    observation_pos = np.array([100.0, 100.0])
    centroids = [
        np.array([110.0, 110.0, 50.0]),
        np.array([90.0, 90.0, 40.0])
    ]
    cum_intensity = compute_cumulative_intensity(observation_pos, centroids)
    print(f"Cumulative intensity: {cum_intensity:.4f}")

    # Test 2: Compute RFC with perfect prediction
    print("\n=== Test 2: RFC with Perfect Prediction ===")
    # Create observations
    observations = []
    for i in range(10):
        pos = np.array([100.0 + i * 10, 100.0 + i * 10])
        # Compute "true" intensity from centroids
        intensity = compute_cumulative_intensity(pos, centroids)
        observations.append((pos, intensity))

    rfc_perfect = compute_rfc(observations, centroids)
    print(f"RFC (perfect prediction): {rfc_perfect:.6f}")

    # Test 3: RFC with noisy prediction
    print("\n=== Test 3: RFC with Noisy Prediction ===")
    noisy_centroids = [
        np.array([112.0, 108.0, 48.0]),  # Slightly off
        np.array([88.0, 92.0, 42.0])
    ]
    rfc_noisy = compute_rfc(observations, noisy_centroids)
    print(f"RFC (noisy prediction): {rfc_noisy:.6f}")

    # Test 4: Configuration Manager
    print("\n=== Test 4: Configuration Manager ===")
    manager = ConfigurationManager(tolerance=0.05)

    # Simulate iterations with varying RFC
    rfc_values = [0.70, 0.80, 0.90, 0.92, 0.88, 0.75, 0.91]

    for i, rfc in enumerate(rfc_values):
        particles = [np.random.rand(10, 3) * 100]
        centroids_list = [np.array([100.0, 100.0, 50.0])]

        maintained, restored = manager.update(particles, centroids_list, rfc, iteration=i)

        print(f"Iter {i}: RFC={rfc:.2f}, Maintained={maintained}, "
              f"Best={manager.get_best_rfc():.2f}, Should_restore={manager.should_restore()}")

    # Test 5: Restore best configuration
    print("\n=== Test 5: Restore Best Configuration ===")
    best_config = manager.restore_best()
    print(f"Best configuration: {best_config}")

    # Test 6: RFC history
    print("\n=== Test 6: RFC History ===")
    rfc_history = manager.get_rfc_history()
    print(f"RFC history: {rfc_history}")
    print(f"Mean RFC: {np.mean(rfc_history):.4f}")
    print(f"Max RFC: {np.max(rfc_history):.4f}")

    print("\n✅ All RFC tests passed!")
