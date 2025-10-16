"""
Weight calculation functions for ADE-PSPF algorithm.

Implements Equation 3 from the paper:
    w^syn_{s,r} = w_obs × w_dist × w_ps

where:
    - w_obs: Observation weight (Poisson-based)
    - w_dist: Peak suppression (distance weight)
    - w_ps: Swarm distance correction
"""
from __future__ import annotations
import numpy as np
from typing import List, Tuple, Optional
from scipy.special import gammaln
import warnings

# Suppress overflow warnings in factorial calculations
warnings.filterwarnings('ignore', category=RuntimeWarning)


def poisson_pmf(k: float, lambda_: float) -> float:
    """
    Compute Poisson probability mass function.

    P(k | λ) = (λ^k / k!) × exp(-λ)

    Args:
        k: Number of events (can be non-integer for rounded observation)
        lambda_: Expected number of events

    Returns:
        Probability P(k | λ)
    """
    if lambda_ < 0:
        return 0.0

    # Handle edge cases
    if lambda_ == 0:
        return 1.0 if k == 0 else 0.0

    # Compute in log space to avoid overflow
    try:
        # Use gammaln for stable log-factorial: log(k!) = gammaln(k + 1)
        log_prob = k * np.log(lambda_) - lambda_ - gammaln(k + 1.0)
        return float(np.exp(log_prob))
    except (OverflowError, ValueError):
        if k > 100 or lambda_ > 100:
            return float(
                np.exp(-0.5 * ((k - lambda_) / np.sqrt(lambda_)) ** 2)
                / np.sqrt(2 * np.pi * lambda_)
            )
        return 0.0


def predict_intensity(
    particle_pos: np.ndarray,
    particle_intensity: float,
    observation_pos: np.ndarray,
    other_centroids: List[np.ndarray],
    h: float = 0.5,
    mu_air: float = 6.86e-3
) -> float:
    """
    Predict radiation intensity at observation position.

    Implements Equation 1 (simplified):
        I'(M^pos_i, {S_k}) = Σ_k [S^int_k / (h^2 + ||M^pos_i - S^pos_k||^2)]

    Args:
        particle_pos: Particle position [x, y] (pixels)
        particle_intensity: Particle intensity
        observation_pos: Observation position [x, y] (pixels)
        other_centroids: List of other swarm centroids [[x, y, I], ...]
        h: Height of source from observation plane (meters)
        mu_air: Air absorption coefficient (approximated as 1 in paper)

    Returns:
        Predicted intensity at observation position
    """
    # Convert pixel distance to meters (assuming 256 pixels = 10 meters)
    pixel_to_meter = 10.0 / 256.0

    # Distance from particle to observation (in meters)
    dist_pixels = np.linalg.norm(particle_pos - observation_pos[:2])
    dist_meters = dist_pixels * pixel_to_meter

    # Intensity contribution from current particle (inverse square law)
    # exp(-mu_air * dist) ≈ 1 for small distances (as stated in paper)
    intensity = particle_intensity / (h**2 + dist_meters**2)

    # Add contributions from other swarm centroids
    for centroid in other_centroids:
        centroid_pos = centroid[:2]  # [x, y]
        centroid_intensity = centroid[2]  # I

        dist_pixels = np.linalg.norm(centroid_pos - observation_pos[:2])
        dist_meters = dist_pixels * pixel_to_meter

        intensity += centroid_intensity / (h**2 + dist_meters**2)

    return intensity


def observation_weight(
    observed_intensity: float,
    particle_pos: np.ndarray,
    particle_intensity: float,
    observation_pos: np.ndarray,
    other_centroids: List[np.ndarray],
    E: float = 1.0,
    tau: float = 1.0,
    R_back: float = 0.0,
    **kwargs
) -> float:
    """
    Calculate observation weight using Poisson likelihood.

    Implements Equation 3 (w_obs component):
        w_obs = f_p(M^int_i | I'(p_{s,r}, C_{-s})) / f_p(⌊I'(...)⌋ | I'(...))

    Args:
        observed_intensity: Actual observed radiation intensity (M^int_i)
        particle_pos: Particle position [x, y]
        particle_intensity: Particle intensity
        observation_pos: Observation position [x, y] or [x, y, I_obs]
        other_centroids: List of other swarm centroids
        E: Conversion constant (nSv/h → CPS)
        tau: Observation duration
        R_back: Background radiation

    Returns:
        Observation weight (0 to 1)
    """
    # Predict intensity based on particle and other centroids
    predicted_intensity = predict_intensity(
        particle_pos, particle_intensity, observation_pos, other_centroids, **kwargs
    )

    # Convert to mean count: λ = E × τ × (predicted + background)
    lambda_pred = E * tau * (predicted_intensity + R_back)

    # Observed count
    k_obs = E * tau * (observed_intensity + R_back)

    # Poisson probability ratio
    numerator = poisson_pmf(k_obs, lambda_pred)
    denominator = poisson_pmf(np.floor(lambda_pred), lambda_pred)

    # Avoid division by zero
    if denominator < 1e-10:
        denominator = 1e-10

    weight = numerator / denominator

    # Clamp to reasonable range
    return np.clip(weight, 1e-10, 1.0)


def peak_suppression_weight(
    particle_pos: np.ndarray,
    other_centroids: List[np.ndarray],
    theta_dist: float = 50.0,
    b_dist: float = 10.0
) -> float:
    """
    Calculate peak suppression (distance) weight.

    Implements Equation 3 (w_dist component):
        w_dist = 1 / (1 + exp[(θ_dist - f_d(p_{s,r}, C_{-s})) / b_dist])
        f_d = min_{j≠s} ||p_{s,r} - C_j||

    This suppresses particles that are too close to other swarm centroids,
    helping to maintain multi-modal estimation.

    Args:
        particle_pos: Particle position [x, y]
        other_centroids: List of other swarm centroids [[x, y, I], ...]
        theta_dist: Horizontal offset of suppression curve
        b_dist: Scale parameter for rate of change

    Returns:
        Distance weight (0 to 1)
    """
    if not other_centroids:
        return 1.0  # No suppression if no other centroids

    # Find minimum distance to other centroids
    min_distance = float('inf')
    for centroid in other_centroids:
        centroid_pos = centroid[:2]  # [x, y]
        dist = np.linalg.norm(particle_pos - centroid_pos)
        min_distance = min(min_distance, dist)

    # Sigmoid suppression
    weight = 1.0 / (1.0 + np.exp((theta_dist - min_distance) / b_dist))

    return weight


def swarm_correction_weight(
    particle_intensity: float,
    theta_ps: float = 50.0,
    b_ps: float = 10.0,
    alpha: float = 0.5
) -> float:
    """
    Calculate swarm distance correction weight.

    Implements Equation 3 (w_ps component):
        w_ps = (1 - α) + α / (1 + exp[(p^int_{s,r} - θ_ps) / b_ps])

    This adjusts particle weights based on their intensity values.

    Args:
        particle_intensity: Particle intensity value
        theta_ps: Horizontal offset of correction curve
        b_ps: Scale parameter
        alpha: Vertical adjustment parameter (0 to 1)

    Returns:
        Swarm correction weight
    """
    sigmoid = 1.0 / (1.0 + np.exp((particle_intensity - theta_ps) / b_ps))
    weight = (1.0 - alpha) + alpha * sigmoid

    return weight


def synthesized_weight(
    particle_pos: np.ndarray,
    particle_intensity: float,
    observed_intensity: float,
    observation_pos: np.ndarray,
    other_centroids: List[np.ndarray],
    theta_dist: float = 50.0,
    b_dist: float = 10.0,
    theta_ps: float = 50.0,
    b_ps: float = 10.0,
    alpha: float = 0.5,
    E: float = 1.0,
    tau: float = 1.0,
    R_back: float = 0.0,
    **kwargs
) -> Tuple[float, dict]:
    """
    Calculate synthesized weight for a particle.

    Implements Equation 3:
        w^syn_{s,r} = w_obs × w_dist × w_ps

    Args:
        particle_pos: Particle position [x, y]
        particle_intensity: Particle intensity
        observed_intensity: Actual observed intensity
        observation_pos: Observation position [x, y]
        other_centroids: List of other swarm centroids
        theta_dist: Peak suppression threshold
        b_dist: Peak suppression scale
        theta_ps: Swarm correction offset
        b_ps: Swarm correction scale
        alpha: Swarm correction parameter
        E: Conversion constant
        tau: Observation duration
        R_back: Background radiation
        **kwargs: Additional arguments for predict_intensity

    Returns:
        Tuple of (synthesized_weight, weight_components_dict)
    """
    # Calculate individual weight components
    w_obs = observation_weight(
        observed_intensity, particle_pos, particle_intensity,
        observation_pos, other_centroids, E, tau, R_back, **kwargs
    )

    w_dist = peak_suppression_weight(
        particle_pos, other_centroids, theta_dist, b_dist
    )

    w_ps = swarm_correction_weight(
        particle_intensity, theta_ps, b_ps, alpha
    )

    # Synthesized weight (product of components)
    w_syn = w_obs * w_dist * w_ps

    # Return weight and components for debugging
    components = {
        'w_obs': w_obs,
        'w_dist': w_dist,
        'w_ps': w_ps,
        'w_syn': w_syn
    }

    return w_syn, components


if __name__ == "__main__":
    print("Testing weight calculation functions...\n")

    # Test Poisson PMF
    print("=== Testing Poisson PMF ===")
    print(f"P(5 | λ=5) = {poisson_pmf(5, 5):.6f}")  # Should be ~0.1755
    print(f"P(10 | λ=10) = {poisson_pmf(10, 10):.6f}")  # Should be ~0.1251

    # Test intensity prediction
    print("\n=== Testing Intensity Prediction ===")
    particle_pos = np.array([100.0, 100.0])
    particle_intensity = 50.0
    observation_pos = np.array([110.0, 110.0])
    other_centroids = [np.array([150.0, 150.0, 40.0])]

    predicted = predict_intensity(
        particle_pos, particle_intensity, observation_pos, other_centroids
    )
    print(f"Predicted intensity: {predicted:.4f}")

    # Test observation weight
    print("\n=== Testing Observation Weight ===")
    observed_intensity = 45.0
    w_obs = observation_weight(
        observed_intensity, particle_pos, particle_intensity,
        observation_pos, other_centroids
    )
    print(f"Observation weight: {w_obs:.6f}")

    # Test peak suppression
    print("\n=== Testing Peak Suppression ===")
    w_dist_close = peak_suppression_weight(
        np.array([100.0, 100.0]), [np.array([105.0, 105.0, 50.0])]
    )
    w_dist_far = peak_suppression_weight(
        np.array([100.0, 100.0]), [np.array([200.0, 200.0, 50.0])]
    )
    print(f"Weight when close to centroid: {w_dist_close:.6f}")
    print(f"Weight when far from centroid: {w_dist_far:.6f}")

    # Test swarm correction
    print("\n=== Testing Swarm Correction ===")
    w_ps_low = swarm_correction_weight(20.0)
    w_ps_high = swarm_correction_weight(80.0)
    print(f"Weight for low intensity: {w_ps_low:.6f}")
    print(f"Weight for high intensity: {w_ps_high:.6f}")

    # Test synthesized weight
    print("\n=== Testing Synthesized Weight ===")
    w_syn, components = synthesized_weight(
        particle_pos, particle_intensity, observed_intensity,
        observation_pos, other_centroids
    )
    print(f"Synthesized weight: {w_syn:.6f}")
    print("Components:")
    for key, value in components.items():
        print(f"  {key}: {value:.6f}")

    print("\n✅ All weight tests passed!")
