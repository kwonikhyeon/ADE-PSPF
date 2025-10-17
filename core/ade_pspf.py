"""
Complete ADE-PSPF Algorithm Implementation.

This module integrates all components to implement the full ADE-PSPF algorithm
as described in the paper "A study of robotic search strategy for multi-radiation
sources in unknown environments".
"""
from __future__ import annotations
import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass

from .particle import Particle, ParticleSwarm, Configuration, initialize_swarms
from .weights import synthesized_weight
from .ade import ade_resample_with_weight_function
from .clustering import mean_shift_clustering, filter_redundant_swarms
from .rfc import compute_rfc, ConfigurationManager


@dataclass
class ADEPSPFConfig:
    """Configuration parameters for ADE-PSPF algorithm."""

    # Swarm parameters
    n_swarms: int = 5  # Number of particle swarms (N_ps)
    n_particles: int = 250  # Particles per swarm (N_p)

    # Iteration parameters
    n_iterations: int = 15  # PSPF main loop iterations
    ade_generations: int = 5  # ADE inner loop generations

    # Weight parameters
    theta_dist: float = 50.0  # Peak suppression threshold
    b_dist: float = 10.0  # Peak suppression scale
    theta_ps: float = 50.0  # Swarm correction offset
    b_ps: float = 10.0  # Swarm correction scale
    alpha_ps: float = 0.5  # Swarm correction parameter

    # ADE parameters
    alpha_ade: float = 0.5  # Elite movement scale
    beta_ade: float = 0.3  # Random movement scale
    CR_base: float = 0.5  # Base crossover rate
    CR_scale: float = 0.3  # Crossover rate scale
    sigma: float = 1.0  # Gaussian noise std

    # Clustering parameters
    bandwidth: Optional[float] = None  # Mean shift bandwidth (auto if None)
    min_cluster_distance: float = 20.0  # Min distance between sources (pixels)

    # RFC parameters
    rfc_threshold: float = 0.85  # RFC threshold for configuration maintenance
    E: float = 1.0  # Conversion constant
    tau: float = 1.0  # Observation duration
    R_back: float = 0.0  # Background radiation
    h: float = 0.5  # Source height (meters)

    # Search space bounds
    x_min: float = 0.0
    x_max: float = 256.0
    y_min: float = 0.0
    y_max: float = 256.0
    intensity_min: float = 0.0
    intensity_max: float = 100.0

    @property
    def bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get search space bounds."""
        min_bound = np.array([self.x_min, self.y_min, self.intensity_min])
        max_bound = np.array([self.x_max, self.y_max, self.intensity_max])
        return min_bound, max_bound


class ADEPSPF:
    """
    Adaptive Differential Evolution - Peak Suppression Particle Filter.

    This class implements the complete ADE-PSPF algorithm for estimating
    multiple radiation source parameters.
    """

    def __init__(
        self,
        config: Optional[ADEPSPFConfig] = None,
        rng: Optional[np.random.Generator] = None
    ):
        """
        Initialize ADE-PSPF estimator.

        Args:
            config: Algorithm configuration (uses defaults if None)
            rng: Random number generator
        """
        self.config = config or ADEPSPFConfig()
        self.rng = rng or np.random.default_rng()

        # Initialize swarms
        self.swarms: List[ParticleSwarm] = []
        self.centroids: List[Optional[np.ndarray]] = []

        # Configuration manager
        self.config_manager = ConfigurationManager(
            tolerance=0.01  # Small tolerance for RFC fluctuations
        )

        # Statistics
        self.iteration = 0
        self.rfc_history: List[float] = []

        self._initialize_swarms()

    def _initialize_swarms(self):
        """Initialize all particle swarms."""
        self.swarms = initialize_swarms(
            n_swarms=self.config.n_swarms,
            n_particles_per_swarm=self.config.n_particles,
            bounds=self.config.bounds,
            rng=self.rng
        )
        self.centroids = [None] * self.config.n_swarms

    def update(
        self,
        observations: List[Tuple[np.ndarray, float]]
    ) -> Dict:
        """
        Update estimation with new observations.

        Args:
            observations: List of (position, intensity) tuples
                - position: [x, y] in pixels
                - intensity: observed radiation intensity

        Returns:
            Dictionary with update results
        """
        self.iteration += 1

        # Step 1: Update particle weights
        self._update_weights(observations)

        # Step 2: ADE resampling
        self._ade_resample(observations)

        # Step 3: Mean shift clustering to find centroids
        self._compute_centroids()

        # Step 4: Compute RFC
        rfc = compute_rfc(
            observations,
            self.centroids,
            E=self.config.E,
            tau=self.config.tau,
            R_back=self.config.R_back,
            h=self.config.h
        )
        self.rfc_history.append(rfc)

        # Step 5: Configuration maintenance
        particles_list = [swarm.to_array() for swarm in self.swarms]
        accepted = self.config_manager.update(
            particles_list,
            self.centroids,
            rfc,
            self.iteration
        )

        # If not accepted, restore best configuration
        if not accepted and self.config_manager.should_restore():
            self._restore_configuration()

        # Filter redundant swarms
        valid_sources = self.get_valid_sources()

        result = {
            'iteration': self.iteration,
            'rfc': rfc,
            'n_sources': len(valid_sources),
            'sources': valid_sources,
            'centroids': [c for c in self.centroids if c is not None],
            'configuration_accepted': accepted,
            'best_rfc': self.config_manager.get_best_rfc()
        }

        return result

    def _update_weights(self, observations: List[Tuple[np.ndarray, float]]):
        """Update weights for all particles in all swarms."""
        # OPTIMIZATION: Limit observations to prevent exponential slowdown
        # Use only the most recent N observations
        max_obs_for_weight = 20  # Configurable threshold
        obs_to_use = observations[-max_obs_for_weight:] if len(observations) > max_obs_for_weight else observations

        for s, swarm in enumerate(self.swarms):
            # Get other swarm centroids (excluding current swarm)
            other_centroids = [
                self.centroids[i] for i in range(len(self.centroids))
                if i != s and self.centroids[i] is not None
            ]

            # Update weights for each particle
            for particle in swarm.particles:
                # Average weight over all observations
                weights_obs = []

                for obs_pos, obs_intensity in obs_to_use:
                    w_syn, _ = synthesized_weight(
                        particle_pos=np.array([particle.x, particle.y]),
                        particle_intensity=particle.intensity,
                        observed_intensity=obs_intensity,
                        observation_pos=obs_pos,
                        other_centroids=other_centroids,
                        theta_dist=self.config.theta_dist,
                        b_dist=self.config.b_dist,
                        theta_ps=self.config.theta_ps,
                        b_ps=self.config.b_ps,
                        alpha=self.config.alpha_ps,
                        E=self.config.E,
                        tau=self.config.tau,
                        R_back=self.config.R_back,
                        h=self.config.h
                    )
                    weights_obs.append(w_syn)

                # Use mean weight over observations
                particle.weight = np.mean(weights_obs) if weights_obs else 0.0

            # Normalize weights
            swarm.normalize_weights()
            swarm.update_best_particle()

    def _ade_resample(self, observations: List[Tuple[np.ndarray, float]]):
        """Perform ADE resampling on all swarms."""
        # OPTIMIZATION: Limit observations to prevent exponential slowdown
        max_obs_for_weight = 20
        obs_to_use = observations[-max_obs_for_weight:] if len(observations) > max_obs_for_weight else observations

        for s, swarm in enumerate(self.swarms):
            # Define weight function for this swarm
            other_centroids = [
                self.centroids[i] for i in range(len(self.centroids))
                if i != s and self.centroids[i] is not None
            ]

            def weight_func(particle_pos, particle_intensity):
                weights_obs = []
                for obs_pos, obs_intensity in obs_to_use:
                    w_syn, _ = synthesized_weight(
                        particle_pos=particle_pos,
                        particle_intensity=particle_intensity,
                        observed_intensity=obs_intensity,
                        observation_pos=obs_pos,
                        other_centroids=other_centroids,
                        theta_dist=self.config.theta_dist,
                        b_dist=self.config.b_dist,
                        theta_ps=self.config.theta_ps,
                        b_ps=self.config.b_ps,
                        alpha=self.config.alpha_ps,
                        E=self.config.E,
                        tau=self.config.tau,
                        R_back=self.config.R_back,
                        h=self.config.h
                    )
                    weights_obs.append(w_syn)
                return np.mean(weights_obs) if weights_obs else 0.0

            # Resample swarm
            self.swarms[s] = ade_resample_with_weight_function(
                swarm=swarm,
                weight_function=weight_func,
                bounds=self.config.bounds,
                n_generations=self.config.ade_generations,
                alpha=self.config.alpha_ade,
                beta=self.config.beta_ade,
                CR_base=self.config.CR_base,
                CR_scale=self.config.CR_scale,
                sigma=self.config.sigma,
                rng=self.rng
            )

    def _compute_centroids(self):
        """Compute centroids for all swarms using mean shift."""
        for s, swarm in enumerate(self.swarms):
            particles = swarm.to_array()
            weights = swarm.weights

            centroid, success = mean_shift_clustering(
                particles,
                weights,
                bandwidth=self.config.bandwidth
            )

            if success:
                self.centroids[s] = centroid
                swarm.update_centroid(centroid)
            else:
                self.centroids[s] = None

    def _restore_configuration(self):
        """Restore best configuration from manager."""
        best_config = self.config_manager.restore_best()

        # Restore particles
        for s, swarm in enumerate(self.swarms):
            particles_array = best_config.particles[s]
            # Weights will be recalculated in next iteration
            weights = np.ones(len(particles_array)) / len(particles_array)
            swarm.from_array(particles_array, weights)

        # Restore centroids
        self.centroids = [
            c.copy() if c is not None else None
            for c in best_config.centroids
        ]

    def get_valid_sources(self) -> List[np.ndarray]:
        """
        Get valid source estimates (filtered redundant swarms).

        Returns:
            List of source parameters [[x, y, intensity], ...]
        """
        # Calculate swarm confidence (average particle weight)
        swarm_weights = []
        for swarm in self.swarms:
            avg_weight = np.mean(swarm.weights) if len(swarm.weights) > 0 else 0.0
            swarm_weights.append(avg_weight)

        # Filter redundant swarms, keeping most confident ones
        valid_indices = filter_redundant_swarms(
            self.centroids,
            min_distance=self.config.min_cluster_distance,
            swarm_weights=swarm_weights
        )

        valid_sources = [self.centroids[i] for i in valid_indices]
        return valid_sources

    def get_current_rfc(self) -> float:
        """Get current RFC value."""
        if not self.rfc_history:
            return 0.0
        return self.rfc_history[-1]

    def get_best_rfc(self) -> float:
        """Get best RFC achieved."""
        return self.config_manager.get_best_rfc()

    def get_statistics(self) -> Dict:
        """Get algorithm statistics."""
        return {
            'iteration': self.iteration,
            'current_rfc': self.get_current_rfc(),
            'best_rfc': self.get_best_rfc(),
            'rfc_history': np.array(self.rfc_history),
            'n_swarms': len(self.swarms),
            'n_valid_sources': len(self.get_valid_sources()),
        }

    def add_swarm(self) -> int:
        """
        Add a new particle swarm (논문 Fig. 11: Dynamic Swarm Adjustment).

        Returns:
            New swarm ID
        """
        from .particle import initialize_swarms

        # Create new swarm with same configuration
        new_swarms = initialize_swarms(
            n_swarms=1,
            n_particles_per_swarm=self.config.n_particles,
            bounds=self.config.bounds,
            rng=self.rng
        )
        new_swarm = new_swarms[0]

        # Update swarm ID to be unique
        new_swarm.swarm_id = len(self.swarms)

        # Add to swarms list
        self.swarms.append(new_swarm)
        self.centroids.append(None)

        return new_swarm.swarm_id

    def reset(self):
        """Reset estimator to initial state."""
        self._initialize_swarms()
        self.config_manager = ConfigurationManager(
            tolerance=0.01  # Small tolerance for RFC fluctuations
        )
        self.iteration = 0
        self.rfc_history.clear()


if __name__ == "__main__":
    from environment.generate_truth import sample_sources, gaussian_field, GRID
    from environment.observation import RadiationObserver

    print("Testing complete ADE-PSPF algorithm...\n")

    # Create ground truth
    print("=== Creating Ground Truth ===")
    rng = np.random.default_rng(42)
    n_sources = 3
    coords, amps, sigmas = sample_sources(GRID, n_sources, rng=rng)
    gt_field = gaussian_field(GRID, coords, amps, sigmas)

    print(f"True sources:")
    for i in range(n_sources):
        print(f"  Source {i}: pos=({coords[i,1]:.1f}, {coords[i,0]:.1f}), "
              f"intensity={amps[i]:.2f}")

    # Create observer
    observer = RadiationObserver(gt_field, GRID)

    # Generate some random observations
    print("\n=== Generating Observations ===")
    n_obs = 20
    observations = []
    for _ in range(n_obs):
        x = rng.integers(30, 226)
        y = rng.integers(30, 226)
        intensity = observer.observe((y, x))
        observations.append((np.array([y, x]), intensity))

    print(f"Generated {len(observations)} observations")

    # Initialize ADE-PSPF
    print("\n=== Initializing ADE-PSPF ===")
    config = ADEPSPFConfig(
        n_swarms=4,
        n_particles=50,  # Reduced for testing
        n_iterations=5,
        ade_generations=3
    )
    estimator = ADEPSPF(config=config, rng=rng)

    # Run estimation
    print("\n=== Running Estimation ===")
    result = estimator.update(observations)

    print(f"\nResults:")
    print(f"  Iteration: {result['iteration']}")
    print(f"  RFC: {result['rfc']:.4f}")
    print(f"  Best RFC: {result['best_rfc']:.4f}")
    print(f"  Number of sources found: {result['n_sources']}")
    print(f"\nEstimated sources:")
    for i, source in enumerate(result['sources']):
        print(f"  Source {i}: pos=({source[1]:.1f}, {source[0]:.1f}), "
              f"intensity={source[2]:.2f}")

    print("\n✅ ADE-PSPF integration test completed!")
