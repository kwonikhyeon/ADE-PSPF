"""
Particle and ParticleSwarm data structures for ADE-PSPF algorithm.

This module implements the core data structures for representing particles
and particle swarms in the multi-source radiation field estimation.
"""
from __future__ import annotations
import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class Particle:
    """
    Represents a single particle in the particle filter.

    Each particle represents a hypothesis of a radiation source with:
    - position: (x, y) in pixel coordinates
    - intensity: radiation source intensity
    - weight: synthesized weight (combination of observation, distance, and swarm weights)
    """
    x: float
    y: float
    intensity: float
    weight: float = 0.0

    def to_array(self) -> np.ndarray:
        """Convert particle to numpy array [x, y, intensity]."""
        return np.array([self.x, self.y, self.intensity], dtype=np.float64)

    @classmethod
    def from_array(cls, arr: np.ndarray, weight: float = 0.0) -> Particle:
        """Create particle from numpy array [x, y, intensity]."""
        return cls(x=float(arr[0]), y=float(arr[1]), intensity=float(arr[2]), weight=weight)

    def copy(self) -> Particle:
        """Create a deep copy of this particle."""
        return Particle(x=self.x, y=self.y, intensity=self.intensity, weight=self.weight)

    def distance_to(self, other: Particle) -> float:
        """Compute Euclidean distance to another particle (position only)."""
        return np.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def __repr__(self) -> str:
        return f"Particle(x={self.x:.2f}, y={self.y:.2f}, I={self.intensity:.2f}, w={self.weight:.4f})"


@dataclass
class ParticleSwarm:
    """
    Represents a swarm of particles estimating a single radiation source.

    Attributes:
        swarm_id: Unique identifier for this swarm
        particles: List of particles in this swarm
        centroid: Current centroid (mean position) of the swarm
        best_particle: Particle with highest weight
        confidence: Confidence score for this swarm
    """
    swarm_id: int
    particles: list[Particle] = field(default_factory=list)
    centroid: Optional[np.ndarray] = None  # [x, y, intensity]
    best_particle: Optional[Particle] = None
    confidence: float = 0.0

    def __post_init__(self):
        """Initialize swarm after creation."""
        if self.particles:
            self.update_best_particle()

    @property
    def num_particles(self) -> int:
        """Get number of particles in this swarm."""
        return len(self.particles)

    @property
    def weights(self) -> np.ndarray:
        """Get all particle weights as numpy array."""
        return np.array([p.weight for p in self.particles], dtype=np.float64)

    @property
    def positions(self) -> np.ndarray:
        """Get all particle positions as (N, 2) array."""
        return np.array([[p.x, p.y] for p in self.particles], dtype=np.float64)

    @property
    def intensities(self) -> np.ndarray:
        """Get all particle intensities as (N,) array."""
        return np.array([p.intensity for p in self.particles], dtype=np.float64)

    def to_array(self) -> np.ndarray:
        """Convert all particles to (N, 3) array [x, y, intensity]."""
        return np.array([p.to_array() for p in self.particles], dtype=np.float64)

    def from_array(self, arr: np.ndarray, weights: Optional[np.ndarray] = None):
        """
        Update particles from (N, 3) array.

        Args:
            arr: (N, 3) array of [x, y, intensity]
            weights: Optional (N,) array of weights
        """
        if weights is None:
            weights = np.zeros(len(arr))

        self.particles = [
            Particle.from_array(arr[i], weight=weights[i])
            for i in range(len(arr))
        ]
        self.update_best_particle()

    def update_best_particle(self):
        """Update the best particle (highest weight)."""
        if not self.particles:
            self.best_particle = None
            return

        self.best_particle = max(self.particles, key=lambda p: p.weight)

    def update_centroid(self, centroid: np.ndarray):
        """
        Update swarm centroid.

        Args:
            centroid: [x, y, intensity] array
        """
        self.centroid = centroid.copy()

    def get_mean_weight(self) -> float:
        """Get mean weight of all particles."""
        if not self.particles:
            return 0.0
        return float(np.mean(self.weights))

    def normalize_weights(self):
        """Normalize weights to sum to 1."""
        weights = self.weights
        if np.sum(weights) > 0:
            weights /= np.sum(weights)
            for i, p in enumerate(self.particles):
                p.weight = weights[i]

    def effective_sample_size(self) -> float:
        """
        Compute effective sample size (ESS).

        ESS = 1 / Σ(w_i^2)

        Returns:
            Effective sample size (N_eff)
        """
        weights = self.weights
        if np.sum(weights) == 0:
            return 0.0

        # Normalize first
        weights = weights / np.sum(weights)
        return 1.0 / np.sum(weights ** 2)

    def resample_if_needed(self, threshold: float = 0.5):
        """
        Check if resampling is needed based on ESS.

        Args:
            threshold: Resample if N_eff < threshold * N

        Returns:
            True if resampling is needed
        """
        n_eff = self.effective_sample_size()
        return n_eff < threshold * self.num_particles

    def __repr__(self) -> str:
        centroid_str = "None"
        if self.centroid is not None:
            centroid_str = f"[{self.centroid[0]:.2f}, {self.centroid[1]:.2f}, {self.centroid[2]:.2f}]"

        return (f"ParticleSwarm(id={self.swarm_id}, N={self.num_particles}, "
                f"centroid={centroid_str}, conf={self.confidence:.4f})")


@dataclass
class Configuration:
    """
    Represents a configuration snapshot for Configuration Maintenance.

    This stores the complete state of all swarms at a given iteration,
    allowing rollback when RFC decreases.
    """
    particles: list[np.ndarray]  # List of (N_p, 3) arrays for each swarm
    centroids: list[np.ndarray]  # List of (3,) arrays [x, y, intensity]
    confidence: float  # RFC value
    iteration: int = 0

    def is_valid(self) -> bool:
        """
        Check if this configuration is valid.

        A configuration is valid if it has at least one non-None centroid.

        Returns:
            True if configuration has at least one valid centroid
        """
        return any(c is not None for c in self.centroids)

    def copy(self) -> Configuration:
        """Create a deep copy of this configuration."""
        return Configuration(
            particles=[p.copy() for p in self.particles],
            centroids=[c.copy() if c is not None else None for c in self.centroids],
            confidence=self.confidence,
            iteration=self.iteration
        )

    def __repr__(self) -> str:
        return (f"Configuration(iter={self.iteration}, "
                f"N_swarms={len(self.centroids)}, RFC={self.confidence:.4f})")


def initialize_particles(
    n_particles: int,
    bounds: Tuple[np.ndarray, np.ndarray],
    rng: Optional[np.random.Generator] = None
) -> list[Particle]:
    """
    Initialize particles uniformly within bounds.

    Args:
        n_particles: Number of particles to create
        bounds: (min_bound, max_bound) where each is [x_min, y_min, I_min]
        rng: Random number generator (optional)

    Returns:
        List of initialized particles
    """
    if rng is None:
        rng = np.random.default_rng()

    min_bound, max_bound = bounds
    particles = []

    for _ in range(n_particles):
        # Uniform sampling in configuration space
        x = rng.uniform(min_bound[0], max_bound[0])
        y = rng.uniform(min_bound[1], max_bound[1])
        intensity = rng.uniform(min_bound[2], max_bound[2])

        particles.append(Particle(x=x, y=y, intensity=intensity, weight=0.0))

    return particles


def initialize_swarms(
    n_swarms: int,
    n_particles_per_swarm: int,
    bounds: Tuple[np.ndarray, np.ndarray],
    rng: Optional[np.random.Generator] = None
) -> list[ParticleSwarm]:
    """
    Initialize multiple particle swarms.

    Args:
        n_swarms: Number of swarms (N_ps)
        n_particles_per_swarm: Number of particles per swarm (N_p)
        bounds: (min_bound, max_bound) for particle initialization
        rng: Random number generator (optional)

    Returns:
        List of initialized particle swarms
    """
    swarms = []

    for swarm_id in range(n_swarms):
        particles = initialize_particles(n_particles_per_swarm, bounds, rng)
        swarm = ParticleSwarm(swarm_id=swarm_id, particles=particles)
        swarms.append(swarm)

    return swarms


if __name__ == "__main__":
    # Test particle and swarm implementation
    print("Testing Particle and ParticleSwarm implementation...\n")

    # Test Particle
    print("=== Testing Particle ===")
    p1 = Particle(x=100.0, y=150.0, intensity=50.0, weight=0.8)
    p2 = Particle(x=110.0, y=160.0, intensity=55.0, weight=0.6)
    print(f"Particle 1: {p1}")
    print(f"Particle 2: {p2}")
    print(f"Distance: {p1.distance_to(p2):.2f}")
    print(f"Array representation: {p1.to_array()}")

    # Test ParticleSwarm
    print("\n=== Testing ParticleSwarm ===")
    bounds = (np.array([0, 0, 10.0]), np.array([256, 256, 100.0]))
    swarm = ParticleSwarm(swarm_id=0)
    particles = initialize_particles(10, bounds, rng=np.random.default_rng(42))
    swarm.particles = particles

    # Set some weights
    for i, p in enumerate(swarm.particles):
        p.weight = np.random.uniform(0, 1)
    swarm.update_best_particle()

    print(f"Swarm: {swarm}")
    print(f"Number of particles: {swarm.num_particles}")
    print(f"Mean weight: {swarm.get_mean_weight():.4f}")
    print(f"Best particle: {swarm.best_particle}")
    print(f"ESS: {swarm.effective_sample_size():.2f}")
    print(f"Need resampling: {swarm.resample_if_needed()}")

    # Test Configuration
    print("\n=== Testing Configuration ===")
    config = Configuration(
        particles=[swarm.to_array()],
        centroids=[np.array([128.0, 128.0, 50.0])],
        confidence=0.92,
        iteration=10
    )
    print(f"Configuration: {config}")
    config_copy = config.copy()
    print(f"Configuration copy: {config_copy}")

    # Test initialize_swarms
    print("\n=== Testing initialize_swarms ===")
    swarms = initialize_swarms(
        n_swarms=3,
        n_particles_per_swarm=5,
        bounds=bounds,
        rng=np.random.default_rng(42)
    )
    for swarm in swarms:
        print(f"  {swarm}")

    print("\n✅ All tests passed!")
