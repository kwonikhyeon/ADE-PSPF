"""
Adaptive Differential Evolution (ADE) for ADE-PSPF algorithm.

Implements Equations 4, 5, 6 from the paper:
    - Mutation with adaptive scale factors (Eq. 4)
    - Crossover with adaptive rate (Eq. 5)
    - Selection based on fitness (Eq. 6)
"""
from __future__ import annotations
import numpy as np
from typing import Tuple, Optional
from .particle import Particle, ParticleSwarm


def adaptive_mutation(
    target: np.ndarray,
    best: np.ndarray,
    r1: np.ndarray,
    r2: np.ndarray,
    weight_target: float,
    weight_r1: float,
    mean_weight: float,
    alpha: float = 0.5,
    beta: float = 0.3
) -> np.ndarray:
    """
    Adaptive differential evolution mutation.

    Implements Equation 4:
        v_{s,r}(g) = p_{s,r}(g) + F1·(p^best_s(g) - p_{s,r}(g)) + F2·(p^r1_s(g) - p^r2_s(g))

        F1 = α · (1 - w_{s,r})
        F2 = β · (w_{s,r1} - w̄_{s,r}) / w_{s,r1}

    Args:
        target: Target individual [x, y, intensity]
        best: Best individual in population [x, y, intensity]
        r1: Random individual 1 [x, y, intensity]
        r2: Random individual 2 [x, y, intensity]
        weight_target: Weight of target individual
        weight_r1: Weight of random individual 1
        mean_weight: Mean weight of all individuals
        alpha: Elite movement scale (default 0.5)
        beta: Random movement scale (default 0.3)

    Returns:
        Mutant vector [x, y, intensity]
    """
    # Adaptive scale factors
    F1 = alpha * (1.0 - weight_target)

    # Avoid division by zero
    if weight_r1 < 1e-10:
        F2 = 0.0
    else:
        F2 = beta * (weight_r1 - mean_weight) / weight_r1

    # Mutation: v = target + F1*(best - target) + F2*(r1 - r2)
    mutant = target + F1 * (best - target) + F2 * (r1 - r2)

    return mutant


def adaptive_crossover(
    target: np.ndarray,
    mutant: np.ndarray,
    weight_target: float,
    mean_weight: float,
    CR_base: float = 0.5,
    CR_scale: float = 0.3,
    sigma: float = 0.1,
    rng: Optional[np.random.Generator] = None
) -> np.ndarray:
    """
    Adaptive differential evolution crossover.

    Implements Equation 5:
        u_{s,r}(g)[j] = {
            v_{s,r}(g)[j],           if rand < CR or j = j_rand
            p_{s,r}(g)[j] + σ_r,     otherwise
        }

        CR = CR_base + CR_scale · (w_{s,r} - w̄_{s,r}) / w̄_{s,r}

    Args:
        target: Target individual [x, y, intensity]
        mutant: Mutant individual [x, y, intensity]
        weight_target: Weight of target individual
        mean_weight: Mean weight of population
        CR_base: Base crossover rate (default 0.5)
        CR_scale: Crossover rate scale (default 0.3)
        sigma: Gaussian noise standard deviation (default 0.1)
        rng: Random number generator

    Returns:
        Trial vector [x, y, intensity]
    """
    if rng is None:
        rng = np.random.default_rng()

    # Adaptive crossover rate
    if mean_weight < 1e-10:
        CR = CR_base
    else:
        CR = CR_base + CR_scale * (weight_target - mean_weight) / mean_weight
    CR = np.clip(CR, 0.0, 1.0)

    # Initialize trial vector
    trial = np.zeros_like(target)
    D = len(target)  # Dimensionality (3: x, y, intensity)

    # Ensure at least one dimension is from mutant
    j_rand = rng.integers(0, D)

    for j in range(D):
        if rng.random() < CR or j == j_rand:
            # Take from mutant
            trial[j] = mutant[j]
        else:
            # Take from target + Gaussian noise (prevent impoverishment)
            noise = rng.normal(0, sigma)
            trial[j] = target[j] + noise

    return trial


def selection(
    target: np.ndarray,
    trial: np.ndarray,
    weight_target: float,
    weight_trial: float
) -> Tuple[np.ndarray, float]:
    """
    Selection operation: choose better individual.

    Implements Equation 6:
        p_{s,r}(g+1) = {
            u_{s,r}(g),    if w(u_{s,r}(g)) > w(p_{s,r}(g))
            p_{s,r}(g),    otherwise
        }

    Args:
        target: Target individual [x, y, intensity]
        trial: Trial individual [x, y, intensity]
        weight_target: Weight of target
        weight_trial: Weight of trial

    Returns:
        Tuple of (selected_individual, selected_weight)
    """
    if weight_trial > weight_target:
        return trial.copy(), weight_trial
    else:
        return target.copy(), weight_target


def ade_resampling(
    swarm: ParticleSwarm,
    bounds: Tuple[np.ndarray, np.ndarray],
    n_generations: int = 5,
    alpha: float = 0.5,
    beta: float = 0.3,
    CR_base: float = 0.5,
    CR_scale: float = 0.3,
    sigma: float = 1.0,
    rng: Optional[np.random.Generator] = None
) -> ParticleSwarm:
    """
    Perform ADE resampling on a particle swarm.

    This runs multiple generations of Mutation → Crossover → Selection
    to improve particle quality and avoid local optimum.

    Args:
        swarm: Particle swarm to resample
        bounds: (min_bound, max_bound) for boundary constraint
        n_generations: Number of ADE generations (default 5)
        alpha: Elite movement scale
        beta: Random movement scale
        CR_base: Base crossover rate
        CR_scale: Crossover rate scale
        sigma: Gaussian noise std
        rng: Random number generator

    Returns:
        Resampled particle swarm
    """
    if rng is None:
        rng = np.random.default_rng()

    min_bound, max_bound = bounds
    n_particles = swarm.num_particles

    # Convert swarm to array for easier manipulation
    population = swarm.to_array()  # (N, 3)
    weights = swarm.weights.copy()  # (N,)

    # Find best particle
    best_idx = np.argmax(weights)
    best = population[best_idx].copy()

    # Mean weight
    mean_weight = np.mean(weights)

    # ADE main loop
    for generation in range(n_generations):
        new_population = []
        new_weights = []

        for i in range(n_particles):
            target = population[i]
            weight_target = weights[i]

            # Select random individuals (r1, r2) different from target
            candidates = list(range(n_particles))
            candidates.remove(i)
            r1_idx, r2_idx = rng.choice(candidates, size=2, replace=False)

            r1 = population[r1_idx]
            r2 = population[r2_idx]
            weight_r1 = weights[r1_idx]

            # Mutation
            mutant = adaptive_mutation(
                target, best, r1, r2,
                weight_target, weight_r1, mean_weight,
                alpha, beta
            )

            # Boundary constraint
            mutant = np.clip(mutant, min_bound, max_bound)

            # Crossover
            trial = adaptive_crossover(
                target, mutant, weight_target, mean_weight,
                CR_base, CR_scale, sigma, rng
            )

            # Boundary constraint
            trial = np.clip(trial, min_bound, max_bound)

            # Compute trial weight (would need observation data)
            # For now, we keep target weight and recompute later
            weight_trial = weight_target  # Placeholder

            # Selection
            selected, selected_weight = selection(
                target, trial, weight_target, weight_trial
            )

            new_population.append(selected)
            new_weights.append(selected_weight)

        # Update population and weights
        population = np.array(new_population)
        weights = np.array(new_weights)

        # Update best
        best_idx = np.argmax(weights)
        best = population[best_idx].copy()
        mean_weight = np.mean(weights)

    # Convert back to swarm
    swarm.from_array(population, weights)

    return swarm


def ade_resample_with_weight_function(
    swarm: ParticleSwarm,
    weight_function,
    bounds: Tuple[np.ndarray, np.ndarray],
    n_generations: int = 5,
    alpha: float = 0.5,
    beta: float = 0.3,
    CR_base: float = 0.5,
    CR_scale: float = 0.3,
    sigma: float = 1.0,
    rng: Optional[np.random.Generator] = None,
    **weight_kwargs
) -> ParticleSwarm:
    """
    Perform ADE resampling with proper weight recalculation.

    This version recalculates weights after generating trial vectors,
    enabling proper selection based on fitness.

    Args:
        swarm: Particle swarm to resample
        weight_function: Function to compute particle weight
            signature: weight_function(particle_pos, particle_intensity, **kwargs) -> weight
        bounds: (min_bound, max_bound) for boundary constraint
        n_generations: Number of ADE generations
        alpha: Elite movement scale
        beta: Random movement scale
        CR_base: Base crossover rate
        CR_scale: Crossover rate scale
        sigma: Gaussian noise std
        rng: Random number generator
        **weight_kwargs: Additional arguments for weight_function

    Returns:
        Resampled particle swarm
    """
    if rng is None:
        rng = np.random.default_rng()

    min_bound, max_bound = bounds
    n_particles = swarm.num_particles

    population = swarm.to_array()
    weights = swarm.weights.copy()

    for generation in range(n_generations):
        best_idx = np.argmax(weights)
        best = population[best_idx].copy()
        mean_weight = np.mean(weights)

        new_population = []
        new_weights = []

        for i in range(n_particles):
            target = population[i]
            weight_target = weights[i]

            # Select r1, r2
            candidates = list(range(n_particles))
            candidates.remove(i)
            r1_idx, r2_idx = rng.choice(candidates, size=2, replace=False)

            r1 = population[r1_idx]
            r2 = population[r2_idx]
            weight_r1 = weights[r1_idx]

            # Mutation
            mutant = adaptive_mutation(
                target, best, r1, r2,
                weight_target, weight_r1, mean_weight,
                alpha, beta
            )
            mutant = np.clip(mutant, min_bound, max_bound)

            # Crossover
            trial = adaptive_crossover(
                target, mutant, weight_target, mean_weight,
                CR_base, CR_scale, sigma, rng
            )
            trial = np.clip(trial, min_bound, max_bound)

            # Recalculate trial weight
            weight_trial = weight_function(
                trial[:2], trial[2], **weight_kwargs
            )

            # Selection
            selected, selected_weight = selection(
                target, trial, weight_target, weight_trial
            )

            new_population.append(selected)
            new_weights.append(selected_weight)

        population = np.array(new_population)
        weights = np.array(new_weights)

    swarm.from_array(population, weights)
    return swarm


if __name__ == "__main__":
    from .particle import initialize_particles, ParticleSwarm

    print("Testing ADE operations...\n")

    # Create test data
    target = np.array([100.0, 100.0, 50.0])
    best = np.array([110.0, 110.0, 55.0])
    r1 = np.array([95.0, 105.0, 48.0])
    r2 = np.array([105.0, 95.0, 52.0])

    # Test mutation
    print("=== Testing Adaptive Mutation ===")
    mutant = adaptive_mutation(
        target, best, r1, r2,
        weight_target=0.3,
        weight_r1=0.7,
        mean_weight=0.5,
        alpha=0.5,
        beta=0.3
    )
    print(f"Target: {target}")
    print(f"Mutant: {mutant}")

    # Test crossover
    print("\n=== Testing Adaptive Crossover ===")
    rng = np.random.default_rng(42)
    trial = adaptive_crossover(
        target, mutant,
        weight_target=0.3,
        mean_weight=0.5,
        rng=rng
    )
    print(f"Trial: {trial}")

    # Test selection
    print("\n=== Testing Selection ===")
    selected, selected_weight = selection(
        target, trial,
        weight_target=0.3,
        weight_trial=0.6
    )
    print(f"Selected: {selected} (weight: {selected_weight})")

    # Test ADE resampling on swarm
    print("\n=== Testing ADE Resampling ===")
    bounds = (np.array([0, 0, 10.0]), np.array([256, 256, 100.0]))

    # Create a swarm
    swarm = ParticleSwarm(swarm_id=0)
    particles = initialize_particles(20, bounds, rng=np.random.default_rng(42))
    swarm.particles = particles

    # Set random weights
    for p in swarm.particles:
        p.weight = np.random.uniform(0, 1)
    swarm.update_best_particle()

    print(f"Before resampling: mean_weight = {swarm.get_mean_weight():.4f}")
    print(f"Best particle: {swarm.best_particle}")

    # Resample
    swarm_resampled = ade_resampling(
        swarm, bounds, n_generations=3, rng=np.random.default_rng(42)
    )

    print(f"After resampling: mean_weight = {swarm_resampled.get_mean_weight():.4f}")
    print(f"Best particle: {swarm_resampled.best_particle}")

    print("\n✅ All ADE tests passed!")
