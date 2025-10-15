"""Core module for ADE-PSPF algorithm."""

from .particle import Particle, ParticleSwarm, Configuration, initialize_swarms
from .weights import (
    poisson_pmf,
    predict_intensity,
    observation_weight,
    peak_suppression_weight,
    swarm_correction_weight,
    synthesized_weight,
)
from .ade import (
    adaptive_mutation,
    adaptive_crossover,
    selection,
    ade_resampling,
    ade_resample_with_weight_function,
)
from .clustering import (
    mean_shift_clustering,
    weighted_mean_shift,
    filter_redundant_swarms,
    adaptive_bandwidth,
)
from .rfc import (
    compute_cumulative_intensity,
    compute_rfc,
    ConfigurationManager,
)

__all__ = [
    # Particle structures
    "Particle",
    "ParticleSwarm",
    "Configuration",
    "initialize_swarms",
    # Weight functions
    "poisson_pmf",
    "predict_intensity",
    "observation_weight",
    "peak_suppression_weight",
    "swarm_correction_weight",
    "synthesized_weight",
    # ADE operations
    "adaptive_mutation",
    "adaptive_crossover",
    "selection",
    "ade_resampling",
    "ade_resample_with_weight_function",
    # Clustering
    "mean_shift_clustering",
    "weighted_mean_shift",
    "filter_redundant_swarms",
    "adaptive_bandwidth",
    # RFC and configuration
    "compute_cumulative_intensity",
    "compute_rfc",
    "ConfigurationManager",
]
