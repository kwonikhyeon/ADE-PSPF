"""
Data Manager Module

Handles data structures for storing exploration snapshots and
provides save/load functionality for session management.
"""
from __future__ import annotations

import pickle
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass, field
from pathlib import Path

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from simulation.exploration.rrt_planner import RRTNode, Pose2D
from simulation.integrated_explorer_v2 import ExplorationPhase


@dataclass
class EstimationStepData:
    """Detailed data from estimation step (ADE-PSPF)."""

    # Input
    observations: List[Tuple[np.ndarray, float]]
    n_observations: int

    # Output
    estimated_sources: List[np.ndarray]
    n_sources: int
    rfc: float
    best_rfc: float

    # Internal state (for visualization)
    particles_by_swarm: List[np.ndarray]  # List of (n_particles, 3) arrays [x, y, intensity]
    swarm_centroids: List[np.ndarray]  # List of (3,) arrays
    swarm_weights: List[float]  # Weight of each swarm
    n_swarms: int

    # Timing
    time_elapsed: float = 0.0


@dataclass
class ExplorationStepData:
    """Detailed data from exploration step (RRT planning)."""

    # Input
    robot_pose: Pose2D
    estimated_sources: List[np.ndarray]
    observation_positions: np.ndarray
    observation_intensities: np.ndarray

    # RRT Tree
    rrt_nodes: List[RRTNode]
    rrt_leaves: List[int]  # Indices of leaf nodes
    n_nodes: int

    # Best path
    best_branch: List[RRTNode]
    best_leaf_idx: int
    best_gain: float

    # Gain information
    node_gains: List[float]  # Gain value for each node
    cumulative_gains: List[float]  # Cumulative gain for each node

    # Timing
    time_elapsed: float = 0.0


@dataclass
class ExecutionStepData:
    """Detailed data from execution step (movement and observation)."""

    # Before execution
    pose_before: Pose2D
    planned_path: List[RRTNode]
    n_planned_nodes: int

    # Execution plan
    n_nodes_to_execute: int
    planned_distance: float

    # After execution
    pose_after: Pose2D
    actual_distance: float
    new_observations: List[Tuple[np.ndarray, float]]
    n_new_observations: int

    # Movement details
    waypoints: List[Pose2D]  # All intermediate poses
    observation_locations: List[Pose2D]  # Where observations were made

    # Timing
    time_elapsed: float = 0.0


@dataclass
class IterationData:
    """Complete data snapshot for one exploration iteration."""

    iteration: int  # 0-indexed

    # Phase information (with defaults for easy initialization)
    phase: ExplorationPhase = None
    rfc_before: float = 0.0
    rfc_after: float = 0.0

    # Step data
    estimation_data: Optional[EstimationStepData] = None
    exploration_data: Optional[ExplorationStepData] = None
    execution_data: Optional[ExecutionStepData] = None

    # Cumulative statistics
    cumulative_observations: int = 0
    total_distance_traveled: float = 0.0

    # Timing
    time_total: float = 0.0

    def has_step(self, step_name: str) -> bool:
        """Check if step data is available."""
        if step_name.lower() == 'estimation':
            return self.estimation_data is not None
        elif step_name.lower() == 'exploration':
            return self.exploration_data is not None
        elif step_name.lower() == 'execution':
            return self.execution_data is not None
        return False


@dataclass
class ExplorationSnapshot:
    """Complete snapshot of an exploration session."""

    # Ground truth
    gt_field: np.ndarray
    true_sources: np.ndarray  # (n_sources, 3) [x, y, intensity]
    n_true_sources: int
    grid_size: int
    pixel_resolution: float

    # Configuration
    random_seed: int
    config_dict: Dict[str, Any]

    # Exploration data
    iteration_data: List[IterationData]
    n_iterations: int

    # Final results
    final_estimated_sources: List[np.ndarray]
    final_rfc: float
    best_rfc: float
    converged: bool
    convergence_iteration: Optional[int]

    # Robot trajectory
    full_trajectory: List[Tuple[float, float]]
    final_pose: Pose2D

    # All observations
    all_observations: List[Tuple[np.ndarray, float]]

    # Statistics
    total_time: float
    total_distance: float

    # Phase history
    phase_history: List[Tuple[int, ExplorationPhase]]  # (iteration, phase)


class DataManager:
    """Manages saving and loading of exploration sessions."""

    def __init__(self):
        self.current_snapshot: Optional[ExplorationSnapshot] = None

    def save_snapshot(self, snapshot: ExplorationSnapshot, filepath: str) -> bool:
        """
        Save exploration snapshot to file.

        Args:
            snapshot: ExplorationSnapshot to save
            filepath: Path to save file (.pkl)

        Returns:
            True if successful, False otherwise
        """
        try:
            filepath = Path(filepath)
            filepath.parent.mkdir(parents=True, exist_ok=True)

            with open(filepath, 'wb') as f:
                pickle.dump(snapshot, f, protocol=pickle.HIGHEST_PROTOCOL)

            print(f"✓ Snapshot saved to: {filepath}")
            return True

        except Exception as e:
            print(f"✗ Failed to save snapshot: {e}")
            return False

    def load_snapshot(self, filepath: str) -> Optional[ExplorationSnapshot]:
        """
        Load exploration snapshot from file.

        Args:
            filepath: Path to snapshot file (.pkl)

        Returns:
            ExplorationSnapshot if successful, None otherwise
        """
        try:
            filepath = Path(filepath)

            if not filepath.exists():
                print(f"✗ File not found: {filepath}")
                return None

            with open(filepath, 'rb') as f:
                snapshot = pickle.load(f)

            if not isinstance(snapshot, ExplorationSnapshot):
                print(f"✗ Invalid snapshot file format")
                return None

            print(f"✓ Snapshot loaded from: {filepath}")
            self.current_snapshot = snapshot
            return snapshot

        except Exception as e:
            print(f"✗ Failed to load snapshot: {e}")
            return None

    def export_iteration_images(
        self,
        snapshot: ExplorationSnapshot,
        output_dir: str,
        render_func: callable
    ) -> bool:
        """
        Export each iteration as an image.

        Args:
            snapshot: ExplorationSnapshot to export
            output_dir: Directory to save images
            render_func: Function(iteration_data, output_path) to render image

        Returns:
            True if successful, False otherwise
        """
        try:
            output_dir = Path(output_dir)
            output_dir.mkdir(parents=True, exist_ok=True)

            for i, iter_data in enumerate(snapshot.iteration_data):
                output_path = output_dir / f"iteration_{i+1:03d}.png"
                render_func(iter_data, output_path)

            print(f"✓ Exported {len(snapshot.iteration_data)} images to: {output_dir}")
            return True

        except Exception as e:
            print(f"✗ Failed to export images: {e}")
            return False

    def get_statistics_summary(self, snapshot: ExplorationSnapshot) -> Dict[str, Any]:
        """
        Generate summary statistics from snapshot.

        Args:
            snapshot: ExplorationSnapshot to analyze

        Returns:
            Dictionary with summary statistics
        """
        return {
            'n_iterations': snapshot.n_iterations,
            'n_observations': len(snapshot.all_observations),
            'total_time': snapshot.total_time,
            'total_distance': snapshot.total_distance,
            'final_rfc': snapshot.final_rfc,
            'best_rfc': snapshot.best_rfc,
            'converged': snapshot.converged,
            'convergence_iteration': snapshot.convergence_iteration,
            'n_true_sources': snapshot.n_true_sources,
            'n_estimated_sources': len(snapshot.final_estimated_sources),
            'random_seed': snapshot.random_seed
        }


# Helper functions for data extraction

def extract_particles_from_swarms(swarms) -> Tuple[List[np.ndarray], List[np.ndarray], List[float]]:
    """
    Extract particle data from ADE-PSPF swarms.

    Args:
        swarms: List of Swarm objects from ADEPSPF

    Returns:
        Tuple of (particles_by_swarm, centroids, weights)
    """
    particles_by_swarm = []
    centroids = []
    weights = []

    for swarm in swarms:
        # Get particles (list of Particle objects)
        particles = swarm.particles

        # Extract positions and intensities
        particle_array = np.array([
            [p.x, p.y, p.intensity] for p in particles
        ], dtype=np.float64)

        particles_by_swarm.append(particle_array)

        # Get centroid
        if hasattr(swarm, 'configuration') and swarm.configuration is not None:
            centroid = swarm.configuration.centroid
            if centroid is not None:
                centroids.append(centroid)
            else:
                # Fallback: compute mean
                centroids.append(particle_array.mean(axis=0))
        else:
            centroids.append(particle_array.mean(axis=0))

        # Get weight
        if hasattr(swarm, 'weight'):
            weights.append(float(swarm.weight))
        else:
            weights.append(1.0 / len(swarms))

    return particles_by_swarm, centroids, weights


def extract_rrt_gain_values(nodes: List[RRTNode]) -> Tuple[List[float], List[float]]:
    """
    Extract gain values from RRT nodes.

    Args:
        nodes: List of RRTNode objects

    Returns:
        Tuple of (node_gains, cumulative_gains)
    """
    node_gains = [node.node_gain for node in nodes]
    cumulative_gains = [node.cumulative_gain for node in nodes]

    return node_gains, cumulative_gains
