"""
Integrated Multi-Source Radiation Exploration System.

Implements the complete Receding Horizon Framework:
    Observation → Estimation (ADE-PSPF) → Exploration (RRT) → Move → Repeat

This module integrates all components:
    - Ground Truth Generation
    - Radiation Observation
    - Source Estimation (ADE-PSPF)
    - Path Planning (RRT)
    - Robot Motion
"""
from __future__ import annotations
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field
from pathlib import Path

from environment.generate_truth import sample_sources, inverse_square_field, GRID
from environment.observation import RadiationObserver
from core.ade_pspf import ADEPSPF, ADEPSPFConfig
from simulation.exploration.rrt_planner import (
    RRTPlanner, PlannerConfig, RadiationGainModel, GainParameters,
    PlanarEnvironment, Pose2D, RRTNode
)


@dataclass
class ExplorationConfig:
    """Configuration for integrated exploration system."""

    # Environment
    grid_size: int = 256
    pixel_resolution: float = 0.04  # meters per pixel

    # Robot parameters
    robot_start_x: float = 128.0  # pixels
    robot_start_y: float = 128.0
    robot_start_theta: float = 0.0

    # Exploration parameters
    max_iterations: int = 20  # Maximum exploration iterations
    branch_execution_ratio: float = 0.3  # Execute 30% of best branch
    min_rfc_threshold: float = 0.85  # Stop if RFC exceeds this

    # Observation parameters
    observations_per_iteration: int = 5  # Observations along the path

    # ADE-PSPF parameters
    adepspf_config: Optional[ADEPSPFConfig] = None

    # RRT parameters
    rrt_config: Optional[PlannerConfig] = None
    gain_params: Optional[GainParameters] = None


@dataclass
class RobotState:
    """Robot state in the exploration."""
    pose: Pose2D
    trajectory: List[Tuple[float, float]] = field(default_factory=list)
    observations: List[Tuple[np.ndarray, float]] = field(default_factory=list)

    def add_observation(self, position: np.ndarray, intensity: float):
        """Add a new observation."""
        self.observations.append((position, intensity))

    def move_to(self, x: float, y: float, theta: float):
        """Update robot pose and record trajectory."""
        self.trajectory.append((self.pose.x, self.pose.y))
        self.pose = Pose2D(x=x, y=y, theta=theta)


@dataclass
class IterationResult:
    """Results from one exploration iteration."""
    iteration: int
    robot_pose: Pose2D
    estimated_sources: List[np.ndarray]
    rfc: float
    best_branch: List[RRTNode]
    observations_made: int
    cumulative_observations: int


class IntegratedExplorer:
    """
    Integrated Multi-Source Radiation Exploration System.

    Implements the receding horizon framework:
    1. Observe: Collect radiation measurements
    2. Estimate: Run ADE-PSPF to estimate sources
    3. Explore: Plan path using RRT with radiation gain
    4. Move: Execute part of the best path
    5. Repeat until convergence or max iterations
    """

    def __init__(
        self,
        gt_field: np.ndarray,
        config: ExplorationConfig,
        rng: Optional[np.random.Generator] = None
    ):
        """
        Initialize integrated explorer.

        Args:
            gt_field: Ground truth radiation field
            config: Exploration configuration
            rng: Random number generator
        """
        self.config = config
        self.rng = rng or np.random.default_rng()

        # Ground truth and observer
        self.gt_field = gt_field
        self.observer = RadiationObserver(gt_field, config.grid_size)

        # Robot state
        self.robot = RobotState(
            pose=Pose2D(
                x=config.robot_start_x,
                y=config.robot_start_y,
                theta=config.robot_start_theta
            )
        )

        # ADE-PSPF estimator
        adepspf_config = config.adepspf_config or ADEPSPFConfig(
            n_swarms=4,
            n_particles=100,  # Reduced for faster execution
            n_iterations=1,  # 1 iteration per exploration step
            ade_generations=3  # Reduced for faster execution
        )
        self.estimator = ADEPSPF(config=adepspf_config, rng=self.rng)

        # RRT planner
        environment = PlanarEnvironment(
            bounds=(0.0, float(config.grid_size), 0.0, float(config.grid_size)),
            obstacles=[]
        )
        gain_params = config.gain_params or GainParameters()
        gain_model = RadiationGainModel(gain_params)
        rrt_config = config.rrt_config or PlannerConfig(
            n_uniform=8,
            max_nodes=80,  # Reduced for faster execution
            min_step=3.0,
            max_step=8.0
        )
        self.planner = RRTPlanner(environment, gain_model, rrt_config)

        # History
        self.iteration_results: List[IterationResult] = []
        self.previous_best_branch: Optional[List[RRTNode]] = None

    def run_exploration(self) -> Dict:
        """
        Run complete exploration process.

        Returns:
            Dictionary with exploration results and statistics
        """
        print(f"Starting integrated exploration...")
        print(f"Initial position: ({self.robot.pose.x:.1f}, {self.robot.pose.y:.1f})")

        # Initial observation at starting position
        self._make_observation(self.robot.pose.x, self.robot.pose.y)

        for iteration in range(self.config.max_iterations):
            print(f"\n{'='*60}")
            print(f"Iteration {iteration + 1}/{self.config.max_iterations}")
            print(f"{'='*60}")

            # Step 1: Estimation (ADE-PSPF)
            result = self._estimation_step()

            # Check convergence
            if result['rfc'] >= self.config.min_rfc_threshold:
                print(f"\n✓ Converged! RFC={result['rfc']:.4f} >= {self.config.min_rfc_threshold}")
                break

            # Step 2: Exploration (RRT path planning)
            best_branch = self._exploration_step(result['sources'])

            if not best_branch or len(best_branch) <= 1:
                print(f"⚠ No valid path found, stopping exploration")
                break

            # Step 3: Execute part of the best path
            observations_made = self._execution_step(best_branch)

            # Record iteration result
            iteration_result = IterationResult(
                iteration=iteration + 1,
                robot_pose=self.robot.pose,
                estimated_sources=result['sources'],
                rfc=result['rfc'],
                best_branch=best_branch,
                observations_made=observations_made,
                cumulative_observations=len(self.robot.observations)
            )
            self.iteration_results.append(iteration_result)

            # Update previous best branch for next iteration
            self.previous_best_branch = best_branch

            print(f"\nIteration {iteration + 1} complete:")
            print(f"  - Estimated sources: {len(result['sources'])}")
            print(f"  - RFC: {result['rfc']:.4f}")
            print(f"  - Observations made: {observations_made}")
            print(f"  - Total observations: {len(self.robot.observations)}")
            print(f"  - Robot position: ({self.robot.pose.x:.1f}, {self.robot.pose.y:.1f})")

        # Final estimation
        final_result = self._estimation_step()

        return self._compile_results(final_result)

    def _make_observation(self, x: float, y: float):
        """Make a radiation observation at given position."""
        # Convert to integer indices for grid access
        x_int = int(round(x))
        y_int = int(round(y))
        intensity = self.observer.observe((y_int, x_int))  # Note: (row, col) = (y, x)
        self.robot.add_observation(np.array([y, x]), intensity)

    def _estimation_step(self) -> Dict:
        """
        Run ADE-PSPF estimation step.

        Returns:
            Estimation results with sources and RFC
        """
        print(f"\n[ESTIMATION] Running ADE-PSPF...")
        print(f"  Observations: {len(self.robot.observations)}")

        import time
        start = time.time()
        result = self.estimator.update(self.robot.observations)
        elapsed = time.time() - start

        print(f"  Estimation completed in {elapsed:.2f}s")
        print(f"  RFC: {result['rfc']:.4f}")
        print(f"  Best RFC: {result['best_rfc']:.4f}")
        print(f"  Sources found: {result['n_sources']}")

        return result

    def _exploration_step(self, estimated_sources: List[np.ndarray]) -> List[RRTNode]:
        """
        Plan exploration path using RRT.

        Args:
            estimated_sources: Estimated source parameters

        Returns:
            Best branch from RRT planning
        """
        print(f"\n[EXPLORATION] Planning path with RRT...")

        if len(estimated_sources) == 0:
            print(f"  ⚠ No sources estimated, random exploration")
            estimated_sources = [np.array([128.0, 128.0, 50.0])]

        # Convert sources to numpy array
        sources = np.array(estimated_sources)

        # Extract observation data
        if len(self.robot.observations) > 0:
            obs_positions = np.array([obs[0] for obs in self.robot.observations])
            obs_intensities = np.array([obs[1] for obs in self.robot.observations])
        else:
            obs_positions = np.array([]).reshape(0, 2)
            obs_intensities = np.array([])

        # Nearest intensity lookup function
        def nearest_intensity_lookup(pose: Pose2D):
            pos = pose.as_array()

            # Find nearest observation
            if len(self.robot.observations) > 0:
                distances = np.linalg.norm(obs_positions - pos[None, :], axis=1)
                nearest_idx = np.argmin(distances)
                nearest_intensity = obs_intensities[nearest_idx]
                distance_near = distances[nearest_idx]

                # Count samples within certain radius
                radius = 20.0  # pixels
                samples_near = np.sum(distances < radius)
            else:
                nearest_intensity = 0.0
                distance_near = 1000.0
                samples_near = 0

            return nearest_intensity, samples_near, distance_near

        # Build RRT tree
        print(f"  Building RRT tree...")
        import time
        start_time = time.time()

        nodes, leaves = self.planner.build_tree(
            root_pose=self.robot.pose,
            sources=sources,
            observation_positions=obs_positions,
            observation_intensities=obs_intensities,
            nearest_intensity_lookup=nearest_intensity_lookup,
            previous_best_branch=self.previous_best_branch
        )

        elapsed = time.time() - start_time
        print(f"  RRT completed in {elapsed:.2f}s")
        print(f"  Nodes generated: {len(nodes)}")
        print(f"  Leaf nodes: {len(leaves)}")

        # Select best branch
        if leaves:
            best_branch, best_leaf = self.planner.select_best_branch(nodes, leaves)
            best_gain = nodes[best_leaf].cumulative_gain
            print(f"  Best branch: {len(best_branch)} nodes, gain={best_gain:.4f}")
            return best_branch
        else:
            return []

    def _execution_step(self, best_branch: List[RRTNode]) -> int:
        """
        Execute part of the best path and make observations.

        Args:
            best_branch: Best branch from RRT

        Returns:
            Number of observations made
        """
        print(f"\n[EXECUTION] Moving along path...")

        # Calculate how many nodes to execute
        n_nodes = max(1, int(len(best_branch) * self.config.branch_execution_ratio))
        n_nodes = min(n_nodes, len(best_branch) - 1)  # Don't go beyond end

        print(f"  Executing {n_nodes}/{len(best_branch)} nodes")

        observations_made = 0

        # Execute nodes along the path
        for i in range(1, n_nodes + 1):  # Skip root (index 0)
            node = best_branch[i]

            # Move robot to node position
            self.robot.move_to(node.pose.x, node.pose.y, node.pose.theta)

            # Make observations at intervals
            if i % max(1, n_nodes // self.config.observations_per_iteration) == 0:
                self._make_observation(node.pose.x, node.pose.y)
                observations_made += 1

        # Always make observation at final position
        if observations_made == 0:
            self._make_observation(self.robot.pose.x, self.robot.pose.y)
            observations_made += 1

        print(f"  Final position: ({self.robot.pose.x:.1f}, {self.robot.pose.y:.1f})")
        print(f"  Observations made: {observations_made}")

        return observations_made

    def _compile_results(self, final_result: Dict) -> Dict:
        """Compile final exploration results."""
        return {
            'iterations': len(self.iteration_results),
            'final_sources': final_result['sources'],
            'final_rfc': final_result['rfc'],
            'best_rfc': final_result['best_rfc'],
            'total_observations': len(self.robot.observations),
            'trajectory': self.robot.trajectory,
            'final_pose': self.robot.pose,
            'iteration_history': self.iteration_results,
            'observations': self.robot.observations
        }


if __name__ == "__main__":
    print("Testing Integrated Explorer...\n")

    # Generate ground truth
    print("=== Generating Ground Truth ===")
    rng = np.random.default_rng(42)
    n_sources = 2  # Reduced from 3 for faster testing
    coords, amps, sigmas = sample_sources(GRID, n_sources, rng=rng)
    gt_field = inverse_square_field(GRID, coords, amps, h=0.5)

    print(f"True sources:")
    for i in range(n_sources):
        print(f"  Source {i}: pos=({coords[i,1]:.1f}, {coords[i,0]:.1f}), "
              f"intensity={amps[i]:.2f}")

    # Create explorer
    print("\n=== Initializing Explorer ===")

    # Use smaller configuration for faster execution
    from core.ade_pspf import ADEPSPFConfig
    adepspf_config = ADEPSPFConfig(
        n_swarms=3,  # Reduced from default 4
        n_particles=50,  # Reduced from default 100
        n_iterations=1,
        ade_generations=3  # Reduced from default
    )

    config = ExplorationConfig(
        max_iterations=5,  # Reduced from 10
        branch_execution_ratio=0.5,  # Increased to move more
        min_rfc_threshold=0.85,
        observations_per_iteration=2,  # Reduced from default
        adepspf_config=adepspf_config
    )
    explorer = IntegratedExplorer(gt_field, config, rng=rng)

    # Run exploration
    print("\n=== Running Exploration ===")
    results = explorer.run_exploration()

    # Print results
    print("\n" + "="*60)
    print("EXPLORATION COMPLETED")
    print("="*60)
    print(f"Total iterations: {results['iterations']}")
    print(f"Total observations: {results['total_observations']}")
    print(f"Final RFC: {results['final_rfc']:.4f}")
    print(f"Best RFC: {results['best_rfc']:.4f}")
    print(f"Sources found: {len(results['final_sources'])}")
    print(f"\nFinal estimated sources:")
    for i, source in enumerate(results['final_sources']):
        print(f"  Source {i}: pos=({source[1]:.1f}, {source[0]:.1f}), "
              f"intensity={source[2]:.2f}")

    print("\n✅ Integrated exploration test completed!")
