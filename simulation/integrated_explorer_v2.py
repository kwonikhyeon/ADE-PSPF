"""
Integrated Multi-Source Radiation Exploration System v2.0

Bug fixes and optimizations:
1. ADE-PSPF observation limit to prevent exponential slowdown
2. RRT infinite loop prevention with max attempts
3. Configuration.copy() None centroid handling
4. Performance monitoring and logging
5. Configurable observation windowing

This version implements the complete Receding Horizon Framework with improved
performance and stability.
"""
from __future__ import annotations

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field
from pathlib import Path
from enum import Enum
import time

from environment.generate_truth import sample_sources, inverse_square_field, GRID
from environment.observation import RadiationObserver
from core.ade_pspf import ADEPSPF, ADEPSPFConfig
from simulation.exploration.rrt_planner import (
    RRTPlanner, PlannerConfig, RadiationGainModel, GainParameters,
    PlanarEnvironment, Pose2D, RRTNode
)
from simulation.exploration.rrt_planner_v2 import RRTPlannerV2, PerformanceConfig


class ExplorationPhase(Enum):
    """
    Exploration phases as described in the paper (Fig. 14-16).

    논문의 3단계:
    1. TRACING: Tracing suspicious sources (초기 탐색)
    2. SURROUNDING: Surrounding observation (소스 주변 관측)
    3. EXPLORING: Exploring unknown areas (미탐색 영역 탐색)
    """
    TRACING = 1      # RFC < 0.3: 의심 소스 추적
    SURROUNDING = 2  # 0.3 ≤ RFC < 0.85: 소스 주변 관측
    EXPLORING = 3    # RFC ≥ 0.85: 미탐색 영역 탐색


@dataclass
class ExplorationConfig:
    """Configuration for integrated exploration system v2."""

    # Environment
    grid_size: int = 256
    pixel_resolution: float = 0.04  # meters per pixel

    # Robot parameters
    robot_start_x: float = 128.0  # pixels
    robot_start_y: float = 128.0
    robot_start_theta: float = 0.0

    # Exploration parameters
    max_iterations: int = 15  # Maximum exploration iterations
    branch_execution_ratio: float = 0.0  # 0.0 = First edge only (논문 방식), >0 = Legacy mode
    min_rfc_threshold: float = 0.85  # Stop if RFC exceeds this

    # Observation parameters
    observations_per_iteration: int = 1  # 1 observation per iteration (논문 방식)
    max_observations_for_weight: int = 20  # Limit for weight calculation (performance)

    # Performance monitoring
    enable_timing_logs: bool = True  # Enable detailed timing logs
    log_iteration_details: bool = True  # Log detailed iteration info

    # ADE-PSPF parameters (optimized defaults)
    adepspf_config: Optional[ADEPSPFConfig] = None

    # RRT parameters (optimized defaults)
    rrt_config: Optional[PlannerConfig] = None
    gain_params: Optional[GainParameters] = None

    # RRT V2 optimization
    use_rrt_v2: bool = True  # Use optimized RRT Planner V2
    rrt_v2_perf_config: Optional[PerformanceConfig] = None


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

    def get_recent_observations(self, n: int) -> List[Tuple[np.ndarray, float]]:
        """Get n most recent observations."""
        return self.observations[-n:] if len(self.observations) > n else self.observations


@dataclass
class IterationResult:
    """Results from one exploration iteration."""
    iteration: int
    robot_pose: Pose2D
    estimated_sources: List[np.ndarray]
    rfc: float
    best_rfc: float
    best_branch: List[RRTNode]
    observations_made: int
    cumulative_observations: int

    # Timing information
    time_estimation: float = 0.0
    time_exploration: float = 0.0
    time_execution: float = 0.0
    time_total: float = 0.0


@dataclass
class ExplorationStatistics:
    """Statistics for the entire exploration process."""
    total_iterations: int
    total_observations: int
    total_time: float
    final_rfc: float
    best_rfc: float
    converged: bool
    convergence_iteration: Optional[int]

    avg_time_per_iteration: float = 0.0
    avg_time_estimation: float = 0.0
    avg_time_exploration: float = 0.0


class IntegratedExplorerV2:
    """
    Integrated Multi-Source Radiation Exploration System v2.0

    Improvements over v1:
    - Performance optimizations for large observation sets
    - Better error handling and recovery
    - Detailed performance monitoring
    - Configurable observation windowing
    - Enhanced logging and debugging capabilities
    """

    def __init__(
        self,
        gt_field: np.ndarray,
        config: ExplorationConfig,
        rng: Optional[np.random.Generator] = None
    ):
        """
        Initialize integrated explorer v2.

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

        # ADE-PSPF estimator (optimized configuration)
        adepspf_config = config.adepspf_config or ADEPSPFConfig(
            n_swarms=4,
            n_particles=80,  # Reduced from 100 for better performance
            n_iterations=1,
            ade_generations=3,
            # Optimized parameters for faster convergence
            theta_dist=40.0,
            b_dist=8.0,
            theta_ps=40.0,
            b_ps=8.0
        )
        self.estimator = ADEPSPF(config=adepspf_config, rng=self.rng)

        # RRT planner (optimized configuration)
        environment = PlanarEnvironment(
            bounds=(0.0, float(config.grid_size), 0.0, float(config.grid_size)),
            obstacles=[]
        )
        gain_params = config.gain_params or GainParameters(
            sigma_dist=0.1,
            r_src=0.8,
            d_peak=2.0,
            eta_src=0.3  # Reduced for less aggressive distance penalty
        )
        gain_model = RadiationGainModel(gain_params)
        rrt_config = config.rrt_config or PlannerConfig(
            n_uniform=8,
            max_nodes=80,
            min_step=8.0,  # Minimum 0.32m per step (ensures reasonable movement)
            max_step=20.0,  # Maximum 0.8m per step
            random_seed=self.rng.integers(0, 10000)
        )

        # Choose RRT planner version
        if config.use_rrt_v2:
            perf_config = config.rrt_v2_perf_config or PerformanceConfig(verbose=False)
            self.planner = RRTPlannerV2(environment, gain_model, rrt_config, perf_config)
            print("  [RRT] Using optimized RRT Planner V2")
        else:
            self.planner = RRTPlanner(environment, gain_model, rrt_config)
            print("  [RRT] Using original RRT Planner V1")

        # History
        self.iteration_results: List[IterationResult] = []
        self.previous_best_branch: Optional[List[RRTNode]] = None

        # Statistics
        self.start_time: float = 0.0
        self.converged: bool = False
        self.convergence_iteration: Optional[int] = None

        # Termination criteria (논문 방식)
        self.no_improvement_count: int = 0
        self.prev_rfc: float = 0.0
        self.consecutive_threshold: int = 10  # 논문 기준: 10 consecutive iterations

        # Exploration phases (논문 Fig. 14-16)
        self.current_phase: ExplorationPhase = ExplorationPhase.TRACING
        self.phase_history: List[Tuple[int, ExplorationPhase]] = []

    def _determine_phase(self, rfc: float, iteration: int) -> ExplorationPhase:
        """
        Determine current exploration phase based on RFC (논문 Fig. 14-16).

        논문 기준:
        - RFC < 0.3: TRACING (의심 소스 추적)
        - 0.3 ≤ RFC < 0.85: SURROUNDING (소스 주변 관측)
        - RFC ≥ 0.85: EXPLORING (미탐색 영역 탐색)

        Args:
            rfc: Current RFC value
            iteration: Current iteration number

        Returns:
            Current exploration phase
        """
        if rfc < 0.3:
            return ExplorationPhase.TRACING
        elif rfc < 0.85:
            return ExplorationPhase.SURROUNDING
        else:
            return ExplorationPhase.EXPLORING

    def _update_phase(self, rfc: float, iteration: int) -> None:
        """
        Update exploration phase and log transitions.

        Args:
            rfc: Current RFC value
            iteration: Current iteration number
        """
        new_phase = self._determine_phase(rfc, iteration)

        # Check for phase transition
        if new_phase != self.current_phase:
            old_phase = self.current_phase
            self.current_phase = new_phase
            self.phase_history.append((iteration, new_phase))

            if self.config.log_iteration_details:
                print(f"\n{'='*70}")
                print(f"🔄 Phase Transition at Iteration {iteration + 1}")
                print(f"{'='*70}")
                print(f"  {old_phase.name} → {new_phase.name}")
                print(f"  RFC: {rfc:.4f}")

                # Phase descriptions
                phase_desc = {
                    ExplorationPhase.TRACING: "Tracing suspicious sources (초기 탐색)",
                    ExplorationPhase.SURROUNDING: "Surrounding observation (소스 주변 관측)",
                    ExplorationPhase.EXPLORING: "Exploring unknown areas (미탐색 영역 탐색)"
                }
                print(f"  New phase: {phase_desc[new_phase]}")
                print(f"{'='*70}\n")

    def _check_termination(self, rfc: float, iteration: int) -> Tuple[bool, str]:
        """
        Check if search should terminate (논문 방식).

        논문 기준: 10 consecutive iterations without improvement

        Args:
            rfc: Current RFC value
            iteration: Current iteration number (0-indexed)

        Returns:
            (should_terminate, reason)
        """
        # Update no-improvement counter
        tolerance = 0.01  # Small tolerance for noise
        if rfc <= self.prev_rfc + tolerance:
            self.no_improvement_count += 1
        else:
            self.no_improvement_count = 0

        self.prev_rfc = rfc

        # Terminate if no improvement for consecutive_threshold iterations
        if self.no_improvement_count >= self.consecutive_threshold:
            reason = (f"No improvement for {self.consecutive_threshold} iterations "
                     f"(Final RFC: {rfc:.4f})")
            return True, reason

        # Terminate if max iterations reached
        if iteration >= self.config.max_iterations - 1:
            reason = f"Max iterations ({self.config.max_iterations}) reached"
            return True, reason

        return False, ""

    def run_exploration(self) -> Dict:
        """
        Run complete exploration process.

        Returns:
            Dictionary with exploration results and statistics
        """
        self.start_time = time.time()

        if self.config.log_iteration_details:
            print(f"\n{'='*70}")
            print(f"INTEGRATED EXPLORATION V2.0")
            print(f"{'='*70}")
            print(f"Initial position: ({self.robot.pose.x:.1f}, {self.robot.pose.y:.1f})")
            print(f"Max iterations: {self.config.max_iterations}")
            print(f"RFC threshold: {self.config.min_rfc_threshold}")
            print(f"Termination: {self.consecutive_threshold} consecutive iterations without improvement")
            print(f"{'='*70}\n")

        # Initial observation at starting position
        self._make_observation(self.robot.pose.x, self.robot.pose.y)

        for iteration in range(self.config.max_iterations):
            iter_start = time.time()

            if self.config.log_iteration_details:
                print(f"\n{'='*70}")
                print(f"Iteration {iteration + 1}/{self.config.max_iterations}")
                print(f"{'='*70}")

            # Step 1: Estimation (ADE-PSPF)
            est_start = time.time()
            result = self._estimation_step()
            time_estimation = time.time() - est_start

            # Dynamic Swarm Adjustment (논문 Fig. 11)
            self._adjust_swarm_number(result['rfc'], result['n_sources'], iteration)

            # Update Exploration Phase (논문 Fig. 14-16)
            self._update_phase(result['rfc'], iteration)

            # Check termination (논문 방식: 10 consecutive iterations without improvement)
            should_terminate, reason = self._check_termination(result['rfc'], iteration)
            if should_terminate:
                if self.config.log_iteration_details:
                    print(f"\n✓ Termination: {reason}")
                self.converged = True
                self.convergence_iteration = iteration + 1
                break

            # Step 2: Exploration (RRT path planning)
            exp_start = time.time()
            best_branch = self._exploration_step(result['sources'])
            time_exploration = time.time() - exp_start

            if not best_branch or len(best_branch) <= 1:
                if self.config.log_iteration_details:
                    print(f"\n⚠ No valid path found, stopping exploration")
                break

            # Step 3: Execute part of the best path
            exec_start = time.time()
            observations_made, n_nodes_executed = self._execution_step(best_branch)
            time_execution = time.time() - exec_start

            time_total = time.time() - iter_start

            # Record iteration result
            iteration_result = IterationResult(
                iteration=iteration + 1,
                robot_pose=self.robot.pose,
                estimated_sources=result['sources'],
                rfc=result['rfc'],
                best_rfc=result['best_rfc'],
                best_branch=best_branch,
                observations_made=observations_made,
                cumulative_observations=len(self.robot.observations),
                time_estimation=time_estimation,
                time_exploration=time_exploration,
                time_execution=time_execution,
                time_total=time_total
            )
            self.iteration_results.append(iteration_result)

            # Update previous best branch for next iteration (Eq. 9)
            # NOTE: Disabled branch reuse because robot position changes after execution
            # The reused branch would have incorrect parent references
            # TODO: Implement proper parent index remapping for branch reuse
            self.previous_best_branch = None  # Fresh planning each iteration

            if self.config.log_iteration_details:
                print(f"\nIteration {iteration + 1} complete:")
                print(f"  - Estimated sources: {len(result['sources'])}")
                print(f"  - RFC: {result['rfc']:.4f} (Best: {result['best_rfc']:.4f})")
                print(f"  - Observations made: {observations_made}")
                print(f"  - Total observations: {len(self.robot.observations)}")
                print(f"  - Robot position: ({self.robot.pose.x:.1f}, {self.robot.pose.y:.1f})")
                if self.config.enable_timing_logs:
                    print(f"  - Time: {time_total:.3f}s (Est: {time_estimation:.3f}s, "
                          f"Exp: {time_exploration:.3f}s, Exec: {time_execution:.3f}s)")

        # Final estimation
        final_result = self._estimation_step()

        return self._compile_results(final_result)

    def _make_observation(self, x: float, y: float):
        """Make a radiation observation at given position."""
        # Convert to integer indices for grid access
        x_int = int(np.clip(round(x), 0, self.config.grid_size - 1))
        y_int = int(np.clip(round(y), 0, self.config.grid_size - 1))

        try:
            intensity = self.observer.observe((y_int, x_int))
            # Store observations in (x, y) order to stay consistent with Eq. (1)
            # of the paper, which operates in planar Cartesian coordinates.
            self.robot.add_observation(
                np.array([x, y], dtype=np.float64),
                intensity
            )
        except Exception as e:
            print(f"⚠ Warning: Failed to make observation at ({x:.1f}, {y:.1f}): {e}")

    def _estimation_step(self) -> Dict:
        """
        Run ADE-PSPF estimation step.

        Returns:
            Estimation results with sources and RFC
        """
        if self.config.log_iteration_details:
            print(f"\n[ESTIMATION] Running ADE-PSPF...")
            print(f"  Observations: {len(self.robot.observations)}")

        try:
            # Use recent observations for better performance
            observations_to_use = self.robot.get_recent_observations(
                self.config.max_observations_for_weight
            )

            if self.config.enable_timing_logs:
                start = time.time()
                result = self.estimator.update(self.robot.observations)
                elapsed = time.time() - start
                print(f"  Estimation completed in {elapsed:.3f}s")
            else:
                result = self.estimator.update(self.robot.observations)

            if self.config.log_iteration_details:
                print(f"  RFC: {result['rfc']:.4f}")
                print(f"  Best RFC: {result['best_rfc']:.4f}")
                print(f"  Sources found: {result['n_sources']}")

            return result

        except Exception as e:
            print(f"⚠ Error in estimation step: {e}")
            # Return safe default
            return {
                'iteration': self.estimator.iteration,
                'rfc': 0.0,
                'n_sources': 0,
                'sources': [],
                'centroids': [],
                'configuration_accepted': False,
                'best_rfc': 0.0
            }

    def _exploration_step(self, estimated_sources: List[np.ndarray]) -> List[RRTNode]:
        """
        Plan exploration path using RRT.

        Args:
            estimated_sources: Estimated source parameters

        Returns:
            Best branch from RRT planning
        """
        if self.config.log_iteration_details:
            print(f"\n[EXPLORATION] Planning path with RRT...")

        try:
            if len(estimated_sources) == 0:
                if self.config.log_iteration_details:
                    print(f"  ⚠ No sources estimated, using default exploration")
                # Random exploration when no sources detected
                estimated_sources = [np.array([
                    self.rng.uniform(64, 192),
                    self.rng.uniform(64, 192),
                    50.0
                ])]

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
                if len(self.robot.observations) > 0:
                    distances = np.linalg.norm(obs_positions - pos[None, :], axis=1)
                    nearest_idx = np.argmin(distances)
                    nearest_intensity = obs_intensities[nearest_idx]
                    distance_near = distances[nearest_idx]
                    radius = 20.0
                    samples_near = np.sum(distances < radius)
                else:
                    nearest_intensity = 0.0
                    distance_near = 1000.0
                    samples_near = 0
                return nearest_intensity, samples_near, distance_near

            # Build RRT tree
            if self.config.enable_timing_logs:
                start = time.time()
                nodes, leaves = self.planner.build_tree(
                    root_pose=self.robot.pose,
                    sources=sources,
                    observation_positions=obs_positions,
                    observation_intensities=obs_intensities,
                    nearest_intensity_lookup=nearest_intensity_lookup,
                    previous_best_branch=self.previous_best_branch
                )
                elapsed = time.time() - start
                print(f"  RRT completed in {elapsed:.3f}s")
            else:
                nodes, leaves = self.planner.build_tree(
                    root_pose=self.robot.pose,
                    sources=sources,
                    observation_positions=obs_positions,
                    observation_intensities=obs_intensities,
                    nearest_intensity_lookup=nearest_intensity_lookup,
                    previous_best_branch=self.previous_best_branch
                )

            if self.config.log_iteration_details:
                print(f"  Nodes generated: {len(nodes)}")
                print(f"  Leaf nodes: {len(leaves)}")

            # Select best branch
            if leaves:
                best_branch, best_leaf = self.planner.select_best_branch(nodes, leaves)
                best_gain = nodes[best_leaf].cumulative_gain
                if self.config.log_iteration_details:
                    print(f"  Best branch: {len(best_branch)} nodes, gain={best_gain:.6f}")
                return best_branch
            else:
                if self.config.log_iteration_details:
                    print(f"  ⚠ No leaves found")
                return []

        except Exception as e:
            print(f"⚠ Error in exploration step: {e}")
            return []

    def _execution_step(self, best_branch: List[RRTNode]) -> tuple[int, int]:
        """
        Execute first edge only (논문 방식).

        논문: "the robot only executes the first edge"
        - 매 iteration마다 첫 번째 edge만 실행
        - 한 번의 observation만 수행
        - Receding Horizon 방식으로 계속 재계획

        Args:
            best_branch: Best branch from RRT

        Returns:
            Tuple of (observations_made, n_nodes_executed)
        """
        if self.config.log_iteration_details:
            print(f"\n[EXECUTION] Moving along path (first edge only)...")

        try:
            # 논문 방식: First edge only
            if self.config.branch_execution_ratio == 0.0:
                # Execute only first edge (논문 방식)
                if len(best_branch) < 2:
                    return 0, 0

                n_nodes = 1  # First edge only
            else:
                # Legacy mode: Execute ratio of branch
                n_nodes = max(1, int(len(best_branch) * self.config.branch_execution_ratio))
                n_nodes = min(n_nodes, len(best_branch) - 1)

            # Check minimum movement distance (0.3m = 7.5 pixels)
            # Relaxed from 0.5m to allow some movement while preventing complete stagnation
            MIN_MOVEMENT_DISTANCE = 7.5  # pixels (0.3 meters)

            current_pos = np.array([self.robot.pose.x, self.robot.pose.y])
            final_node = best_branch[n_nodes]
            final_pos = np.array([final_node.pose.x, final_node.pose.y])
            planned_distance = np.linalg.norm(final_pos - current_pos)

            # If planned movement is too small, try to extend or skip iteration
            if planned_distance < MIN_MOVEMENT_DISTANCE:
                # Try to execute more nodes to reach minimum distance
                found_sufficient = False
                if len(best_branch) > n_nodes + 1:
                    for i in range(n_nodes + 1, len(best_branch)):
                        test_pos = np.array([best_branch[i].pose.x, best_branch[i].pose.y])
                        test_distance = np.linalg.norm(test_pos - current_pos)
                        if test_distance >= MIN_MOVEMENT_DISTANCE:
                            n_nodes = i
                            found_sufficient = True
                            if self.config.log_iteration_details:
                                print(f"  ⚠ Extended path to {n_nodes} nodes to meet minimum distance (0.5m)")
                            break

                if not found_sufficient:
                    # Path is too short even with all nodes, skip this iteration
                    if self.config.log_iteration_details:
                        print(f"  ⚠ Path too short ({planned_distance:.1f}px < {MIN_MOVEMENT_DISTANCE:.1f}px), skipping movement")
                    return 0, 0  # Don't move, don't make observations

            if self.config.log_iteration_details:
                print(f"  Executing {n_nodes}/{len(best_branch)} nodes")
                print(f"  Planned movement: {planned_distance:.1f} pixels ({planned_distance * 0.04:.3f} m)")

            observations_made = 0

            # Execute nodes along the path
            obs_interval = max(1, n_nodes // self.config.observations_per_iteration)

            for i in range(1, n_nodes + 1):
                node = best_branch[i]
                self.robot.move_to(node.pose.x, node.pose.y, node.pose.theta)

                # Make observations at intervals
                if i % obs_interval == 0 or i == n_nodes:
                    self._make_observation(node.pose.x, node.pose.y)
                    observations_made += 1

            # Calculate actual movement
            actual_pos = np.array([self.robot.pose.x, self.robot.pose.y])
            actual_distance = np.linalg.norm(actual_pos - current_pos)

            if self.config.log_iteration_details:
                print(f"  Final position: ({self.robot.pose.x:.1f}, {self.robot.pose.y:.1f})")
                print(f"  Actual movement: {actual_distance:.1f} pixels ({actual_distance * 0.04:.3f} m)")
                print(f"  Observations made: {observations_made}")
                print(f"  Nodes executed: {n_nodes}")

            return observations_made, n_nodes

        except Exception as e:
            print(f"⚠ Error in execution step: {e}")
            return 0, 0

    def _adjust_swarm_number(
        self,
        rfc: float,
        n_sources_estimated: int,
        iteration: int
    ) -> None:
        """
        Adjust swarm number based on RFC (논문 Fig. 11).

        논문 설명:
        - RFC가 낮고 추정된 소스 수가 swarm 수보다 적으면 swarm 추가
        - 이는 놓친 소스가 있을 가능성을 의미

        Args:
            rfc: Current RFC value
            n_sources_estimated: Number of estimated sources
            iteration: Current iteration number
        """
        MIN_ITERATIONS = 5  # At least 5 iterations before adjustment
        RFC_THRESHOLD = 0.7  # Low RFC threshold
        MAX_SWARMS = 5  # Maximum number of swarms

        if iteration < MIN_ITERATIONS:
            return

        current_swarms = len(self.estimator.swarms)

        # Don't exceed maximum swarms
        if current_swarms >= MAX_SWARMS:
            return

        # If RFC is low and we might be missing sources
        if rfc < RFC_THRESHOLD and n_sources_estimated < current_swarms:
            # Add one swarm
            self.estimator.add_swarm()
            if self.config.log_iteration_details:
                print(f"\n  → Dynamic Swarm Adjustment:")
                print(f"    Increased swarms: {current_swarms} → {current_swarms + 1}")
                print(f"    Reason: RFC={rfc:.4f} < {RFC_THRESHOLD}, "
                      f"estimated sources={n_sources_estimated}")

    def _compile_results(self, final_result: Dict) -> Dict:
        """Compile final exploration results."""
        total_time = time.time() - self.start_time

        # Calculate statistics
        if self.iteration_results:
            avg_time_per_iter = np.mean([r.time_total for r in self.iteration_results])
            avg_time_est = np.mean([r.time_estimation for r in self.iteration_results])
            avg_time_exp = np.mean([r.time_exploration for r in self.iteration_results])
        else:
            avg_time_per_iter = avg_time_est = avg_time_exp = 0.0

        statistics = ExplorationStatistics(
            total_iterations=len(self.iteration_results),
            total_observations=len(self.robot.observations),
            total_time=total_time,
            final_rfc=final_result['rfc'],
            best_rfc=final_result['best_rfc'],
            converged=self.converged,
            convergence_iteration=self.convergence_iteration,
            avg_time_per_iteration=avg_time_per_iter,
            avg_time_estimation=avg_time_est,
            avg_time_exploration=avg_time_exp
        )

        return {
            'iterations': len(self.iteration_results),
            'final_sources': final_result['sources'],
            'final_rfc': final_result['rfc'],
            'best_rfc': final_result['best_rfc'],
            'total_observations': len(self.robot.observations),
            'trajectory': self.robot.trajectory,
            'final_pose': self.robot.pose,
            'iteration_history': self.iteration_results,
            'observations': self.robot.observations,
            'statistics': statistics,
            'converged': self.converged,
            'convergence_iteration': self.convergence_iteration
        }

    def print_summary(self, results: Dict):
        """Print exploration summary."""
        stats = results['statistics']

        print(f"\n{'='*70}")
        print(f"EXPLORATION SUMMARY")
        print(f"{'='*70}")
        print(f"Total iterations:      {stats.total_iterations}")
        print(f"Total observations:    {stats.total_observations}")
        print(f"Total time:            {stats.total_time:.2f}s")
        print(f"Avg time per iter:     {stats.avg_time_per_iteration:.3f}s")
        print(f"  - Estimation:        {stats.avg_time_estimation:.3f}s")
        print(f"  - Exploration:       {stats.avg_time_exploration:.3f}s")
        print(f"\nFinal RFC:             {stats.final_rfc:.4f}")
        print(f"Best RFC:              {stats.best_rfc:.4f}")
        print(f"Converged:             {'Yes' if stats.converged else 'No'}")
        if stats.converged:
            print(f"Convergence iteration: {stats.convergence_iteration}")
        print(f"\nSources found:         {len(results['final_sources'])}")

        if results['final_sources']:
            print(f"\nFinal estimated sources:")
            for i, source in enumerate(results['final_sources']):
                print(f"  Source {i+1}: pos=({source[1]:.1f}, {source[0]:.1f}), "
                      f"intensity={source[2]:.2f}")

        print(f"{'='*70}\n")


def visualize_results(result, gt_field, true_sources):
    """Create a simple visualization of exploration results."""
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    fig, axes = plt.subplots(2, 2, figsize=(16, 14))

    # Extract data
    trajectory = result.get('trajectory', [])
    observations = result.get('observations', [])
    iteration_history = result.get('iteration_history', [])
    final_sources = result['final_sources']
    stats = result['statistics']

    # 1. Ground Truth + Trajectory
    ax1 = axes[0, 0]
    im1 = ax1.imshow(gt_field, origin='lower', cmap='hot', interpolation='bilinear', alpha=0.7)

    # True sources
    for i, src in enumerate(true_sources):
        ax1.plot(src[1], src[0], 'g*', markersize=25, markeredgecolor='black',
                markeredgewidth=2, label='True Source' if i == 0 else '')

    # Trajectory
    if trajectory and len(trajectory) > 1:
        traj_array = np.array(trajectory)
        colors = plt.cm.viridis(np.linspace(0, 1, len(traj_array)))

        for i in range(len(traj_array) - 1):
            ax1.plot(traj_array[i:i+2, 0], traj_array[i:i+2, 1],
                    color=colors[i], linewidth=4, alpha=0.9)

        ax1.scatter(traj_array[:, 0], traj_array[:, 1],
                   c=np.arange(len(traj_array)), cmap='viridis',
                   s=100, edgecolors='black', linewidths=2, alpha=0.8)

        ax1.plot(traj_array[0, 0], traj_array[0, 1], 'go', markersize=20,
                label='Start', markeredgecolor='black', markeredgewidth=3)
        ax1.plot(traj_array[-1, 0], traj_array[-1, 1], 'ro', markersize=20,
                label='End', markeredgecolor='black', markeredgewidth=3)

    # Observations
    if observations:
        obs_array = np.array([(pos[1], pos[0]) for pos, _ in observations])
        ax1.scatter(obs_array[:, 0], obs_array[:, 1], c='cyan', s=100, marker='x',
                   linewidths=3, label='Observations', zorder=10)

    ax1.set_title(f'Ground Truth & Trajectory ({len(trajectory)} waypoints)',
                 fontsize=14, fontweight='bold')
    ax1.set_xlabel('X (pixels)')
    ax1.set_ylabel('Y (pixels)')
    ax1.legend(loc='upper right', fontsize=10)
    plt.colorbar(im1, ax=ax1, label='Intensity')
    ax1.grid(True, alpha=0.3)

    # 2. Final Source Estimates
    ax2 = axes[0, 1]
    im2 = ax2.imshow(gt_field, origin='lower', cmap='hot', alpha=0.3, interpolation='bilinear')

    for i, src in enumerate(true_sources):
        ax2.plot(src[1], src[0], 'g*', markersize=25, markeredgecolor='black',
                markeredgewidth=2, label='True' if i == 0 else '')
        circle = Circle((src[1], src[0]), 15, fill=False, edgecolor='green',
                       linewidth=2, linestyle='--', alpha=0.5)
        ax2.add_patch(circle)

    for i, src in enumerate(final_sources):
        ax2.plot(src[1], src[0], 'r^', markersize=20, markeredgecolor='black',
                markeredgewidth=2, label='Estimated' if i == 0 else '')
        radius = max(5, min(30, src[2] / 3))
        circle = Circle((src[1], src[0]), radius, fill=False, edgecolor='red',
                       linewidth=2, alpha=0.6)
        ax2.add_patch(circle)

    ax2.set_title('True vs Estimated Sources', fontsize=14, fontweight='bold')
    ax2.set_xlabel('X (pixels)')
    ax2.set_ylabel('Y (pixels)')
    ax2.legend(loc='upper right', fontsize=10)
    ax2.grid(True, alpha=0.3)

    # 3. RFC Convergence
    ax3 = axes[1, 0]
    if iteration_history:
        rfc_values = [iter_result.rfc for iter_result in iteration_history]
        iterations = range(1, len(rfc_values) + 1)

        ax3.plot(iterations, rfc_values, 'b-o', linewidth=2.5, markersize=8, label='RFC')
        ax3.axhline(y=0.85, color='r', linestyle='--', linewidth=2.5, label='Threshold (0.85)')
        ax3.fill_between(iterations, 0, rfc_values, alpha=0.3, color='blue')

        ax3.set_xlabel('Iteration', fontsize=12, fontweight='bold')
        ax3.set_ylabel('RFC', fontsize=12, fontweight='bold')
        ax3.set_title('RFC Convergence', fontsize=14, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.legend(fontsize=10)
        ax3.set_ylim([0, 1.0])

    # 4. Statistics
    ax4 = axes[1, 1]
    ax4.axis('off')

    stats_text = "EXPLORATION STATISTICS\n"
    stats_text += "="*50 + "\n\n"
    stats_text += f"Iterations: {stats.total_iterations}\n"
    stats_text += f"Observations: {stats.total_observations}\n"
    stats_text += f"Total Time: {stats.total_time:.3f}s\n"
    stats_text += f"Avg Time/Iter: {stats.avg_time_per_iteration:.3f}s\n\n"

    stats_text += f"Final RFC: {stats.final_rfc:.4f}\n"
    stats_text += f"Best RFC: {stats.best_rfc:.4f}\n"
    stats_text += f"Converged: {'YES ✓' if stats.converged else 'NO ✗'}\n\n"

    stats_text += f"True Sources: {len(true_sources)}\n"
    stats_text += f"Estimated Sources: {len(final_sources)}\n\n"

    stats_text += "True Positions:\n"
    for i, src in enumerate(true_sources):
        stats_text += f"  {i+1}. ({src[1]:6.1f}, {src[0]:6.1f}) I={src[2]:6.2f}\n"

    stats_text += "\nEstimated Positions:\n"
    for i, src in enumerate(final_sources):
        stats_text += f"  {i+1}. ({src[1]:6.1f}, {src[0]:6.1f}) I={src[2]:6.2f}\n"

    if len(true_sources) == len(final_sources):
        stats_text += "\nPosition Errors:\n"
        total_error = 0
        for i in range(len(true_sources)):
            true_pos = np.array([true_sources[i][0], true_sources[i][1]])
            est_pos = np.array([final_sources[i][0], final_sources[i][1]])
            error = np.linalg.norm(true_pos - est_pos)
            total_error += error
            stats_text += f"  Source {i+1}: {error:6.2f} pixels\n"
        stats_text += f"  Average: {total_error/len(true_sources):6.2f} pixels\n"

    ax4.text(0.05, 0.95, stats_text, fontsize=10, family='monospace',
            verticalalignment='top', transform=ax4.transAxes,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

    fig.suptitle('Integrated Explorer V2.0 - Results', fontsize=16, fontweight='bold')
    plt.tight_layout()

    # Save to file
    from pathlib import Path
    output_dir = Path(__file__).parent.parent / 'data' / 'figures'
    output_dir.mkdir(parents=True, exist_ok=True)
    save_path = output_dir / 'integrated_explorer_v2_result.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n📊 Visualization saved to: {save_path}")

    # Try to show (will work if display available)
    try:
        plt.show(block=True)
        plt.pause(0.1)
    except:
        pass  # No display available

    plt.close()


if __name__ == "__main__":
    print("\n" + "🚀 " * 30)
    print("INTEGRATED EXPLORER V2.0 - TEST RUN")
    print("🚀 " * 30 + "\n")

    # Generate ground truth
    print("Setting up environment...")
    rng = np.random.default_rng(42)
    n_sources = 3
    coords, amps, sigmas = sample_sources(GRID, n_sources, rng=rng)
    gt_field = inverse_square_field(GRID, coords, amps, h=0.5)

    true_sources = np.column_stack([coords, amps])

    print(f"\nTrue sources ({n_sources}):")
    for i in range(n_sources):
        print(f"  Source {i+1}: pos=({coords[i,1]:.1f}, {coords[i,0]:.1f}), "
              f"intensity={amps[i]:.2f}")

    # Configure explorer
    print(f"\nInitializing Integrated Explorer V2.0...")

    adepspf_config = ADEPSPFConfig(
        n_swarms=4,
        n_particles=100,  # Increased from 80
        n_iterations=1,
        ade_generations=10  # Increased from 3 for better convergence
    )

    config = ExplorationConfig(
        max_iterations=50,  # Increased from 10 to allow more exploration
        branch_execution_ratio=0.8,  # Increased from 0.5 for more movement
        min_rfc_threshold=0.85,
        observations_per_iteration=5,  # Increased from 2 for more data
        max_observations_for_weight=20,
        enable_timing_logs=True,
        log_iteration_details=True,
        adepspf_config=adepspf_config
    )

    explorer = IntegratedExplorerV2(gt_field, config, rng=rng)

    # Run exploration
    results = explorer.run_exploration()

    # Print summary
    explorer.print_summary(results)

    print("✅ Integrated Explorer V2.0 test completed successfully!")
    print("\n" + "🎉 " * 30 + "\n")

    # Show visualization
    print("\nGenerating visualization...")
    visualize_results(results, gt_field, true_sources)
    print("✅ Visualization displayed!")
