"""
Explorer Controller Module

Controls the exploration process and manages data flow between
the integrated explorer and the visualization system.
"""
from __future__ import annotations

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import time
from typing import List, Tuple, Optional, Dict, Any
from copy import deepcopy

from environment.generate_truth import sample_sources, inverse_square_field, GRID
from simulation.integrated_explorer_v3 import (
    IntegratedExplorerV3, ExplorationConfigV3
)
from core.ade_pspf import ADEPSPFConfig
from simulation.exploration.rrt_planner import Pose2D

from visualization.data_manager import (
    ExplorationSnapshot, IterationData,
    EstimationStepData, ExplorationStepData, ExecutionStepData,
    extract_particles_from_swarms, extract_rrt_gain_values
)


class ExplorerController:
    """
    Controls exploration execution and data collection.

    This controller acts as an interface between the IntegratedExplorerV3
    and the visualization system, capturing detailed snapshots at each step.
    """

    def __init__(self):
        """Initialize controller."""
        self.gt_field: Optional[np.ndarray] = None
        self.true_sources: Optional[np.ndarray] = None
        self.n_sources: int = 0
        self.random_seed: int = 42

        self.explorer: Optional[IntegratedExplorerV3] = None
        self.config: Optional[ExplorationConfigV3] = None

        self.snapshot: Optional[ExplorationSnapshot] = None
        self.is_running: bool = False
        self.is_completed: bool = False

    def generate_ground_truth(
        self,
        n_sources: int = 3,
        seed: int = 42,
        grid_size: int = 256
    ) -> bool:
        """
        Generate new ground truth field.

        Args:
            n_sources: Number of radiation sources
            seed: Random seed
            grid_size: Grid size in pixels

        Returns:
            True if successful
        """
        try:
            print(f"\n{'='*70}")
            print(f"GENERATING GROUND TRUTH")
            print(f"{'='*70}")
            print(f"Sources: {n_sources}, Seed: {seed}, Grid: {grid_size}x{grid_size}")

            rng = np.random.default_rng(seed)

            # Generate sources
            coords, amps, sigmas = sample_sources(GRID, n_sources, rng=rng)
            gt_field = inverse_square_field(GRID, coords, amps, h=0.5)

            # Store
            self.gt_field = gt_field
            self.true_sources = np.column_stack([coords, amps])
            self.n_sources = n_sources
            self.random_seed = seed

            print(f"\nGenerated sources:")
            for i in range(n_sources):
                print(f"  Source {i+1}: pos=({coords[i,1]:.1f}, {coords[i,0]:.1f}), "
                      f"intensity={amps[i]:.2f}")

            print(f"{'='*70}\n")

            # Reset exploration state
            self.is_completed = False
            self.snapshot = None
            self.explorer = None

            return True

        except Exception as e:
            print(f"✗ Error generating ground truth: {e}")
            return False

    def initialize_explorer(
        self,
        max_iterations: int = 15,
        n_swarms: int = 4,
        n_particles: int = 80,
        ade_generations: int = 3,
        branch_execution_ratio: float = 0.0,  # First edge only (paper mode)
        observations_per_iteration: int = 1,
        enable_logs: bool = False
    ) -> bool:
        """
        Initialize the explorer with given configuration.

        Args:
            max_iterations: Maximum exploration iterations
            n_swarms: Number of ADE-PSPF swarms
            n_particles: Particles per swarm
            ade_generations: ADE generations per iteration
            branch_execution_ratio: Ratio of path to execute (0.0 = first edge only)
            observations_per_iteration: Observations per iteration
            enable_logs: Enable detailed logging

        Returns:
            True if successful
        """
        try:
            if self.gt_field is None:
                print("✗ No ground truth generated. Call generate_ground_truth() first.")
                return False

            print(f"\n{'='*70}")
            print(f"INITIALIZING EXPLORER")
            print(f"{'='*70}")

            # Configure ADE-PSPF
            adepspf_config = ADEPSPFConfig(
                n_swarms=n_swarms,
                n_particles=n_particles,
                n_iterations=1,
                ade_generations=ade_generations,
                theta_dist=40.0,
                b_dist=8.0,
                theta_ps=40.0,
                b_ps=8.0
            )

            # Configure exploration
            self.config = ExplorationConfigV3(
                grid_size=256,
                pixel_resolution=0.04,
                robot_start_x=128.0,
                robot_start_y=128.0,
                robot_start_theta=0.0,
                max_iterations=max_iterations,
                # branch_execution_ratio=branch_execution_ratio,
                min_rfc_threshold=0.85,
                # observations_per_iteration=observations_per_iteration,
                # max_observations_for_weight=20,
                # enable_timing_logs=enable_logs,
                # log_iteration_details=enable_logs,
                # adepspf_config=adepspf_config
            )

            # Create explorer
            rng = np.random.default_rng(self.random_seed)
            self.explorer = IntegratedExplorerV3(self.gt_field, self.config, rng=rng)

            print(f"Explorer initialized:")
            print(f"  Max iterations: {max_iterations}")
            print(f"  Swarms: {n_swarms}, Particles: {n_particles}")
            print(f"  ADE generations: {ade_generations}")
            print(f"  Execution mode: {'First edge only' if branch_execution_ratio == 0.0 else f'{branch_execution_ratio*100:.0f}% of path'}")
            print(f"{'='*70}\n")

            return True

        except Exception as e:
            print(f"✗ Error initializing explorer: {e}")
            import traceback
            traceback.print_exc()
            return False

    def run_exploration_with_snapshots(self, progress_callback=None) -> bool:
        """
        Run complete exploration and capture detailed snapshots.

        This is a modified version of IntegratedExplorerV3.run_exploration()
        that captures detailed data at each step for visualization.

        Args:
            progress_callback: Optional callback function(iteration, max_iter, status)
                              Called after each iteration to update GUI

        Returns:
            True if successful
        """
        if self.explorer is None:
            print("✗ Explorer not initialized. Call initialize_explorer() first.")
            return False

        if self.is_running:
            print("✗ Exploration already running.")
            return False

        try:
            self.is_running = True
            self.is_completed = False

            print(f"\n{'='*70}")
            print(f"RUNNING EXPLORATION WITH SNAPSHOT CAPTURE")
            print(f"{'='*70}\n")

            start_time = time.time()
            iteration_snapshots = []

            # Initial observation
            self.explorer._make_observation(
                self.explorer.robot.pose.x,
                self.explorer.robot.pose.y
            )

            # Main exploration loop
            for iteration in range(self.config.max_iterations):
                print(f"\n{'='*70}")
                print(f"Iteration {iteration + 1}/{self.config.max_iterations}")
                print(f"{'='*70}")

                iter_start_time = time.time()

                # Create iteration snapshot
                iter_snapshot = IterationData(iteration=iteration)

                # STEP 1: ESTIMATION
                print(f"\n[STEP 1/3] Estimation (ADE-PSPF)...")
                est_start = time.time()
                estimation_result = self.explorer._estimation_step()
                est_time = time.time() - est_start

                # Capture estimation data
                particles_by_swarm, centroids, weights = extract_particles_from_swarms(
                    self.explorer.estimator.swarms
                )

                iter_snapshot.estimation_data = EstimationStepData(
                    observations=list(self.explorer.robot.observations),
                    n_observations=len(self.explorer.robot.observations),
                    estimated_sources=estimation_result['sources'],
                    n_sources=estimation_result['n_sources'],
                    rfc=estimation_result['rfc'],
                    best_rfc=estimation_result['best_rfc'],
                    particles_by_swarm=particles_by_swarm,
                    swarm_centroids=centroids,
                    swarm_weights=weights,
                    n_swarms=len(self.explorer.estimator.swarms),
                    time_elapsed=est_time
                )

                print(f"  ✓ RFC: {estimation_result['rfc']:.4f}, Sources: {estimation_result['n_sources']}")

                # Dynamic swarm adjustment
                self.explorer._adjust_swarm_number(
                    estimation_result['rfc'],
                    estimation_result['n_sources'],
                    iteration
                )

                # Update phase
                phase_before = self.explorer.current_phase
                self.explorer._update_phase(estimation_result['rfc'], iteration)
                iter_snapshot.phase = self.explorer.current_phase
                iter_snapshot.rfc_before = self.explorer.prev_rfc
                iter_snapshot.rfc_after = estimation_result['rfc']

                # Check termination
                should_terminate, reason = self.explorer._check_termination(
                    estimation_result['rfc'], iteration
                )

                if should_terminate:
                    print(f"\n✓ Termination: {reason}")
                    self.explorer.converged = True
                    self.explorer.convergence_iteration = iteration + 1
                    iteration_snapshots.append(iter_snapshot)
                    break

                # STEP 2: EXPLORATION
                print(f"\n[STEP 2/3] Exploration (RRT Planning)...")
                exp_start = time.time()

                # Use V3's _planning_step to ensure branch reuse is handled correctly
                planning_result = self.explorer._planning_step(estimation_result['sources'])
                exp_time = time.time() - exp_start

                if not planning_result.success or len(planning_result.best_branch) <= 1:
                    print(f"  ⚠ No valid path found, stopping exploration")
                    iteration_snapshots.append(iter_snapshot)
                    break

                # Extract data from planning result
                best_branch = planning_result.best_branch
                nodes = planning_result.nodes
                leaves = planning_result.leaves

                # Save best branch for next iteration (Eq. 9)
                # This mimics what V3's run_exploration() does
                if self.config.enable_branch_reuse and len(best_branch) > 0:
                    self.explorer.previous_best_branch = best_branch
                else:
                    self.explorer.previous_best_branch = None

                # Extract gain values
                node_gains, cumulative_gains = extract_rrt_gain_values(nodes)

                # Get observation data
                if len(self.explorer.robot.observations) > 0:
                    obs_positions = np.array([obs[0] for obs in self.explorer.robot.observations])
                    obs_intensities = np.array([obs[1] for obs in self.explorer.robot.observations])
                else:
                    obs_positions = np.array([]).reshape(0, 2)
                    obs_intensities = np.array([])

                iter_snapshot.exploration_data = ExplorationStepData(
                    robot_pose=self.explorer.robot.pose,
                    estimated_sources=estimation_result['sources'],
                    observation_positions=obs_positions,
                    observation_intensities=obs_intensities,
                    rrt_nodes=nodes,
                    rrt_leaves=leaves,
                    n_nodes=len(nodes),
                    best_branch=best_branch,
                    best_leaf_idx=leaves[0] if leaves else -1,
                    best_gain=nodes[leaves[0]].cumulative_gain if leaves else 0.0,
                    node_gains=node_gains,
                    cumulative_gains=cumulative_gains,
                    time_elapsed=exp_time
                )

                print(f"  ✓ RRT nodes: {len(nodes)}, Best branch: {len(best_branch)} nodes")

                # STEP 3: EXECUTION
                print(f"\n[STEP 3/3] Execution (Movement)...")
                exec_start = time.time()

                pose_before = Pose2D(
                    x=self.explorer.robot.pose.x,
                    y=self.explorer.robot.pose.y,
                    theta=self.explorer.robot.pose.theta
                )

                observations_before = len(self.explorer.robot.observations)

                # Execute movement
                observations_made, n_nodes_executed = self.explorer._execution_step(best_branch)

                exec_time = time.time() - exec_start

                pose_after = self.explorer.robot.pose

                # Get new observations
                new_obs = list(self.explorer.robot.observations[observations_before:])

                # Calculate distance
                dist = np.linalg.norm(
                    np.array([pose_after.x, pose_after.y]) -
                    np.array([pose_before.x, pose_before.y])
                )

                # n_nodes_executed already returned from _execution_step()

                iter_snapshot.execution_data = ExecutionStepData(
                    pose_before=pose_before,
                    planned_path=best_branch,
                    n_planned_nodes=len(best_branch),
                    n_nodes_to_execute=n_nodes_executed,
                    planned_distance=dist,
                    pose_after=pose_after,
                    actual_distance=dist,
                    new_observations=new_obs,
                    n_new_observations=len(new_obs),
                    waypoints=[pose_before, pose_after],
                    observation_locations=[pose_after] if new_obs else [],
                    time_elapsed=exec_time
                )

                print(f"  ✓ Moved {dist:.1f} pixels, Observations: {len(new_obs)}")

                # Update cumulative stats
                iter_snapshot.cumulative_observations = len(self.explorer.robot.observations)
                iter_snapshot.total_distance_traveled = sum(
                    snap.execution_data.actual_distance
                    for snap in iteration_snapshots
                    if snap.execution_data is not None
                ) + dist

                iter_snapshot.time_total = time.time() - iter_start_time

                # Store snapshot
                iteration_snapshots.append(iter_snapshot)

                # previous_best_branch was already saved after planning step
                print(f"\n✓ Iteration {iteration + 1} completed in {iter_snapshot.time_total:.2f}s")

                # Call progress callback if provided
                if progress_callback is not None:
                    try:
                        status = f"Iteration {iteration + 1}/{self.config.max_iterations} - RFC: {estimation_result['rfc']:.4f}"
                        progress_callback(iteration + 1, self.config.max_iterations, status)
                    except Exception as e:
                        print(f"⚠ Progress callback error: {e}")

            # Final estimation
            print(f"\n{'='*70}")
            print(f"FINAL ESTIMATION")
            print(f"{'='*70}")
            final_result = self.explorer._estimation_step()

            total_time = time.time() - start_time

            # Create exploration snapshot
            self.snapshot = ExplorationSnapshot(
                gt_field=self.gt_field,
                true_sources=self.true_sources,
                n_true_sources=self.n_sources,
                grid_size=self.config.grid_size,
                pixel_resolution=self.config.pixel_resolution,
                random_seed=self.random_seed,
                config_dict=self._config_to_dict(),
                iteration_data=iteration_snapshots,
                n_iterations=len(iteration_snapshots),
                final_estimated_sources=final_result['sources'],
                final_rfc=final_result['rfc'],
                best_rfc=final_result['best_rfc'],
                converged=self.explorer.converged,
                convergence_iteration=self.explorer.convergence_iteration,
                full_trajectory=self.explorer.robot.trajectory,
                final_pose=self.explorer.robot.pose,
                all_observations=list(self.explorer.robot.observations),
                total_time=total_time,
                total_distance=sum(
                    snap.execution_data.actual_distance
                    for snap in iteration_snapshots
                    if snap.execution_data is not None
                ),
                phase_history=self.explorer.phase_history
            )

            self.is_completed = True
            self.is_running = False

            print(f"\n{'='*70}")
            print(f"EXPLORATION COMPLETED")
            print(f"{'='*70}")
            print(f"Total iterations: {len(iteration_snapshots)}")
            print(f"Total observations: {len(self.explorer.robot.observations)}")
            print(f"Total time: {total_time:.2f}s")
            print(f"Final RFC: {final_result['rfc']:.4f}")
            print(f"Best RFC: {final_result['best_rfc']:.4f}")
            print(f"Converged: {self.explorer.converged}")
            print(f"{'='*70}\n")

            return True

        except Exception as e:
            print(f"\n✗ Error during exploration: {e}")
            import traceback
            traceback.print_exc()
            self.is_running = False
            return False

    def _exploration_step_with_tree_data(
        self, estimated_sources: List[np.ndarray]
    ) -> Tuple[List, List, List]:
        """
        Modified exploration step that returns tree data.

        Returns:
            Tuple of (best_branch, nodes, leaves)
        """
        try:
            if len(estimated_sources) == 0:
                estimated_sources = [np.array([
                    self.explorer.rng.uniform(64, 192),
                    self.explorer.rng.uniform(64, 192),
                    50.0
                ])]

            sources = np.array(estimated_sources)

            # ⚡ Performance fix: Limit observations to prevent slowdown in later iterations
            MAX_OBS_FOR_RRT = 20  # Use only recent N observations for RRT planning

            if len(self.explorer.robot.observations) > 0:
                observations = self.explorer.robot.observations
                # Use only recent observations if we have too many
                if len(observations) > MAX_OBS_FOR_RRT:
                    observations = observations[-MAX_OBS_FOR_RRT:]

                obs_positions = np.array([obs[0] for obs in observations])
                obs_intensities = np.array([obs[1] for obs in observations])
            else:
                obs_positions = np.array([]).reshape(0, 2)
                obs_intensities = np.array([])

            def nearest_intensity_lookup(pose):
                pos = pose.as_array()
                # Use the already limited observations (obs_positions, obs_intensities)
                if len(obs_positions) > 0:
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
            nodes, leaves = self.explorer.planner.build_tree(
                root_pose=self.explorer.robot.pose,
                sources=sources,
                observation_positions=obs_positions,
                observation_intensities=obs_intensities,
                nearest_intensity_lookup=nearest_intensity_lookup,
                previous_best_branch=self.explorer.previous_best_branch
            )

            # Select best branch
            if leaves:
                best_branch, best_leaf = self.explorer.planner.select_best_branch(nodes, leaves)
                return best_branch, nodes, leaves
            else:
                return [], nodes, leaves

        except Exception as e:
            print(f"⚠ Error in exploration step: {e}")
            return [], [], []

    def _config_to_dict(self) -> Dict[str, Any]:
        """Convert configuration to dictionary."""
        if self.config is None:
            return {}

        return {
            'grid_size': self.config.grid_size,
            'pixel_resolution': self.config.pixel_resolution,
            'max_iterations': self.config.max_iterations,
            'branch_execution_ratio': self.config.branch_execution_ratio,
            'observations_per_iteration': self.config.observations_per_iteration,
            'n_swarms': self.config.adepspf_config.n_swarms if self.config.adepspf_config else 4,
            'n_particles': self.config.adepspf_config.n_particles if self.config.adepspf_config else 80,
            'ade_generations': self.config.adepspf_config.ade_generations if self.config.adepspf_config else 3
        }

    def get_snapshot(self) -> Optional[ExplorationSnapshot]:
        """Get the current exploration snapshot."""
        return self.snapshot

    def get_iteration_data(self, iteration: int) -> Optional[IterationData]:
        """Get data for a specific iteration."""
        if self.snapshot is None or iteration < 0 or iteration >= self.snapshot.n_iterations:
            return None
        return self.snapshot.iteration_data[iteration]
