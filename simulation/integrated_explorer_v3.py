"""
Integrated Explorer V3 - Robust Implementation

Major improvements over V2:
1. Explicit state management with verification
2. Mandatory robot movement checks
3. Clear separation of concerns
4. Comprehensive logging
5. No branch reuse (fresh planning each iteration)

Author: V3 Redesign
Date: 2025-10-16
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import Enum
import time
import numpy as np

from simulation.exploration.rrt_planner import (
    Pose2D, RRTNode, RRTPlanner, PlannerConfig,
    RadiationGainModel, GainParameters, PlanarEnvironment
)
from simulation.exploration.rrt_planner_v2 import RRTPlannerV2, PerformanceConfig
from core.ade_pspf import ADEPSPF, ADEPSPFConfig
from core.dynamic_swarm_adjustment import DynamicSwarmAdjuster, SwarmAdjustmentConfig
from environment.observation import RadiationObserver


# ═══════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════

@dataclass
class ExplorationConfigV3:
    """Configuration for integrated exploration system v3."""

    # Environment
    grid_size: int = 256
    pixel_resolution: float = 0.04  # meters per pixel

    # Robot parameters
    robot_start_x: float = 128.0
    robot_start_y: float = 128.0
    robot_start_theta: float = 0.0

    # Exploration parameters
    max_iterations: int = 15
    min_rfc_threshold: float = 0.85

    # ADE-PSPF parameters
    n_swarms: int = 4
    n_particles_per_swarm: int = 80
    ade_generations: int = 3

    # RRT parameters
    use_rrt_v2: bool = True
    rrt_max_nodes: int = 80
    rrt_min_step: float = 13.0  # pixels
    rrt_max_step: float = 26.0  # pixels
    enable_branch_reuse: bool = True  # Eq. 9: Reuse previous best branch

    # Execution parameters
    execute_first_edge_only: bool = True  # Paper mode
    min_movement_distance: float = 7.5  # pixels (0.3m)
    branch_execution_ratio: float = 0.0  # For controller compatibility (0.0 = first edge only)
    observations_per_iteration: int = 1  # For controller compatibility (paper mode)

    # Logging
    enable_verbose_logging: bool = True
    enable_state_verification: bool = True
    enable_movement_verification: bool = True
    log_iteration_details: bool = True  # For compatibility with controller

    @property
    def adepspf_config(self):
        """For controller compatibility - return a simple object with ADE-PSPF params."""
        class ADEPSPFConfigWrapper:
            def __init__(self, n_swarms, n_particles, ade_generations):
                self.n_swarms = n_swarms
                self.n_particles = n_particles
                self.ade_generations = ade_generations

        return ADEPSPFConfigWrapper(
            self.n_swarms,
            self.n_particles_per_swarm,
            self.ade_generations
        )


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


# ═══════════════════════════════════════════════════════════════════
# State and Result Dataclasses
# ═══════════════════════════════════════════════════════════════════

@dataclass
class IterationState:
    """Immutable state snapshot at a point in time."""
    iteration: int
    robot_pose: Pose2D
    trajectory_length: int
    observations_count: int
    estimated_sources: List[np.ndarray]
    rfc: float
    best_rfc: float


@dataclass
class EstimationStepResult:
    """Result of estimation step."""
    sources: List[np.ndarray]
    n_sources: int
    rfc: float
    best_rfc: float
    time_elapsed: float
    success: bool


@dataclass
class PlanningStepResult:
    """Result of RRT planning step."""
    tree_root_pose: Pose2D  # Where tree was built from
    nodes: List[RRTNode]
    leaves: List[int]
    best_branch: List[RRTNode]
    best_leaf_idx: int
    goal_pose: Optional[Pose2D]  # End of best branch
    best_gain: float
    time_elapsed: float
    success: bool


@dataclass
class ExecutionStepResult:
    """Result of execution step."""
    pose_before: Pose2D
    pose_after: Pose2D
    target_pose: Pose2D  # Intended target
    movement_distance: float
    moved_successfully: bool
    observations_made: int
    trajectory_grew: bool
    time_elapsed: float
    success: bool


@dataclass
class IterationResult:
    """Complete result of one exploration iteration."""
    iteration: int
    state_before: IterationState
    state_after: IterationState

    estimation_result: Optional[EstimationStepResult]
    planning_result: Optional[PlanningStepResult]
    execution_result: Optional[ExecutionStepResult]

    robot_moved: bool
    movement_distance: float
    goal_changed: bool

    total_time: float
    success: bool


# ═══════════════════════════════════════════════════════════════════
# Enhanced Robot State with Verification
# ═══════════════════════════════════════════════════════════════════

class RobotStateV3:
    """
    Robot state with mandatory verification.

    All state changes are logged and verified.
    """

    def __init__(self, initial_pose: Pose2D, verbose: bool = True):
        self.pose = initial_pose
        self.trajectory: List[Tuple[float, float]] = [(initial_pose.x, initial_pose.y)]
        self.observations: List[Tuple[np.ndarray, float]] = []
        self.verbose = verbose

    def move_to(self, x: float, y: float, theta: float) -> bool:
        """
        Move robot and verify state change.

        Returns:
            True if move successful, False otherwise
        """
        # Record old position
        old_x, old_y = self.pose.x, self.pose.y

        # Update pose
        self.pose = Pose2D(x=x, y=y, theta=theta)

        # Add new position to trajectory
        self.trajectory.append((x, y))

        # Verify movement
        moved = (abs(old_x - x) > 0.01) or (abs(old_y - y) > 0.01)

        if self.verbose:
            if not moved:
                print(f"⚠️  WARNING: Robot didn't move! Still at ({old_x:.1f}, {old_y:.1f})")
            else:
                dist = np.sqrt((x - old_x)**2 + (y - old_y)**2)
                print(f"✓ Robot moved: ({old_x:.1f}, {old_y:.1f}) → ({x:.1f}, {y:.1f}), "
                      f"dist={dist:.1f}px ({dist*0.04:.3f}m)")

        return moved

    def add_observation(self, position: np.ndarray, intensity: float):
        """Add observation."""
        self.observations.append((position, intensity))
        if self.verbose:
            print(f"✓ Observation added at ({position[0]:.1f}, {position[1]:.1f}), "
                  f"intensity={intensity:.2f}")

    def get_state_summary(self) -> str:
        """Get human-readable state summary."""
        return (f"Pose=({self.pose.x:.1f}, {self.pose.y:.1f}, θ={self.pose.theta:.2f}), "
                f"Trajectory={len(self.trajectory)} points, "
                f"Observations={len(self.observations)}")


# ═══════════════════════════════════════════════════════════════════
# Main Explorer Class
# ═══════════════════════════════════════════════════════════════════

class IntegratedExplorerV3:
    """
    Robust exploration system with verified state management.

    Key improvements over V2:
    - Explicit state capture before/after each step
    - Mandatory movement verification
    - No branch reuse (fresh planning)
    - Comprehensive logging
    - Clear error messages
    """

    def __init__(
        self,
        gt_field: np.ndarray,
        config: Optional[ExplorationConfigV3] = None,
        rng: Optional[np.random.Generator] = None
    ):
        """Initialize explorer V3."""
        self.config = config or ExplorationConfigV3()
        self.rng = rng or np.random.default_rng()
        self.gt_field = gt_field

        # Initialize robot with verbose logging
        initial_pose = Pose2D(
            x=self.config.robot_start_x,
            y=self.config.robot_start_y,
            theta=self.config.robot_start_theta
        )
        self.robot = RobotStateV3(
            initial_pose=initial_pose,
            verbose=self.config.enable_verbose_logging
        )

        # Initialize observer
        self.observer = RadiationObserver(gt_field)

        # Initialize ADE-PSPF estimator
        adepspf_config = ADEPSPFConfig(
            n_swarms=self.config.n_swarms,
            n_particles=self.config.n_particles_per_swarm,
            ade_generations=self.config.ade_generations,
            x_max=float(self.config.grid_size),
            y_max=float(self.config.grid_size)
        )
        self.estimator = ADEPSPF(adepspf_config, rng=self.rng)

        # Initialize RRT planner
        environment = PlanarEnvironment(
            bounds=(0.0, float(self.config.grid_size), 0.0, float(self.config.grid_size)),
            obstacles=[]
        )
        gain_model = RadiationGainModel(GainParameters())
        rrt_config = PlannerConfig(
            n_uniform=8,
            max_nodes=self.config.rrt_max_nodes,
            min_step=self.config.rrt_min_step,
            max_step=self.config.rrt_max_step,
            random_seed=self.rng.integers(0, 10000)
        )

        if self.config.use_rrt_v2:
            perf_config = PerformanceConfig(verbose=False)
            self.planner = RRTPlannerV2(environment, gain_model, rrt_config, perf_config)
            if self.config.enable_verbose_logging:
                print("  [RRT] Using optimized RRT Planner V2")
        else:
            self.planner = RRTPlanner(environment, gain_model, rrt_config)
            if self.config.enable_verbose_logging:
                print("  [RRT] Using original RRT Planner V1")

        # State tracking
        self.iteration_results: List[IterationResult] = []
        self.best_rfc_overall: float = 0.0
        self.previous_goal: Optional[Pose2D] = None
        self.previous_best_branch: Optional[List] = None  # For controller compatibility (always None in V3)

        # Phase tracking (for compatibility with controller)
        self.current_phase: ExplorationPhase = ExplorationPhase.TRACING
        self.phase_history: List[Tuple[int, ExplorationPhase]] = []

        # Termination tracking
        self.prev_rfc: float = 0.0
        self.no_improvement_count: int = 0
        self.consecutive_threshold: int = 10  # Paper: 10 consecutive iterations

        # Dynamic swarm adjustment (논문 Fig. 11)
        # Balanced settings: allow adjustment but with proper clustering validation
        swarm_adj_config = SwarmAdjustmentConfig(
            low_rfc_threshold=0.75,
            very_low_rfc_threshold=0.55,
            min_iterations_before_adjustment=8,
            check_interval=4,
            max_swarms=config.n_swarms + 3,  # Allow some extra swarms (will be filtered by clustering)
            enabled=True
        )
        self.swarm_adjuster = DynamicSwarmAdjuster(swarm_adj_config)

        if self.config.enable_verbose_logging:
            print(f"\n{'='*70}")
            print("EXPLORER V3 INITIALIZED")
            print(f"{'='*70}")
            print(f"Config: {self.config.max_iterations} iterations, "
                  f"RRT V{'2' if self.config.use_rrt_v2 else '1'}")
            print(f"Robot initial state: {self.robot.get_state_summary()}")
            print(f"Dynamic swarm adjustment: {'Enabled' if swarm_adj_config.enabled else 'Disabled'}")
            print(f"{'='*70}\n")

    def _log(self, message: str, level: str = "INFO"):
        """Conditional logging."""
        if self.config.enable_verbose_logging:
            prefix = {"INFO": "ℹ️ ", "WARN": "⚠️ ", "ERROR": "❌", "SUCCESS": "✅"}
            print(f"{prefix.get(level, '')} {message}")

    # Implementation continues in next message...

    def _capture_state(self, iteration: int, rfc: float) -> IterationState:
        """Capture current state snapshot."""
        # Get estimated sources from estimator
        sources = []
        for swarm in self.estimator.swarms:
            if swarm.centroid is not None:
                sources.append(swarm.centroid)

        return IterationState(
            iteration=iteration,
            robot_pose=Pose2D(x=self.robot.pose.x, y=self.robot.pose.y, theta=self.robot.pose.theta),
            trajectory_length=len(self.robot.trajectory),
            observations_count=len(self.robot.observations),
            estimated_sources=sources,
            rfc=rfc,
            best_rfc=self.best_rfc_overall
        )

    def _make_observation(self, x: float, y: float):
        """Make observation at given position."""
        # Convert to integer indices for grid access
        x_int = int(np.clip(round(x), 0, self.config.grid_size - 1))
        y_int = int(np.clip(round(y), 0, self.config.grid_size - 1))

        try:
            intensity = self.observer.observe((y_int, x_int))
            # Store observations in (x, y) order
            self.robot.add_observation(
                np.array([x, y], dtype=np.float64),
                intensity
            )
        except Exception as e:
            if self.config.enable_verbose_logging:
                print(f"⚠️  Warning: Failed to make observation at ({x:.1f}, {y:.1f}): {e}")

    def _estimation_step(self):
        """
        Run ADE-PSPF estimation with current observations.

        Returns:
            Dictionary with estimation results (for controller compatibility)
        """
        start_time = time.time()

        self._log(f"[ESTIMATION] Running with {len(self.robot.observations)} observations")

        try:
            if len(self.robot.observations) == 0:
                return {
                    'sources': [],
                    'n_sources': 0,
                    'rfc': 0.0,
                    'best_rfc': 0.0,
                    'iteration': 0,
                    'centroids': [],
                    'configuration_accepted': False
                }

            # Run ADE-PSPF update
            result = self.estimator.update(self.robot.observations)

            # Update best RFC
            self.best_rfc_overall = max(self.best_rfc_overall, result['rfc'])
            result['best_rfc'] = self.best_rfc_overall

            elapsed = time.time() - start_time

            self._log(f"[ESTIMATION] RFC={result['rfc']:.4f}, Sources={len(result['sources'])}, Time={elapsed:.3f}s", "SUCCESS")

            return result

        except Exception as e:
            self._log(f"[ESTIMATION] Failed: {e}", "ERROR")
            return {
                'sources': [],
                'n_sources': 0,
                'rfc': 0.0,
                'best_rfc': 0.0,
                'iteration': 0,
                'centroids': [],
                'configuration_accepted': False
            }

    def _planning_step(self, sources: List[np.ndarray]) -> PlanningStepResult:
        """
        Plan path using RRT with strict verification.
        """
        start_time = time.time()

        # Get CURRENT robot position
        current_pose = Pose2D(x=self.robot.pose.x, y=self.robot.pose.y, theta=self.robot.pose.theta)

        self._log(f"[PLANNING] Starting RRT from ({current_pose.x:.1f}, {current_pose.y:.1f})")

        try:
            # Prepare observation data
            if len(self.robot.observations) > 0:
                obs_positions = np.array([obs[0] for obs in self.robot.observations])
                obs_intensities = np.array([obs[1] for obs in self.robot.observations])
            else:
                obs_positions = np.array([]).reshape(0, 2)
                obs_intensities = np.array([])

            # Create lookup function
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

            # Convert sources to array
            sources_array = np.array(sources) if len(sources) > 0 else np.array([[128.0, 128.0, 50.0]])

            # Prepare previous best branch for reuse (Eq. 9)
            branch_to_reuse = None
            if self.config.enable_branch_reuse and self.previous_best_branch is not None:
                # Reuse only the nodes after the first edge (which was executed)
                # Skip the first two nodes (root and first child that was executed)
                if len(self.previous_best_branch) > 2:
                    branch_to_reuse = self.previous_best_branch[2:]
                    self._log(f"[PLANNING] Reusing {len(branch_to_reuse)} nodes from previous branch")
                else:
                    self._log(f"[PLANNING] Previous branch too short for reuse ({len(self.previous_best_branch)} nodes)")

            # Build RRT tree with branch reuse
            nodes, leaves = self.planner.build_tree(
                root_pose=current_pose,
                sources=sources_array,
                observation_positions=obs_positions,
                observation_intensities=obs_intensities,
                nearest_intensity_lookup=nearest_intensity_lookup,
                previous_best_branch=branch_to_reuse  # Eq. 9: Reuse remaining branch
            )

            # Verify tree root matches current position
            if self.config.enable_state_verification:
                root_node = nodes[0]
                assert abs(root_node.pose.x - current_pose.x) < 0.01, \
                    f"Tree root X mismatch: {root_node.pose.x} != {current_pose.x}"
                assert abs(root_node.pose.y - current_pose.y) < 0.01, \
                    f"Tree root Y mismatch: {root_node.pose.y} != {current_pose.y}"

            # Select best branch
            if leaves:
                best_branch, best_leaf_idx = self.planner.select_best_branch(nodes, leaves)
                best_gain = nodes[best_leaf_idx].cumulative_gain

                # Get goal
                goal_pose = best_branch[-1].pose if len(best_branch) > 0 else None

                if goal_pose:
                    self._log(f"[PLANNING] Goal: ({goal_pose.x:.1f}, {goal_pose.y:.1f}), "
                              f"Branch length: {len(best_branch)}, Gain: {best_gain:.6f}", "SUCCESS")
                else:
                    self._log(f"[PLANNING] No valid goal found", "WARN")

                elapsed = time.time() - start_time

                return PlanningStepResult(
                    tree_root_pose=current_pose,
                    nodes=nodes,
                    leaves=leaves,
                    best_branch=best_branch,
                    best_leaf_idx=best_leaf_idx,
                    goal_pose=goal_pose,
                    best_gain=best_gain,
                    time_elapsed=elapsed,
                    success=True
                )
            else:
                self._log(f"[PLANNING] No leaves found in tree", "WARN")
                elapsed = time.time() - start_time

                return PlanningStepResult(
                    tree_root_pose=current_pose,
                    nodes=nodes,
                    leaves=[],
                    best_branch=[],
                    best_leaf_idx=-1,
                    goal_pose=None,
                    best_gain=0.0,
                    time_elapsed=elapsed,
                    success=False
                )

        except Exception as e:
            self._log(f"[PLANNING] Failed: {e}", "ERROR")
            import traceback
            traceback.print_exc()

            return PlanningStepResult(
                tree_root_pose=current_pose,
                nodes=[],
                leaves=[],
                best_branch=[],
                best_leaf_idx=-1,
                goal_pose=None,
                best_gain=0.0,
                time_elapsed=time.time() - start_time,
                success=False
            )

    def _execution_step(self, best_branch: List[RRTNode]):
        """
        Execute path with mandatory verification.
        """
        start_time = time.time()

        if len(best_branch) < 2:
            self._log(f"[EXECUTION] No valid path to execute", "WARN")
            return ExecutionStepResult(
                pose_before=self.robot.pose,
                pose_after=self.robot.pose,
                target_pose=self.robot.pose,
                movement_distance=0.0,
                moved_successfully=False,
                observations_made=0,
                trajectory_grew=False,
                time_elapsed=time.time() - start_time,
                success=False
            )

        # Record state BEFORE movement
        pose_before = Pose2D(x=self.robot.pose.x, y=self.robot.pose.y, theta=self.robot.pose.theta)
        trajectory_len_before = len(self.robot.trajectory)

        self._log(f"[EXECUTION] Before: ({pose_before.x:.1f}, {pose_before.y:.1f}), "
                  f"Trajectory length: {trajectory_len_before}")

        # Execute FIRST EDGE only (paper mode)
        target_node = best_branch[1]
        target_pose = target_node.pose

        self._log(f"[EXECUTION] Target: ({target_pose.x:.1f}, {target_pose.y:.1f})")

        # MOVE robot
        moved = self.robot.move_to(target_pose.x, target_pose.y, target_pose.theta)

        # Make observation at new position
        if moved:
            self._make_observation(target_pose.x, target_pose.y)
            observations_made = 1
        else:
            observations_made = 0

        # Record state AFTER movement
        pose_after = self.robot.pose
        trajectory_len_after = len(self.robot.trajectory)

        self._log(f"[EXECUTION] After: ({pose_after.x:.1f}, {pose_after.y:.1f}), "
                  f"Trajectory length: {trajectory_len_after}")

        # Calculate actual movement
        distance = np.sqrt(
            (pose_after.x - pose_before.x)**2 +
            (pose_after.y - pose_before.y)**2
        )

        trajectory_grew = trajectory_len_after > trajectory_len_before

        # Verify movement
        if self.config.enable_movement_verification:
            if not moved:
                self._log(f"[EXECUTION] Robot.move_to() returned False!", "ERROR")

            if distance < 0.1:
                self._log(f"[EXECUTION] Robot barely moved! Distance = {distance:.4f}px", "ERROR")

            if not trajectory_grew:
                self._log(f"[EXECUTION] Trajectory didn't grow! "
                          f"{trajectory_len_before} → {trajectory_len_after}", "ERROR")

        success = moved and distance > 0.1 and trajectory_grew

        elapsed = time.time() - start_time

        if success:
            self._log(f"[EXECUTION] Movement successful: {distance:.1f}px ({distance*0.04:.3f}m)", "SUCCESS")
        else:
            self._log(f"[EXECUTION] Movement failed!", "ERROR")

        # Store result internally
        self._last_execution_result = ExecutionStepResult(
            pose_before=pose_before,
            pose_after=pose_after,
            target_pose=target_pose,
            movement_distance=distance,
            moved_successfully=success,
            observations_made=observations_made,
            trajectory_grew=trajectory_grew,
            time_elapsed=elapsed,
            success=success
        )

        # Return tuple for controller compatibility (observations_made, n_nodes_executed)
        n_nodes_executed = 1  # Always 1 in paper mode (first edge only)
        return (observations_made, n_nodes_executed)

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

    def _adjust_swarm_number(
        self,
        rfc: float,
        n_sources_estimated: int,
        iteration: int
    ) -> None:
        """
        Adjust swarm number based on RFC (논문 Fig. 11).

        Uses DynamicSwarmAdjuster for sophisticated adjustment logic.

        Args:
            rfc: Current RFC value
            n_sources_estimated: Number of estimated sources
            iteration: Current iteration number
        """
        # Use the sophisticated DynamicSwarmAdjuster
        should_add, reason = self.swarm_adjuster.should_add_swarm(
            estimator=self.estimator,
            iteration=iteration,
            current_rfc=rfc,
            n_valid_sources=n_sources_estimated
        )

        if should_add:
            success = self.swarm_adjuster.add_swarm_to_estimator(
                estimator=self.estimator,
                iteration=iteration,
                reason=reason
            )
            if not success and self.config.log_iteration_details:
                print(f"  ⚠️  Failed to add swarm at iteration {iteration}")

    def run_exploration(self) -> bool:
        """
        Run complete exploration with verification.

        Returns:
            True if successful
        """
        self._log(f"\n{'='*70}")
        self._log("STARTING EXPLORATION")
        self._log(f"{'='*70}\n")

        # Make initial observation
        self._make_observation(self.robot.pose.x, self.robot.pose.y)

        for iteration in range(self.config.max_iterations):
            self._log(f"\n{'='*70}")
            self._log(f"ITERATION {iteration + 1}/{self.config.max_iterations}")
            self._log(f"{'='*70}\n")

            iter_start_time = time.time()

            # STEP 1: Capture state before
            state_before = self._capture_state(iteration, 0.0)  # RFC updated after estimation

            # STEP 2: Estimation
            estimation_result = self._estimation_step()
            if estimation_result['n_sources'] == 0 and len(self.robot.observations) > 0:
                self._log(f"Estimation failed, stopping exploration", "ERROR")
                break

            # Update state with RFC
            state_before = self._capture_state(iteration, estimation_result['rfc'])

            # STEP 3: Planning
            planning_result = self._planning_step(estimation_result['sources'])
            if not planning_result.success or len(planning_result.best_branch) < 2:
                self._log(f"Planning failed, stopping exploration", "WARN")
                break

            # STEP 4: Execution
            observations_made, n_nodes_executed = self._execution_step(planning_result.best_branch)
            execution_result = self._last_execution_result  # Get stored result

            # STEP 5: Capture state after
            state_after = self._capture_state(iteration + 1, estimation_result['rfc'])

            # STEP 6: Verify changes
            robot_moved = execution_result.moved_successfully
            movement_distance = execution_result.movement_distance

            goal_changed = True
            if self.previous_goal and planning_result.goal_pose:
                goal_dist = np.sqrt(
                    (planning_result.goal_pose.x - self.previous_goal.x)**2 +
                    (planning_result.goal_pose.y - self.previous_goal.y)**2
                )
                goal_changed = goal_dist > 1.0

            self.previous_goal = planning_result.goal_pose

            # Save best branch for next iteration (Eq. 9)
            if self.config.enable_branch_reuse and len(planning_result.best_branch) > 0:
                self.previous_best_branch = planning_result.best_branch
                self._log(f"[ITERATION] Saved best branch with {len(self.previous_best_branch)} nodes for reuse")
            else:
                self.previous_best_branch = None

            # Create iteration result
            iter_result = IterationResult(
                iteration=iteration,
                state_before=state_before,
                state_after=state_after,
                estimation_result=estimation_result,
                planning_result=planning_result,
                execution_result=execution_result,
                robot_moved=robot_moved,
                movement_distance=movement_distance,
                goal_changed=goal_changed,
                total_time=time.time() - iter_start_time,
                success=robot_moved
            )

            self.iteration_results.append(iter_result)

            # Update exploration phase (논문 Fig. 14-16)
            self._update_phase(estimation_result['rfc'], iteration)

            # Dynamic swarm adjustment (논문 Fig. 11)
            self._adjust_swarm_number(
                rfc=estimation_result['rfc'],
                n_sources_estimated=estimation_result['n_sources'],
                iteration=iteration
            )

            # Log summary
            self._log(f"\n{'='*70}")
            self._log(f"ITERATION {iteration + 1} SUMMARY")
            self._log(f"{'='*70}")
            self._log(f"Robot moved: {robot_moved} ({movement_distance:.1f}px)")
            self._log(f"Goal changed: {goal_changed}")
            self._log(f"RFC: {estimation_result['rfc']:.4f}")
            self._log(f"Sources: {estimation_result['n_sources']}")
            self._log(f"Swarms: {len(self.estimator.swarms)}")
            self._log(f"Phase: {self.current_phase.name}")
            self._log(f"Time: {iter_result.total_time:.3f}s")
            self._log(f"{'='*70}\n")

            # Check termination (논문 방식)
            should_terminate, term_reason = self._check_termination(
                estimation_result['rfc'], iteration
            )
            if should_terminate:
                self._log(f"✓ Termination: {term_reason}", "SUCCESS")
                break

        self._log(f"\n{'='*70}")
        self._log("EXPLORATION COMPLETED")
        self._log(f"{'='*70}")
        self._log(f"Total iterations: {len(self.iteration_results)}")
        self._log(f"Final RFC: {self.best_rfc_overall:.4f}")
        self._log(f"Robot final position: ({self.robot.pose.x:.1f}, {self.robot.pose.y:.1f})")
        self._log(f"Trajectory points: {len(self.robot.trajectory)}")
        self._log(f"Observations: {len(self.robot.observations)}")
        self._log(f"{'='*70}\n")

        return True
