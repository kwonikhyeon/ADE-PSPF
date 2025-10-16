"""
Visualization Panels Module

Implements individual visualization panels for displaying different
aspects of the exploration process.
"""
from __future__ import annotations

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.patches import Circle, FancyArrowPatch
from typing import Optional, List, Tuple
from abc import ABC, abstractmethod

from visualization.data_manager import (
    IterationData, EstimationStepData, ExplorationStepData,
    ExecutionStepData, ExplorationSnapshot
)
from simulation.integrated_explorer_v2 import ExplorationPhase


class VisualizationPanel(ABC):
    """Base class for visualization panels."""

    def __init__(self, ax: Axes, title: str):
        """
        Initialize panel.

        Args:
            ax: Matplotlib axes to draw on
            title: Panel title
        """
        self.ax = ax
        self.title = title
        self.is_initialized = False
        self.colorbar = None  # Store colorbar reference

    @abstractmethod
    def update(self, snapshot: ExplorationSnapshot, iteration: int, step: str = 'all'):
        """
        Update panel with new data.

        Args:
            snapshot: Complete exploration snapshot
            iteration: Current iteration (0-indexed)
            step: Step to display ('all', 'estimation', 'exploration', 'execution')
        """
        pass

    def clear(self):
        """Clear the panel."""
        # Remove colorbar if it exists
        if self.colorbar is not None:
            self.colorbar.remove()
            self.colorbar = None

        self.ax.clear()
        self.ax.set_title(self.title, fontsize=10, fontweight='bold')
        self.is_initialized = False


class GroundTruthPanel(VisualizationPanel):
    """Panel 1: Ground Truth Field + Robot Trajectory."""

    def __init__(self, ax: Axes):
        super().__init__(ax, "Ground Truth & Trajectory")

    def update(self, snapshot: ExplorationSnapshot, iteration: int, step: str = 'all'):
        """Update ground truth and trajectory visualization."""
        self.clear()

        # Draw ground truth field
        im = self.ax.imshow(
            snapshot.gt_field,
            origin='lower',
            cmap='hot',
            interpolation='bilinear',
            alpha=0.7,
            extent=[0, snapshot.grid_size, 0, snapshot.grid_size]
        )

        # Draw true sources
        for i, src in enumerate(snapshot.true_sources):
            self.ax.plot(
                src[1], src[0], 'g*',
                markersize=20,
                markeredgecolor='black',
                markeredgewidth=2,
                label='True Source' if i == 0 else '',
                zorder=10
            )

        # Draw trajectory up to current iteration
        if iteration >= 0 and len(snapshot.full_trajectory) > 0:
            # Get trajectory up to current iteration
            iter_data = snapshot.iteration_data[min(iteration, len(snapshot.iteration_data) - 1)]
            n_points = min(iteration + 2, len(snapshot.full_trajectory))

            if n_points > 1:
                traj = np.array(snapshot.full_trajectory[:n_points])

                # Draw trajectory with gradient color
                colors = plt.cm.viridis(np.linspace(0, 1, len(traj)))

                for i in range(len(traj) - 1):
                    self.ax.plot(
                        traj[i:i+2, 0], traj[i:i+2, 1],
                        color=colors[i],
                        linewidth=2,
                        alpha=0.8,
                        zorder=5
                    )

                # Mark start and current position
                self.ax.plot(
                    traj[0, 0], traj[0, 1], 'go',
                    markersize=12,
                    label='Start',
                    markeredgecolor='black',
                    markeredgewidth=2,
                    zorder=8
                )

                self.ax.plot(
                    traj[-1, 0], traj[-1, 1], 'ro',
                    markersize=12,
                    label='Current',
                    markeredgecolor='black',
                    markeredgewidth=2,
                    zorder=8
                )

        # Draw observations up to current iteration
        if iteration >= 0:
            n_obs = snapshot.iteration_data[min(iteration, len(snapshot.iteration_data) - 1)].cumulative_observations
            obs_to_show = snapshot.all_observations[:n_obs]

            if obs_to_show:
                obs_array = np.array([(pos[0], pos[1]) for pos, _ in obs_to_show])
                self.ax.scatter(
                    obs_array[:, 0], obs_array[:, 1],
                    c='cyan', s=50, marker='x',
                    linewidths=2,
                    label='Observations',
                    zorder=7,
                    alpha=0.6
                )

        self.ax.set_xlabel('X (pixels)', fontsize=9)
        self.ax.set_ylabel('Y (pixels)', fontsize=9)
        self.ax.legend(loc='upper right', fontsize=8)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(0, snapshot.grid_size)
        self.ax.set_ylim(0, snapshot.grid_size)

        # Add colorbar only once
        if not self.is_initialized:
            self.colorbar = plt.colorbar(im, ax=self.ax, label='Intensity', fraction=0.046, pad=0.04)
            self.is_initialized = True


class EstimationPanel(VisualizationPanel):
    """Panel 2: ADE-PSPF Estimation (Particles, Swarms, Sources)."""

    def __init__(self, ax: Axes):
        super().__init__(ax, "Estimation (ADE-PSPF)")

    def update(self, snapshot: ExplorationSnapshot, iteration: int, step: str = 'all'):
        """Update estimation visualization."""
        self.clear()

        if iteration < 0 or iteration >= len(snapshot.iteration_data):
            return

        iter_data = snapshot.iteration_data[iteration]

        if iter_data.estimation_data is None:
            self.ax.text(0.5, 0.5, 'No estimation data', ha='center', va='center',
                        transform=self.ax.transAxes, fontsize=12)
            return

        est_data = iter_data.estimation_data

        # Draw faded ground truth as background
        self.ax.imshow(
            snapshot.gt_field,
            origin='lower',
            cmap='hot',
            interpolation='bilinear',
            alpha=0.2,
            extent=[0, snapshot.grid_size, 0, snapshot.grid_size]
        )

        # Draw particles by swarm (different colors)
        swarm_colors = plt.cm.Set1(np.linspace(0, 1, est_data.n_swarms))

        for i, (particles, color) in enumerate(zip(est_data.particles_by_swarm, swarm_colors)):
            if len(particles) > 0:
                self.ax.scatter(
                    particles[:, 0], particles[:, 1],
                    c=[color],
                    s=10,
                    alpha=0.4,
                    label=f'Swarm {i+1}' if i < 4 else None,
                    zorder=3
                )

        # Draw swarm centroids
        for i, (centroid, color) in enumerate(zip(est_data.swarm_centroids, swarm_colors)):
            self.ax.plot(
                centroid[0], centroid[1],
                marker='o',
                color=color,
                markersize=10,
                markeredgecolor='black',
                markeredgewidth=1.5,
                zorder=5
            )

        # Draw estimated sources
        for i, src in enumerate(est_data.estimated_sources):
            self.ax.plot(
                src[0], src[1], 'r^',
                markersize=15,
                markeredgecolor='black',
                markeredgewidth=2,
                label='Estimated' if i == 0 else '',
                zorder=7
            )

            # Draw circle based on intensity
            radius = max(5, min(25, src[2] / 4))
            circle = Circle(
                (src[0], src[1]),
                radius,
                fill=False,
                edgecolor='red',
                linewidth=2,
                alpha=0.6,
                zorder=6
            )
            self.ax.add_patch(circle)

        # Draw true sources for reference
        for i, src in enumerate(snapshot.true_sources):
            self.ax.plot(
                src[1], src[0], 'g*',
                markersize=15,
                markeredgecolor='black',
                markeredgewidth=1,
                alpha=0.5,
                label='True' if i == 0 else '',
                zorder=4
            )

        self.ax.set_xlabel('X (pixels)', fontsize=9)
        self.ax.set_ylabel('Y (pixels)', fontsize=9)
        self.ax.legend(loc='upper right', fontsize=8)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(0, snapshot.grid_size)
        self.ax.set_ylim(0, snapshot.grid_size)

        # Add RFC info
        self.ax.text(
            0.02, 0.98,
            f'RFC: {est_data.rfc:.4f}\nSources: {est_data.n_sources}\nSwarms: {est_data.n_swarms}',
            transform=self.ax.transAxes,
            fontsize=9,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7)
        )


class ExplorationPanel(VisualizationPanel):
    """Panel 3: RRT Tree and Path Planning."""

    def __init__(self, ax: Axes):
        super().__init__(ax, "Exploration (RRT Planning)")

    def update(self, snapshot: ExplorationSnapshot, iteration: int, step: str = 'all'):
        """Update RRT tree visualization."""
        self.clear()

        if iteration < 0 or iteration >= len(snapshot.iteration_data):
            return

        iter_data = snapshot.iteration_data[iteration]

        if iter_data.exploration_data is None:
            self.ax.text(0.5, 0.5, 'No exploration data', ha='center', va='center',
                        transform=self.ax.transAxes, fontsize=12)
            return

        exp_data = iter_data.exploration_data

        # Draw very faded ground truth
        self.ax.imshow(
            snapshot.gt_field,
            origin='lower',
            cmap='gray',
            interpolation='bilinear',
            alpha=0.15,
            extent=[0, snapshot.grid_size, 0, snapshot.grid_size]
        )

        # Draw RRT nodes and edges
        for node in exp_data.rrt_nodes:
            if node.parent is not None:
                parent = exp_data.rrt_nodes[node.parent]
                self.ax.plot(
                    [parent.pose.x, node.pose.x],
                    [parent.pose.y, node.pose.y],
                    'gray', linewidth=0.5, alpha=0.3, zorder=1
                )

        # Draw nodes colored by cumulative gain
        if len(exp_data.cumulative_gains) > 0:
            gains = np.array(exp_data.cumulative_gains)
            gains_normalized = (gains - gains.min()) / (gains.max() - gains.min() + 1e-10)

            positions = np.array([[n.pose.x, n.pose.y] for n in exp_data.rrt_nodes])

            scatter = self.ax.scatter(
                positions[:, 0], positions[:, 1],
                c=gains_normalized,
                cmap='viridis',
                s=20,
                alpha=0.5,
                zorder=3
            )

        # Draw best branch
        if len(exp_data.best_branch) > 1:
            branch_positions = np.array([[n.pose.x, n.pose.y] for n in exp_data.best_branch])

            self.ax.plot(
                branch_positions[:, 0], branch_positions[:, 1],
                'r-',
                linewidth=3,
                alpha=0.9,
                label='Best Path',
                zorder=5
            )

            # Mark start and end of best branch
            self.ax.plot(
                branch_positions[0, 0], branch_positions[0, 1],
                'go', markersize=10,
                markeredgecolor='black',
                markeredgewidth=1.5,
                label='Start',
                zorder=6
            )

            self.ax.plot(
                branch_positions[-1, 0], branch_positions[-1, 1],
                'ro', markersize=10,
                markeredgecolor='black',
                markeredgewidth=1.5,
                label='Goal',
                zorder=6
            )

        # Draw leaf nodes
        if exp_data.rrt_leaves:
            leaf_positions = np.array([
                [exp_data.rrt_nodes[i].pose.x, exp_data.rrt_nodes[i].pose.y]
                for i in exp_data.rrt_leaves
            ])
            self.ax.scatter(
                leaf_positions[:, 0], leaf_positions[:, 1],
                c='orange', s=80, marker='*',
                edgecolors='black',
                linewidths=1,
                label='Leaves',
                zorder=4
            )

        # Draw estimated sources
        for i, src in enumerate(exp_data.estimated_sources):
            self.ax.plot(
                src[0], src[1], 'b^',
                markersize=12,
                markeredgecolor='black',
                markeredgewidth=1,
                alpha=0.6,
                label='Est. Source' if i == 0 else '',
                zorder=7
            )

        self.ax.set_xlabel('X (pixels)', fontsize=9)
        self.ax.set_ylabel('Y (pixels)', fontsize=9)
        self.ax.legend(loc='upper right', fontsize=8)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(0, snapshot.grid_size)
        self.ax.set_ylim(0, snapshot.grid_size)

        # Add info
        self.ax.text(
            0.02, 0.98,
            f'Nodes: {exp_data.n_nodes}\nBest Gain: {exp_data.best_gain:.6f}\nBranch Length: {len(exp_data.best_branch)}',
            transform=self.ax.transAxes,
            fontsize=9,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7)
        )


class ExecutionPanel(VisualizationPanel):
    """Panel 4: Execution Details (Movement and Observation)."""

    def __init__(self, ax: Axes):
        super().__init__(ax, "Execution (Movement)")

    def update(self, snapshot: ExplorationSnapshot, iteration: int, step: str = 'all'):
        """Update execution visualization."""
        self.clear()

        if iteration < 0 or iteration >= len(snapshot.iteration_data):
            return

        iter_data = snapshot.iteration_data[iteration]

        if iter_data.execution_data is None:
            self.ax.text(0.5, 0.5, 'No execution data', ha='center', va='center',
                        transform=self.ax.transAxes, fontsize=12)
            return

        exec_data = iter_data.execution_data

        # Get zoom region around robot
        center_x = (exec_data.pose_before.x + exec_data.pose_after.x) / 2
        center_y = (exec_data.pose_before.y + exec_data.pose_after.y) / 2
        zoom_size = 80  # pixels

        x_min = max(0, center_x - zoom_size)
        x_max = min(snapshot.grid_size, center_x + zoom_size)
        y_min = max(0, center_y - zoom_size)
        y_max = min(snapshot.grid_size, center_y + zoom_size)

        # Draw ground truth (zoomed)
        self.ax.imshow(
            snapshot.gt_field,
            origin='lower',
            cmap='hot',
            interpolation='bilinear',
            alpha=0.4,
            extent=[0, snapshot.grid_size, 0, snapshot.grid_size]
        )

        # Draw planned path
        if len(exec_data.planned_path) > 1:
            path_positions = np.array([[n.pose.x, n.pose.y] for n in exec_data.planned_path])
            self.ax.plot(
                path_positions[:, 0], path_positions[:, 1],
                'gray', linewidth=2, linestyle='--',
                alpha=0.5,
                label='Planned Path',
                zorder=3
            )

        # Draw executed portion (first edge or ratio)
        if exec_data.n_nodes_to_execute > 0 and len(exec_data.planned_path) > exec_data.n_nodes_to_execute:
            executed_path = exec_data.planned_path[:exec_data.n_nodes_to_execute + 1]
            exec_positions = np.array([[n.pose.x, n.pose.y] for n in executed_path])
            self.ax.plot(
                exec_positions[:, 0], exec_positions[:, 1],
                'b-', linewidth=3,
                alpha=0.8,
                label='Executed',
                zorder=4
            )

        # Draw robot position before (blue)
        self.ax.plot(
            exec_data.pose_before.x, exec_data.pose_before.y,
            'bo', markersize=15,
            markeredgecolor='black',
            markeredgewidth=2,
            label='Before',
            zorder=6
        )

        # Draw arrow showing heading before
        dx_before = 10 * np.cos(exec_data.pose_before.theta)
        dy_before = 10 * np.sin(exec_data.pose_before.theta)
        arrow_before = FancyArrowPatch(
            (exec_data.pose_before.x, exec_data.pose_before.y),
            (exec_data.pose_before.x + dx_before, exec_data.pose_before.y + dy_before),
            arrowstyle='->', mutation_scale=20, linewidth=2,
            color='blue', zorder=5
        )
        self.ax.add_patch(arrow_before)

        # Draw robot position after (red)
        self.ax.plot(
            exec_data.pose_after.x, exec_data.pose_after.y,
            'ro', markersize=15,
            markeredgecolor='black',
            markeredgewidth=2,
            label='After',
            zorder=6
        )

        # Draw arrow showing heading after
        dx_after = 10 * np.cos(exec_data.pose_after.theta)
        dy_after = 10 * np.sin(exec_data.pose_after.theta)
        arrow_after = FancyArrowPatch(
            (exec_data.pose_after.x, exec_data.pose_after.y),
            (exec_data.pose_after.x + dx_after, exec_data.pose_after.y + dy_after),
            arrowstyle='->', mutation_scale=20, linewidth=2,
            color='red', zorder=5
        )
        self.ax.add_patch(arrow_after)

        # Draw movement vector
        self.ax.annotate(
            '',
            xy=(exec_data.pose_after.x, exec_data.pose_after.y),
            xytext=(exec_data.pose_before.x, exec_data.pose_before.y),
            arrowprops=dict(arrowstyle='->', lw=2, color='green', alpha=0.7),
            zorder=5
        )

        # Draw new observations
        for i, (pos, intensity) in enumerate(exec_data.new_observations):
            self.ax.plot(
                pos[0], pos[1], 'g*',
                markersize=12,
                markeredgecolor='black',
                markeredgewidth=1.5,
                label='New Obs' if i == 0 else '',
                zorder=7
            )

        self.ax.set_xlabel('X (pixels)', fontsize=9)
        self.ax.set_ylabel('Y (pixels)', fontsize=9)
        self.ax.legend(loc='upper right', fontsize=8)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)

        # Add info
        self.ax.text(
            0.02, 0.98,
            f'Distance: {exec_data.actual_distance:.1f} px\n'
            f'New Obs: {exec_data.n_new_observations}\n'
            f'Executed: {exec_data.n_nodes_to_execute}/{exec_data.n_planned_nodes} nodes',
            transform=self.ax.transAxes,
            fontsize=9,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7)
        )


class ConvergencePanel(VisualizationPanel):
    """Panel 5: RFC Convergence History."""

    def __init__(self, ax: Axes):
        super().__init__(ax, "RFC Convergence")

    def update(self, snapshot: ExplorationSnapshot, iteration: int, step: str = 'all'):
        """Update convergence plot."""
        self.clear()

        if len(snapshot.iteration_data) == 0:
            return

        # Extract RFC values
        iterations_list = list(range(1, len(snapshot.iteration_data) + 1))
        rfc_values = [iter_data.rfc_after for iter_data in snapshot.iteration_data]

        # Plot RFC
        self.ax.plot(
            iterations_list, rfc_values,
            'b-o', linewidth=2, markersize=6,
            label='RFC', zorder=3
        )

        # Highlight current iteration
        if 0 <= iteration < len(snapshot.iteration_data):
            current_rfc = snapshot.iteration_data[iteration].rfc_after
            self.ax.plot(
                iteration + 1, current_rfc,
                'ro', markersize=12,
                markeredgecolor='black',
                markeredgewidth=2,
                label='Current',
                zorder=5
            )

        # Draw threshold line
        self.ax.axhline(
            y=0.85, color='r', linestyle='--',
            linewidth=2, label='Threshold (0.85)',
            zorder=2
        )

        # Fill area under curve
        self.ax.fill_between(
            iterations_list, 0, rfc_values,
            alpha=0.3, color='blue', zorder=1
        )

        # Mark phase transitions
        phase_colors = {
            ExplorationPhase.TRACING: 'yellow',
            ExplorationPhase.SURROUNDING: 'orange',
            ExplorationPhase.EXPLORING: 'green'
        }

        for phase_iter, phase in snapshot.phase_history:
            if phase_iter <= len(iterations_list):
                self.ax.axvline(
                    x=phase_iter + 1,
                    color=phase_colors.get(phase, 'gray'),
                    linestyle=':',
                    linewidth=1.5,
                    alpha=0.5,
                    zorder=2
                )

        self.ax.set_xlabel('Iteration', fontsize=9, fontweight='bold')
        self.ax.set_ylabel('RFC', fontsize=9, fontweight='bold')
        self.ax.legend(fontsize=8, loc='lower right')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_ylim([0, 1.0])
        self.ax.set_xlim([0.5, len(iterations_list) + 0.5])

        # Add best RFC line
        if snapshot.best_rfc > 0:
            self.ax.axhline(
                y=snapshot.best_rfc, color='green',
                linestyle='-.', linewidth=1.5,
                label=f'Best ({snapshot.best_rfc:.4f})',
                alpha=0.7, zorder=2
            )


class StatisticsPanel(VisualizationPanel):
    """Panel 6: Text-based Statistics Display."""

    def __init__(self, ax: Axes):
        super().__init__(ax, "Statistics")

    def update(self, snapshot: ExplorationSnapshot, iteration: int, step: str = 'all'):
        """Update statistics display."""
        self.clear()
        self.ax.axis('off')

        if iteration < 0 or iteration >= len(snapshot.iteration_data):
            return

        iter_data = snapshot.iteration_data[iteration]

        # Build statistics text
        stats_text = "═" * 35 + "\n"
        stats_text += f"ITERATION {iteration + 1}/{snapshot.n_iterations}\n"
        stats_text += "═" * 35 + "\n\n"

        # Phase info
        phase_names = {
            ExplorationPhase.TRACING: "TRACING",
            ExplorationPhase.SURROUNDING: "SURROUNDING",
            ExplorationPhase.EXPLORING: "EXPLORING"
        }
        stats_text += f"Phase: {phase_names.get(iter_data.phase, 'UNKNOWN')}\n\n"

        # Estimation info
        if iter_data.estimation_data:
            est = iter_data.estimation_data
            stats_text += "─── Estimation ───\n"
            stats_text += f"  RFC:          {est.rfc:.4f}\n"
            stats_text += f"  Best RFC:     {est.best_rfc:.4f}\n"
            stats_text += f"  Sources:      {est.n_sources}\n"
            stats_text += f"  Swarms:       {est.n_swarms}\n"
            stats_text += f"  Observations: {est.n_observations}\n"
            stats_text += f"  Time:         {est.time_elapsed:.3f}s\n\n"

        # Exploration info
        if iter_data.exploration_data:
            exp = iter_data.exploration_data
            stats_text += "─── Exploration ───\n"
            stats_text += f"  RRT Nodes:    {exp.n_nodes}\n"
            stats_text += f"  Leaf Nodes:   {len(exp.rrt_leaves)}\n"
            stats_text += f"  Best Gain:    {exp.best_gain:.6f}\n"
            stats_text += f"  Branch Len:   {len(exp.best_branch)}\n"
            stats_text += f"  Time:         {exp.time_elapsed:.3f}s\n\n"

        # Execution info
        if iter_data.execution_data:
            exec_data = iter_data.execution_data
            stats_text += "─── Execution ───\n"
            stats_text += f"  Distance:     {exec_data.actual_distance:.1f} px\n"
            stats_text += f"  Distance:     {exec_data.actual_distance * snapshot.pixel_resolution:.3f} m\n"
            stats_text += f"  New Obs:      {exec_data.n_new_observations}\n"
            stats_text += f"  Executed:     {exec_data.n_nodes_to_execute}/{exec_data.n_planned_nodes}\n"
            stats_text += f"  Time:         {exec_data.time_elapsed:.3f}s\n\n"

        # Cumulative stats
        stats_text += "─── Cumulative ───\n"
        stats_text += f"  Total Obs:    {iter_data.cumulative_observations}\n"
        stats_text += f"  Total Dist:   {iter_data.total_distance_traveled:.1f} px\n"
        stats_text += f"  Total Time:   {iter_data.time_total:.3f}s\n\n"

        # Final status
        stats_text += "─── Status ───\n"
        stats_text += f"  Converged:    {'YES ✓' if snapshot.converged else 'NO ✗'}\n"
        if snapshot.converged:
            stats_text += f"  Conv. Iter:   {snapshot.convergence_iteration}\n"

        # Display text
        self.ax.text(
            0.05, 0.95,
            stats_text,
            fontsize=8,
            family='monospace',
            verticalalignment='top',
            transform=self.ax.transAxes,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )
