"""
Interactive Explorer Application

Main GUI application for visualizing the ADE-PSPF exploration process.
Combines Tkinter for controls and Matplotlib for visualization.
"""
from __future__ import annotations

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import numpy as np
from typing import Optional
from pathlib import Path

from visualization.explorer_controller import ExplorerController
from visualization.data_manager import DataManager, ExplorationSnapshot
from visualization.visualization_panels import (
    GroundTruthPanel, EstimationPanel, ExplorationPanel,
    ExecutionPanel, ConvergencePanel, StatisticsPanel
)


class InteractiveExplorerApp:
    """
    Interactive visualization application for multi-source exploration.

    Features:
    - Step-by-step visualization of each iteration
    - Interactive controls for navigation
    - Ground truth generation and configuration
    - Auto-play animation
    - Session save/load
    - Image export
    """

    def __init__(self, root: tk.Tk):
        """Initialize the application."""
        self.root = root
        self.root.title("ADE-PSPF Interactive Explorer")
        self.root.geometry("1600x1600")

        # Set window close protocol to use safe exit
        self.root.protocol("WM_DELETE_WINDOW", self._safe_exit)

        # Controllers and data
        self.controller = ExplorerController()
        self.data_manager = DataManager()
        self.snapshot: Optional[ExplorationSnapshot] = None

        # UI state
        self.current_iteration = 0
        self.is_playing = False
        self.play_speed = 1.0  # seconds per iteration

        # Create UI components
        self._create_menubar()
        self._create_control_panel()
        self._create_visualization_area()
        self._create_navigation_panel()
        self._create_status_bar()

        # Initialize with default GT
        self._generate_default_gt()

    def _create_menubar(self):
        """Create menu bar."""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Session...", command=self._save_session)
        file_menu.add_command(label="Load Session...", command=self._load_session)
        file_menu.add_separator()
        file_menu.add_command(label="Export Images...", command=self._export_images)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self._safe_exit)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self._show_about)

    def _create_control_panel(self):
        """Create control panel at top."""
        control_frame = ttk.LabelFrame(self.root, text="Control Panel", padding="10")
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)

        # Ground Truth controls
        gt_frame = ttk.Frame(control_frame)
        gt_frame.pack(side=tk.LEFT, padx=10)

        ttk.Label(gt_frame, text="Sources:").grid(row=0, column=0, padx=5)
        self.sources_var = tk.IntVar(value=3)
        sources_spinbox = ttk.Spinbox(
            gt_frame, from_=1, to=10, width=5,
            textvariable=self.sources_var
        )
        sources_spinbox.grid(row=0, column=1, padx=5)

        ttk.Label(gt_frame, text="Seed:").grid(row=0, column=2, padx=5)
        self.seed_var = tk.IntVar(value=42)
        seed_entry = ttk.Entry(gt_frame, width=10, textvariable=self.seed_var)
        seed_entry.grid(row=0, column=3, padx=5)

        ttk.Button(
            gt_frame, text="Generate New GT",
            command=self._generate_new_gt
        ).grid(row=0, column=4, padx=5)

        # Exploration controls
        exp_frame = ttk.Frame(control_frame)
        exp_frame.pack(side=tk.LEFT, padx=20)

        ttk.Label(exp_frame, text="Max Iterations:").grid(row=0, column=0, padx=5)
        self.max_iter_var = tk.IntVar(value=15)
        max_iter_spinbox = ttk.Spinbox(
            exp_frame, from_=1, to=100, width=5,
            textvariable=self.max_iter_var
        )
        max_iter_spinbox.grid(row=0, column=1, padx=5)

        self.run_button = ttk.Button(
            exp_frame, text="Run Exploration",
            command=self._run_exploration
        )
        self.run_button.grid(row=0, column=2, padx=5)

        self.status_label = ttk.Label(exp_frame, text="Ready", foreground="green")
        self.status_label.grid(row=0, column=3, padx=10)

    def _create_visualization_area(self):
        """Create visualization area with 6 panels."""
        # Create matplotlib figure
        self.fig = Figure(figsize=(16, 10))
        self.fig.suptitle('ADE-PSPF Interactive Explorer', fontsize=14, fontweight='bold')

        # Create 2x3 grid of subplots
        gs = self.fig.add_gridspec(2, 3, hspace=0.3, wspace=0.3)

        ax1 = self.fig.add_subplot(gs[0, 0])
        ax2 = self.fig.add_subplot(gs[0, 1])
        ax3 = self.fig.add_subplot(gs[0, 2])
        ax4 = self.fig.add_subplot(gs[1, 0])
        ax5 = self.fig.add_subplot(gs[1, 1])
        ax6 = self.fig.add_subplot(gs[1, 2])

        # Create panels
        self.panels = {
            'ground_truth': GroundTruthPanel(ax1),
            'estimation': EstimationPanel(ax2),
            'exploration': ExplorationPanel(ax3),
            'execution': ExecutionPanel(ax4),
            'convergence': ConvergencePanel(ax5),
            'statistics': StatisticsPanel(ax6)
        }

        # Embed in Tkinter
        canvas_frame = ttk.Frame(self.root)
        canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Add matplotlib toolbar
        toolbar = NavigationToolbar2Tk(self.canvas, canvas_frame)
        toolbar.update()

    def _create_navigation_panel(self):
        """Create navigation panel at bottom."""
        nav_frame = ttk.LabelFrame(self.root, text="Navigation", padding="10")
        nav_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)

        # Iteration controls
        control_row = ttk.Frame(nav_frame)
        control_row.pack(side=tk.TOP, fill=tk.X, pady=5)

        # First/Previous/Next/Last buttons
        ttk.Button(
            control_row, text="⏮ First",
            command=self._go_to_first
        ).pack(side=tk.LEFT, padx=2)

        ttk.Button(
            control_row, text="◀ Prev",
            command=self._go_to_prev
        ).pack(side=tk.LEFT, padx=2)

        # Iteration slider
        slider_frame = ttk.Frame(control_row)
        slider_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)

        self.iteration_var = tk.IntVar(value=0)
        self.iteration_slider = ttk.Scale(
            slider_frame,
            from_=0, to=0,
            orient=tk.HORIZONTAL,
            variable=self.iteration_var,
            command=self._on_slider_change
        )
        self.iteration_slider.pack(side=tk.TOP, fill=tk.X)

        self.iteration_label = ttk.Label(
            slider_frame,
            text="Iteration: 0/0"
        )
        self.iteration_label.pack(side=tk.TOP)

        ttk.Button(
            control_row, text="Next ▶",
            command=self._go_to_next
        ).pack(side=tk.LEFT, padx=2)

        ttk.Button(
            control_row, text="Last ⏭",
            command=self._go_to_last
        ).pack(side=tk.LEFT, padx=2)

        # Auto-play controls
        play_row = ttk.Frame(nav_frame)
        play_row.pack(side=tk.TOP, fill=tk.X, pady=5)

        self.play_button = ttk.Button(
            play_row, text="▶ Play",
            command=self._toggle_play
        )
        self.play_button.pack(side=tk.LEFT, padx=5)

        ttk.Label(play_row, text="Speed:").pack(side=tk.LEFT, padx=5)

        self.speed_var = tk.DoubleVar(value=1.0)
        speed_slider = ttk.Scale(
            play_row,
            from_=0.1, to=3.0,
            orient=tk.HORIZONTAL,
            variable=self.speed_var,
            length=150
        )
        speed_slider.pack(side=tk.LEFT, padx=5)

        self.speed_label = ttk.Label(play_row, text="1.0s")
        self.speed_label.pack(side=tk.LEFT, padx=5)

        self.speed_var.trace_add('write', self._update_speed_label)

        # Exit button
        exit_row = ttk.Frame(nav_frame)
        exit_row.pack(side=tk.TOP, fill=tk.X, pady=10)

        ttk.Button(
            exit_row,
            text="🚪 Exit Application",
            command=self._safe_exit
        ).pack(side=tk.TOP, padx=5, pady=5)

    def _create_status_bar(self):
        """Create status bar at very bottom."""
        self.status_bar = ttk.Label(
            self.root,
            text="Ready",
            relief=tk.SUNKEN,
            anchor=tk.W
        )
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    # ============== Ground Truth Management ==============

    def _generate_default_gt(self):
        """Generate default ground truth on startup."""
        self._set_status("Generating default ground truth...", "blue")
        success = self.controller.generate_ground_truth(
            n_sources=self.sources_var.get(),
            seed=self.seed_var.get()
        )
        if success:
            self._set_status("Default GT generated. Click 'Run Exploration' to start.", "green")
        else:
            self._set_status("Failed to generate GT", "red")

    def _generate_new_gt(self):
        """Generate new ground truth with current settings."""
        self._set_status("Generating new ground truth...", "blue")

        success = self.controller.generate_ground_truth(
            n_sources=self.sources_var.get(),
            seed=self.seed_var.get()
        )

        if success:
            self._set_status("New GT generated. Click 'Run Exploration' to start.", "green")
            self.snapshot = None
            self.current_iteration = 0
            self._update_visualization()
        else:
            self._set_status("Failed to generate GT", "red")
            messagebox.showerror("Error", "Failed to generate ground truth")

    # ============== Exploration Management ==============

    def _run_exploration(self):
        """Run complete exploration with snapshot capture."""
        self._set_status("Running exploration...", "blue")
        self.run_button.config(state='disabled')

        # Force GUI update to show status change
        self.root.update_idletasks()

        # Initialize explorer
        success = self.controller.initialize_explorer(
            max_iterations=self.max_iter_var.get(),
            n_swarms=4,
            n_particles=80,
            ade_generations=3,
            branch_execution_ratio=0.0,
            observations_per_iteration=1,
            enable_logs=True  # Enable logs to show progress
        )

        if not success:
            self._set_status("Failed to initialize explorer", "red")
            self.run_button.config(state='normal')
            messagebox.showerror("Error", "Failed to initialize explorer")
            return

        # Run exploration with progress callback
        def progress_update(iteration, max_iter, status):
            """Update GUI with progress."""
            progress_text = f"[{iteration}/{max_iter}] {status}"
            self._set_status(progress_text, "blue")
            self.root.update()  # Force GUI update to prevent freezing

        success = self.controller.run_exploration_with_snapshots(
            progress_callback=progress_update
        )

        if success:
            self.snapshot = self.controller.get_snapshot()
            self.current_iteration = 0

            # Update slider range
            if self.snapshot:
                max_iter = self.snapshot.n_iterations - 1
                self.iteration_slider.config(to=max_iter)
                self.iteration_var.set(0)

            self._update_visualization()
            self._set_status(
                f"Exploration complete! {self.snapshot.n_iterations} iterations, "
                f"RFC: {self.snapshot.final_rfc:.4f}",
                "green"
            )
        else:
            self._set_status("Exploration failed", "red")
            messagebox.showerror("Error", "Exploration failed")

        self.run_button.config(state='normal')

    # ============== Navigation ==============

    def _go_to_first(self):
        """Go to first iteration."""
        if self.snapshot:
            self.current_iteration = 0
            self.iteration_var.set(0)
            self._update_visualization()

    def _go_to_prev(self):
        """Go to previous iteration."""
        if self.snapshot and self.current_iteration > 0:
            self.current_iteration -= 1
            self.iteration_var.set(self.current_iteration)
            self._update_visualization()

    def _go_to_next(self):
        """Go to next iteration."""
        if self.snapshot and self.current_iteration < self.snapshot.n_iterations - 1:
            self.current_iteration += 1
            self.iteration_var.set(self.current_iteration)
            self._update_visualization()

    def _go_to_last(self):
        """Go to last iteration."""
        if self.snapshot:
            self.current_iteration = self.snapshot.n_iterations - 1
            self.iteration_var.set(self.current_iteration)
            self._update_visualization()

    def _on_slider_change(self, value):
        """Handle slider value change."""
        if self.snapshot and not self.is_playing:
            self.current_iteration = int(float(value))
            self._update_visualization()

    # ============== Auto-play ==============

    def _toggle_play(self):
        """Toggle auto-play mode."""
        if not self.snapshot:
            messagebox.showwarning("Warning", "No exploration data. Run exploration first.")
            return

        self.is_playing = not self.is_playing

        if self.is_playing:
            self.play_button.config(text="⏸ Pause")
            self._play_next_frame()
        else:
            self.play_button.config(text="▶ Play")

    def _play_next_frame(self):
        """Play next frame in animation."""
        if not self.is_playing:
            return

        if self.current_iteration < self.snapshot.n_iterations - 1:
            self.current_iteration += 1
            self.iteration_var.set(self.current_iteration)
            self._update_visualization()

            # Schedule next frame
            delay = int(self.speed_var.get() * 1000)
            self.root.after(delay, self._play_next_frame)
        else:
            # Reached end
            self.is_playing = False
            self.play_button.config(text="▶ Play")

    def _update_speed_label(self, *args):
        """Update speed label."""
        speed = self.speed_var.get()
        self.speed_label.config(text=f"{speed:.1f}s")

    # ============== Visualization ==============

    def _update_visualization(self):
        """Update all visualization panels."""
        if not self.snapshot:
            return

        # Update iteration label with trajectory info
        n_traj_points = len(self.snapshot.full_trajectory)
        if n_traj_points > 0:
            current_pos = self.snapshot.full_trajectory[min(self.current_iteration, n_traj_points-1)]
            iter_text = (f"Iteration: {self.current_iteration + 1}/{self.snapshot.n_iterations} | "
                        f"Robot: ({current_pos[0]:.1f}, {current_pos[1]:.1f}) | "
                        f"Trajectory: {n_traj_points} points")
        else:
            iter_text = f"Iteration: {self.current_iteration + 1}/{self.snapshot.n_iterations}"

        self.iteration_label.config(text=iter_text)

        # Update all panels
        for panel in self.panels.values():
            panel.update(self.snapshot, self.current_iteration, 'all')

        self.canvas.draw()

    # ============== File Operations ==============

    def _save_session(self):
        """Save current session to file."""
        if not self.snapshot:
            messagebox.showwarning("Warning", "No exploration data to save.")
            return

        filepath = filedialog.asksaveasfilename(
            defaultextension=".pkl",
            filetypes=[("Pickle files", "*.pkl"), ("All files", "*.*")]
        )

        if filepath:
            success = self.data_manager.save_snapshot(self.snapshot, filepath)
            if success:
                messagebox.showinfo("Success", f"Session saved to:\n{filepath}")
            else:
                messagebox.showerror("Error", "Failed to save session")

    def _load_session(self):
        """Load session from file."""
        filepath = filedialog.askopenfilename(
            filetypes=[("Pickle files", "*.pkl"), ("All files", "*.*")]
        )

        if filepath:
            snapshot = self.data_manager.load_snapshot(filepath)
            if snapshot:
                self.snapshot = snapshot
                self.current_iteration = 0

                # Update slider
                max_iter = self.snapshot.n_iterations - 1
                self.iteration_slider.config(to=max_iter)
                self.iteration_var.set(0)

                # Update GT settings
                self.sources_var.set(snapshot.n_true_sources)
                self.seed_var.set(snapshot.random_seed)

                self._update_visualization()
                self._set_status(
                    f"Session loaded: {snapshot.n_iterations} iterations",
                    "green"
                )
                messagebox.showinfo("Success", f"Session loaded from:\n{filepath}")
            else:
                messagebox.showerror("Error", "Failed to load session")

    def _export_images(self):
        """Export all iterations as images."""
        if not self.snapshot:
            messagebox.showwarning("Warning", "No exploration data to export.")
            return

        output_dir = filedialog.askdirectory(title="Select Output Directory")

        if output_dir:
            self._set_status("Exporting images...", "blue")

            def render_func(iter_data, output_path):
                # Temporarily update to this iteration
                old_iter = self.current_iteration
                self.current_iteration = iter_data.iteration
                self._update_visualization()
                self.fig.savefig(output_path, dpi=150, bbox_inches='tight')
                self.current_iteration = old_iter

            success = self.data_manager.export_iteration_images(
                self.snapshot, output_dir, render_func
            )

            if success:
                self._set_status(f"Images exported to {output_dir}", "green")
                messagebox.showinfo("Success", f"Images exported to:\n{output_dir}")
            else:
                self._set_status("Failed to export images", "red")
                messagebox.showerror("Error", "Failed to export images")

    # ============== Utility ==============

    def _set_status(self, message: str, color: str = "black"):
        """Set status message."""
        self.status_bar.config(text=message)
        self.status_label.config(text=message, foreground=color)
        self.root.update_idletasks()

    def _safe_exit(self):
        """Safely exit the application with confirmation and cleanup."""
        # Stop auto-play if running
        if self.is_playing:
            self.is_playing = False
            self.play_button.config(text="▶ Play")

        # Ask for confirmation
        result = messagebox.askyesno(
            "Exit Application",
            "Are you sure you want to exit?\n\nAny unsaved data will be lost.",
            icon='question'
        )

        if result:
            try:
                # Clean up matplotlib resources
                plt.close('all')

                # Update status
                self.status_bar.config(text="Shutting down...")
                self.root.update_idletasks()

                # Destroy the window
                self.root.quit()
                self.root.destroy()

            except Exception as e:
                # If cleanup fails, force quit
                print(f"Error during cleanup: {e}")
                self.root.destroy()

    def _show_about(self):
        """Show about dialog."""
        about_text = """
ADE-PSPF Interactive Explorer v1.0

Interactive visualization tool for multi-source
radiation exploration using ADE-PSPF algorithm.

Features:
• Step-by-step iteration visualization
• Ground truth generation and configuration
• Auto-play animation
• Session save/load
• Image export

Developed for research visualization.
        """
        messagebox.showinfo("About", about_text)

    def run(self):
        """Start the application main loop."""
        self.root.mainloop()


def main():
    """Main entry point."""
    root = tk.Tk()
    app = InteractiveExplorerApp(root)
    app.run()


if __name__ == "__main__":
    main()
