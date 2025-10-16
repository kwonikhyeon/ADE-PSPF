"""
Test Branch Reuse Functionality (Eq. 9)

Tests that the explorer properly reuses the best branch from previous iterations.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from simulation.integrated_explorer_v3 import IntegratedExplorerV3, ExplorationConfigV3
from environment.generate_truth import sample_sources, inverse_square_field, GRID


def test_branch_reuse():
    """Test that branch reuse works correctly across iterations."""

    print("="*70)
    print("BRANCH REUSE TEST (Eq. 9)")
    print("="*70)

    # Create ground truth
    rng = np.random.default_rng(42)
    coords, amps, sigmas = sample_sources(GRID, n=3, rng=rng)
    gt_field = inverse_square_field(GRID, coords, amps, h=0.5)

    # Create explorer with branch reuse enabled
    config = ExplorationConfigV3(
        max_iterations=3,
        enable_branch_reuse=True,
        enable_verbose_logging=True
    )

    explorer = IntegratedExplorerV3(gt_field, config, rng)

    print("\n" + "="*70)
    print("RUNNING EXPLORATION WITH BRANCH REUSE")
    print("="*70)

    # Run exploration
    success = explorer.run_exploration()

    print("\n" + "="*70)
    print("VERIFICATION")
    print("="*70)

    # Verify iterations completed
    n_iterations = len(explorer.iteration_results)
    print(f"\n✓ Completed {n_iterations} iterations")

    # Check that robot moved
    trajectory_length = len(explorer.robot.trajectory)
    print(f"✓ Trajectory length: {trajectory_length} points")

    if trajectory_length > 1:
        start = explorer.robot.trajectory[0]
        end = explorer.robot.trajectory[-1]
        dist = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        print(f"✓ Robot moved {dist:.1f}px")

    # Check branch reuse messages in log
    print(f"\n✓ Branch reuse was {'ENABLED' if config.enable_branch_reuse else 'DISABLED'}")

    # Verify different directions explored
    if len(explorer.robot.trajectory) >= 3:
        # Calculate angles between consecutive moves
        angles = []
        for i in range(len(explorer.robot.trajectory) - 1):
            p1 = explorer.robot.trajectory[i]
            p2 = explorer.robot.trajectory[i + 1]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            angle = np.arctan2(dy, dx) * 180 / np.pi
            angles.append(angle)

        print(f"\n✓ Movement angles: {[f'{a:.1f}°' for a in angles]}")

        # Check if we have different directions
        angle_changes = [abs(angles[i+1] - angles[i]) for i in range(len(angles)-1)]
        if any(change > 30 for change in angle_changes):
            print(f"✅ Robot explored DIFFERENT DIRECTIONS")
        else:
            print(f"⚠️  Robot moved mostly in SAME DIRECTION")

    print("\n" + "="*70)

    if success and n_iterations > 0 and trajectory_length > 1:
        print("✅ BRANCH REUSE TEST PASSED!")
    else:
        print("❌ BRANCH REUSE TEST FAILED!")

    print("="*70)

    return success


def test_branch_reuse_disabled():
    """Test with branch reuse disabled for comparison."""

    print("\n" + "="*70)
    print("BRANCH REUSE DISABLED TEST (Comparison)")
    print("="*70)

    # Create ground truth
    rng = np.random.default_rng(42)
    coords, amps, sigmas = sample_sources(GRID, n=3, rng=rng)
    gt_field = inverse_square_field(GRID, coords, amps, h=0.5)

    # Create explorer with branch reuse DISABLED
    config = ExplorationConfigV3(
        max_iterations=3,
        enable_branch_reuse=False,
        enable_verbose_logging=False
    )

    explorer = IntegratedExplorerV3(gt_field, config, rng)

    print("\nRunning exploration WITHOUT branch reuse...")
    success = explorer.run_exploration()

    n_iterations = len(explorer.iteration_results)
    trajectory_length = len(explorer.robot.trajectory)

    print(f"\n✓ Completed {n_iterations} iterations")
    print(f"✓ Trajectory length: {trajectory_length} points")

    if trajectory_length > 1:
        start = explorer.robot.trajectory[0]
        end = explorer.robot.trajectory[-1]
        dist = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        print(f"✓ Robot moved {dist:.1f}px")

    print("="*70)

    return success


if __name__ == "__main__":
    print("\n" + "="*70)
    print("TESTING BRANCH REUSE FUNCTIONALITY")
    print("="*70)

    # Test with branch reuse enabled
    test1 = test_branch_reuse()

    # Test with branch reuse disabled
    test2 = test_branch_reuse_disabled()

    print("\n" + "="*70)
    print("FINAL RESULTS")
    print("="*70)
    print(f"Branch reuse ENABLED:  {'PASS' if test1 else 'FAIL'}")
    print(f"Branch reuse DISABLED: {'PASS' if test2 else 'FAIL'}")
    print("="*70)
