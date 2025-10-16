"""
Test Integrated Explorer V3

Verify that V3 solves the robot movement and goal change issues.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from simulation.integrated_explorer_v3 import IntegratedExplorerV3, ExplorationConfigV3
from environment.generate_truth import sample_sources, inverse_square_field


def test_v3_basic():
    """Test V3 basic functionality."""
    print("\n" + "="*70)
    print("EXPLORER V3 - BASIC TEST")
    print("="*70)

    # Generate ground truth
    rng = np.random.default_rng(789)
    coords, amps, sigmas = sample_sources(grid=256, n=3, rng=rng)
    field_gt = inverse_square_field(grid=256, coords=coords, amps=amps)

    # Configuration with verbose logging
    config = ExplorationConfigV3(
        max_iterations=5,
        use_rrt_v2=True,
        enable_verbose_logging=True,
        enable_state_verification=True,
        enable_movement_verification=True
    )

    # Create explorer
    explorer = IntegratedExplorerV3(gt_field=field_gt, config=config, rng=rng)

    # Run exploration
    print("\nRunning exploration...")
    success = explorer.run_exploration()

    # Verify results
    print("\n" + "="*70)
    print("VERIFICATION")
    print("="*70)

    # Check iterations
    n_iterations = len(explorer.iteration_results)
    print(f"\nTotal iterations: {n_iterations}")
    assert n_iterations > 0, "No iterations completed!"

    # Check robot movement
    print("\nRobot Movement:")
    trajectory = explorer.robot.trajectory
    print(f"  Trajectory points: {len(trajectory)}")

    total_distance = 0.0
    for i in range(len(trajectory) - 1):
        dx = trajectory[i+1][0] - trajectory[i][0]
        dy = trajectory[i+1][1] - trajectory[i][1]
        dist = np.sqrt(dx**2 + dy**2)
        total_distance += dist
        print(f"  Step {i}: ({trajectory[i][0]:.1f}, {trajectory[i][1]:.1f}) → "
              f"({trajectory[i+1][0]:.1f}, {trajectory[i+1][1]:.1f}), dist={dist:.1f}px")

    print(f"\nTotal distance: {total_distance:.1f}px ({total_distance*0.04:.3f}m)")

    assert total_distance > 10.0, f"Robot barely moved! Only {total_distance:.1f}px"

    # Check goal changes
    print("\nGoal Changes:")
    goals = []
    for i, result in enumerate(explorer.iteration_results):
        if result.planning_result and result.planning_result.goal_pose:
            goal = result.planning_result.goal_pose
            goals.append((goal.x, goal.y))
            print(f"  Iteration {i}: Goal at ({goal.x:.1f}, {goal.y:.1f}), "
                  f"Changed: {result.goal_changed}")

    # Check if goals changed
    if len(goals) > 1:
        same_count = 0
        for i in range(len(goals) - 1):
            dx = goals[i+1][0] - goals[i][0]
            dy = goals[i+1][1] - goals[i][1]
            dist = np.sqrt(dx**2 + dy**2)
            if dist < 1.0:
                same_count += 1

        print(f"\nGoals that didn't change: {same_count}/{len(goals)-1}")

        # Allow some goals to be same by chance, but not all
        assert same_count < len(goals) - 1, "Goals never changed!"

    # Check observations
    print("\nObservations:")
    obs_count = len(explorer.robot.observations)
    print(f"  Total observations: {obs_count}")
    assert obs_count >= n_iterations, f"Not enough observations! {obs_count} < {n_iterations}"

    # Summary
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    print(f"✅ Robot moved {total_distance:.1f}px over {n_iterations} iterations")
    print(f"✅ {len(goals)} goals generated")
    print(f"✅ {obs_count} observations made")
    print(f"✅ {len(trajectory)} trajectory points")
    print("="*70)

    print("\n✅ ALL TESTS PASSED!")

    return True


if __name__ == '__main__':
    success = test_v3_basic()
    exit(0 if success else 1)
