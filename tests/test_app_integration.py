"""
Integration test for the visualization app.
Verify that robot movement and goal changes work correctly.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from visualization.explorer_controller import ExplorerController
from simulation.integrated_explorer_v2 import ExplorationConfig


def test_app_integration():
    """Test complete app workflow."""
    print("\n" + "="*70)
    print("APP INTEGRATION TEST")
    print("="*70)

    # Configuration
    config = ExplorationConfig(
        max_iterations=5,
        use_rrt_v2=True,
        enable_timing_logs=False,
        log_iteration_details=False
    )

    # Setup controller
    controller = ExplorerController()
    controller.config = config

    # Generate ground truth
    print("\n1. Generating ground truth...")
    success = controller.generate_ground_truth(n_sources=3, seed=456)
    assert success, "Ground truth generation failed"
    print("   ✓ Ground truth generated")

    # Initialize explorer
    print("\n2. Initializing explorer...")
    success = controller.initialize_explorer(max_iterations=5, enable_logs=False)
    assert success, "Explorer initialization failed"
    print("   ✓ Explorer initialized")

    # Get initial state
    initial_pose = controller.explorer.robot.pose
    print(f"\n3. Initial robot position: ({initial_pose.x:.2f}, {initial_pose.y:.2f})")

    # Run exploration
    print("\n4. Running exploration...")
    success = controller.run_exploration_with_snapshots()
    assert success, "Exploration failed"
    print("   ✓ Exploration completed")

    # Get snapshot
    snapshot = controller.get_snapshot()
    print(f"\n5. Snapshot created with {len(snapshot.iteration_data)} iterations")

    # Verify robot movement
    print("\n" + "="*70)
    print("VERIFICATION: Robot Movement")
    print("="*70)

    trajectory = snapshot.full_trajectory
    print(f"\nTrajectory points: {len(trajectory)}")

    if len(trajectory) < 2:
        print("❌ FAIL: Robot didn't move (only 1 trajectory point)")
        return False

    total_distance = 0.0
    for i in range(len(trajectory) - 1):
        dx = trajectory[i+1][0] - trajectory[i][0]
        dy = trajectory[i+1][1] - trajectory[i][1]
        dist = np.sqrt(dx**2 + dy**2)
        total_distance += dist
        print(f"  Step {i}: ({trajectory[i][0]:.1f}, {trajectory[i][1]:.1f}) → "
              f"({trajectory[i+1][0]:.1f}, {trajectory[i+1][1]:.1f}), "
              f"dist={dist:.1f}px")

    print(f"\nTotal distance: {total_distance:.2f} pixels ({total_distance*0.04:.3f} m)")

    if total_distance < 10.0:
        print("❌ FAIL: Robot barely moved")
        return False

    print("✅ PASS: Robot moved correctly")

    # Verify goal changes
    print("\n" + "="*70)
    print("VERIFICATION: Goal Changes")
    print("="*70)

    goals = []
    for i, iter_data in enumerate(snapshot.iteration_data):
        if iter_data.exploration_data and len(iter_data.exploration_data.best_branch) > 0:
            goal_node = iter_data.exploration_data.best_branch[-1]
            goal_pos = (goal_node.pose.x, goal_node.pose.y)
            goals.append(goal_pos)
            print(f"  Iteration {i}: Goal at ({goal_pos[0]:.1f}, {goal_pos[1]:.1f})")

    if len(goals) < 2:
        print("❌ FAIL: Not enough goals to compare")
        return False

    # Check if goals are changing
    same_count = 0
    for i in range(len(goals) - 1):
        dx = goals[i+1][0] - goals[i][0]
        dy = goals[i+1][1] - goals[i][1]
        dist = np.sqrt(dx**2 + dy**2)
        if dist < 1.0:
            same_count += 1
            print(f"  ⚠ Goal {i} → {i+1}: Almost no change ({dist:.2f}px)")

    if same_count > len(goals) * 0.5:
        print(f"❌ FAIL: Too many goals are the same ({same_count}/{len(goals)-1})")
        return False

    print("✅ PASS: Goals are changing")

    # Verify observations
    print("\n" + "="*70)
    print("VERIFICATION: Observations")
    print("="*70)

    observations = snapshot.all_observations
    print(f"\nTotal observations: {len(observations)}")

    if len(observations) < config.max_iterations:
        print(f"⚠ WARNING: Expected at least {config.max_iterations} observations")

    print("✅ PASS: Observations recorded")

    # Final summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print("✅ All checks passed!")
    print(f"   - Robot moved {total_distance:.1f} pixels")
    print(f"   - {len(goals)} goals generated")
    print(f"   - {len(observations)} observations made")
    print(f"   - {len(trajectory)} trajectory points")

    return True


if __name__ == '__main__':
    success = test_app_integration()
    exit(0 if success else 1)
