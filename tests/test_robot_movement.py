"""
Debug script to check if robot is actually moving during exploration.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from visualization.explorer_controller import ExplorerController
from simulation.integrated_explorer_v2 import ExplorationConfig
from environment.generate_truth import sample_sources, inverse_square_field


def test_robot_movement():
    """Test if robot actually moves during exploration."""
    print("\n" + "="*70)
    print("ROBOT MOVEMENT DEBUG TEST")
    print("="*70)

    # Configuration
    config = ExplorationConfig(
        max_iterations=5,
        use_rrt_v2=True,
        enable_timing_logs=True,
        log_iteration_details=True
    )

    # Run exploration
    controller = ExplorerController()
    controller.config = config
    controller.generate_ground_truth(n_sources=3, seed=42)
    controller.initialize_explorer(max_iterations=5, enable_logs=True)

    # Get initial position
    initial_pose = controller.explorer.robot.pose
    print(f"\nInitial robot position: ({initial_pose.x:.2f}, {initial_pose.y:.2f})")

    # Run exploration
    print("\nRunning exploration...")
    success = controller.run_exploration_with_snapshots()

    if not success:
        print("❌ Exploration failed")
        return

    # Get snapshot
    snapshot = controller.get_snapshot()

    # Analyze trajectory
    print("\n" + "="*70)
    print("TRAJECTORY ANALYSIS")
    print("="*70)

    trajectory = snapshot.full_trajectory
    print(f"\nTotal trajectory points: {len(trajectory)}")

    if len(trajectory) == 0:
        print("❌ ERROR: No trajectory recorded!")
        return

    print(f"\nTrajectory:")
    for i, (x, y) in enumerate(trajectory):
        print(f"  Point {i}: ({x:.2f}, {y:.2f})")

    # Calculate distances
    if len(trajectory) > 1:
        print(f"\nMovement distances:")
        total_distance = 0.0
        for i in range(len(trajectory) - 1):
            dx = trajectory[i+1][0] - trajectory[i][0]
            dy = trajectory[i+1][1] - trajectory[i][1]
            dist = np.sqrt(dx**2 + dy**2)
            total_distance += dist
            print(f"  Step {i} → {i+1}: {dist:.2f} pixels ({dist*0.04:.3f} m)")

        print(f"\nTotal distance traveled: {total_distance:.2f} pixels ({total_distance*0.04:.3f} m)")

        if total_distance < 1.0:
            print("❌ ERROR: Robot barely moved!")
        else:
            print("✅ Robot is moving correctly")
    else:
        print("❌ ERROR: Only one trajectory point (robot didn't move)")

    # Check observations
    print("\n" + "="*70)
    print("OBSERVATION ANALYSIS")
    print("="*70)

    observations = snapshot.all_observations
    print(f"\nTotal observations: {len(observations)}")

    if len(observations) > 0:
        print(f"\nObservation positions:")
        for i, (pos, intensity) in enumerate(observations[:10]):  # First 10
            print(f"  Obs {i}: pos=({pos[0]:.2f}, {pos[1]:.2f}), intensity={intensity:.4f}")
        if len(observations) > 10:
            print(f"  ... and {len(observations) - 10} more")

    # Check iteration data
    print("\n" + "="*70)
    print("PER-ITERATION ANALYSIS")
    print("="*70)

    for i, iter_data in enumerate(snapshot.iteration_data):
        print(f"\nIteration {i}:")

        if iter_data.exploration_data:
            best_branch = iter_data.exploration_data.best_branch
            print(f"  Best branch length: {len(best_branch)}")
            if len(best_branch) >= 2:
                first_node = best_branch[0]
                second_node = best_branch[1]
                dx = second_node.pose.x - first_node.pose.x
                dy = second_node.pose.y - first_node.pose.y
                dist = np.sqrt(dx**2 + dy**2)
                print(f"  First edge: ({first_node.pose.x:.2f}, {first_node.pose.y:.2f}) "
                      f"→ ({second_node.pose.x:.2f}, {second_node.pose.y:.2f})")
                print(f"  First edge distance: {dist:.2f} pixels ({dist*0.04:.3f} m)")

        if iter_data.execution_data:
            exec_data = iter_data.execution_data
            print(f"  Actual distance moved: {exec_data.actual_distance:.2f} pixels ({exec_data.actual_distance*0.04:.3f} m)")
            print(f"  Observations made: {exec_data.observations_made}")

    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    print(f"✓ Total iterations: {len(snapshot.iteration_data)}")
    print(f"✓ Total trajectory points: {len(trajectory)}")
    print(f"✓ Total observations: {len(observations)}")
    if len(trajectory) > 1:
        print(f"✓ Total distance: {total_distance:.2f} pixels ({total_distance*0.04:.3f} m)")

    return success


if __name__ == '__main__':
    test_robot_movement()
