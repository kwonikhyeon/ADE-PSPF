"""
Test if RRT goals (best branch endpoints) change between iterations.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from visualization.explorer_controller import ExplorerController
from simulation.integrated_explorer_v2 import ExplorationConfig


def test_goal_changes():
    """Test if goals change between iterations."""
    print("\n" + "="*70)
    print("GOAL CHANGE ANALYSIS")
    print("="*70)

    # Configuration
    config = ExplorationConfig(
        max_iterations=3,  # Shorter test
        use_rrt_v2=True,
        enable_timing_logs=False,
        log_iteration_details=False  # Less output
    )

    # Run exploration
    controller = ExplorerController()
    controller.config = config
    controller.generate_ground_truth(n_sources=3, seed=123)  # Different seed
    controller.initialize_explorer(max_iterations=3, enable_logs=False)

    # Run exploration
    print("\nRunning exploration...")
    success = controller.run_exploration_with_snapshots()

    if not success:
        print("❌ Exploration failed")
        return

    # Get snapshot
    snapshot = controller.get_snapshot()

    # Analyze goals for each iteration
    print("\n" + "="*70)
    print("GOAL POSITION ANALYSIS")
    print("="*70)

    goals = []
    robot_positions = []
    estimated_sources_list = []

    for i, iter_data in enumerate(snapshot.iteration_data):
        print(f"\n{'='*70}")
        print(f"Iteration {i}")
        print(f"{'='*70}")

        # Robot position
        if i < len(snapshot.full_trajectory):
            robot_pos = snapshot.full_trajectory[i]
            robot_positions.append(robot_pos)
            print(f"Robot position: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")

        # Estimated sources
        if iter_data.estimation_data:
            est_sources = iter_data.estimation_data.estimated_sources
            estimated_sources_list.append(est_sources)
            print(f"Estimated sources: {len(est_sources)}")
            for j, src in enumerate(est_sources):
                print(f"  Source {j}: ({src[0]:.2f}, {src[1]:.2f}), intensity={src[2]:.2f}")

        # Goal (best branch endpoint)
        if iter_data.exploration_data:
            best_branch = iter_data.exploration_data.best_branch
            if len(best_branch) > 0:
                goal_node = best_branch[-1]
                goal_pos = (goal_node.pose.x, goal_node.pose.y)
                goals.append(goal_pos)
                print(f"Goal position: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})")
                print(f"Best branch length: {len(best_branch)}")
                print(f"Best gain: {iter_data.exploration_data.best_gain:.6f}")
            else:
                print("⚠ No best branch")

    # Calculate goal changes
    print("\n" + "="*70)
    print("GOAL MOVEMENT ANALYSIS")
    print("="*70)

    if len(goals) > 1:
        print(f"\nTotal iterations with goals: {len(goals)}")

        for i in range(len(goals) - 1):
            dx = goals[i+1][0] - goals[i][0]
            dy = goals[i+1][1] - goals[i][1]
            dist = np.sqrt(dx**2 + dy**2)
            print(f"\nGoal {i} → {i+1}:")
            print(f"  From: ({goals[i][0]:.2f}, {goals[i][1]:.2f})")
            print(f"  To:   ({goals[i+1][0]:.2f}, {goals[i+1][1]:.2f})")
            print(f"  Distance: {dist:.2f} pixels ({dist*0.04:.3f} m)")

        # Check if goals are static
        total_goal_movement = 0.0
        for i in range(len(goals) - 1):
            dx = goals[i+1][0] - goals[i][0]
            dy = goals[i+1][1] - goals[i][1]
            total_goal_movement += np.sqrt(dx**2 + dy**2)

        avg_goal_movement = total_goal_movement / (len(goals) - 1)

        print(f"\n{'='*70}")
        print(f"SUMMARY")
        print(f"{'='*70}")
        print(f"Total goal movement: {total_goal_movement:.2f} pixels ({total_goal_movement*0.04:.3f} m)")
        print(f"Average goal movement per iteration: {avg_goal_movement:.2f} pixels ({avg_goal_movement*0.04:.3f} m)")

        if avg_goal_movement < 5.0:
            print("\n❌ WARNING: Goals are barely changing!")
            print("   This suggests RRT is selecting similar endpoints each iteration.")
        else:
            print("\n✅ Goals are changing normally")

    # Check estimated sources changes
    print("\n" + "="*70)
    print("ESTIMATED SOURCES CHANGE ANALYSIS")
    print("="*70)

    if len(estimated_sources_list) > 1:
        for i in range(min(5, len(estimated_sources_list) - 1)):
            src_before = estimated_sources_list[i]
            src_after = estimated_sources_list[i+1]

            print(f"\nIteration {i} → {i+1}:")
            print(f"  Sources count: {len(src_before)} → {len(src_after)}")

            if len(src_before) > 0 and len(src_after) > 0:
                # Compare first source
                dx = src_after[0][0] - src_before[0][0]
                dy = src_after[0][1] - src_before[0][1]
                dist = np.sqrt(dx**2 + dy**2)
                print(f"  First source moved: {dist:.2f} pixels")

    return success


if __name__ == '__main__':
    test_goal_changes()
