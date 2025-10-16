"""
Diagnose performance issues in RRT Planning step
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time
import numpy as np
from visualization.explorer_controller import ExplorerController

def diagnose_performance():
    """Diagnose where the time is being spent."""
    print("\n" + "="*70)
    print("PERFORMANCE DIAGNOSIS")
    print("="*70)

    # Create controller
    controller = ExplorerController()

    # Generate GT
    print("\n[1/4] Generating ground truth...")
    start = time.time()
    controller.generate_ground_truth(n_sources=3, seed=42)
    print(f"  Time: {time.time() - start:.3f}s")

    # Initialize explorer
    print("\n[2/4] Initializing explorer...")
    start = time.time()
    controller.initialize_explorer(
        max_iterations=5,
        n_swarms=4,
        n_particles=80,
        ade_generations=3,
        enable_logs=True  # Enable to see timing
    )
    print(f"  Time: {time.time() - start:.3f}s")

    # Make initial observation
    controller.explorer._make_observation(
        controller.explorer.robot.pose.x,
        controller.explorer.robot.pose.y
    )

    # Run ONE iteration manually with detailed timing
    print("\n[3/4] Running ONE iteration with detailed timing...")
    print("="*70)

    # STEP 1: Estimation
    print("\n>>> STEP 1: ESTIMATION")
    est_start = time.time()
    estimation_result = controller.explorer._estimation_step()
    est_time = time.time() - est_start
    print(f"  Total estimation time: {est_time:.3f}s")
    print(f"  RFC: {estimation_result['rfc']:.4f}")
    print(f"  Sources: {estimation_result['n_sources']}")

    # STEP 2: Exploration (RRT) - WITH DETAILED TIMING
    print("\n>>> STEP 2: EXPLORATION (RRT) - DETAILED")

    # Prepare data
    estimated_sources = estimation_result['sources']
    if len(estimated_sources) == 0:
        estimated_sources = [np.array([
            controller.explorer.rng.uniform(64, 192),
            controller.explorer.rng.uniform(64, 192),
            50.0
        ])]
    sources = np.array(estimated_sources)

    if len(controller.explorer.robot.observations) > 0:
        obs_positions = np.array([obs[0] for obs in controller.explorer.robot.observations])
        obs_intensities = np.array([obs[1] for obs in controller.explorer.robot.observations])
    else:
        obs_positions = np.array([]).reshape(0, 2)
        obs_intensities = np.array([])

    print(f"  Sources to plan for: {len(sources)}")
    print(f"  Observations: {len(controller.explorer.robot.observations)}")

    # Nearest intensity lookup function
    def nearest_intensity_lookup(pose):
        pos = pose.as_array()
        if len(controller.explorer.robot.observations) > 0:
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

    # Time RRT tree building
    print("\n  Building RRT tree...")
    rrt_start = time.time()

    nodes, leaves = controller.explorer.planner.build_tree(
        root_pose=controller.explorer.robot.pose,
        sources=sources,
        observation_positions=obs_positions,
        observation_intensities=obs_intensities,
        nearest_intensity_lookup=nearest_intensity_lookup,
        previous_best_branch=controller.explorer.previous_best_branch
    )

    rrt_time = time.time() - rrt_start
    print(f"  RRT tree building time: {rrt_time:.3f}s ⚠️")
    print(f"  Nodes generated: {len(nodes)}")
    print(f"  Leaf nodes: {len(leaves)}")

    # Time best branch selection
    print("\n  Selecting best branch...")
    select_start = time.time()
    if leaves:
        best_branch, best_leaf = controller.explorer.planner.select_best_branch(nodes, leaves)
        select_time = time.time() - select_start
        print(f"  Best branch selection time: {select_time:.3f}s")
        print(f"  Best branch length: {len(best_branch)}")
    else:
        select_time = 0
        print("  No leaves found!")

    total_exp_time = rrt_time + select_time
    print(f"\n  Total exploration time: {total_exp_time:.3f}s")

    # STEP 3: Execution
    print("\n>>> STEP 3: EXECUTION")
    exec_start = time.time()
    if leaves:
        observations_made = controller.explorer._execution_step(best_branch)
        exec_time = time.time() - exec_start
        print(f"  Execution time: {exec_time:.3f}s")
        print(f"  Observations made: {observations_made}")
    else:
        exec_time = 0

    # Summary
    print("\n" + "="*70)
    print("[4/4] TIMING SUMMARY")
    print("="*70)
    total_iter_time = est_time + total_exp_time + exec_time

    print(f"\nOne iteration breakdown:")
    print(f"  Estimation:  {est_time:6.3f}s ({est_time/total_iter_time*100:5.1f}%)")
    print(f"  Exploration: {total_exp_time:6.3f}s ({total_exp_time/total_iter_time*100:5.1f}%) ⚠️")
    print(f"    - RRT tree: {rrt_time:6.3f}s")
    print(f"    - Selection: {select_time:6.3f}s")
    print(f"  Execution:   {exec_time:6.3f}s ({exec_time/total_iter_time*100:5.1f}%)")
    print(f"  {'─'*30}")
    print(f"  TOTAL:       {total_iter_time:6.3f}s")

    print(f"\nEstimated time for 15 iterations: {total_iter_time * 15:.1f}s ({total_iter_time * 15 / 60:.1f} minutes)")

    # Identify bottleneck
    if total_exp_time > est_time and total_exp_time > exec_time:
        print(f"\n⚠️  BOTTLENECK: RRT Exploration is the slowest step!")
        print(f"    Taking {total_exp_time/total_iter_time*100:.1f}% of iteration time")

    # Check RRT configuration
    print("\n" + "="*70)
    print("RRT CONFIGURATION")
    print("="*70)
    planner_config = controller.explorer.planner.config
    print(f"  max_nodes:     {planner_config.max_nodes}")
    print(f"  n_uniform:     {planner_config.n_uniform}")
    print(f"  min_step:      {planner_config.min_step}")
    print(f"  max_step:      {planner_config.max_step}")

    # Recommendations
    print("\n" + "="*70)
    print("RECOMMENDATIONS")
    print("="*70)

    if rrt_time > 1.0:
        print("\n⚠️  RRT is very slow (>1s). Possible causes:")
        print("  1. Too many RRT nodes (max_nodes=80)")
        print("  2. Gain calculation is expensive")
        print("  3. Previous best branch reuse is slow")
        print("  4. Nearest neighbor search is slow")

        print("\n💡 Suggested fixes:")
        print("  1. Reduce max_nodes: 80 → 40-50")
        print("  2. Reduce n_uniform: 8 → 4-6")
        print("  3. Profile the RRT planner code")
        print("  4. Consider caching gain calculations")

    if est_time > 0.5:
        print("\n⚠️  Estimation is also slow (>0.5s)")
        print("  - Current particles: {0}".format(controller.config.adepspf_config.n_particles))
        print("  - Current swarms: {0}".format(controller.config.adepspf_config.n_swarms))
        print("  - ADE generations: {0}".format(controller.config.adepspf_config.ade_generations))

    print("\n" + "="*70)
    return True


if __name__ == "__main__":
    diagnose_performance()
