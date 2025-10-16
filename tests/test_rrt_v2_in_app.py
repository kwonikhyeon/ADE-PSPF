"""
Test RRT V2 integration in the actual visualization app.

Compare V1 vs V2 performance in realistic scenario using explorer_controller.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import time
from visualization.explorer_controller import ExplorerController
from simulation.integrated_explorer_v2 import ExplorationConfig
from environment.generate_truth import sample_sources, inverse_square_field


def test_v1_performance():
    """Test with original RRT V1."""
    print("\n" + "="*70)
    print("Testing with RRT Planner V1 (Original)")
    print("="*70)

    # Configuration with V1
    config = ExplorationConfig(
        max_iterations=5,  # Short test
        use_rrt_v2=False  # Use V1
    )

    # Generate ground truth
    rng = np.random.default_rng(42)
    coords, amps, sigmas = sample_sources(grid=256, n=3, rng=rng)
    field_gt = inverse_square_field(grid=256, coords=coords, amps=amps)

    # Convert to format expected by explorer (x, y, intensity)
    sources_gt = np.column_stack([coords[:, 1], coords[:, 0], amps])  # (y,x) -> (x,y)

    # Use ExplorerController
    controller = ExplorerController(config=config)
    controller.set_ground_truth(field_gt, sources_gt)
    controller.initialize_explorer(random_seed=42)

    start_time = time.time()
    success = controller.run_exploration_with_snapshots()
    total_time = time.time() - start_time

    if not success:
        raise RuntimeError("Exploration failed")

    print(f"\n✓ Exploration completed")
    snapshot = controller.get_snapshot()
    print(f"  Total iterations: {len(snapshot.iteration_data)}")
    print(f"  Total time: {total_time:.2f}s")
    print(f"  Average per iteration: {total_time/len(snapshot.iteration_data):.2f}s")

    # Extract timing per iteration
    iteration_times = []
    for iter_data in snapshot.iteration_data:
        # Calculate iteration time from step times
        iter_time = 0.0
        if iter_data.estimation_data:
            iter_time += iter_data.estimation_data.time_elapsed
        if iter_data.exploration_data:
            iter_time += iter_data.exploration_data.time_elapsed
        if iter_data.execution_data:
            iter_time += iter_data.execution_data.time_elapsed
        iteration_times.append(iter_time)
        print(f"  Iteration {iter_data.iteration + 1}: {iter_time:.3f}s")

    return iteration_times, total_time


def test_v2_performance():
    """Test with optimized RRT V2."""
    print("\n" + "="*70)
    print("Testing with RRT Planner V2 (Optimized)")
    print("="*70)

    # Configuration with V2
    config = ExplorationConfig(
        max_iterations=5,  # Short test
        use_rrt_v2=True  # Use V2
    )

    # Generate ground truth (same as V1)
    rng = np.random.default_rng(42)
    coords, amps, sigmas = sample_sources(grid=256, n=3, rng=rng)
    field_gt = inverse_square_field(grid=256, coords=coords, amps=amps)

    # Convert to format expected by explorer (x, y, intensity)
    sources_gt = np.column_stack([coords[:, 1], coords[:, 0], amps])  # (y,x) -> (x,y)

    # Use ExplorerController
    controller = ExplorerController(config=config)
    controller.set_ground_truth(field_gt, sources_gt)
    controller.initialize_explorer(random_seed=42)

    start_time = time.time()
    success = controller.run_exploration_with_snapshots()
    total_time = time.time() - start_time

    if not success:
        raise RuntimeError("Exploration failed")

    print(f"\n✓ Exploration completed")
    snapshot = controller.get_snapshot()
    print(f"  Total iterations: {len(snapshot.iteration_data)}")
    print(f"  Total time: {total_time:.2f}s")
    print(f"  Average per iteration: {total_time/len(snapshot.iteration_data):.2f}s")

    # Extract timing per iteration
    iteration_times = []
    for iter_data in snapshot.iteration_data:
        # Calculate iteration time from step times
        iter_time = 0.0
        if iter_data.estimation_data:
            iter_time += iter_data.estimation_data.time_elapsed
        if iter_data.exploration_data:
            iter_time += iter_data.exploration_data.time_elapsed
        if iter_data.execution_data:
            iter_time += iter_data.execution_data.time_elapsed
        iteration_times.append(iter_time)
        print(f"  Iteration {iter_data.iteration + 1}: {iter_time:.3f}s")

    return iteration_times, total_time


def main():
    """Run performance comparison."""
    print("\n" + "🧪 "*35)
    print("RRT V1 vs V2 - REAL-WORLD PERFORMANCE TEST")
    print("🧪 "*35)

    # Test V1
    try:
        v1_times, v1_total = test_v1_performance()
    except Exception as e:
        print(f"\n❌ V1 test failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Test V2
    try:
        v2_times, v2_total = test_v2_performance()
    except Exception as e:
        print(f"\n❌ V2 test failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Compare results
    print("\n" + "="*70)
    print("COMPARISON SUMMARY")
    print("="*70)

    print(f"\nTotal Time:")
    print(f"  V1: {v1_total:.2f}s")
    print(f"  V2: {v2_total:.2f}s")
    print(f"  Speedup: {v1_total/v2_total:.2f}x")

    print(f"\nPer-Iteration Comparison:")
    for i in range(min(len(v1_times), len(v2_times))):
        speedup = v1_times[i] / v2_times[i] if v2_times[i] > 0 else 1.0
        print(f"  Iteration {i+1}: V1={v1_times[i]:.3f}s, V2={v2_times[i]:.3f}s, "
              f"Speedup={speedup:.2f}x")

    avg_speedup = np.mean([v1_times[i]/v2_times[i] for i in range(len(v1_times))])
    print(f"\n✅ Average speedup: {avg_speedup:.2f}x")

    if avg_speedup > 1.0:
        print(f"🎉 V2 is faster!")
    elif avg_speedup < 1.0:
        print(f"⚠️  V2 is slower (may need investigation)")
    else:
        print(f"➡️  V1 and V2 have similar performance")

    return 0


if __name__ == '__main__':
    exit(main())
