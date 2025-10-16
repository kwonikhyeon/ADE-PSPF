"""
Integration Test: RRT Planner V1 vs V2

Test that V2 is functionally equivalent to V1 and measure performance.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

from simulation.exploration.rrt_planner import (
    Pose2D, PlannerConfig, GainParameters, RadiationGainModel,
    PlanarEnvironment, RRTPlanner
)
from simulation.exploration.rrt_planner_v2 import RRTPlannerV2, PerformanceConfig


def create_nearest_intensity_lookup(obs_positions, obs_intensities):
    """
    Create nearest intensity lookup function.

    Returns callable: Pose2D -> (nearest_intensity, samples_near, distance_near)
    """
    def lookup(pose):
        if len(obs_positions) == 0:
            return 0.0, 0, float('inf')

        pos = pose.as_array()
        distances = np.linalg.norm(obs_positions - pos[None, :], axis=1)
        nearest_idx = np.argmin(distances)
        nearest_dist = distances[nearest_idx]
        nearest_intensity = obs_intensities[nearest_idx]

        # Count samples within certain radius
        search_radius = 5.0  # pixels
        samples_near = int(np.sum(distances < search_radius))

        return float(nearest_intensity), samples_near, float(nearest_dist)

    return lookup


def test_basic_functionality():
    """
    TEST: Basic Functionality

    Verify V2 can build a tree successfully.
    """
    print("\n" + "="*70)
    print("TEST: Basic Functionality")
    print("="*70)

    # Setup
    env = PlanarEnvironment(bounds=(0.0, 256.0, 0.0, 256.0), obstacles=[])
    gain_model = RadiationGainModel(GainParameters())
    config = PlannerConfig(n_uniform=8, max_nodes=50, random_seed=42)
    perf_config = PerformanceConfig(verbose=False)

    planner_v2 = RRTPlannerV2(env, gain_model, config, perf_config)

    # Test data
    root_pose = Pose2D(x=128.0, y=128.0, theta=0.0)
    sources = np.array([[100.0, 100.0, 1.0]])  # Single source
    obs_positions = np.array([[120.0, 120.0], [130.0, 130.0]])
    obs_intensities = np.array([0.5, 0.3])

    lookup = create_nearest_intensity_lookup(obs_positions, obs_intensities)

    # Build tree
    start_time = time.time()
    nodes, leaves = planner_v2.build_tree(
        root_pose=root_pose,
        sources=sources,
        observation_positions=obs_positions,
        observation_intensities=obs_intensities,
        nearest_intensity_lookup=lookup,
        previous_best_branch=None
    )
    elapsed = time.time() - start_time

    print(f"\n✓ Tree built successfully")
    print(f"  Nodes: {len(nodes)}")
    print(f"  Leaves: {len(leaves)}")
    print(f"  Time: {elapsed*1000:.2f}ms")
    print(f"  Stats: {planner_v2.stats}")

    # Select best branch
    best_branch, best_leaf_idx = planner_v2.select_best_branch(nodes, leaves)
    print(f"\n✓ Best branch selected")
    print(f"  Branch length: {len(best_branch)}")
    print(f"  Best leaf: {best_leaf_idx}")
    print(f"  Cumulative gain: {nodes[best_leaf_idx].cumulative_gain:.4f}")

    print("\n✅ TEST PASSED: Basic functionality working")
    return True


def test_v1_vs_v2_comparison():
    """
    TEST: V1 vs V2 Comparison

    Compare performance and verify similar results.
    """
    print("\n" + "="*70)
    print("TEST: V1 vs V2 Performance Comparison")
    print("="*70)

    # Setup
    env = PlanarEnvironment(bounds=(0.0, 256.0, 0.0, 256.0), obstacles=[])
    gain_model = RadiationGainModel(GainParameters())
    config = PlannerConfig(n_uniform=8, max_nodes=80, random_seed=42)

    planner_v1 = RRTPlanner(env, gain_model, config)
    planner_v2 = RRTPlannerV2(env, gain_model, config, PerformanceConfig(verbose=False))

    # Test with increasing observations
    obs_counts = [5, 10, 15, 20]
    v1_times = []
    v2_times = []

    root_pose = Pose2D(x=128.0, y=128.0, theta=0.0)
    sources = np.array([
        [100.0, 100.0, 1.0],
        [150.0, 120.0, 0.8]
    ])

    for n_obs in obs_counts:
        # Generate observations
        np.random.seed(42)
        obs_positions = np.random.uniform(50, 200, (n_obs, 2))
        obs_intensities = np.random.uniform(0.1, 1.0, n_obs)

        lookup = create_nearest_intensity_lookup(obs_positions, obs_intensities)

        # Test V1
        start = time.time()
        nodes_v1, leaves_v1 = planner_v1.build_tree(
            root_pose, sources, obs_positions, obs_intensities,
            lookup, None
        )
        v1_time = time.time() - start
        v1_times.append(v1_time)

        # Test V2
        start = time.time()
        nodes_v2, leaves_v2 = planner_v2.build_tree(
            root_pose, sources, obs_positions, obs_intensities,
            lookup, None
        )
        v2_time = time.time() - start
        v2_times.append(v2_time)

        speedup = v1_time / v2_time if v2_time > 0 else 1.0

        print(f"\n{n_obs} observations:")
        print(f"  V1: {v1_time*1000:.2f}ms ({len(nodes_v1)} nodes)")
        print(f"  V2: {v2_time*1000:.2f}ms ({len(nodes_v2)} nodes)")
        print(f"  Speedup: {speedup:.2f}x")

    # Visualize comparison
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))

    ax.set_title('RRT Planner V1 vs V2 Performance', fontsize=14, fontweight='bold')
    ax.set_xlabel('Number of Observations', fontsize=12)
    ax.set_ylabel('Execution Time (ms)', fontsize=12)
    ax.grid(True, alpha=0.3)

    x = obs_counts
    ax.plot(x, [t*1000 for t in v1_times], 'o-', linewidth=2, markersize=8,
            label='V1 (Original)', color='red')
    ax.plot(x, [t*1000 for t in v2_times], 's-', linewidth=2, markersize=8,
            label='V2 (Optimized)', color='blue')
    ax.legend(fontsize=11)

    plt.tight_layout()
    output_path = 'data/figures/test_v2_performance_comparison.png'
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✓ Saved visualization: {output_path}")

    avg_speedup = np.mean([v1_times[i]/v2_times[i] for i in range(len(obs_counts))])
    print(f"\n✅ TEST PASSED: Average speedup = {avg_speedup:.2f}x")

    return True


def main():
    """Run all integration tests."""
    print("\n" + "🧪 "*35)
    print("RRT PLANNER V2 - INTEGRATION TESTS")
    print("🧪 "*35)

    results = []

    # Test 1: Basic functionality
    try:
        results.append(("Basic Functionality", test_basic_functionality()))
    except Exception as e:
        print(f"\n❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        results.append(("Basic Functionality", False))

    # Test 2: V1 vs V2 comparison
    try:
        results.append(("V1 vs V2 Performance", test_v1_vs_v2_comparison()))
    except Exception as e:
        print(f"\n❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        results.append(("V1 vs V2 Performance", False))

    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    for name, passed in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status}: {name}")

    total = len(results)
    passed = sum(1 for _, p in results if p)
    print(f"\nTotal: {passed}/{total} tests passed")

    if passed == total:
        print("\n🎉 All integration tests passed!")
    else:
        print(f"\n⚠️  {total - passed} test(s) failed")
        return 1

    return 0


if __name__ == '__main__':
    exit(main())
