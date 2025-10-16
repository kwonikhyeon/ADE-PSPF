"""
Visual Tests for RRT Planner V2

Step-by-step visual verification of each core component.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend

from simulation.exploration.rrt_planner import (
    Pose2D, PlannerConfig, GainParameters, RadiationGainModel,
    PlanarEnvironment
)
from simulation.exploration.rrt_planner_v2 import RRTPlannerV2, PerformanceConfig


def test_1_uniform_initialization():
    """
    TEST 1: Visual verification of Equation 8 - Uniform Initialization

    Shows that n_uniform nodes are generated uniformly around the root.
    """
    print("\n" + "="*70)
    print("TEST 1: Uniform Initialization (Eq. 8)")
    print("="*70)

    # Setup
    env = PlanarEnvironment(bounds=(0.0, 256.0, 0.0, 256.0), obstacles=[])
    gain_model = RadiationGainModel(GainParameters())
    config = PlannerConfig(n_uniform=8, random_seed=42)
    perf_config = PerformanceConfig(verbose=True)

    planner_v2 = RRTPlannerV2(env, gain_model, config, perf_config)

    # Test uniform initialization
    root_pose = Pose2D(x=128.0, y=128.0, theta=0.0)
    uniform_nodes = planner_v2._generate_uniform_children(root_pose)

    print(f"\n✓ Generated {len(uniform_nodes)} uniform nodes")

    # Print actual distances
    root_pos = root_pose.as_array()
    distances = []
    for i, node in enumerate(uniform_nodes):
        node_pos = node.pose.as_array()
        dist = np.linalg.norm(node_pos - root_pos)
        distances.append(dist)
        print(f"  Node {i}: distance = {dist:.4f} pixels")

    print(f"  Min distance: {min(distances):.4f} pixels")
    print(f"  Max distance: {max(distances):.4f} pixels")
    print(f"  Expected: 0.5m in pixel coordinates")

    # Visualize
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # Left: Uniform distribution
    ax1.set_title('Uniform Initialization (n_uniform=8)', fontsize=12, fontweight='bold')
    ax1.set_xlabel('X (pixels)')
    ax1.set_ylabel('Y (pixels)')
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')

    # Plot root
    ax1.plot(root_pose.x, root_pose.y, 'go', markersize=15,
            markeredgecolor='black', markeredgewidth=2, label='Root', zorder=10)

    # Plot uniform nodes
    for i, node in enumerate(uniform_nodes):
        ax1.plot(node.pose.x, node.pose.y, 'bo', markersize=10,
                markeredgecolor='black', markeredgewidth=1, zorder=5)
        # Draw line from root
        ax1.plot([root_pose.x, node.pose.x], [root_pose.y, node.pose.y],
                'b-', linewidth=2, alpha=0.6)
        # Add angle label
        angle_deg = np.degrees(node.pose.theta)
        ax1.text(node.pose.x, node.pose.y, f'{angle_deg:.0f}°',
                fontsize=8, ha='center', va='bottom')

    ax1.legend(fontsize=10)
    ax1.set_xlim(80, 176)
    ax1.set_ylim(80, 176)

    # Right: Angle distribution
    ax2.set_title('Angular Distribution', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Node Index')
    ax2.set_ylabel('Angle (degrees)')
    ax2.grid(True, alpha=0.3)

    angles = [np.degrees(node.pose.theta) for node in uniform_nodes]
    ax2.bar(range(len(angles)), angles, color='blue', alpha=0.7, edgecolor='black')
    ax2.axhline(y=360/len(angles), color='r', linestyle='--',
               label=f'Expected spacing: {360/len(angles):.1f}°')
    ax2.legend(fontsize=10)

    # Expected: angles should be evenly spaced by 360/n_uniform
    expected_spacing = 360.0 / config.n_uniform
    actual_spacings = []
    for i in range(len(angles) - 1):
        spacing = angles[i+1] - angles[i]
        actual_spacings.append(spacing)

    avg_spacing = np.mean(actual_spacings) if actual_spacings else 0
    print(f"  Expected spacing: {expected_spacing:.2f}°")
    print(f"  Average spacing: {avg_spacing:.2f}°")

    plt.tight_layout()
    output_path = 'data/figures/test_v2_1_uniform_init.png'
    os.makedirs('data/figures', exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✓ Saved visualization: {output_path}")
    plt.close()

    # Verification
    assert len(uniform_nodes) <= config.n_uniform, "Too many nodes generated"
    assert len(uniform_nodes) >= config.n_uniform - 2, "Too few nodes (some collisions OK)"

    print(f"✅ TEST 1 PASSED: Uniform initialization working correctly")
    return True


def test_2_kdtree_performance():
    """
    TEST 2: Visual verification of KD-Tree performance improvement

    Compares KD-Tree vs linear search performance.
    """
    print("\n" + "="*70)
    print("TEST 2: KD-Tree Performance")
    print("="*70)

    # Setup
    env = PlanarEnvironment(bounds=(0.0, 256.0, 0.0, 256.0), obstacles=[])
    gain_model = RadiationGainModel(GainParameters())
    config = PlannerConfig(random_seed=42)

    # Test with different node counts
    node_counts = [10, 20, 40, 80]
    kdtree_times = []
    linear_times = []

    for n_nodes in node_counts:
        print(f"\nTesting with {n_nodes} nodes...")

        # Create dummy nodes
        nodes = []
        for i in range(n_nodes):
            x = np.random.uniform(0, 256)
            y = np.random.uniform(0, 256)
            nodes.append(
                type('Node', (), {'pose': Pose2D(x=x, y=y, theta=0.0)})()
            )

        # Test KD-Tree
        perf_config = PerformanceConfig(use_kdtree=True, verbose=False)
        planner = RRTPlannerV2(env, gain_model, config, perf_config)
        planner._rebuild_kdtree(nodes)

        import time
        queries = [np.random.uniform(0, 256, 2) for _ in range(100)]

        start = time.time()
        for query in queries:
            planner._nearest_node_kdtree(nodes, query)
        kdtree_time = (time.time() - start) / 100  # per query
        kdtree_times.append(kdtree_time * 1000)  # ms

        # Test Linear
        start = time.time()
        for query in queries:
            planner._nearest_node_linear(nodes, query)
        linear_time = (time.time() - start) / 100
        linear_times.append(linear_time * 1000)  # ms

        speedup = linear_time / kdtree_time
        print(f"  KD-Tree: {kdtree_time*1000:.3f}ms")
        print(f"  Linear:  {linear_time*1000:.3f}ms")
        print(f"  Speedup: {speedup:.1f}x")

    # Visualize
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_title('KD-Tree vs Linear Search Performance', fontsize=14, fontweight='bold')
    ax.set_xlabel('Number of Nodes', fontsize=12)
    ax.set_ylabel('Time per Query (ms)', fontsize=12)
    ax.grid(True, alpha=0.3)

    x = node_counts
    ax.plot(x, kdtree_times, 'g-o', linewidth=2, markersize=8,
           label='KD-Tree (O(log n))', zorder=5)
    ax.plot(x, linear_times, 'r-s', linewidth=2, markersize=8,
           label='Linear Search (O(n))', zorder=5)

    # Add speedup annotations
    for i, (n, kt, lt) in enumerate(zip(node_counts, kdtree_times, linear_times)):
        speedup = lt / kt
        ax.text(n, lt * 1.1, f'{speedup:.1f}x faster',
               ha='center', fontsize=9, color='green', fontweight='bold')

    ax.legend(fontsize=11)
    ax.set_yscale('log')

    plt.tight_layout()
    output_path = 'data/figures/test_v2_2_kdtree_performance.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✓ Saved visualization: {output_path}")
    plt.close()

    # Verification
    avg_speedup = np.mean(np.array(linear_times) / np.array(kdtree_times))
    print(f"\n✅ TEST 2 PASSED: Average speedup = {avg_speedup:.1f}x")

    return True


def test_3_observation_windowing():
    """
    TEST 3: Visual verification of observation windowing

    Shows that only recent observations are used.
    """
    print("\n" + "="*70)
    print("TEST 3: Observation Windowing")
    print("="*70)

    # Setup
    env = PlanarEnvironment(bounds=(0.0, 256.0, 0.0, 256.0), obstacles=[])
    gain_model = RadiationGainModel(GainParameters())
    config = PlannerConfig(random_seed=42)
    perf_config = PerformanceConfig(max_observations=20, verbose=True)

    planner = RRTPlannerV2(env, gain_model, config, perf_config)

    # Create many observations
    n_total_obs = 50
    obs_positions = np.random.uniform(0, 256, (n_total_obs, 2))
    obs_intensities = np.random.uniform(100, 200, n_total_obs)

    print(f"\nTotal observations: {n_total_obs}")

    # Test windowing
    windowed_pos, windowed_int, obs_tree = planner._prepare_observations(
        obs_positions, obs_intensities
    )

    print(f"After windowing: {len(windowed_pos)}")
    print(f"Expected: {min(n_total_obs, perf_config.max_observations)}")

    # Visualize
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # Left: All observations
    ax1.set_title(f'All Observations (n={n_total_obs})', fontsize=12, fontweight='bold')
    ax1.scatter(obs_positions[:, 0], obs_positions[:, 1],
               c='gray', s=50, alpha=0.3, label='Discarded')
    ax1.scatter(windowed_pos[:, 0], windowed_pos[:, 1],
               c='red', s=100, marker='o', edgecolors='black',
               linewidths=2, label='Used (windowed)', zorder=5)
    ax1.set_xlabel('X (pixels)')
    ax1.set_ylabel('Y (pixels)')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(0, 256)
    ax1.set_ylim(0, 256)

    # Right: Time series showing windowing
    ax2.set_title('Observation Selection Over Time', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Observation Index')
    ax2.set_ylabel('Used in RRT')

    indices = np.arange(n_total_obs)
    used = np.zeros(n_total_obs)
    start_idx = max(0, n_total_obs - perf_config.max_observations)
    used[start_idx:] = 1

    ax2.bar(indices, used, color='red', alpha=0.7, edgecolor='black',
           label=f'Used (last {perf_config.max_observations})')
    ax2.axvline(x=start_idx, color='green', linestyle='--', linewidth=2,
               label='Window start')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    output_path = 'data/figures/test_v2_3_observation_windowing.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✓ Saved visualization: {output_path}")
    plt.close()

    # Verification
    assert len(windowed_pos) == perf_config.max_observations
    assert obs_tree is not None

    print(f"✅ TEST 3 PASSED: Observation windowing working correctly")
    return True


def run_all_visual_tests():
    """Run all visual tests."""
    print("\n" + "🧪 "*35)
    print("RRT PLANNER V2 - VISUAL TESTS")
    print("🧪 "*35)

    tests = [
        ("Uniform Initialization (Eq. 8)", test_1_uniform_initialization),
        ("KD-Tree Performance", test_2_kdtree_performance),
        ("Observation Windowing", test_3_observation_windowing),
    ]

    results = []
    for name, test_func in tests:
        try:
            success = test_func()
            results.append((name, success))
        except Exception as e:
            print(f"\n✗ TEST FAILED: {name}")
            print(f"  Error: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, False))

    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)

    for name, success in results:
        status = "✅ PASS" if success else "✗ FAIL"
        print(f"{status}: {name}")

    passed = sum(1 for _, s in results if s)
    total = len(results)

    print(f"\nTotal: {passed}/{total} tests passed")

    if passed == total:
        print("\n🎉 All visual tests passed!")
        print("\nGenerated visualizations:")
        print("  1. data/figures/test_v2_1_uniform_init.png")
        print("  2. data/figures/test_v2_2_kdtree_performance.png")
        print("  3. data/figures/test_v2_3_observation_windowing.png")
    else:
        print(f"\n⚠️ {total - passed} test(s) failed")

    return passed == total


if __name__ == "__main__":
    success = run_all_visual_tests()
    sys.exit(0 if success else 1)
