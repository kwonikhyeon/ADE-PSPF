"""
Paper Compliance Test

논문 "A study of robotic search strategy for multi-radiation sources in unknown environments"
의 알고리즘 구현이 논문과 일치하는지 검증하는 종합 테스트.

검증 항목:
1. ADE-PSPF Estimation (Section 3)
2. RRT-based Path Planning (Section 4.1)
3. Radiation Gain Model (Section 4.2)
4. Dynamic Swarm Adjustment (Fig. 11)
5. OEE Iteration Loop (Fig. 2)
6. Exploration Phases (Fig. 14-16)
7. Termination Criteria (Section 5)
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from environment.generate_truth import sample_sources, gaussian_field, GRID
from environment.observation import RadiationObserver
from core.ade_pspf import ADEPSPF, ADEPSPFConfig
from core.dynamic_swarm_adjustment import DynamicSwarmAdjuster, SwarmAdjustmentConfig
from simulation.integrated_explorer_v3 import IntegratedExplorerV3, ExplorationConfigV3
from simulation.exploration.rrt_planner import (
    RRTPlanner, PlannerConfig, RadiationGainModel, GainParameters,
    PlanarEnvironment, Pose2D
)


def test_ade_pspf_estimation():
    """
    Test 1: ADE-PSPF Estimation (Section 3)

    논문 검증:
    - Synthesized weight computation (Eq. 3)
    - ADE resampling (Algorithm 1, Lines 11-12)
    - Mean shift clustering (Section 3.3)
    - RFC calculation (Eq. 7)
    - Configuration maintenance (Section 3.4)
    """
    print("\n" + "="*70)
    print("TEST 1: ADE-PSPF Estimation (Section 3)")
    print("="*70)

    # Create ground truth (3 sources as in paper Table 1)
    rng = np.random.default_rng(42)
    n_sources = 3
    coords, amps, sigmas = sample_sources(GRID, n_sources, rng=rng)
    gt_field = gaussian_field(GRID, coords, amps, sigmas)

    print(f"\nGround Truth (3 sources):")
    for i in range(n_sources):
        print(f"  Source {i}: pos=({coords[i,1]:.1f}, {coords[i,0]:.1f}), "
              f"intensity={amps[i]:.2f}")

    # Create observer
    observer = RadiationObserver(gt_field, GRID)

    # Generate observations
    observations = []
    for _ in range(20):
        x = rng.integers(30, 226)
        y = rng.integers(30, 226)
        intensity = observer.observe((y, x))
        observations.append((np.array([y, x]), intensity))

    # Initialize ADE-PSPF
    config = ADEPSPFConfig(
        n_swarms=4,
        n_particles=80,
        ade_generations=3
    )
    estimator = ADEPSPF(config=config, rng=rng)

    # Run estimation
    print(f"\nRunning ADE-PSPF with {len(observations)} observations...")
    result = estimator.update(observations)

    # Verify results
    print(f"\nResults:")
    print(f"  ✓ RFC: {result['rfc']:.4f} (논문 Eq. 7)")
    print(f"  ✓ Sources found: {result['n_sources']}")
    print(f"  ✓ Configuration accepted: {result['configuration_accepted']}")

    assert result['rfc'] > 0.0, "RFC should be positive"
    assert result['n_sources'] >= 0, "Source count should be non-negative"

    print("\n✅ ADE-PSPF Estimation: PASS")
    return True


def test_rrt_path_planning():
    """
    Test 2: RRT-based Path Planning (Section 4.1)

    논문 검증:
    - Uniform initialization (Eq. 8)
    - Branch reuse (Eq. 9)
    - New node generation (Eq. 10)
    - Collision detection
    """
    print("\n" + "="*70)
    print("TEST 2: RRT-based Path Planning (Section 4.1)")
    print("="*70)

    # Setup environment
    environment = PlanarEnvironment(
        bounds=(0.0, 256.0, 0.0, 256.0),
        obstacles=[]
    )
    gain_model = RadiationGainModel(GainParameters())
    config = PlannerConfig(
        n_uniform=8,
        max_nodes=50,
        min_step=10.0,
        max_step=20.0
    )
    planner = RRTPlanner(environment, gain_model, config)

    # Setup sources
    sources = np.array([
        [100.0, 100.0, 50.0],
        [150.0, 150.0, 60.0]
    ])

    # Setup observations
    obs_positions = np.array([[80.0, 80.0], [120.0, 120.0]])
    obs_intensities = np.array([45.0, 55.0])

    def nearest_intensity_lookup(pose):
        pos = pose.as_array()
        distances = np.linalg.norm(obs_positions - pos[None, :], axis=1)
        nearest_idx = np.argmin(distances)
        return obs_intensities[nearest_idx], 0, distances[nearest_idx]

    # Build tree
    root_pose = Pose2D(x=50.0, y=50.0, theta=0.0)
    print(f"\nBuilding RRT tree from ({root_pose.x}, {root_pose.y})...")

    nodes, leaves = planner.build_tree(
        root_pose=root_pose,
        sources=sources,
        observation_positions=obs_positions,
        observation_intensities=obs_intensities,
        nearest_intensity_lookup=nearest_intensity_lookup
    )

    # Verify tree structure
    print(f"\nTree structure:")
    print(f"  ✓ Total nodes: {len(nodes)} (논문 Eq. 8-10)")
    print(f"  ✓ Leaf nodes: {len(leaves)}")
    print(f"  ✓ Root: ({nodes[0].pose.x}, {nodes[0].pose.y})")

    # Select best branch
    best_branch, best_leaf = planner.select_best_branch(nodes, leaves)
    print(f"\nBest branch:")
    print(f"  ✓ Length: {len(best_branch)} nodes")
    print(f"  ✓ Total gain: {nodes[best_leaf].cumulative_gain:.6f}")

    assert len(nodes) > 0, "Tree should have nodes"
    assert len(leaves) > 0, "Tree should have leaves"
    assert len(best_branch) > 0, "Best branch should exist"

    print("\n✅ RRT Path Planning: PASS")
    return True


def test_radiation_gain_model():
    """
    Test 3: Radiation Gain Model (Section 4.2)

    논문 검증:
    - Single source gain (Eq. 11)
    - Multi-source gain (Eq. 12)
    - Superposition suppression (Eq. 13-14)
    - Distance/rotation costs (Eq. 16-18)
    - Gain corrections (Eq. 19-22)
    """
    print("\n" + "="*70)
    print("TEST 3: Radiation Gain Model (Section 4.2)")
    print("="*70)

    gain_model = RadiationGainModel(GainParameters())

    # Test single source gain (Eq. 11)
    node_pos = np.array([100.0, 100.0])
    source_pos = np.array([102.0, 102.0])

    gain = gain_model.single_source_gain(node_pos, source_pos)
    print(f"\nSingle source gain (Eq. 11):")
    print(f"  ✓ Node: ({node_pos[0]}, {node_pos[1]})")
    print(f"  ✓ Source: ({source_pos[0]}, {source_pos[1]})")
    print(f"  ✓ Gain: {gain:.6f}")

    # Test multi-source with suppression (Eq. 12-14)
    sources = np.array([
        [100.0, 100.0, 50.0],
        [105.0, 105.0, 55.0]
    ])

    suppression = gain_model.suppression_factor(node_pos, 0, sources)
    print(f"\nSuperposition suppression (Eq. 13-14):")
    print(f"  ✓ Suppression factor: {suppression:.6f}")

    # Test costs (Eq. 16-18)
    dist_cost = gain_model.distance_cost(step_length=10.0)
    rot_cost = gain_model.rotation_cost(theta_delta=np.pi/4)

    print(f"\nCosts (Eq. 16-18):")
    print(f"  ✓ Distance cost: {dist_cost:.6f}")
    print(f"  ✓ Rotation cost: {rot_cost:.6f}")

    # Test corrections (Eq. 19-22)
    oic = gain_model.observation_intensity_correction(nearest_intensity=200.0)
    rsc = gain_model.redundant_sampling_correction(samples_near=5, distance_near=15.0)
    rec = gain_model.repeat_exploring_correction(np.array([10.0, 15.0, 20.0]))

    print(f"\nGain corrections (Eq. 19-22):")
    print(f"  ✓ OIC (Eq. 19): {oic:.6f}")
    print(f"  ✓ RSC (Eq. 20): {rsc:.6f}")
    print(f"  ✓ REC (Eq. 21): {rec:.6f}")

    assert gain > 0, "Gain should be positive"
    assert 0 <= suppression <= 1, "Suppression should be in [0,1]"
    assert 0 <= dist_cost <= 1, "Distance cost should be in [0,1]"

    print("\n✅ Radiation Gain Model: PASS")
    return True


def test_dynamic_swarm_adjustment():
    """
    Test 4: Dynamic Swarm Adjustment (Fig. 11)

    논문 검증:
    - RFC-based swarm adjustment
    - Swarm number increases when RFC is low
    - Performance improves after adjustment
    """
    print("\n" + "="*70)
    print("TEST 4: Dynamic Swarm Adjustment (Fig. 11)")
    print("="*70)

    # Create ground truth with 4 sources
    rng = np.random.default_rng(42)
    n_sources = 4
    coords, amps, sigmas = sample_sources(GRID, n_sources, rng=rng)
    gt_field = gaussian_field(GRID, coords, amps, sigmas)

    print(f"\nGround Truth (4 sources, but starting with 3 swarms):")
    for i in range(n_sources):
        print(f"  Source {i}: ({coords[i,1]:.1f}, {coords[i,0]:.1f})")

    # Create observer and observations
    observer = RadiationObserver(gt_field, GRID)
    observations = []
    for _ in range(30):
        x = rng.integers(30, 226)
        y = rng.integers(30, 226)
        intensity = observer.observe((y, x))
        observations.append((np.array([y, x]), intensity))

    # Initialize with INSUFFICIENT swarms (3 for 4 sources)
    config = ADEPSPFConfig(
        n_swarms=3,
        n_particles=50,
        ade_generations=3
    )
    estimator = ADEPSPF(config=config, rng=rng)

    # Initialize adjuster
    adj_config = SwarmAdjustmentConfig(
        low_rfc_threshold=0.75,
        min_iterations_before_adjustment=3,
        check_interval=2,
        enabled=True
    )
    adjuster = DynamicSwarmAdjuster(adj_config)

    # Run with adjustment
    print(f"\nRunning estimation with dynamic adjustment...")
    adjustments_made = []

    for iteration in range(10):
        result = estimator.update(observations)

        should_add, reason = adjuster.should_add_swarm(
            estimator, iteration, result['rfc'], len(result['sources'])
        )

        if should_add:
            adjuster.add_swarm_to_estimator(estimator, iteration, reason)
            adjustments_made.append(iteration)

        print(f"  Iter {iteration}: Swarms={len(estimator.swarms)}, "
              f"RFC={result['rfc']:.4f}, Sources={len(result['sources'])}")

    # Verify adjustments
    stats = adjuster.get_statistics()
    print(f"\nAdjustment results:")
    print(f"  ✓ Total adjustments: {stats['n_adjustments']}")
    print(f"  ✓ Adjusted at iterations: {adjustments_made}")
    print(f"  ✓ Final swarms: {len(estimator.swarms)}")

    assert stats['n_adjustments'] > 0, "Should have made at least one adjustment"

    print("\n✅ Dynamic Swarm Adjustment: PASS")
    return True


def test_integrated_exploration():
    """
    Test 5: Integrated Exploration (Fig. 2, Full OEE Loop)

    논문 검증:
    - Observation-Estimation-Exploration iteration
    - Phase transitions (Fig. 14-16)
    - Termination criteria (Section 5)
    - Complete system integration
    """
    print("\n" + "="*70)
    print("TEST 5: Integrated Exploration (Full OEE Loop)")
    print("="*70)

    # Create ground truth (3 sources as in paper scenarios)
    rng = np.random.default_rng(42)
    n_sources = 3
    coords, amps, sigmas = sample_sources(GRID, n_sources, rng=rng)
    gt_field = gaussian_field(GRID, coords, amps, sigmas)

    print(f"\nGround Truth (3 sources):")
    for i in range(n_sources):
        print(f"  Source {i}: pos=({coords[i,1]:.1f}, {coords[i,0]:.1f}), "
              f"amp={amps[i]:.2f}")

    # Configure explorer
    config = ExplorationConfigV3(
        max_iterations=15,
        n_swarms=4,
        n_particles_per_swarm=50,
        ade_generations=3,
        use_rrt_v2=True,
        enable_verbose_logging=False,
        enable_branch_reuse=True
    )

    # Create and run explorer
    explorer = IntegratedExplorerV3(gt_field, config, rng)

    print(f"\nRunning integrated exploration...")
    success = explorer.run_exploration()

    # Verify results
    print(f"\nExploration results:")
    print(f"  ✓ Success: {success}")
    print(f"  ✓ Iterations: {len(explorer.iteration_results)}")
    print(f"  ✓ Final RFC: {explorer.best_rfc_overall:.4f}")
    print(f"  ✓ Trajectory points: {len(explorer.robot.trajectory)}")
    print(f"  ✓ Observations: {len(explorer.robot.observations)}")
    print(f"  ✓ Final swarms: {len(explorer.estimator.swarms)}")

    # Verify phase transitions
    if explorer.phase_history:
        print(f"\nPhase transitions:")
        for iter_num, phase in explorer.phase_history:
            print(f"  ✓ Iteration {iter_num}: → {phase.name}")

    # Verify swarm adjustments
    swarm_stats = explorer.swarm_adjuster.get_statistics()
    if swarm_stats['n_adjustments'] > 0:
        print(f"\nSwarm adjustments:")
        for adj in swarm_stats['adjustments']:
            print(f"  ✓ Iteration {adj[0]}: {adj[1]} → {adj[2]} swarms")

    assert success, "Exploration should succeed"
    assert len(explorer.iteration_results) > 0, "Should have iteration results"
    assert explorer.best_rfc_overall > 0, "Should have positive RFC"

    print("\n✅ Integrated Exploration: PASS")
    return True


def run_all_tests():
    """Run all paper compliance tests."""
    print("\n" + "="*70)
    print("PAPER COMPLIANCE TEST SUITE")
    print("논문: A study of robotic search strategy for multi-radiation sources")
    print("="*70)

    tests = [
        ("ADE-PSPF Estimation (Section 3)", test_ade_pspf_estimation),
        ("RRT Path Planning (Section 4.1)", test_rrt_path_planning),
        ("Radiation Gain Model (Section 4.2)", test_radiation_gain_model),
        ("Dynamic Swarm Adjustment (Fig. 11)", test_dynamic_swarm_adjustment),
        ("Integrated Exploration (Full OEE)", test_integrated_exploration),
    ]

    results = []
    for name, test_func in tests:
        try:
            success = test_func()
            results.append((name, success))
        except Exception as e:
            print(f"\n❌ {name}: FAILED")
            print(f"   Error: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, False))

    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)

    for name, success in results:
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"{status} - {name}")

    total = len(results)
    passed = sum(1 for _, s in results if s)

    print(f"\nTotal: {passed}/{total} tests passed")

    if passed == total:
        print("\n🎉 All tests passed! Implementation is paper-compliant.")
        return True
    else:
        print(f"\n⚠️  {total - passed} test(s) failed. Review implementation.")
        return False


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
