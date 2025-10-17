"""
Test Interactive App Integration

Tests that the interactive explorer app correctly uses the paper-compliant
exploration algorithms including dynamic swarm adjustment.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from visualization.explorer_controller import ExplorerController


def test_controller_with_dynamic_swarm_adjustment():
    """
    Test ExplorerController with dynamic swarm adjustment.

    This verifies that the interactive app (via ExplorerController)
    correctly integrates:
    1. ADE-PSPF Estimation
    2. RRT-based Exploration
    3. Dynamic Swarm Adjustment (Fig. 11)
    4. Phase transitions
    """
    print("\n" + "="*70)
    print("TEST: Interactive App Integration")
    print("="*70)

    # Create controller
    controller = ExplorerController()

    # Generate ground truth (4 sources to trigger swarm adjustment)
    print("\nGenerating ground truth with 4 sources...")
    success = controller.generate_ground_truth(
        n_sources=4,
        seed=42,
        grid_size=256
    )
    assert success, "Ground truth generation failed"

    # Initialize explorer with INSUFFICIENT swarms (3 for 4 sources)
    print("\nInitializing explorer with 3 swarms (insufficient for 4 sources)...")
    success = controller.initialize_explorer(
        max_iterations=15,
        n_swarms=3,  # Start with 3 swarms
        n_particles=50,  # Smaller for faster test
        ade_generations=2,
        enable_logs=True  # Enable detailed logging
    )
    assert success, "Explorer initialization failed"

    # Run exploration with snapshot capture
    print("\nRunning exploration with snapshot capture...")
    success = controller.run_exploration_with_snapshots()
    assert success, "Exploration failed"

    # Verify results
    snapshot = controller.get_snapshot()
    assert snapshot is not None, "No snapshot captured"

    print("\n" + "="*70)
    print("VERIFICATION RESULTS")
    print("="*70)

    # Check iterations
    print(f"\n1. Exploration Iterations:")
    print(f"   Total iterations: {snapshot.n_iterations}")
    print(f"   Converged: {snapshot.converged}")
    assert snapshot.n_iterations > 0, "No iterations recorded"

    # Check RFC
    print(f"\n2. RFC Performance:")
    print(f"   Final RFC: {snapshot.final_rfc:.4f}")
    print(f"   Best RFC: {snapshot.best_rfc:.4f}")
    assert snapshot.best_rfc > 0, "Best RFC should be positive"

    # Check swarm adjustment
    swarm_stats = controller.explorer.swarm_adjuster.get_statistics()
    print(f"\n3. Dynamic Swarm Adjustment:")
    print(f"   Initial swarms: 3")
    print(f"   Final swarms: {len(controller.explorer.estimator.swarms)}")
    print(f"   Total adjustments: {swarm_stats['n_adjustments']}")

    if swarm_stats['n_adjustments'] > 0:
        print(f"   Adjustment details:")
        for adj in swarm_stats['adjustments']:
            print(f"     - Iteration {adj[0]}: {adj[1]} → {adj[2]} swarms")
            print(f"       Reason: {adj[3]}")

    # Should have made at least one adjustment
    assert swarm_stats['n_adjustments'] > 0, "Should have adjusted swarms"

    # Check phase transitions
    print(f"\n4. Phase Transitions:")
    if snapshot.phase_history:
        for iter_num, phase in snapshot.phase_history:
            phase_names = {1: "TRACING", 2: "SURROUNDING", 3: "EXPLORING"}
            print(f"   Iteration {iter_num}: → {phase_names.get(phase.value, 'UNKNOWN')}")
    else:
        print(f"   No phase transitions recorded")

    # Check observations
    print(f"\n5. Observations:")
    print(f"   Total observations: {len(snapshot.all_observations)}")
    print(f"   Trajectory points: {len(snapshot.full_trajectory)}")
    assert len(snapshot.all_observations) > 0, "Should have observations"

    # Check estimated sources
    print(f"\n6. Source Estimation:")
    print(f"   True sources: {snapshot.n_true_sources}")
    print(f"   Estimated sources: {len(snapshot.final_estimated_sources)}")
    if snapshot.final_estimated_sources:
        print(f"   Estimated locations:")
        for i, src in enumerate(snapshot.final_estimated_sources):
            print(f"     Source {i+1}: ({src[0]:.1f}, {src[1]:.1f}), intensity={src[2]:.2f}")

    # Check iteration data structure
    print(f"\n7. Snapshot Data Quality:")
    print(f"   Iterations with data: {len(snapshot.iteration_data)}")

    if len(snapshot.iteration_data) > 0:
        first_iter = snapshot.iteration_data[0]
        print(f"   First iteration has:")
        print(f"     - Estimation data: {first_iter.estimation_data is not None}")
        print(f"     - Exploration data: {first_iter.exploration_data is not None}")
        print(f"     - Execution data: {first_iter.execution_data is not None}")

        assert first_iter.estimation_data is not None, "Missing estimation data"
        assert first_iter.exploration_data is not None, "Missing exploration data"
        assert first_iter.execution_data is not None, "Missing execution data"

    print("\n" + "="*70)
    print("✅ All checks passed!")
    print("="*70)
    print("\nSummary:")
    print(f"  • Interactive app correctly integrates paper algorithms")
    print(f"  • Dynamic swarm adjustment working: {swarm_stats['n_adjustments']} adjustments")
    print(f"  • Phase transitions tracked: {len(snapshot.phase_history)} transitions")
    print(f"  • Final performance: RFC = {snapshot.final_rfc:.4f}")

    return True


if __name__ == "__main__":
    try:
        success = test_controller_with_dynamic_swarm_adjustment()
        if success:
            print("\n🎉 Integration test passed!")
            sys.exit(0)
        else:
            print("\n❌ Integration test failed!")
            sys.exit(1)
    except Exception as e:
        print(f"\n❌ Test error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
