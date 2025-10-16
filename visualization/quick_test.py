"""
Quick test to verify basic functionality
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from visualization.explorer_controller import ExplorerController
from visualization.data_manager import DataManager

def quick_test():
    """Quick test with minimal configuration."""
    print("\n" + "="*70)
    print("QUICK TEST: Interactive Explorer App")
    print("="*70)

    # Test 1: GT generation
    print("\n[1/4] Testing Ground Truth generation...")
    controller = ExplorerController()
    success = controller.generate_ground_truth(n_sources=2, seed=42)
    if not success:
        print("✗ FAILED: GT generation")
        return False
    print("✓ GT generation OK")

    # Test 2: Explorer initialization
    print("\n[2/4] Testing Explorer initialization...")
    success = controller.initialize_explorer(
        max_iterations=2,
        n_swarms=2,
        n_particles=20,  # Reduced for speed
        ade_generations=1,  # Reduced for speed
        enable_logs=False
    )
    if not success:
        print("✗ FAILED: Explorer initialization")
        return False
    print("✓ Explorer initialization OK")

    # Test 3: Run exploration (minimal)
    print("\n[3/4] Testing Exploration with snapshots (1 iteration)...")
    # Manually run just 1 iteration for testing
    try:
        controller.explorer._make_observation(
            controller.explorer.robot.pose.x,
            controller.explorer.robot.pose.y
        )

        # Just test estimation step
        estimation_result = controller.explorer._estimation_step()
        print(f"  Estimation: RFC={estimation_result['rfc']:.4f}, Sources={estimation_result['n_sources']}")
        print("✓ Estimation step OK")

        # Test exploration step
        best_branch, nodes, leaves = controller._exploration_step_with_tree_data(
            estimation_result['sources']
        )
        print(f"  Exploration: Nodes={len(nodes)}, Branch={len(best_branch)}")
        print("✓ Exploration step OK")

    except Exception as e:
        print(f"✗ FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False

    # Test 4: Data structures
    print("\n[4/4] Testing data structures...")
    from visualization.data_manager import (
        EstimationStepData, ExplorationStepData, IterationData
    )

    # Create sample iteration data
    iter_data = IterationData(iteration=0)
    iter_data.phase = controller.explorer.current_phase
    iter_data.rfc_before = 0.0
    iter_data.rfc_after = estimation_result['rfc']

    print(f"  IterationData: iteration={iter_data.iteration}, phase={iter_data.phase}")
    print("✓ Data structures OK")

    print("\n" + "="*70)
    print("✅ ALL QUICK TESTS PASSED!")
    print("="*70)
    print("\nThe app is ready to use. Run:")
    print("  python3 visualization/interactive_explorer_app.py")
    print("="*70 + "\n")

    return True


if __name__ == "__main__":
    success = quick_test()
    sys.exit(0 if success else 1)
