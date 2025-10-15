"""
Simple Integration Test - 기존 시스템을 그대로 사용하여 성능 측정
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import time
from utils.performance_metrics import compute_all_metrics, format_metrics


def test_simple_3source():
    """
    기존 시스템을 그대로 사용하여 3-source 시나리오 테스트
    """
    from environment.generate_truth import inverse_square_field
    from simulation.integrated_explorer_v2 import IntegratedExplorerV2, ExplorationConfig

    print("\n" + "="*70)
    print("Simple 3-Source Integration Test")
    print("="*70)

    # True sources
    true_sources = [
        np.array([128.0, 180.0, 50.0]),
        np.array([90.0, 128.0, 60.0]),
        np.array([166.0, 90.0, 55.0])
    ]

    print("\nTrue sources:")
    for i, src in enumerate(true_sources):
        print(f"  Source {i+1}: x={src[0]:.1f}px, y={src[1]:.1f}px, I={src[2]:.1f}")

    # Create radiation field
    coords = np.array([src[:2] for src in true_sources])
    amps = np.array([src[2] for src in true_sources])
    radiation_field = inverse_square_field(grid=256, coords=coords, amps=amps, h=0.5)

    # Configure exploration
    config = ExplorationConfig(
        max_iterations=30,
        log_iteration_details=False
    )

    # Run exploration
    explorer = IntegratedExplorerV2(
        gt_field=radiation_field,
        config=config,
        rng=np.random.default_rng(42)
    )

    print("\nRunning exploration...")
    start_time = time.time()
    results = explorer.run_exploration()
    elapsed_time = time.time() - start_time

    # Extract results
    final_sources = results['final_sources']
    final_rfc = results['final_rfc']

    print(f"\nExploration complete:")
    print(f"  Time: {elapsed_time:.1f}s")
    print(f"  Iterations: {results['iterations_completed']}")
    print(f"  RFC: {final_rfc:.4f}")
    print(f"  Estimated sources: {len(final_sources)}")

    for i, src in enumerate(final_sources):
        print(f"    Source {i+1}: x={src[0]:.1f}px, y={src[1]:.1f}px, I={src[2]:.1f}")

    # Compute metrics
    metrics = compute_all_metrics(
        true_sources=true_sources,
        estimated_sources=final_sources,
        rfc=final_rfc
    )

    print("\n" + format_metrics(metrics))

    return metrics


if __name__ == "__main__":
    metrics = test_simple_3source()

    print("\n✅ Simple integration test complete!")
    print(f"\nKey Results:")
    print(f"  SR: {metrics['success_rate']:.1f}%")
    print(f"  RFC: {metrics['rfc']:.4f}")
    if metrics['localization_error_mean'] != float('inf'):
        print(f"  LE: {metrics['localization_error_mean']:.3f}m")
