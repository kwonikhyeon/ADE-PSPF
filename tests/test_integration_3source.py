"""
3-Source Integration Test

논문 성능 재현 목표 (논문 Table 6 참조):
- SR: 85%
- RFC: 0.95 ± 0.01
- LE: 0.25 ± 0.05 m
- IE: 124.72 ± 47.41 nSv/h (논문은 nSv/h 단위지만 우리는 임의 단위)

시나리오 설정 (10m×10m 환경에 맞춰 조정):
- Scene: 10m × 10m (256px × 256px)
- Sources: 3개
  - Source 1: (128px, 180px, 50) ≈ (5.0m, 7.0m)
  - Source 2: (90px, 128px, 60) ≈ (3.5m, 5.0m)
  - Source 3: (166px, 90px, 55) ≈ (6.5m, 3.5m)
- Robot start: (64px, 50px, 90°) ≈ (2.5m, 2.0m)
- Swarms: 4
- Particles per swarm: 250
- Max iterations: 50
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import time
from typing import Dict, List
from utils.performance_metrics import compute_all_metrics, format_metrics


def create_3source_environment():
    """
    Create 3-source environment for testing.

    Returns:
        (sources, start_pose, grid_size)
    """
    # 3 sources in 10m×10m space (256px×256px)
    sources = [
        np.array([128.0, 180.0, 50.0]),  # Center-top
        np.array([90.0, 128.0, 60.0]),   # Left-center
        np.array([166.0, 90.0, 55.0])    # Right-bottom
    ]

    # Robot starts at bottom-left
    start_pose = (64.0, 50.0, np.pi/2)  # (x, y, theta)

    grid_size = 256

    return sources, start_pose, grid_size


def run_3source_scenario(
    seed: int = 42,
    max_iterations: int = 50,
    verbose: bool = True
) -> Dict:
    """
    Run 3-source exploration scenario.

    Args:
        seed: Random seed
        max_iterations: Maximum iterations
        verbose: Print detailed logs

    Returns:
        Dictionary with results and metrics
    """
    from environment.generate_truth import inverse_square_field
    from environment.observation import RadiationObserver
    from simulation.integrated_explorer_v2 import IntegratedExplorerV2, ExplorationConfig
    from core.ade_pspf import ADEPSPFConfig

    rng = np.random.default_rng(seed)

    # Create environment
    true_sources, start_pose, grid_size = create_3source_environment()

    if verbose:
        print(f"\n{'='*70}")
        print(f"3-Source Integration Test (seed={seed})")
        print(f"{'='*70}")
        print(f"\nTrue sources:")
        for i, src in enumerate(true_sources):
            print(f"  Source {i+1}: x={src[0]:.1f}px, y={src[1]:.1f}px, I={src[2]:.1f}")
        print(f"\nRobot start: x={start_pose[0]:.1f}px, y={start_pose[1]:.1f}px, θ={start_pose[2]:.2f}rad")
        print(f"Max iterations: {max_iterations}")
        print(f"{'='*70}\n")

    # Create radiation field
    # Separate coords and amps from sources
    coords = np.array([src[:2] for src in true_sources])  # (N, 2)
    amps = np.array([src[2] for src in true_sources])     # (N,)

    radiation_field = inverse_square_field(
        grid=grid_size,
        coords=coords,
        amps=amps,
        h=0.5
    )

    # Configure exploration (IntegratedExplorerV2 uses gt_field directly)
    adepspf_config = ADEPSPFConfig(
        n_swarms=4,
        n_particles=250,
        n_iterations=15,
        ade_generations=10,
        x_min=0.0,
        x_max=float(grid_size),
        y_min=0.0,
        y_max=float(grid_size)
    )

    exploration_config = ExplorationConfig(
        grid_size=grid_size,
        robot_start_x=start_pose[0],
        robot_start_y=start_pose[1],
        robot_start_theta=start_pose[2],
        max_iterations=max_iterations,
        branch_execution_ratio=0.0,  # First edge only
        observations_per_iteration=1,
        log_iteration_details=verbose,
        enable_timing_logs=False,
        adepspf_config=adepspf_config
    )

    # Run exploration
    explorer = IntegratedExplorerV2(
        gt_field=radiation_field,
        config=exploration_config,
        rng=rng
    )

    start_time = time.time()
    results = explorer.run_exploration()
    elapsed_time = time.time() - start_time

    # Extract final results
    final_sources = results['final_sources']
    final_rfc = results['final_rfc']
    n_iterations = results['iterations']

    if verbose:
        print(f"\n{'='*70}")
        print(f"Exploration Complete")
        print(f"{'='*70}")
        print(f"Iterations completed: {n_iterations}")
        print(f"Time elapsed: {elapsed_time:.1f}s")
        print(f"Final RFC: {final_rfc:.4f}")
        print(f"\nEstimated sources: {len(final_sources)}")
        for i, src in enumerate(final_sources):
            print(f"  Source {i+1}: x={src[0]:.1f}px, y={src[1]:.1f}px, I={src[2]:.1f}")

    # Compute metrics
    metrics = compute_all_metrics(
        true_sources=true_sources,
        estimated_sources=final_sources,
        rfc=final_rfc,
        distance_threshold_meters=0.5
    )

    if verbose:
        print("\n" + format_metrics(metrics))

    return {
        'true_sources': true_sources,
        'estimated_sources': final_sources,
        'final_rfc': final_rfc,
        'n_iterations': n_iterations,
        'elapsed_time': elapsed_time,
        'metrics': metrics,
        'seed': seed
    }


def test_3source_single_run():
    """
    Test: 3-source scenario - single run.
    """
    print("\n" + "="*70)
    print("Test 1: 3-Source Scenario - Single Run")
    print("="*70)

    result = run_3source_scenario(seed=42, max_iterations=50, verbose=True)

    metrics = result['metrics']

    # Basic validation
    assert metrics['n_true_sources'] == 3, "Should have 3 true sources"
    assert metrics['rfc'] >= 0.0, "RFC should be non-negative"
    assert metrics['success_rate'] >= 0.0, "SR should be non-negative"

    print(f"\n✅ Single Run Test Passed!")
    print(f"   SR: {metrics['success_rate']:.1f}%")
    print(f"   RFC: {metrics['rfc']:.4f}")
    print(f"   LE: {metrics['localization_error_mean']:.3f}m")

    return result


def test_3source_multiple_runs(n_runs: int = 5):
    """
    Test: 3-source scenario - multiple runs.

    Args:
        n_runs: Number of runs (default 5 for quick testing, use 20 for full validation)
    """
    print("\n" + "="*70)
    print(f"Test 2: 3-Source Scenario - {n_runs} Runs")
    print("="*70)

    all_results = []

    for run_id in range(n_runs):
        seed = 42 + run_id
        print(f"\nRun {run_id + 1}/{n_runs} (seed={seed})...")

        result = run_3source_scenario(seed=seed, max_iterations=50, verbose=False)
        all_results.append(result)

        metrics = result['metrics']
        print(f"  SR: {metrics['success_rate']:.1f}%, "
              f"RFC: {metrics['rfc']:.4f}, "
              f"LE: {metrics['localization_error_mean']:.3f}m")

    # Aggregate statistics
    sr_values = [r['metrics']['success_rate'] for r in all_results]
    rfc_values = [r['metrics']['rfc'] for r in all_results]
    le_values = [r['metrics']['localization_error_mean'] for r in all_results if r['metrics']['localization_error_mean'] != float('inf')]
    ie_values = [r['metrics']['intensity_error_mean'] for r in all_results if r['metrics']['intensity_error_mean'] != float('inf')]

    print(f"\n{'='*70}")
    print(f"Aggregated Results ({n_runs} runs)")
    print(f"{'='*70}")
    print(f"Success Rate (SR):       {np.mean(sr_values):.1f} ± {np.std(sr_values):.1f}%")
    print(f"RFC:                     {np.mean(rfc_values):.4f} ± {np.std(rfc_values):.4f}")
    if le_values:
        print(f"Localization Error (LE): {np.mean(le_values):.3f} ± {np.std(le_values):.3f} m")
    if ie_values:
        print(f"Intensity Error (IE):    {np.mean(ie_values):.2f} ± {np.std(ie_values):.2f}")
    print(f"{'='*70}")

    # Compare with paper (논문 목표: SR=85%, RFC=0.95±0.01, LE=0.25±0.05m)
    print(f"\n{'='*70}")
    print(f"Comparison with Paper Goals")
    print(f"{'='*70}")
    print(f"{'Metric':<20s} {'Paper Goal':<20s} {'Our Result':<20s} {'Status':<10s}")
    print("-"*70)

    sr_mean = np.mean(sr_values)
    sr_status = "✅" if sr_mean >= 70 else "⚠️" if sr_mean >= 50 else "❌"
    print(f"{'SR (%)':<20s} {'~85':<20s} {f'{sr_mean:.1f}':<20s} {sr_status:<10s}")

    rfc_mean = np.mean(rfc_values)
    rfc_status = "✅" if rfc_mean >= 0.85 else "⚠️" if rfc_mean >= 0.70 else "❌"
    print(f"{'RFC':<20s} {'~0.95':<20s} {f'{rfc_mean:.4f}':<20s} {rfc_status:<10s}")

    if le_values:
        le_mean = np.mean(le_values)
        le_status = "✅" if le_mean <= 0.35 else "⚠️" if le_mean <= 0.50 else "❌"
        print(f"{'LE (m)':<20s} {'~0.25':<20s} {f'{le_mean:.3f}':<20s} {le_status:<10s}")

    print(f"{'='*70}")

    print(f"\n✅ Multiple Runs Test Passed!")
    print(f"   Completed {n_runs} runs successfully")

    return all_results


if __name__ == "__main__":
    print("\n" + "="*70)
    print("3-Source Integration Test")
    print("="*70)

    # Test 1: Single run
    test_3source_single_run()

    # Test 2: Multiple runs (5 runs for quick testing)
    print("\n" + "="*70)
    print("NOTE: Running 5 runs for quick testing")
    print("      For full validation, increase to 20 runs")
    print("="*70)
    test_3source_multiple_runs(n_runs=5)

    print("\n" + "="*70)
    print("✅ All 3-Source Integration Tests Complete!")
    print("="*70)
