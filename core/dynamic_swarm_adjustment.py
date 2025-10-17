"""
Dynamic Swarm Adjustment Module (논문 Fig. 11)

Implements automatic swarm number adjustment based on RFC performance.

논문 설명:
"When the swarm number is less than the actual source number, ADE-PSPF
cannot accurately estimate the source parameters. However, each swarm is
loosely coupled in ADE-PSPF. The proposed method can increase the number
of swarms to infer more precise source parameters." (Section 5.1)

Key Concepts:
1. RFC 기반 swarm 수 조정
2. Swarm 수가 부족하면 RFC가 낮음
3. Swarm 추가 시 빠르게 수렴
4. Swarm 수가 실제 소스 수를 초과해도 안정적
"""
from __future__ import annotations
from typing import TYPE_CHECKING
from dataclasses import dataclass

if TYPE_CHECKING:
    from core.ade_pspf import ADEPSPF


@dataclass
class SwarmAdjustmentConfig:
    """Configuration for dynamic swarm adjustment."""

    # RFC thresholds for adjustment (balanced to match paper behavior)
    low_rfc_threshold: float = 0.75  # RFC below this triggers consideration
    very_low_rfc_threshold: float = 0.55  # RFC below this triggers immediate add

    # Timing parameters (balanced - allow convergence but not too slow)
    min_iterations_before_adjustment: int = 8  # Wait at least N iterations
    check_interval: int = 4  # Check every N iterations

    # Swarm limits
    max_swarms: int = 10  # Maximum number of swarms
    min_swarms: int = 2   # Minimum number of swarms

    # Performance tracking (stricter criteria)
    stability_window: int = 5  # Number of iterations to check RFC stability (was 3)
    improvement_threshold: float = 0.08  # Minimum RFC improvement required (was 0.05)

    # Enable/disable
    enabled: bool = True  # Master switch


class DynamicSwarmAdjuster:
    """
    Manages automatic swarm number adjustment based on RFC.

    논문 Fig. 11 구현:
    - RFC가 낮고 안정적이지 않으면 swarm 추가
    - RFC가 개선되지 않으면 계속 추가 (최대값까지)
    - Valid sources 수도 함께 고려
    """

    def __init__(self, config: SwarmAdjustmentConfig | None = None):
        """Initialize adjuster."""
        self.config = config or SwarmAdjustmentConfig()
        self.rfc_history: list[float] = []
        self.n_swarms_history: list[int] = []
        self.adjustment_history: list[tuple[int, int, int, str]] = []  # (iteration, old, new, reason)

    def should_add_swarm(
        self,
        estimator: ADEPSPF,
        iteration: int,
        current_rfc: float,
        n_valid_sources: int
    ) -> tuple[bool, str]:
        """
        Determine if a swarm should be added.

        Args:
            estimator: ADE-PSPF estimator instance
            iteration: Current iteration number
            current_rfc: Current RFC value
            n_valid_sources: Number of valid sources detected

        Returns:
            (should_add, reason) tuple
        """
        if not self.config.enabled:
            return False, "Dynamic adjustment disabled"

        # Track history
        self.rfc_history.append(current_rfc)
        self.n_swarms_history.append(len(estimator.swarms))

        # Check if we can add more swarms
        if len(estimator.swarms) >= self.config.max_swarms:
            return False, f"Already at maximum swarms ({self.config.max_swarms})"

        # Wait minimum iterations before first adjustment
        if iteration < self.config.min_iterations_before_adjustment:
            return False, f"Waiting for minimum iterations ({iteration}/{self.config.min_iterations_before_adjustment})"

        # Only check at intervals
        if iteration % self.config.check_interval != 0:
            return False, f"Not at check interval (iteration {iteration})"

        # Case 1: RFC is very low → immediate add
        if current_rfc < self.config.very_low_rfc_threshold:
            return True, f"RFC very low ({current_rfc:.3f} < {self.config.very_low_rfc_threshold})"

        # Case 2: RFC is low and not improving
        if current_rfc < self.config.low_rfc_threshold:
            if len(self.rfc_history) >= self.config.stability_window:
                recent_rfcs = self.rfc_history[-self.config.stability_window:]
                rfc_improvement = max(recent_rfcs) - min(recent_rfcs)

                if rfc_improvement < self.config.improvement_threshold:
                    return True, (f"RFC low ({current_rfc:.3f}) and not improving "
                                f"(improvement={rfc_improvement:.3f} < {self.config.improvement_threshold})")

        # Case 3: Valid sources >= swarms (논문 Fig. 11 관찰)
        # But be more conservative - require significant difference and low RFC
        if n_valid_sources > len(estimator.swarms):
            # Only add if RFC is quite low AND sources significantly exceed swarms
            if current_rfc < 0.70 and n_valid_sources >= len(estimator.swarms) + 2:
                if len(estimator.swarms) < self.config.max_swarms:
                    return True, (f"Valid sources ({n_valid_sources}) >> swarms ({len(estimator.swarms)}) "
                                f"and RFC < 0.70")

        return False, "No adjustment needed"

    def add_swarm_to_estimator(
        self,
        estimator: ADEPSPF,
        iteration: int,
        reason: str
    ) -> bool:
        """
        Add a new swarm to the estimator.

        Args:
            estimator: ADE-PSPF estimator instance
            iteration: Current iteration number
            reason: Reason for adding swarm

        Returns:
            True if swarm was added successfully
        """
        old_n_swarms = len(estimator.swarms)

        try:
            # Add swarm using estimator's method
            new_swarm_id = estimator.add_swarm()
            new_n_swarms = len(estimator.swarms)

            # Record adjustment
            self.adjustment_history.append((iteration, old_n_swarms, new_n_swarms, reason))

            print(f"\n{'='*70}")
            print(f"🔧 SWARM ADJUSTMENT @ Iteration {iteration}")
            print(f"{'='*70}")
            print(f"  Swarms: {old_n_swarms} → {new_n_swarms} (+1)")
            print(f"  Reason: {reason}")
            print(f"  New Swarm ID: {new_swarm_id}")
            print(f"{'='*70}\n")

            return True

        except Exception as e:
            print(f"❌ Failed to add swarm: {e}")
            return False

    def get_statistics(self) -> dict:
        """Get adjustment statistics."""
        return {
            'n_adjustments': len(self.adjustment_history),
            'adjustments': self.adjustment_history,
            'rfc_history': self.rfc_history,
            'n_swarms_history': self.n_swarms_history,
            'config': self.config
        }

    def reset(self):
        """Reset adjustment history."""
        self.rfc_history.clear()
        self.n_swarms_history.clear()
        self.adjustment_history.clear()


def should_adjust_swarms(
    estimator: ADEPSPF,
    iteration: int,
    adjuster: DynamicSwarmAdjuster | None = None
) -> tuple[bool, str]:
    """
    Convenience function for checking swarm adjustment.

    Usage:
        adjuster = DynamicSwarmAdjuster()

        # In exploration loop:
        result = estimator.update(observations)
        should_add, reason = should_adjust_swarms(estimator, iteration, adjuster)
        if should_add:
            adjuster.add_swarm_to_estimator(estimator, iteration, reason)
    """
    if adjuster is None:
        adjuster = DynamicSwarmAdjuster()

    current_rfc = estimator.get_current_rfc()
    n_valid = len(estimator.get_valid_sources())

    return adjuster.should_add_swarm(estimator, iteration, current_rfc, n_valid)


if __name__ == "__main__":
    """Test dynamic swarm adjustment."""
    from environment.generate_truth import sample_sources, gaussian_field, GRID
    from environment.observation import RadiationObserver
    from core.ade_pspf import ADEPSPF, ADEPSPFConfig
    import numpy as np

    print("Testing Dynamic Swarm Adjustment...\n")

    # Create ground truth with 4 sources
    print("=== Creating Ground Truth (4 sources) ===")
    rng = np.random.default_rng(42)
    n_sources = 4
    coords, amps, sigmas = sample_sources(GRID, n_sources, rng=rng)
    gt_field = gaussian_field(GRID, coords, amps, sigmas)

    print(f"True sources:")
    for i in range(n_sources):
        print(f"  Source {i}: pos=({coords[i,1]:.1f}, {coords[i,0]:.1f}), "
              f"intensity={amps[i]:.2f}")

    # Create observer
    observer = RadiationObserver(gt_field, GRID)

    # Generate observations
    print("\n=== Generating Observations ===")
    n_obs = 30
    observations = []
    for _ in range(n_obs):
        x = rng.integers(30, 226)
        y = rng.integers(30, 226)
        intensity = observer.observe((y, x))
        observations.append((np.array([y, x]), intensity))

    print(f"Generated {len(observations)} observations")

    # Initialize ADE-PSPF with INSUFFICIENT swarms (3 swarms for 4 sources)
    print("\n=== Initializing ADE-PSPF (3 swarms, insufficient for 4 sources) ===")
    config = ADEPSPFConfig(
        n_swarms=3,  # Intentionally less than actual sources
        n_particles=50,
        n_iterations=20,
        ade_generations=3
    )
    estimator = ADEPSPF(config=config, rng=rng)

    # Initialize adjuster
    adj_config = SwarmAdjustmentConfig(
        low_rfc_threshold=0.75,
        very_low_rfc_threshold=0.60,
        min_iterations_before_adjustment=3,
        check_interval=2
    )
    adjuster = DynamicSwarmAdjuster(adj_config)

    # Run estimation with dynamic adjustment
    print("\n=== Running Estimation with Dynamic Adjustment ===")
    for iteration in range(1, 16):
        # Update estimation
        result = estimator.update(observations)

        print(f"\nIteration {iteration}:")
        print(f"  Swarms: {len(estimator.swarms)}")
        print(f"  RFC: {result['rfc']:.4f}")
        print(f"  Valid Sources: {len(result['sources'])}")

        # Check if we should adjust swarms
        should_add, reason = adjuster.should_add_swarm(
            estimator, iteration, result['rfc'], len(result['sources'])
        )

        if should_add:
            adjuster.add_swarm_to_estimator(estimator, iteration, reason)
        else:
            print(f"  No adjustment: {reason}")

    # Show final results
    print("\n=== Final Results ===")
    final_result = estimator.get_statistics()
    print(f"Final swarms: {final_result['n_swarms']}")
    print(f"Final RFC: {final_result['current_rfc']:.4f}")
    print(f"Valid sources found: {final_result['n_valid_sources']}")

    stats = adjuster.get_statistics()
    print(f"\nAdjustment summary:")
    print(f"  Total adjustments: {stats['n_adjustments']}")
    for adj in stats['adjustments']:
        print(f"  - Iteration {adj[0]}: {adj[1]} → {adj[2]} swarms ({adj[3]})")

    print("\n✅ Dynamic swarm adjustment test completed!")
