"""
Observation module - Reads radiation intensity from ground truth at specific positions.

This module simulates radiation measurements by reading values directly from the
ground truth radiation field. It acts as a simple sensor simulator without noise
or measurement errors.
"""
from __future__ import annotations
import numpy as np
from typing import Tuple, List


class RadiationObserver:
    """
    Simple radiation observer that reads values from ground truth field.

    In a real scenario, this would be replaced by actual sensor readings with:
    - Poisson noise
    - Background radiation fluctuations
    - Sensor measurement errors

    For simulation purposes, we directly read from the ground truth field.
    """

    def __init__(self, ground_truth_field: np.ndarray, grid_size: int = 256):
        """
        Initialize the radiation observer.

        Args:
            ground_truth_field: Ground truth radiation field (grid_size x grid_size)
            grid_size: Size of the grid in pixels (default: 256)
        """
        self.gt_field = ground_truth_field
        self.grid_size = grid_size
        self.observation_history: List[Tuple[int, int, float]] = []

    def observe(self, position: Tuple[int, int]) -> float:
        """
        Observe radiation intensity at a specific position.

        Args:
            position: (y, x) position in pixel coordinates

        Returns:
            Radiation intensity at the given position
        """
        y, x = position

        # Boundary check
        if not (0 <= y < self.grid_size and 0 <= x < self.grid_size):
            raise ValueError(f"Position {position} is out of bounds [0, {self.grid_size})")

        # Read intensity from ground truth
        intensity = float(self.gt_field[y, x])

        # Record observation
        self.observation_history.append((y, x, intensity))

        return intensity

    def observe_batch(self, positions: List[Tuple[int, int]]) -> np.ndarray:
        """
        Observe radiation intensity at multiple positions.

        Args:
            positions: List of (y, x) positions in pixel coordinates

        Returns:
            Array of radiation intensities
        """
        intensities = []
        for pos in positions:
            intensities.append(self.observe(pos))
        return np.array(intensities)

    def get_observation_history(self) -> np.ndarray:
        """
        Get all recorded observations.

        Returns:
            Array of shape (N, 3) where each row is (y, x, intensity)
        """
        if not self.observation_history:
            return np.array([]).reshape(0, 3)
        return np.array(self.observation_history)

    def clear_history(self):
        """Clear observation history."""
        self.observation_history.clear()


def pixel_to_meter(pixel_pos: Tuple[int, int], grid_size: int = 256,
                   world_size: float = 10.0) -> Tuple[float, float]:
    """
    Convert pixel coordinates to meter coordinates.

    Args:
        pixel_pos: (y, x) position in pixel coordinates
        grid_size: Size of the grid in pixels
        world_size: Physical size of the world in meters

    Returns:
        (y_m, x_m) position in meters
    """
    y_px, x_px = pixel_pos
    scale = world_size / grid_size
    y_m = y_px * scale
    x_m = x_px * scale
    return y_m, x_m


def meter_to_pixel(meter_pos: Tuple[float, float], grid_size: int = 256,
                   world_size: float = 10.0) -> Tuple[int, int]:
    """
    Convert meter coordinates to pixel coordinates.

    Args:
        meter_pos: (y_m, x_m) position in meters
        grid_size: Size of the grid in pixels
        world_size: Physical size of the world in meters

    Returns:
        (y, x) position in pixel coordinates
    """
    y_m, x_m = meter_pos
    scale = grid_size / world_size
    y_px = int(round(y_m * scale))
    x_px = int(round(x_m * scale))
    return y_px, x_px


if __name__ == "__main__":
    # Test observation module
    from generate_truth import sample_sources, gaussian_field, GRID

    print("Testing Radiation Observer...")

    # Create a simple ground truth field
    coords, amps, sigmas = sample_sources(GRID, n=3)
    gt_field = gaussian_field(GRID, coords, amps, sigmas)

    # Initialize observer
    observer = RadiationObserver(gt_field, GRID)

    # Test single observation
    test_pos = (128, 128)
    intensity = observer.observe(test_pos)
    print(f"\n✓ Single observation at {test_pos}: {intensity:.4f}")

    # Test batch observation
    test_positions = [(64, 64), (128, 128), (192, 192)]
    intensities = observer.observe_batch(test_positions)
    print(f"\n✓ Batch observation:")
    for pos, intens in zip(test_positions, intensities):
        print(f"  Position {pos}: {intens:.4f}")

    # Test coordinate conversion
    pixel_pos = (128, 128)
    meter_pos = pixel_to_meter(pixel_pos)
    back_to_pixel = meter_to_pixel(meter_pos)
    print(f"\n✓ Coordinate conversion:")
    print(f"  Pixel {pixel_pos} → Meter {meter_pos} → Pixel {back_to_pixel}")

    # Show observation history
    history = observer.get_observation_history()
    print(f"\n✓ Observation history: {len(history)} measurements")

    print("\n✅ All tests passed!")
