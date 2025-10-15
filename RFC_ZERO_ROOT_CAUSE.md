# RFC = 0.0000 Root Cause Analysis

## Executive Summary

**Critical Bug Found**: The ADE-PSPF algorithm predicts intensities using an **inverse square law** model, but the ground truth generates intensities using a **Gaussian decay** model. This fundamental mismatch causes RFC to always be 0, preventing convergence.

---

## Problem Description

### Observation
- RFC remains at 0.0000 across all iterations
- Estimated source positions are 40-65 pixels away from true positions
- Predicted intensities are ~20-115, but observed intensities are ~0.0002-0.0020
- **Prediction error: ~10,000,000% (factor of 10 million)**

### Debug Output

```
6. PREDICTION ACCURACY CHECK:
   Obs @ (128.0, 128.0): true=0.0002, pred=20.4371, error=20.4370 (13106561.4%)
   Obs @ (130.0, 140.0): true=0.0020, pred=19.1394, error=19.1375 (966184.2%)
   Obs @ (140.0, 120.0): true=0.0012, pred=22.0388, error=22.0376 (1842535.1%)
   Obs @ (120.0, 150.0): true=0.0004, pred=20.4704, error=20.4700 (5204745.9%)
   Average error: 20.5205
```

---

## Root Cause

### Ground Truth Model (generate_truth.py)

The ground truth uses **2D isotropic Gaussian fields**:

```python
def gaussian_field(grid: int, coords: np.ndarray, amps: np.ndarray, sigmas: np.ndarray):
    """Return (H, W) field composed of isotropic 2‑D Gaussians."""
    yy, xx = np.meshgrid(np.arange(grid), np.arange(grid), indexing="ij")
    field = np.zeros((grid, grid), dtype=np.float32)
    for (y0, x0), A, s in zip(coords, amps, sigmas, strict=True):
        d2 = (yy - y0) ** 2 + (xx - x0) ** 2
        field += A * np.exp(-d2 / (2.0 * s ** 2))  # ← GAUSSIAN DECAY
    return field
```

**Key equation**:
```
I_true(x, y) = A * exp(-d² / (2σ²))
```

Where:
- `A` = peak amplitude (30-100)
- `d` = distance from source (pixels)
- `σ` = spatial spread (10-20 pixels)

**Characteristics**:
- At source center (d=0): I = A (30-100)
- At distance d=σ: I = A * exp(-0.5) ≈ 0.6A
- At distance d=2σ: I = A * exp(-2) ≈ 0.135A
- At distance d=3σ: I = A * exp(-4.5) ≈ 0.011A
- At distance d=4σ: I = A * exp(-8) ≈ 0.0003A ← **this is what we observe at (128, 128)**

### ADE-PSPF Prediction Model (weights.py)

The ADE-PSPF uses **inverse square law with height**:

```python
def predict_intensity(
    particle_pos: np.ndarray,
    particle_intensity: float,
    observation_pos: np.ndarray,
    other_centroids: List[np.ndarray],
    h: float = 0.5,
    mu_air: float = 6.86e-3
) -> float:
    # Convert pixel distance to meters
    pixel_to_meter = 10.0 / 256.0

    dist_pixels = np.linalg.norm(particle_pos - observation_pos[:2])
    dist_meters = dist_pixels * pixel_to_meter

    # Inverse square law
    intensity = particle_intensity / (h**2 + dist_meters**2)  # ← INVERSE SQUARE

    # Add contributions from other sources...
```

**Key equation**:
```
I_pred(x, y) = A / (h² + d²_meters)
```

Where:
- `A` = source intensity parameter (what ADE-PSPF estimates)
- `d_meters` = distance from source (meters)
- `h` = height of source (0.5 meters)

**Characteristics** (assuming A=60, h=0.5):
- At d=0m: I = 60 / 0.25 = **240**
- At d=1m: I = 60 / 1.25 ≈ **48**
- At d=2m: I = 60 / 4.25 ≈ **14**
- At d=5m (128 pixels): I = 60 / 25.25 ≈ **2.4** ← **observed is 0.0002!**

---

## Why This Causes RFC = 0

### RFC Calculation

RFC is calculated using Poisson likelihood ratios:

```python
# Equation 7 from paper
F = (1/N_o) Σ [P(k_obs | λ_pred) / P(floor(λ_pred) | λ_pred)]

where:
    λ_pred = E * τ * (I_pred + R_back)
    k_obs = E * τ * (I_obs + R_back)
```

### Concrete Example

For observation at (128, 128):
- True observed intensity: `I_obs = 0.0002`
- Predicted intensity: `I_pred = 20.4371`
- E = 1.0, τ = 1.0, R_back = 0.0

```
k_obs = 1.0 * 1.0 * 0.0002 = 0.0002
λ_pred = 1.0 * 1.0 * 20.4371 = 20.4371

P(k=0.0002 | λ=20.4371) ≈ 0.0 (Poisson of observing 0.0002 when expecting 20.4)
P(k=20 | λ=20.4371) ≈ 0.089

Ratio ≈ 0.0 / 0.089 ≈ 0.0
```

**Result**: RFC = 0.0000

---

## Solutions

### Option 1: Fix Ground Truth to Match Paper (Recommended)

The paper uses **3D inverse square law for gamma radiation**:

```
I(r) = (A₀ / r²) * exp(-μ * r)

where:
    A₀ = source strength
    r = 3D distance = sqrt(d² + h²)
    μ = air absorption coefficient
    h = height difference
```

**Implementation**:
```python
def inverse_square_field(grid: int, coords: np.ndarray, amps: np.ndarray, h: float = 0.5):
    """Generate field using 3D inverse square law."""
    pixel_to_meter = 10.0 / 256.0
    yy, xx = np.meshgrid(np.arange(grid), np.arange(grid), indexing="ij")
    field = np.zeros((grid, grid), dtype=np.float32)

    for (y0, x0), A in zip(coords, amps, strict=True):
        # 2D distance in meters
        d2_meters = ((yy - y0) * pixel_to_meter) ** 2 + ((xx - x0) * pixel_to_meter) ** 2

        # 3D distance with height
        r2 = d2_meters + h ** 2

        # Inverse square law (simplified, μ ≈ 0)
        field += A / r2

    return field
```

**Pros**:
- Matches paper's physics model
- Consistent with ADE-PSPF prediction
- More realistic for gamma radiation

**Cons**:
- Need to update existing ground truth generation
- May need to adjust intensity ranges

### Option 2: Fix ADE-PSPF to Use Gaussian Model

Modify `predict_intensity()` to use Gaussian decay:

```python
def predict_intensity_gaussian(
    particle_pos: np.ndarray,
    particle_intensity: float,
    particle_sigma: float,  # Add sigma parameter
    observation_pos: np.ndarray,
    other_centroids: List[np.ndarray]
) -> float:
    # Distance in pixels
    dist_pixels = np.linalg.norm(particle_pos - observation_pos[:2])

    # Gaussian decay
    intensity = particle_intensity * np.exp(-dist_pixels**2 / (2 * particle_sigma**2))

    # Add contributions from other sources...
    for centroid in other_centroids:
        centroid_pos = centroid[:2]
        centroid_intensity = centroid[2]
        centroid_sigma = centroid[3]  # Need to add sigma to centroids

        dist = np.linalg.norm(centroid_pos - observation_pos[:2])
        intensity += centroid_intensity * np.exp(-dist**2 / (2 * centroid_sigma**2))

    return intensity
```

**Pros**:
- No need to regenerate ground truth
- Keeps existing test data

**Cons**:
- Doesn't match paper's physics
- Need to estimate sigma parameter (add dimension to particles)
- Less physically accurate for radiation

### Option 3: Scale Conversion Layer

Add a conversion between the two models:

```python
def convert_gaussian_to_inverse_square(A_gaussian, sigma_gaussian, h):
    """Convert Gaussian parameters to equivalent inverse square parameters."""
    # Empirically match at some reference distance
    # This is an approximation and won't be perfect
    A_inv = A_gaussian * (h**2 + (sigma_gaussian / 2)**2)
    return A_inv
```

**Pros**:
- Quick fix
- Can keep existing code mostly unchanged

**Cons**:
- Not physically accurate
- Will still have model mismatch issues
- RFC may not reach 0.85 threshold

---

## Recommendation

**Use Option 1**: Fix ground truth to use inverse square law.

**Reasons**:
1. **Paper compliance**: The paper explicitly describes inverse square law for gamma radiation
2. **Physical accuracy**: Inverse square law is correct for point source radiation
3. **Algorithm design**: ADE-PSPF was designed for this model
4. **Long-term maintainability**: Having consistent physics throughout

**Implementation Plan**:
1. Update `generate_truth.py` to use inverse square law
2. Adjust intensity ranges if needed (may need higher values)
3. Regenerate all test data
4. Verify RFC convergence
5. Update documentation

---

## Verification

After fixing, we should see:
- RFC > 0.01 after first iteration
- RFC steadily increasing (0.3 → 0.5 → 0.7 → 0.85)
- Predicted intensities within 10-50% of observed
- Robot exploring towards high-radiation areas
- Convergence to RFC ≥ 0.85 within 20-50 iterations

---

## Testing Script

Run this after fixing:

```bash
python3 tests/debug_rfc_calculation.py
```

Expected output:
```
6. PREDICTION ACCURACY CHECK:
   Obs @ (128.0, 128.0): true=0.45, pred=0.52, error=0.07 (15.6%)
   Obs @ (130.0, 140.0): true=1.23, pred=1.18, error=0.05 (4.1%)
   ...
   Average error: 0.08

RFC: 0.723  ← Should be > 0.01
```

---

## References

- Paper: "A study of robotic search strategy for multi-radiation sources in unknown environments"
- Equation 1: Inverse square law for radiation intensity
- Equation 7: RFC calculation
- File: `core/weights.py:54-100` (predict_intensity function)
- File: `environment/generate_truth.py:56-64` (gaussian_field function)
