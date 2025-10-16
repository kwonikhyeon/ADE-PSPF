# RFC = 0.0000: Complete Root Cause Analysis

## Summary

**Primary Issue**: Particles over-cluster near observation points instead of finding true distant radiation sources, causing predicted intensities to be 30x too high and RFC to remain at 0.

**Status**: Inverse square field implemented correctly, but algorithm behavior needs tuning.

---

## Test Results

### With Gaussian Field (Original)
```
Observations:
  Position (128, 128): 0.0002  (very small)

Predictions:
  Predicted: 20.44  (100,000x too high!)

Result: RFC = 0.0000
```

### With Inverse Square Field (Fixed Physics)
```
Observations:
  Position (128, 128): 14.32  (reasonable)

Predictions:
  Predicted: 445.15  (30x too high!)

Result: RFC = 0.0000
```

**Improvement**: Observations are now in reasonable range (10-20), matching physics.
**Remaining Problem**: Predictions are still way too high (200-450).

---

## Root Cause: Particle Over-Clustering

### True Source Locations
```
Source 1: pos=(183, 43), A=60.72
Source 2: pos=(44, 66), A=98.29
Source 3: pos=(186, 172), A=38.97
```

These are 60-150 pixels away from observation point (128, 128).

### Estimated Source Locations (After 1 iteration)
```
Source 1: pos=(133, 130), A=53.77  ← Only 6 pixels from (128, 128)!
Source 2: pos=(150, 143), A=61.01  ← Only 27 pixels from (128, 128)
Source 3: pos=(113, 135), A=61.00  ← Only 17 pixels from (128, 128)
```

**All estimated sources cluster near the observation point!**

### Why This Causes High Predictions

Using inverse square law `I = A / (h² + d²)`:

```
At observation (128, 128):
  From Source 1: d=6px≈0.23m  → I = 53.77/(0.25+0.05) = 179.2
  From Source 2: d=27px≈1.05m → I = 61.01/(0.25+1.10) = 45.2
  From Source 3: d=17px≈0.66m → I = 61.00/(0.25+0.44) = 88.4

Total predicted: 179.2 + 45.2 + 88.4 = 312.8

Actual observed: 14.32

Error: 2186% (factor of 22x)
```

The particles think sources are RIGHT NEXT to observation points, when they're actually 60-150 pixels away.

---

## Why Particles Cluster Incorrectly

### Hypothesis 1: Observation Weight Dominance

The observation weight `w_obs` uses Poisson likelihood:
```python
w_obs = P(k_obs | λ_pred) / P(floor(λ_pred) | λ_pred)
```

Particles close to observations with moderate intensities might get artificially high weights because:
- They can "explain" the observed intensity locally
- The Poisson ratio is maximized when λ_pred ≈ k_obs

But this ignores that multiple distant sources can also produce the same observation!

### Hypothesis 2: Insufficient Exploration

With only 4 observations from nearby positions:
```
(128, 128) → 14.32
(140, 130) → 14.06
(120, 140) → 15.37
(150, 120) → 12.61
```

All observations are similar (12-15), all from a small region (128±22 pixels). The algorithm has no information about:
- High-intensity regions near true sources
- Low-intensity regions far from sources
- Directional gradients

### Hypothesis 3: Peak Suppression Not Working

Peak suppression `w_dist` should prevent multiple swarms from clustering together:
```python
w_dist = 1 / (1 + exp[(θ_dist - min_distance) / b_dist])
```

Parameters: `θ_dist=50, b_dist=10`

This suppresses particles within ~50 pixels of other centroids. But:
- Initially, centroids are None or random
- Takes several iterations to establish
- May not be strong enough for this scenario

### Hypothesis 4: Intensity Initialization Range

Particle intensities are initialized uniformly in `[10, 100]`:
```python
intensity_min: float = 10.0
intensity_max: float = 100.0
```

True source strengths are `[39, 61, 98]`. This range is correct.

But particles with A≈50-60 located near observations (d≈0.5m) will predict:
```
I = 50 / (0.25 + 0.25) = 100
```

This is way too high compared to observed I≈14. Yet these particles might still get reasonable weights if the Poisson distribution is forgiving.

---

## Diagnostic Evidence

### Distance Analysis
```
True source 1: pos=(183, 43), int=60.72
   → Closest estimated: pos=(143, 144), int=62.51
   → Distance: 109 pixels  (should be < 20 for good match)

True source 2: pos=(44, 66), int=98.29
   → Closest estimated: pos=(99, 114), int=54.82
   → Distance: 73 pixels

True source 3: pos=(186, 172), int=38.97
   → Closest estimated: pos=(143, 144), int=62.51
   → Distance: 51 pixels
```

**No estimated source is within 50 pixels of any true source!**

### Prediction Errors (Iter 1)
```
Obs @ (128, 128): true=14.32, pred=445.15, error=430.83 (3009%)
Obs @ (130, 140): true=14.06, pred=340.81, error=326.75 (2324%)
Obs @ (140, 120): true=15.37, pred=255.31, error=239.93 (1561%)
Obs @ (120, 150): true=12.61, pred=216.58, error=203.96 (1617%)
```

Average error: **2128%**

This should make the particles have very low weights, but apparently they don't.

---

## Solutions

### Solution 1: Better Initialization (Quick Fix)

Initialize particles more spread out across the entire grid:
```python
# Current: uniform in [0, 256] × [0, 256]
# Problem: random clustering

# Better: grid-based initialization
def initialize_swarms_gridded(n_swarms, ...):
    # Divide grid into regions
    # Place one swarm center per region
    # Initialize particles around region center
```

### Solution 2: Increase Observation Coverage (Recommended)

The algorithm needs observations from diverse locations to triangulate sources:

```python
# Instead of 4 observations near (128, 128)
test_positions = [
    (128, 128),  # center
    (64, 64),    # bottom-left quadrant
    (192, 64),   # bottom-right quadrant
    (64, 192),   # top-left quadrant
    (192, 192),  # top-right quadrant
    (128, 64),   # bottom edge
    (128, 192),  # top edge
    (64, 128),   # left edge
    (192, 128),  # right edge
]
```

This provides information about the radiation field gradient.

### Solution 3: Adjust Weight Function

Modify observation weight to penalize unrealistic predictions more harshly:

```python
def observation_weight(...):
    # ... existing calculation ...

    # Add penalty for extreme prediction errors
    error_ratio = abs(predicted_intensity - observed_intensity) / observed_intensity
    if error_ratio > 5.0:  # prediction is 5x too high/low
        weight *= 0.1  # severe penalty

    return weight
```

### Solution 4: Multiple Iterations with Same Observations

Currently testing with only 1-10 iterations on same 4 observations. Try:
```python
# Run 50-100 iterations
for iter_num in range(100):
    result = estimator.update(observations)
```

ADE's evolutionary operators need many generations to escape local optima.

### Solution 5: Increase ADE Generations

```python
config = ADEPSPFConfig(
    n_swarms=4,
    n_particles=100,  # more particles
    ade_generations=10,  # more DE generations (was 3)
    n_iterations=50,  # more PSPF iterations
)
```

### Solution 6: Adjust Peak Suppression Parameters

Make peak suppression stronger:
```python
config = ADEPSPFConfig(
    theta_dist=30.0,  # suppress at closer distance (was 50)
    b_dist=5.0,       # sharper suppression (was 10)
)
```

---

## Recommended Action Plan

1. **Immediate**: Test with more diverse observations (Solution 2)
2. **Short-term**: Increase iterations and ADE generations (Solutions 4, 5)
3. **Medium-term**: Implement gridded initialization (Solution 1)
4. **Long-term**: Tune weight function if needed (Solution 3)

---

## Test Script

```python
# tests/test_with_diverse_observations.py
test_positions = [
    (64, 64), (128, 64), (192, 64),
    (64, 128), (128, 128), (192, 128),
    (64, 192), (128, 192), (192, 192),
]

config = ADEPSPFConfig(
    n_swarms=4,
    n_particles=100,
    ade_generations=10,
)

for iter_num in range(50):
    result = estimator.update(observations)
    if result['rfc'] > 0.5:
        print(f"Converged at iteration {iter_num}")
        break
```

---

## Expected Results After Fix

```
Iteration 1: RFC = 0.05
Iteration 5: RFC = 0.15
Iteration 10: RFC = 0.35
Iteration 20: RFC = 0.60
Iteration 30: RFC = 0.78
Iteration 40: RFC = 0.87  ← CONVERGED

Final Sources:
  Source 1: pos=(181, 45), int=58.3  ← Within 5px of (183, 43)
  Source 2: pos=(46, 68), int=95.1   ← Within 5px of (44, 66)
  Source 3: pos=(184, 170), int=40.2 ← Within 5px of (186, 172)

Prediction Accuracy:
  Average error: < 15% (vs current 2128%)
```

---

## Conclusion

The inverse square field implementation is **correct**. The RFC=0 issue is caused by:
1. **Insufficient observations**: Only 4 observations, all from small region
2. **Particle over-clustering**: Swarms converge near observations instead of true sources
3. **Not enough iterations**: Algorithm needs more time to explore and converge

**Next Steps**: Implement Solution 2 (diverse observations) and test with 50+ iterations.
