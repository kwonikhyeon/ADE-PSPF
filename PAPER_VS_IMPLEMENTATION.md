# 논문 vs 구현 차이점 분석

## 📋 전체 요약

현재 구현은 논문의 **핵심 알고리즘은 대부분 정확하게 구현**되어 있으나, 몇 가지 **중요한 차이점과 누락된 부분**이 있습니다.

---

## ✅ 정확하게 구현된 부분

### 1. **ADE-PSPF 추정 알고리즘** (Section 3)

#### Synthesized Weight 계산 (Eq. 3)
**논문 (Page 4, Eq. 3)**:
```
w_syn = w_obs × w_ps × w_dist
```

**구현** (`core/weights.py`):
```python
def synthesized_weight(
    observations: List[Tuple[np.ndarray, float]],
    particle: Particle,
    other_centroids: List[np.ndarray],
    config: ADEPSPFConfig
) -> float:
    w_obs = observation_weight(observations, particle, other_centroids, config)
    w_dist = distance_correction(particle, other_centroids, config)
    w_ps = swarm_correction(particle, config)
    return w_obs * w_dist * w_ps
```
✅ **정확히 일치**

#### ADE Mutation (Eq. 4)
**논문 (Page 4, Eq. 4)**:
```
v_s,r(g) = p_s,r(g) + F1·(p_best_s(g) - p_s,r(g)) + F2·(p_r1_s(g) - p_r2_s(g))
where F1 = α·(1-w_s,r), F2 = β·(w_s,r1 - w_s,r̄)/w_s,r1
```

**구현** (`core/ade.py`):
```python
def ade_mutation(
    particles: List[Particle],
    weights: np.ndarray,
    best_idx: int,
    config: ADEPSPFConfig,
    rng: np.random.Generator
) -> np.ndarray:
    # F1 = alpha * (1 - w_i)
    F1 = config.alpha_ade * (1.0 - weights[i])

    # F2 = beta * (w_r1 - w_mean) / w_r1
    F2 = config.beta_ade * (weights[r1] - w_mean) / (weights[r1] + 1e-10)

    mutants[i] = (
        particles[i].state
        + F1 * (particles[best_idx].state - particles[i].state)
        + F2 * (particles[r1].state - particles[r2].state)
    )
```
✅ **정확히 일치**

#### RFC 계산 (Eq. 7)
**논문 (Page 4, Eq. 7)**:
```
F = (1/N_o) × Σ[fp(M_i|I'_cum) / fp(⌊I'_cum⌋|I'_cum)]
```

**구현** (`core/rfc.py`):
```python
def compute_rfc(
    observations: List[Tuple[np.ndarray, float]],
    centroids: List[np.ndarray],
    config: ADEPSPFConfig
) -> float:
    total_ratio = 0.0
    for obs_pos, obs_intensity in observations:
        predicted = predict_intensity(obs_pos, centroids, config)

        # Poisson probability ratio
        ratio = (
            poisson_pmf(obs_intensity, predicted) /
            poisson_pmf(np.floor(predicted), predicted)
        )
        total_ratio += ratio

    return total_ratio / len(observations)
```
✅ **정확히 일치**

---

### 2. **RRT 경로 생성** (Section 4.1)

#### Uniform Initialization (Eq. 8)
**논문 (Page 5, Eq. 8)**:
```
T'_init = ∅ ∪ ψ, where (ψ_h ∩ C_free) ∈ ψ
```

**구현** (`simulation/exploration/rrt_planner.py:172-190`):
```python
def _add_uniform_nodes(self):
    """Add uniform nodes in circumferential directions."""
    for i in range(self.config.n_uniform):
        angle = 2 * np.pi * i / self.config.n_uniform
        dx = self.config.min_step * np.cos(angle)
        dy = self.config.min_step * np.sin(angle)

        new_x = self.robot_pose.x + dx
        new_y = self.robot_pose.y + dy
        new_theta = angle

        new_pose = Pose2D(new_x, new_y, new_theta)
        if self.env.is_collision_free(self.robot_pose, new_pose):
            # Add node
```
✅ **정확히 일치**

#### Radiation Gain Model (Eq. 11-12)
**논문 (Page 5, Eq. 11-12)**:
```
G̃_src(n_t) = σ_dist^gain + (1-σ_dist^gain)/√(2πr_src) × exp(-(d_src^(t) - d_src)²/(2r_src²))

Multi-source: G̃_src(n_t) = Σ G̃_src^(t,k)(n_t)
```

**구현** (`simulation/exploration/rrt_planner.py:90-110`):
```python
def compute_radiation_gain(
    self, node_pos: np.ndarray, sources: List[np.ndarray]
) -> float:
    total_gain = 0.0
    for source in sources:
        distance = np.linalg.norm(node_pos - source[:2])

        # Gaussian gain around optimal distance
        gain = (
            self.params.sigma_gain_dist +
            (1.0 - self.params.sigma_gain_dist) / np.sqrt(2 * np.pi * self.params.r_src) *
            np.exp(-((distance - self.params.d_src)**2) / (2 * self.params.r_src**2))
        )
        total_gain += gain

    return total_gain
```
✅ **정확히 일치**

---

## ⚠️ 부분적으로 다른 부분

### 1. **Gain Correction 미구현**

#### 논문 요구사항 (Section 4.2.3, Page 7)

**Observation Intensity Correction (OIC)** - Eq. 19:
```
C_obs(n_t) = σ_src^gain + (1-σ_src^gain)/(1+exp[(T_src^gain - I_t^near/I_src^back)/S_src^gain])
```

**Redundant Sampling Correction (RSC)** - Eq. 20:
```
C_rs(n_t) = 1 + (σ_rs-1)×exp(N_sam^(t) - S_rs×d_near^(t))/(exp(...) + exp(...))
```

**Repeat Exploring Correction (REC)** - Eq. 21:
```
C_rex(n_t) = Π[exp(-1/((d^(t,b)/S_rex) + σ_rex))]
```

**최종 Gain** - Eq. 22:
```
Gain_cum(n_t) = Gain_cum(n_t-1) + Gain_src(n_t) × C_dist × C_rot × C_obs × C_rs × C_rex
```

#### 현재 구현 (`simulation/exploration/rrt_planner.py:317-345`)

**구현된 것**:
- ✅ Distance cost (C_dist) - Eq. 17
- ✅ Rotation cost (C_rot) - Eq. 18
- ✅ Radiation gain (Gain_src)

**누락된 것**:
- ❌ Observation Intensity Correction (C_obs)
- ❌ Redundant Sampling Correction (C_rs)
- ❌ Repeat Exploring Correction (C_rex)

```python
# 현재 구현 - Eq. 17, 18만
def _calculate_cumulative_gain(self, nodes: List[RRTNode]) -> None:
    for i in range(1, len(nodes)):
        node = nodes[i]
        parent = nodes[node.parent]

        # Radiation gain
        rad_gain = self.gain_model.compute_radiation_gain(...)

        # Distance cost (Eq. 17)
        dist_cost = np.exp(-self.gain_model.params.eta_gain_src * path_distance)

        # Rotation cost (Eq. 18)
        rot_cost = np.exp(theta**2 / sigma_theta**2)

        # ❌ Missing: C_obs, C_rs, C_rex
        node.cumulative_gain = parent.cumulative_gain + rad_gain * dist_cost * rot_cost
```

**영향**:
- 로봇이 이미 방문한 영역을 반복 탐색할 수 있음
- 잘못된 예측(pseudo-source) 주변을 계속 탐색할 수 있음
- 탐색 효율성 감소

---

### 2. **Configuration Maintenance 미구현**

#### 논문 요구사항 (Section 3.4, Page 4)

**논문 설명**:
> "The configuration maintenance will start when the current RFC is lower than the historical best confidence. It utilizes the particle swarms, which have the highest confidence, as the basis for recovery."

**의미**:
- RFC가 이전 최고값보다 낮아지면 이전 최고 configuration으로 복원
- Multi-modal balance가 깨졌을 때 최선의 상태로 복귀

#### 현재 구현

**구현된 것** (`core/rfc.py:110-150`):
```python
class ConfigurationManager:
    def update(self, config: Configuration, rfc: float) -> Configuration:
        if config.is_valid():
            self.history.append((config, rfc))

            # Update best configuration
            if rfc > self.best_rfc:
                self.best_config = config.copy()
                self.best_rfc = rfc

        return config  # ❌ 항상 현재 config 반환
```

**누락된 로직**:
```python
# 논문에 따르면:
def update(self, config: Configuration, rfc: float) -> Configuration:
    if rfc < self.best_rfc:
        # RFC가 감소하면 best configuration으로 복원
        return self.best_config.copy()
    else:
        # RFC가 증가하면 현재 config 유지
        if rfc > self.best_rfc:
            self.best_config = config.copy()
            self.best_rfc = rfc
        return config
```

**영향**:
- 추정이 잘못된 방향으로 발산할 수 있음
- 논문 Fig. 15, 17에서 보여주는 안정적인 RFC 증가 패턴이 나타나지 않을 수 있음

---

### 3. **Branch Execution Ratio 차이**

#### 논문 (Section 4.1, Page 5)

**논문 언급**:
> "the robot only executes the first edge of the branch whose leaf node has the highest gain"

**의미**: 로봇은 최고 gain 브랜치의 **첫 번째 edge만** 실행

#### 현재 구현 (`simulation/integrated_explorer_v2.py:516-528`)

```python
# Calculate how many nodes to execute
n_nodes = max(1, int(len(best_branch) * self.config.branch_execution_ratio))
n_nodes = min(n_nodes, len(best_branch) - 1)

# Default: branch_execution_ratio = 0.8 (80% 실행)
```

**차이점**:
- 논문: 첫 edge만 실행 → 매 iteration마다 re-planning
- 구현: 80% 실행 → 더 긴 경로 실행 후 re-planning

**영향**:
- 구현이 더 효율적일 수 있음 (적은 re-planning)
- 하지만 환경 변화에 덜 반응적

**논문 방식으로 변경하려면**:
```python
config = ExplorationConfig(
    branch_execution_ratio=0.0,  # First edge only
    # OR
    observations_per_iteration=1  # One observation per iteration
)
```

---

### 4. **Swarm Number Adjustment 미구현**

#### 논문 (Section 5.1, Page 9, Fig. 11)

**논문 설명**:
> "When the number of swarms is insufficient, the predicted source number is wrong. However, when the number of swarms exceeded the number of sources, ADE-PSPF quickly found more accurate parameters."

**논문 실험** (Fig. 11):
- Steps 1-10: 3 swarms 사용 → RFC 낮음
- Step 11: **Swarm 수를 5개로 증가**
- Steps 11-20: RFC 급증, LE/IE 감소

**논문 권장**:
> "Therefore, when the robot searches for sources, the proposed method can adjust the number of swarms based on RFC."

#### 현재 구현

**고정된 swarm 수** (`simulation/integrated_explorer_v2.py:752-757`):
```python
adepspf_config = ADEPSPFConfig(
    n_swarms=4,  # ❌ 고정값
    n_particles=100,
    # ...
)
```

**누락된 로직**:
```python
# 논문에 따르면:
if rfc < threshold and iterations > min_iterations:
    # Swarm 수 증가
    n_swarms += 1
    estimator.add_swarm()  # Not implemented
```

**영향**:
- Source 개수가 예상보다 많을 때 정확도 감소
- 논문 Fig. 11과 같은 동적 개선 불가능

---

## ❌ 완전히 누락된 부분

### 1. **Superposition Suppression (Eq. 13-15)**

#### 논문 (Section 4.2.1, Page 6)

**문제점**:
> "When multiple point sources are concentrated in a local area, the superposition of gains will lead to abnormalities in the common neighborhood" (Fig. 4)

**해결책** - Suppression Factor (Eq. 13):
```
F_src^(t,k)(n_t) = Σ[1 - C_sup^dist + C_sup^dist/(1+exp[(T_sup^dist - d_t^(k,j))/S_sup^dist])]

where d_t^(k,j) = ||n_t - (C_k^pos + C_j^pos)/2||
```

**Suppression Range Coefficient** (Eq. 14):
```
C_sup^dist = σ_r^dist + (1-σ_r^dist)/(1+exp[(d_r^(k,j) - T_r^dist)/S_r^dist])
```

**최종 Multi-source Gain** (Eq. 15):
```
G_src(n_t) = Σ G̃_src^(t,k)(n_t) × F_src^(t,k)(n_t)
```

#### 현재 구현

```python
# simulation/exploration/rrt_planner.py:90-110
def compute_radiation_gain(self, node_pos, sources):
    total_gain = 0.0
    for source in sources:
        # Single source gain calculation
        gain = ...
        total_gain += gain  # ❌ Simple summation, no suppression

    return total_gain
```

**영향**:
- 가까운 여러 소스가 있을 때 abnormal gain 발생 (Fig. 4)
- Common neighborhood에서 올바른 surrounding observation 불가 (Fig. 5)
- 논문 Fig. 14(g)처럼 모든 소스 주변을 탐색하지 못할 수 있음

---

### 2. **Exploration Phases 명확한 구분 부재**

#### 논문 (Section 5.2, Page 10-11)

**논문의 3단계 탐색**:

1. **Tracing Suspicious Sources** (iterations 1-36, Fig. 14a-c)
   - 예측된 소스 추적
   - OIC로 pseudo-source의 gain 감소

2. **Surrounding Observation** (iterations 37-63, Fig. 14d-g)
   - 소스 주변 관측
   - REC로 위치 다양성 보장

3. **Exploring Unknown Areas** (iterations 64-83, Fig. 14h-i)
   - 미탐색 영역 탐색
   - RSC와 REC로 중복 탐색 방지

#### 현재 구현

```python
# 단순히 max_iterations만 실행
for iteration in range(config.max_iterations):
    # Estimation
    # Exploration (RRT)
    # Execution
    # ❌ No explicit phase management
```

**누락**:
- Phase 전환 로직 없음
- 각 phase별 파라미터 조정 없음
- 논문 Fig. 14처럼 체계적인 3단계 탐색 불가능

---

### 3. **Termination Criteria 단순화**

#### 논문 (Section 5.2.1, Page 10)

**논문 종료 조건**:
> "the robot continues to search until the RFC is not improved in **ten consecutive iterations**"

#### 현재 구현 (`simulation/integrated_explorer_v2.py:290-295`)

```python
if rfc >= self.config.min_rfc_threshold:
    print(f"✓ Converged! RFC={rfc:.4f} >= {self.config.min_rfc_threshold}")
    break
```

**차이점**:
- 논문: 10회 연속 RFC 개선 없으면 종료
- 구현: RFC ≥ 0.85면 즉시 종료

**누락된 로직**:
```python
no_improvement_count = 0
for iteration in range(max_iterations):
    if rfc <= prev_rfc:
        no_improvement_count += 1
    else:
        no_improvement_count = 0

    if no_improvement_count >= 10:
        break  # Stop after 10 iterations without improvement
```

**영향**:
- RFC가 0.85 도달 후에도 개선될 수 있는데 조기 종료
- 논문 Fig. 15, 17처럼 90%+ RFC 도달 불가능

---

## 📊 구현 vs 논문 성능 비교

### 논문 성능 (Table 6, Page 14)

| Metric | Proposed Strategy | BP | NBVP |
|--------|------------------|-----|------|
| **3-source scenario** |
| SR | 85% | 80% | 65% |
| RFC | 0.95 ± 0.01 | 0.86 ± 0.03 | 0.77 ± 0.11 |
| LE | **0.25 ± 0.05 m** | 1.19 ± 0.37 m | 1.32 ± 0.61 m |
| IE | 124.72 ± 47.41 | 129.96 ± 37.64 | 104.82 ± 18.86 |
| **4-source scenario** |
| SR | 70% | 45% | 50% |
| RFC | 0.88 ± 0.05 | 0.72 ± 0.05 | 0.79 ± 0.11 |
| LE | **0.43 ± 0.25 m** | 1.20 ± 0.24 m | 0.92 ± 0.62 m |
| IE | 77.68 ± 47.01 | 97.15 ± 33.77 | 145.42 ± 1.01 |

### 현재 구현 성능 (추정)

**Quick Test 결과** (`tests/test_integrated_quick.py`):
```
총 반복: 1 iteration
최종 RFC: 1.0000
총 관측: 4
이동 거리: 0.0px (0.00m)
```

**문제점**:
- RFC=1.0이지만 관측 4개만으로 달성 → **Convergence Paradox**
- 논문처럼 83 iterations, RFC=0.94가 아님

**예상 성능 차이 원인**:
1. ❌ Gain corrections (OIC, RSC, REC) 누락 → 비효율적 탐색
2. ❌ Superposition suppression 누락 → Common neighborhood 탐색 실패
3. ❌ Configuration maintenance 누락 → 불안정한 추정

---

## 🔧 수정 우선순위

### 🔴 High Priority (핵심 알고리즘)

#### 1. **Gain Corrections 구현** (Eq. 19-22)
**이유**: 논문 성능의 핵심, Fig. 7 전후 성능 차이 큼

**구현 위치**: `simulation/exploration/rrt_planner.py`

**예상 코드**:
```python
class GainCorrections:
    def observation_intensity_correction(self, node, observations):
        """Eq. 19: OIC"""
        I_near = self._find_nearest_observation_intensity(node)
        C_obs = (
            self.sigma_gain_src +
            (1 - self.sigma_gain_src) /
            (1 + np.exp((self.T_gain_src - I_near / self.I_back_src) / self.S_gain_src))
        )
        return C_obs

    def redundant_sampling_correction(self, node, sources, observations):
        """Eq. 20: RSC"""
        d_near = self._distance_to_nearest_source(node, sources)
        N_sam = self._count_nearby_observations(node, observations)

        C_rs = 1 + (
            (self.sigma_rs - 1) * np.exp(N_sam - self.S_rs * d_near) /
            (np.exp(self.S_rs * d_near - N_sam) + np.exp(N_sam - self.S_rs * d_near))
        )
        return C_rs

    def repeat_exploring_correction(self, node, observations):
        """Eq. 21: REC"""
        C_rex = 1.0
        for obs_pos, _ in observations:
            d_tb = np.linalg.norm(node.pose.position - obs_pos)
            C_rex *= np.exp(-1.0 / ((d_tb / self.S_rex) + self.sigma_rex))
        return C_rex

# In RRTPlanner._calculate_cumulative_gain():
def _calculate_cumulative_gain(self, nodes, sources, observations):
    corrections = GainCorrections(self.gain_model.params)

    for node in nodes[1:]:
        C_obs = corrections.observation_intensity_correction(node, observations)
        C_rs = corrections.redundant_sampling_correction(node, sources, observations)
        C_rex = corrections.repeat_exploring_correction(node, observations)

        node.cumulative_gain = (
            parent.cumulative_gain +
            rad_gain * dist_cost * rot_cost *
            C_obs * C_rs * C_rex  # Eq. 22
        )
```

**예상 효과**:
- 탐색 효율 **2-3배** 향상 (논문 Fig. 8)
- RFC 수렴 속도 증가
- Pseudo-source 탐색 방지

---

#### 2. **Superposition Suppression 구현** (Eq. 13-15)
**이유**: Multi-source scenario에서 필수, Fig. 4-5 차이

**구현 위치**: `simulation/exploration/rrt_planner.py`

**예상 코드**:
```python
def compute_radiation_gain_with_suppression(
    self, node_pos, sources
) -> float:
    """Eq. 15: Multi-source gain with suppression."""
    if len(sources) <= 1:
        # Single source: no suppression needed
        return self.compute_radiation_gain(node_pos, sources)

    total_gain = 0.0
    for k, source_k in enumerate(sources):
        # Single source gain (Eq. 11)
        gain_k = self._single_source_gain(node_pos, source_k)

        # Suppression factor (Eq. 13)
        F_k = self._suppression_factor(node_pos, source_k, sources, k)

        total_gain += gain_k * F_k  # Eq. 15

    return total_gain

def _suppression_factor(self, node_pos, source_k, sources, k):
    """Eq. 13: Suppression factor for kth source."""
    F_k = 0.0
    for j, source_j in enumerate(sources):
        if j == k:
            continue

        # Distance from node to midpoint of two sources
        midpoint = (source_k[:2] + source_j[:2]) / 2
        d_kj = np.linalg.norm(node_pos - midpoint)

        # Suppression range coefficient (Eq. 14)
        d_r_kj = np.linalg.norm(source_k[:2] - source_j[:2])
        C_dist_sup = (
            self.params.sigma_dist_r +
            (1 - self.params.sigma_dist_r) /
            (1 + np.exp((d_r_kj - self.params.T_dist_r) / self.params.S_dist_r))
        )

        # Suppression factor
        F_k += (
            1 - C_dist_sup +
            C_dist_sup / (1 + np.exp((self.params.T_dist_sup - d_kj) / self.params.S_dist_sup))
        )

    return F_k
```

**예상 효과**:
- Common neighborhood abnormal gain 제거 (Fig. 4 → Fig. 5)
- 모든 소스 주변 surrounding observation 가능
- Multi-source accuracy 향상

---

#### 3. **Configuration Maintenance 구현** (Section 3.4)
**이유**: 추정 안정성 보장, RFC 발산 방지

**구현 위치**: `core/rfc.py`

**예상 코드**:
```python
class ConfigurationManager:
    def update(self, config: Configuration, rfc: float) -> Configuration:
        """
        Update configuration with maintenance.

        Returns best configuration if RFC decreases, otherwise current.
        """
        if not config.is_valid():
            # Invalid config: restore best
            return self.best_config.copy() if self.best_config else config

        # Add to history
        self.history.append((config, rfc))

        # Configuration maintenance
        if rfc < self.best_rfc:
            # RFC decreased: restore best configuration
            print(f"  ⚠ Configuration maintenance: "
                  f"RFC {rfc:.4f} < best {self.best_rfc:.4f}, restoring...")
            return self.best_config.copy()

        # RFC improved: update best
        if rfc > self.best_rfc:
            self.best_config = config.copy()
            self.best_rfc = rfc

        return config
```

**예상 효과**:
- 안정적인 RFC 증가 (논문 Fig. 15, 17)
- Multi-modal balance 유지
- 추정 발산 방지

---

### 🟡 Medium Priority (탐색 효율)

#### 4. **Termination Criteria 개선**
```python
def _check_termination(self, rfc: float, iteration: int) -> bool:
    """Check if search should terminate (논문 방식)."""
    # Update no-improvement counter
    if rfc <= self.prev_rfc:
        self.no_improvement_count += 1
    else:
        self.no_improvement_count = 0

    self.prev_rfc = rfc

    # Terminate if no improvement for 10 iterations
    if self.no_improvement_count >= 10:
        return True

    # Terminate if max iterations reached
    if iteration >= self.config.max_iterations:
        return True

    return False
```

#### 5. **Dynamic Swarm Number Adjustment**
```python
def _adjust_swarm_number(self, rfc: float, n_sources_estimated: int):
    """Adjust swarm number based on RFC (논문 Fig. 11)."""
    if rfc < 0.85 and n_sources_estimated < len(self.estimator.swarms):
        # Add swarm if estimation struggling
        self.estimator.add_swarm()
        print(f"  → Increased swarms to {len(self.estimator.swarms)}")
```

---

### 🟢 Low Priority (일치성)

#### 6. **Branch Execution Ratio → First Edge Only**
```python
config = ExplorationConfig(
    branch_execution_ratio=0.0,  # Execute first edge only (논문 방식)
    observations_per_iteration=1  # One observation per iteration
)
```

#### 7. **Exploration Phases 명시적 관리**
```python
class ExplorationPhase(Enum):
    TRACING = 1      # Tracing suspicious sources
    SURROUNDING = 2  # Surrounding observation
    EXPLORING = 3    # Exploring unknown areas

def _determine_phase(self, iteration, rfc, coverage):
    """Determine current exploration phase."""
    if rfc < 0.3:
        return ExplorationPhase.TRACING
    elif rfc < 0.85:
        return ExplorationPhase.SURROUNDING
    else:
        return ExplorationPhase.EXPLORING
```

---

## 📈 예상 개선 효과

### 수정 전 (현재)
```
3-source scenario (추정):
- SR: ~60%
- RFC: 0.85 ± 0.10
- LE: 1.0 ± 0.5 m
- Iterations: ~20
```

### 수정 후 (논문 수준)
```
3-source scenario (목표):
- SR: 85%  (+25%p)
- RFC: 0.95 ± 0.01  (+0.10)
- LE: 0.25 ± 0.05 m  (-75%)
- Iterations: ~80 (더 철저한 탐색)
```

---

## 🎯 결론

### ✅ 잘 구현된 것
1. ADE-PSPF 핵심 알고리즘 (Eq. 3-7)
2. RRT 기본 구조 (Eq. 8-12)
3. RFC 계산 (Eq. 7)
4. Synthesized weight (Eq. 3)

### ❌ 누락/다른 것
1. **Gain corrections** (OIC, RSC, REC) - Eq. 19-22 ← **가장 중요**
2. **Superposition suppression** - Eq. 13-15 ← **Multi-source 필수**
3. **Configuration maintenance** - Section 3.4 ← **안정성**
4. **Termination criteria** - 10 consecutive iterations
5. **Dynamic swarm adjustment** - Fig. 11
6. **Branch execution** - First edge only vs 80%
7. **Exploration phases** - 3-phase system

### 📝 권장사항

**즉시 구현 필요**:
1. Gain corrections (Eq. 19-22)
2. Superposition suppression (Eq. 13-15)
3. Configuration maintenance

**추후 개선**:
4. Termination criteria
5. Dynamic swarm adjustment
6. Exploration phase management

이 수정을 완료하면 논문과 거의 동일한 성능 (85% SR, 0.95 RFC, 0.25m LE)을 달성할 수 있을 것으로 예상됩니다.
