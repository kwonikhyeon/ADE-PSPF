# 논문 구현 완성 TODO List

논문 "A study of robotic search strategy for multi-radiation sources in unknown environments"와 완전히 일치하도록 구현을 완성하기 위한 작업 목록입니다.

---

## 🔴 High Priority (핵심 알고리즘)

### 1. ✅ Gain Corrections 구현 (Eq. 19-22) - **가장 중요!**

**목표**: 탐색 전략의 핵심 메커니즘 구현

#### 1.1 Observation Intensity Correction (OIC) - Eq. 19
**목적**: Pseudo-source 탐색 방지 (잘못된 예측 회피)

**구현 위치**: `simulation/exploration/gain_corrections.py` (새 파일)

**공식**:
```
C_obs(n_t) = σ_src^gain + (1-σ_src^gain) / (1 + exp[(T_src^gain - I_t^near/I_src^back) / S_src^gain])
```

**파라미터**:
- `σ_src^gain = 0.1`: OIC 배경값
- `T_src^gain = 2.0`: 임계값 (observed/background ratio)
- `S_src^gain = 0.5`: 스케일 파라미터
- `I_src^back`: 배경 방사선 강도

**구현 내용**:
```python
def observation_intensity_correction(
    node: RRTNode,
    observations: List[Tuple[np.ndarray, float]],
    I_back: float,
    params: GainCorrectionParams
) -> float:
    """
    Eq. 19: Observation Intensity Correction.

    낮은 관측 강도 → C_obs 낮음 → Gain 감소 (pseudo-source 회피)
    높은 관측 강도 → C_obs 높음 → Gain 증가 (실제 source 탐색)
    """
    # Find nearest observation to this node
    node_pos = np.array([node.pose.x, node.pose.y])

    if not observations:
        return 1.0

    # Get closest observation
    distances = [np.linalg.norm(node_pos - obs_pos) for obs_pos, _ in observations]
    nearest_idx = np.argmin(distances)
    _, I_near = observations[nearest_idx]

    # Intensity ratio
    ratio = I_near / (I_back + 1e-10)

    # Sigmoid correction
    C_obs = (
        params.sigma_gain_src +
        (1.0 - params.sigma_gain_src) /
        (1.0 + np.exp((params.T_gain_src - ratio) / params.S_gain_src))
    )

    return C_obs
```

**예상 효과**:
- RFC 40% → 90% 개선 (논문 Fig. 8)
- Pseudo-source 반복 탐색 방지 (논문 Fig. 7a)

---

#### 1.2 Redundant Sampling Correction (RSC) - Eq. 20
**목적**: 특정 소스 과도한 샘플링 방지

**공식**:
```
C_rs(n_t) = 1 + (σ_rs - 1) × exp(N_sam^(t) - S_rs×d_near^(t)) / (exp(S_rs×d_near^(t) - N_sam^(t)) + exp(N_sam^(t) - S_rs×d_near^(t)))
```

**파라미터**:
- `σ_rs = 0.2`: RSC 오프셋
- `S_rs = 0.1`: 스케일 파라미터

**구현 내용**:
```python
def redundant_sampling_correction(
    node: RRTNode,
    sources: List[np.ndarray],
    observations: List[Tuple[np.ndarray, float]],
    params: GainCorrectionParams
) -> float:
    """
    Eq. 20: Redundant Sampling Correction.

    특정 소스 주변에 관측 많음 → C_rs 낮음 → Gain 감소
    미탐색 소스 → C_rs 높음 → Gain 유지
    """
    if not sources:
        return 1.0

    node_pos = np.array([node.pose.x, node.pose.y])

    # Find nearest source
    distances = [np.linalg.norm(node_pos - src[:2]) for src in sources]
    d_near = min(distances)
    nearest_src = sources[np.argmin(distances)]

    # Count observations near this source
    N_sam = 0
    for obs_pos, _ in observations:
        dist_to_src = np.linalg.norm(obs_pos - nearest_src[:2])
        if dist_to_src < 50.0:  # Within 2m radius
            N_sam += 1

    # Correction factor
    exp_pos = np.exp(N_sam - params.S_rs * d_near)
    exp_neg = np.exp(params.S_rs * d_near - N_sam)

    C_rs = 1.0 + (params.sigma_rs - 1.0) * exp_pos / (exp_neg + exp_pos)

    return C_rs
```

**예상 효과**:
- 모든 소스 균형있게 탐색
- RFC 50% → 90% 개선 (논문 Fig. 8)

---

#### 1.3 Repeat Exploring Correction (REC) - Eq. 21
**목적**: 미탐색 영역 탐색 촉진

**공식**:
```
C_rex(n_t) = Π[exp(-1 / ((d^(t,b)/S_rex) + σ_rex))]
```

**파라미터**:
- `S_rex = 20.0`: 스케일 파라미터 (20 pixels = 0.8m)
- `σ_rex = 0.01`: 오프셋

**구현 내용**:
```python
def repeat_exploring_correction(
    node: RRTNode,
    observations: List[Tuple[np.ndarray, float]],
    params: GainCorrectionParams
) -> float:
    """
    Eq. 21: Repeat Exploring Correction.

    방문한 영역 근처 → C_rex 낮음 → Gain 감소
    미방문 영역 → C_rex 높음 → Gain 유지
    """
    if not observations:
        return 1.0

    node_pos = np.array([node.pose.x, node.pose.y])

    C_rex = 1.0
    for obs_pos, _ in observations:
        d_tb = np.linalg.norm(node_pos - obs_pos)

        # Product of corrections from all observations
        C_rex *= np.exp(-1.0 / ((d_tb / params.S_rex) + params.sigma_rex))

    return C_rex
```

**예상 효과**:
- 전체 영역 커버리지 증가 (20% → 80%)
- RFC 65% → 90% 개선 (논문 Fig. 8)

---

#### 1.4 RRT Gain 계산에 통합 - Eq. 22
**목적**: 모든 correction을 cumulative gain에 적용

**공식**:
```
Gain_cum(n_t) = Gain_cum(n_t-1) + Gain_src(n_t) × C_dist × C_rot × C_obs × C_rs × C_rex
```

**구현 위치**: `simulation/exploration/rrt_planner.py:_calculate_cumulative_gain()`

**수정 내용**:
```python
def _calculate_cumulative_gain(
    self,
    nodes: List[RRTNode],
    sources: List[np.ndarray],
    observations: List[Tuple[np.ndarray, float]]
) -> None:
    """Calculate cumulative gain with all corrections (Eq. 22)."""
    from .gain_corrections import GainCorrections

    corrections = GainCorrections(self.gain_model.params)
    I_back = 0.0  # Background radiation (from config)

    for i in range(1, len(nodes)):
        node = nodes[i]
        parent = nodes[node.parent]

        # 1. Radiation gain (Eq. 11-12)
        rad_gain = self.gain_model.compute_radiation_gain(...)

        # 2. Distance cost (Eq. 17)
        dist_cost = np.exp(-eta * path_distance)

        # 3. Rotation cost (Eq. 18)
        rot_cost = np.exp(theta**2 / sigma_theta**2)

        # 4. Observation Intensity Correction (Eq. 19)
        C_obs = corrections.observation_intensity_correction(
            node, observations, I_back
        )

        # 5. Redundant Sampling Correction (Eq. 20)
        C_rs = corrections.redundant_sampling_correction(
            node, sources, observations
        )

        # 6. Repeat Exploring Correction (Eq. 21)
        C_rex = corrections.repeat_exploring_correction(
            node, observations
        )

        # 7. Cumulative gain (Eq. 22)
        node.cumulative_gain = (
            parent.cumulative_gain +
            rad_gain * dist_cost * rot_cost *
            C_obs * C_rs * C_rex
        )
```

---

#### 1.5 단독 테스트 작성
**파일**: `tests/test_gain_corrections.py`

**테스트 항목**:
- OIC: Pseudo-source vs Real source gain 비교
- RSC: 과도 샘플링된 소스 vs 미탐색 소스 gain 비교
- REC: 방문한 영역 vs 미방문 영역 gain 비교
- 통합: Eq. 22 cumulative gain 계산 검증

---

## 🔴 High Priority (Multi-source)

### 2. ✅ Superposition Suppression 구현 (Eq. 13-15)

**목표**: 가까운 소스들의 common neighborhood abnormal gain 제거

#### 2.1 Suppression Factor - Eq. 13
**목적**: 두 소스의 중간 지점 근처에서 gain 억제

**공식**:
```
F_src^(t,k)(n_t) = Σ[1 - C_sup^dist + C_sup^dist / (1 + exp[(T_sup^dist - d_t^(k,j)) / S_sup^dist])]

where d_t^(k,j) = ||n_t - (C_k^pos + C_j^pos)/2||
```

**파라미터**:
- `T_sup^dist = 50.0`: 억제 시작 거리 (pixels)
- `S_sup^dist = 10.0`: 억제 스케일

**구현 위치**: `simulation/exploration/radiation_gain.py`

```python
def _suppression_factor(
    self,
    node_pos: np.ndarray,
    source_k: np.ndarray,
    sources: List[np.ndarray],
    k: int
) -> float:
    """
    Eq. 13: Suppression factor for kth source.

    두 소스가 가까우면 중간 지점 근처의 gain 억제
    """
    F_k = 0.0

    for j, source_j in enumerate(sources):
        if j == k:
            continue

        # Distance from node to midpoint of two sources
        midpoint = (source_k[:2] + source_j[:2]) / 2.0
        d_kj_t = np.linalg.norm(node_pos - midpoint)

        # Distance between two sources
        d_r_kj = np.linalg.norm(source_k[:2] - source_j[:2])

        # Suppression range coefficient (Eq. 14)
        C_dist_sup = self._suppression_range_coefficient(d_r_kj)

        # Suppression factor
        F_k += (
            1.0 - C_dist_sup +
            C_dist_sup / (1.0 + np.exp(
                (self.params.T_dist_sup - d_kj_t) / self.params.S_dist_sup
            ))
        )

    return F_k
```

---

#### 2.2 Suppression Range Coefficient - Eq. 14

**공식**:
```
C_sup^dist = σ_r^dist + (1 - σ_r^dist) / (1 + exp[(d_r^(k,j) - T_r^dist) / S_r^dist])
```

**파라미터**:
- `σ_r^dist = 0.1`: 배경값
- `T_r^dist = 100.0`: 소스 간 거리 임계값 (pixels)
- `S_r^dist = 20.0`: 스케일

```python
def _suppression_range_coefficient(self, d_r_kj: float) -> float:
    """
    Eq. 14: Suppression range coefficient.

    소스 간 거리가 가까우면 억제 강도 증가
    """
    C_dist_sup = (
        self.params.sigma_dist_r +
        (1.0 - self.params.sigma_dist_r) /
        (1.0 + np.exp((d_r_kj - self.params.T_dist_r) / self.params.S_dist_r))
    )
    return C_dist_sup
```

---

#### 2.3 Multi-source Gain with Suppression - Eq. 15

**공식**:
```
G_src(n_t) = Σ G̃_src^(t,k)(n_t) × F_src^(t,k)(n_t)
```

**구현 위치**: `simulation/exploration/radiation_gain.py`

```python
def compute_radiation_gain_with_suppression(
    self,
    node_pos: np.ndarray,
    sources: List[np.ndarray]
) -> float:
    """
    Eq. 15: Multi-source radiation gain with superposition suppression.

    Fig. 4 (abnormal gain) → Fig. 5 (suppressed gain)
    """
    if len(sources) <= 1:
        # Single source: no suppression needed
        return self.compute_radiation_gain(node_pos, sources)

    total_gain = 0.0

    for k, source_k in enumerate(sources):
        # Single source gain (Eq. 11)
        gain_k = self._single_source_gain(node_pos, source_k)

        # Suppression factor (Eq. 13)
        F_k = self._suppression_factor(node_pos, source_k, sources, k)

        # Eq. 15
        total_gain += gain_k * F_k

    return total_gain
```

---

#### 2.4 단독 테스트
**파일**: `tests/test_superposition_suppression.py`

**테스트 시나리오**:
- 2개 소스 (거리 50px): Common neighborhood gain 비교
- 2개 소스 (거리 100px): Suppression 강도 확인
- 3개 소스: Fig. 5 재현

---

## 🔴 High Priority (안정성)

### 3. ✅ Configuration Maintenance 구현 (Section 3.4)

**목표**: RFC 감소 시 best configuration 복원

**구현 위치**: `core/rfc.py:ConfigurationManager`

#### 3.1 RFC 감소 감지 및 복원

**논문 설명**:
> "The configuration maintenance will start when the current RFC is lower than the historical best confidence."

```python
class ConfigurationManager:
    def __init__(self):
        self.best_config: Optional[Configuration] = None
        self.best_rfc: float = 0.0
        self.history: List[Tuple[Configuration, float]] = []
        self.maintenance_count: int = 0  # For logging

    def update(self, config: Configuration, rfc: float) -> Configuration:
        """
        Update configuration with maintenance.

        Returns:
            Best configuration if RFC decreased, otherwise current.
        """
        if not config.is_valid():
            # Invalid config: restore best if available
            if self.best_config:
                self.maintenance_count += 1
                return self.best_config.copy()
            return config

        # Add to history
        self.history.append((config, rfc))

        # Configuration maintenance
        if rfc < self.best_rfc - 0.01:  # Tolerance for noise
            # RFC decreased: restore best configuration
            if self.best_config:
                print(f"  ⚠ Configuration maintenance triggered:")
                print(f"    Current RFC: {rfc:.4f}")
                print(f"    Best RFC: {self.best_rfc:.4f}")
                print(f"    Restoring best configuration...")
                self.maintenance_count += 1
                return self.best_config.copy()

        # RFC improved or maintained: update best
        if rfc > self.best_rfc:
            self.best_config = config.copy()
            self.best_rfc = rfc

        return config
```

#### 3.2 단독 테스트
**파일**: `tests/test_configuration_maintenance.py`

**테스트 시나리오**:
- RFC 증가: Best config 업데이트 확인
- RFC 감소: Best config 복원 확인
- Multi-modal balance: 논문 Fig. 11 재현

---

## 🟡 Medium Priority (효율성)

### 4. ✅ Termination Criteria 개선

**목표**: 10회 연속 RFC 개선 없으면 종료

**현재**: RFC ≥ 0.85면 즉시 종료
**논문**: 10 consecutive iterations without improvement

**구현 위치**: `simulation/integrated_explorer_v2.py`

```python
class IntegratedExplorerV2:
    def __init__(self, ...):
        self.no_improvement_count = 0
        self.prev_rfc = 0.0
        self.consecutive_threshold = 10  # 논문 기준

    def _check_termination(self, rfc: float, iteration: int) -> bool:
        """Check if search should terminate (논문 방식)."""
        # Update no-improvement counter
        if rfc <= self.prev_rfc + 0.01:  # Small tolerance
            self.no_improvement_count += 1
        else:
            self.no_improvement_count = 0

        self.prev_rfc = rfc

        # Terminate if no improvement for 10 iterations
        if self.no_improvement_count >= self.consecutive_threshold:
            print(f"\n✓ Termination: No improvement for {self.consecutive_threshold} iterations")
            print(f"  Final RFC: {rfc:.4f}")
            return True

        # Terminate if max iterations reached
        if iteration >= self.config.max_iterations:
            print(f"\n✓ Termination: Max iterations ({self.config.max_iterations}) reached")
            return True

        return False
```

---

### 5. ✅ Dynamic Swarm Number Adjustment (Fig. 11)

**목표**: RFC 낮으면 swarm 수 증가

**논문 실험** (Fig. 11):
- Steps 1-10: 3 swarms → RFC 낮음
- Step 11: 5 swarms로 증가 → RFC 급증

**구현 위치**: `simulation/integrated_explorer_v2.py`

```python
def _adjust_swarm_number(
    self,
    rfc: float,
    n_sources_estimated: int,
    iteration: int
) -> None:
    """
    Adjust swarm number based on RFC (논문 Fig. 11).

    RFC < threshold and sources < swarms → Add swarm
    """
    MIN_ITERATIONS = 5  # At least 5 iterations before adjustment
    RFC_THRESHOLD = 0.7  # Low RFC threshold

    if iteration < MIN_ITERATIONS:
        return

    current_swarms = len(self.estimator.swarms)

    # If RFC is low and we might be missing sources
    if rfc < RFC_THRESHOLD and n_sources_estimated < current_swarms:
        # Add one swarm
        self.estimator.add_swarm()
        print(f"\n  → Dynamic adjustment: Increased swarms to {len(self.estimator.swarms)}")
        print(f"    Reason: RFC={rfc:.4f} < {RFC_THRESHOLD}, sources={n_sources_estimated}")

# In ADEPSPF class:
def add_swarm(self) -> None:
    """Add a new particle swarm."""
    new_swarm = initialize_swarms(
        n_swarms=1,
        n_particles=self.config.n_particles,
        bounds=self.config.bounds,
        rng=self.rng
    )[0]
    self.swarms.append(new_swarm)
    self.centroids.append(None)
```

---

## 🟢 Low Priority (일치성)

### 6. ✅ Branch Execution → First Edge Only

**논문**: "the robot only executes the first edge"
**현재**: 80% 실행

**구현 위치**: `simulation/integrated_explorer_v2.py:ExplorationConfig`

```python
config = ExplorationConfig(
    branch_execution_ratio=0.0,  # First edge only (논문 방식)
    observations_per_iteration=1,  # One observation per iteration
)
```

**또는 명시적 구현**:
```python
def _execution_step(self, best_branch: List[RRTNode]) -> int:
    """Execute first edge only (논문 방식)."""
    if len(best_branch) < 2:
        return 0

    # Execute only first edge
    target_node = best_branch[1]

    # Move robot
    self.robot.move_to(
        target_node.pose.x,
        target_node.pose.y,
        target_node.pose.theta
    )

    # Make observation
    obs_intensity = self.observer.measure(
        np.array([target_node.pose.x, target_node.pose.y])
    )
    self.robot.add_observation(
        np.array([target_node.pose.x, target_node.pose.y]),
        obs_intensity
    )

    return 1  # One observation made
```

---

### 7. ✅ Exploration Phases 명시적 관리

**논문의 3단계**:
1. Tracing suspicious sources (iterations 1-36)
2. Surrounding observation (iterations 37-63)
3. Exploring unknown areas (iterations 64-83)

**구현 위치**: `simulation/integrated_explorer_v2.py`

```python
from enum import Enum

class ExplorationPhase(Enum):
    TRACING = 1      # Tracing suspicious sources
    SURROUNDING = 2  # Surrounding observation
    EXPLORING = 3    # Exploring unknown areas

class IntegratedExplorerV2:
    def __init__(self, ...):
        self.current_phase = ExplorationPhase.TRACING

    def _determine_phase(
        self,
        iteration: int,
        rfc: float,
        coverage: float
    ) -> ExplorationPhase:
        """
        Determine current exploration phase.

        논문 기준:
        - RFC < 0.3: Tracing
        - 0.3 ≤ RFC < 0.85: Surrounding
        - RFC ≥ 0.85: Exploring
        """
        if rfc < 0.3:
            return ExplorationPhase.TRACING
        elif rfc < 0.85:
            return ExplorationPhase.SURROUNDING
        else:
            return ExplorationPhase.EXPLORING

    def _log_phase_transition(
        self,
        old_phase: ExplorationPhase,
        new_phase: ExplorationPhase,
        iteration: int
    ):
        """Log phase transition (논문 Fig. 14-16)."""
        if old_phase != new_phase:
            print(f"\n{'='*70}")
            print(f"Phase Transition at Iteration {iteration}")
            print(f"  {old_phase.name} → {new_phase.name}")
            print(f"{'='*70}\n")
```

---

## 🧪 통합 테스트

### 8. ✅ 3-source Scenario (논문 Table 6)

**목표**: 논문 성능 재현

**파일**: `tests/test_paper_scenario_3source.py`

**시나리오 설정**:
```python
# 논문 Table 4 참조
scene_size = (21, 21)  # meters
sources = [
    (0.5, 2.0, 1050),   # (x, y, intensity)
    (-1.5, 0.0, 1200),
    (1.5, -1.5, 950)
]
robot_start = (-7.0, -8.0, 90°)
n_swarms = 4
n_particles = 250
```

**목표 성능** (논문 Table 6):
- SR: 85%
- RFC: 0.95 ± 0.01
- LE: 0.25 ± 0.05 m
- IE: 124.72 ± 47.41 nSv/h

**테스트 방법**:
- 20회 반복 실행
- 평균 및 표준편차 계산
- 논문 결과와 비교

---

### 9. ✅ 4-source Scenario (논문 Table 6)

**목표**: 복잡한 시나리오 성능 검증

**파일**: `tests/test_paper_scenario_4source.py`

**시나리오 설정**:
```python
# 논문 Table 4 참조
scene_size = (21, 21)  # meters
sources = [
    (0.5, -4.0, 900),
    (-1.5, -6.0, 950),
    (2.5, 7.0, 985),
    (0.5, 5.0, 950)
]
robot_start = (-3.0, 7.0, 0°)
n_swarms = 5
n_particles = 250
```

**목표 성능** (논문 Table 6):
- SR: 70%
- RFC: 0.88 ± 0.05
- LE: 0.43 ± 0.25 m
- IE: 77.68 ± 47.01 nSv/h

---

### 10. ✅ 최종 성능 검증

**목표**: 논문 Table 6 완전 재현

**비교 지표**:
| Metric | 논문 (3-src) | 구현 | 차이 |
|--------|-------------|------|------|
| SR | 85% | ??? | ??? |
| RFC | 0.95±0.01 | ??? | ??? |
| LE | 0.25±0.05m | ??? | ??? |
| IE | 124.72±47.41 | ??? | ??? |

**검증 기준**:
- SR: ±5%p 이내
- RFC: ±0.02 이내
- LE: ±0.1m 이내
- IE: ±20 nSv/h 이내

---

## 📊 진행 상황

- [x] High Priority (1-3): 3/3 완료 ✅
  - [x] 1. Gain Corrections (6/6) ✅
  - [x] 2. Superposition Suppression (5/5) ✅
  - [x] 3. Configuration Maintenance (3/3) ✅

- [x] Medium Priority (4-5): 2/2 완료 ✅
  - [x] 4. Termination Criteria ✅
  - [x] 5. Dynamic Swarm Adjustment ✅

- [x] Low Priority (6-7): 2/2 완료 ✅
  - [x] 6. Branch Execution ✅
  - [x] 7. Exploration Phases ✅

- [ ] Integration Tests (8-10): 0/3 완료
  - [ ] 8. 3-source scenario
  - [ ] 9. 4-source scenario
  - [ ] 10. Performance validation

---

## 🎯 예상 일정

### Phase 1: Core Algorithms (1-2주)
- Week 1: Gain Corrections + Superposition Suppression
- Week 2: Configuration Maintenance + 단독 테스트

### Phase 2: Enhancements (1주)
- Termination Criteria + Dynamic Swarm Adjustment

### Phase 3: Integration & Validation (1주)
- 3-source & 4-source scenarios
- 논문 성능 검증 및 조정

**총 예상 기간**: 3-4주

---

## 📝 참고 문서

- [PAPER_VS_IMPLEMENTATION.md](PAPER_VS_IMPLEMENTATION.md): 상세 차이점 분석
- [RRT_INTEGRATION_SUCCESS.md](RRT_INTEGRATION_SUCCESS.md): RRT Gain=0 수정 완료
- 논문 PDF: Page 5-7 (Section 4.2 Radiation gain evaluation)
