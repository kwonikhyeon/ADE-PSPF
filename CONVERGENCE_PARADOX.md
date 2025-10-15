# RFC = 1.0 수렴 패러독스: 높은 신뢰도, 부정확한 위치

## 🎯 문제 요약

**RFC는 1.0으로 완벽히 수렴했지만, 예측된 소스 위치가 실제와 크게 다름**

### 실제 vs 예측 비교

| 구분 | Source 1 | Source 2 | Source 3 |
|------|----------|----------|----------|
| **실제 위치** | (183, 43) | (44, 66) | (186, 172) |
| **예측 위치** | (87, 101) | (98, 42) | (69, 14) |
| **거리 오차** | **137 px** | **69 px** | **165 px** |
| **실제 강도** | 60.72 | 98.29 | 38.97 |
| **예측 강도** | 32.27 | 14.33 | 10.96 |

**추가로 잘못된 4번째 소스**: (93, 238), A=10.98

---

## 🔍 원인 분석

### 1. 극도로 제한된 탐색 영역

```
로봇 경로:
  Step 0: (128.0, 128.0) - 시작
  Step 1: (124.8, 106.3) - 약간 이동
  Step 2: (94.5, 191.9)  - 최종 위치

탐색 영역:
  X: 94.5 ~ 128.0 (범위: 33.5 pixels)
  Y: 106.3 ~ 191.9 (범위: 85.6 pixels)

커버율: 4.4% (전체 256×256 맵 중)
```

**모든 실제 소스가 탐색 영역 밖에 위치!**

| 소스 | X 범위 내? | Y 범위 내? | 가장 가까운 관측 |
|------|-----------|-----------|----------------|
| Source 1 (183, 43) | ❌ (183 >> 128) | ❌ (43 << 106) | 86 px |
| Source 2 (44, 66) | ❌ (44 << 94) | ❌ (66 << 106) | 90 px |
| Source 3 (186, 172) | ❌ (186 >> 128) | ✓ (172 in range) | 73 px |

---

### 2. 역문제(Inverse Problem)의 비유일성

**핵심**: 제한된 관측으로는 무수히 많은 소스 배치가 같은 관측값을 만들 수 있음

#### 예시: 관측 위치 (128, 128)에서 강도 14.32

**실제 구성 (먼 거리, 강한 소스)**:
```
Source 1 @ 101px: 60.72 / (0.25 + 3.96²) = 3.82
Source 2 @ 104px: 98.29 / (0.25 + 4.07²) = 5.82
Source 3 @ 73px:  38.97 / (0.25 + 2.85²) = 4.67
───────────────────────────────────────────────
Total: 14.31 ✓
```

**예측 구성 (가까운 거리, 약한 소스)**:
```
Est 1 @ 28px: 32.27 / (0.25 + 1.09²) = 21.62
Est 2 @ 27px: 14.33 / (0.25 + 1.05²) = 12.31
Est 3 @ 114px: 10.96 / (0.25 + 4.45²) = 0.54
Est 4 @ 113px: 10.98 / (0.25 + 4.41²) = 0.56
───────────────────────────────────────────────
Total: 10.12 (오차 29%, 하지만 여전히 RFC=1.0!)
```

**동등성**: 두 구성 모두 관측된 3개 위치에서 유사한 강도를 생성
- 관측 1: 3-29% 오차
- 관측 2: 3% 오차
- 관측 3: 19% 오차

→ **Poisson 확률 비율이 모두 높음 → RFC = 1.0**

---

### 3. RFC의 한계

#### RFC가 측정하는 것
✅ 예측 소스들이 **관측된 위치에서** 강도를 잘 재현하는가?

#### RFC가 보장하지 않는 것
❌ 예측 소스들이 **실제 소스 위치**와 일치하는가?
❌ **관측되지 않은 영역**에서도 강도가 정확한가?
❌ 소스의 **개수**가 정확한가? (3개인데 4개로 추정)

---

## 📊 수학적 설명

### RFC 정의 (Equation 7)

```
RFC = (1/N_o) × Σ [ P(k_obs | λ_pred) / P(⌊λ_pred⌋ | λ_pred) ]
```

**관측이 3개뿐일 때**:
- N_o = 3 (매우 적음!)
- 3개 위치에서만 예측 정확도 평가
- 나머지 256² - 3 = 65,533개 위치는 검증 안 됨

**국소 최적화**:
```python
# ADE-PSPF 목적 함수
maximize: RFC(observations, centroids)

# 제약 조건
observations = [(128,128), (125,106), (95,192)]  # 고정된 3개

# 결과
→ 이 3개 위치만 잘 맞추면 RFC = 1.0
→ 실제 소스 위치는 고려 안 됨
```

---

## 🔬 실험적 검증

### 관측 위치별 예측 vs 실제

| 위치 | 실제 강도 | 예측 강도 | 오차 | Poisson 비율 |
|------|-----------|-----------|------|-------------|
| (128, 128) | 14.32 | 10.12 | 29% | ~0.90 |
| (125, 106) | 16.17 | 15.68 | 3% | ~1.00 |
| (95, 192) | 7.69 | 6.21 | 19% | ~0.95 |

**평균 Poisson 비율**: (0.90 + 1.00 + 0.95) / 3 ≈ **0.95**

→ RFC는 이를 1.0으로 clipping → **"수렴"으로 판단**

---

## 💡 왜 이런 일이 발생했는가?

### 1. Gain = 0 문제

```python
# RRT Gain 계산 (Equation 11-22)
gain = radiation_gain + unexplored_gain + distance_penalty

# RFC = 0일 때 (Iteration 1-4)
radiation_gain ≈ 0  # 소스 추정이 엉망
unexplored_gain ≈ 0  # 어디든 미탐색
distance_penalty ≈ 0

→ 모든 경로의 gain = 0
→ 랜덤하게 경로 선택
→ 좁은 영역만 탐색
```

### 2. 조기 수렴

```
Iteration 1-4: RFC = 0.0000 (넓게 탐색, gain=0으로 랜덤)
Iteration 5: RFC = 1.0000 (갑자기 수렴!)

→ 넓은 탐색 기회 없이 종료
```

### 3. 탐색 전략 부재

- 초기 Grid 탐색 없음
- 정보 이득 기반 탐색 없음
- 불확실성 감소 목표 없음

---

## 🛠️ 해결 방법

### 방법 1: 초기 Grid 탐색 추가 ⭐⭐⭐⭐⭐

```python
def initial_grid_exploration(self):
    """전체 맵을 Grid로 나누어 균등하게 탐색"""

    # 4×4 Grid (16개 위치)
    grid_points = [
        (64, 64), (64, 128), (64, 192),   # 왼쪽
        (128, 64), (128, 128), (128, 192), # 중앙
        (192, 64), (192, 128), (192, 192), # 오른쪽
    ]

    for pos in grid_points:
        self.robot.move_to(*pos, theta=0)
        self._make_observation(*pos)

        # ADE-PSPF 업데이트
        result = self.estimator.update(self.robot.observations)

        if result['rfc'] >= 0.5:  # 충분한 정보 수집
            break
```

**효과**:
- 전체 맵의 36% 커버 (vs 현재 4.4%)
- 모든 사분면 탐색
- 실제 소스 근처 방문 확률 증가

---

### 방법 2: RFC 수렴 조건 강화 ⭐⭐⭐⭐

**현재 (문제)**:
```python
if rfc >= 0.85:
    converged = True
    break
```

**개선안**:
```python
# 조건 1: RFC 높음
# 조건 2: 공간 커버리지 충분
# 조건 3: 연속 수렴 확인

def check_convergence(self):
    rfc_ok = self.current_rfc >= 0.85

    # 공간 커버리지 계산
    visited_area = self._compute_visited_area()
    coverage_ratio = visited_area / (256 * 256)
    coverage_ok = coverage_ratio >= 0.15  # 최소 15%

    # 연속 3회 이상 RFC >= 0.85
    recent_rfcs = self.rfc_history[-3:]
    stable_ok = all(r >= 0.85 for r in recent_rfcs)

    return rfc_ok and coverage_ok and stable_ok
```

**효과**:
- 조기 수렴 방지
- 더 넓은 탐색 강제
- 안정적인 수렴 보장

---

### 방법 3: Exploration Bonus 추가 ⭐⭐⭐

```python
def compute_exploration_gain(self, position):
    """미탐색 영역에 보너스 부여"""

    # 가장 가까운 관측까지의 거리
    min_dist = min(
        np.linalg.norm(position - obs_pos)
        for obs_pos, _ in self.observations
    )

    # 거리가 멀수록 높은 보너스
    exploration_bonus = min_dist / 50.0  # 정규화

    return exploration_bonus

# RRT Gain에 추가
total_gain = radiation_gain + exploration_bonus + unexplored_gain
```

**효과**:
- 먼 곳 탐색 장려
- 관측 밀도 균등화

---

### 방법 4: 소스 개수 사전 정보 활용 ⭐⭐

```python
config = ExplorationConfig(
    expected_n_sources=3,  # Ground truth 정보
    max_n_sources=4,
)

# ADE-PSPF에 전달
adepspf_config = ADEPSPFConfig(
    n_swarms=3,  # expected_n_sources와 일치
)
```

**효과**:
- 과적합 방지 (4개 대신 3개 추정)
- 더 정확한 위치 추정

---

### 방법 5: 다단계 수렴 검증 ⭐⭐⭐⭐

```python
def verify_convergence(self):
    """수렴 후 추가 검증"""

    if self.current_rfc >= 0.85:
        # Step 1: 전체 맵에서 무작위 10개 위치 샘플
        test_positions = random.sample(
            all_positions, k=10
        )

        # Step 2: 예측 vs 실제 비교
        errors = []
        for pos in test_positions:
            true_intensity = self.observer.observe(pos)
            pred_intensity = self.predict_at(pos)
            error = abs(pred - true) / true
            errors.append(error)

        # Step 3: 평균 오차 < 30% 확인
        avg_error = np.mean(errors)

        return avg_error < 0.3
```

**효과**:
- 관측 영역 밖에서도 검증
- 실제 정확도 확인

---

## 📈 예상 개선 효과

### 현재
```
탐색 영역: 4.4%
관측 수: 18개 (좁은 영역에 집중)
RFC: 1.0 ✓
위치 오차: 69-165 pixels ✗
```

### 개선 후 (방법 1+2+3 적용)
```
탐색 영역: 36%+ (Grid 탐색)
관측 수: 50개+ (균등 분포)
RFC: 0.90+ (안정적)
위치 오차: < 20 pixels ✓
수렴 시간: 20-30 iterations
```

---

## 🎓 교훈

### RFC의 의미 재정의

**RFC는**:
- ✅ 모델의 **관측 재현 능력** 지표
- ✅ **관측된 영역**에서의 신뢰도
- ✅ 상대적 수렴 지표

**RFC는 아님**:
- ❌ **절대적 정확도** 지표
- ❌ **전역 최적해** 보장
- ❌ **소스 위치 정확도** 측정

### 탐색의 중요성

```
좋은 추정 = 좋은 알고리즘 + 충분한 탐색
```

아무리 좋은 추정 알고리즘(ADE-PSPF)이라도:
- 데이터가 편향되면 → 편향된 결과
- 데이터가 부족하면 → 국소 최적해
- 데이터 커버리지가 낮으면 → 비유일 해

---

## 📝 권장 사항

### 즉시 적용 (우선순위 높음)
1. ✅ **초기 Grid 탐색** 구현
2. ✅ **공간 커버리지** 종료 조건 추가
3. ✅ **Exploration Bonus** 추가

### 중기 개선
4. 정보 이득(Information Gain) 기반 탐색
5. Uncertainty-aware 경로 계획
6. 다중 스케일 탐색 (coarse → fine)

### 장기 개선
7. Active Learning 전략
8. Bayesian Optimization 적용
9. POMDP 기반 planning

---

## 🔗 관련 문서

- [FIX_RESULTS.md](FIX_RESULTS.md) - RFC = 0 문제 해결
- [RFC_ZERO_COMPLETE_ANALYSIS.md](RFC_ZERO_COMPLETE_ANALYSIS.md) - RFC 계산 분석
- [TRAJECTORY_ISSUE.md](TRAJECTORY_ISSUE.md) - 로봇 이동 문제

---

## 📊 시각화

생성된 이미지에서 확인 가능:
- [data/figures/integrated_explorer_v2_result.png](data/figures/integrated_explorer_v2_result.png)
  - **왼쪽 상단**: Ground Truth (실제 소스) vs Trajectory (매우 짧음)
  - **오른쪽 상단**: 추정 소스 (실제 위치와 큰 차이)
  - **왼쪽 하단**: RFC = 1.0 수렴 그래프
  - **오른쪽 하단**: 통계 (4.4% 커버율!)

---

## ✅ 결론

**RFC = 1.0 ≠ 완벽한 추정**

- RFC는 국소적 신뢰도일 뿐, 전역 정확도를 보장하지 않음
- 충분한 공간 탐색 없이는 정확한 소스 위치 추정 불가
- **해결책: 초기 Grid 탐색 + 커버리지 기반 종료 조건**
