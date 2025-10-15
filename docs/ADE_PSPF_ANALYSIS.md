# ADE-PSPF Algorithm Analysis

## 📚 논문 수식 상세 분석

### 1. 방사선 전파 모델 (Equation 1)

**관측 위치에서의 누적 방사선 강도**

```
M^int_i = I'_cum(M^pos_i, {S_k}^Ns_{k=1}) = Eτ_i(∑^Ns_{k=1} I(M^pos_i, S_k) + R^i_back)

I(M^pos_i, S_k) = (S^int_k) / (h^2 + ||M^pos_i - S^pos_k||^2) · exp(-μ_air ||M^pos_i - S^pos_k||)
```

**변수 설명:**
- `M^int_i`: i번째 관측 지점의 방사선 강도 (mean radiation count)
- `M^pos_i`: i번째 관측 지점의 위치
- `S_k = [S^x_k, S^y_k, S^int_k]`: k번째 소스의 파라미터 (x, y, intensity)
- `N_s`: 총 소스 개수
- `E`: nSv/h → CPS 변환 상수
- `τ_i`: i번째 관측의 지속 시간
- `R^i_back`: 변동하는 배경 방사선
- `h`: 소스의 높이 (관측 평면으로부터)
- `μ_air`: 공기 흡수 계수 (Co-60의 경우 6.86 × 10^-3 m^-1)

**근사:**
작은 탐색 영역(수십 미터)에서 `exp(-μ_air ||M^pos_i - S^pos_k||) ≈ 1`

---

### 2. Particle Swarm 구조 (Equation 2)

```
P_s = {p_{s,r} ∈ R^3}^Np_{r=1}, s ∈ [1, ..., N_ps]
```

**변수 설명:**
- `P_s`: s번째 particle swarm
- `p_{s,r}`: s번째 swarm의 r번째 particle
- `N_p`: 각 swarm 내의 particle 개수
- `N_ps`: 전체 swarm 개수
- `p_{s,r} = [x, y, intensity]`: 각 particle은 3차원 벡터 (위치 + 강도)

**물리적 의미:**
- 각 particle swarm은 하나의 방사선 소스를 추정
- Swarm 개수가 실제 소스 개수보다 많을 수 있음 (redundancy)
- 각 particle은 소스의 가능한 상태를 나타냄

---

### 3. Synthesized Weight 계산 (Equation 3)

```
w^syn_{s,r} = w_obs(M^int_i, p_{s,r}, C_{-s}) · w_ps(p_{s,r}, θ_ps) · w_dist(p_{s,r}, C_{-s})
```

#### 3.1 Observation Weight

```
w_obs(M^int_i, p_{s,r}, C_{-s}) = f_p(M^int_i | I'(p_{s,r}, C_{-s})) / f_p(⌊I'(p_{s,r}, C_{-s})⌋ | I'(p_{s,r}, C_{-s}))

f_p(N_Z | λ({S_k})) = (λ({S_k})^N_Z / N_Z!) exp(-λ({S_k}))
```

- `f_p`: Poisson 확률 분포 함수
- `C_{-s}`: 다른 swarm들의 중심 (현재 swarm 제외)
- `⌊·⌋`: 반올림 연산
- `N_Z`: 센서에 도달한 입자 수
- `λ({S_k})`: 소스 집합에 기반한 평균 입자 수

#### 3.2 Peak Suppression (Distance Weight)

```
w_dist(p_{s,r}, C_{-s}) = 1 / (1 + exp[(θ_dist - f_d(p_{s,r}, C_{-s})) / b_dist])

f_d(p_{s,r}, C_{-s}) = min_{j≠s} ||p_{s,r} - C_j||
```

- `f_d`: 현재 particle과 다른 swarm 중심 간의 최소 거리
- `θ_dist`: 억제 곡선의 수평 오프셋
- `b_dist`: 변화율 조정 scale parameter

#### 3.3 Swarm Distance Correction

```
w_ps(p_{s,r}, θ_ps) = (1 - α) + α · (1 / (1 + exp[(p^int_{s,r} - θ_ps) / b_ps]))
```

- `α`: 수직 조정 파라미터
- `θ_ps`: 보정 곡선의 수평 오프셋
- `b_ps`: scale parameter
- `p^int_{s,r}`: particle의 강도 값

---

### 4. ADE Mutation (Equation 4)

```
v_{s,r}(g) = p_{s,r}(g) + F_1 · (p^best_s(g) - p_{s,r}(g)) + F_2 · (p^r1_s(g) - p^r2_s(g))

F_1 = α · (1 - w_{s,r})
F_2 = β · (w_{s,r1} - w̄_{s,r}) / w_{s,r1}
```

**변수 설명:**
- `v_{s,r}(g)`: g세대의 mutant individual
- `p_{s,r}(g)`: target individual (현재 particle)
- `p^best_s(g)`: 가장 높은 weight를 가진 particle
- `p^r1_s(g), p^r2_s(g)`: 랜덤 선택된 particles (r ≠ r1 ≠ r2)
- `F_1`: elite movement scale (최고 particle로의 이동 제어)
- `F_2`: random movement scale (탐색 다양성 제어)
- `α, β`: scale 조정 상수
- `w_{s,r}`: p_{s,r}에 해당하는 weight
- `w_{s,r1}`: p^r1_s에 해당하는 weight
- `w̄_{s,r}`: weight의 평균값

**적응적 특성:**
- Weight가 낮은 particle은 더 큰 F_1 → 최적 위치로 빠르게 이동
- Weight가 높은 particle은 작은 F_1 → 현재 위치 유지 (exploitation)

---

### 5. ADE Crossover (Equation 5)

```
u_{s,r}(g)[j] = {
    v_{s,r}(g)[j],           if rand < CR or j = j_rand
    p_{s,r}(g)[j] + σ_r,     otherwise
}

CR = CR_base + CR_scale · (w_{s,r} - w̄_{s,r}) / w̄_{s,r}
```

**변수 설명:**
- `u_{s,r}(g)`: trial individual
- `j`: 차원 인덱스 (x, y, intensity)
- `j_rand`: 랜덤 선택된 차원 (최소 1개 차원은 mutation 보장)
- `CR`: crossover rate (적응적)
- `CR_base`: 기본 crossover rate
- `CR_scale`: crossover rate 조정 scale
- `σ_r`: 평균 0인 Gaussian noise (particle impoverishment 방지)

**적응적 특성:**
- Weight가 높은 particle → 높은 CR → mutation의 영향 증가
- Weight가 낮은 particle → 낮은 CR → 원래 값 유지 + noise

---

### 6. ADE Selection (Equation 6)

```
p_{s,r}(g + 1) = {
    u_{s,r}(g),    if w(u_{s,r}(g)) > w(p_{s,r}(g))
    p_{s,r}(g),    otherwise
}
```

**선택 기준:**
- Trial individual의 weight가 더 높으면 선택
- 그렇지 않으면 현재 individual 유지

---

### 7. RFC (Radiation Field Confidence) (Equation 7)

```
F({M^int_i, M^pos_i}^No_{i=1}, {C_s}^Nps_{s=1}) =
    (1/N_o) · ∑^No_{i=1} [
        f_p(M^int_i | I'_cum(M^pos_i, {C_s}^Nps_{s=1})) /
        f_p(⌊I'_cum(M^pos_i, {C_s}^Nps_{s=1})⌋ | I'_cum(M^pos_i, {C_s}^Nps_{s=1}))
    ]
```

**변수 설명:**
- `F`: 전체 방사선 필드 신뢰도
- `N_o`: 관측 개수
- `{C_s}^Nps_{s=1}`: 모든 swarm의 중심 (예측 결과)
- `I'_cum`: 모든 중심 기반 누적 강도 예측

**물리적 의미:**
- 각 관측 지점에서 예측과 실제 측정의 Poisson 확률 비율
- 값이 1에 가까울수록 예측이 정확함
- Configuration Maintenance에서 사용 (최고 RFC 상태 저장 및 복원)

---

## 🎯 핵심 알고리즘 특징

### 1. Multi-modal 추정
- 각 swarm이 독립적으로 하나의 소스를 추정
- Swarm 간 간섭은 peak suppression으로 해결

### 2. 적응적 진화
- Weight 기반으로 F_1, F_2, CR이 자동 조정
- 좋은 위치의 particle은 exploitation
- 나쁜 위치의 particle은 exploration

### 3. 지역 최적값 회피
- ADE의 mutation 연산으로 다양성 유지
- Gaussian noise 추가로 particle impoverishment 방지
- Configuration maintenance로 최적 상태 보존

### 4. 신뢰도 기반 관리
- RFC로 예측 품질 정량화
- 낮은 RFC 시 이전 최적 상태로 복원

---

## 📊 파라미터 범위 (논문 기준)

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `N_ps` | Swarm 개수 | 4-5 |
| `N_p` | Swarm당 particle 개수 | 150-300 |
| `α` | Elite movement scale | 0.5-1.0 |
| `β` | Random movement scale | 0.3-0.5 |
| `CR_base` | Base crossover rate | 0.5-0.7 |
| `CR_scale` | Crossover scale | 0.2-0.3 |
| `θ_dist` | Distance threshold | 조정 필요 |
| `b_dist` | Distance scale | 조정 필요 |
| `θ_ps` | Swarm correction offset | 조정 필요 |
| `b_ps` | Swarm correction scale | 조정 필요 |

---

## 🔄 다음 단계

1. ✅ 수식 분석 완료
2. ⏳ Algorithm 1 수도코드 분석
3. ⏳ 데이터 구조 설계
4. ⏳ 단계별 구현
