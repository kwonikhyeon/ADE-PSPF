# Algorithm 1: ADE-PSPF 수도코드 분석

## 📋 논문의 Algorithm 1 구조

```
Algorithm 1 The ADE-PSPF Algorithm

Input: Sensor data {M^int_i, M^pos_i}
Output: Radiation source parameters S = {S_s}^Ns_{s=1}

1:  for s=1 to N_ps do
2:      for r=1 to N_p do
3:          p_{s,r} ~rand(config_Gmin, config_Gmax)
4:      end for
5:  end for
6:  for j=1,...,N_i do
7:      for s=1 to N_ps do
8:          for r=1 to N_p do
9:              w^syn_{s,r} ← w^obs_{s,r} · w^ps_{s,r} · w^dist_{s,r}
10:         end for
11:     end for
12:     C_s ←Mean Shift ({p_{s,r}}^Np_{r=1}, {w^syn_{s,r}}^Np_{r=1})
13: end for
14: if F ≥ THR_conf then
15:     S = {S_s}^Ns_{s=1} ← {C^new_s, P^new_s, F_s^new}^Nps_{s=1} - {C_s, P_s, F_s}^Nps_{s=1}
16: else then
17:     repeat Lines 6-13 loop
18: end if
19: if F_s^new > THR_conf then
20:     S = {S_s}^Ns_{s=1} ← {C^new_s}^Nps_{s=1}, N_ps ≥ N_s
21: else then
22:     repeat Lines 6-13 loop
23: end if
```

---

## 🔍 상세 분석

### Phase 1: 초기화 (Lines 1-5)

```python
for s in range(1, N_ps + 1):  # N_ps개의 swarm
    for r in range(1, N_p + 1):  # 각 swarm당 N_p개 particle
        p[s, r] = random_uniform(config_Gmin, config_Gmax)
```

**목적:**
- 모든 particle을 설정 공간 내에서 랜덤 초기화
- `config_Gmin`: 최소 설정 [x_min, y_min, intensity_min]
- `config_Gmax`: 최대 설정 [x_max, y_max, intensity_max]

**설정 공간 예시:**
```python
config_Gmin = [0, 0, 10.0]          # [x_min, y_min, I_min]
config_Gmax = [256, 256, 100.0]     # [x_max, y_max, I_max]
```

---

### Phase 2: ADE-PSPF 메인 루프 (Lines 6-13)

```python
for iteration in range(N_i):  # N_i번 반복
    # Step 1: Weight 계산 (Lines 7-11)
    for s in range(1, N_ps + 1):
        for r in range(1, N_p + 1):
            w_syn[s, r] = w_obs[s, r] * w_ps[s, r] * w_dist[s, r]

    # Step 2: ADE Resampling (논문의 implicit step)
    for s in range(1, N_ps + 1):
        particles[s] = ade_resampling(particles[s], weights[s], config)

    # Step 3: Mean Shift Clustering (Line 12)
    for s in range(1, N_ps + 1):
        C[s] = mean_shift(particles[s], weights[s])
```

**주의사항:**
- 논문의 Algorithm 1에는 **ADE resampling이 명시되지 않음**
- 실제로는 Lines 6-13 사이에 수행됨 (Section 3.3 참조)
- Mean Shift는 각 swarm의 중심을 찾는 클러스터링

---

### Phase 3: Configuration Maintenance (Lines 14-18)

```python
# RFC 계산
F = compute_rfc(observations, centroids)

if F >= THR_conf:
    # 새로운 configuration이 더 나음
    S = update_configuration(C_new, P_new, F_new)
else:
    # RFC가 낮아지면 이전 최적 상태로 복원
    restore_best_configuration()
    repeat_lines_6_13()
```

**목적:**
- RFC가 임계값 이상이면 configuration 업데이트
- RFC가 임계값 미만이면 최적 configuration 복원 후 재시도
- Multi-modal balance가 깨졌을 때 복구

**Configuration:**
```python
Configuration = {
    'centroids': C_s,      # Swarm 중심들
    'particles': P_s,      # Particle swarms
    'confidence': F_s      # RFC 값
}
```

---

### Phase 4: 최종 Source 결정 (Lines 19-23)

```python
if F_new > THR_conf:
    # 유효한 swarm의 중심만 source로 선택
    S = filter_valid_sources(C_new)  # N_ps ≥ N_s
else:
    # 여전히 낮으면 다시 시도
    repeat_lines_6_13()
```

**Source 필터링:**
- Mean Shift가 클러스터링을 실패한 redundant swarm 제거
- 유효한 중심만 최종 source parameter로 선택

---

## 🔄 전체 워크플로우

```
┌─────────────────────────────────────────────────────────┐
│ 1. Initialization                                       │
│    - Random sample N_ps × N_p particles                 │
│    - Each particle p = [x, y, intensity]                │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│ 2. ADE-PSPF Main Loop (N_i iterations)                  │
│    ┌─────────────────────────────────────────────────┐  │
│    │ 2.1 Synthesized Weight Calculation              │  │
│    │     - w_obs: Observation weight (Poisson)       │  │
│    │     - w_dist: Peak suppression                  │  │
│    │     - w_ps: Swarm distance correction           │  │
│    │     → w_syn = w_obs × w_dist × w_ps             │  │
│    └───────────────────┬─────────────────────────────┘  │
│                        │                                 │
│    ┌───────────────────▼─────────────────────────────┐  │
│    │ 2.2 ADE Resampling (per swarm)                  │  │
│    │     FOR generation g = 1 to ADE_generations:    │  │
│    │       - Mutation: v = p + F1(best-p) + F2(r1-r2)│  │
│    │       - Crossover: u = crossover(p, v, CR)      │  │
│    │       - Selection: p_new = select(p, u, w)      │  │
│    └───────────────────┬─────────────────────────────┘  │
│                        │                                 │
│    ┌───────────────────▼─────────────────────────────┐  │
│    │ 2.3 Mean Shift Clustering                       │  │
│    │     - Find centroid C_s for each swarm          │  │
│    │     - Filter redundant swarms                   │  │
│    └─────────────────────────────────────────────────┘  │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│ 3. RFC Calculation                                      │
│    F = (1/N_o) Σ [Poisson_prob(M_i | predict(C_s))]    │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│ 4. Configuration Maintenance                            │
│    IF F >= THR_conf:                                    │
│       Update configuration                              │
│    ELSE:                                                │
│       Restore best configuration & retry                │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│ 5. Final Source Selection                               │
│    Filter valid centroids → S = {S_1, ..., S_Ns}       │
└─────────────────────────────────────────────────────────┘
```

---

## 📊 주요 변수 정리

| Variable | Type | Shape | Description |
|----------|------|-------|-------------|
| `p_{s,r}` | array | (3,) | s번째 swarm의 r번째 particle [x, y, I] |
| `w^syn_{s,r}` | float | - | Synthesized weight |
| `C_s` | array | (3,) | s번째 swarm의 centroid [x, y, I] |
| `F` | float | - | RFC (Radiation Field Confidence) |
| `S` | list | (N_s, 3) | 최종 source parameters |
| `N_ps` | int | - | Swarm 개수 (≥ N_s) |
| `N_p` | int | - | Swarm당 particle 개수 |
| `N_i` | int | - | PSPF 메인 루프 반복 횟수 |

---

## ⚙️ 하이퍼파라미터

| Parameter | Description | Typical Range |
|-----------|-------------|---------------|
| `N_ps` | Number of swarms | 4-5 |
| `N_p` | Particles per swarm | 150-300 |
| `N_i` | PSPF iterations | 10-20 |
| `ADE_generations` | ADE inner loop | 5-10 |
| `THR_conf` | RFC threshold | 0.85-0.95 |
| `α` | Elite scale | 0.5-1.0 |
| `β` | Random scale | 0.3-0.5 |
| `CR_base` | Base crossover | 0.5-0.7 |

---

## 🎯 구현 시 주의사항

### 1. ADE Resampling 위치
- Algorithm 1에는 명시 안 되어 있지만 필수
- Lines 6-13 루프 내에서 수행
- Weight 계산 후, Mean Shift 전에 실행

### 2. Configuration 구조
```python
best_config = {
    'particles': P_best,    # 최적 particle 상태
    'centroids': C_best,    # 최적 중심
    'confidence': F_best    # 최고 RFC
}
```

### 3. Swarm 개수 조정
- 실제 source 개수를 모르므로 N_ps > N_s로 설정
- Redundant swarm은 Mean Shift에서 클러스터링 실패
- 유효한 centroid만 최종 결과로 반환

### 4. 수렴 조건
- RFC가 임계값 이상
- 또는 최대 반복 횟수 도달
- Configuration maintenance가 안정화

---

## ✅ 다음 단계

1. ✅ 수식 분석 완료
2. ✅ Algorithm 1 수도코드 분석 완료
3. ⏳ Particle 및 ParticleSwarm 클래스 설계
4. ⏳ Weight 계산 함수 구현
5. ⏳ ADE resampling 구현
6. ⏳ Mean Shift 구현
7. ⏳ RFC 계산 및 Configuration Maintenance 구현
8. ⏳ 전체 통합 및 테스트
