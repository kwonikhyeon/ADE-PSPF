# 로봇 이동 거리 분석

## 📍 실제 로봇 경로

### 전체 이동 경로

```
시작:      (128.0, 128.0)
Iter 1 →   (124.8, 106.3)  [이동: 22.0 pixels = 0.88 m]
Iter 2 →   (124.8, 106.3)  [이동: 0.0 pixels = 정지]
Iter 3 →   (124.8, 106.3)  [이동: 0.0 pixels = 정지]
Iter 4 →   (94.5, 191.9)   [이동: 91.2 pixels = 3.65 m]
```

---

## 📊 이동 거리 통계

### 반복별 이동

| 반복 | 시작 위치 | 끝 위치 | 거리 (px) | 거리 (m) | 상태 |
|------|-----------|---------|-----------|----------|------|
| **Iter 1** | (128.0, 128.0) | (124.8, 106.3) | **22.0** | **0.88** | ✓ 정상 |
| **Iter 2** | (124.8, 106.3) | (124.8, 106.3) | **0.0** | **0.00** | ❌ 정지 |
| **Iter 3** | (124.8, 106.3) | (124.8, 106.3) | **0.0** | **0.00** | ❌ 정지 |
| **Iter 4** | (124.8, 106.3) | (94.5, 191.9) | **91.2** | **3.65** | ✓ 정상 |

### 전체 통계

```
총 이동 거리:    113.2 pixels (4.53 meters)
평균 이동:       28.3 pixels/iteration
직선 거리:       74.4 pixels (2.98 meters)
경로 효율:       65.7%

정지 횟수:       2회 (Iter 2, 3)
```

---

## 🔍 RRT 경로 생성 분석

### Iteration별 RRT 결과

#### **Iteration 1** ✓
```
Best branch: 5 nodes
Executing: 4/5 nodes (80% - branch_execution_ratio=0.8)
Gain: 0.000000

실제 이동: (128.0, 128.0) → (124.8, 106.3)
거리: 22.0 pixels

분석:
- RRT가 5개 노드 경로 생성
- 80%인 4개 노드 실행
- Gain=0이지만 경로는 정상 생성됨
```

#### **Iteration 2** ❌
```
Best branch: 4 nodes
Executing: 3/4 nodes (75%)
Gain: 0.000000

실제 이동: (124.8, 106.3) → (124.8, 106.3)
거리: 0.0 pixels ← 문제!

분석:
- RRT가 4개 노드 경로 생성
- 3개 노드 실행 시도
- 하지만 실제로는 이동 안 함
- 원인: 생성된 경로가 현재 위치 주변만 맴도는 경로
```

#### **Iteration 3** ❌
```
Best branch: 5 nodes
Executing: 4/5 nodes (80%)
Gain: 0.000000

실제 이동: (124.8, 106.3) → (124.8, 106.3)
거리: 0.0 pixels ← 문제!

분석:
- Iteration 2와 동일한 패턴
- 같은 위치에서 맴돌고 있음
```

#### **Iteration 4** ✓
```
Best branch: 21 nodes ← 긴 경로!
Executing: 16/21 nodes (76%)
Gain: 0.000000

실제 이동: (124.8, 106.3) → (94.5, 191.9)
거리: 91.2 pixels ← 큰 이동!

분석:
- RRT가 21개 노드의 긴 경로 생성
- 16개 노드 실행 (충분한 이동)
- 이후 Iter 5에서 RFC=1.0 수렴
```

---

## 🎯 각 측정 위치 간 거리

### 주요 이동 구간

#### **시작 → Iter 1**
```
거리: 22.0 pixels (0.88 meters)

좌표 변화:
  ΔX = 128.0 - 124.8 = 3.2 pixels (동쪽으로 약간)
  ΔY = 128.0 - 106.3 = 21.7 pixels (남쪽으로 크게)

방향: 남남동(SSE)
```

#### **Iter 1 → Iter 2, 3** (정지)
```
거리: 0.0 pixels

원인:
- Gain = 0 → 모든 방향이 동등
- RRT가 랜덤한 경로 생성
- 생성된 경로가 현재 위치 근처만 탐색
- branch_execution_ratio=0.8이지만 이동 거리 짧음
```

#### **Iter 3 → Iter 4**
```
거리: 91.2 pixels (3.65 meters)

좌표 변화:
  ΔX = 124.8 - 94.5 = 30.3 pixels (서쪽으로)
  ΔY = 106.3 - 191.9 = -85.6 pixels (북쪽으로 크게)

방향: 북북서(NNW)

분석:
- 21개 노드의 긴 경로
- 실제 방사선 소스 방향과 다름
  (Source 3 (186, 172)가 북동쪽인데 북서쪽으로 이동)
```

---

## ⚠️ 문제점

### 1. 정지 문제 (Iter 2, 3)

**원인**:
```python
# Gain이 모두 0일 때
gain = radiation_gain + unexplored_gain + distance_penalty
     = 0 + 0 + 0 = 0

# RRT는 랜덤 경로 생성
path = random_exploration()

# 생성된 경로가 짧거나 현재 위치 근처
→ branch_execution_ratio=0.8 적용해도 이동 거리 ≈ 0
```

**해결책**:
- Gain=0일 때 최소 이동 거리 강제
- 또는 Grid 기반 탐색으로 전환

### 2. 방향성 문제

**현재**: 로봇이 랜덤하게 이동
```
Iter 1: 남남동 (SSE)
Iter 4: 북북서 (NNW)
```

**이상적**: 방사선 gradient 방향으로 이동
```
Source 1 (183, 43):  동남쪽 (SE)
Source 2 (44, 66):   서남쪽 (SW)
Source 3 (186, 172): 동북쪽 (NE)

→ 세 소스의 중간 방향으로 이동해야 함
```

### 3. 탐색 범위 제한

**현재 탐색 영역**:
```
X: 94.5 ~ 128.0 (범위: 33.5 pixels)
Y: 106.3 ~ 191.9 (범위: 85.6 pixels)
```

**전체 맵**:
```
X: 0 ~ 256 (범위: 256 pixels)
Y: 0 ~ 256 (범위: 256 pixels)
```

**커버율**: 4.4% ❌

---

## 💡 이동 거리 요약

### 질문: "현재 위치에서 다음 측정 위치까지의 거리가 얼마야?"

**답변**:

| 구간 | 거리 (pixels) | 거리 (meters) | 비고 |
|------|---------------|---------------|------|
| **시작 → Iter 1** | **22.0** | **0.88** | 정상 이동 |
| **Iter 1 → Iter 2** | **0.0** | **0.00** | ❌ 정지 |
| **Iter 2 → Iter 3** | **0.0** | **0.00** | ❌ 정지 |
| **Iter 3 → Iter 4** | **91.2** | **3.65** | 큰 이동 |
| **평균** | **28.3** | **1.13** | - |

### 관측 간 거리 분포

```
최소 이동: 0.0 pixels (정지)
최대 이동: 91.2 pixels
평균 이동: 28.3 pixels (1.13 meters)
중앙값: 11.0 pixels (0.44 meters)
```

---

## 📐 참고: 스케일 변환

### Pixel ↔ Meter 변환

```
1 pixel = 0.04 meters = 4 cm
1 meter = 25 pixels

예시:
- 10 pixels = 0.4 m = 40 cm
- 50 pixels = 2.0 m
- 100 pixels = 4.0 m
```

### 실제 소스까지의 거리

시작 위치 (128, 128)에서:

| 소스 | 위치 | 거리 (px) | 거리 (m) |
|------|------|-----------|----------|
| Source 1 | (183, 43) | **101** | **4.04** |
| Source 2 | (44, 66) | **104** | **4.18** |
| Source 3 | (186, 172) | **73** | **2.91** |

**로봇이 4번 반복 동안 이동한 거리 (113px)**로는:
- Source 1, 2 근처까지 갈 수 있음
- 하지만 실제로는 전혀 다른 방향으로 이동함

---

## ✅ 개선 방안

### 1. 최소 이동 거리 강제

```python
if actual_movement < min_movement_threshold:
    # 강제로 먼 곳으로 이동
    random_far_position = sample_far_position(current_pos, min_dist=50)
    move_to(random_far_position)
```

### 2. Gradient 기반 이동

```python
# 방사선 gradient 계산
gradient = compute_radiation_gradient(current_pos, observations)

# Gradient 방향으로 이동
next_pos = current_pos + step_size * gradient
```

### 3. Grid 탐색

```python
# Gain=0일 때 Grid 탐색으로 전환
if all_gains_zero:
    next_pos = next_grid_point()
```

---

## 📊 현재 vs 이상적 비교

| 항목 | 현재 | 이상적 |
|------|------|--------|
| 평균 이동 거리 | 28.3 px | **50-100 px** |
| 정지 횟수 | 2회 | **0회** |
| 탐색 커버율 | 4.4% | **30-50%** |
| 방향성 | 랜덤 | **Gradient 기반** |
| 총 이동 거리 | 113 px | **300-500 px** |

---

## 🔗 관련 문서

- [CONVERGENCE_PARADOX.md](CONVERGENCE_PARADOX.md) - RFC=1.0이지만 부정확한 이유
- [FIX_RESULTS.md](FIX_RESULTS.md) - RFC=0 문제 해결
