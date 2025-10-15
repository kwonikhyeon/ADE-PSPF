# 로봇 경로 시각화 문제 분석

## 문제

시각화 이미지에서 로봇의 탐사 경로가 거의 보이지 않음.

## 원인 분석

### 1. RRT 모듈은 정상 작동 ✅
```
RRT Results:
  Total nodes: 80
  Leaf nodes: 23
  Best branch length: 3 nodes
  Total path distance: 14.19 pixels
```

### 2. 실제 문제: 로봇이 거의 움직이지 않음 ⚠️

**디버그 결과 (5 iterations)**:
```
Trajectory points: 5
Total distance: 8.82 pixels
Robot final position: (136.8, 126.9)
```

**반복되는 패턴**:
- Iteration 1: 이동 (128.0, 128.0) → (136.8, 126.9) ✓
- Iteration 2-5: **같은 위치 반복** (136.8, 126.9) ✗

### 3. 근본 원인: Gain 계산 실패

**모든 gain이 0.000000**:
```
Best branch: 2 nodes, gain=0.000021
Node 0: gain=0.000000
Node 1: gain=0.000000
```

**왜 Gain이 0인가?**
1. RFC = 0.0000 (소스 추정 실패)
2. 소스 추정이 없으면 → Radiation Gain이 0
3. Gain이 0이면 → 로봇이 어디로 가야 할지 모름
4. 결과: 같은 위치 반복

### 4. ADE-PSPF 수렴 실패

**50 iterations 결과**:
```
Final RFC: 0.0000
Best RFC: 0.0000
Converged: NO
Sources Found: 3/3 (하지만 위치가 부정확)
```

## 해결 방안

### 단기 해결책 (파라미터 튜닝)

#### 1. ADE-PSPF 파라미터 증가 ✅ (이미 적용)
```python
ADEPSPFConfig(
    n_swarms=4,
    n_particles=100,      # 80 → 100
    ade_generations=5,    # 3 → 5
)
```

#### 2. 탐사 설정 개선 ✅ (이미 적용)
```python
ExplorationConfig(
    branch_execution_ratio=0.8,          # 0.5 → 0.8
    observations_per_iteration=3,        # 2 → 3
)
```

#### 3. 추가 시도 가능

**A. 초기 Random Exploration 추가**:
```python
# RFC가 낮을 때는 random exploration
if rfc < 0.1:
    # Random direction 선택
    # 강제로 탐사 수행
```

**B. Gain Model 파라미터 조정**:
```python
GainParameters(
    sigma_dist=0.05,      # 0.1 → 0.05 (더 민감하게)
    eta_src=0.1,          # 0.3 → 0.1 (거리 페널티 감소)
    d_peak=5.0,           # 2.0 → 5.0 (peak distance 증가)
)
```

**C. 더 많은 초기 관측**:
```python
# 시작 시 grid pattern으로 관측
for dx, dy in [(10,0), (0,10), (-10,0), (0,-10)]:
    observe(start_x + dx, start_y + dy)
```

### 장기 해결책 (알고리즘 개선)

#### 1. Exploration-Exploitation 균형
```python
# Epsilon-greedy 방식
if random() < epsilon or rfc < threshold:
    # Random exploration
    explore_randomly()
else:
    # RRT gain-based exploration
    explore_by_rrt()
```

#### 2. Multi-stage Exploration
```python
# Stage 1: Wide exploration (gain 무시, 넓게 탐사)
# Stage 2: Focused exploration (gain 기반)
# Stage 3: Fine-tuning (RFC 수렴)
```

#### 3. Adaptive Parameters
```python
# RFC에 따라 파라미터 자동 조정
if rfc < 0.3:
    increase_particles()
    increase_exploration_range()
elif rfc < 0.6:
    normal_parameters()
else:
    focus_on_accuracy()
```

## 현재 상태

### 작동하는 것 ✅
- RRT 경로 생성 (80 nodes, 정상)
- ADE-PSPF 실행 (크래시 없음)
- 시각화 생성 (이미지 정상)
- 소스 감지 (3/3 감지)

### 작동하지 않는 것 ❌
- 소스 위치 추정 (RFC=0)
- 의미 있는 탐사 경로 (8.82 pixels / 50 iterations)
- Gain 계산 (모두 0)
- 수렴 (RFC 증가 없음)

## 테스트 결과

### 개선된 파라미터로 재테스트 필요

현재 적용된 개선사항:
- ✅ particles: 80 → 100
- ✅ generations: 3 → 5
- ✅ branch_execution_ratio: 0.5 → 0.8
- ✅ observations_per_iter: 2 → 3

**예상 효과**:
- RFC 증가 가능성
- 더 나은 소스 추정
- Gain 값 증가
- 실제 탐사 경로 생성

## 다음 단계

1. **즉시**: 개선된 파라미터로 full exploration 재실행
2. **단기**: Gain model 파라미터 튜닝
3. **중기**: Random exploration 추가
4. **장기**: Multi-stage exploration 구현

## 참고

- 디버그 스크립트: `tests/debug_trajectory.py`
- RRT 테스트: `tests/debug_rrt_detailed.py`
- 시각화: `tests/visualize_full_exploration.py`

---

**작성일**: 2025-10-15
**상태**: 분석 완료, 해결책 제시
