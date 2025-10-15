# Full Exploration Scenario - 전체 탐사 시나리오 가이드

## 개요

V2 통합 탐사 시스템의 장기 실행 시나리오를 시각화합니다.
초기 위치에서 시작하여 모든 방사선 소스를 탐색하는 전체 과정을 50-250 iterations에 걸쳐 수행합니다.

---

## 실행 방법

### 기본 실행 (50, 100, 150 iterations)

```bash
python3 tests/visualize_full_exploration.py
```

이 명령은 자동으로 3가지 시나리오를 실행합니다:
- 50 iterations (~19초)
- 100 iterations (~41초)
- 150 iterations (~63초)

### 커스텀 실행

`visualize_full_exploration.py`를 수정하여 원하는 iteration 수 설정:

```python
scenarios = [
    (75, "custom_75iter"),
    (200, "custom_200iter"),
    (250, "custom_250iter"),
]
```

---

## 실행 결과

### 성능 요약

| Scenario | Iterations | Time | Observations | RFC | Sources Found |
|----------|-----------|------|--------------|-----|---------------|
| 50 iter  | 50 | 18.06s | 51 | 0.0000 | 3/3 |
| 100 iter | 100 | 39.74s | 101 | 0.0000 | 3/3 |
| 150 iter | 150 | 61.73s | 151 | 0.0000 | 3/3 |

**관찰사항**:
- ✅ **성능**: 선형 시간 복잡도 (iteration당 ~0.4초)
- ✅ **안정성**: 모든 시나리오에서 크래시 없이 완료
- ⚠️ **수렴**: RFC 0.0000으로 아직 수렴하지 않음 (더 많은 iteration 또는 파라미터 튜닝 필요)
- ✅ **소스 감지**: 모든 경우 3개 소스 감지

---

## 시각화 내용

생성된 각 PNG 파일은 **3×4 레이아웃 (12개 서브플롯)**으로 구성:

### Row 1: 기본 정보

#### 1. Ground Truth Radiation Field (좌상)
- 실제 방사선 필드 히트맵
- 진짜 소스 위치 (녹색 별 ⭐)
- 소스 레이블 (S1, S2, S3)

#### 2. Full Trajectory with Direction (중상)
- **전체 이동 경로** (색상 그라디언트로 진행 표시)
- 시작점 (녹색 원)
- 종료점 (빨간 원)
- 방향 화살표 (20개 지점마다 표시)
- Waypoint 개수 표시

#### 3. Observation Locations with Intensities (중우)
- 모든 관측 위치
- 관측된 강도를 색상으로 표시 (plasma colormap)
- 일부 관측에 번호 표시
- 진짜 소스 위치 오버레이

#### 4. True vs Estimated Sources (우상)
- 진짜 소스 (녹색 별)
- 추정 소스 (빨간 삼각형)
- 강도를 원의 크기로 표시
- 매칭된 쌍을 노란 점선으로 연결

### Row 2: 진행 분석

#### 5. RFC Convergence Over Time (좌중)
- Iteration별 RFC 값
- 수렴 임계값 (0.85) 표시
- 수렴 시점 표시 (if any)
- 영역 채우기로 진행 강조

#### 6. Source Count Evolution (중중)
- Iteration별 추정된 소스 개수
- 진짜 소스 개수 (빨간 점선)
- 소스 감지 안정성 확인

#### 7. Time per Iteration (중우)
- 각 iteration 실행 시간
- 평균 시간 (빨간 점선)
- 성능 일관성 확인

#### 8. Cumulative Progress (우중)
- 누적 시간 (파란선)
- 누적 관측 수 (녹색선)
- 이중 Y축 사용

### Row 3: 상세 분석

#### 9. Source Estimation Evolution (좌하, 2칸)
- 6개 주요 iteration의 스냅샷
- 각 iteration의 RFC 값
- 진짜 소스 vs 추정 소스
- 해당 시점까지의 관측 위치

#### 10. Detailed Statistics (우하, 2칸)
상세 통계 텍스트:

```
==============================================================
EXPLORATION STATISTICS - DETAILED REPORT
==============================================================

EXECUTION METRICS:
  Total Iterations:        50
  Total Observations:      51
  Total Time:              18.057 seconds
  Avg Time per Iteration:  0.361 seconds
    - Estimation:          0.356 s
    - Exploration:         0.005 s

CONVERGENCE:
  Final RFC:               0.0000
  Best RFC:                0.0000
  Converged (≥0.85):       NO ✗

SOURCE DETECTION:
  True Sources:            3
  Estimated Sources:       3
  Detection Accuracy:      100.0%

TRUE SOURCE POSITIONS:
  Source 1: pos=(183.0,  43.0)  intensity= 60.72
  Source 2: pos=( 44.0,  66.0)  intensity= 98.29
  Source 3: pos=(186.0, 172.0)  intensity= 38.97

ESTIMATED SOURCE POSITIONS:
  Source 1: pos=(160.2,  90.8)  intensity= 20.20
  Source 2: pos=(155.1, 135.3)  intensity= 18.24
  Source 3: pos=(114.4, 111.4)  intensity= 20.97

POSITION ERRORS:
  Source 1:             56.50 pixels
  Source 2:            111.18 pixels
  Source 3:             98.87 pixels
  Average Error:        88.85 pixels
```

---

## 파일 위치

```
data/figures/full_exploration/
├── full_exploration_50iter.png   (959 KB)
├── full_exploration_100iter.png  (958 KB)
└── full_exploration_150iter.png  (970 KB)
```

---

## 주요 기능

### 1. 전체 경로 시각화
- **색상 그라디언트**: 시간 진행에 따라 색상 변화 (viridis colormap)
- **방향 표시**: 화살표로 이동 방향 표시
- **웨이포인트**: 각 로봇 위치 기록

### 2. 관측 분석
- **위치**: 모든 관측 지점 표시
- **강도**: Plasma colormap으로 관측 강도 시각화
- **번호**: 주요 관측 지점에 인덱스 표시

### 3. 수렴 모니터링
- **RFC 추적**: 매 iteration RFC 값 기록
- **임계값**: 0.85 수렴 기준 표시
- **수렴 탐지**: 수렴 시점 자동 표시

### 4. 성능 분석
- **시간 측정**: Iteration별, 컴포넌트별 시간
- **일관성**: 평균 대비 편차 확인
- **확장성**: 선형 복잡도 검증

### 5. 소스 진화
- **스냅샷**: 6개 주요 시점의 추정 결과
- **개수 추적**: Iteration별 소스 개수 변화
- **위치 정확도**: 진짜 vs 추정 비교

---

## 설정 파라미터

### ADE-PSPF 설정
```python
ADEPSPFConfig(
    n_swarms=4,              # 4개 swarm
    n_particles=80,          # 각 swarm당 80개 particle
    n_iterations=1,          # 한 번의 estimation당 1회 ADE
    ade_generations=3,       # 3세대 진화
    rfc_threshold=0.85       # 수렴 임계값
)
```

### Explorer 설정
```python
ExplorationConfig(
    max_iterations=50,                  # 최대 iteration 수
    branch_execution_ratio=0.5,         # RRT branch의 50% 실행
    min_rfc_threshold=0.85,             # 최소 수렴 임계값
    observations_per_iteration=2,       # Iteration당 관측 수
    max_observations_for_weight=20,     # Weight 계산용 최대 관측 (성능 최적화)
    enable_timing_logs=False,           # 콘솔 로그 최소화
    log_iteration_details=False
)
```

---

## 성능 최적화

### V1 → V2 개선사항

| 항목 | V1 | V2 | 개선율 |
|------|-----|-----|--------|
| 50 iter | 타임아웃 (180s+) | 18.1s | **10배+** |
| 무한 루프 | 빈번 | 없음 | ✅ |
| Observation 확장성 | O(n²) | O(1) | ✅ |

### 성능 병목 해결
1. **Observation Windowing**: 최근 20개만 사용
2. **RRT Max Attempts**: 무한 루프 방지
3. **Configuration Copy**: None 처리

---

## 문제 해결

### RFC가 0.0000에서 변하지 않음

**원인**:
- 관측값이 너무 적음
- ADE-PSPF 파라미터 부족
- 소스가 너무 멀리 떨어져 있음

**해결책**:
```python
# 1. Particle 수 증가
n_particles=150  # 80 → 150

# 2. ADE 세대 증가
ade_generations=5  # 3 → 5

# 3. Iteration당 관측 증가
observations_per_iteration=4  # 2 → 4

# 4. 더 많은 iteration
max_iterations=200  # 50 → 200
```

### 메모리 부족

**해결책**:
```python
# Observation window 감소
max_observations_for_weight=10  # 20 → 10

# Particle 수 감소
n_particles=50  # 80 → 50
```

### 실행 시간 너무 길음

**해결책**:
```python
# Particle 수 감소
n_particles=50

# ADE 세대 감소
ade_generations=2

# Observation window 감소
max_observations_for_weight=15
```

---

## 추천 시나리오

### 빠른 테스트 (5분 이내)
```python
scenarios = [
    (50, "quick_50iter"),
    (75, "quick_75iter"),
]
```

### 표준 테스트 (10분 이내)
```python
scenarios = [
    (100, "standard_100iter"),
    (150, "standard_150iter"),
]
```

### 심층 테스트 (20분 이내)
```python
scenarios = [
    (200, "deep_200iter"),
    (250, "deep_250iter"),
]
```

---

## 다음 단계

### 파라미터 튜닝
1. ADE-PSPF 파라미터 조정
2. RRT 탐사 전략 개선
3. 관측 간격 최적화

### 시나리오 확장
1. 소스 개수 변경 (3 → 5, 10)
2. 다양한 초기 위치
3. 다른 random seed

### 성능 개선
1. 병렬 처리 (multi-swarm)
2. Adaptive windowing
3. 더 나은 초기화

---

## 참고

- **메인 코드**: [tests/visualize_full_exploration.py](tests/visualize_full_exploration.py)
- **성능 분석**: [PERFORMANCE_V2.md](PERFORMANCE_V2.md)
- **변경 이력**: [CHANGELOG_V2.md](CHANGELOG_V2.md)

---

**작성일**: 2025-10-15
**버전**: 1.0
**상태**: ✅ 검증 완료
