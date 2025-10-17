# Interactive Explorer App 사용 가이드

## 개요

`interactive_explorer_app.py`는 논문 알고리즘을 시각화하는 GUI 애플리케이션입니다.
이제 **완전한 논문 구현**이 통합되어 있습니다:

- ✅ **ADE-PSPF Estimation** (Section 3)
- ✅ **RRT-based Exploration** (Section 4)
- ✅ **Dynamic Swarm Adjustment** (Fig. 11) ⭐ 신규
- ✅ **Phase Transitions** (Fig. 14-16) ⭐ 신규
- ✅ **Paper-compliant Termination** (Section 5) ⭐ 신규

---

## 실행 방법

### 1. 기본 실행

```bash
cd /home/ikhyeon/research_ws/ade_pspf_repro
python3 visualization/interactive_explorer_app.py
```

### 2. GUI 조작

#### 2.1 Ground Truth 생성
1. **Sources**: 소스 개수 설정 (1-10)
2. **Seed**: 랜덤 시드 (재현성)
3. **"Generate New GT"** 버튼 클릭

#### 2.2 Exploration 실행
1. **Max Iterations**: 최대 탐색 횟수 설정 (기본: 15)
2. **"Run Exploration"** 버튼 클릭
3. 실시간으로 진행 상황 표시됨

#### 2.3 결과 확인 (6개 패널)

**상단 행**:
- **Ground Truth**: 실제 방사선장 (빨간색 = 높은 강도)
- **Estimation**: 추정된 소스 위치 (파란색 점)
- **Exploration**: RRT 경로 계획 (초록색 = 선택된 경로)

**하단 행**:
- **Execution**: 로봇 궤적 및 관측 위치
- **Convergence**: RFC 수렴 그래프 (논문 기준)
- **Statistics**: 알고리즘 통계 (swarm 수, 소스 수 등)

#### 2.4 Navigation (하단 패널)
- **⏮ First / ◀ Prev / Next ▶ / Last ⏭**: 반복 단계별 이동
- **Slider**: 특정 iteration으로 이동
- **▶ Play**: 자동 재생 (속도 조절 가능)

---

## 논문 알고리즘 동작 확인

### ✅ Dynamic Swarm Adjustment (Fig. 11)

탐색 중 **swarm 수가 자동으로 증가**하는지 확인:

1. **초기 설정**: Sources = 4, Max Iterations = 15
2. **Run Exploration** 실행
3. **터미널 출력 확인**:

```
======================================================================
🔧 SWARM ADJUSTMENT @ Iteration 6
======================================================================
  Swarms: 3 → 4 (+1)
  Reason: RFC very low (0.201 < 0.6)
  New Swarm ID: 3
======================================================================
```

4. **Statistics 패널**에서 "Swarms" 숫자가 증가하는지 확인

**예상 동작**:
- Iteration 6: 3 → 4 swarms
- Iteration 9: 4 → 5 swarms
- Iteration 12: 5 → 6 swarms

### ✅ Phase Transitions (Fig. 14-16)

RFC 값에 따라 **탐색 단계가 자동 전환**:

**터미널 출력**:
```
======================================================================
🔄 Phase Transition at Iteration 4
======================================================================
  TRACING → SURROUNDING
  RFC: 0.3152
  New phase: Surrounding observation (소스 주변 관측)
======================================================================
```

**단계별 특징**:
- **TRACING** (RFC < 0.3): 의심 소스 추적, 넓은 탐색
- **SURROUNDING** (0.3 ≤ RFC < 0.85): 소스 주변 집중 관측
- **EXPLORING** (RFC ≥ 0.85): 미탐색 영역 탐색

### ✅ Termination Criteria

논문 기준으로 자동 종료:

**종료 조건**:
1. **10 consecutive iterations without improvement**
2. **RFC threshold reached** (0.85 이상)
3. **Max iterations reached**

**터미널 출력**:
```
✓ Termination: No improvement for 10 iterations (Final RFC: 0.9378)
```

---

## 주요 기능 설명

### 1. Estimation Panel

**표시 내용**:
- 🔵 파란색 점: 추정된 소스 위치 (centroids)
- 🟡 작은 점: 파티클 분포 (각 swarm)
- 숫자: 추정된 강도 값

**확인 사항**:
- Swarm 수가 iteration마다 변하는지 (Dynamic Adjustment)
- Centroid가 실제 소스에 수렴하는지

### 2. Exploration Panel

**표시 내용**:
- 회색 선: RRT 트리 (후보 경로들)
- 🟢 초록색 선: 선택된 최적 경로
- 빨간 점: 목표 위치

**확인 사항**:
- 소스 주변으로 경로가 생성되는지 (Radiation Gain Model)
- Branch reuse 동작 확인 (이전 경로 재사용)

### 3. Convergence Panel

**표시 내용**:
- RFC 그래프 (0~1)
- 빨간 선: RFC threshold (0.85)
- 수직선: Phase transitions

**확인 사항**:
- RFC가 점진적으로 증가하는지
- 목표 threshold 달성 여부

### 4. Statistics Panel

**표시 내용**:
```
Iteration: 10/15
RFC: 0.7531
Sources: 4 → 5
Swarms: 4 → 6  ⭐ 동적 증가
Phase: SURROUNDING
```

**확인 사항**:
- Swarm 수 변화 (Dynamic Adjustment)
- Phase 변화
- Sources 수 변화

---

## 고급 사용법

### 1. 논문 시나리오 재현

#### Scenario 1: Three-source scenario (논문 Fig. 14)

```python
# GUI 설정:
Sources: 3
Seed: 42
Max Iterations: 15

# 기대 결과:
- Initial swarms: 4
- Final RFC: > 0.90
- Phase transitions: TRACING → SURROUNDING → EXPLORING
```

#### Scenario 2: Four-source scenario (논문 Fig. 16)

```python
# GUI 설정:
Sources: 4
Seed: 42
Max Iterations: 15

# 기대 결과:
- Initial swarms: 3 (insufficient)
- Dynamic adjustment: 3 → 4 → 5 → 6 swarms
- Final RFC: > 0.85
```

### 2. 저장 및 불러오기

#### 세션 저장
1. **File → Save Session...**
2. 파일 이름 입력 (예: `test_run.pkl`)
3. 모든 iteration 데이터 저장됨

#### 세션 불러오기
1. **File → Load Session...**
2. 저장된 `.pkl` 파일 선택
3. 저장된 상태에서 시작

#### 이미지 내보내기
1. **File → Export Images...**
2. 출력 폴더 선택
3. 모든 iteration을 개별 이미지로 저장

---

## 문제 해결

### Q1: "No valid path found" 오류

**원인**: RRT가 유효한 경로를 찾지 못함

**해결**:
- Max Iterations 증가
- Sources 수 확인 (너무 많으면 복잡도 증가)

### Q2: RFC가 수렴하지 않음

**원인**: Swarm 수 부족 또는 관측 부족

**해결**:
- Dynamic Swarm Adjustment가 동작하는지 확인 (터미널)
- Max Iterations를 20-30으로 증가
- Sources 수와 Initial Swarms 비율 확인

### Q3: 실행 속도가 느림

**원인**: 많은 파티클 또는 RRT 노드

**해결**:
- Particles per swarm 감소 (80 → 50)
- RRT max nodes 감소 (기본: 80)
- RRT V2 사용 확인 (자동 최적화)

### Q4: Swarm adjustment가 안 됨

**원인**: RFC가 threshold보다 높음

**확인**:
```python
# IntegratedExplorerV3 설정:
SwarmAdjustmentConfig(
    low_rfc_threshold=0.75,      # RFC < 0.75일 때 고려
    very_low_rfc_threshold=0.60, # RFC < 0.60일 때 즉시 추가
    enabled=True                  # 활성화 확인
)
```

---

## 터미널 출력 해석

### 정상 실행 예시

```
======================================================================
Iteration 10/15
======================================================================

[STEP 1/3] Estimation (ADE-PSPF)...
  ✓ RFC: 0.7531, Sources: 4

🔧 SWARM ADJUSTMENT @ Iteration 10
  Swarms: 4 → 5 (+1)
  Reason: RFC low (0.753) and not improving

🔄 Phase Transition at Iteration 10
  TRACING → SURROUNDING
  RFC: 0.7531

[STEP 2/3] Exploration (RRT Planning)...
  ✓ RRT nodes: 80, Best branch: 5 nodes

[STEP 3/3] Execution (Movement)...
  ✓ Moved 15.2 pixels, Observations: 1

✓ Iteration 10 completed in 0.08s
```

**해석**:
1. ✅ Estimation 성공, RFC 증가
2. 🔧 Swarm 자동 추가 (Dynamic Adjustment)
3. 🔄 Phase 전환 (TRACING → SURROUNDING)
4. ✅ RRT 경로 계획 성공
5. ✅ 로봇 이동 및 관측 성공

---

## 최종 요약

### 실행 전 체크리스트

- [ ] Python 3.8+ 설치
- [ ] 필요 라이브러리 설치 (`requirements.txt`)
- [ ] Ground Truth 생성 완료
- [ ] Max Iterations 설정 (권장: 15)

### 논문 알고리즘 확인 사항

- [ ] Dynamic Swarm Adjustment 동작 (터미널 출력)
- [ ] Phase Transitions 발생 (터미널 출력)
- [ ] RFC 수렴 (Convergence 패널)
- [ ] 최종 RFC > 0.85 달성

### 성공적인 실행 기준

```
Final Results:
  ✓ Iterations: 15
  ✓ Final RFC: 0.9378 (> 0.85 threshold)
  ✓ Swarm adjustments: 2-3회
  ✓ Phase transitions: 2-3회
  ✓ Sources found: 정확도 높음
```

---

## 추가 정보

### 관련 파일

- **Main App**: `visualization/interactive_explorer_app.py`
- **Controller**: `visualization/explorer_controller.py`
- **Core Algorithm**: `simulation/integrated_explorer_v3.py`
- **Dynamic Adjustment**: `core/dynamic_swarm_adjustment.py`

### 테스트

```bash
# 통합 테스트
python3 tests/test_interactive_app_integration.py

# 논문 준수성 테스트
python3 tests/test_paper_compliance.py
```

### 문서

- **논문 준수 보고서**: `PAPER_COMPLIANCE_REPORT.md`
- **알고리즘 설명**: 각 모듈의 docstring 참조

---

**작성일**: 2025-10-17
**버전**: 1.0 (Full Paper Implementation)
**상태**: ✅ 검증 완료
