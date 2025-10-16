# 📊 ADE-PSPF 통합 시각화 앱

## 🎯 개요

**ADE-PSPF Interactive Explorer**는 다중 방사선 소스 탐색 알고리즘의 동작을 **단계별로 시각화**하는 인터랙티브 GUI 애플리케이션입니다.

이 앱을 통해 다음을 확인할 수 있습니다:
- ✅ **각 iteration의 상세 과정** (Estimation → Exploration → Execution)
- ✅ **ADE-PSPF의 particle 분포와 수렴 과정**
- ✅ **RRT 경로 계획과 gain 계산**
- ✅ **로봇의 실제 이동과 관측 수행**
- ✅ **RFC 수렴 과정과 exploration phase 변화**

---

## 🚀 빠른 시작

### 1. 테스트 실행
```bash
cd visualization
python3 quick_test.py
```

### 2. 앱 실행
```bash
python3 interactive_explorer_app.py
```

### 3. GUI에서 탐색 실행
1. **Generate New GT** 클릭 (선택 사항)
2. **Run Exploration** 클릭
3. 완료 후 슬라이더로 iteration 탐색

---

## 📁 파일 구조

```
visualization/
├── __init__.py                    # Package 초기화
├── README.md                      # 상세 문서
├── USAGE.md                       # 사용법 가이드
├── data_manager.py                # 데이터 구조 및 저장/로드
├── explorer_controller.py         # 탐색 실행 제어
├── visualization_panels.py        # 6개 시각화 패널 구현
├── interactive_explorer_app.py    # 메인 GUI 앱 (Tkinter + Matplotlib)
├── test_app.py                    # 전체 테스트 스크립트
└── quick_test.py                  # 빠른 기능 확인 스크립트
```

---

## 🎨 주요 기능

### 1. 6개 전문 패널

#### Panel 1: Ground Truth & Trajectory
- Ground truth radiation field
- 로봇 전체 이동 경로
- 관측 위치
- 실제 소스 위치

#### Panel 2: Estimation (ADE-PSPF)
- Particle 분포 (swarm별)
- Swarm centroids
- 추정된 소스 위치
- RFC 값

#### Panel 3: Exploration (RRT Planning)
- RRT tree 전체
- Gain 분포 (색상 코딩)
- Best branch (최적 경로)
- Leaf nodes

#### Panel 4: Execution (Movement)
- 이동 전/후 로봇 pose
- 실행된 경로 (확대 뷰)
- 새로운 관측 위치
- 이동 거리

#### Panel 5: Convergence (RFC Plot)
- RFC vs iteration
- Threshold line
- Phase 변화 마커
- 현재 iteration 표시

#### Panel 6: Statistics
- 현재 iteration 정보
- 각 step의 상세 통계
- 누적 통계
- 수렴 상태

### 2. 인터랙티브 제어

#### Navigation
- **슬라이더**: 특정 iteration으로 점프
- **이전/다음 버튼**: 단계별 이동
- **First/Last 버튼**: 처음/끝으로 이동
- **Auto-Play**: 애니메이션 재생 (속도 조절 가능)

#### Ground Truth 제어
- **Sources**: 소스 개수 설정 (1~10)
- **Seed**: Random seed 입력
- **Generate New GT**: 새로운 GT 생성

#### Exploration 제어
- **Max Iterations**: 최대 iteration 설정
- **Run Exploration**: 탐색 시작

### 3. 데이터 관리

#### 세션 저장/로드
- 전체 탐색 데이터를 .pkl 파일로 저장
- 나중에 다시 불러와서 분석 가능
- GT 설정도 함께 저장

#### 이미지 Export
- 각 iteration을 PNG 이미지로 저장
- 논문/프레젠테이션용 그림 생성

---

## 💻 기술 스택

### GUI Framework
- **Tkinter**: 기본 윈도우 및 컨트롤
- **Matplotlib**: 시각화 (6개 패널)
- **FigureCanvasTkAgg**: Tkinter-Matplotlib 통합

### 데이터 구조
- **Dataclasses**: 타입 안전한 데이터 구조
- **Numpy**: 고성능 수치 계산
- **Pickle**: 세션 저장/로드

### 아키텍처
```
┌─────────────────────────────────────┐
│  InteractiveExplorerApp (Main GUI) │
│  - Tkinter UI                       │
│  - Event handlers                   │
│  - Navigation controls              │
└──────────────┬──────────────────────┘
               │
       ┌───────┴───────┐
       │               │
┌──────▼──────┐  ┌────▼────────────────┐
│  Explorer   │  │ Visualization       │
│  Controller │  │ Panels (6개)        │
│  - GT gen   │  │ - GroundTruthPanel  │
│  - Explorer │  │ - EstimationPanel   │
│  - Snapshot │  │ - ExplorationPanel  │
└──────┬──────┘  │ - ExecutionPanel    │
       │         │ - ConvergencePanel  │
       │         │ - StatisticsPanel   │
       │         └─────────────────────┘
┌──────▼──────────────┐
│  IntegratedExplorerV2│
│  - ADE-PSPF         │
│  - RRT Planner      │
│  - Execution        │
└─────────────────────┘
```

---

## 📊 시각화 예시

### Iteration 진행 과정

```
Iteration 1:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Panel 1: 로봇이 시작 위치에서 첫 이동
Panel 2: Particles가 넓게 분산됨
Panel 3: RRT가 여러 방향 탐색
Panel 4: 첫 edge 실행, 1개 관측
Panel 5: RFC = 0.0000 (초기)
Panel 6: Phase = TRACING

Iteration 5:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Panel 1: 로봇이 소스 근처로 이동
Panel 2: Particles가 소스 주변에 모임
Panel 3: RRT가 다른 소스 방향 탐색
Panel 4: 소스 근처 관측 수행
Panel 5: RFC = 0.4523 (개선 중)
Panel 6: Phase = SURROUNDING

Iteration 10:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Panel 1: 여러 소스 방문 완료
Panel 2: 모든 소스 정확히 추정
Panel 3: RRT가 미탐색 영역 탐색
Panel 4: 빈 공간 확인 관측
Panel 5: RFC = 0.8821 (거의 수렴)
Panel 6: Phase = EXPLORING
```

---

## 🔬 사용 사례

### 1. 알고리즘 이해
- 각 단계의 동작 원리 학습
- Particle filter의 작동 방식 관찰
- RRT 경로 계획 과정 이해

### 2. 파라미터 튜닝
- 다양한 설정 실험
- Swarm/Particle 수 조정
- Max iterations 최적화

### 3. 논문 작성
- 각 iteration 스크린샷
- RFC 수렴 그래프
- 경로 비교 시각화

### 4. 디버깅
- 예상과 다른 동작 확인
- Particle 분포 이상 감지
- RRT gain 계산 검증

---

## ⚙️ 설정 가이드

### 빠른 테스트
```python
Sources: 2
Seed: 42
Max Iterations: 5
Swarms: 2
Particles: 40
→ 약 30초 완료
```

### 표준 실험
```python
Sources: 3
Seed: 42
Max Iterations: 15
Swarms: 4
Particles: 80
→ 수 분 소요
```

### 고정밀 실험
```python
Sources: 5
Seed: 원하는 값
Max Iterations: 30
Swarms: 6
Particles: 100
→ 10분 이상 소요
```

---

## 📚 참고 문서

- [README.md](visualization/README.md): 상세한 기능 설명 및 API
- [USAGE.md](visualization/USAGE.md): 단계별 사용 가이드
- 논문: "A study of robotic search strategy for multi-radiation sources in unknown environments"

---

## 🎯 주요 통찰

이 앱을 통해 다음을 발견할 수 있습니다:

1. **ADE-PSPF의 적응성**
   - Particles가 소스 발견 시 빠르게 수렴
   - Dynamic swarm adjustment로 누락 소스 탐지

2. **RRT의 균형**
   - Source gain과 exploration gain의 trade-off
   - 좋은 경로는 "가까운 소스 관측 + 새 영역 탐색"

3. **Receding Horizon의 이점**
   - 첫 edge만 실행하여 유연성 확보
   - 새 관측으로 더 나은 경로 재계획

4. **Phase 전환**
   - TRACING: 초기 빠른 소스 발견
   - SURROUNDING: 소스 주변 정밀 관측
   - EXPLORING: 전체 영역 커버리지 확보

---

## 🎓 학습 경로

### Level 1: 기본 이해
1. quick_test.py로 기능 확인
2. 기본 GT로 탐색 실행
3. 6개 패널 각각의 의미 파악

### Level 2: 알고리즘 분석
1. Estimation 패널에서 particle 수렴 관찰
2. Exploration 패널에서 gain 분포 분석
3. Convergence 패널에서 phase 변화 이해

### Level 3: 실험 설계
1. 다양한 소스 배치 실험
2. 파라미터 영향 분석
3. 세션 저장하여 비교

### Level 4: 커스터마이징
1. Python API로 프로그래매틱 사용
2. 커스텀 시각화 생성
3. Batch processing으로 대량 실험

---

## 🐛 문제 해결

### 앱이 시작되지 않음
```bash
# tkinter 설치
sudo apt-get install python3-tk

# 테스트 실행
python3 quick_test.py
```

### 탐색이 너무 느림
```python
# 파라미터 줄이기
Max Iterations: 15 → 5
Particles: 80 → 40
Swarms: 4 → 2
```

### GUI가 멈춘 것처럼 보임
```
정상입니다!
탐색 중에는 GUI가 응답 없을 수 있습니다.
완료되면 자동으로 복구됩니다.
```

---

## 🎉 시작하기

```bash
cd visualization
python3 quick_test.py           # 테스트
python3 interactive_explorer_app.py  # 실행
```

**즐거운 시각화 되세요!** 🚀

---

## 📧 문의

버그 리포트, 기능 제안, 질문은 GitHub Issues를 통해 제출해주세요.
