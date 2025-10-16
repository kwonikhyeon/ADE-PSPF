# ADE-PSPF Interactive Explorer

**Interactive Visualization Tool for Multi-Source Radiation Exploration**

이 도구는 ADE-PSPF(Adaptive Differential Evolution Particle Swarm Particle Filter) 알고리즘을 사용한 다중 방사선 소스 탐색 과정을 **단계별로 시각화**하는 인터랙티브 GUI 애플리케이션입니다.

---

## 📋 목차

1. [주요 기능](#주요-기능)
2. [시스템 요구사항](#시스템-요구사항)
3. [설치 방법](#설치-방법)
4. [사용 방법](#사용-방법)
5. [GUI 구성](#gui-구성)
6. [시각화 패널 설명](#시각화-패널-설명)
7. [고급 사용법](#고급-사용법)
8. [문제 해결](#문제-해결)

---

## 🎯 주요 기능

### ✅ 실시간 시각화
- **6개의 전문 패널**로 탐색 과정의 모든 측면 시각화
- Ground Truth, ADE-PSPF 추정, RRT 경로 계획, 실행 과정을 동시에 확인

### 🎮 인터랙티브 제어
- **Iteration 단위 탐색**: 슬라이더로 특정 iteration으로 이동
- **이전/다음 버튼**: 단계별로 진행 과정 확인
- **Auto-Play 모드**: 전체 탐색 과정을 애니메이션으로 재생

### 🔧 Ground Truth 제어
- **소스 개수 조정**: 1~10개의 방사선 소스 설정
- **Random Seed 제어**: 재현 가능한 실험 환경
- **즉시 재생성**: 버튼 클릭으로 새로운 GT 생성

### 💾 데이터 관리
- **세션 저장/로드**: 탐색 결과를 파일로 저장하고 나중에 다시 로드
- **이미지 Export**: 각 iteration을 개별 PNG 이미지로 저장
- **통계 요약**: 전체 탐색 과정의 통계 정보 확인

---

## 💻 시스템 요구사항

### Python 버전
- Python 3.8 이상 (3.10 권장)

### 필수 패키지
```
numpy
matplotlib
tkinter (Python 표준 라이브러리)
```

### 운영체제
- Linux (테스트 완료)
- macOS (호환 가능)
- Windows (호환 가능, tkinter 설치 필요)

---

## 📦 설치 방법

### 1. 프로젝트 디렉토리로 이동
```bash
cd /home/ikhyeon/research_ws/ade_pspf_repro
```

### 2. 필수 패키지 확인
대부분의 패키지는 이미 설치되어 있습니다. tkinter만 확인:

```bash
python3 -c "import tkinter; print('tkinter OK')"
```

만약 tkinter가 없다면:
```bash
# Ubuntu/Debian
sudo apt-get install python3-tk

# macOS (Homebrew)
brew install python-tk
```

### 3. 테스트 실행
```bash
cd visualization
python3 test_app.py
```

모든 테스트가 통과하면 설치 완료입니다!

---

## 🚀 사용 방법

### 기본 실행

```bash
cd visualization
python3 interactive_explorer_app.py
```

또는 프로젝트 루트에서:

```bash
python3 -m visualization.interactive_explorer_app
```

### 단계별 사용 가이드

#### 1️⃣ Ground Truth 생성
- **Sources**: 방사선 소스 개수 설정 (기본값: 3)
- **Seed**: Random seed 입력 (기본값: 42)
- **Generate New GT** 버튼 클릭

#### 2️⃣ 탐색 실행
- **Max Iterations**: 최대 iteration 수 설정 (기본값: 15)
- **Run Exploration** 버튼 클릭
- 탐색이 완료될 때까지 대기 (수십 초 ~ 몇 분)

#### 3️⃣ 결과 탐색
- **슬라이더**: 원하는 iteration으로 이동
- **이전/다음 버튼**: 단계별로 진행
- **Play 버튼**: 자동 재생 시작
- **Speed 슬라이더**: 재생 속도 조절 (0.1초 ~ 3.0초)

#### 4️⃣ 데이터 저장 (선택)
- **File > Save Session**: 현재 세션을 .pkl 파일로 저장
- **File > Load Session**: 저장된 세션 불러오기
- **File > Export Images**: 모든 iteration을 PNG로 저장

---

## 🖥️ GUI 구성

### 상단 - Control Panel
```
┌─────────────────────────────────────────────────────────┐
│ [Ground Truth 제어]        [Exploration 제어]           │
│ Sources: [3]  Seed: [42]   Max Iter: [15]               │
│ [Generate New GT]          [Run Exploration]  [Status]  │
└─────────────────────────────────────────────────────────┘
```

### 중앙 - 시각화 패널 (2x3 그리드)
```
┌──────────────┬──────────────┬──────────────┐
│ Panel 1      │ Panel 2      │ Panel 3      │
│ Ground Truth │ Estimation   │ Exploration  │
│ & Trajectory │ (ADE-PSPF)   │ (RRT)        │
├──────────────┼──────────────┼──────────────┤
│ Panel 4      │ Panel 5      │ Panel 6      │
│ Execution    │ Convergence  │ Statistics   │
│ (Movement)   │ (RFC Plot)   │ (Info)       │
└──────────────┴──────────────┴──────────────┘
```

### 하단 - Navigation Panel
```
┌─────────────────────────────────────────────────────────┐
│ [⏮ First] [◀ Prev]  [Slider: ===●===]  [Next ▶] [Last ⏭] │
│             Iteration: 5/15                              │
│ [▶ Play]  Speed: [──●───] 1.0s                          │
└─────────────────────────────────────────────────────────┘
```

---

## 📊 시각화 패널 설명

### Panel 1: Ground Truth & Trajectory
**표시 내용:**
- 🔥 Ground truth radiation field (heatmap)
- ⭐ True source positions (green stars)
- 🛤️ Robot trajectory (color-coded by time)
- ✖️ Observation locations (cyan crosses)
- 🔴 Current robot position

**용도:** 전체 탐색 경로와 실제 소스 위치 확인

---

### Panel 2: Estimation (ADE-PSPF)
**표시 내용:**
- 🔵 Particles by swarm (different colors per swarm)
- 🎯 Swarm centroids (colored circles)
- 🔺 Estimated source positions (red triangles)
- ⭐ True sources (green stars, for reference)
- 📊 RFC value and swarm count

**용도:** ADE-PSPF의 추정 과정과 particle 분포 확인

---

### Panel 3: Exploration (RRT Planning)
**표시 내용:**
- 🌳 RRT tree (gray nodes and edges)
- 🌈 Node colors by cumulative gain
- 🔴 Best branch (thick red line)
- ⭐ Leaf nodes (orange stars)
- 🔺 Estimated sources (blue triangles)

**용도:** RRT 경로 계획 과정과 gain 분포 확인

---

### Panel 4: Execution (Movement)
**표시 내容:**
- 🔵 Robot pose before (blue circle)
- 🔴 Robot pose after (red circle)
- ➡️ Heading arrows
- 🟢 Movement vector (green arrow)
- ⭐ New observations (green stars)
- 📏 Distance and node count info

**용도:** 실제 실행 과정과 움직임 상세 확인 (확대 뷰)

---

### Panel 5: Convergence (RFC Plot)
**표시 内容:**
- 📈 RFC vs iteration plot
- 🔴 Threshold line (0.85)
- 🟡🟠🟢 Phase transition markers
- 🔴 Current iteration highlight
- 🟢 Best RFC line

**용도:** RFC 수렴 과정과 exploration phase 변화 확인

---

### Panel 6: Statistics
**표시 内容:**
- 📋 Current iteration info
- 🔄 Exploration phase
- ⏱️ Timing information
- 📊 Cumulative statistics
- ✓ Convergence status

**용도:** 텍스트 기반 상세 통계 정보 확인

---

## 🔧 고급 사용법

### 1. 프로그래매틱 사용

Python 코드에서 controller를 직접 사용할 수 있습니다:

```python
from visualization.explorer_controller import ExplorerController
from visualization.data_manager import DataManager

# Controller 생성
controller = ExplorerController()

# Ground Truth 생성
controller.generate_ground_truth(n_sources=3, seed=42)

# Explorer 초기화
controller.initialize_explorer(
    max_iterations=10,
    n_swarms=4,
    n_particles=80
)

# 탐색 실행
controller.run_exploration_with_snapshots()

# Snapshot 가져오기
snapshot = controller.get_snapshot()

# 데이터 저장
data_manager = DataManager()
data_manager.save_snapshot(snapshot, 'my_session.pkl')
```

### 2. 커스텀 시각화

개별 패널을 사용하여 커스텀 시각화 생성:

```python
import matplotlib.pyplot as plt
from visualization.visualization_panels import GroundTruthPanel
from visualization.data_manager import DataManager

# Snapshot 로드
data_manager = DataManager()
snapshot = data_manager.load_snapshot('my_session.pkl')

# 커스텀 figure 생성
fig, ax = plt.subplots(figsize=(10, 10))
panel = GroundTruthPanel(ax)

# 특정 iteration 시각화
panel.update(snapshot, iteration=5, step='all')

plt.savefig('custom_visualization.png', dpi=150)
plt.show()
```

### 3. Batch Processing

여러 설정으로 탐색을 자동 실행:

```python
from visualization.explorer_controller import ExplorerController

controller = ExplorerController()

# 여러 설정 테스트
configs = [
    {'n_sources': 2, 'seed': 42},
    {'n_sources': 3, 'seed': 42},
    {'n_sources': 5, 'seed': 42},
]

for config in configs:
    controller.generate_ground_truth(**config)
    controller.initialize_explorer(max_iterations=15)
    controller.run_exploration_with_snapshots()

    snapshot = controller.get_snapshot()
    filename = f"session_n{config['n_sources']}_s{config['seed']}.pkl"
    data_manager.save_snapshot(snapshot, filename)
```

---

## ❓ 문제 해결

### Q1: "No module named 'tkinter'" 오류
**A:** tkinter를 설치하세요:
```bash
# Ubuntu/Debian
sudo apt-get install python3-tk

# macOS
brew install python-tk
```

### Q2: GUI가 매우 느리게 실행됩니다
**A:** 다음을 시도하세요:
1. Max Iterations를 줄이기 (예: 15 → 10)
2. 소스 개수를 줄이기 (예: 5 → 3)
3. 테스트 스크립트로 기능 확인 후 GUI 사용

### Q3: 탐색이 완료되지 않고 멈춥니다
**A:** 이는 정상적인 종료 조건입니다:
- RFC가 개선되지 않으면 10회 연속 후 자동 종료
- 또는 Max Iterations 도달 시 종료

### Q4: "Failed to generate GT" 오류
**A:** 다음을 확인하세요:
1. `environment/generate_truth.py`가 존재하는지
2. GRID 변수가 올바르게 정의되어 있는지
3. numpy가 설치되어 있는지

### Q5: 이미지 export가 실패합니다
**A:** 출력 디렉토리에 쓰기 권한이 있는지 확인하세요.

---

## 📚 추가 정보

### 알고리즘 상세
- **Estimation**: ADE-PSPF (Adaptive Differential Evolution Particle Swarm Particle Filter)
- **Exploration**: RRT (Rapidly-exploring Random Tree) with radiation gain
- **Execution**: Receding Horizon Framework (first edge only)

### 탐색 Phase
1. **TRACING** (RFC < 0.3): 의심 소스 추적
2. **SURROUNDING** (0.3 ≤ RFC < 0.85): 소스 주변 관측
3. **EXPLORING** (RFC ≥ 0.85): 미탐색 영역 탐색

### 종료 조건
- 10회 연속 RFC 개선 없음 (논문 방식)
- 또는 Max Iterations 도달

---

## 📝 파일 구조

```
visualization/
├── __init__.py                    # Package init
├── README.md                      # 이 문서
├── data_manager.py                # 데이터 구조 및 저장/로드
├── explorer_controller.py         # 탐색 실행 제어
├── visualization_panels.py        # 6개 시각화 패널
├── interactive_explorer_app.py    # 메인 GUI 앱
└── test_app.py                    # 테스트 스크립트
```

---

## 🎓 참고 논문

이 도구는 다음 논문의 알고리즘을 구현합니다:

"A study of robotic search strategy for multi-radiation sources in unknown environments"

---

## 📧 문의

버그 리포트나 기능 요청은 GitHub Issues를 통해 제출해주세요.

---

## 🎉 즐거운 시각화 되세요!

이 도구를 통해 복잡한 탐색 알고리즘의 동작을 직관적으로 이해할 수 있기를 바랍니다.
