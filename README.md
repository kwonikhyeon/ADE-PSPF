# ADE-PSPF Reproduction

**"A study of robotic search strategy for multi-radiation sources in unknown environments"** 논문 구현 프로젝트

## 📖 논문 정보

- **제목**: A study of robotic search strategy for multi-radiation sources in unknown environments
- **저자**: Hua Bai, Wenrui Gao, et al.
- **출처**: Robotics and Autonomous Systems 169 (2023) 104529
- **DOI**: 10.1016/j.robot.2023.104529

## 🎯 프로젝트 목표

미지의 환경에서 여러 방사선 소스를 자율적으로 탐색하는 로봇 전략 구현

### 핵심 알고리즘

1. **ADE-PSPF (Adaptive Differential Evolution - Peak Suppression Particle Filter)**
   - 지역 최적값 회피를 위한 적응적 차분 진화 알고리즘
   - 다중 방사선 소스 추정

2. **Radiation Gain Model**
   - 다중 소스 방사선 이득 모델
   - 미지 영역 탐색과 알려진 방사선 필드 활용의 균형

3. **Receding Horizon Planning**
   - Observation → Estimation → Exploration 반복

## 📂 프로젝트 구조

```
ade_pspf_repro/
├── config/              # 설정 파일
├── core/                # 핵심 알고리즘
│   ├── pspf.py         # Peak Suppression Particle Filter
│   ├── ade_optimizer.py # Adaptive Differential Evolution
│   └── ade_pspf.py     # ADE-PSPF 통합
├── data/                # 데이터 저장
│   ├── results/        # 실험 결과
│   ├── maps/           # 맵 데이터
│   └── figures/        # 시각화 결과
├── environment/         # 환경 시뮬레이션
│   ├── generate_truth.py  # Ground Truth 생성
│   └── observation.py     # 관측 시뮬레이션
├── experiments/         # 실험 스크립트
├── simulation/          # 시뮬레이션 파이프라인
├── tests/               # 테스트 코드
└── utils/               # 유틸리티 함수
```

## 🔧 환경 설정

### Ground Truth 생성

본 프로젝트는 `/home/ikhyeon/research_ws/coverage_path_planner`의 환경 설정을 따릅니다.

- **Grid Size**: 256 × 256 pixels (≈ 10m × 10m)
- **Pixel Resolution**: 0.04 m/pixel
- **Sources**: 1-4 개의 방사선 소스
- **Intensity Range**: 30.0 - 100.0 (MeV scale)
- **Spatial Spread (σ)**: 10.0 - 20.0 pixels

### Observation 모듈

실제 센서 대신 Ground Truth에서 직접 값을 읽는 시뮬레이션 방식:
- Poisson noise 미적용 (향후 추가 가능)
- Background radiation fluctuation 미적용
- 순수한 GT 값 기반 측정

## 🚀 사용법

### Ground Truth 생성

```python
from environment.generate_truth import sample_sources, inverse_square_field, GRID

# 소스 샘플링
coords, amps, sigmas = sample_sources(GRID, n=3)

# 방사선 필드 생성 (3D 역제곱 법칙 - ADE-PSPF와 일치)
gt_field = inverse_square_field(GRID, coords, amps, h=0.5)

# 레거시: Gaussian field (사용하지 않음)
# gt_field = gaussian_field(GRID, coords, amps, sigmas)
```

### 관측 시뮬레이션

```python
from environment.observation import RadiationObserver

# Observer 초기화
observer = RadiationObserver(gt_field, GRID)

# 단일 관측
intensity = observer.observe((128, 128))

# 배치 관측
positions = [(64, 64), (128, 128), (192, 192)]
intensities = observer.observe_batch(positions)

# 좌표 변환 (pixel ↔ meter)
from environment.observation import pixel_to_meter, meter_to_pixel

meter_pos = pixel_to_meter((128, 128))  # (5.0, 5.0) 미터
pixel_pos = meter_to_pixel((5.0, 5.0))  # (128, 128) 픽셀
```

### RRT 탐사 데모 (논문 Sec. 4)

```bash
python3 simulation/visualize_rrt.py
```

- RRT 트리, 후보 브랜치, 최적 브랜치를 시각화한 `data/figures/rrt_demo.png`가 생성된다.
- Gain 계산은 Sec. 4.2의 Eq. (11)~(22)를 그대로 구현해 사용한다.

### 통합 시스템 테스트

**최소 통합 테스트** (빠른 검증용):
```bash
python3 tests/test_minimal_integration.py
```
- Observation → Estimation → Exploration 전체 파이프라인 테스트
- 결과: `data/figures/integrated/minimal_integration_test.png`

**완전 통합 탐사 시스템** (Receding Horizon Framework):
```bash
# V2 기본 실행 (권장 - 자동 시각화 포함)
python3 simulation/integrated_explorer_v2.py
# 결과: data/figures/integrated_explorer_v2_result.png

# V2 간단한 시각화 (2, 5, 10 iterations)
python3 tests/visualize_v2_simple.py
# 결과: data/figures/v2_simple/*.png

# V2 전체 탐사 시나리오 시각화 (50, 100, 150 iterations)
python3 tests/visualize_full_exploration.py
# 결과: data/figures/full_exploration/*.png

# V1 (원본 버전)
python3 simulation/integrated_explorer.py
```
- 로봇이 반복적으로 관측-추정-탐사 사이클을 수행
- RFC 기반 수렴 판정
- **V2 개선사항**: 무한 루프 수정, 100배 성능 향상, 안정성 개선
- **자동 시각화**: 실행 후 자동으로 plot 생성 및 저장
- **간단 시각화**: `data/figures/v2_simple/` (기본 테스트)
- **전체 시각화**: `data/figures/full_exploration/` (장기 탐사 시나리오)
- 자세한 성능 분석: [PERFORMANCE_V2.md](PERFORMANCE_V2.md)

## 📊 구현 상태

- ✅ 논문 분석 및 시스템 구조 이해
- ✅ 폴더 구조 생성
- ✅ Environment 모듈 (generate_truth.py)
- ✅ Observation 모듈 (GT 기반 시뮬레이션)
- ✅ Estimation 모듈 - PSPF 구현 완료
- ✅ Estimation 모듈 - ADE 최적화 구현 완료
- ✅ Estimation 모듈 - ADE-PSPF 통합 완료
- ✅ Exploration 모듈 - RRT 경로 생성 (Eq. 8-10 구현)
- ✅ Exploration 모듈 - Radiation Gain Model (Eq. 11-22 구현)
- ✅ Exploration 모듈 - Gain Corrections 완료
- ✅ 전체 통합 - Receding Horizon Framework 구현 완료
- ✅ 통합 시스템 테스트 및 시각화 완료
- ✅ **V2 버그 수정 및 성능 최적화 완료** (2025-10-14)
  - RRT 무한 루프 수정
  - ADE-PSPF 성능 병목 현상 해결 (100배 개선)
  - Configuration.copy() 버그 수정
  - RRT Equation 17 누적 거리 계산 정확도 개선
- ✅ **RFC = 0 문제 완전 해결** (2025-10-15)
  - Ground Truth 물리 모델 수정 (Gaussian → Inverse Square)
  - 매개변수 최적화 (Particles, ADE Generations, Observations)
  - RFC 수렴 확인 (0.0000 → 1.0000 in 5 iterations)
  - 자동 시각화 생성
  - 상세 문서: [FIX_RESULTS.md](FIX_RESULTS.md)
- ✅ **RRT Gain=0 문제 해결** (2025-10-15)
  - Gain=0 상황에서 짧은 경로 생성 문제 해결 (0.07m → 3.30m, **47.5배 개선**)
  - `select_best_branch` 수정: 최대 Gain < 0.001일 때 가장 긴 브랜치 선택
  - 모든 RRT 설정에서 성능 향상 (9.2x ~ 47.5x)
  - 통합 시스템 정상 작동 확인
  - 상세 문서: [RRT_PATH_ANALYSIS.md](RRT_PATH_ANALYSIS.md), [RRT_INTEGRATION_SUCCESS.md](RRT_INTEGRATION_SUCCESS.md)

## 🧪 테스트

### 단위 테스트

각 모듈별 독립 테스트:
```bash
# ADE-PSPF 알고리즘 테스트 (시각화 포함)
python3 tests/test_ade_pspf_visual.py
# 결과: data/figures/*.png (Ground truth, 추정 결과, RFC 히스토리 등)

# RRT 경로 계획 테스트
python3 simulation/visualize_rrt.py
# 결과: data/figures/rrt_demo.png
```

### 통합 테스트

전체 시스템 통합 테스트:
```bash
# 최소 통합 테스트 (권장)
python3 tests/test_minimal_integration.py

# V2 성능 테스트 (빠름 - 1.68초)
python3 simulation/integrated_explorer_v2.py

# 완전 통합 테스트 (시간 소요)
python3 tests/test_integrated_system.py
```

**최신 성능 (2025-10-15 수정 후)**:
- **RFC 수렴**: 5회 반복만에 1.0000 달성 ✅
- **실행 시간**: 4.93초
- **총 관측 수**: 18
- **로봇 이동**: 74.4 pixels

**성능 비교**:
- V1: 타임아웃 (180초+) 또는 무한 루프
- V2 (수정 전): RFC = 0.0000, 로봇 정지
- V2 (수정 후): RFC = 1.0000, 정상 수렴 ✅
- V2: 10 iterations 완료 (1.68초) - **100배 향상**

## 📝 참고사항

### 측정값 읽기 방식

다른 파일에서 특정 지점의 방사선 값을 읽을 때는 항상 GT에서 직접 읽는 방식을 사용:

```python
# ✅ 올바른 방식
from environment.observation import RadiationObserver

observer = RadiationObserver(gt_field)
intensity = observer.observe(position)

# ❌ 잘못된 방식 (실제 센서 시뮬레이션 X)
# 실제 Poisson noise나 센서 에러를 추가하지 않음
```

### 논문 재현성

본 프로젝트는 논문의 모든 핵심 수식을 정확히 구현했습니다:

**Estimation (Section 3):**
- ✅ Eq. 1-3: Synthesized Weight 계산
- ✅ Eq. 4-6: ADE Mutation, Crossover, Selection
- ✅ Eq. 7: RFC 계산 및 Configuration Maintenance

**Exploration (Section 4):**
- ✅ Eq. 8-10: RRT 트리 구조 및 초기화
- ✅ Eq. 11-16: Radiation Gain Model
- ✅ Eq. 17: **누적 경로 거리** (V2에서 수정 완료)
- ✅ Eq. 18-22: 각종 보정 계수 (거리, 회전, 관측 강도, 중복 샘플링, 반복 탐사)

## 🔧 알려진 이슈 및 향후 과제

### V2에서 해결된 이슈
- ✅ RRT 무한 루프 (max_attempts 추가로 해결)
- ✅ ADE-PSPF 성능 병목 (observation windowing으로 해결)
- ✅ Configuration.copy() 크래시 (None 처리로 해결)
- ✅ RRT Equation 17 정확도 (cumulative_distance 추가)

### 향후 개선 방향
- [ ] Observation windowing 크기 자동 조정 (현재 고정값 20)
- [ ] RRT 샘플링 효율성 개선 (adaptive step size)
- [ ] RFC 수렴 속도 향상 (더 나은 초기화)
- [ ] 병렬 처리 도입 (multi-swarm 병렬화)
- [ ] 실시간 시각화 (matplotlib animation)
- [ ] 더 많은 소스 (4개 이상) 테스트 및 검증

## 📚 참고 자료

- 원본 논문: [Robotics and Autonomous Systems 169 (2023) 104529](https://doi.org/10.1016/j.robot.2023.104529)
- 참조 프로젝트: `/home/ikhyeon/research_ws/coverage_path_planner`

## 📜 라이선스

This is an academic reproduction project for research purposes.
