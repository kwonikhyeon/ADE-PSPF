# 🚀 Quick Start Guide

## 빠른 시작 (3단계)

### 1️⃣ 테스트 실행

```bash
cd /home/ikhyeon/research_ws/ade_pspf_repro/visualization
python3 quick_test.py
```

모든 테스트가 통과하면 앱이 정상 작동합니다.

### 2️⃣ 앱 실행

```bash
python3 interactive_explorer_app.py
```

### 3️⃣ GUI에서 실행

1. **Generate New GT** 버튼 클릭 (또는 기본 GT 사용)
2. **Run Exploration** 버튼 클릭
3. 탐색이 완료되면 슬라이더로 iteration 탐색

---

## 📖 상세 사용법

### 기본 워크플로우

```
Ground Truth 생성 → Explorer 초기화 → 탐색 실행 → 결과 시각화
```

### Ground Truth 설정

- **Sources**: 방사선 소스 개수 (1~10)
  - 추천: 2~3개 (빠른 테스트)
  - 실제: 3~5개

- **Seed**: Random seed
  - 같은 seed = 같은 GT
  - 다른 seed = 다른 GT 배치

- **Generate New GT**: 새 GT 생성
  - 탐색 데이터는 초기화됨
  - 새로운 실험 시작

### 탐색 설정

- **Max Iterations**: 최대 반복 횟수
  - 테스트: 5~10
  - 실제: 15~30
  - 주의: 많을수록 시간 오래 걸림

- **Run Exploration**: 탐색 시작
  - 진행 중 GUI가 응답 없을 수 있음 (정상)
  - 완료되면 자동으로 iteration 0으로 이동

### Navigation 사용법

#### 슬라이더
- 마우스로 드래그하여 특정 iteration 선택
- 실시간으로 시각화 업데이트

#### 버튼
- **⏮ First**: 첫 번째 iteration
- **◀ Prev**: 이전 iteration
- **Next ▶**: 다음 iteration
- **Last ⏭**: 마지막 iteration

#### Auto-Play
- **▶ Play**: 자동 재생 시작
- **⏸ Pause**: 재생 일시정지
- **Speed 슬라이더**: 재생 속도 조절
  - 0.1초 (빠름) ~ 3.0초 (느림)

### 파일 작업

#### 세션 저장
1. **File > Save Session...**
2. 저장 위치 선택
3. .pkl 파일로 저장됨
4. 나중에 다시 불러올 수 있음

#### 세션 로드
1. **File > Load Session...**
2. .pkl 파일 선택
3. 전체 탐색 데이터 복원
4. GT 설정도 자동 복원

#### 이미지 Export
1. **File > Export Images...**
2. 출력 디렉토리 선택
3. 각 iteration이 PNG로 저장됨
4. 파일명: `iteration_001.png`, `iteration_002.png`, ...

---

## 🎨 시각화 패널 읽는 법

### Panel 1: Ground Truth & Trajectory
- **빨간 점**: 현재 로봇 위치
- **초록 점**: 시작 위치
- **색이 변하는 선**: 경로 (파란색→노란색 = 시간 순서)
- **파란 X**: 관측 위치
- **초록 별**: 실제 소스

**보는 법**: 로봇이 어디로 이동했는지 전체 경로 확인

---

### Panel 2: Estimation (ADE-PSPF)
- **작은 점들**: Particles (여러 색 = 여러 swarm)
- **큰 점**: Swarm 중심
- **빨간 삼각형**: 추정된 소스
- **빨간 원**: 추정된 강도 (원이 클수록 강함)
- **희미한 초록 별**: 실제 소스 (비교용)

**보는 법**: Particles가 소스 근처에 모이는지 확인

---

### Panel 3: Exploration (RRT)
- **회색 선과 점**: RRT tree
- **색이 있는 점**: Gain 값 (노란색 = 높음, 보라색 = 낮음)
- **굵은 빨간 선**: 선택된 최적 경로
- **주황 별**: Leaf 노드 (경로 끝점 후보)
- **파란 삼각형**: 추정된 소스 (참고)

**보는 법**: RRT가 어느 방향을 탐색하는지, 왜 그 경로를 선택했는지 확인

---

### Panel 4: Execution
- **파란 원**: 이동 전 위치
- **빨간 원**: 이동 후 위치
- **화살표**: 로봇 heading (방향)
- **초록 화살표**: 이동 벡터
- **초록 별**: 새로운 관측 위치
- **회색 점선**: 계획된 경로
- **파란 실선**: 실제 실행된 부분

**보는 법**: 실제로 어떻게 움직였는지 상세히 확인 (확대 뷰)

---

### Panel 5: Convergence
- **파란 선**: RFC 변화
- **빨간 점선**: Threshold (0.85)
- **초록 점선**: 최고 RFC
- **빨간 점**: 현재 iteration
- **색 막대 (옅음)**: Phase 변화
  - 노랑: TRACING
  - 주황: SURROUNDING
  - 초록: EXPLORING

**보는 법**: RFC가 수렴하는지, 어느 phase인지 확인

---

### Panel 6: Statistics
- **Phase**: 현재 탐색 단계
- **Estimation**: RFC, 소스 개수, Swarm 개수
- **Exploration**: RRT 노드 수, Gain 값
- **Execution**: 이동 거리, 새 관측 수
- **Cumulative**: 누적 통계
- **Status**: 수렴 여부

**보는 법**: 숫자로 상세 정보 확인

---

## 💡 팁과 트릭

### 빠른 테스트
```python
# Sources: 2, Seed: 42, Max Iterations: 5
# 약 30초 내 완료
```

### 실제 실험
```python
# Sources: 3-5, Seed: 원하는 값, Max Iterations: 15-20
# 수 분 소요
```

### 재현 가능한 실험
```python
# 같은 Seed를 사용하면 동일한 GT 생성
# 논문 재현이나 비교 실험에 유용
```

### 디버깅
```python
# 문제가 생기면:
# 1. quick_test.py 실행
# 2. 문제 확인
# 3. 설정 조정 (particles 줄이기, iterations 줄이기)
```

---

## ⚡ 성능 최적화

### 느릴 때
1. **Particles 줄이기**: 80 → 40
2. **Swarms 줄이기**: 4 → 2
3. **ADE generations 줄이기**: 3 → 1
4. **Max iterations 줄이기**: 15 → 10

### 정확도 높이려면
1. **Particles 늘리기**: 80 → 100
2. **ADE generations 늘리기**: 3 → 5
3. **Max iterations 늘리기**: 15 → 30

**Trade-off**: 정확도 ↔ 속도

---

## 🐛 자주 발생하는 문제

### Q: "No module named 'tkinter'"
```bash
sudo apt-get install python3-tk
```

### Q: GUI가 안 열림
```bash
# X11 forwarding이나 display가 필요
# 서버 환경이면 로컬에서 실행하거나 VNC 사용
```

### Q: 탐색이 너무 느림
```python
# quick_test.py로 기본 기능 확인
# Max Iterations를 5로 줄이기
# Particles를 40으로 줄이기
```

### Q: RFC가 개선 안 됨
```python
# 정상입니다
# 10회 연속 개선 없으면 자동 종료 (논문 방식)
```

### Q: 세션 파일이 너무 큼
```python
# 정상입니다 (.pkl 파일은 수십~수백 MB 가능)
# 모든 particle, node 데이터가 포함됨
```

---

## 📞 도움 받기

### 문제 확인 순서
1. `python3 quick_test.py` 실행
2. 에러 메시지 확인
3. README.md의 문제 해결 섹션 참고
4. GitHub Issues에 보고

### 버그 리포트 작성 시
- Python 버전
- OS 정보
- 에러 메시지 전체
- 재현 방법

---

## 🎓 학습 자료

### 알고리즘 이해하기
1. Panel 2 (Estimation)로 ADE-PSPF 작동 이해
2. Panel 3 (Exploration)로 RRT 경로 계획 이해
3. Panel 5 (Convergence)로 수렴 과정 이해

### 실험 아이디어
- 다른 소스 개수로 비교
- 다른 Seed로 다양한 배치 테스트
- RFC 변화 관찰

---

## 🎉 즐거운 실험 되세요!

더 궁금한 점은 README.md를 참고하세요.
