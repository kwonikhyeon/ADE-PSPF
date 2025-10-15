# Repository Guidelines

## 연구 구현 목표
이 저장소의 유일한 목적은 `A study of robotic search strategy for multi-radiation sources in unknown environments.pdf`에 제시된 알고리즘, 실험 설정, 평가 절차를 가능한 한 그대로 재현하는 것이다. 모든 기여자는 논문에서 정의된 ADE-PSPF 파이프라인(관측·추정·탐사 루프)과 파라미터 테이블을 기준선으로 삼고, 코드나 데이터를 추가할 때 해당 근거를 명확히 남겨야 한다. 추가 가정이나 단순화를 도입했다면 주석과 문서에 근거 페이지 또는 식 번호를 명시한다.

## 프로젝트 구조 및 모듈 구성
`core/`는 ADE-PSPF 알고리즘 핵심 모듈로, 파티클 스웜, 적응형 차분 진화, RFC 계산을 책임진다. `environment/`는 논문에서 제시한 환경 가정을 반영한 진실 필드 생성과 관측 시뮬레이션을 포함한다. 실험 스크립트는 `experiments/`에 두고 `core.ade_pspf.ADEPSPF`를 직접 호출한다. 시각화와 결과물은 논문 Figure 재현을 위해 `data/figures`, 수치 로그는 `data/results`에 저장한다. 참고 문서는 `docs/`, 설정 템플릿은 `config/`, 공용 유틸은 `utils/`에 정리한다. 테스트는 `tests/`에 두고 모듈 이름을 반영한 파일명(`tests/test_<module>.py`)을 사용한다.

## 빌드·테스트·개발 명령
Python 3.10 이상과 동일한 NumPy/Matplotlib 스택을 사용한다.
```bash
python -m venv .venv && source .venv/bin/activate
pip install numpy matplotlib scipy
```
논문 Figure 5~7과 유사한 시각 검증을 위해 루트에서 다음을 실행한다.
```bash
python tests/test_ade_pspf_visual.py
```
주피터나 개별 스크립트를 사용할 때는 논문과 동일한 상대 경로 import를 위해 `export PYTHONPATH=$(pwd)`를 선행한다.

## 코딩 스타일 및 네이밍 규칙
모든 파이썬 코드는 PEP 8의 네 칸 들여쓰기와 타입 힌트를 지킨다. 파일·함수는 snake_case, 클래스는 CamelCase, 논문에서 가져온 상수는 대문자 스네이크로 표기한다(예: `RFC_THRESHOLD`). 복잡한 수식이나 논문 기호(θ, σ 등)는 주석으로 원문 식 번호를 덧붙인다. NumPy 연산을 우선 활용하고, 난수 발생 시에는 `numpy.random.Generator`를 주입 받아 재현성을 유지한다.

## 테스트 가이드라인
`tests/` 디렉터리의 스크립트는 논문의 실험 시나리오(소스 수, 관측 횟수, 반복 횟수)를 그대로 반영해야 한다. 새로운 테스트를 추가할 때는 `seed` 파라미터를 노출하고 실행 로그에 기록한다. 수치 검증은 `np.allclose` 등으로 허용 오차를 명시하며, 시각 결과물을 생성하면 `data/figures`에 저장 경로를 출력해 리뷰어가 논문 Figure와 대비할 수 있도록 한다.

## 커뮤니케이션
- 모든 에이전트–사용자 대화는 한국어로 진행한다.

## 커밋 및 PR 가이드라인
커밋 메시지는 `feat:`, `fix:`, `docs:` 등 명령형 태그를 사용하고 72자 이내로 요약한다. 본문에는 참조한 논문 페이지/식, 실험 설정, 생성된 산출물 경로(`data/figures/...`)를 기재한다. Pull Request에는 구현한 논문 섹션(예: Algorithm 1, Section 4.2), 사용한 하이퍼파라미터, 비교 그래프 또는 테이블을 첨부한다. 종속성 변경이나 설정값 수정 시 `docs/`와 `config/`의 대응 문서를 동시에 갱신해 논문 재현 목표를 유지한다.
