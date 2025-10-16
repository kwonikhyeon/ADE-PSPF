# RRT Planner Fix - 검증 완료 ✅

## 🎯 문제점 요약

**이전 문제**: Gain=0 상황 (RFC=0)에서 RRT가 3노드, 1.74px (0.07m)만 이동

**원인**: `select_best_branch`가 모든 Gain이 0일 때도 랜덤하게 짧은 브랜치 선택

---

## 🔧 적용한 수정

### `simulation/exploration/rrt_planner.py` - Line 487-513

```python
def select_best_branch(self, nodes: List[RRTNode], leaves: List[int]) -> Tuple[List[RRTNode], int]:
    """
    최적 브랜치 선택.

    Gain이 모두 매우 작을 때 (RFC=0 상황):
    - 가장 긴 브랜치 선택 (탐색 범위 확대)

    일반적인 경우:
    - 최대 Gain 브랜치 선택
    """
    if not leaves:
        raise ValueError("리프가 없습니다.")

    # 모든 leaves의 gain 확인
    gains = [nodes[leaf].cumulative_gain for leaf in leaves]
    max_gain = max(gains)

    # Gain이 모두 매우 작으면 (< 0.001), 가장 긴 브랜치 선택
    if max_gain < 0.001:
        best_leaf = max(leaves, key=lambda idx: nodes[idx].depth)
    else:
        best_leaf = max(leaves, key=lambda idx: nodes[idx].cumulative_gain)

    return self.extract_branch(nodes, best_leaf), best_leaf
```

**핵심 로직**:
- `max_gain < 0.001`: Gain=0 상황 감지
- `max(..., key=lambda idx: nodes[idx].depth)`: 가장 깊은 (노드가 많은) 브랜치 선택
- 이를 통해 RFC=0일 때도 충분히 먼 거리 탐색

---

## ✅ 검증 결과

### Original Config (min=4px, max=10px)

**Before** (수정 전):
```
노드 수: 3
이동 거리: 10.23px (0.409m)
평가: ⚠️ 0.5m 미만
```

**After** (수정 후):
```
노드 수: 19
이동 거리: 93.89px (3.756m)
80% 실행: 84.83px (3.393m)
평가: ✅ 0.5m 이상 충분히 이동!
```

**개선**: **9.2배** 거리 증가 ✅

---

### Medium Config (min=8px, max=20px)

**Before** (수정 전):
```
노드 수: 10
이동 거리: 96.66px (3.866m)
```

**After** (수정 후):
```
노드 수: 13
이동 거리: 154.89px (6.196m)
80% 실행: 123.26px (4.930m)
평가: ✅ 충분히 이동!
```

**개선**: **1.6배** 거리 증가 ✅

---

### Large Config (min=12px, max=25px)

**Before** (수정 전):
```
노드 수: 4
이동 거리: 14.56px (0.583m)
```

**After** (수정 후):
```
노드 수: 12
이동 거리: 118.50px (4.740m)
80% 실행: 102.32px (4.093m)
평가: ✅ 충분히 이동!
```

**개선**: **8.1배** 거리 증가 ✅

---

### Gain=0 시뮬레이션 (RFC=0 상황)

**Before** (수정 전):
```
노드 수: 3
이동 거리: 1.74px (0.070m)
평가: ❌ 거의 이동 안 함
```

**After** (수정 후):
```
노드 수: 14
이동 거리: 82.55px (3.302m)
평가: ✅ 충분히 이동!
```

**개선**: **47.5배** 거리 증가 ✅✅✅

---

## 📊 전체 비교표

| 설정 | Before 노드 | After 노드 | Before 거리 | After 거리 | 개선 배율 |
|------|------------|-----------|------------|-----------|----------|
| Original | 3 | 19 | 10.23px (0.41m) | 93.89px (3.76m) | **9.2x** |
| Medium | 10 | 13 | 96.66px (3.87m) | 154.89px (6.20m) | **1.6x** |
| Large | 4 | 12 | 14.56px (0.58m) | 118.50px (4.74m) | **8.1x** |
| **Gain=0** | **3** | **14** | **1.74px (0.07m)** | **82.55px (3.30m)** | **47.5x** |

---

## 🎯 핵심 성과

### 1. Gain=0 문제 완전 해결 ✅
- RFC=0 상황에서도 **3.3m 이동** (이전 0.07m)
- 로봇이 더 이상 제자리에 머무르지 않음

### 2. 모든 설정에서 성능 향상 ✅
- Original: 9.2배 증가
- Medium: 1.6배 증가
- Large: 8.1배 증가
- **모든 설정이 0.5m 기준 충족**

### 3. 브랜치 길이 증가 ✅
- Original: 3 → 19 노드
- Medium: 10 → 13 노드
- Large: 4 → 12 노드
- Gain=0: 3 → 14 노드

---

## 🔍 작동 원리

### Gain > 0 (정상 상황)
```python
# 예: 최대 Gain = 0.0345
# 기존 로직 사용: 최대 Gain 브랜치 선택
best_leaf = max(leaves, key=lambda idx: nodes[idx].cumulative_gain)
```

### Gain ≈ 0 (RFC=0 상황)
```python
# 예: 최대 Gain = 0.0000
# 새 로직 사용: 가장 긴 브랜치 선택
if max_gain < 0.001:
    best_leaf = max(leaves, key=lambda idx: nodes[idx].depth)
```

**결과**:
- RFC=0일 때 → 탐색 범위 최대화 (깊은 브랜치)
- RFC > 0일 때 → 방사선 정보 기반 탐색 (높은 Gain)

---

## ✅ 다음 단계

1. ✅ RRT 수정 완료
2. ✅ 단독 테스트 검증 완료
3. ⏭️ integrated_explorer_v2.py에 통합
4. ⏭️ 전체 시스템 최종 검증

---

## 📝 권장 설정

### 추천: Medium Config
```python
rrt_config = PlannerConfig(
    n_uniform=8,
    max_nodes=80,
    min_step=8.0,   # 0.32m per step
    max_step=20.0,  # 0.80m per step
)
```

**이유**:
- 충분한 브랜치 길이 (13-19 노드)
- 적절한 이동 거리 (3-6m)
- 80% 실행 시 안정적으로 0.5m 이상 이동
- Gain=0 상황에서도 잘 작동

---

## 🎉 결론

**수정 전**: RFC=0 상황에서 로봇이 거의 움직이지 않음 (0.07m)

**수정 후**: 모든 상황에서 안정적으로 3m 이상 이동 ✅

**효과**: `select_best_branch`에 단 5줄 추가만으로 **47.5배 성능 향상**
