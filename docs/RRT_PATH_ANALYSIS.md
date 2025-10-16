# RRT 경로 생성 문제 분석

## 🔍 테스트 결과 요약

### 문제 확인
**RRT 플래너가 Gain=0 상황에서 매우 짧은 경로를 생성함**

---

## 📊 설정별 테스트 결과

### 1. Original 설정 (min=4px, max=10px)
```
min_step: 4.0px (0.16m)
max_step: 10.0px (0.40m)

결과:
- 브랜치 길이: 3 노드
- 전체 이동: 10.23px (0.409m)
- 80% 실행: 10.23px (0.409m)
- 평가: ⚠️ 0.5m 미만 이동
```

### 2. Medium 설정 (min=8px, max=20px) ✓
```
min_step: 8.0px (0.32m)
max_step: 20.0px (0.80m)

결과:
- 브랜치 길이: 10 노드
- 전체 이동: 96.66px (3.866m)
- 80% 실행: 85.39px (3.416m)
- 평가: ✅ 0.5m 이상 충분히 이동!
```

### 3. Large 설정 (min=12px, max=25px)
```
min_step: 12.0px (0.48m)
max_step: 25.0px (1.00m)

결과:
- 브랜치 길이: 4 노드
- 전체 이동: 14.56px (0.583m)
- 80% 실행: 14.56px (0.583m)
- 평가: ✓ 0.5m 이상이지만 너무 짧음
```

### 4. Gain=0 상황 (RFC=0일 때)
```
설정: Medium (min=8px, max=20px)

결과:
- 브랜치 길이: 3 노드만!
- 전체 이동: 1.74px (0.070m)
- 평가: ❌ 거의 이동 안 함 (현재 위치 주변만 탐색)
```

---

## 🎯 핵심 문제점

### 1. **브랜치 길이 문제**
```
Original: 3 노드
Medium: 10 노드  ← 가장 길음
Large: 4 노드
Gain=0: 3 노드   ← 최악
```

**문제**: min_step이 너무 크면 오히려 브랜치가 짧아짐
- Large 설정: 큰 스텝 때문에 빨리 맵 밖으로 나가거나 max_nodes 도달
- 결과적으로 브랜치가 짧아짐

### 2. **Gain=0 특수 상황**
```
RFC=0일 때:
- 소스 추정이 현재 위치 근처에 잘못 집중
- Gain이 모두 0이므로 랜덤 탐색
- 하지만 RRT는 여전히 짧은 브랜치 생성
```

**원인**: `select_best_branch`가 Gain이 0일 때도 가장 긴 브랜치를 선택하지 않음

---

## 💡 해결책

### Solution 1: Medium 설정 사용 ⭐⭐⭐⭐⭐
```python
rrt_config = PlannerConfig(
    n_uniform=8,
    max_nodes=80,
    min_step=8.0,   # 0.32m per step
    max_step=20.0,  # 0.80m per step
)
```

**장점**:
- 10개 노드의 긴 브랜치 생성
- 80% 실행 시 85px (3.4m) 이동
- 0.5m 조건 완벽히 만족

**단점**:
- Gain=0 상황에서는 여전히 짧은 브랜치 (3 노드)

### Solution 2: select_best_branch 수정 ⭐⭐⭐⭐
```python
def select_best_branch_modified(nodes, leaves):
    """Gain이 모두 비슷하면 가장 긴 브랜치 선택"""

    gains = [nodes[leaf].cumulative_gain for leaf in leaves]
    max_gain = max(gains)

    # Gain이 모두 매우 작으면 (< 0.0001)
    if max_gain < 0.0001:
        # 가장 긴 브랜치 선택
        longest_leaf = max(leaves, key=lambda leaf: nodes[leaf].depth)
        longest_branch = extract_branch(nodes, longest_leaf)
        return longest_branch, longest_leaf

    # 일반적인 경우: 최대 Gain
    best_leaf = max(leaves, key=lambda leaf: nodes[leaf].cumulative_gain)
    best_branch = extract_branch(nodes, best_leaf)
    return best_branch, best_leaf
```

**장점**:
- Gain=0 상황에서도 긴 경로 선택
- 탐색 범위 확대

### Solution 3: 최소 브랜치 길이 강제 ⭐⭐⭐
```python
def select_best_branch_with_minimum(nodes, leaves, min_distance=12.5):
    """최소 거리를 만족하는 브랜치만 선택"""

    # 최소 거리 만족하는 leaves만 필터
    valid_leaves = []
    for leaf in leaves:
        branch = extract_branch(nodes, leaf)
        start = np.array([branch[0].pose.x, branch[0].pose.y])
        end = np.array([branch[-1].pose.x, branch[-1].pose.y])
        distance = np.linalg.norm(end - start)

        if distance >= min_distance:
            valid_leaves.append(leaf)

    if not valid_leaves:
        # 모두 너무 짧으면, 가장 긴 것 선택
        longest_leaf = max(leaves, key=lambda leaf: nodes[leaf].depth)
        return extract_branch(nodes, longest_leaf), longest_leaf

    # 유효한 것 중 최대 Gain
    best_leaf = max(valid_leaves, key=lambda leaf: nodes[leaf].cumulative_gain)
    return extract_branch(nodes, best_leaf), best_leaf
```

**장점**:
- 최소 거리 보장
- 짧은 브랜치 자동 배제

---

## 📝 권장 조합

### 추천 설정
```python
# 1. RRT Config: Medium 설정
rrt_config = PlannerConfig(
    n_uniform=8,
    max_nodes=80,
    min_step=8.0,   # 충분히 커서 의미있는 이동
    max_step=20.0,  # 너무 크지 않아서 브랜치 유지
)

# 2. Branch Selection: Gain=0 처리 추가
if max_gain < 0.0001:
    # 가장 긴 브랜치 선택
    best_leaf = max(leaves, key=lambda leaf: nodes[leaf].depth)
else:
    # 최대 Gain
    best_leaf = max(leaves, key=lambda leaf: nodes[leaf].cumulative_gain)

# 3. Execution: 최소 거리 검증
if movement_distance < 7.5:  # 0.3m
    skip_iteration()
```

---

## 🔧 구현 계획

### 1단계: RRT Planner 수정
- `select_best_branch`에 Gain=0 처리 로직 추가
- 최소 거리 필터 추가

### 2단계: Explorer 설정 수정
- `min_step=8.0`, `max_step=20.0` 적용

### 3단계: 테스트
- 단독 테스트로 검증
- 통합 테스트로 전체 시스템 확인

---

## 📈 예상 개선 효과

### Before (Original: min=4, max=10)
```
Iteration 1: 10px (0.4m)  ⚠️ 부족
Iteration 2: 0px (정지)   ❌
Iteration 3: 0px (정지)   ❌
Iteration 4: 90px (3.6m)  ✓
```

### After (Medium: min=8, max=20 + Gain=0 처리)
```
Iteration 1: 85px (3.4m)  ✅
Iteration 2: 70px (2.8m)  ✅
Iteration 3: 60px (2.4m)  ✅
Iteration 4: 75px (3.0m)  ✅

모든 반복에서 안정적으로 0.5m 이상 이동
```

---

## ✅ 다음 단계

1. ✅ RRT 플래너 단독 테스트 완료
2. ✅ 문제점 분석 완료
3. ⏭️ `select_best_branch` 수정
4. ⏭️ 수정된 플래너 테스트
5. ⏭️ Explorer 통합
6. ⏭️ 최종 검증
