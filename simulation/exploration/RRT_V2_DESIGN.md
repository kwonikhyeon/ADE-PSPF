# RRT Planner V2 - Design Document

## 📋 목표

1. **논문 정확성**: Section 4.1-4.2, Equations 8-22를 정확히 구현
2. **성능 최적화**: 기존 대비 3-5배 속도 향상
3. **호환성**: 기존 인터페이스와 100% 호환
4. **유지보수성**: 명확한 문서화 및 테스트

---

## 📊 기존 코드 분석

### Input/Output 인터페이스

#### **입력 (`build_tree` 메서드)**
```python
def build_tree(
    root_pose: Pose2D,                      # 현재 로봇 위치
    sources: np.ndarray,                    # (N_s, 3) 추정된 소스들
    observation_positions: np.ndarray,      # (N_o, 2) 관측 위치들
    observation_intensities: np.ndarray,    # (N_o,) 관측 강도들
    nearest_intensity_lookup: Callable,     # 함수: Pose2D -> (intensity, samples, distance)
    previous_best_branch: Optional[List[RRTNode]] = None
) -> Tuple[List[RRTNode], List[int]]       # (노드 리스트, leaf 인덱스들)
```

#### **출력**
```python
# 1. nodes: List[RRTNode]
#    - RRT 트리의 모든 노드 (루트 포함)
#    - 각 노드는 parent 인덱스를 통해 연결
#
# 2. leaves: List[int]
#    - Leaf 노드의 인덱스들
#    - 경로 선택 시 이 중에서 선택

# select_best_branch() 사용
best_branch, best_leaf_idx = planner.select_best_branch(nodes, leaves)
# best_branch: List[RRTNode] - 루트부터 best leaf까지의 경로
```

#### **데이터 구조**
```python
@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float  # radians

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y])

@dataclass
class RRTNode:
    pose: Pose2D
    parent: Optional[int]           # 부모 노드 인덱스
    step_length: float              # 부모로부터의 거리
    cumulative_distance: float      # 루트부터의 누적 거리
    cumulative_gain: float          # 루트부터의 누적 gain
    node_gain: float                # 이 노드의 gain
    depth: int                      # 트리 깊이

@dataclass
class PlannerConfig:
    max_nodes: int = 80             # 최대 노드 수
    n_uniform: int = 8              # 균일 초기화 개수
    min_step: float = 8.0           # 최소 step (픽셀)
    max_step: float = 20.0          # 최대 step (픽셀)
    random_seed: int = 42
```

---

## 🐛 성능 병목 지점

### 1. **Nearest Neighbor 검색** (Line 366-410)
```python
# 현재: O(n) 선형 검색
tree_points = np.array([n.pose.as_array() for n in nodes])  # ⚠️ 매번 재생성
nearest_idx = self._nearest_node(tree_points, sample)       # ⚠️ O(n) 검색
```
**문제**:
- 80개 노드 × 80번 반복 = 6,400회 거리 계산
- 매번 배열 재생성

**해결책**:
- KD-Tree 사용: O(log n)
- 또는 점진적 배열 업데이트

### 2. **Observation Lookup** (Line 429)
```python
nearest_intensity, samples_near, distance_near = nearest_intensity_lookup(pose)
```
**문제**:
- 모든 observation과의 거리 계산 (O(n_obs))
- Iteration 15에서 15개 obs × 80 nodes = 1,200회

**해결책**:
- Observation windowing (최대 20개)
- KD-Tree for observations
- Caching

### 3. **Distance 계산 중복** (Line 432-435)
```python
if observation_positions.size > 0:
    distances = np.linalg.norm(observation_positions - pose.as_array()[None, :], axis=1)
```
**문제**:
- `nearest_intensity_lookup`에서 이미 계산했는데 또 계산

**해결책**:
- Lookup 결과 재사용
- 거리 배열 캐싱

### 4. **배열 복사 오버헤드**
```python
tree_points = np.array([n.pose.as_array() for n in nodes])  # 매번 생성
```
**문제**:
- 메모리 할당 및 복사 반복

**해결책**:
- Pre-allocated array + incremental update

---

## 🎯 V2 최적화 전략

### Strategy 1: **Spatial Indexing** (최우선)
```python
from scipy.spatial import cKDTree

class RRTPlannerV2:
    def __init__(self, ...):
        self._tree_points = np.zeros((max_nodes, 2))  # Pre-allocate
        self._n_nodes = 0
        self._kdtree = None

    def _add_node(self, node: RRTNode):
        self._tree_points[self._n_nodes] = node.pose.as_array()
        self._n_nodes += 1
        # Rebuild KD-tree periodically (every 10 nodes)
        if self._n_nodes % 10 == 0:
            self._kdtree = cKDTree(self._tree_points[:self._n_nodes])
```

**예상 개선**: O(n) → O(log n), **10-20배 속도 향상**

### Strategy 2: **Observation Windowing + Caching**
```python
class RRTPlannerV2:
    def build_tree(self, ...):
        # Limit observations
        MAX_OBS = 20
        if len(observation_positions) > MAX_OBS:
            obs_positions = observation_positions[-MAX_OBS:]
            obs_intensities = observation_intensities[-MAX_OBS:]

        # Build KD-Tree for observations (once per iteration)
        obs_tree = cKDTree(obs_positions) if len(obs_positions) > 0 else None

        # Cache lookup results
        self._obs_cache = {}
```

**예상 개선**: O(n_obs) → O(log n_obs), **5-10배 속도 향상**

### Strategy 3: **Vectorized Gain Computation**
```python
# 현재: Loop over sources
for i, source in enumerate(sources):
    gain += self.single_source_gain(node_pos, source[:2])

# V2: Vectorized
def compute_total_gain_vectorized(self, node_pos: np.ndarray, sources: np.ndarray):
    # sources: (N_s, 3)
    distances = np.linalg.norm(sources[:, :2] - node_pos[None, :], axis=1)
    gains = self._vectorized_single_source_gain(distances)
    return np.sum(gains)
```

**예상 개선**: **2-3배 속도 향상**

### Strategy 4: **Early Stopping**
```python
# Max gain에 도달하면 조기 종료
if len(leaves) > 0:
    best_gain = max(nodes[leaf].cumulative_gain for leaf in leaves)
    if best_gain > GAIN_THRESHOLD:  # 충분히 좋은 경로 발견
        break
```

**예상 개선**: 평균 **20-30% 노드 감소**

### Strategy 5: **Lazy Evaluation**
```python
# Gain 계산을 필요할 때만 수행
class LazyRRTNode:
    def __init__(self, ...):
        self._gain = None

    @property
    def node_gain(self):
        if self._gain is None:
            self._gain = self._compute_gain()
        return self._gain
```

**예상 개선**: **10-15% 속도 향상**

---

## 📐 V2 구현 계획

### Phase 1: 핵심 최적화 (필수)
1. ✅ Observation windowing (MAX_OBS=20)
2. ✅ KD-Tree for nearest neighbor search
3. ✅ KD-Tree for observation lookup
4. ✅ Pre-allocated arrays

### Phase 2: 알고리즘 최적화 (권장)
5. ✅ Vectorized gain computation
6. ✅ Early stopping
7. ✅ Distance caching

### Phase 3: 고급 최적화 (선택)
8. ⚠️ Lazy evaluation
9. ⚠️ Parallel gain computation
10. ⚠️ Adaptive sampling

---

## 🧪 테스트 전략

### 1. **Unit Tests**
```python
def test_nearest_neighbor():
    # KD-Tree vs Linear search 결과 비교

def test_gain_computation():
    # Vectorized vs Loop 결과 비교

def test_observation_windowing():
    # Windowed vs Full observations 비교
```

### 2. **Integration Tests**
```python
def test_build_tree_compatibility():
    # V1과 V2의 출력 형식 비교
    # 같은 입력 → 같은 형식의 출력

def test_with_explorer():
    # explorer_controller와 통합 테스트
```

### 3. **Performance Benchmarks**
```python
def benchmark_v1_vs_v2():
    # 1, 5, 10, 15 iterations 비교
    # Time per iteration
    # Memory usage
```

### 4. **Visual Verification**
```python
def test_paper_correctness():
    # 논문 Figure와 비교
    # Gain 분포 확인
    # 경로 선택 검증
```

---

## 📈 예상 성능 개선

### 현재 (V1)
```
Iteration  1: 0.064s
Iteration  5: 0.111s
Iteration 10: 0.274s
Iteration 15: 0.500s (예상)
Total: ~3.5s
```

### 목표 (V2)
```
Iteration  1: 0.020s  (3.2배 빠름)
Iteration  5: 0.025s  (4.4배 빠름)
Iteration 10: 0.030s  (9.1배 빠름)
Iteration 15: 0.035s  (14.3배 빠름)
Total: ~0.5s (7배 빠름)
```

### 최소 목표
```
Total: <1.5s (2.3배 이상 개선)
일정한 속도 유지 (max/min < 2.0)
```

---

## 🔧 구현 우선순위

### Priority 1 (Critical - 주말 내)
- [x] Observation windowing
- [ ] KD-Tree nearest neighbor
- [ ] Pre-allocated arrays
- [ ] Basic compatibility tests

### Priority 2 (High - 다음 주)
- [ ] KD-Tree observation lookup
- [ ] Vectorized gains
- [ ] Performance benchmarks
- [ ] Integration with explorer

### Priority 3 (Medium - 여유 있을 때)
- [ ] Early stopping
- [ ] Distance caching
- [ ] Visual verification
- [ ] Documentation

### Priority 4 (Low - 선택사항)
- [ ] Lazy evaluation
- [ ] Parallel processing
- [ ] Adaptive sampling

---

## 📝 API 호환성 체크리스트

- [ ] `Pose2D` 동일
- [ ] `RRTNode` 동일
- [ ] `PlannerConfig` 동일
- [ ] `build_tree()` 시그니처 동일
- [ ] `select_best_branch()` 시그니처 동일
- [ ] Return types 동일
- [ ] `explorer_controller.py` 수정 없이 작동

---

## 🎓 논문 방정식 체크리스트

### Equation 8: Uniform Initialization
- [ ] n_uniform개 균일 분포 생성
- [ ] 각 방향 2π/n_uniform

### Equation 9: Branch Reuse
- [ ] 이전 best branch 재사용
- [ ] Parent indices 올바르게 매핑

### Equation 10: Node Pose
- [ ] 새 노드 위치 계산
- [ ] Heading angle 계산

### Equations 11-16: Radiation Gain
- [ ] Single source gain (Eq. 11)
- [ ] Suppression factor (Eq. 13-14)
- [ ] Intensity correction (Eq. 15)
- [ ] Distance penalty (Eq. 16)

### Equations 17-19: Distance Gain
- [ ] Cumulative distance (Eq. 17)
- [ ] Heading change penalty (Eq. 18)
- [ ] g_dist 계산 (Eq. 19)

### Equations 20-22: Entropy Gain
- [ ] Repeat exploring correction (Eq. 21)
- [ ] Redundant sampling correction (Eq. 22)
- [ ] g_entropy 계산 (Eq. 20)

---

## 🚀 Next Steps

1. **지금 당장**: Observation windowing 확인 (이미 적용됨)
2. **오늘**: KD-Tree nearest neighbor 구현
3. **내일**: 전체 RRTPlannerV2 클래스 작성
4. **모레**: 테스트 및 벤치마크
5. **주말**: 통합 및 검증

---

## 📚 참고 자료

- **Original File**: `simulation/exploration/rrt_planner.py`
- **Usage**: `visualization/explorer_controller.py` line 522
- **Paper**: Section 4.1-4.2, Equations 8-22
- **Performance Analysis**: `visualization/PERFORMANCE_FIX.md`
