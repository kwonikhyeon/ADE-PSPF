"""
RRT 탐사 모듈 구현 (논문 Sec. 4).

논문 "A study of robotic search strategy for multi-radiation sources in unknown
environments" 의 Sec. 4.1~4.2 수식(Eqs. 8~22)을 코드화한 모듈이다.
관측·추정 모듈이 예측한 다중 방사선원을 입력으로 받아 균일 초기화,
이전 최적 브랜치 재사용, RRT 확장, 복합 Gain 계산(Radiation Gain + 보정)
까지 수행하고, 이득이 최대인 브랜치를 반환한다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

import math
import numpy as np

# ────────────────────────────────────────────────────────────────────
# 데이터 구조 정의
# ────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Pose2D:
    """랜덤 트리 노드의 자세 (Sec. 4.1, Eq. 10)."""

    x: float
    y: float
    theta: float  # 라디안

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=np.float64)


@dataclass
class RRTNode:
    """RRT 트리 노드."""

    pose: Pose2D
    parent: Optional[int]
    step_length: float
    cumulative_distance: float  # Eq. 17: 루트부터의 누적 경로 거리
    cumulative_gain: float
    node_gain: float
    depth: int


@dataclass(frozen=True)
class RectObstacle:
    """축정렬 사각형 장애물."""

    xmin: float
    xmax: float
    ymin: float
    ymax: float

    def contains_segment(self, p0: np.ndarray, p1: np.ndarray) -> bool:
        """선분 p0→p1가 사각형과 교차하는지 단순 충돌 검사."""
        # 선분이 사각형 내부를 통과하는지 Bresenham 대신 AABB 교차 검사
        # (충분조건) 선분 bbox와 사각형이 겹치고, 한 점이라도 내부에 있으면 충돌
        min_x, max_x = min(p0[0], p1[0]), max(p0[0], p1[0])
        min_y, max_y = min(p0[1], p1[1]), max(p0[1], p1[1])
        if max_x < self.xmin or min_x > self.xmax or max_y < self.ymin or min_y > self.ymax:
            return False

        # 사각형 모서리 중 하나라도 선분 방향과 교차 가능성이 있으면 충돌로 간주
        # 정확한 clipping 대신 간단한 OBB 포함체크로 충분
        corners = np.array(
            [
                [self.xmin, self.ymin],
                [self.xmin, self.ymax],
                [self.xmax, self.ymin],
                [self.xmax, self.ymax],
            ],
            dtype=np.float64,
        )
        if np.any(((corners[:, 0] >= min(p0[0], p1[0])) &
                   (corners[:, 0] <= max(p0[0], p1[0])) &
                   (corners[:, 1] >= min(p0[1], p1[1])) &
                   (corners[:, 1] <= max(p0[1], p1[1])))):
            return True

        # 선분이 사각형 안쪽을 완전히 관통하는 경우: 선분 중점 검사
        midpoint = 0.5 * (p0 + p1)
        if self.xmin <= midpoint[0] <= self.xmax and self.ymin <= midpoint[1] <= self.ymax:
            return True
        return False


@dataclass
class PlanarEnvironment:
    """평면 환경 정의."""

    bounds: Tuple[float, float, float, float]  # xmin, xmax, ymin, ymax
    obstacles: Sequence[RectObstacle] = field(default_factory=list)

    def is_inside(self, point: np.ndarray) -> bool:
        xmin, xmax, ymin, ymax = self.bounds
        return xmin <= point[0] <= xmax and ymin <= point[1] <= ymax

    def is_segment_collision_free(self, start: np.ndarray, end: np.ndarray) -> bool:
        if not self.is_inside(end):
            return False
        for obs in self.obstacles:
            if obs.contains_segment(start, end):
                return False
        return True


# ────────────────────────────────────────────────────────────────────
# 방사선 Gain 모델 (Sec. 4.2, Eqs. 11~22)
# ────────────────────────────────────────────────────────────────────


@dataclass
class GainParameters:
    """Gain 및 보정 파라미터 (논문 Eq. 11~22 참고)."""

    sigma_dist: float = 0.1  # σ_dist, 배경 이득
    r_src: float = 0.8       # r_src, Gaussian 폭 (m)
    d_peak: float = 2.0      # d_src^{gain}, 최적 관측 거리 (m)
    eta_src: float = 0.4     # Eq. 16, 거리 감쇠 계수
    sigma_theta: float = math.radians(45.0)  # Eq. 18, 회전 비용 스케일

    # Superposition suppression (Eq. 13, 14)
    sigma_r_dist: float = 0.2
    t_sup: float = 1.5
    s_sup: float = 0.5
    t_r_dist: float = 1.5
    s_r_dist: float = 0.5
    sigma_crt: float = 0.5

    # Observation intensity correction (Eq. 19)
    sigma_src_gain: float = 0.3
    t_src_gain: float = 1.2
    s_src_gain: float = 0.4
    i_background: float = 150.0  # nSv/h, Table 1 참고

    # Redundant sampling correction (Eq. 22)
    sigma_rs: float = 0.2
    s_rs: float = 1.0

    # Repeat exploring correction (Eq. 21)
    sigma_rex: float = 0.05
    s_rex: float = 1.5

    gain_sigma_dist_max: float = 1.0  # 이득 상한


class RadiationGainModel:
    """다중 방사선 Gain 및 보정 계산기."""

    def __init__(self, params: GainParameters):
        self.params = params

    def single_source_gain(self, node_pos: np.ndarray, source_pos: np.ndarray) -> float:
        """Eq. 11에 해당하는 단일 소스 이득."""
        d = np.linalg.norm(node_pos - source_pos)
        sigma_dist = self.params.sigma_dist
        r_src = self.params.r_src
        d_peak = self.params.d_peak
        prefactor = (1.0 - sigma_dist) / math.sqrt(2.0 * math.pi * r_src**2)
        gain_value = sigma_dist + prefactor * math.exp(-((d - d_peak) ** 2) / (2.0 * r_src**2))
        return min(gain_value, self.params.gain_sigma_dist_max)

    def suppression_factor(
        self,
        node_pos: np.ndarray,
        source_idx: int,
        sources: np.ndarray,
    ) -> float:
        """Eq. 13 기반 중첩 억제 계수."""
        if len(sources) <= 1:
            return 1.0

        params = self.params
        sigma_r_dist = params.sigma_r_dist
        t_sup = params.t_sup
        s_sup = params.s_sup
        t_r_dist = params.t_r_dist
        s_r_dist = params.s_r_dist
        sigma_crt = params.sigma_crt

        suppression = 1.0
        node = node_pos
        source = sources[source_idx]

        for j, other in enumerate(sources):
            if j == source_idx:
                continue
            midpoint = 0.5 * (source[:2] + other[:2])
            d_t = np.linalg.norm(node - midpoint)
            term = (1.0 - sigma_r_dist) / (1.0 + math.exp((t_sup - d_t) / s_sup))
            suppression *= sigma_r_dist + term

            d_ij = np.linalg.norm(source[:2] - other[:2])
            range_term = (1.0 - sigma_crt) / (1.0 + math.exp((t_r_dist - d_ij) / s_r_dist))
            suppression *= sigma_crt + range_term

        return suppression

    def distance_cost(self, step_length: float) -> float:
        """Eq. 16, 거리 비용."""
        return math.exp(-self.params.eta_src * step_length)

    def rotation_cost(self, theta_delta: float) -> float:
        """Eq. 18, 회전 비용."""
        sigma_theta = self.params.sigma_theta
        if sigma_theta <= 1e-9:
            return 1.0
        return math.exp(-(theta_delta ** 2) / (2.0 * sigma_theta ** 2))

    def observation_intensity_correction(self, nearest_intensity: float) -> float:
        """Eq. 19, 관측 강도 보정."""
        params = self.params
        ratio = nearest_intensity / max(params.i_background, 1e-6)
        logistic = 1.0 / (1.0 + math.exp((params.t_src_gain - ratio) / params.s_src_gain))
        return params.sigma_src_gain + (1.0 - params.sigma_src_gain) * logistic

    def redundant_sampling_correction(self, samples_near: int, distance_near: float) -> float:
        """Eq. 22, 중복 탐사 보정."""
        params = self.params
        numerator = math.exp(params.s_rs * distance_near - samples_near)
        denominator = numerator + math.exp(samples_near - params.s_rs * distance_near)
        return params.sigma_rs + (1.0 - params.sigma_rs) * (numerator / max(denominator, 1e-9))

    def repeat_exploring_correction(self, distances_to_observations: np.ndarray) -> float:
        """Eq. 21, 반복 탐사 보정."""
        if distances_to_observations.size == 0:
            return 1.0
        params = self.params
        factors = np.exp(-1.0 * (distances_to_observations / params.s_rex) + params.sigma_rex)
        return float(np.prod(factors))

    def compute_gain(
        self,
        node: np.ndarray,
        parent: Optional[np.ndarray],
        step_length: float,
        heading_change: float,
        sources: np.ndarray,
        nearest_intensity: float,
        samples_near: int,
        distance_near: float,
        distances_to_observations: np.ndarray,
    ) -> float:
        """Eq. 15~22 통합."""
        if len(sources) == 0:
            return 0.0

        # Eq. 12: 다중 소스 이득
        gains = []
        for idx, src in enumerate(sources):
            base_gain = self.single_source_gain(node, src[:2])
            suppression = self.suppression_factor(node, idx, sources)
            gains.append(base_gain * suppression)
        g_src = float(np.sum(gains))

        # 비용 및 보정
        c_dist = self.distance_cost(step_length)
        c_rot = self.rotation_cost(heading_change)
        c_obs = self.observation_intensity_correction(nearest_intensity)
        c_rs = self.redundant_sampling_correction(samples_near, distance_near)
        c_rex = self.repeat_exploring_correction(distances_to_observations)

        total_gain = g_src * c_dist * c_rot * c_obs * c_rs * c_rex
        return total_gain


# ────────────────────────────────────────────────────────────────────
# RRT Planner 구현
# ────────────────────────────────────────────────────────────────────


@dataclass
class PlannerConfig:
    """RRT Planner 설정."""

    n_uniform: int = 8
    max_nodes: int = 120
    min_step: float = 0.8
    max_step: float = 2.2
    random_seed: Optional[int] = None


class RRTPlanner:
    """
    논문 Sec. 4.1~4.2 기반 RRT 탐사 플래너.

    - Eq. 8: 균일 초기화 (n_uniform)
    - Eq. 9: 이전 최적 브랜치 재사용
    - Eq. 10: 새 노드의 자세 계산
    - Eqs. 11~22: Gain + 보정 계산
    """

    def __init__(
        self,
        environment: PlanarEnvironment,
        gain_model: RadiationGainModel,
        config: PlannerConfig | None = None,
    ):
        self.env = environment
        self.gain_model = gain_model
        self.config = config or PlannerConfig()
        self.rng = np.random.default_rng(self.config.random_seed)

    def _sample_free_space(self) -> np.ndarray:
        xmin, xmax, ymin, ymax = self.env.bounds
        for _ in range(1000):
            sample = np.array(
                [
                    self.rng.uniform(xmin, xmax),
                    self.rng.uniform(ymin, ymax),
                ],
                dtype=np.float64,
            )
            if self.env.is_inside(sample):
                return sample
        raise RuntimeError("Free space sampling 실패")

    def _generate_uniform_children(self, root_pose: Pose2D) -> List[RRTNode]:
        """Eq. 8: 균일하게 성장한 초기 분기."""
        nodes: List[RRTNode] = []
        n = self.config.n_uniform
        headings = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        base_heading = root_pose.theta

        for heading in headings:
            step = float(self.rng.uniform(self.config.min_step, self.config.max_step))
            direction = np.array([math.cos(heading + base_heading), math.sin(heading + base_heading)])
            new_point = root_pose.as_array() + step * direction
            if not self.env.is_segment_collision_free(root_pose.as_array(), new_point):
                continue

            pose = Pose2D(x=float(new_point[0]), y=float(new_point[1]), theta=float(heading + base_heading))
            node = RRTNode(
                pose=pose,
                parent=0,
                step_length=step,
                cumulative_distance=step,  # 루트부터의 누적 거리
                cumulative_gain=0.0,
                node_gain=0.0,
                depth=1,
            )
            nodes.append(node)
        return nodes

    def _reuse_best_branch(self, best_branch: Sequence[RRTNode]) -> List[RRTNode]:
        """Eq. 9: 이전 최적 브랜치 잔여 분기 추가."""
        nodes: List[RRTNode] = []
        for idx, node in enumerate(best_branch):
            if node.parent is None:
                continue
            new_node = RRTNode(
                pose=node.pose,
                parent=node.parent,
                step_length=node.step_length,
                cumulative_distance=node.cumulative_distance,
                cumulative_gain=node.cumulative_gain,
                node_gain=node.node_gain,
                depth=node.depth,
            )
            nodes.append(new_node)
        return nodes

    def _nearest_node(self, points: np.ndarray, query: np.ndarray) -> int:
        distances = np.linalg.norm(points - query[None, :], axis=1)
        return int(np.argmin(distances))

    def build_tree(
        self,
        root_pose: Pose2D,
        sources: np.ndarray,
        observation_positions: np.ndarray,
        observation_intensities: np.ndarray,
        nearest_intensity_lookup,
        previous_best_branch: Optional[List[RRTNode]] = None,
    ) -> Tuple[List[RRTNode], List[int]]:
        """
        RRT 트리 구축 및 최적 브랜치 탐색.

        Args:
            root_pose: 현재 로봇 자세
            sources: 예측된 소스 배열 (N_s × 3)
            observation_positions: 기존 관측 좌표 (N_o × 2)
            observation_intensities: 관측 강도 (N_o,)
            nearest_intensity_lookup: callable -> node pos ↦ (nearest_intensity, samples_near, distance_near)
            previous_best_branch: 이전 iteration 최고 브랜치 (루트 제외)
        Returns:
            (nodes 리스트, leaf 인덱스 후보)
        """
        nodes: List[RRTNode] = [RRTNode(pose=root_pose, parent=None, step_length=0.0,
                                        cumulative_distance=0.0, cumulative_gain=0.0,
                                        node_gain=0.0, depth=0)]

        nodes.extend(self._generate_uniform_children(root_pose))

        if previous_best_branch:
            nodes.extend(self._reuse_best_branch(previous_best_branch))

        # Add max attempts to prevent infinite loop
        max_attempts = self.config.max_nodes * 5  # Allow 5x attempts
        attempts = 0

        while len(nodes) < self.config.max_nodes and attempts < max_attempts:
            attempts += 1

            sample = self._sample_free_space()
            tree_points = np.array([n.pose.as_array() for n in nodes])
            nearest_idx = self._nearest_node(tree_points, sample)
            nearest_node = nodes[nearest_idx]

            direction = sample - nearest_node.pose.as_array()
            norm = np.linalg.norm(direction)
            if norm < 1e-6:
                continue

            direction /= norm
            step = float(self.rng.uniform(self.config.min_step, self.config.max_step))
            new_point = nearest_node.pose.as_array() + step * direction

            if not self.env.is_segment_collision_free(nearest_node.pose.as_array(), new_point):
                continue

            theta = math.atan2(direction[1], direction[0])
            pose = Pose2D(x=float(new_point[0]), y=float(new_point[1]), theta=float(theta))

            heading_change = _normalize_angle(theta - nearest_node.pose.theta)
            nearest_intensity, samples_near, distance_near = nearest_intensity_lookup(pose)

            # 관측 위치들과의 거리 (Eq. 21)
            if observation_positions.size > 0:
                distances = np.linalg.norm(observation_positions - pose.as_array()[None, :], axis=1)
            else:
                distances = np.array([], dtype=np.float64)

            # Eq. 17: 누적 경로 거리 계산
            cumulative_dist = nearest_node.cumulative_distance + step

            gain_value = self.gain_model.compute_gain(
                node=pose.as_array(),
                parent=nearest_node.pose.as_array(),
                step_length=cumulative_dist,  # Eq. 17: 누적 거리 사용
                heading_change=heading_change,
                sources=sources,
                nearest_intensity=nearest_intensity,
                samples_near=samples_near,
                distance_near=distance_near,
                distances_to_observations=distances,
            )

            new_node = RRTNode(
                pose=pose,
                parent=nearest_idx,
                step_length=step,
                cumulative_distance=cumulative_dist,  # Eq. 17: 누적 거리 저장
                cumulative_gain=nearest_node.cumulative_gain + gain_value,
                node_gain=gain_value,
                depth=nearest_node.depth + 1,
            )
            nodes.append(new_node)

        # Check if loop terminated early
        if attempts >= max_attempts:
            print(f"  ⚠ RRT: Reached max attempts ({max_attempts}), stopping with {len(nodes)} nodes")

        # 리프 후보: 자식이 없는 노드
        has_child = {i: False for i in range(len(nodes))}
        for idx, node in enumerate(nodes):
            if node.parent is not None:
                has_child[node.parent] = True
        leaves = [i for i, has_c in has_child.items() if not has_c]
        return nodes, leaves

    def extract_branch(self, nodes: List[RRTNode], leaf_idx: int) -> List[RRTNode]:
        branch: List[RRTNode] = []
        idx = leaf_idx
        while isinstance(idx, int):
            node = nodes[idx]
            branch.append(node)
            if node.parent is None:
                break
            idx = node.parent
        branch.reverse()
        return branch

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
        # 이는 RFC=0 상황에서 랜덤 탐색 시 더 넓은 범위를 탐색하기 위함
        if max_gain < 0.001:
            # 가장 깊은 (노드가 많은) 브랜치 선택
            best_leaf = max(leaves, key=lambda idx: nodes[idx].depth)
        else:
            # 일반적인 경우: 최대 Gain 선택
            best_leaf = max(leaves, key=lambda idx: nodes[idx].cumulative_gain)

        return self.extract_branch(nodes, best_leaf), best_leaf


def _normalize_angle(angle: float) -> float:
    """[-pi, pi] 범위로 정규화."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


__all__ = [
    "Pose2D",
    "RRTNode",
    "RectObstacle",
    "PlanarEnvironment",
    "GainParameters",
    "RadiationGainModel",
    "PlannerConfig",
    "RRTPlanner",
]
