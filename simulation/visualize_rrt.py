"""
RRT 탐사 모듈 데모 및 시각화 스크립트.

논문 Sec. 4의 RRT + Radiation Gain 모델이 어떻게 작동하는지 확인하기 위해
간단한 환경과 예측 방사선원을 구성하고, 트리를 확장한 뒤 결과를
./data/figures/rrt_demo.png 에 저장한다.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys
from typing import Callable, List, Tuple

import numpy as np
import matplotlib.pyplot as plt

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from simulation.exploration.rrt_planner import (
    GainParameters,
    PlanarEnvironment,
    PlannerConfig,
    Pose2D,
    RadiationGainModel,
    RectObstacle,
    RRTNode,
    RRTPlanner,
)

FIGURE_PATH = ROOT_DIR / "data" / "figures" / "rrt_demo.png"


# ────────────────────────────────────────────────────────────────────
# 환경 및 관측 생성
# ────────────────────────────────────────────────────────────────────


def create_demo_environment() -> PlanarEnvironment:
    """시뮬레이션 환경 구성 (장애물 없음)."""
    bounds = (-5.0, 5.0, -5.0, 5.0)
    obstacles: List[RectObstacle] = []
    return PlanarEnvironment(bounds=bounds, obstacles=obstacles)


def sample_observations(
    sources: np.ndarray,
    rng: np.random.Generator,
    n_samples: int = 30,
    h: float = 0.5,
) -> Tuple[np.ndarray, np.ndarray]:
    """Eq. 1 기반으로 관측 위치와 강도를 샘플링."""
    observations: List[np.ndarray] = []
    intensities: List[float] = []

    def intensity_at(point: np.ndarray) -> float:
        total = 0.0
        for src in sources:
            dist = np.linalg.norm(point - src[:2])
            total += src[2] / (h**2 + dist**2)
        return total * 1e3  # 스케일링 (nSv/h 수준으로)

    for _ in range(n_samples):
        src_idx = rng.integers(0, len(sources))
        direction = rng.normal(size=2)
        direction /= np.linalg.norm(direction)
        radius = rng.uniform(0.8, 2.5)
        position = sources[src_idx, :2] + radius * direction
        observations.append(position)
        noisy_intensity = intensity_at(position) + rng.normal(0.0, 25.0) + 150.0
        intensities.append(max(noisy_intensity, 1.0))

    return np.vstack(observations), np.array(intensities)


def make_nearest_intensity_lookup(
    sources: np.ndarray,
    observation_positions: np.ndarray,
    observation_intensities: np.ndarray,
) -> Callable[[Pose2D], Tuple[float, int, float]]:
    """Eq. 19 & Eq. 22에 필요한 값을 반환하는 lookup 함수 생성."""
    def lookup(pose: Pose2D) -> Tuple[float, int, float]:
        pos = pose.as_array()

        if observation_positions.size == 0:
            nearest_intensity = 150.0
        else:
            dists = np.linalg.norm(observation_positions - pos[None, :], axis=1)
            idx = int(np.argmin(dists))
            nearest_intensity = float(observation_intensities[idx])

        if len(sources) == 0:
            return nearest_intensity, 0, 0.0

        dists_src = np.linalg.norm(sources[:, :2] - pos[None, :], axis=1)
        nearest_src_idx = int(np.argmin(dists_src))
        distance_near = float(dists_src[nearest_src_idx])
        nearby_obs = np.linalg.norm(
            observation_positions - sources[nearest_src_idx, :2],
            axis=1,
        )
        samples_near = int(np.sum(nearby_obs < 1.5))

        return nearest_intensity, samples_near, distance_near

    return lookup


# ────────────────────────────────────────────────────────────────────
# 시각화
# ────────────────────────────────────────────────────────────────────


def plot_rrt_result(
    environment: PlanarEnvironment,
    sources: np.ndarray,
    observations: np.ndarray,
    nodes: List[RRTNode],
    best_branch: List[RRTNode],
    output_path: Path,
) -> None:
    """트리, 장애물, 관측, 소스, 최적 브랜치를 시각화."""
    fig, ax = plt.subplots(figsize=(8, 8))

    xmin, xmax, ymin, ymax = environment.bounds
    ax.set_xlim(xmin - 0.5, xmax + 0.5)
    ax.set_ylim(ymin - 0.5, ymax + 0.5)
    ax.set_aspect("equal")
    ax.set_title("ADE-PSPF RRT 탐사 (Sec. 4)")

    # 트리 에지
    for idx, node in enumerate(nodes):
        if node.parent is None:
            continue
        parent = nodes[node.parent]
        xs = [parent.pose.x, node.pose.x]
        ys = [parent.pose.y, node.pose.y]
        ax.plot(xs, ys, color="lightsteelblue", linewidth=1.0, alpha=0.6)

    # 최적 브랜치 강조
    for prev, curr in zip(best_branch[:-1], best_branch[1:]):
        ax.plot(
            [prev.pose.x, curr.pose.x],
            [prev.pose.y, curr.pose.y],
            color="purple",
            linewidth=2.5,
        )

    # 관측 위치
    ax.scatter(
        observations[:, 0],
        observations[:, 1],
        c="lime",
        edgecolors="black",
        label="Observations",
        s=40,
    )

    # 소스 위치
    ax.scatter(
        sources[:, 0],
        sources[:, 1],
        c="red",
        marker="*",
        s=200,
        edgecolors="white",
        linewidths=1.5,
        label="Predicted Sources",
    )

    # 루트 및 종단 노드에 주석
    root = nodes[0]
    ax.scatter(root.pose.x, root.pose.y, c="royalblue", s=120, label="Robot")
    leaf = best_branch[-1]
    ax.scatter(leaf.pose.x, leaf.pose.y, c="purple", s=120, marker="^", label="Best Leaf")

    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.4)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"✓ RRT 시각화 저장: {output_path}")


# ────────────────────────────────────────────────────────────────────
# 실행
# ────────────────────────────────────────────────────────────────────


def main() -> None:
    rng = np.random.default_rng(42)

    environment = create_demo_environment()

    sources = np.array(
        [
            [2.5, 2.0, 950.0],
            [-1.2, 3.0, 1100.0],
            [0.5, -2.5, 1000.0],
        ],
        dtype=np.float64,
    )

    observation_positions, observation_intensities = sample_observations(sources, rng)

    gain_model = RadiationGainModel(GainParameters())
    planner = RRTPlanner(
        environment=environment,
        gain_model=gain_model,
        config=PlannerConfig(
            n_uniform=8,
            max_nodes=150,
            min_step=0.8,
            max_step=2.0,
            random_seed=24,
        ),
    )

    root_pose = Pose2D(x=-4.0, y=-4.0, theta=math.radians(45.0))

    lookup = make_nearest_intensity_lookup(
        sources=sources,
        observation_positions=observation_positions,
        observation_intensities=observation_intensities,
    )

    nodes, leaves = planner.build_tree(
        root_pose=root_pose,
        sources=sources,
        observation_positions=observation_positions,
        observation_intensities=observation_intensities,
        nearest_intensity_lookup=lookup,
        previous_best_branch=None,
    )
    best_branch, _ = planner.select_best_branch(nodes, leaves)

    # 시각화
    FIGURE_PATH.parent.mkdir(parents=True, exist_ok=True)
    plot_rrt_result(
        environment=environment,
        sources=sources,
        observations=observation_positions,
        nodes=nodes,
        best_branch=best_branch,
        output_path=FIGURE_PATH,
    )


if __name__ == "__main__":
    main()
