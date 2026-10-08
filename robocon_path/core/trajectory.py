"""最终轨迹点: 就是要点表里写的那五行数据.

约定(与参考工程完全一致)
------------------------
    {vx[mm/s], vy[mm/s], x[mm], y[mm], yaw[rad]}

* 首点、末点的速度必须为 0: 参考工程(Chassis2026_R1_H723/path_data.c 与
  2026R2_conbat/path_follower)都靠"速度为 0 的行"触发终点精定位;
* yaw 有两种写法, 由导出模式决定:
    - "tangent": 曲线切向航向(给纯跟踪/pure pursuit 当前馈参考)
    - "linear" : 起止航向之间按弧长线性插值(与 BR_McuBsplinePathGenerator 一致)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .geometry import Pose, wrap_pi


@dataclass
class TrajectoryPoint:
    vx: float = 0.0          # mm/s 世界系 X 前馈
    vy: float = 0.0          # mm/s 世界系 Y 前馈
    x: float = 0.0           # mm
    y: float = 0.0           # mm
    yaw: float = 0.0         # rad

    def as_row(self) -> Tuple[float, float, float, float, float]:
        return (self.vx, self.vy, self.x, self.y, self.yaw)


@dataclass
class TrajectoryMeta:
    """导出时写进注释头的信息, 方便回溯复现."""

    route_name: str = "TR_route_01"
    scene_name: str = ""
    map_image: str = ""
    resolution_mm: float = 25.0
    robot_radius_mm: float = 400.0
    safety_margin_mm: float = 60.0
    origin_mm: Tuple[float, float] = (0.0, 0.0)   # 点表原点在本场地系下的坐标
    start_pose: Pose = field(default_factory=Pose)
    goal_pose: Pose = field(default_factory=Pose)
    point_gap_mm: float = 100.0
    v_max_mps: float = 1.5
    a_max_mps2: float = 1.5
    j_max_mps3: float = 25.0
    a_lat_max_mps2: float = 1.2
    v_min_mps: float = 0.06
    heading_mode: str = "tangent"
    length_mm: float = 0.0
    min_radius_mm: float = float("inf")
    max_curvature: float = 0.0
    total_time_s: float = 0.0
    peak_v_mm_s: float = 0.0
    min_clearance_mm: float = 0.0
    astar_expanded: int = 0
    astar_length_mm: float = 0.0
    los_points: int = 0
    spline_points: int = 0
    spine_segments: int = 0
    notes: str = ""
    created_at: str = ""

    def to_dict(self) -> Dict[str, Any]:
        d = dict(self.__dict__)
        d["start_pose"] = list(self.start_pose.as_tuple())
        d["goal_pose"] = list(self.goal_pose.as_tuple())
        return d


@dataclass
class Trajectory:
    points: List[TrajectoryPoint] = field(default_factory=list)
    meta: TrajectoryMeta = field(default_factory=TrajectoryMeta)
    # 画图用的辅助曲线(可选)
    speed_s_mm: np.ndarray = field(default_factory=lambda: np.zeros(0))
    speed_v_mm_s: np.ndarray = field(default_factory=lambda: np.zeros(0))
    curvature: np.ndarray = field(default_factory=lambda: np.zeros(0))
    searched_cells: List[Tuple[int, int]] = field(default_factory=list)
    # 关键点(便于在画布上区分"平滑前/后")
    los_points: List[Tuple[float, float]] = field(default_factory=list)
    smooth_points: List[Tuple[float, float]] = field(default_factory=list)
    messages: List[str] = field(default_factory=list)
    # Retained access map revalidates every segment immediately before export.
    navigation_map: Any = field(default=None, repr=False)
    support_samples: List[Tuple[str, float]] = field(default_factory=list)

    @property
    def count(self) -> int:
        return len(self.points)

    def as_rows(self) -> List[Tuple[float, float, float, float, float]]:
        return [p.as_row() for p in self.points]
