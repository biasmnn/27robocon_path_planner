"""点表导出: 统一的行数据 + CSV.

坐标平移
--------
导出的 x/y 会减去 `origin_mm`(TR 里程计原点在本场地系下的坐标), 这样
点表坐标就是"TR 自己坐标系下的坐标", 与参考工程 `path_data.c` 的用法一致。
"""

from __future__ import annotations

import csv
import io
import math
from types import SimpleNamespace
from typing import List, Optional, Sequence, Tuple

from ..core.trajectory import Trajectory, TrajectoryPoint
from ..core.file_io import atomic_text


def rows_for_export(traj: Trajectory,
                    origin_mm: Optional[Tuple[float, float]] = None,
                    ) -> List[Tuple[float, float, float, float, float]]:
    """返回导出用的 [vx, vy, x, y, yaw] 行, x/y 已按 origin 平移."""
    if traj.navigation_map is not None:
        traj.navigation_map.validate(traj.points)
    elif traj.meta.scene_name == "nvwa_butian_2027_from_sim":
        raise ValueError("仿真地图点表缺少物理通行校验，拒绝导出")
    ox, oy = origin_mm if origin_mm is not None else traj.meta.origin_mm
    rows: List[Tuple[float, float, float, float, float]] = []
    for p in traj.points:
        if not all(math.isfinite(v) for v in p.as_row()):
            raise ValueError("点表含非有限值，拒绝导出")
        rows.append((p.vx, p.vy, p.x - ox, p.y - oy, p.yaw))
    if traj.navigation_map is not None:
        # C/CSV serialize x/y at 0.001mm. Validate the actual rounded output,
        # translated back into the planner frame, before writing any file.
        traj.navigation_map.validate([SimpleNamespace(x=round(r[2],3)+ox,
            y=round(r[3],3)+oy) for r in rows])
    return rows


def trajectory_to_csv(traj: Trajectory,
                      origin_mm: Optional[Tuple[float, float]] = None,
                      with_header: bool = True) -> str:
    """CSV 文本: 序号/弧长/位置/航向/曲率/速度 + 末尾五个点表列."""
    buf = io.StringIO()
    writer = csv.writer(buf, lineterminator="\n")

    if with_header:
        m = traj.meta
        writer.writerow([f"# route={m.route_name}",
                         f"scene={m.scene_name}",
                         f"created={m.created_at}"])
        writer.writerow([f"# length_mm={m.length_mm:.1f}",
                         f"points={traj.count}",
                         f"gap_mm={m.point_gap_mm:.1f}",
                         f"est_time_s={m.total_time_s:.3f}",
                         f"v_max={m.v_max_mps}",
                         f"a_max={m.a_max_mps2}",
                         f"a_lat_max={m.a_lat_max_mps2}"])
        writer.writerow((["surface","support_z_nominal_mm"] if traj.navigation_map else [])+
                        ["index", "s_mm", "x_mm", "y_mm", "yaw_deg", "yaw_rad",
                         "curvature_1_mm", "radius_mm", "v_mm_s", "vx_mm_s", "vy_mm_s"])

    origin = origin_mm if origin_mm is not None else traj.meta.origin_mm
    rows = rows_for_export(traj, origin)

    # 弧长/曲率: 用元信息里的长度按索引线性近似(导出精度足够)
    n = max(1, len(rows) - 1)
    total = traj.meta.length_mm
    for i, (vx, vy, x, y, yaw) in enumerate(rows):
        s_mm = total * i / n
        kappa = float(traj.curvature[i]) if i < len(traj.curvature) else 0.0
        radius = (1.0 / abs(kappa)) if abs(kappa) > 1e-12 else float("inf")
        support=traj.navigation_map.support(traj.points[i].x,traj.points[i].y) if traj.navigation_map else None
        writer.writerow(([support[0],f"{support[1]:.3f}"] if support else [])+
                        [i, f"{s_mm:.2f}", f"{x:.3f}", f"{y:.3f}",
                         f"{yaw * 57.29577951308232:.3f}", f"{yaw:.6f}",
                         f"{kappa:.8f}", ("inf" if radius == float("inf") else f"{radius:.1f}"),
                         f"{((vx * vx + vy * vy) ** 0.5):.2f}", f"{vx:.3f}", f"{vy:.3f}"])
    return buf.getvalue()


def write_csv(path: str, traj: Trajectory,
              origin_mm: Optional[Tuple[float, float]] = None) -> None:
    atomic_text(path,trajectory_to_csv(traj,origin_mm),"utf-8-sig")
