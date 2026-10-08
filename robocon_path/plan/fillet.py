"""圆弧倒角平滑: 把折线拐角换成与两条边相切的圆弧.

为什么优先用它, 而不是 B 样条
---------------------------
B 样条(哪怕加了控制点倒角)在**窄通道**里会去切内线, 实测直接切进障碍, 而且
想救回来只能不断缩小倒角量, 最后曲率还是达不到要求。圆弧倒角的几何是**可控**
的:

* 圆弧与两条边相切, 由转角 theta 与圆角半径 R 唯一确定, 曲率恒为 1/R ——
  正好就是底盘的最小转弯半径, 不多不少;
* 圆弧能不能放进这个角里, 可以解析判断(受两条边长限制), 也可以按需要
  **先验校验**(在 costmap 上逐点查离障);
* 校验不过就自动缩小 R; 实在放不下就退回原折点(并明确报出来), 绝不会偷偷
  产生一条车转不过去或者会撞的路径。

这比"先拟合再检查再重试"的 B 样条路线可靠得多, 也和舵轮底盘的物理最贴合
(定曲率圆弧 = 舵向角固定)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from ..core.costmap import CostMap
from ..core.geometry import Point
from .curvature import CurveGeometry, compute_geometry


@dataclass
class FilletResult:
    ok: bool
    points: List[Point] = field(default_factory=list)
    geometry: Optional[CurveGeometry] = None
    message: str = ""
    min_clearance_mm: float = float("inf")
    max_curvature: float = 0.0
    min_radius_mm: float = float("inf")
    corners_rounded: int = 0
    corners_total: int = 0
    corners_failed: int = 0

    @property
    def length_mm(self) -> float:
        return self.geometry.length_mm if self.geometry is not None else 0.0


def _dedup(points: Sequence[Point], eps: float = 1e-6) -> List[Point]:
    out: List[Point] = []
    for p in points:
        q = (float(p[0]), float(p[1]))
        if not out or math.hypot(q[0] - out[-1][0], q[1] - out[-1][1]) > eps:
            out.append(q)
    return out


def fillet_polyline(points: Sequence[Point],
                    radius_mm: float,
                    costmap: Optional[CostMap] = None,
                    arc_step_mm: float = 8.0,
                    min_clearance_mm: float = 0.0,
                    radius_tries: int = 5) -> FilletResult:
    """把折线的每个拐角换成相切圆弧.

    参数
    ----
    points          : LOS 简化后的关键点
    radius_mm       : 目标圆角半径(= 底盘最小转弯半径)
    costmap         : 用于校验圆弧是否贴障碍; None 则跳过
    arc_step_mm     : 圆弧离散化步长
    min_clearance_mm: 圆弧上每一点要求的最小离障距离
    radius_tries    : 某个角放不下时, 把 R 折半重试的次数
    """
    pts = _dedup(points)
    if len(pts) < 3:
        geo = compute_geometry(pts) if len(pts) >= 2 else None
        return FilletResult(bool(pts), pts, geo, message="关键点少于 3 个, 不做倒角")

    out: List[Point] = [pts[0]]
    rounded = 0
    failed = 0
    total_corners = len(pts) - 2

    for i in range(1, len(pts) - 1):
        prev_p, cur, next_p = pts[i - 1], pts[i], pts[i + 1]
        v1 = (cur[0] - prev_p[0], cur[1] - prev_p[1])
        v2 = (next_p[0] - cur[0], next_p[1] - cur[1])
        l1 = math.hypot(*v1)
        l2 = math.hypot(*v2)
        if l1 < 1e-6 or l2 < 1e-6:
            continue
        u1 = (v1[0] / l1, v1[1] / l1)      # 指向当前点
        u2 = (v2[0] / l2, v2[1] / l2)      # 背离当前点
        dot=max(-1.,min(1.,u1[0]*u2[0]+u1[1]*u2[1]))
        cross=u1[0]*u2[1]-u1[1]*u2[0]
        turn=math.atan2(abs(cross),dot)       # 0=straight, pi=U-turn
        theta=math.pi-turn                  # interior angle
        # 转角太小就不倒角(省点, 也避免数值噪声)
        if turn < 1e-8:
            out.append(cur)
            continue
        if math.pi-turn<1e-6:
            failed+=1
            out.append(cur)
            continue

        sin_half = math.sin(theta * 0.5)
        if sin_half < 1e-6:
            out.append(cur)
            continue
        # 切点到顶点的距离 t = R / tan(theta/2); 圆弧张角 = pi - theta
        cot_half = 1.0 / math.tan(theta * 0.5)

        # ⚠ 每个角按"可用边长"反推最大可行半径。
        # 不这么做的话, 相邻两个拐角如果只隔 100mm 量级(实测出现过 112mm 的双拐点),
        # 两个 R=800 的圆角会互相叠在一起, 路径先冲到角上再折回来, 曲率直接爆掉
        # (实测 R=6mm)。这里让每个角最多用掉相邻边的一半, 就不会打架。
        r_feasible = 0.5 * min(l1, l2) / cot_half
        r_target = min(radius_mm, r_feasible)
        if r_target <= 1.0:
            out.append(cur)
            continue

        best_arc: Optional[List[Point]] = None
        r_try = r_target
        for _ in range(max(1, radius_tries)):
            t = r_try * cot_half
            # 切点不能越过相邻点(留 2mm 余量)
            if t > l1 - 2.0 or t > l2 - 2.0:
                r_try *= 0.5
                continue
            a = (cur[0] - u1[0] * t, cur[1] - u1[1] * t)   # 入切点
            b = (cur[0] + u2[0] * t, cur[1] + u2[1] * t)   # 出切点
            # a->vertex is parallel to the incoming tangent. Its dot with a
            # normal is zero and cannot determine turn direction. Use cross.
            sign=1. if cross>0 else -1.
            nx,ny=-u1[1]*sign,u1[0]*sign
            center = (a[0] + nx * r_try, a[1] + ny * r_try)
            arc = _arc_points(center, r_try, a, b, arc_step_mm)
            if (arc and _arc_ok(arc, costmap, min_clearance_mm)
                    and (costmap is None or costmap.segment_is_free(out[-1],a))):
                best_arc = arc
                break
            r_try *= 0.5

        if best_arc is None:
            failed += 1
            out.append(cur)          # 放不下安全圆角: 退回原折点
        else:
            rounded += 1
            out.extend(best_arc)
    out.append(pts[-1])

    out=_dedup(out)
    geo = compute_geometry(out, smooth_window=3, curvature_span_mm=40.0)
    clearance = _min_clearance(costmap, out)
    need = min_clearance_mm
    ok = True
    notes: List[str] = []
    if costmap is not None and clearance < need:
        ok = False
        notes.append(f"倒角后离障 {clearance:.0f}mm < 要求 {need:.0f}mm")
    if costmap is not None and not costmap.polyline_is_free(out):
        ok=False
        notes.append("倒角连接线段不安全")
    if failed:
        notes.append(f"{failed}/{total_corners} 个拐角放不下安全圆角(已保留原折点)")
    message = "; ".join(notes) if notes else "OK"
    return FilletResult(ok, out, geo, message=message,
                        min_clearance_mm=float(clearance),
                        max_curvature=geo.max_abs_curvature,
                        min_radius_mm=geo.min_radius_mm(),
                        corners_rounded=rounded, corners_total=total_corners,
                        corners_failed=failed)


# --------------------------------------------------------------------------- #
def _perp_toward(u: Tuple[float, float], target: Point, base: Point
                 ) -> Tuple[float, float]:
    """返回"从 base 指向 target 那一侧的 u 的左法向"单位向量."""
    n = (-u[1], u[0])
    to_target = (target[0] - base[0], target[1] - base[1])
    if n[0] * to_target[0] + n[1] * to_target[1] < 0.0:
        n = (-n[0], -n[1])
    return n


def _arc_points(center: Point, radius: float, a: Point, b: Point,
                step_mm: float) -> List[Point]:
    """从 a 到 b 沿圆弧采样(方向取转角较小的那一边)."""
    a0 = math.atan2(a[1] - center[1], a[0] - center[0])
    a1 = math.atan2(b[1] - center[1], b[0] - center[0])
    d = a1 - a0
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    n = max(2, int(abs(d) * radius / max(1.0, step_mm)) + 1)
    pts: List[Point] = []
    for i in range(n + 1):
        ang = a0 + d * i / n
        pts.append((center[0] + radius * math.cos(ang),
                    center[1] + radius * math.sin(ang)))
    # 两端吸附到精确切点, 避免累积误差
    pts[0] = a
    pts[-1] = b
    return pts


def _arc_ok(arc: Sequence[Point], costmap: Optional[CostMap],
            min_clearance_mm: float) -> bool:
    if costmap is None:
        return True
    arr = np.asarray(arc, dtype=np.float64)
    clr = costmap.clearance_many_mm(arr[:, 0], arr[:, 1])
    return bool(np.all(clr >= min_clearance_mm)) and costmap.polyline_is_free(arc)


def _min_clearance(costmap: Optional[CostMap], pts: Sequence[Point]) -> float:
    if costmap is None:
        return float("inf")
    arr = np.asarray(pts, dtype=np.float64)
    return float(np.min(costmap.clearance_many_mm(arr[:, 0], arr[:, 1])))
