"""视线法(LOS)简化: 把 A* 的锯齿路径压成少量关键点.

与 2D/13_bspline_path_smoothing.py 里的 `simplify_path_by_line_of_sight` 同思路,
但碰撞判定交给 CostMap(按真实毫米采样), 不是按格子数:
栅格分辨率一变, 判定密度不会跟着变松/变紧.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from ..core.costmap import CostMap
from ..core.geometry import Point


def simplify_by_los(costmap: CostMap, path: Sequence[Point],
                    require_keep_clearance: bool = True) -> List[Point]:
    """贪心 LOS 简化.

    从起点出发, 每次尽量往后找"还能直线看到"的最远点, 中间点全部丢掉.
    复杂度 O(n^2) 最坏, 但 n 是 A* 路径点数, 且提前 break, 实际很快.
    """
    pts = [(float(p[0]), float(p[1])) for p in path]
    if len(pts) <= 2:
        return pts

    def visible(a: Point, b: Point) -> bool:
        if require_keep_clearance:
            return costmap.segment_is_free(a, b)
        return costmap.segment_is_free(a, b)

    out: List[Point] = [pts[0]]
    current = 0
    n = len(pts)
    while current < n - 1:
        nxt = n - 1
        while nxt > current + 1:
            if visible(pts[current], pts[nxt]):
                break
            nxt -= 1
        out.append(pts[nxt])
        current = nxt
    return out


def resample_polyline(path: Sequence[Point], step_mm: float) -> List[Point]:
    """把折线按固定弧长重采样(含首末点)."""
    pts = [(float(p[0]), float(p[1])) for p in path]
    if len(pts) < 2 or step_mm <= 0.0:
        return pts

    out: List[Point] = [pts[0]]
    carry = 0.0
    for i in range(1, len(pts)):
        a, b = pts[i - 1], pts[i]
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg <= 1e-12:
            continue
        t = 0.0
        while carry + (seg - t) >= step_mm:
            need = step_mm - carry
            t += need
            out.append((a[0] + (b[0] - a[0]) * (t / seg),
                        a[1] + (b[1] - a[1]) * (t / seg)))
            carry = 0.0
        carry += seg - t
    last = pts[-1]
    if math.hypot(last[0] - out[-1][0], last[1] - out[-1][1]) > 1e-6:
        out.append(last)
    return out


def prune_close_points(path: Sequence[Point], min_gap_mm: float) -> List[Point]:
    """去掉挨得太近的点(点表里太密的点对纯跟踪没意义, 只会占容量)."""
    pts = [(float(p[0]), float(p[1])) for p in path]
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    for p in pts[1:-1]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= min_gap_mm:
            out.append(p)
    out.append(pts[-1])
    return out
