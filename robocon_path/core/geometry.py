"""几何图元: 点/线段/多边形/圆, 都用场地毫米坐标.

约定
----
所有几何量单位 mm, 角度 rad, 场地系为右手系且 y 轴向上(z 向上时逆时针为正 yaw).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

Point = Tuple[float, float]


# --------------------------------------------------------------------------- #
# 基础运算
# --------------------------------------------------------------------------- #
def wrap_pi(a: float) -> float:
    """把角度归一化到 (-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def dist(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def lerp(a: Point, b: Point, t: float) -> Point:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


# --------------------------------------------------------------------------- #
# 多边形
# --------------------------------------------------------------------------- #
def rect_polygon(cx: float, cy: float, w: float, h: float) -> List[Point]:
    """以 (cx, cy) 为中心、宽 w 高 h 的矩形(逆时针)."""
    hw, hh = w * 0.5, h * 0.5
    return [
        (cx - hw, cy - hh),
        (cx + hw, cy - hh),
        (cx + hw, cy + hh),
        (cx - hw, cy + hh),
    ]


def circle_polygon(cx: float, cy: float, r: float, segments: int = 48) -> List[Point]:
    """用正多边形近似圆, 用于栅格化."""
    segments = max(8, int(segments))
    return [
        (cx + r * math.cos(2.0 * math.pi * i / segments),
         cy + r * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def point_in_polygon(pt: Point, poly: Sequence[Point]) -> bool:
    """射线法. 边界上的点结果不保证, 但对栅格化够用."""
    x, y = pt
    inside = False
    n = len(poly)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            x_cross = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < x_cross:
                inside = not inside
        j = i
    return inside


def polygon_bbox(poly: Sequence[Point]) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


# --------------------------------------------------------------------------- #
# 线段 / 距离
# --------------------------------------------------------------------------- #
def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    """点 p 到线段 ab 的最短距离."""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq <= 1e-12:
        return dist(p, a)
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg_len_sq
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    proj = (ax + t * dx, ay + t * dy)
    return dist(p, proj)


def polyline_length(pts: Sequence[Point]) -> float:
    total = 0.0
    for i in range(1, len(pts)):
        total += dist(pts[i - 1], pts[i])
    return total


# --------------------------------------------------------------------------- #
# 形位
# --------------------------------------------------------------------------- #
@dataclass
class Pose:
    """场地系位姿, mm / rad."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    def to_m(self) -> Tuple[float, float, float]:
        return (self.x * 0.001, self.y * 0.001, self.yaw)

    @staticmethod
    def from_m(x_m: float, y_m: float, yaw_rad: float) -> "Pose":
        return Pose(x_m * 1000.0, y_m * 1000.0, yaw_rad)

    def copy(self) -> "Pose":
        return Pose(self.x, self.y, self.yaw)

    def is_finite(self) -> bool:
        return all(math.isfinite(v) for v in (self.x, self.y, self.yaw))


@dataclass
class Polygon:
    """凸/凹多边形障碍或参考区域."""

    points: List[Point] = field(default_factory=list)
    name: str = ""
    kind: str = "obstacle"  # obstacle | zone | reference
    color: str = ""

    def is_valid(self) -> bool:
        return len(self.points) >= 3

    def bbox(self) -> Tuple[float, float, float, float]:
        return polygon_bbox(self.points)


@dataclass
class Circle:
    """圆形障碍(如五色石基座)."""

    cx: float = 0.0
    cy: float = 0.0
    r: float = 0.0
    name: str = ""

    def to_polygon(self, segments: int = 48) -> Polygon:
        return Polygon(circle_polygon(self.cx, self.cy, self.r, segments),
                       name=self.name, kind="obstacle")

    def bbox(self) -> Tuple[float, float, float, float]:
        return (self.cx - self.r, self.cy - self.r, self.cx + self.r, self.cy + self.r)


@dataclass
class Wall:
    """线段型障碍(围栏/隔板), 比多边形省事."""

    x1: float = 0.0
    y1: float = 0.0
    x2: float = 0.0
    y2: float = 0.0
    thickness: float = 50.0
    name: str = ""

    def to_polygon(self) -> Polygon:
        return rect_from_segment((self.x1, self.y1), (self.x2, self.y2), self.thickness,
                                 name=self.name)

    def bbox(self) -> Tuple[float, float, float, float]:
        return self.to_polygon().bbox()


def rect_from_segment(a: Point, b: Point, thickness: float, name: str = "") -> Polygon:
    """把线段加厚成矩形多边形."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return Polygon([], name=name)
    nx, ny = -dy / length * thickness * 0.5, dx / length * thickness * 0.5
    return Polygon([
        (a[0] + nx, a[1] + ny),
        (b[0] + nx, b[1] + ny),
        (b[0] - nx, b[1] - ny),
        (a[0] - nx, a[1] - ny),
    ], name=name)
