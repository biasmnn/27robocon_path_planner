"""栅格通行图: 场景栅格化 + 障碍膨胀 + 碰撞查询.

关键点
------
* 障碍先按真实几何栅格化成 blocked 掩膜;
* 再用**精确欧氏距离变换**(scipy.ndimage.distance_transform_edt)对"到最近障碍的距离"
  做一次全局计算, 于是:
      可行 = dist_to_obstacle >= robot_radius + safety_margin
  这一步等价于"按车体外接圆半径做圆形膨胀", 但任意分辨率下都是毫秒级,
  不会像逐障碍双层循环那样随像素数爆炸.
* 因此对角线移动**不需要**额外的"禁止穿墙角"检查:
  只要两个相邻格子都满足圆膨胀约束, 斜穿的扫掠区域必然也是安全的.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage

from .geometry import Point, Pose
from .scene import Scene

try:  # Pillow 在依赖里, 但做个兜底避免 import 期就炸
    from PIL import Image, ImageDraw
except Exception:  # pragma: no cover
    Image = None  # type: ignore
    ImageDraw = None  # type: ignore


@dataclass
class GridSpec:
    """栅格参数(仅描述几何, 不持有数据)."""

    x_min: float
    y_min: float
    resolution: float
    width: int
    height: int

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        col = int(math.floor((x - self.x_min) / self.resolution))
        row = int(math.floor((y - self.y_min) / self.resolution))
        return row, col

    def cell_to_world(self, row: int, col: int) -> Point:
        return (self.x_min + (col + 0.5) * self.resolution,
                self.y_min + (row + 0.5) * self.resolution)

    def inside(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def extent(self) -> Tuple[float, float, float, float]:
        return (self.x_min, self.y_min,
                self.x_min + self.width * self.resolution,
                self.y_min + self.height * self.resolution)


class CostMap:
    """栅格化后的可行走地图."""

    def __init__(self, scene: Scene, resolution: Optional[float] = None):
        self.scene = scene
        res = float(resolution if resolution is not None else scene.resolution)
        if res <= 0.0:
            raise ValueError("resolution 必须为正")
        self.resolution = res

        x_min, y_min, x_max, y_max = scene.extent
        width = max(2, int(math.ceil((x_max - x_min) / res)))
        height = max(2, int(math.ceil((y_max - y_min) / res)))
        self.spec = GridSpec(x_min, y_min, res, width, height)

        self.blocked_radius = scene.blocked_radius()

        # 1) 障碍掩膜
        raw_blocked = self._rasterize(scene.obstacle_polygons())
        # 2) 到最近障碍的距离(mm), 精确欧氏。
        #    scipy 的 distance_transform_edt 对障碍格本身返回 0, 对紧邻的一格返回 1,
        #    所以 "格数 * 分辨率" 少算了半格到一格(实测偏保守 25mm),
        #    这里 + 0.5 格把"到障碍边界的距离"补回来。
        self.distance_mm = (ndimage.distance_transform_edt(~raw_blocked) + 0.5) * res
        # 3) 可行区域
        self.cost = self.distance_mm >= self.blocked_radius
        self.raw_blocked = raw_blocked

        # 调试用: 膨胀区域 = 有障碍但离得不够远
        self.inflated = (~raw_blocked) & (~self.cost)

    # ------------------------------------------------------------------ #
    # 构建
    # ------------------------------------------------------------------ #
    def _rasterize(self, polygons: Sequence) -> np.ndarray:
        """把多边形列表画到栅格上; 返回 True=被障碍占据."""
        if Image is None or ImageDraw is None:  # pragma: no cover
            return self._rasterize_numpy(polygons)

        img = Image.new("1", (self.spec.width, self.spec.height), 0)
        draw = ImageDraw.Draw(img)
        k = 1.0 / self.resolution
        for poly in polygons:
            pts = poly.points
            if len(pts) < 3:
                continue
            pix = [((x - self.spec.x_min) * k, (y - self.spec.y_min) * k) for x, y in pts]
            draw.polygon(pix, fill=1)
        arr = np.array(img, dtype=bool)
        # PIL 的 y 轴向下(行 0 在最小 y), 与我们的 row 约定一致, 无需翻转
        return arr

    def _rasterize_numpy(self, polygons: Sequence) -> np.ndarray:  # pragma: no cover
        from .geometry import point_in_polygon

        mask = np.zeros((self.spec.height, self.spec.width), dtype=bool)
        for row in range(self.spec.height):
            for col in range(self.spec.width):
                x, y = self.spec.cell_to_world(row, col)
                for poly in polygons:
                    if point_in_polygon((x, y), poly.points):
                        mask[row, col] = True
                        break
        return mask

    # ------------------------------------------------------------------ #
    # 查询
    # ------------------------------------------------------------------ #
    def is_free_cell(self, row: int, col: int) -> bool:
        return self.spec.inside(row, col) and bool(self.cost[row, col])

    def is_free_world(self, x: float, y: float) -> bool:
        row, col = self.spec.world_to_cell(x, y)
        return self.is_free_cell(row, col)

    def clearance_mm(self, x: float, y: float) -> float:
        """到最近障碍的距离(mm); 越界返回 0."""
        row, col = self.spec.world_to_cell(x, y)
        if not self.spec.inside(row, col):
            return 0.0
        return float(self.distance_mm[row, col])

    def clearance_many_mm(self, xs, ys) -> np.ndarray:
        """批量查离障距离(mm); 越界返回 0. 供 B 样条密集校验用."""
        xs_arr = np.asarray(xs, dtype=np.float64)
        ys_arr = np.asarray(ys, dtype=np.float64)
        cols = np.floor((xs_arr - self.spec.x_min) / self.resolution).astype(np.int64)
        rows = np.floor((ys_arr - self.spec.y_min) / self.resolution).astype(np.int64)
        np.clip(rows, 0, self.spec.height - 1, out=rows)
        np.clip(cols, 0, self.spec.width - 1, out=cols)
        inside = ((xs_arr >= self.spec.x_min) &
                  (xs_arr < self.spec.x_min + self.spec.width * self.resolution) &
                  (ys_arr >= self.spec.y_min) &
                  (ys_arr < self.spec.y_min + self.spec.height * self.resolution))
        out = self.distance_mm[rows, cols]
        return np.where(inside, out, 0.0)

    def free_cell_count(self) -> int:
        return int(np.count_nonzero(self.cost))

    def nearest_free_cell(self, x: float, y: float,
                          max_radius_mm: float = 2000.0) -> Optional[Tuple[int, int]]:
        """找离 (x, y) 最近的可行格; 找不到返回 None.

        先在局部窗口里挑一个离得最近的可行格, 再围绕它做一次**亚格细化**:
        直接在连续空间里搜索, 避免"吸附到 1.1m 外的格心"这种精度损失。
        """
        row0, col0 = self.spec.world_to_cell(x, y)
        if self.is_free_cell(row0, col0):
            return (row0, col0)

        r_cells = max(2, int(math.ceil(max_radius_mm / self.resolution)))
        r0 = max(0, row0 - r_cells)
        r1 = min(self.spec.height, row0 + r_cells + 1)
        c0 = max(0, col0 - r_cells)
        c1 = min(self.spec.width, col0 + r_cells + 1)
        if r0 >= r1 or c0 >= c1:
            return None

        window = self.cost[r0:r1, c0:c1]
        if not window.any():
            return None

        rows, cols = np.nonzero(window)
        rows = rows + r0
        cols = cols + c0
        cx = self.spec.x_min + (cols + 0.5) * self.resolution
        cy = self.spec.y_min + (rows + 0.5) * self.resolution
        d2 = (cx - x) ** 2 + (cy - y) ** 2
        idx = int(np.argmin(d2))
        row, col = int(rows[idx]), int(cols[idx])

        # 亚格细化: 在候选格附近的连续网格上找"最近的可行点"
        best = (row, col)
        best_d2 = float(d2[idx])
        fine_step = self.resolution * 0.25
        span = self.resolution * 1.5
        offsets = np.arange(-span, span + 1e-9, fine_step)
        for dy in offsets:
            for dx in offsets:
                px, py = x + dx, y + dy
                r, c = self.spec.world_to_cell(px, py)
                if not self.is_free_cell(r, c):
                    continue
                d2f = dx * dx + dy * dy
                if d2f < best_d2:
                    best_d2 = d2f
                    best = (r, c)
        return best

    def snap_point(self, x: float, y: float) -> Optional[Point]:
        cell = self.nearest_free_cell(x, y)
        if cell is None:
            return None
        return self.spec.cell_to_world(*cell)

    # ------------------------------------------------------------------ #
    # 线段 / 折线 碰撞检查
    # ------------------------------------------------------------------ #
    def segment_is_free(self, a: Point, b: Point,
                        sample_step_mm: Optional[float] = None) -> bool:
        """线段是否全程在可行区内.

        采样步长默认取 0.5 格, 足够密, 不会漏掉细障碍.
        """
        step = float(sample_step_mm if sample_step_mm else self.resolution * 0.5)
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(length / step)))
        for i in range(n + 1):
            t = i / n
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            if not self.is_free_world(x, y):
                return False
        return True

    def polyline_is_free(self, pts: Sequence[Point]) -> bool:
        for i in range(1, len(pts)):
            if not self.segment_is_free(pts[i - 1], pts[i]):
                return False
        return True

    def path_min_clearance(self, pts: Sequence[Point]) -> float:
        """折线（按密集采样）上的最小离障距离, 用来判断余量够不够."""
        if len(pts) < 2:
            return float("inf")
        best = float("inf")
        step = self.resolution * 0.5
        for i in range(1, len(pts)):
            a, b = pts[i - 1], pts[i]
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, int(math.ceil(length / step)))
            for k in range(n + 1):
                t = k / n
                x = a[0] + (b[0] - a[0]) * t
                y = a[1] + (b[1] - a[1]) * t
                c = self.clearance_mm(x, y)
                if c < best:
                    best = c
        return best


def build_costmap(scene: Scene, resolution: Optional[float] = None) -> CostMap:
    return CostMap(scene, resolution)
