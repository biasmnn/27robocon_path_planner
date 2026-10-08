"""A* 栅格寻路(八邻域 + octile 启发).

与 2D/11_astar_obstacle_inflation.py 的思路一致, 工程化后的差别:
* 用 numpy 数组存 g / closed, 用 heapq 存 open, 大栅格也不慢;
* 因为 CostMap 用的是"精确圆膨胀", 对角移动不需要额外穿墙角检查;
* 起点/终点落在膨胀区内时自动吸附到最近可行格, 并把吸附前的原始点补到路径两端,
  这样"贴边取点"也能跑(吸附距离会一并返回, 用于提示)。
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from ..core.costmap import CostMap
from ..core.geometry import Point

SQRT2 = math.sqrt(2.0)

# 八邻域: (d_row, d_col, cost)
NEIGHBORS_8 = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2),
)


@dataclass
class AStarResult:
    ok: bool
    path: List[Point] = field(default_factory=list)        # 场地系 mm
    searched_cells: List[Tuple[int, int]] = field(default_factory=list)
    message: str = ""
    start_snapped_mm: float = 0.0
    goal_snapped_mm: float = 0.0
    expanded: int = 0

    @property
    def length_mm(self) -> float:
        total = 0.0
        for i in range(1, len(self.path)):
            total += math.hypot(self.path[i][0] - self.path[i - 1][0],
                                self.path[i][1] - self.path[i - 1][1])
        return total


class GridAStar:
    def __init__(self, costmap: CostMap):
        self.cm = costmap
        self.spec = costmap.spec

    # ------------------------------------------------------------------ #
    def search(self,
               start: Point,
               goal: Point,
               collect_searched: bool = False,
               max_expand: int = 4_000_000) -> AStarResult:
        spec = self.spec
        start_cell = self.cm.nearest_free_cell(*start)
        goal_cell = self.cm.nearest_free_cell(*goal)
        if start_cell is None:
            return AStarResult(False, message="起点附近找不到可行格(障碍太多?)")
        if goal_cell is None:
            return AStarResult(False, message="终点附近找不到可行格(障碍太多?)")

        sr, sc = start_cell
        gr, gc = goal_cell
        start_snap = math.hypot(spec.cell_to_world(sr, sc)[0] - start[0],
                                spec.cell_to_world(sr, sc)[1] - start[1])
        goal_snap = math.hypot(spec.cell_to_world(gr, gc)[0] - goal[0],
                               spec.cell_to_world(gr, gc)[1] - goal[1])

        if (sr, sc) == (gr, gc):
            return AStarResult(True, path=[tuple(start), tuple(goal)],
                               message="起点终点在同一格", start_snapped_mm=start_snap,
                               goal_snapped_mm=goal_snap)

        width, height = spec.width, spec.height
        g_score = np.full((height, width), np.inf, dtype=np.float64)
        parent = np.full((height, width), -1, dtype=np.int32)
        closed = np.zeros((height, width), dtype=bool)

        def flat(row: int, col: int) -> int:
            return row * width + col

        g_score[sr, sc] = 0.0
        start_f = self._heuristic(sr, sc, gr, gc)
        open_heap: List[Tuple[float, float, int, int]] = [(start_f, 0.0, sr, sc)]
        searched: List[Tuple[int, int]] = []
        expanded = 0

        cost = self.cm.cost

        while open_heap:
            _f, g_cur, row, col = heapq.heappop(open_heap)
            if closed[row, col]:
                continue
            if g_cur > g_score[row, col] + 1e-9:
                continue
            closed[row, col] = True
            expanded += 1
            if collect_searched:
                searched.append((row, col))
            if (row, col) == (gr, gc):
                path_cells = self._reconstruct(parent, (sr, sc), (gr, gc))
                pts = [spec.cell_to_world(r, c) for r, c in path_cells]
                # 把用户点的原始坐标接到两端, 减少"贴点被吸附"造成的误差
                pts[0] = (float(start[0]), float(start[1]))
                pts[-1] = (float(goal[0]), float(goal[1]))
                return AStarResult(True, path=pts, searched_cells=searched,
                                   start_snapped_mm=start_snap,
                                   goal_snapped_mm=goal_snap, expanded=expanded)

            if expanded > max_expand:
                return AStarResult(False, message="搜索节点超上限, 请检查起终点是否被围死",
                                   searched_cells=searched, expanded=expanded)

            for d_row, d_col, step_cost in NEIGHBORS_8:
                nr, nc = row + d_row, col + d_col
                if nr < 0 or nr >= height or nc < 0 or nc >= width:
                    continue
                if not cost[nr, nc] or closed[nr, nc]:
                    continue
                # Match the supplied 2D reference: never cut a blocked corner.
                if d_row and d_col and (not cost[row,nc] or not cost[nr,col]):
                    continue
                # Surface entrances and swept clearance also constrain edges.
                # Restrict the extra work to the physically modelled map.
                if hasattr(self.cm,"edge_is_free_cells") and not self.cm.edge_is_free_cells(row,col,nr,nc):
                    continue
                tentative = g_cur + step_cost
                if tentative + 1e-9 >= g_score[nr, nc]:
                    continue
                g_score[nr, nc] = tentative
                parent[nr, nc] = flat(row, col)
                f = tentative + self._heuristic(nr, nc, gr, gc)
                heapq.heappush(open_heap, (f, tentative, nr, nc))

        return AStarResult(False, message="无可行路径(被障碍完全隔断)",
                           searched_cells=searched, start_snapped_mm=start_snap,
                           goal_snapped_mm=goal_snap, expanded=expanded)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _heuristic(row: int, col: int, gr: int, gc: int) -> float:
        """octile 距离, 与八邻域代价一致 => 可采纳且一致."""
        dx = abs(col - gc)
        dy = abs(row - gr)
        return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)

    def _reconstruct(self, parent: np.ndarray, start_cell: Tuple[int, int],
                     goal_cell: Tuple[int, int]) -> List[Tuple[int, int]]:
        width = self.spec.width
        cells: List[Tuple[int, int]] = []
        cur = goal_cell
        guard = 0
        while cur != start_cell:
            cells.append(cur)
            p = int(parent[cur[0], cur[1]])
            if p < 0:
                break
            cur = (p // width, p % width)
            guard += 1
            if guard > self.spec.width * self.spec.height:
                break
        cells.append(start_cell)
        cells.reverse()
        return cells


def astar_path(costmap: CostMap, start: Point, goal: Point,
               collect_searched: bool = False) -> AStarResult:
    return GridAStar(costmap).search(start, goal, collect_searched=collect_searched)
