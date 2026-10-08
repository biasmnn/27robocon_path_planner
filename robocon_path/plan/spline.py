"""B 样条平滑(带曲率约束与避障回退).

与 2D/13_bspline_path_smoothing.py 的 `bspline_smooth_path` 同源(都用
scipy.interpolate.splprep/splev), 但补齐了三个工程上必须有的东西:

1. **避障保证**: 平滑后的曲线用 CostMap 按毫米密集校验; 若穿进膨胀区,
   就在"违规最严重处"把关键点切开, 分段重新拟合(递归), 直到全段安全或放弃。
2. **曲率约束**: 算出最大曲率, 若超过上限(由最小转弯半径决定),
   就加大平滑系数/增加控制点重新拟合, 最多试若干次。
3. **采样密度与参数域无关**: 拟合后按弧长重采样, 不用 linspace(0,1,n),
   避免"参数域均匀但空间上疏密不均"导致下游速度算不准。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.interpolate import splprep, splev

from ..core.costmap import CostMap
from ..core.geometry import Point
from .curvature import CurveGeometry, compute_geometry


@dataclass
class SplineResult:
    ok: bool
    points: List[Point] = field(default_factory=list)   # 密集采样后的曲线点(场地系 mm)
    geometry: Optional[CurveGeometry] = None
    message: str = ""
    min_clearance_mm: float = 0.0
    max_curvature: float = 0.0        # 1/mm
    min_radius_mm: float = float("inf")
    segments: int = 0                 # 被切成了几段

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


def merge_short_segments(points: Sequence[Point], min_segment_mm: float,
                         costmap: Optional[CostMap] = None,
                         min_clearance_mm: float = 0.0) -> List[Point]:
    """把过短的中间边合并掉, 避免"双拐点"卡死倒角.

    为什么需要
    ----------
    LOS 简化在拐角处有时会留下两个挨得很近的关键点(实测出现过 112mm 的两条边,
    形成两个直角)。倒角量受"最短边 * 0.45"限制, 于是每个拐角最多只能倒 50mm,
    最终曲率半径只有 200mm 量级, 车根本转不过来。

    做法: 只要中间某条边短于 min_segment_mm, 就用"该边端点 + 它两侧邻点"的重心
    代替这两个点(等价于钝化一个很窄的凸起)。反复迭代直到没有过短的边。

    ⚠ 合并会把路径往凸起内侧挪, 可能挪进障碍。传入 costmap 时, 如果合并后的点
    离障不足就**放弃这次合并**(那个短边留给倒角去保守处理)。
    """
    pts = _dedup(points)
    if len(pts) < 3 or min_segment_mm <= 0.0:
        return pts

    def safe(p: Point) -> bool:
        if costmap is None:
            return True
        return costmap.clearance_mm(p[0], p[1]) >= min_clearance_mm

    for _ in range(12):
        lens = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                for i in range(len(pts) - 1)]
        # 找最短的中间边(不碰首末边)
        best_i = -1
        best_len = float("inf")
        for i in range(1, len(lens) - 1):
            if lens[i] < best_len:
                best_len = lens[i]
                best_i = i
        if best_i < 0 or best_len >= min_segment_mm or len(pts) <= 3:
            break
        p_prev = pts[best_i - 1]
        p_a = pts[best_i]
        p_b = pts[best_i + 1]
        p_next = pts[best_i + 2]
        cx = (p_prev[0] + p_a[0] + p_b[0] + p_next[0]) * 0.25
        cy = (p_prev[1] + p_a[1] + p_b[1] + p_next[1]) * 0.25
        if not safe((cx, cy)):
            # 合并不安全: 把这个边"标记为不可再合并", 直接结束(避免死循环)
            break
        pts = pts[:best_i] + [(cx, cy)] + pts[best_i + 2:]
    return pts


def _densify_chunk(chunk: np.ndarray, step_mm: float) -> np.ndarray:
    """把 2 个控制点的退化段加密成折线(用于校验与拼接).

    ⚠ 关键坑: 退化成折线时如果只保留端点, "最小离障"就只量了两个角点 ——
    实测出现过"4 个角点离障 850mm"把一条**穿墙的直线段**判成合格方案。
    所以这里必须按步长插值出中间点。
    """
    if chunk.shape[0] < 2:
        return chunk
    seg = np.hypot(np.diff(chunk[:, 0]), np.diff(chunk[:, 1]))
    total = float(np.sum(seg))
    n = max(2, int(math.ceil(total / max(1.0, step_mm))) + 1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    if total <= 1e-9:
        return chunk[:1]
    s_new = np.linspace(0.0, total, n)
    return np.column_stack((np.interp(s_new, s, chunk[:, 0]),
                            np.interp(s_new, s, chunk[:, 1])))


def round_corners(points: Sequence[Point], radius_mm: float,
                  max_ratio: float = 0.45,
                  costmap: Optional[CostMap] = None,
                  min_clearance_mm: float = 0.0) -> List[Point]:
    """把尖角切成两个控制点(倒角), 让后面的 B 样条不在拐点处留下尖角.

    为什么必须做
    ------------
    LOS 简化后的关键点往往有接近 90° 的急转。直接对它做 B 样条插值(s=0)时,
    曲线**必须穿过**这个点, 而相邻控制点又在两侧把曲线往外拉, 结果就在该点
    附近形成一个极小的转弯半径 —— 实测出现过 R=32mm、曲率 3e-2 /mm 的尖角,
    折算成横向加速度是车根本承受不了的。

    做法: 对每个中间点 P, 沿前后两条边各退 `d` 得到 A、B 两点, 用 A、B 代替 P。
    `d = min(radius_mm, max_ratio * min(|PA|, |PB|))`, 保证不会跨过相邻点。

    ⚠ 倒角会把路径往角内侧收, 有可能比原折线更贴近障碍。所以:
      1. 传入 costmap 时会检查倒角点的离障距离, 不够就把 d 折半重试(最多 4 次);
      2. 实在不行就保留原点(不倒角), 由后续的碰撞检查兜底。
    """
    pts = _dedup(points)
    if len(pts) < 3 or radius_mm <= 0.0:
        return pts

    def ok(p: Point) -> bool:
        if costmap is None:
            return True
        return costmap.clearance_mm(p[0], p[1]) >= min_clearance_mm

    def path_ok(p: Point, q: Point) -> bool:
        """检查 p->q 整条线段的离障余量.

        ⚠ 关键坑: 只检查 p、q 两个点是**不够的** —— 倒角后 A、B 之间的直线段
        才是真正会切进障碍的那一段。实测出现过"A、B 各查出 200/79mm, 但中间
        只有 56mm", 导致穿墙的方案被判成合格。
        """
        if costmap is None:
            return True
        n = max(2, int(math.hypot(q[0] - p[0], q[1] - p[1])
                       / max(1.0, costmap.resolution * 0.5)) + 1)
        for i in range(n + 1):
            t = i / n
            x = p[0] + (q[0] - p[0]) * t
            y = p[1] + (q[1] - p[1]) * t
            if costmap.clearance_mm(x, y) < min_clearance_mm:
                return False
        return True

    out: List[Point] = [pts[0]]
    for i in range(1, len(pts) - 1):
        prev_p, cur, next_p = pts[i - 1], pts[i], pts[i + 1]
        v1x, v1y = cur[0] - prev_p[0], cur[1] - prev_p[1]
        v2x, v2y = next_p[0] - cur[0], next_p[1] - cur[1]
        l1, l2 = math.hypot(v1x, v1y), math.hypot(v2x, v2y)
        if l1 < 1e-6 or l2 < 1e-6:
            continue
        # 转角很小(接近直线)就不用倒角
        cos_t = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (l1 * l2)))
        turn = math.acos(cos_t)
        if turn < math.radians(8.0):
            out.append(cur)
            continue

        d = min(radius_mm, max_ratio * min(l1, l2))
        placed = False
        for _ in range(5):
            if d < 1.0:
                break
            a = (cur[0] - v1x / l1 * d, cur[1] - v1y / l1 * d)
            b = (cur[0] + v2x / l2 * d, cur[1] + v2y / l2 * d)
            if (ok(a) and ok(b) and path_ok(a, b)
                    and path_ok(out[-1], a)):
                out.append(a)
                out.append(b)
                placed = True
                break
            d *= 0.5
        if not placed:
            out.append(cur)
    out.append(pts[-1])
    return out


def _fit_one_segment(ctrl: np.ndarray, smooth: float, dense: int) -> Optional[np.ndarray]:
    """对一段控制点做 B 样条拟合, 返回密集采样点 (dense, 2).

    点数不足或退化时返回 None.
    """
    n = ctrl.shape[0]
    if n < 3:
        return None
    k = min(3, n - 1)
    try:
        tck, _u = splprep([ctrl[:, 0], ctrl[:, 1]], s=float(smooth), k=k)
    except Exception:
        return None
    u_new = np.linspace(0.0, 1.0, int(dense))
    try:
        x_new, y_new = splev(u_new, tck)
    except Exception:
        return None
    pts = np.column_stack((np.asarray(x_new, dtype=np.float64),
                           np.asarray(y_new, dtype=np.float64)))
    if not np.all(np.isfinite(pts)):
        return None
    return pts


def _dense_count(length_mm: float, step_mm: float, low: int = 200, high: int = 6000) -> int:
    n = int(math.ceil(max(length_mm, 1.0) / max(step_mm, 1.0))) + 2
    return max(low, min(high, n))


def _group_index(base: Sequence[Point], expanded: Sequence[Point]) -> List[int]:
    """expanded 里每个点对应 base 的哪个下标.

    规则: 第 0 个点 -> 0; 之后每遇到一个与 base[i] 重合的点, 说明到了第 i 个
    原始关键点; 夹在 base[i-1] 与 base[i] 之间的点(倒角点)全部归到 i-1。
    这样 [groups[i-1], groups[i]] 正好切出"base[i-1] -> base[i] 那一段"的全部控制点。
    """
    n = len(expanded)
    m = len(base)
    groups = [0] * n
    seg = 0
    for j in range(n):
        p = expanded[j]
        # 如果这个点正好是 base 里的某个点, 就更新段号
        for i in range(seg, m):
            if _close(p, base[i]):
                seg = i
                break
        groups[j] = seg
    if n:
        groups[0] = 0
        groups[-1] = m - 1
    return groups


def _close(a: Point, b: Point, eps: float = 1e-6) -> bool:
    return abs(a[0] - b[0]) <= eps and abs(a[1] - b[1]) <= eps


def bspline_smooth_path(path: Sequence[Point],
                        costmap: Optional[CostMap] = None,
                        smooth: float = 0.0,
                        sample_step_mm: float = 5.0,
                        max_curvature: float = 0.0,
                        max_depth: int = 4,
                        curvature_retries: int = 6,
                        clearance_margin_mm: float = 0.0,
                        corner_radius_mm: float = 0.0) -> SplineResult:
    """把折线关键点拟合成光滑曲线.

    参数
    ----
    path             : LOS 简化后的关键点(场地系 mm)
    costmap          : 用于避障校验; None 则跳过避障校验
    smooth           : splprep 的平滑系数 s, 0 = 严格插值全部关键点
    sample_step_mm   : 密集采样步长(mm), 越小越密
    max_curvature    : 曲率上限 1/mm, <=0 表示不检查(留给调用方判断)
    max_depth        : 避障失败时递归切分的最大层数
    curvature_retries: 曲率超限时加大平滑系数的重试次数
    clearance_margin_mm : 额外的离障余量(在 costmap 判定之外再加一层)
    corner_radius_mm : 拐点倒角半径(mm); 0 表示按最小转弯半径自动取
    """
    ctrl_all = _dedup(path)
    if len(ctrl_all) < 3:
        geo = compute_geometry(ctrl_all) if len(ctrl_all) >= 2 else None
        return SplineResult(bool(ctrl_all), list(ctrl_all), geo,
                            message="关键点少于 3 个, 不做平滑")

    # 拐点倒角: 不做这一步, B 样条会在急转处留尖角(实测 R 到过 32mm)
    # 倒角量取 1.6 倍最小转弯半径: 经验上这样才能让拐角处的实际曲率半径达
    # 到要求(取 1.0 倍时实测 R 只有 217mm, 远达不到 800mm 的目标)。
    if corner_radius_mm <= 0.0:
        corner_radius_mm = (1.6 / max_curvature) if max_curvature > 1e-9 else 900.0
    required_clearance = ((costmap.blocked_radius + clearance_margin_mm)
                          if costmap is not None else 0.0)

    # 整体拟合(不要按原始关键点分段!): 分段的话, 每段的端点曲率是自由的,
    # 两个独立样条在接缝处拼接会留下 R 只有 50mm 量级的折角, 反而比整体拟合糟。
    # 尖角问题已经由上面的倒角解决。
    # 先合并过短的中间边(否则倒角量被最短边卡死, 曲率半径达不到要求)
    ctrl_all = merge_short_segments(ctrl_all,
                                    min_segment_mm=1.2 * corner_radius_mm,
                                    costmap=costmap,
                                    min_clearance_mm=required_clearance)

    # 尝试若干组参数: 逐次收紧倒角比例 / 加大平滑系数, 目标是同时满足
    # "不碰撞" + "曲率不超限"。
    # 与早期版本的区别: **优先接受完全满足要求的方案**; 一个都不满足时, 把
    # "离障有余量"的那一档(而不是离障最少的那一档)交出去, 并且 ok=False,
    # 由 pipeline 决定丢弃并回退。
    ratio_tries = (0.45, 0.32, 0.22, 0.12)
    smooth_tries = (smooth, smooth + 50.0, smooth + 200.0, smooth + 800.0)
    best_good: Optional[Tuple[np.ndarray, float, int]] = None
    best_safe: Optional[Tuple[np.ndarray, float, int]] = None

    for ratio in ratio_tries:
        rounded = round_corners(ctrl_all, corner_radius_mm, max_ratio=ratio,
                                costmap=costmap,
                                min_clearance_mm=required_clearance)
        for s_try in smooth_tries:
            segments = []
            _fit_recursive(np.asarray(rounded, dtype=np.float64), s_try,
                           sample_step_mm, costmap, clearance_margin_mm,
                           max_depth, segments, 0)
            if not segments:
                continue
            pts = np.vstack(segments)
            clr = _min_clearance_array(costmap, pts) if costmap is not None \
                else float("inf")
            geo_try = compute_geometry([(float(p[0]), float(p[1])) for p in pts],
                                       smooth_window=5)
            kappa_ok = (max_curvature <= 0.0
                        or geo_try.max_abs_curvature <= max_curvature)
            clear_ok = (costmap is None or clr >= required_clearance)

            if best_safe is None or clr > best_safe[1]:
                best_safe = (pts, clr, len(segments))
            if clear_ok and kappa_ok:
                best_good = (pts, clr, len(segments))
                break
        if best_good is not None:
            break

    chosen = best_good if best_good is not None else best_safe
    if chosen is None:
        geo = compute_geometry(ctrl_all)
        return SplineResult(False, list(ctrl_all), geo,
                            message="B 样条拟合失败(控制点退化?), 已回退为折线",
                            min_clearance_mm=_min_clearance(costmap, ctrl_all))

    points, clearance, n_segments = chosen

    geo = compute_geometry(points, smooth_window=5)
    ok = True
    notes: List[str] = []
    if costmap is not None and clearance < required_clearance:
        ok = False
        notes.append(f"离障余量不足: 最小 {clearance:.0f}mm < 要求 "
                     f"{required_clearance:.0f}mm")
    if max_curvature > 0.0 and geo.max_abs_curvature > max_curvature:
        ok = False
        notes.append(f"曲率超限: 最小转弯半径 {geo.min_radius_mm():.0f}mm < 要求 "
                     f"{1.0 / max_curvature:.0f}mm")
    message = "; ".join(notes) if notes else "OK"

    return SplineResult(ok, [(float(p[0]), float(p[1])) for p in points], geo,
                        message=message,
                        min_clearance_mm=float(clearance),
                        max_curvature=geo.max_abs_curvature,
                        min_radius_mm=geo.min_radius_mm(),
                        segments=n_segments)


def _fit_recursive(ctrl: np.ndarray, smooth: float, step_mm: float,
                   costmap: Optional[CostMap], margin_mm: float,
                   depth_left: int, out: List[np.ndarray], level: int) -> None:
    """递归拟合: 穿障碍就切开重来.

    切分点选"违规最严重"的位置。为了避免切出只有 2 个点的退化段(那一段会退化成
    折线并在端点上留下折角), 切分点会往中间靠拢, 保证两边都至少 3 个控制点。
    """
    if ctrl.shape[0] < 3:
        # 退化成折线, 必须加密后再交出去(否则校验只看角点, 会漏掉中间的穿墙段)
        out.append(_densify_chunk(ctrl, step_mm))
        return

    length = float(np.sum(np.hypot(np.diff(ctrl[:, 0]), np.diff(ctrl[:, 1]))))
    dense = _dense_count(length, step_mm)
    pts = _fit_one_segment(ctrl, smooth, dense)
    if pts is None:
        out.append(ctrl.copy())
        return

    if costmap is None or depth_left <= 0:
        out.append(pts)
        return

    clearance = costmap.clearance_many_mm(pts[:, 0], pts[:, 1])
    need = costmap.blocked_radius + margin_mm
    bad = clearance < need
    if not bad.any():
        out.append(pts)
        return

    # 找违规最严重处, 作为切分位置
    bad_idx = np.nonzero(bad)[0]
    worst = int(bad_idx[np.argmin(clearance[bad_idx])])
    ratio = worst / max(1, len(pts) - 1)
    n_ctrl = ctrl.shape[0]
    split = int(round(ratio * (n_ctrl - 1)))
    # 两边都要 >= 3 个控制点, 否则切了也没意义
    lo = 2
    hi = n_ctrl - 3
    if hi < lo:
        out.append(_densify_chunk(ctrl, step_mm))   # 控制点太少, 加密成折线
        return
    split = max(lo, min(hi, split))

    _fit_recursive(ctrl[: split + 1], smooth, step_mm, costmap, margin_mm,
                   depth_left - 1, out, level + 1)
    _fit_recursive(ctrl[split:], smooth, step_mm, costmap, margin_mm,
                   depth_left - 1, out, level + 1)


def _min_clearance(costmap: Optional[CostMap], pts: Sequence[Point]) -> float:
    if costmap is None:
        return float("inf")
    arr = np.asarray(pts, dtype=np.float64)
    return float(np.min(costmap.clearance_many_mm(arr[:, 0], arr[:, 1])))


def _min_clearance_array(costmap: Optional[CostMap], pts: np.ndarray) -> float:
    if costmap is None:
        return float("inf")
    return float(np.min(costmap.clearance_many_mm(pts[:, 0], pts[:, 1])))
