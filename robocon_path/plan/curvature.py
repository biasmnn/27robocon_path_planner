"""弧长 / 切向 / 曲率.

曲率符号约定(很重要, 决定 wz 的正负)
------------------------------------
场地系是右手系(y 向上), yaw 逆时针为正. 定义

    yaw_i = atan2(dy, ds)
    kappa_i = d(yaw)/ds

于是 **左转为正**: kappa > 0 <=> 车头往逆时针偏 <=> wz = v * kappa > 0.
车上如果发现左转给的是负 wz, 就是坐标系定反了, 改这里一处即可, 不要改两处.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from ..core.geometry import Point, wrap_pi


@dataclass
class CurveGeometry:
    points: np.ndarray            # (N, 2) mm
    s_mm: np.ndarray              # (N,) 从起点算起的弧长 mm
    tangent: np.ndarray           # (N, 2) 单位切向
    yaw_rad: np.ndarray           # (N,) 切向角 rad, (-pi, pi]
    curvature: np.ndarray         # (N,) 1/mm, 左正右负

    @property
    def length_mm(self) -> float:
        return float(self.s_mm[-1]) if len(self.s_mm) else 0.0

    @property
    def max_abs_curvature(self) -> float:
        if len(self.curvature) == 0:
            return 0.0
        return float(np.max(np.abs(self.curvature)))

    def min_radius_mm(self) -> float:
        k = self.max_abs_curvature
        return float("inf") if k <= 1e-12 else 1.0 / k


def compute_geometry(points: Sequence[Point],
                     smooth_window: int = 0,
                     curvature_span_mm: float = 0.0,
                     curvature_eps: float = 2.0e-6) -> CurveGeometry:
    """由点列算弧长/切向/曲率.

    参数
    ----
    points             : 按行驶顺序排列的点(通常来自 B 样条按弧长重采样)
    smooth_window      : >1 时对切向角做滑动平均, 抑制采样噪声
    curvature_span_mm  : >0 时曲率用"跨 d mm 的有限差分"算, 而不是用相邻两点。
        ⚠ 这个很重要: 直线段上密集采样时, atan2 的量化噪声会让相邻点之间
        出现 ±1e-4 的"假转角", 除以 10mm 就得到 1e-5 /mm 的假曲率 ——
        折算成转弯半径只有几百毫米, 会让"最小转弯半径"检查误报。
        用 50mm 的跨度做差分, 噪声被摊薄一个量级, 真弯的曲率基本不受影响。
    curvature_eps      : 小于这个值的曲率直接归零(噪声floor)
    """
    pts = np.asarray([(float(p[0]), float(p[1])) for p in points], dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] < 2:
        empty = np.zeros(0, dtype=np.float64)
        return CurveGeometry(pts.reshape(-1, 2), empty, np.zeros((0, 2)),
                             empty, empty)

    # 去掉重复点, 否则切向会 NaN
    keep = [0]
    for i in range(1, pts.shape[0]):
        if np.hypot(pts[i, 0] - pts[keep[-1], 0], pts[i, 1] - pts[keep[-1], 1]) > 1e-9:
            keep.append(i)
    pts = pts[keep]
    if pts.shape[0] < 2:
        empty = np.zeros(0, dtype=np.float64)
        return CurveGeometry(pts, empty, np.zeros((0, 2)), empty, empty)

    d = np.diff(pts, axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    s = np.concatenate(([0.0], np.cumsum(seg)))

    # 切向: 中心差分(端点用单侧差分)
    t = np.zeros_like(pts)
    t[1:-1] = pts[2:] - pts[:-2]
    t[0] = pts[1] - pts[0]
    t[-1] = pts[-1] - pts[-2]
    norm = np.hypot(t[:, 0], t[:, 1])
    norm[norm < 1e-12] = 1.0
    t = t / norm[:, None]

    yaw = np.arctan2(t[:, 1], t[:, 0])

    if smooth_window and smooth_window >= 3:
        yaw = _smooth_wrapped(yaw, int(smooth_window))
        t = np.column_stack((np.cos(yaw), np.sin(yaw)))

    kappa = _curvature_from_yaw(yaw, s, curvature_span_mm)
    if curvature_eps > 0.0:
        kappa = np.where(np.abs(kappa) < curvature_eps, 0.0, kappa)

    return CurveGeometry(pts, s, t, yaw, kappa)


def _curvature_from_yaw(yaw: np.ndarray, s: np.ndarray,
                        span_mm: float) -> np.ndarray:
    """kappa = d(yaw)/ds, 用跨 span_mm 的有限差分(span<=0 时用相邻点)."""
    n = len(yaw)
    kappa = np.zeros(n, dtype=np.float64)
    if n < 2:
        return kappa

    if span_mm <= 0.0:
        # 相邻点差分
        ds = np.diff(s)
        ds = np.where(ds < 1e-9, 1e-9, ds)
        dyaw = _wrap_array(np.diff(yaw))
        kappa[:-1] = dyaw / ds
        kappa[-1] = kappa[-2] if n >= 2 else 0.0
        return kappa

    # 对每个点, 找前后各约 span/2 的点做差分
    half = span_mm * 0.5
    lo = np.searchsorted(s, s - half, side="left")
    hi = np.searchsorted(s, s + half, side="right") - 1
    hi = np.clip(hi, 0, n - 1)
    lo = np.clip(lo, 0, n - 1)
    # 防止自比较
    hi = np.maximum(hi, 0)
    valid = hi > lo
    ds = np.where(valid, s[hi] - s[lo], 1.0)
    ds = np.where(ds < 1e-9, 1e-9, ds)
    dyaw = _wrap_array(yaw[hi] - yaw[lo])
    kappa = np.where(valid, dyaw / ds, 0.0)
    # 端点用最近的有效值
    if n >= 3:
        kappa[0] = kappa[1]
        kappa[-1] = kappa[-2]
    return kappa


def resample_by_arclength(points: Sequence[Point], s_mm: Sequence[float],
                          step_mm: float, max_points: int = 0) -> Tuple[np.ndarray, np.ndarray]:
    """按弧长等距重采样; 返回 (新点, 新弧长).

    max_points > 0 时自动放大 step, 保证点数不超过上限(首末点必须保留).
    """
    pts = np.asarray(points, dtype=np.float64)
    s = np.asarray(s_mm, dtype=np.float64)
    if pts.shape[0] < 2:
        return pts, s if len(s) == len(pts) else np.zeros(len(pts))

    total = float(s[-1])
    if total <= 1e-9 or step_mm <= 0.0:
        return pts, s

    n_target = int(math.floor(total / step_mm)) + 1
    if max_points and n_target > max_points:
        n_target = max_points
        step_mm = total / max(1, (n_target - 1))
        n_target = int(math.floor(total / step_mm)) + 1

    s_new = np.linspace(0.0, total, max(2, n_target))
    x_new = np.interp(s_new, s, pts[:, 0])
    y_new = np.interp(s_new, s, pts[:, 1])
    return np.column_stack((x_new, y_new)), s_new


# --------------------------------------------------------------------------- #
# 工具
# --------------------------------------------------------------------------- #
def _wrap_array(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def _smooth_wrapped(a: np.ndarray, window: int) -> np.ndarray:
    """对角度序列做滑动平均(先解缠, 再平均, 再折回)."""
    unwrapped = np.unwrap(a)
    if window % 2 == 0:
        window += 1
    kernel = np.ones(window) / window
    pad = window // 2
    padded = np.pad(unwrapped, pad, mode="edge")
    smooth = np.convolve(padded, kernel, mode="valid")
    return _wrap_array(smooth[: len(a)])


def yaw_at(yaw: np.ndarray, s: np.ndarray, s_query: float) -> float:
    """按弧长插值航向角."""
    if len(yaw) == 0:
        return 0.0
    idx = int(np.searchsorted(s, s_query))
    if idx <= 0:
        return float(yaw[0])
    if idx >= len(s):
        return float(yaw[-1])
    t = (s_query - s[idx - 1]) / max(1e-9, s[idx] - s[idx - 1])
    return float(yaw[idx - 1] + wrap_pi(yaw[idx] - yaw[idx - 1]) * t)


def curvature_at(curvature: np.ndarray, s: np.ndarray, s_query: float) -> float:
    if len(curvature) == 0:
        return 0.0
    return float(np.interp(s_query, s, curvature))
