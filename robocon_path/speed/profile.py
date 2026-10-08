"""速度剖面: 给曲线路径上的每个点算"该开多快".

单位约定(整个模块内部统一, 只在最外层换算)
------------------------------------------
长度 mm, 速度 mm/s, 加速度 mm/s^2, jerk mm/s^3。
`SpeedLimits` 对外用国际单位(m/s, m/s^2, m/s^3), 进入 `plan_speed_profile`
后**第一件事就是整体换算成毫米制**, 之后所有内部函数只认毫米, 避免混用出错。

算法(离线跑, 顺序无关)
---------------------
1. **曲率限速**:  v_curve = min(v_max, sqrt(a_lat / |kappa|))
   这是"过弯会不会甩出去"的唯一约束, 参考工程里缺的就是这一条。
2. **可达集包络**: 时间最优解 = min(前向可达, 后向可达), 两条都是
   v^2 关于 s 的线性函数, 可以解析求:
       v^2 <= v_start^2 + 2*a*s         (从起点能加速到多快)
       v^2 <= v_end^2   + 2*a*(L - s)   (为了能刹住最多能多快)
   ⚠ 不要用逐格递推! 每步增量只有 2*a*ds 量级, float64 会把增量吃掉,
     实测会把 3m 处的 3000mm/s 压成 94.9mm/s。
3. **jerk 限幅**: 在加速度域做投影, 再反解速度上限。
4. **最低速度**:  运动点抬到 v_min(板端静摩擦经验值, my27R2 里是 0.12m/s);
   首末点仍强制 0, 用来触发跟点器的终点精定位。

输出 (s, v) 一维查表: 板端只做插值, O(1); 而且是**按弧长**索引,
所以"跑快了提前减速、跑慢了继续给速度"的闭环特性得以保留。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class SpeedLimits:
    """底盘能力上限(对外统一国际单位)."""

    v_max: float = 1.5          # m/s
    a_max: float = 1.5          # m/s^2 纵向加/减速度
    j_max: float = 25.0         # m/s^3 加加速度(平滑用)
    a_lat_max: float = 1.2      # m/s^2 横向(过弯向心)加速度上限
    v_min: float = 0.06         # m/s 最小驱动速度(0 = 允许停)
    v_start: float = 0.0        # m/s 起点速度
    v_end: float = 0.0          # m/s 终点速度
    center_half_width: float = 0.0   # m 左右轮中心半宽; >0 时启用角速度限制
    wz_max: float = 2.5         # rad/s 角速度上限


@dataclass
class SpeedProfile:
    s_mm: np.ndarray
    v_mm_s: np.ndarray
    v_limit_mm_s: np.ndarray           # 曲率限速(未做可达集前的上限)
    curvature: np.ndarray
    limits: SpeedLimits
    total_time_s: float = 0.0
    peak_v_mm_s: float = 0.0

    @property
    def length_mm(self) -> float:
        return float(self.s_mm[-1]) if len(self.s_mm) else 0.0

    def v_at(self, s_query_mm: float) -> float:
        if len(self.s_mm) == 0:
            return 0.0
        return float(np.interp(s_query_mm, self.s_mm, self.v_mm_s))

    def a_at(self, s_query_mm: float) -> float:
        """用速度梯度估算纵向加速度 m/s^2 (仅用于显示/检查)."""
        if len(self.s_mm) < 2:
            return 0.0
        i = int(np.searchsorted(self.s_mm, s_query_mm))
        i = max(1, min(len(self.s_mm) - 1, i))
        ds = self.s_mm[i] - self.s_mm[i - 1]
        if ds <= 1e-9:
            return 0.0
        v1, v2 = self.v_mm_s[i - 1], self.v_mm_s[i]
        return (v2 * v2 - v1 * v1) * 0.5 / ds * 0.001

    def summary(self) -> str:
        return (f"长度 {self.length_mm / 1000.0:.3f} m, "
                f"峰值速度 {self.peak_v_mm_s:.0f} mm/s, "
                f"预计耗时 {self.total_time_s:.2f} s")


# --------------------------------------------------------------------------- #
# 主入口
# --------------------------------------------------------------------------- #
def plan_speed_profile(s_mm: Sequence[float],
                       curvature: np.ndarray,
                       limits: SpeedLimits,
                       smooth_passes: int = 3) -> SpeedProfile:
    """核心: 由弧长 + 曲率算速度剖面(输入 mm, 输出 mm/s)."""
    s = np.asarray(s_mm, dtype=np.float64).copy()
    k = np.asarray(curvature, dtype=np.float64).copy()
    n = len(s)
    if n < 2:
        return SpeedProfile(s, np.zeros(n), np.zeros(n), k, limits, 0.0, 0.0)

    # 保证 s 严格单调(重复点会让 ds=0 除零)
    for i in range(1, n):
        if s[i] <= s[i - 1]:
            s[i] = s[i - 1] + 1e-6
    ds = np.diff(s)                                     # mm

    # ---- 统一换算到毫米制 ----
    v_max = max(0.0, limits.v_max) * 1000.0             # mm/s
    a_max = max(1e-6, limits.a_max) * 1000.0            # mm/s^2
    j_max = max(0.0, limits.j_max) * 1000.0             # mm/s^3
    a_lat = max(0.0, limits.a_lat_max) * 1000.0         # mm/s^2
    v_min = max(0.0, limits.v_min) * 1000.0             # mm/s
    v_start = max(0.0, limits.v_start) * 1000.0
    v_end = max(0.0, limits.v_end) * 1000.0
    wz_max = max(0.0, limits.wz_max)                     # rad/s
    half_w = max(0.0, limits.center_half_width) * 1000.0  # mm

    # ---- 1) 曲率限速 ----
    v_lim = np.full(n, v_max, dtype=np.float64)
    abs_k = np.abs(k)
    curved = abs_k > 1e-12
    if a_lat > 0.0 and curved.any():
        v_curve = np.full(n, v_max, dtype=np.float64)
        v_curve[curved] = np.sqrt(a_lat / abs_k[curved])
        v_lim = np.minimum(v_lim, v_curve)
    if wz_max > 0.0 and curved.any():
        v_wz = np.full(n, v_max, dtype=np.float64)
        v_wz[curved] = wz_max / abs_k[curved] * 1000.0
        v_lim = np.minimum(v_lim, v_wz)
    if half_w > 0.0 and wz_max > 0.0 and curved.any():
        v_side = np.full(n, v_max, dtype=np.float64)
        v_side[curved] = (wz_max * half_w) / abs_k[curved] * 1000.0
        v_lim = np.minimum(v_lim, v_side)

    # ---- 2) 时间最优包络 = min(前向可达, 后向可达) ----
    v = _reachability_envelope(v_lim, s, a_max, v_start, v_end)

    # 限幅后再夹一次, 保证可行
    v = np.minimum(v, v_lim)
    v = np.minimum(v, v_max)
    v = np.maximum(v, 0.0)
    v[0] = min(v[0], v_start)
    v[-1] = min(v[-1], v_end)

    # ---- 4) 最低速度(只抬运动点) ----
    if v_min > 0.0 and n > 2:
        interior = np.ones(n, dtype=bool)
        interior[0] = False
        interior[-1] = False
        v[interior] = np.maximum(v[interior], v_min)
    if limits.v_end > 0.0:
        v[-1] = max(v[-1], v_end)
    if limits.v_start > 0.0:
        v[0] = max(v[0], v_start)

    # ---- 5) 时间积分 ----
    v_mid = 0.5 * (v[1:] + v[:-1])
    v_mid = np.where(v_mid < 1.0, 1.0, v_mid)
    total_time = float(np.sum(ds / v_mid))

    return SpeedProfile(s, v, v_lim, k, limits, total_time, float(np.max(v)))


def _reachability_envelope(v_lim: np.ndarray, s_mm: np.ndarray, a_max_mm: float,
                           v_start_mm: float, v_end_mm: float) -> np.ndarray:
    """时间最优速度包络(不含 jerk) = min(前向可达, 后向可达), 解析求解.

    物理依据: |a| <= a_max 时
        v^2 <= v_start^2 + 2*a_max*s           (s = 距起点的弧长 mm)
        v^2 <= v_end^2   + 2*a_max*(L - s)     (L = 总长 mm)
    两条都是 v^2 关于 s 的线性函数, 逐点取小即可。

    ⚠ 不要换成逐格递推 `v[i] = sqrt(v[i-1]^2 + 2*a*ds)`: 每步增量只有
    2*a*ds 量级(典型 15 (mm/s)^2), 而 v^2 会涨到 1e6 量级, float64 在
    "小增量累加大数"时会丢增量, 实测把 3m 处的 3000mm/s 压成 94.9mm/s。
    """
    n = len(v_lim)
    if n < 2:
        return v_lim.astype(np.float64).copy()

    total_mm = float(s_mm[-1])
    two_a = 2.0 * a_max_mm
    v_from_start = np.sqrt(np.maximum(0.0, v_start_mm ** 2 + two_a * s_mm))
    v_from_end = np.sqrt(np.maximum(0.0, v_end_mm ** 2 + two_a * (total_mm - s_mm)))

    env = np.minimum(v_lim.astype(np.float64), np.minimum(v_from_start, v_from_end))

    # 还要考虑 v_lim 自身跳变带来的额外可达性约束: 解析式只处理了
    # "v_start/v_end 两个端点"这一个恒定加速度限制, 而曲率限速会让 v_lim
    # 在某处突然降低(例如直线 1500 直接掉到弯道 1095)。此时车必须提前减速,
    # 否则在限速跳变处会出现"要 105 m/s^2 才减得下来"的不物理结果。
    #
    # 做法: 再叠一层"软上限"前向传播 sqrt(cap^2 + 2a*ds)。因为每步用的都是
    # 最大可行加速度, 所以结果一定可行。
    # ⚠ 为什么这里可以逐格递推而前面不行: 这里每步都是 min(可能不变), 一旦
    # cap 饱和就不再增长, 不存在"小增量累加到已饱和大数"导致的精度丢失。
    soft = np.minimum(env, v_lim.astype(np.float64))
    for i in range(1, n):
        ds = s_mm[i] - s_mm[i - 1]
        if ds <= 0.0:
            continue
        reachable = math.sqrt(max(0.0, soft[i - 1] ** 2 + two_a * ds))
        if soft[i] > reachable:
            soft[i] = reachable

    # 同理做一次反向(终点侧同样可能是被限速卡住的)
    for i in range(n - 2, -1, -1):
        ds = s_mm[i + 1] - s_mm[i]
        if ds <= 0.0:
            continue
        reachable = math.sqrt(max(0.0, soft[i + 1] ** 2 + two_a * ds))
        if soft[i] > reachable:
            soft[i] = reachable

    return np.maximum(soft, 0.0)


def _s_curve_scan(v_lim: np.ndarray, s_mm: np.ndarray, a_max: float,
                  j_max: float, v_start: float, v_end: float) -> np.ndarray:
    """时间最优 + jerk 受限的前向/后向扫描(全部毫米制, 自变量为弧长).

    做法
    ----
    从 (v_i, a_i) 推 (v_{i+1}, a_{i+1}), 每步同时受两个约束:
      * 加速度上限      |a| <= a_max
      * jerk 上限       |da/dt| <= j_max,  dt = ds / v_mid
    把 jerk 约束换算成"每步允许的加速度增量":
        da = j_max * dt = j_max * ds / v_mid
    在 v 还没起来时(v_mid 很小), dt 大, da 也大, 所以起步段会自动走得动;
    速度起来之后 da 变小, 加速度就变得平滑 —— 这正是 S 型曲线该有的形状。

    前向从起点推到终点(决定"能加速到多快"), 后向从终点推回起点
    (决定"为了刹住最多能多快"), 逐点取小 = 时间最优解。

    ⚠ 踩过的坑
      1) 先用 v^2 解析包络再在速度域限 |Δv|, 结果要把加速度拉到 212 m/s^2。
      2) 用"加速度域投影再积分"的两段式, 起步段被压得远低于可行值。
      3) 千万别用逐格递推 v^2 += 2*a*ds 算包络: 增量只有 15 量级而 v^2 到 1e6,
         float64 会吃掉增量(3m 处 3000mm/s 被压成 94.9)。
    """
    n = len(v_lim)
    if n < 2:
        return v_lim.astype(np.float64).copy()

    cap = v_lim.astype(np.float64).copy()

    def sweep(v_boundary: float, forward: bool) -> np.ndarray:
        v = cap.copy()
        if forward:
            v[0] = min(v[0], v_boundary)
            idx = range(1, n)
        else:
            v[-1] = min(v[-1], v_boundary)
            idx = range(n - 2, -1, -1)

        a = 0.0
        for i in idx:
            j = i - 1 if forward else i + 1
            ds = abs(s_mm[i] - s_mm[j])
            v_j = max(1.0, v[j])
            if j_max > 0.0:
                da = j_max * ds / v_j
            else:
                da = a_max
            a_cap = min(a_max, abs(a) + da)
            if a < 0.0:
                a_cap = -a_cap
            v_phys = math.sqrt(max(0.0, v_j * v_j + 2.0 * a_cap * ds))
            v[i] = max(0.0, min(v[i], v_phys))
            if ds > 1e-9:
                a = (v[i] * v[i] - v_j * v_j) / (2.0 * ds)
            a = min(max(a, -a_max), a_max)
        return v

    v_fwd = sweep(v_start, True)
    v_bwd = sweep(v_end, False)
    return np.minimum(v_fwd, v_bwd)


def _apply_jerk_limit(v: np.ndarray, ds: np.ndarray, j_max: float,
                      a_max: float, v_max: float, passes: int) -> np.ndarray:
    """把 jerk 上限施加到速度包络上(**只在加速度域限幅**, 单位全为毫米制).

    推导
    ----
    曲线上 v 是弧长 s 的函数, 纵向加速度
        a = v * (dv/ds)        [mm/s^2]
    jerk 受限:  |da/dt| = |da/ds| * v <= j_max
      => |a_i - a_{i-1}| <= (j_max / v_i) * ds_i           ...(J)

    做法: 迭代地
      1. 由当前 v 估 a_i;
      2. 前向/反向各做一次投影 a_i <- clip(a_i, a_{i±1} ∓ J, a_{i±1} ± J);
      3. 夹到 ±a_max;
      4. 由 a_i 反解该点允许的速度上限
             v_{i+1} = v_i + a_i*ds / (v_i + a_i*ds/2)     (变加速解析解)
         并用 min() 压到当前包络上 —— 只减不增, 结果一定可行。

    ⚠ 踩过的坑(两次)
      1) 在速度域写 |Δv| <= (j_max/v_floor)*ds 并取 v_floor=0.02,
         低速段每步只能加约 1.8mm/s, 从起步就把峰值掐在 95mm/s。
      2) 用 v_{i+1} = v_i + a_i*ds/v_i 积分, v_i≈0 时同样把起步龟速化。
         必须用 (v_i+v_{i+1})/2 的变加速解析解。
    """
    out = v.astype(np.float64).copy()
    n = len(out)
    if j_max <= 0.0 or n < 3:
        return out

    a = np.zeros(n, dtype=np.float64)
    vm = np.maximum(1.0, out)
    for i in range(n - 1):
        a[i] = vm[i] * (out[i + 1] - out[i]) / max(1e-6, ds[i])
    a[-1] = a[-2]

    for _ in range(max(1, passes)):
        for i in range(1, n):
            v_i = max(1.0, out[i])
            J = j_max * ds[i - 1] / v_i
            a[i] = min(max(a[i], a[i - 1] - J), a[i - 1] + J)
            a[i] = min(max(a[i], -a_max), a_max)
        for i in range(n - 2, -1, -1):
            v_i = max(1.0, out[i])
            J = j_max * ds[i] / v_i
            a[i] = min(max(a[i], a[i + 1] - J), a[i + 1] + J)
            a[i] = min(max(a[i], -a_max), a_max)

    v_new = out.copy()
    for i in range(n - 1):
        v_i = max(1.0, v_new[i])
        acc = a[i]
        denom = v_i + acc * ds[i] * 0.5
        if denom <= 1e-3:
            continue
        v_next = v_i + acc * ds[i] / denom
        v_new[i + 1] = min(v_new[i + 1], max(0.0, v_next), v_max)
    return v_new


def trajectory_times(s_mm: np.ndarray, v_mm_s: np.ndarray) -> np.ndarray:
    """由 (s, v) 反推每个点的时刻, 用于画 v-t 曲线."""
    n = len(s_mm)
    if n < 2:
        return np.zeros(n)
    ds = np.diff(s_mm)
    v_mid = 0.5 * (v_mm_s[1:] + v_mm_s[:-1])
    v_mid = np.where(v_mid < 1.0, 1.0, v_mid)
    return np.concatenate(([0.0], np.cumsum(ds / v_mid)))
