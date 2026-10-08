"""端到端流水线: A* -> LOS 简化 -> B 样条平滑 -> 速度规划 -> 点表.

各步骤彼此解耦, 任一步可以关掉(比如只想看折线就 use_spline=False),
也方便在 UI 上单独重跑某一步。
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .core.costmap import CostMap, build_costmap
from .core.geometry import Point, Pose, wrap_pi
from .core.scene import Scene
from .core.surface_navigation import SurfaceMap, required as surface_required
from .core.trajectory import Trajectory, TrajectoryMeta, TrajectoryPoint
from .plan.astar import AStarResult, astar_path
from .plan.curvature import compute_geometry, yaw_at
from .plan.fillet import fillet_polyline
from .plan.los import simplify_by_los
from .plan.spline import bspline_smooth_path
from .speed.profile import SpeedLimits, plan_speed_profile


@dataclass
class PipelineConfig:
    """一次生成的全部可调参数."""

    route_name: str = "TR_route_01"

    # 地图
    resolution_mm: float = 25.0
    # Configured circular envelope; must include chassis/arm/load projections.
    # A 700x700 square needs a 495mm enclosing radius, not 350mm.
    # Keep the existing default; actual robot dimensions must be configured.
    robot_radius_mm: float = 300.0
    # 5mm 留白: 曲线平滑 + 按弧长重采样会带来 1~2mm 的擦边, 没有余量时
    # 最终点表可能有个别点离障 298mm(要求 300)。实测 5mm 就足以让典型路线全部干净通过。
    safety_margin_mm: float = 5.0

    # 规划
    use_spline: bool = True
    simplify_los: bool = True
    spline_smooth: float = 0.0
    spline_step_mm: float = 5.0
    min_radius_mm: float = 400.0          # 最小转弯半径 -> 曲率上限 = 1/R
    spline_max_depth: int = 4
    profile_step_mm: float = 10.0         # 速度剖面的计算步长(必须远小于点表间距)

    # 点表
    point_gap_mm: float = 100.0
    max_points: int = 512                 # 0 = 不限
    heading_mode: str = "tangent"         # tangent | linear
    prune_collinear: bool = True          # 丢掉直线段上的冗余点
    prune_tolerance_mm: float = 2.0       # 共线容差(mm)
    max_point_gap_mm: float = 200.0       # 精简后允许的最大点间距(保证纯跟踪有点可用)
    curvature_eps: float = 2.0e-6         # 曲率噪声门限 1/mm (0 = 不过滤)

    # 速度
    v_max_mps: float = 1.5
    a_max_mps2: float = 1.5
    j_max_mps3: float = 25.0
    a_lat_max_mps2: float = 1.2
    v_min_mps: float = 0.06
    v_start_mps: float = 0.0
    v_end_mps: float = 0.0
    center_half_width_m: float = 0.0
    wz_max_rad_s: float = 2.5

    # 其它
    keep_searched: bool = False
    climb_mode: str = "ground"            # ground by default; climb modes are not exposed in UI
    team: str = "auto"                    # auto | red | blue
    origin_mm: Tuple[float, float] = (0.0, 0.0)

    # 哪些"会变的东西"要当成障碍
    include_tower_tops: bool = False      # 地面公共区的塔顶阵列
    include_stacks: bool = False          # 储存区的料堆
    extra_blocked_tags: Tuple[str, ...] = ()
    # 场景 JSON 里可选的语义分组 "group" (见 AGENT.md 第 4 节):
    #   fixed / climb / transfer / dynamic / reference
    # 这两个是"按语义批量开/关"的口子, 比逐个 tag 方便。
    enable_groups: Tuple[str, ...] = ()   # 强制打开的 group
    disable_groups: Tuple[str, ...] = ()  # 强制关闭的 group

    def speed_limits(self) -> SpeedLimits:
        return SpeedLimits(
            v_max=self.v_max_mps,
            a_max=self.a_max_mps2,
            j_max=self.j_max_mps3,
            a_lat_max=self.a_lat_max_mps2,
            v_min=self.v_min_mps,
            v_start=self.v_start_mps,
            v_end=self.v_end_mps,
            center_half_width=self.center_half_width_m,
            wz_max=self.wz_max_rad_s,
        )


def resolve_scene(scene: Scene, cfg: PipelineConfig) -> Scene:
    """按配置生成一份"实际用于栅格化"的场景副本.

    永久障碍(围栏/隔板/基座/L1 台体)始终算; 坡道/阶梯**始终不可通行**
    (规则 2.16/3.2.1: 它们是登高结构不是平地); 传递区始终可停留;
    塔顶和料堆这类位置会变的东西默认不算, 免得把通道挤到过不去。

    ⚠ 注意: ramp_stair / transfer / l1_body / fence / divider / pedestal
    是**通行性铁律**, 本函数不提供关闭它们的开关 —— 关了就会撞上去或到不了传递区。
    """
    tmp = Scene.from_dict(scene.to_dict())
    tmp.resolution = cfg.resolution_mm
    tmp.robot_radius = cfg.robot_radius_mm
    tmp.safety_margin = cfg.safety_margin_mm

    disable = []
    if not cfg.include_tower_tops:
        disable.append("tower_top")
    if not cfg.include_stacks:
        disable.append("stack")
    for tag in disable:
        tmp.set_obstacle_enabled(tag, False)
    for tag in cfg.extra_blocked_tags:
        tmp.set_obstacle_enabled(tag, True)

    # 新增的语义分组(group)开关
    for g in cfg.disable_groups:
        tmp.set_group_enabled(g, False)
    for g in cfg.enable_groups:
        tmp.set_group_enabled(g, True)

    # 兜底: 不管上面怎么开, 通行性铁律必须成立
    tmp.set_obstacle_enabled("ramp_stair", True)
    tmp.set_obstacle_enabled("l1_body", True)
    tmp.set_obstacle_enabled("fence", True)
    tmp.set_obstacle_enabled("divider", True)
    tmp.set_obstacle_enabled("pedestal", True)
    tmp.set_obstacle_enabled("transfer", False)
    return tmp


def build_map(scene: Scene, cfg: PipelineConfig) -> CostMap:
    """按配置把场景栅格化."""
    tmp = resolve_scene(scene, cfg)
    if surface_required(tmp):
        return SurfaceMap(tmp, cfg.resolution_mm,climb_mode=cfg.climb_mode)
    return build_costmap(tmp, cfg.resolution_mm)


def _collinear_keep(x: np.ndarray, y: np.ndarray, k: np.ndarray,
                    tol_mm: float) -> np.ndarray:
    """找出"直线段上的冗余点"并返回要保留的下标.

    判据: 若某点到前后两个保留点连线的垂直距离 < tol, 且该点处曲率接近 0,
    就认为它是冗余的(丢掉不影响几何)。首末点永远保留。
    """
    n = len(x)
    if n < 4:
        return np.arange(n)
    keep = [0]
    anchor = 0
    for i in range(1, n - 1):
        if abs(k[i]) > 1e-7:
            # 曲率不为 0 的点必须先保留(它承载了转弯信息)
            keep.append(i)
            anchor = i
            continue
        # 当前点到 anchor->i+1 这条线的距离
        x1, y1 = x[anchor], y[anchor]
        x2, y2 = x[i + 1], y[i + 1]
        dx, dy = x2 - x1, y2 - y1
        seg = math.hypot(dx, dy)
        if seg < 1e-9:
            continue
        dist = abs((x[i] - x1) * dy - (y[i] - y1) * dx) / seg
        if dist > tol_mm:
            keep.append(i)
            anchor = i
    keep.append(n - 1)
    return np.asarray(sorted(set(keep)), dtype=np.int64)


def plan_route(scene: Scene,
               start: Pose,
               goal: Pose,
               cfg: PipelineConfig,
               costmap: Optional[CostMap] = None,
               progress: Optional[callable] = None) -> Trajectory:
    """跑完整条流水线, 返回可直接导出的 Trajectory."""

    def step(msg: str) -> None:
        if progress is not None:
            progress(msg)

    t0 = time.perf_counter()
    traj = Trajectory()
    messages: List[str] = []

    # ---------------- 1) 栅格地图 ----------------
    step("构建栅格地图...")
    cm = costmap if costmap is not None else build_map(scene, cfg)
    if surface_required(scene):
        # A caller-supplied ground map cannot bypass physical access checks.
        team = cfg.team if cfg.team!="auto" else ("red" if start.x < 0 else "blue" if start.x > 0 else ("red" if goal.x < 0 else "blue"))
        cm = SurfaceMap(resolve_scene(scene,cfg),cfg.resolution_mm,team=team,climb_mode=cfg.climb_mode)
        if not cm.is_free_world(start.x,start.y) or not cm.is_free_world(goal.x,goal.y):
            traj.messages=["当前仅规划地面区：起终点不得位于坡道、阶梯、高平台、障碍膨胀区或对方区域；拒绝吸附跨区。" if cfg.climb_mode=="ground" else "起终点无合法支承/净空或越入对方区域；拒绝吸附跨区。"]
            return traj
        messages.append(f"{team}: 仅地面区；坡道、阶梯、传递平台均封闭，路径绕行。" if cfg.climb_mode=="ground" else f"{team}: 地面→端口→坡道/阶梯→传递区；禁止侧入；圆包络含安全余量。")
    if cm.free_cell_count() < 16:
        messages.append("可行区域太小: 检查车体半径/安全余量是否过大")
        traj.messages = messages
        return traj

    # ---------------- 2) A* ----------------
    step("A* 搜索...")
    start_pt = (start.x, start.y)
    goal_pt = (goal.x, goal.y)
    astar_res: AStarResult = astar_path(cm, start_pt, goal_pt,
                                        collect_searched=cfg.keep_searched)
    if not astar_res.ok:
        messages.append(f"A* 失败: {astar_res.message}")
        traj.messages = messages
        return traj
    if astar_res.start_snapped_mm > cfg.resolution_mm * 1.5:
        messages.append(f"提示: 起点被吸附了 {astar_res.start_snapped_mm:.0f}mm "
                        f"(起点在膨胀区内)")
    if astar_res.goal_snapped_mm > cfg.resolution_mm * 1.5:
        messages.append(f"提示: 终点被吸附了 {astar_res.goal_snapped_mm:.0f}mm "
                        f"(终点在膨胀区内)")

    # 吸附后的路径端点必须重新校验一次: 吸附有可能把点挪到障碍另一侧
    # (实测: 点在中央隔板上被吸附 313mm, 直接跨到了对方半场)。
    if not cm.is_free_world(*astar_res.path[0]) or not cm.is_free_world(*astar_res.path[-1]):
        bad_end = "起点" if not cm.is_free_world(*astar_res.path[0]) else "终点"
        messages.append(f"错误: {bad_end}吸附后仍落在不可通行区, 拒绝生成点表。"
                        f"请把起终点往空地里挪一点(注意中央隔板/围栏都会占位置)。")
        traj.messages = messages
        return traj

    traj.searched_cells = astar_res.searched_cells
    astar_len = astar_res.length_mm

    # ---------------- 3) LOS 简化 ----------------
    key_points: List[Point] = list(astar_res.path)
    if cfg.simplify_los:
        step("视线法简化...")
        key_points = simplify_by_los(cm, key_points)
    traj.los_points = list(key_points)

    # ---------------- 4) 平滑 ----------------
    # 顺序: 圆弧倒角(fillet) -> B 样条 -> 折线加密。
    # 为什么优先 fillet: 定曲率圆弧的几何是解析可控的(曲率恒为 1/R), 而且能
    # 在 costmap 上先验校验; B 样条在窄通道里会切内线, 实测会切进障碍。
    curve_points: List[Point] = list(key_points)
    spline_clearance = float("inf")
    spline_segments = 0
    if cfg.use_spline and len(key_points) >= 3:
        step("圆弧倒角平滑...")
        max_kappa = (1.0 / cfg.min_radius_mm) if cfg.min_radius_mm > 0 else 0.0
        fr = fillet_polyline(key_points,
                             radius_mm=cfg.min_radius_mm,
                             costmap=cm,
                             arc_step_mm=cfg.spline_step_mm,
                             min_clearance_mm=cm.blocked_radius)
        accepted = False
        if fr.ok and fr.points and fr.min_clearance_mm >= cm.blocked_radius:
            curve_points = list(fr.points)
            spline_clearance = fr.min_clearance_mm
            spline_segments = fr.corners_rounded
            accepted = True
            messages.append(f"圆弧倒角: {fr.corners_rounded}/{fr.corners_total} 个拐角已圆化, "
                            f"离障 {fr.min_clearance_mm:.0f}mm, 最小转弯半径 "
                            f"{fr.min_radius_mm:.0f}mm "
                            f"(目标 {cfg.min_radius_mm:.0f}mm)")
            if max_kappa > 0.0 and fr.max_curvature > max_kappa:
                messages.append(f"注意: 有拐角受可用空间限制, 实际最小转弯半径 "
                                f"{fr.min_radius_mm:.0f}mm 小于目标 "
                                f"{cfg.min_radius_mm:.0f}mm, 该处速度会自动降下来")
            if fr.corners_failed:
                messages.append(f"注意: {fr.corners_failed} 个拐角放不下安全圆角, "
                                f"已保留原折点")

        if not accepted:
            step("B 样条平滑...")
            sp = bspline_smooth_path(key_points,
                                     costmap=cm,
                                     smooth=cfg.spline_smooth,
                                     sample_step_mm=cfg.spline_step_mm,
                                     max_curvature=max_kappa,
                                     max_depth=cfg.spline_max_depth)
            if sp.ok and sp.points and sp.min_clearance_mm >= cm.blocked_radius:
                curve_points = list(sp.points)
                spline_clearance = sp.min_clearance_mm
                spline_segments = sp.segments
                accepted = True
                messages.append(f"B 样条平滑: 离障 {sp.min_clearance_mm:.0f}mm, "
                                f"最小转弯半径 {sp.min_radius_mm:.0f}mm")
            else:
                # ⚠ 关键: 两种平滑都不达标时必须回退到"折线加密", 绝不接受失败结果
                messages.append(f"平滑未达标, 已回退为折线加密: "
                                f"{fr.message if fr.points else ''} "
                                f"{sp.message if sp.points else ''}".strip())
                curve_points = list(key_points)
    traj.smooth_points = list(curve_points)
    if isinstance(cm,SurfaceMap) and not cm.polyline_is_free(curve_points):
        curve_points=list(key_points)
        if not cm.polyline_is_free(curve_points):
            curve_points=list(astar_res.path)
        if not cm.polyline_is_free(curve_points):
            traj.messages=messages+["搜索/平滑路径违反端口或扫掠约束，拒绝生成。"]
            return traj
        messages.append("平滑触碰物理边界，使用已复核的折线。")

    # ---------------- 5) 几何量 ----------------
    # ⚠ 关键: 速度剖面必须在**细网格**上算, 再重采样到点表间距。
    # 否则如果关键点很稀(例如一条直线只有首末两个点), 前后向扫描会被
    # "相邻点间距"量化: 实测 100mm 间距下峰值只能到 95mm/s, 而解析解是
    # 1500mm/s。所以这里先按 profile_step_mm 加密, 与是否开平滑无关。
    step("加密曲线用于速度规划...")
    curve_arr = np.asarray([(float(p[0]), float(p[1])) for p in curve_points],
                           dtype=np.float64)
    if curve_arr.shape[0] >= 2:
        seg = np.hypot(np.diff(curve_arr[:, 0]), np.diff(curve_arr[:, 1]))
        total_rough = float(np.sum(seg))
        n_fine = int(math.ceil(total_rough / max(1.0, cfg.profile_step_mm))) + 1
        if n_fine > curve_arr.shape[0]:
            s_rough = np.concatenate(([0.0], np.cumsum(seg)))
            # Keep original vertices as well as uniform fine samples: otherwise
            # interpolation can create a chord across a bend or obstacle edge.
            s_fine = np.unique(np.concatenate((np.linspace(0.0,total_rough,n_fine),s_rough)))
            curve_arr = np.column_stack((np.interp(s_fine, s_rough, curve_arr[:, 0]),
                                         np.interp(s_fine, s_rough, curve_arr[:, 1])))
            curve_points = [(float(x), float(y)) for x, y in curve_arr]
            traj.smooth_points = list(curve_points)

    step("计算弧长/曲率...")
    # curvature_span_mm 用"点间距的若干倍"做差分跨度: 太短会被采样噪声淹没,
    # 太长会把真弯的曲率抹平。取 5 倍点间距、且不小于 30mm。
    geo = compute_geometry(
        curve_points,
        smooth_window=5,
        curvature_span_mm=max(30.0, 5.0 * max(1.0, cfg.point_gap_mm / 4.0)),
        curvature_eps=cfg.curvature_eps,
    )
    if len(geo.s_mm) < 2:
        messages.append("曲线点数不足, 无法生成点表")
        traj.messages = messages
        return traj

    # ---------------- 6) 速度剖面 ----------------
    step("速度规划...")
    limits = cfg.speed_limits()
    profile = plan_speed_profile(geo.s_mm, geo.curvature, limits)

    # ---------------- 7) 按弧长重采样成点表 ----------------
    step("生成点表...")
    gap = max(1.0, cfg.point_gap_mm)
    total_len = float(geo.s_mm[-1])
    n_pts = int(math.floor(total_len / gap)) + 1
    if cfg.max_points and n_pts > cfg.max_points:
        n_pts = cfg.max_points
        gap = total_len / max(1, (n_pts - 1))
        n_pts = int(math.floor(total_len / gap)) + 1
        messages.append(f"点间距自动放宽到 {gap:.1f}mm 以满足点数上限 {cfg.max_points}")
    n_pts = max(2, n_pts)

    s_out = np.linspace(0.0, total_len, n_pts)
    if isinstance(cm,SurfaceMap):
        # Preserve real bends. Resampling must not replace a bend with a chord
        # that crosses a platform edge. Straight-run points stay inexpensive.
        d=np.diff(geo.points,axis=0)
        cross=d[:-1,0]*d[1:,1]-d[:-1,1]*d[1:,0]
        bends=np.nonzero(np.abs(cross)>1e-6)[0]+1
        s_out=np.unique(np.concatenate((s_out,geo.s_mm[bends])))
        n_pts=len(s_out)
        if cfg.max_points and n_pts>cfg.max_points:
            traj.messages=messages+["保留安全拐点后超过点数上限，拒绝出表；请分段规划。"]
            return traj
    x_out = np.interp(s_out, geo.s_mm, geo.points[:, 0])
    y_out = np.interp(s_out, geo.s_mm, geo.points[:, 1])
    v_out = np.interp(s_out, geo.s_mm, profile.v_mm_s)
    k_out = np.interp(s_out, geo.s_mm, geo.curvature)
    yaw_tangent = np.array([yaw_at(geo.yaw_rad, geo.s_mm, s) for s in s_out])

    # 可选: 丢掉直线段上的冗余点(纯跟踪只要有足够的点插值就够了)
    if cfg.prune_collinear and n_pts > 8 and not isinstance(cm,SurfaceMap):
        keep = _collinear_keep(x_out, y_out, k_out, cfg.prune_tolerance_mm)
        # 保证点表不会过稀: 纯跟踪要找前视点, 太稀(每米不到 2 个)会抖
        min_keep = max(8, int(total_len / max(1.0, cfg.max_point_gap_mm)))
        if len(keep) < min_keep:
            stride = max(1, n_pts // min_keep)
            keep = np.arange(0, n_pts, stride, dtype=np.int64)
            if keep[-1] != n_pts - 1:
                keep = np.append(keep, n_pts - 1)
        if len(keep) >= 8:
            removed = n_pts - len(keep)
            s_out, x_out, y_out = s_out[keep], x_out[keep], y_out[keep]
            v_out, k_out = v_out[keep], k_out[keep]
            yaw_tangent = yaw_tangent[keep]
            n_pts = len(keep)
            if removed > 0:
                messages.append(f"直线段冗余点已精简 {removed} 个(容差 "
                                f"{cfg.prune_tolerance_mm:.0f}mm), 保留 {n_pts} 点")

    # 航向模式
    yaw_start = float(start.yaw)
    yaw_end = float(goal.yaw)
    if cfg.heading_mode == "linear":
        delta = wrap_pi(yaw_end - yaw_start)
        frac = s_out / max(1e-9, total_len)
        yaw_out = yaw_start + delta * frac
    else:
        yaw_out = yaw_tangent.copy()
        yaw_out[0] = yaw_start
        yaw_out[-1] = yaw_end

    # 速度处理: 首末 0; 其余按 v_min 抬升; 前馈按各自航向分解
    v_final = v_out.copy()
    v_final[0] = max(0.0, cfg.v_start_mps) * 1000.0
    v_final[-1] = max(0.0, cfg.v_end_mps) * 1000.0
    if isinstance(cm,SurfaceMap): v_final[0]=v_final[-1]=0.

    for i in range(n_pts):
        velocity_yaw = yaw_tangent[i] if isinstance(cm,SurfaceMap) else yaw_out[i]
        vx = v_final[i] * math.cos(velocity_yaw)
        vy = v_final[i] * math.sin(velocity_yaw)
        traj.points.append(TrajectoryPoint(vx=vx, vy=vy,
                                           x=float(x_out[i]), y=float(y_out[i]),
                                           yaw=float(yaw_out[i])))

    # ---------------- 7.5) 最终校验(硬门槛) ----------------
    # 点表里任何一点落在膨胀区里都是不可接受的, 这里做最后一次兜底:
    # 逐点插值会引入误差, 平滑曲线的密集校验点与最终重采样点也不是同一批。
    worst = 0.0
    bad_pts = 0
    for p in traj.points:
        c = cm.clearance_mm(p.x, p.y)
        if c < cm.blocked_radius:
            bad_pts += 1
        worst = max(worst, cm.blocked_radius - c)
    min_clear_table = float(min((cm.clearance_mm(p.x, p.y) for p in traj.points),
                                default=float("inf")))
    if bad_pts:
        messages.append(f"警告: 点表有 {bad_pts} 个点离障不足(最小 "
                        f"{min_clear_table:.0f}mm < {cm.blocked_radius:.0f}mm)")
    if isinstance(cm,SurfaceMap):
        try:
            traj.support_samples=cm.validate(traj.points)
        except ValueError as exc:
            traj.points=[]
            traj.messages=messages+[str(exc)]
            return traj
        traj.navigation_map=cm
        states=list(dict.fromkeys(s for s,z in traj.support_samples))
        messages.append("整段扫掠/端口校验 PASS: "+" → ".join(states))
    if spline_clearance == float("inf"):
        spline_clearance = min_clear_table

    # ---------------- 8) 元信息 ----------------
    # 用最终点表自己的几何量写元信息, 保证"注释里写的"和"表里的"一致
    final_geo = compute_geometry(
        [(p.x, p.y) for p in traj.points],
        smooth_window=3,
        curvature_span_mm=max(30.0, 2.0 * gap),
        curvature_eps=cfg.curvature_eps,
    )
    traj.meta = TrajectoryMeta(
        route_name=cfg.route_name,
        scene_name=scene.meta.name,
        resolution_mm=cfg.resolution_mm,
        robot_radius_mm=cfg.robot_radius_mm,
        safety_margin_mm=cfg.safety_margin_mm,
        origin_mm=tuple(cfg.origin_mm),
        start_pose=start.copy(),
        goal_pose=goal.copy(),
        point_gap_mm=float(gap),
        v_max_mps=cfg.v_max_mps,
        a_max_mps2=cfg.a_max_mps2,
        j_max_mps3=cfg.j_max_mps3,
        a_lat_max_mps2=cfg.a_lat_max_mps2,
        v_min_mps=cfg.v_min_mps,
        heading_mode=cfg.heading_mode,
        length_mm=total_len,
        min_radius_mm=(1.0 / final_geo.max_abs_curvature)
        if final_geo.max_abs_curvature > 1e-9 else float("inf"),
        max_curvature=final_geo.max_abs_curvature,
        total_time_s=profile.total_time_s,
        peak_v_mm_s=profile.peak_v_mm_s,
        min_clearance_mm=float(spline_clearance),
        astar_expanded=astar_res.expanded,
        astar_length_mm=astar_len,
        los_points=len(key_points),
        spline_points=len(curve_points),
        spine_segments=spline_segments,
        notes="; ".join(messages),
        created_at=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    )

    traj.speed_s_mm = np.asarray(profile.s_mm, dtype=np.float64)
    traj.speed_v_mm_s = np.asarray(profile.v_mm_s, dtype=np.float64)
    traj.curvature = np.asarray(final_geo.curvature, dtype=np.float64)
    traj.messages = list(messages)

    step(f"完成: {n_pts} 点, {total_len / 1000.0:.3f} m, "
         f"{profile.total_time_s:.2f} s ({(time.perf_counter() - t0) * 1000:.0f} ms)")
    return traj
