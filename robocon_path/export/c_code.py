"""生成可直接编译进工程的 C / C++ 点表文件.

三种风格(对应你手上的两套参考工程)
----------------------------------
1. `float_array`  : `float name[][5] = {...};` + `Pos_control_t name_path = {...};`
                    对应 Chassis2026_R1_H723/Path_tracking/path_data.c 的写法。
2. `path_point`   : `const PathFollower::PathPoint name[] = {...};`
                    + `const uint16_t name_count = N;`
                    对应 2026R2_conbat/MDK-ARM/Route_Plan/path.cpp 的写法。
3. `c_struct`     : 纯 C 的 `TR_PathPoint_t name[] = {...};`, 不依赖任何 C++ 类,
                    给不想改类型定义的工程用。

所有的 x/y 都会按 origin 平移, 首末点速度强制为 0(触发终点精定位)。
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Sequence, Tuple

from ..core.trajectory import Trajectory
from ..core.file_io import atomic_text
from .csv_export import rows_for_export

STYLE_FLOAT_ARRAY = "float_array"
STYLE_PATH_POINT = "path_point"
STYLE_C_STRUCT = "c_struct"


@dataclass
class CCodeConfig:
    array_name: str = "TR_route_01"
    style: str = STYLE_FLOAT_ARRAY
    origin_mm: Optional[Tuple[float, float]] = None
    indent: str = "    "
    per_line: int = 4                 # float_array 每行放几个点
    include_comments: bool = True
    header_guard: bool = True
    declare_extern: bool = True


def _fmt(v: float, nd: int = 3) -> str:
    if v != v:                        # NaN
        return "0.0f"
    if v in (float("inf"), float("-inf")):
        return "0.0f"
    return f"{v:.{nd}f}f"


def _row_tag(index: int, total: int) -> str:
    """给点表加行尾注释; 只在首点和末点上标."""
    if index == 0:
        return "   /* start, v=0 */"
    if index == total - 1:
        return "   /* goal, v=0 */"
    return ""


def _meta_comment(traj: Trajectory, name: str) -> List[str]:
    m = traj.meta
    lines = [
        "/*",
        f" * {name}  --  ROBOCON 2027 TR 离线路径点表 (自动生成, 不要手改)",
        f" *",
        f" * 生成时间   : {m.created_at}",
        f" * 场景       : {m.scene_name}",
        f" * 栅格分辨率 : {m.resolution_mm:.1f} mm/格",
        f" * 车体半径   : {m.robot_radius_mm:.0f} mm (+安全余量 {m.safety_margin_mm:.0f} mm)",
        f" * 路径长度   : {m.length_mm:.1f} mm",
        f" * 点数       : {traj.count} (间距 {m.point_gap_mm:.1f} mm)",
        f" * 预计耗时   : {m.total_time_s:.3f} s (峰值 {m.peak_v_mm_s:.0f} mm/s)",
        f" * 速度规划   : vmax={m.v_max_mps} m/s, amax={m.a_max_mps2} m/s^2, "
        f"jmax={m.j_max_mps3} m/s^3, alat_max={m.a_lat_max_mps2} m/s^2",
        f" * 最小转弯半径: {m.min_radius_mm:.0f} mm",
        f" * 起点位姿   : x={m.start_pose.x:.1f}, y={m.start_pose.y:.1f}, "
        f"yaw={m.start_pose.yaw:.4f} rad",
        f" * 终点位姿   : x={m.goal_pose.x:.1f}, y={m.goal_pose.y:.1f}, "
        f"yaw={m.goal_pose.yaw:.4f} rad",
        f" * 点表原点   : ({m.origin_mm[0]:.1f}, {m.origin_mm[1]:.1f}) mm "
        f"(已从坐标中扣除)",
        f" * 航向模式   : {m.heading_mode}",
        f" * A* 展开格数: {m.astar_expanded}, 简化后关键点 {m.los_points}, "
        f"平滑采样 {m.spline_points}",
    ]
    if m.notes:
        lines.append(f" * 备注       : {m.notes}")
    lines.append(" *")
    lines.append(" * 点格式: {vx[mm/s], vy[mm/s], x[mm], y[mm], yaw[rad]}")
    lines.append(" *   首点/末点速度必须为 0 (跟点器用它触发终点精定位)")
    lines.append(" */")
    return lines


def trajectory_to_c_code(traj: Trajectory, cfg: Optional[CCodeConfig] = None) -> str:
    cfg = cfg or CCodeConfig()
    name = cfg.array_name.strip() or "TR_route_01"
    rows = rows_for_export(traj, cfg.origin_mm)

    out: List[str] = []
    if cfg.include_comments:
        out += _meta_comment(traj, name)
        out.append("")

    if cfg.style == STYLE_PATH_POINT:
        out.append(f"const PathFollower::PathPoint {name}[] = {{")
        for i, r in enumerate(rows):
            out.append(f"{cfg.indent}{{{_fmt(r[0])}, {_fmt(r[1])}, {_fmt(r[2])}, "
                       f"{_fmt(r[3])}, {_fmt(r[4], 6)}}},{_row_tag(i, len(rows))}")
        out.append("};")
        out.append(f"const uint16_t {name}_count = {len(rows)}U;")

    elif cfg.style == STYLE_C_STRUCT:
        out.append("typedef struct")
        out.append("{")
        out.append(f"{cfg.indent}float vx_mm_s;")
        out.append(f"{cfg.indent}float vy_mm_s;")
        out.append(f"{cfg.indent}float x_mm;")
        out.append(f"{cfg.indent}float y_mm;")
        out.append(f"{cfg.indent}float yaw_rad;")
        out.append("} TR_PathPoint_t;")
        out.append("")
        out.append(f"const TR_PathPoint_t {name}[] = {{")
        for i, r in enumerate(rows):
            out.append(f"{cfg.indent}{{{_fmt(r[0])}, {_fmt(r[1])}, {_fmt(r[2])}, "
                       f"{_fmt(r[3])}, {_fmt(r[4], 6)}}},{_row_tag(i, len(rows))}")
        out.append("};")
        out.append(f"const uint16_t {name}_count = {len(rows)}U;")

    else:  # STYLE_FLOAT_ARRAY
        out.append(f"float {name}[][5] = {{")
        per_line = max(1, int(cfg.per_line))
        for base in range(0, len(rows), per_line):
            chunk = rows[base:base + per_line]
            cells = ", ".join(
                "{" + ", ".join(_fmt(v, 6 if idx == 4 else 3) for idx, v in enumerate(r)) + "}"
                for r in chunk
            )
            # 注释按这一行的"最后一点"算, 否则 start 注释会挂到第 4 个点上
            last_idx = base + len(chunk) - 1
            out.append(f"{cfg.indent}{cells},{_row_tag(last_idx, len(rows))}")
        out.append("};")
        out.append("")
        out.append(f"Pos_control_t {name}_path = {{( float (*)[5] ){name}, "
                   f"0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};")
        out.append(f"/* 点数: {len(rows)} */")

    return "\n".join(out) + "\n"


def write_c_code(path: str, traj: Trajectory,
                 cfg: Optional[CCodeConfig] = None) -> None:
    atomic_text(path,trajectory_to_c_code(traj,cfg))
