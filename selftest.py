"""无头自检: 不启动界面, 直接跑一遍完整流水线并输出结果图.

用法:
    python selftest.py                 # 用内置默认场地
    python selftest.py scene.json      # 用指定场景

会打印每一步的中间量, 并在 out/ 下生成:
    out/selftest_map.png       栅格地图 + 路径叠加
    out/selftest_curves.png    弧长-速度 / 弧长-曲率 曲线
    out/selftest_route.csv     点表 CSV
    out/selftest_route.c       点表 C 代码
"""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from robocon_path.core.geometry import Pose                      # noqa: E402
from robocon_path.core.scene import Scene, build_default_scene    # noqa: E402
from robocon_path.export import rows_for_export, trajectory_to_c_code, trajectory_to_csv  # noqa: E402
from robocon_path.export.c_code import CCodeConfig                # noqa: E402
from robocon_path.pipeline import PipelineConfig, build_map, plan_route  # noqa: E402
from robocon_path.core.file_io import atomic_text, atomic_png
from robocon_path.core.surface_navigation import required as surface_required


def pick_points(scene: Scene):
    """在红队半场挑一对起终点: 启动区A -> 红传递区.

    这条路线会穿过 L1 南边界的换乘通道缺口(坡道/阶梯不能当平地走, 但通道
    中央那条地面车道可以走), 是条有弯、且能验证避障的路线。
    """
    if surface_required(scene):
        return Pose(-4200.,4300.,0.),Pose(-1200.,-4100.,0.)
    start = Pose(-5100.0, -4550.0, 0.0)
    goal = Pose(-3500.0, -2100.0, 0.0)      # 红传递区中心
    return start, goal


# --------------------------------------------------------------------------- #
# 场地连通性验收: python selftest.py --verify [场景.json]
# --------------------------------------------------------------------------- #
# 这些点位就是规则里的功能位置。改过场地模板(JSON)之后必须跑这个, 全 OK 才算没改坏。
VERIFY_ROUTES = [
    # (名称, 起点, 终点, 说明)
    ("启动区A -> 红传递区", (-5100.0, -4550.0), (-500.0, -2100.0),
     "最核心的一条: TR 要从启动区走到传递区交接"),
    ("红传递区 -> 第一公共区", (-500.0, -2100.0), (-450.0, -4250.0),
     "去南方第一公共区取塔顶"),
    ("第一公共区 -> 红传递区", (-450.0, -4250.0), (-500.0, -2100.0),
     "取完送回传递区"),
    ("红储存区 -> 红传递区", (-4500.0, 4400.0), (-500.0, -2100.0),
     "从北方储存区横穿到传递区"),
    ("启动区A -> 红储存区", (-5100.0, -4550.0), (-4500.0, 4400.0),
     "沿西跑道南北贯通"),
    ("环绕红半场", (-5100.0, -4550.0), (-5100.0, 4400.0),
     "验证地面环整体连通"),
    ("北跑道红侧 -> 红传递区", (-4250.0, 4000.0), (-500.0, -2100.0),
     "北侧绕回传递区"),
    ("启动A -> 第二公共区红侧", (-5100.0, -4550.0), (-600.0, 4000.0),
     "去北方第二公共区(五色石)"),
]
# 必须**不可通行**的点(坡道/阶梯面上), 用来确认登高结构没被误标成平地
VERIFY_BLOCKED = [
    ("坡道红中心", -1000.0, 0.0),
    ("阶梯红中心", -1800.0, -2200.0),
    ("L1 台体中心", 0.0, 0.0),
]
# 必须**可停留**的点(传递区面上)
VERIFY_FREE = [
    ("红传递区中心", -500.0, -2100.0),
    ("蓝传递区中心", 500.0, -2100.0),
]


def verify_only(scene_path: str | None) -> int:
    """只做场地校验, 不规划点表。改过场景 JSON 后跑这个。"""
    if scene_path and os.path.isfile(scene_path):
        scene = Scene.load(scene_path)
        print(f"校验场景: {scene_path}")
        if surface_required(scene):
            from verify_from_sim import main as verify_ground
            return verify_ground(scene_path)
    else:
        scene = build_default_scene()
        print("校验内置默认场地")

    cfg = PipelineConfig(robot_radius_mm=scene.robot_radius,
                         safety_margin_mm=scene.safety_margin)
    cm = build_map(scene, cfg)
    print(f"栅格 {cm.spec.width}x{cm.spec.height} @ {cfg.resolution_mm:.0f}mm, "
          f"可行 {cm.free_cell_count()} 格, 阻塞半径 {cm.blocked_radius:.0f}mm")
    print()

    failures: list[str] = []

    # 1) 连通性: 每条路线都要能搜通, 且点表 0 越界
    print("[1] 路线连通性")
    for name, s, g, why in VERIFY_ROUTES:
        traj = plan_route(scene, Pose(*s, 0.0), Pose(*g, 0.0), cfg, costmap=cm)
        bad = sum(1 for p in traj.points if not cm.is_free_world(p.x, p.y))
        ok = traj.count >= 2 and bad == 0
        if not ok:
            failures.append(f"路线不通: {name} ({why}) 点={traj.count} 越界={bad}")
        print(f"    {'OK ' if ok else 'BAD'} {name:26s} 点={traj.count:3d} "
              f"长度={traj.meta.length_mm:6.0f} 越界={bad}")

    # 2) 登高结构必须不可通行
    print("\n[2] 坡道/阶梯/L1 必须不可通行")
    for name, x, y in VERIFY_BLOCKED:
        free = cm.is_free_world(x, y)
        ok = not free
        if not ok:
            failures.append(f"不该能走却标成可走: {name} ({x:.0f},{y:.0f})")
        print(f"    {'OK ' if ok else 'BAD'} {name:14s} ({x:7.0f},{y:7.0f}) "
              f"free={free} clr={cm.clearance_mm(x, y):6.0f}")

    # 3) 传递区必须可停留
    print("\n[3] 传递区必须可停留")
    for name, x, y in VERIFY_FREE:
        free = cm.is_free_world(x, y)
        ok = free
        if not ok:
            failures.append(f"传递区不可达: {name} ({x:.0f},{y:.0f})")
        print(f"    {'OK ' if ok else 'BAD'} {name:14s} ({x:7.0f},{y:7.0f}) "
              f"free={free} clr={cm.clearance_mm(x, y):6.0f}")

    print()
    if failures:
        print("=== 场地校验 FAIL ===")
        for f in failures:
            print("  -", f)
        return 1
    print("=== 场地校验 PASS "
          f"({len(VERIFY_ROUTES)} 条路线 + {len(VERIFY_BLOCKED)} 个障碍点 "
          f"+ {len(VERIFY_FREE)} 个可停留点) ===")
    return 0


def main() -> int:
    out_dir = os.path.join(HERE, "out")
    os.makedirs(out_dir, exist_ok=True)

    args = [a for a in sys.argv[1:]]
    if "--verify" in args:
        args.remove("--verify")
        return verify_only(args[0] if args else None)

    if args and os.path.isfile(args[0]):
        scene = Scene.load(args[0])
        print(f"已加载场景: {args[0]}")
    else:
        scene = build_default_scene()
        print("使用内置默认场地(女娲补天 11000x11000, 按规则V1重建)")

    cfg = PipelineConfig(
        route_name="TR_test_route",
        resolution_mm=25.0,
        # Keep old regression settings; source scene uses its own configured
        # circular envelope and margin below, never lower them for PASS.
        robot_radius_mm=300.0,
        safety_margin_mm=0.0,
        point_gap_mm=80.0,
        max_points=512,
        v_max_mps=1.2,
        a_max_mps2=1.2,
        j_max_mps3=25.0,
        a_lat_max_mps2=1.0,
        v_min_mps=0.06,
        min_radius_mm=700.0,
        keep_searched=True,
    )
    if surface_required(scene):
        cfg.robot_radius_mm=scene.robot_radius
        cfg.safety_margin_mm=scene.safety_margin
    start, goal = pick_points(scene)
    print(f"起点 {start.as_tuple()}")
    print(f"终点 {goal.as_tuple()}")

    t0 = time.perf_counter()
    cm = build_map(scene, cfg)
    print(f"[地图] {cm.spec.width}x{cm.spec.height} 格 @ {cfg.resolution_mm}mm, "
          f"可行 {cm.free_cell_count()} 格, 阻塞半径 {cm.blocked_radius:.0f}mm, "
          f"耗时 {time.perf_counter() - t0:.2f}s")

    logs = []
    traj = plan_route(scene, start, goal, cfg, costmap=cm, progress=logs.append)
    for line in logs:
        print("  -", line)
    for msg in traj.messages:
        print("  !", msg)

    if traj.count < 2:
        print("!! 生成失败")
        return 1

    # ---------------- 校验 ----------------
    rows = rows_for_export(traj)
    ok = True

    if abs(rows[0][0]) > 1e-6 or abs(rows[0][1]) > 1e-6:
        print("!! 首点速度不为 0"); ok = False
    if abs(rows[-1][0]) > 1e-6 or abs(rows[-1][1]) > 1e-6:
        print("!! 末点速度不为 0"); ok = False

    speeds = [math.hypot(r[0], r[1]) for r in rows]
    vmax = cfg.v_max_mps * 1000.0
    if max(speeds) > vmax + 1e-3:
        print(f"!! 超速 {max(speeds):.1f} > {vmax}"); ok = False

    # 加速度检查(按弧长算 a = (v2^2-v1^2)/2ds)
    worst_a = 0.0
    for i in range(1, len(rows)):
        ds = math.hypot(rows[i][2] - rows[i - 1][2], rows[i][3] - rows[i - 1][3]) * 0.001
        if ds < 1e-9:
            continue
        a = abs(speeds[i] ** 2 - speeds[i - 1] ** 2) * 1e-6 / (2.0 * ds)
        worst_a = max(worst_a, a)
    print(f"[速度] 峰值 {max(speeds):.0f} mm/s, 最大纵向加速度 {worst_a:.2f} m/s^2")

    # 横向加速度
    worst_alat = 0.0
    for i, r in enumerate(rows):
        k = abs(float(traj.curvature[i])) if i < len(traj.curvature) else 0.0
        worst_alat = max(worst_alat, (speeds[i] * 0.001) ** 2 * k)
    print(f"[速度] 最大横向加速度 {worst_alat:.2f} m/s^2 (上限 {cfg.a_lat_max_mps2})")

    # 碰撞检查: 每个点都要在可行区内
    bad_pts = 0
    min_clear = float("inf")
    for r in rows:
        if not cm.is_free_world(r[2], r[3]):
            bad_pts += 1
        min_clear = min(min_clear, cm.clearance_mm(r[2], r[3]))
    print(f"[避障] 越界点 {bad_pts} 个, 最小离障 {min_clear:.0f}mm "
          f"(要求 {cm.blocked_radius:.0f}mm)")
    if bad_pts:
        ok = False

    # 点间距
    gaps = [math.hypot(rows[i][2] - rows[i - 1][2], rows[i][3] - rows[i - 1][3])
            for i in range(1, len(rows))]
    print(f"[点表] {len(rows)} 点, 间距 min {min(gaps):.1f} / max {max(gaps):.1f} / "
          f"mean {sum(gaps) / len(gaps):.1f} mm")
    print(f"[点表] 长度 {traj.meta.length_mm:.1f} mm, 耗时 {traj.meta.total_time_s:.2f} s, "
          f"最小转弯半径 {traj.meta.min_radius_mm:.0f} mm")
    print(f"[点表] A* 展开 {traj.meta.astar_expanded} 格, A* 长度 "
          f"{traj.meta.astar_length_mm:.1f}mm, 简化后 {traj.meta.los_points} 关键点, "
          f"平滑 {traj.meta.spline_points} 点")

    # ---------------- 导出 ----------------
    csv_path = os.path.join(out_dir, "selftest_route.csv")
    atomic_text(csv_path,trajectory_to_csv(traj),"utf-8-sig")
    c_path = os.path.join(out_dir, "selftest_route.c")
    atomic_text(c_path,trajectory_to_c_code(traj,CCodeConfig(array_name="TR_test_route")))
    cpp_path = os.path.join(out_dir, "selftest_route_pathpoint.cpp")
    atomic_text(cpp_path,trajectory_to_c_code(traj,CCodeConfig(array_name="TR_test_route",style="path_point")))
    print(f"[导出] {csv_path}")
    print(f"[导出] {c_path}")
    print(f"[导出] {cpp_path}")

    # ---------------- 画图 ----------------
    try:
        _draw(cm, traj, os.path.join(out_dir, "selftest_map.png"))
        _draw_curves(traj, os.path.join(out_dir, "selftest_curves.png"))
        print(f"[绘图] {os.path.join(out_dir, 'selftest_map.png')}")
        print(f"[绘图] {os.path.join(out_dir, 'selftest_curves.png')}")
    except Exception as exc:  # pragma: no cover
        print(f"[绘图] 跳过: {exc}")

    print("\n=== 自检结果:", "PASS" if ok else "FAIL", "===")
    return 0 if ok else 1


def _draw(cm, traj, path: str) -> None:
    from PIL import Image, ImageDraw

    spec = cm.spec
    base = np.zeros((spec.height, spec.width, 3), dtype=np.uint8)
    base[...] = (245, 245, 245)
    base[cm.inflated] = (255, 190, 190)
    base[cm.raw_blocked] = (90, 90, 90)

    img = Image.fromarray(base, mode="RGB")
    draw = ImageDraw.Draw(img)
    k = 1.0 / cm.resolution

    def to_px(x, y):
        return ((x - spec.x_min) * k, (y - spec.y_min) * k)

    # A* 搜索痕迹
    if traj.searched_cells:
        for row, col in traj.searched_cells[:: max(1, len(traj.searched_cells) // 20000)]:
            x = spec.x_min + (col + 0.5) * cm.resolution
            y = spec.y_min + (row + 0.5) * cm.resolution
            px, py = to_px(x, y)
            draw.point((px, py), fill=(180, 210, 240))

    # LOS 关键点
    if len(traj.los_points) > 1:
        draw.line([to_px(*p) for p in traj.los_points], fill=(255, 150, 0), width=1)
    # 平滑曲线
    if len(traj.smooth_points) > 1:
        draw.line([to_px(*p) for p in traj.smooth_points], fill=(0, 120, 200), width=2)
    # 最终点表点
    for i, p in enumerate(traj.points):
        if i % 4 == 0:
            px, py = to_px(p.x, p.y)
            draw.ellipse([px - 1, py - 1, px + 1, py + 1], fill=(200, 0, 0))

    sx, sy = to_px(traj.meta.start_pose.x, traj.meta.start_pose.y)
    gx, gy = to_px(traj.meta.goal_pose.x, traj.meta.goal_pose.y)
    draw.ellipse([sx - 5, sy - 5, sx + 5, sy + 5], outline=(0, 160, 0), width=3)
    draw.ellipse([gx - 5, gy - 5, gx + 5, gy + 5], outline=(190, 0, 0), width=3)

    atomic_png(path,img)


def _draw_curves(traj, path: str) -> None:
    from PIL import Image, ImageDraw

    W, H = 900, 320
    img = Image.new("RGB", (W, H), (255, 255, 255))
    draw = ImageDraw.Draw(img)

    s = np.asarray(traj.speed_s_mm)
    v = np.asarray(traj.speed_v_mm_s)
    if len(s) < 2:
        atomic_png(path,img)
        return
    k = np.asarray(traj.curvature)
    margin = 40

    def plot(vals, color, y0, h, label):
        if len(vals) == 0:
            return
        vmax = max(1e-9, float(np.max(np.abs(vals))))
        pts = []
        for i in range(len(vals)):
            px = margin + (W - 2 * margin) * (s[i] / s[-1])
            py = y0 + h - h * (vals[i] / vmax)
            pts.append((px, py))
        draw.line(pts, fill=color, width=2)
        draw.text((margin, y0 - 14), f"{label}  max={vmax:.3g}", fill=color)

    plot(v, (200, 0, 0), 40, 110, "v [mm/s]")
    plot(k, (0, 110, 200), 190, 90, "kappa [1/mm]")
    atomic_png(path,img)


if __name__ == "__main__":
    raise SystemExit(main())
