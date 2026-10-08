"""按仿真几何重建的场地模板 —— 专用验收协议.

`selftest.py --verify` 里的路线/点位是给"旧模板"写的, 换模板后点位会变。
本脚本是**新模板专属**的验收: 点位取自仿真实测几何换算, 改模板后必须跑这个。

检查项
======
A. 关键点位通行性
     - 地面跑道(红/蓝) 必须可走
     - L1 台体 / 坡道 / 阶梯 必须**不可走**(规则 2.16 + 3.2.1)
     - 传递区为高平台，地面模式必须**不可进入**
     - 两个基座 / 公共区 / 储存区 / 启动区 必须可走
B. 典型地面路线连通性(全部线段圆包络扫掠；不进入登高区域)
C. 通道口可达性: TR 必须能从地面跑道走到坡道的"地面端"(爬升入口)

用法
====
    python verify_from_sim.py                                    # 用默认新模板
    python verify_from_sim.py scenes/xxx.json
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from robocon_path.core.geometry import Pose                      # noqa: E402
from robocon_path.core.scene import Scene                         # noqa: E402
from robocon_path.pipeline import PipelineConfig, build_map, plan_route  # noqa: E402
from robocon_path.plan.astar import astar_path                    # noqa: E402

DEFAULT_SCENE = os.path.join(HERE, "scenes", "nvwa_butian_2027_from_sim.json")

# --------------------------------------------------------------------------- #
# 由 build_template_from_sim.py 的常量推出的关键点位(planner 帧, mm)
# --------------------------------------------------------------------------- #
CORRIDOR_X_RED = (-3500.0, -3000.0)      # (中心, 参考)
# OBJ009/014低端对应planner y=2700；入口中心需在坡道之外，留305mm车体净空。
RAMP_FOOT = (-3500.0, 3100.0)           # 源坡道低端y=2700，留305mm车体净空
TRANSFER_RED = (-3500.0, -1300.0)       # OBJ013顶面平台
TRANSFER_BLUE = (3500.0, -1300.0)       # OBJ018，严格x镜像、y相同
RAMP_RED = (-3500.0, 950.0)             # OBJ009真实中心
STAIR_RED = (-3500.0, -2250.0)          # 三级露出台阶中心
L1_CENTER = (0.0, 0.0)

# (名称, x, y, 期望 free)
PROBES = [
    ("地面跑道(红,西侧)",   -4900.0, 0.0,      True),
    ("地面跑道(红,南侧)",   -3000.0, -4900.0,  True),
    ("地面跑道(蓝,东侧)",    4900.0, 0.0,      True),
    ("红传递区中心(高平台)", TRANSFER_RED[0], TRANSFER_RED[1], False),
    ("蓝传递区中心(高平台)", TRANSFER_BLUE[0], TRANSFER_BLUE[1], False),
    ("红坡道中心",          RAMP_RED[0], RAMP_RED[1], False),
    ("红阶梯中心",          STAIR_RED[0], STAIR_RED[1], False),
    ("L1 台体中心",         0.0, 0.0,          False),
    ("红储存区",            -4500.0, 5000.0,   True),
    ("红启动区A",           -5150.0, -5150.0,  True),
    ("第一公共区",          -450.0, -4250.0,   True),
    ("第二公共区/五色石旁",  -450.0, 4150.0,   True),
]

# (名称, 起点, 终点)
ROUTES = [
    ("启动区A -> 阶梯前地面",    (-5150.0, -5150.0), (-3500.0,-3100.0)),
    ("阶梯前地面 -> 启动区A",    (-3500.0,-3100.0), (-5150.0, -5150.0)),
    ("启动区A -> 第一公共区",    (-5150.0, -5150.0), (-450.0, -4250.0)),
    ("启动区A -> 红储存区",      (-5150.0, -5150.0), (-4500.0, 5000.0)),
    ("图示北侧 -> 第一公共区旁", (-4200.0,4300.0), (-1200.0,-4100.0)),
    ("启动区A -> 五色石基座旁",  (-5150.0, -5150.0), (-450.0, 4150.0)),
    ("环绕红半场",               (-5150.0, -5150.0), (-5100.0, 4400.0)),
    ("启动区A -> 坡道地面端",    (-5150.0, -5150.0), RAMP_FOOT),
]


def main(path: str | None = None) -> int:
    path = path if path is not None else (sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SCENE)
    if not os.path.isfile(path):
        print(f"找不到场景: {path}")
        return 1
    scene = Scene.load(path)
    cfg = PipelineConfig(robot_radius_mm=scene.robot_radius,
                         safety_margin_mm=scene.safety_margin)
    cm = build_map(scene, cfg)
    print(f"场景: {os.path.basename(path)}")
    print(f"栅格 {cm.spec.width}x{cm.spec.height} @ {cfg.resolution_mm:.0f}mm, "
          f"可行 {cm.free_cell_count()} 格, 阻塞半径 {cm.blocked_radius:.0f}mm")
    fails: list[str] = []

    print("\n[A] 关键点位通行性")
    print("    当前仅地面：坡道/阶梯/传递高平台全部不可进入；enabled保持原值。")
    for name, x, y, want in PROBES:
        # Keep climb structures blocked in the ground projection. Their support
        # layer is accessible only through the verified end entrance.
        clearance=cm.clearance_mm(x,y)
        got=cm.is_free_world(x,y)
        ok = got == want
        if not ok:
            fails.append(f"{name} ({x:.0f},{y:.0f}) free={got} 期望 {want}")
        print(f"    {'OK ' if ok else 'BAD'} {name:20s} ({x:7.0f},{y:7.0f}) "
              f"free={got!s:5s} clr={clearance:6.0f}")

    print("\n[B] 典型路线连通性")
    for name, s, g in ROUTES:
        t = plan_route(scene, Pose(*s, 0.0), Pose(*g, 0.0), cfg, costmap=cm)
        bad = sum(1 for p in t.points if not cm.is_free_world(p.x, p.y))
        segments_ok=t.count>=2 and t.navigation_map is not None and t.navigation_map.polyline_is_free([(p.x,p.y) for p in t.points])
        ok = t.count >= 2 and bad == 0 and segments_ok
        ok=ok and all(state=="ground" and z==0 for state,z in t.support_samples)
        if not ok:
            fails.append(f"路线不通: {name} 点={t.count} 越界={bad}")
        print(f"    {'OK ' if ok else 'BAD'} {name:24s} 点={t.count:3d} "
              f"长度={t.meta.length_mm:6.0f} 越界={bad}")
        if t.navigation_map is not None:
            print("       "+" → ".join(dict.fromkeys(s for s,z in t.support_samples))+"；整段扫掠="+str(segments_ok))
        elif not ok: print("       "+"; ".join(t.messages))

    print("\n[C] 坡道前地面可达性(只到地面，不登高)")
    for label, foot in (("红坡道地面端", RAMP_FOOT),
                        ("蓝坡道地面端", (3500.0, 3100.0))):
        r = astar_path(cm, (-5150.0, -5150.0) if foot[0] < 0 else (5150.0, -5150.0), foot)
        ok = r.ok
        if not ok:
            fails.append(f"爬升入口不可达: {label} {foot}")
        print(f"    {'OK ' if ok else 'BAD'} {label:16s} -> {r.message or ('len=%d' % len(r.path))}")

    print()
    if fails:
        print("=== 场地校验 FAIL ===")
        for f in fails:
            print("  -", f)
        return 1
    print(f"=== 地面场地校验 PASS ({len(PROBES)} 点位 + {len(ROUTES)} 路线 + 2 入口前地面) ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
