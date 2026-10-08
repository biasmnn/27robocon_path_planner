"""场地几何实验台: 快速试算换乘通道的布置是否可行。

不启动界面, 直接构造候选场景, 检查:
  1. 通道"地面可达车道"是否连通到地面环(从启动区能不能搜到通道口)
  2. 车道宽度是否够车体通过(离障 >= 阻塞半径)
  3. 传递区是否落在 L1 缺口内、且面积/位置符合规则
  4. 坡道/阶梯是否确实不可作为平地通行

用法: python field_lab.py
"""

from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from robocon_path.core.geometry import Pose, rect_polygon          # noqa: E402
from robocon_path.core.scene import Scene, SceneMeta               # noqa: E402
from robocon_path.pipeline import PipelineConfig, build_map        # noqa: E402
from robocon_path.plan.astar import astar_path                     # noqa: E402

FIELD = 11000.0
HALF = FIELD / 2

# ---------------------------------------------------------------------------
# 候选布置参数(全部可调)
# ---------------------------------------------------------------------------
NOTCH_DEPTH = 2500.0          # L1 南边界上的缺口深度
CORRIDOR_HALF_W = 2600.0      # 换乘通道半宽(跨隔板)
LANE_W = 800.0                # 通道内"地面可达车道"宽度
RAMP_W = 500.0                # 单侧坡道宽度
STAIR_W = 500.0               # 单侧阶梯宽度
TRANSFER_D = 1200.0           # 传递区沿 y 的进深
RAMP_LEN = 3500.0
STAIR_LEN = 900.0
L1_SIZE = 6000.0
L1_HALF = L1_SIZE / 2
FENCE = 50.0

NOTCH_NORTH = -L1_HALF                 # -3000
NOTCH_SOUTH = NOTCH_NORTH - NOTCH_DEPTH
TRANSFER_Y = NOTCH_NORTH - TRANSFER_D * 0.5
RAMP_Y = NOTCH_SOUTH - RAMP_LEN * 0.5
STAIR_Y = NOTCH_SOUTH - RAMP_LEN - STAIR_LEN * 0.5

# 通道横向分区(红队半场; 蓝队镜像)
LANE_X = (-CORRIDOR_HALF_W, -CORRIDOR_HALF_W + LANE_W)        # 地面车道
RAMP_X = (LANE_X[1], LANE_X[1] + RAMP_W)                      # 坡道
STAIR_X = (RAMP_X[1], RAMP_X[1] + STAIR_W)                    # 阶梯


def rect(cx, cy, w, h, name, tag, layer="ground", enabled=True):
    return {"type": "rect", "cx": cx, "cy": cy, "w": w, "h": h,
            "name": name, "tag": tag, "layer": layer, "enabled": enabled}


def build_candidate() -> Scene:
    sc = Scene(meta=SceneMeta(name="lab"))
    sc.extent = (-5600.0, -5600.0, 5600.0, 5600.0)
    t = FENCE
    sc.obstacles += [
        rect(0, HALF - t / 2, FIELD, t, "围栏N", "fence"),
        rect(0, -(HALF - t / 2), FIELD, t, "围栏S", "fence"),
        rect(HALF - t / 2, 0, t, FIELD, "围栏E", "fence"),
        rect(-(HALF - t / 2), 0, t, FIELD, "围栏W", "fence"),
    ]
    # 隔板: 地面北跑道 + 通道以南的地面段
    sc.obstacles.append(rect(0, L1_HALF + 1250, 50, 2500, "隔板(北)", "divider"))
    y_div_s = NOTCH_SOUTH - (NOTCH_SOUTH - (-HALF)) * 0.5
    sc.obstacles.append(rect(0, y_div_s, 50, NOTCH_SOUTH + HALF, "隔板(南)", "divider"))

    # L1 台体: 中间挖缺口
    x0, x1 = -CORRIDOR_HALF_W, CORRIDOR_HALF_W
    y_gap = NOTCH_NORTH
    sc.obstacles += [
        rect(0, (y_gap + L1_HALF) / 2, L1_SIZE, L1_HALF - y_gap, "L1北块", "l1_body"),
        rect((x0 - L1_HALF) / 2, (NOTCH_SOUTH + y_gap) / 2, x0 + L1_HALF,
             NOTCH_DEPTH, "L1西南块", "l1_body"),
        rect((x1 + L1_HALF) / 2, (NOTCH_SOUTH + y_gap) / 2, L1_HALF - x1,
             NOTCH_DEPTH, "L1东南块", "l1_body"),
    ]

    # 坡道/阶梯(不可平地通行) + 传递区(可停留)
    for sign in (-1.0, 1.0):
        rx = sign * (LANE_X[1] + RAMP_W / 2) * -1 if sign < 0 else (RAMP_X[0] + RAMP_W / 2)
        # 红队: x 为负; 蓝队: 镜像到正
        if sign < 0:
            rcx = LANE_X[1] + RAMP_W / 2 - CORRIDOR_HALF_W * 0  # 负半场
            rcx = LANE_X[1] + RAMP_W / 2
        else:
            rcx = -(LANE_X[1] + RAMP_W / 2)
        sc.obstacles.append(rect(rcx, RAMP_Y, RAMP_W, RAMP_LEN,
                                 f"坡道{'红' if sign < 0 else '蓝'}", "ramp_stair", "L1"))
        sc.obstacles.append(rect(rcx, STAIR_Y, STAIR_W, STAIR_LEN,
                                 f"阶梯{'红' if sign < 0 else '蓝'}", "ramp_stair", "L1"))

    sc.obstacles.append(rect(0, TRANSFER_Y, 2 * CORRIDOR_HALF_W, TRANSFER_D,
                             "传递区", "transfer", "L1", enabled=False))
    return sc


def main() -> int:
    print("候选布置参数:")
    for k in ("NOTCH_DEPTH", "CORRIDOR_HALF_W", "LANE_W", "RAMP_W", "STAIR_W",
              "TRANSFER_D", "RAMP_LEN", "STAIR_LEN"):
        print(f"  {k:16s} = {globals()[k]}")
    print(f"  缺口 y ∈ [{NOTCH_SOUTH:.0f}, {NOTCH_NORTH:.0f}]")
    print(f"  传递区 y ∈ [{TRANSFER_Y - TRANSFER_D/2:.0f}, {TRANSFER_Y + TRANSFER_D/2:.0f}]")
    print(f"  坡道   y ∈ [{RAMP_Y - RAMP_LEN/2:.0f}, {RAMP_Y + RAMP_LEN/2:.0f}]")
    print(f"  阶梯   y ∈ [{STAIR_Y - STAIR_LEN/2:.0f}, {STAIR_Y + STAIR_LEN/2:.0f}]")
    print(f"  车道 x ∈ [{LANE_X[0]:.0f}, {LANE_X[1]:.0f}] / 镜像")
    print(f"  坡道 x ∈ [{RAMP_X[0]:.0f}, {RAMP_X[1]:.0f}] / 镜像")

    sc = build_candidate()
    cfg = PipelineConfig(robot_radius_mm=300.0, safety_margin_mm=0.0,
                         resolution_mm=25.0, point_gap_mm=100.0)
    cm = build_map(sc, cfg)
    print(f"\n栅格 {cm.spec.width}x{cm.spec.height}, 可行 {cm.free_cell_count()} 格, "
          f"阻塞半径 {cm.blocked_radius:.0f}")

    # 关键点通行性
    pts = [
        ("地面环(红, 西跑道)", -4250.0, -4500.0),
        ("地面环(红, 西跑道北)", -4250.0, 3000.0),
        ("通道口(地面, 南)", -2200.0, -5400.0),
        ("车道中段", -2200.0, -3500.0),
        ("车道北端(传递区口)", -2200.0, -2200.0),
        ("传递区红中心", -1300.0, TRANSFER_Y),
        ("坡道红中心", LANE_X[1] + RAMP_W / 2, RAMP_Y),
        ("启动区A", -5100.0, -4550.0),
    ]
    for name, x, y in pts:
        print(f"  {name:22s} ({x:7.0f},{y:7.0f}) free={cm.is_free_world(x, y)!s:5s} "
              f"clr={cm.clearance_mm(x, y):6.0f}")

    # A* 连通性
    tests = [
        ("启动区A -> 车道中段", (-5100, -4550), (-2200, -3500)),
        ("启动区A -> 传递区红", (-5100, -4550), (-1300, TRANSFER_Y)),
        ("传递区红 -> 第一公共区红侧", (-1300, TRANSFER_Y), (-450, -4250)),
        ("车道中段 -> 北跑道红侧", (-2200, -3500), (-4250, 4000)),
    ]
    ok_all = True
    print()
    for name, s, g in tests:
        r = astar_path(cm, s, g)
        ok = r.ok
        ok_all &= ok
        print(f"  {'OK ' if ok else 'BAD'} {name:26s} {r.message or 'len=%d' % len(r.path)}")
    print("\n=== 候选布置:", "可行" if ok_all else "不可行", "===")
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
