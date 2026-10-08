"""场地可视化: 把当前场地场景渲染成一张俯视图 PNG.

用途: 核对场地建模对不对(尤其在你实测完、改过 JSON 之后)。
     白色 = TR 可通行, 粉色 = 膨胀后不可通行, 深灰 = 障碍本身,
     细线框 = 参考区域, 红框 = 塔顶/料堆(默认不算障碍)。

用法:
    python draw_field.py                    # 用内置默认场地
    python draw_field.py scenes/xxx.json    # 用指定场景
"""

from __future__ import annotations

import os
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from robocon_path.core.geometry import circle_polygon, rect_polygon   # noqa: E402
from robocon_path.core.scene import Scene, build_default_scene        # noqa: E402
from robocon_path.pipeline import PipelineConfig, build_map           # noqa: E402
from robocon_path.core.file_io import atomic_png


def render(scene: Scene, cfg: PipelineConfig, out_path: str, scale: int = 3) -> str:
    cm = build_map(scene, cfg)
    spec = cm.spec
    k = 1.0 / cm.resolution

    base = np.zeros((spec.height, spec.width, 3), dtype=np.uint8)
    base[...] = (38, 40, 44)
    base[cm.cost] = (236, 238, 240)
    base[cm.inflated] = (255, 208, 208)
    base[cm.raw_blocked & ~cm.cost] = (72, 72, 72)
    if hasattr(cm,"support"):
        base[cm.raw_blocked & cm.cost] = (187,225,199)
    img = Image.fromarray(base, "RGB")
    draw = ImageDraw.Draw(img)

    def px(x, y):
        return ((x - spec.x_min) * k, (y - spec.y_min) * k)

    # 参考区域: 细线框
    for z in scene.zones:
        if not isinstance(z, dict) or z.get("type") != "rect":
            continue
        if z.get("enabled", True) is False:
            continue
        pts = [px(x, y) for x, y in rect_polygon(z["cx"], z["cy"], z["w"], z["h"])]
        draw.polygon(pts, outline=z.get("color") or "#888888")

    # 动态障碍(默认不算): 红框标出位置
    for o in scene.obstacles:
        if not isinstance(o, dict):
            continue
        if str(o.get("tag", "")) not in ("tower_top", "stack"):
            continue
        if o.get("type") == "rect":
            draw.rectangle([px(o["cx"] - o["w"] / 2, o["cy"] - o["h"] / 2),
                            px(o["cx"] + o["w"] / 2, o["cy"] + o["h"] / 2)],
                           outline=(200, 60, 60))

    if scale > 1:
        img = img.resize((spec.width * scale, spec.height * scale), Image.NEAREST)
    # Costmap rows grow with world y; image rows grow downward.
    # Match the interactive canvas: north (+y) must be at the top.
    img = img.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
    atomic_png(out_path,img)
    return out_path


def main() -> int:
    out_dir = os.path.join(HERE, "out")
    os.makedirs(out_dir, exist_ok=True)

    if len(sys.argv) > 1 and os.path.isfile(sys.argv[1]):
        scene = Scene.load(sys.argv[1])
        print(f"已加载: {sys.argv[1]}")
    else:
        scene = build_default_scene()
        print("使用内置默认场地")

    cfg = PipelineConfig(resolution_mm=scene.resolution,
                         robot_radius_mm=scene.robot_radius,
                         safety_margin_mm=scene.safety_margin)
    path = os.path.join(out_dir, "field_topview.png")
    render(scene, cfg, path)
    print(f"已出图: {path}")
    print(f"  分辨率 {cfg.resolution_mm:.0f}mm/格, 车体半径 {cfg.robot_radius_mm:.0f}mm, "
          f"安全余量 {cfg.safety_margin_mm:.0f}mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
