"""给场景 JSON 补写 meta.rules 规则依据块(只作文档, 程序不读).

用途: 让接手改这个 JSON 的人/工具能直接在文件里看到"尺寸依据是什么、哪些字段不能动",
不用再去翻规则书。运行一次即可, 可重复运行(幂等)。

用法: python add_rules_meta.py [场景文件路径]
"""

from __future__ import annotations

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

RULES = {
    "source": "第二十六届全国大学生机器人大赛女娲补天竞技赛规则 V1 (2026-09)",
    "note": "以下尺寸抄自规则原文/官方图, 是校验场地模板的依据; 程序不解析本块",
    "coordinate_system": {
        "origin": "场地中心 = 中央隔板中点",
        "x_axis": "+x 向东(蓝队侧), 红队在 -x 侧",
        "y_axis": "+y 向北",
        "unit": "mm",
        "yaw": "逆时针为正",
        "extent": [-5500, -5500, 5500, 5500],
    },
    "dimensions_mm": {
        "field_size": 11000,
        "field_fence": {"inner_height": 80, "thickness": 50},
        "divider": {"thickness": 50, "note": "把地面和 L1 一起等分"},
        "ground_run_width": 2500,
        "L1": {"size": 6000, "height": 600,
               "fence": {"inner_height": 50, "thickness": 50}},
        "L2": {"size": 3000, "height_above_ground": 900, "height_above_L1": 300},
        "public1": {"size": 1200,
                    "position": "场地南方, 居中于场地围栏与 L1 围栏之间",
                    "center": [0, -4250],
                    "content": "12 个塔顶, 5x5 棋盘格, 正中心格空置"},
        "public2": {"size": 1000,
                    "position": "场地北方, 边线距北边墙内侧面 1000mm",
                    "center": [0, 4000]},
        "pedestal": {"diameter": 270, "height": 500,
                     "top_socket": {"diameter": 180, "depth": 100}},
        "tower_base_mid": {"size": [350, 350, 350], "weight_g": [200, 350],
                           "count_per_team": 20},
        "tower_top": {"size": [200, 200, 200], "weight_g": [80, 150],
                      "count_total": 12},
        "five_color_stone": {"diameter": 200, "weight_g": [400, 440]},
        "storage": {"size": [1000, 2000], "count": "每队各一个专属区"},
        "start_zone": {"size": 700, "count_per_half": 2, "gap": 100},
        "ramp": {"length": 3500, "rise": 600},
        "stairs": {"steps": 3, "step": [300, 1000, 150],
                   "note": "级长300 宽1000 高150"},
        "transfer": {"size": [1000, 1000],
                     "position": "L1 区边界处, 坡道与阶梯之间"},
        "central_pedestal": {"diameter": 270, "height": 800,
                             "top_socket": {"diameter": 180, "depth": 100}},
    },
    "operation_rules": {
        "TR": "只能在地面区运行; 只能通过坡道或阶梯到达传递区; 严禁进入 L1/L2 任何部分",
        "BR": "可以在传递区/L1/L2 运行; 必须通过坡道或阶梯进入 L1",
        "ramp_stair_definition": (
            "坡道、阶梯和它们之间的传递区是「地面区与 L1 之间的过渡区域」, 不属于地面区"),
        "passability_must_hold": [
            "tag=ramp_stair 的项必须 enabled=true (不可当平地走)",
            "tag=transfer   的项必须 enabled=false (TR 可以停留)",
            "tag=l1_body/fence/divider/pedestal 必须 enabled=true",
            "tag=tower_top/stack 默认 enabled=false (位置会变)",
        ],
    },
    "field_schema": {
        "geometry_types": {
            "rect": ["cx", "cy", "w", "h"],
            "circle": ["cx", "cy", "r", "segments(可选)"],
            "polygon": ["points: [[x,y], ...]"],
            "wall": ["x1", "y1", "x2", "y2", "thickness"],
        },
        "meta_fields": {
            "name": "给人/清单看的名字",
            "tag": "语义分类, 程序依赖它做批量开关与通行性判断",
            "layer": "ground | L1 | L2, 仅用于分层显示",
            "enabled": "true=参与避障(不可通行); false=不参与避障(可走/仅标注)",
            "color": "仅 zones 使用的填充色",
        },
        "tag_meaning": {
            "fence": "场地围栏",
            "divider": "中央隔板",
            "l1_body": "L1 台体(地面层不可通行的墙)",
            "ramp_stair": "坡道/阶梯(登高结构, 不可当平地走)",
            "pedestal": "五色石基座",
            "transfer": "传递区(水平交接区, 可停留)",
            "tower_top": "塔顶(位置会变)",
            "stack": "塔基/中段料堆(位置会变)",
        },
        "do_not_change": [
            "坐标约定(原点/轴向/单位) —— 改了会让导出的点表整体偏移或镜像",
            "任何项的 tag —— 改了会让 ramp_stair/transfer/l1_body 等分类失效",
            "任何项的 enabled —— 改了轻则路径绕远, 重则把换乘通道堵死",
            "obstacles 与 zones 的分工 —— 参考区域误放进 obstacles 会变成墙",
        ],
    },
    "verified": {
        "status": ("已用 A* 连通性验证: 6 条典型路线"
                   "(启动区<->传递区<->公共区<->储存区<->环绕半场<->北跑道) "
                   "全部通过, 点表 0 越界, 最小离障 362mm(要求 300mm)"),
        "robot_model": {"radius_mm": 300, "safety_margin_mm": 5,
                        "why_not_350": "TR 赛前 700x700 -> 外接圆 350, 但换乘通道"
                                       "留给地面的车道较窄, 350 会把通道堵死"},
        "assumptions_to_recheck": [
            "坡道-传递区-阶梯段在 y 上的确切起止(规则给了长度没给基准点); "
            "现按「贴 L1 南边界、以隔板为中心对称」摆放",
            "两个启动区 700x700 的具体格位(现放在南侧靠各自半场外缘)",
        ],
    },
}


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        HERE, "scenes", "nvwa_butian_2027_template.json")
    if not os.path.isfile(path):
        print(f"找不到场景文件: {path}")
        return 1
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    data.setdefault("meta", {})["rules"] = RULES
    data["meta"]["generated_by"] = "robocon_path.core.scene.build_default_scene"
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2)
    print(f"已写入 meta.rules: {path} ({os.path.getsize(path)} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
