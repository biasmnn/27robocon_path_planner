"""场地场景: 描述"哪里有障碍、哪里有参考区域", 可存成 JSON.

坐标约定
--------
* 场地系单位 mm, 右手系, y 轴向上, yaw 逆时针为正.
* 原点放在**场地中心 = 中央隔板中点**, +x 向东(蓝队侧), +y 向北,
  场地范围 ``[-5500, +5500]^2``.
* TR 里程计原点如果不在这里, 导出点表时用 origin 平移即可.

================================ JSON 场景文件格式 ================================
顶层结构::

    {
      "version": 1,
      "meta": {
        "name": "<场景名>",
        "note": "<自由说明>",
        "origin_note": "<原点约定说明>",
        "rules": { ... }          # 可选: 规则依据, 只作文档, 程序不读
      },
      "field_size": 11000.0,      # 场地边长 mm
      "extent": [x_min, y_min, x_max, y_max],   # 栅格覆盖范围 mm
      "resolution": 25.0,         # 栅格分辨率 mm/格 (UI 可覆盖)
      "robot_radius": 300.0,      # 车体外接半径 mm (UI 可覆盖)
      "safety_margin": 5.0,       # 额外安全余量 mm (UI 可覆盖)
      "obstacles": [ <几何项> ],  # 不可通行/受限的东西
      "zones":     [ <几何项> ],  # 参考区域, 只做可视化与取点参考, 不参与避障
      "background": { ... }       # 底图标定参数, 程序写入
    }

几何项 ``<几何项>`` 支持 4 种 ``type``::

    {"type": "rect",    "cx": .., "cy": .., "w": .., "h": ..}
    {"type": "circle",  "cx": .., "cy": .., "r": .., "segments": 48}
    {"type": "polygon", "points": [[x, y], [x, y], ...]}
    {"type": "wall",    "x1": .., "y1": .., "x2": .., "y2": .., "thickness": 50}

每个几何项都可带这些元字段(程序会读)::

    "name"    : 给人和 Watch/清单看的名字
    "tag"     : **语义分类, 程序依赖它做批量开关**. 现有关键 tag:
                fence / divider / l1_body / ramp_stair / pedestal / transfer /
                tower_top / stack
                —— 见 ``CLIMB_TAGS`` / ``FIXED_TAGS`` / ``DYNAMIC_TAGS``.
    "layer"   : "ground" | "L1" | "L2", 仅用于分层显示
    "enabled" : true  = 参与避障(**不可通行**)
                false = 不参与避障(可以走/只是标注)
                ⚠ 这是通行性开关, 不是"显示开关". 坡道/阶梯必须 true,
                  传递区必须 false, 否则 TR 要么撞上去、要么到不了传递区.
    "color"   : 区域填充色, 仅 zones 用

仿真生成地图的障碍项还带 ``navigation`` (不改变几何、tag/enabled)::

    {"kind": "ramp" | "stairs" | "transfer", "side": "red" | "blue",
     "entry_axis": "y", "top_height_mm": 600.0,
     "ground_end_y_mm": ... , "rule": "..."}

``ground_end_y_mm`` 仅坡道/阶梯需要，且必须等于真实地面端边界。
enabled=true仍禁止地面横穿爬升结构；SurfaceMap仅开放合法支承通路，
由其端口进入/退出，且整个机器人包络不得跨侧边或L1边界。
transfer=false表示可在其顶面停留，并非可以从地面侧向进入。
layer字段仍仅显示；navigation承担高度/连接语义。旧模板不具有此物理保证。

⚠ 改 JSON / 让别的工具改 JSON 时, 以下字段**绝对不能动**, 否则会静默出错:
  * 坐标约定(原点/轴向/单位) —— 动了点表就整体偏移或镜像;
  * 每个几何项的 ``tag`` —— 动了会让 ramp_stair / transfer / l1_body 的分类失效;
  * 每个几何项的 ``enabled`` —— 动了轻则路径绕远, 重则把通道堵死;
  * ``obstacles`` 与 ``zones`` 的分工 —— 参考区域误放进 obstacles 会变成墙.
==============================================================================
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from .geometry import Circle, Point, Polygon, Pose, Wall, rect_polygon
from .file_io import atomic_text

FIELD_SIZE = 11000.0          # 场地边长 mm
HALF = FIELD_SIZE * 0.5       # 5500
FENCE_THICKNESS = 50.0        # 场地围栏厚 mm
FENCE_INNER_H = 80.0          # 场地围栏内高 mm(示意, 平面图用不到)
L1_FENCE_THICKNESS = 50.0     # L1 围栏厚 mm(内高 50)
DIVIDER_THICKNESS = 50.0      # 中央隔板厚 mm(把"地面 + L1"一起等分)

# ---- 三层平台(规则 V1 1.1 / 2.15 / 2.17, 均已确认) ----
L1_SIZE = 6000.0              # L1 区 6000 x 6000, 居中 -> 地面四周各留 2500mm 宽
L2_SIZE = 3000.0              # L2 区 3000 x 3000, 居中
L1_HEIGHT = 600.0             # 高出地面 600
L2_HEIGHT = 900.0             # 高出地面 900

# ---- 地面公共区(规则 V1 2.7) ----
PUBLIC1_SIZE = 1200.0         # 第一公共区 1200x1200, 在场地南方, 居中于场地围栏与 L1 围栏之间
PUBLIC2_SIZE = 1000.0         # 第二公共区 1000x1000, 在场地北方, 边线距北边墙内侧面 1000mm
PEDESTAL_D = 270.0            # 五色石基座 直径 270, 高 500
TOWER_TOP_SIZE = 200.0        # 塔顶 200x200x200
TOWER_TOP_COUNT = 12          # 第一公共区里 12 个塔顶, 摆在 5x5 棋盘格里, 正中心空置

# ---- 储存区 / 启动区(规则 V1 2.3 / 2.6) ----
STORAGE_W = 1000.0
STORAGE_H = 2000.0
START_SIZE = 700.0
START_GAP = 100.0             # 两个启动区之间的间隙

# ---------------------------------------------------------------------------
# 下面这些"区域中心"是从规则 V1 与官方图里推出来的, 现场实测后可直接改;
# 也可以在界面里拖动/改 scenes/*.json。
# 坐标系: 原点在场地中心(=中央隔板中点), +x 向东(蓝队侧), +y 向北, 单位 mm。
# ---------------------------------------------------------------------------
L1_HALF = L1_SIZE * 0.5                       # 3000
GROUND_RUN = HALF - L1_HALF                   # 2500: L1 围栏到场地围栏之间的跑道宽
PUBLIC1_CENTER = (0.0, -(L1_HALF + GROUND_RUN * 0.5))   # (0, -4250) 第一公共区(南方)
PUBLIC2_CENTER = (0.0, HALF - 1000.0 - PUBLIC2_SIZE * 0.5)  # (0, +4000) 第二公共区(北方)
PEDESTAL_CENTER = PUBLIC2_CENTER              # 五色石基座立在第二公共区中央

# ---- 坡道 / 阶梯 / 传递区(规则 V1 2.16 / 2.18) ----
# ⚠ 关键(2.16 + 3.2.1 + 3.4.5 要一起读):
#   2.16: "坡道、阶梯**和它们之间的传递区**是地面区与 L1 区之间的**过渡区域**"
#         -> 坡道和阶梯**不属于地面区**。
#   3.2.1: "TR 只能在地面区运行, **只能通过坡道或阶梯**到达传递区"
#         -> TR 不是"走过"坡道/阶梯, 而是**爬上去**。
#   3.4.5: "**在坡道或阶梯上的 TR** 可以直接将比赛用品交给 BR"
#         -> 爬到一半也算, 进一步说明坡道/阶梯是登高通道。
#   结论: **A* 绝不能把坡道/阶梯当成普通地面去寻路**, 但它们又是 TR 到传递区
#         唯一的通路。pipeline 里对"终点在传递区"做了两级寻路处理。
TRANSFER_SIZE = 1000.0        # 传递区 1000x1000(水平区域)
TRANSFER_HALF_W = 1000.0      # 单侧传递区沿隔板方向的半宽
RAMP_LEN = 3500.0             # 坡道长 3500(沿爬升方向)
RAMP_W = 1000.0               # 坡道宽 1000
STAIR_STEP_LEN = 300.0        # 三级阶梯每级长 300
STAIR_STEP_W = 1000.0         # 每级宽 1000
STAIR_STEPS = 3
STAIR_LEN = STAIR_STEP_LEN * STAIR_STEPS      # 阶梯总长 900
RAMP_RISE = 600.0             # 爬升高度 = L1 高度(坡度约 9.7 度)

# 换乘通道: L1 南边界上的一处缺口, 从缺口口往北掏进 L1, **以中央隔板为中心左右对称**。
# 通道横向分三条带(每侧): 靠隔板的地面车道 | 坡道 | 阶梯 —— 三条带尺寸相同。
#
# ⚠ 通行建模(规则 2.16 + 3.2.1 + 3.4.5 一起读):
#   坡道/阶梯**不是平地** —— TR 不能"走过"它们, 但**只能通过它们**到达传递区。
#   所以: 坡道/阶梯 tag="ramp_stair" 且 **enabled=True(在地面图里不可走)**;
#         传递区  tag="transfer"   且 enabled=False(TR 可以停留/行驶, 是水平交接区);
#   这样 A* 不会把坡道/阶梯当平地寻路, 而传递区仍可到达 —— 与规则一致。
#
# 这组数值是跑实验台验证过的: 车道宽 600, 300mm 半径的车能过。
NOTCH_HALF_W = 2000.0                     # L1 缺口半宽(以隔板为中心, = 传递区半宽)
NOTCH_Y0, NOTCH_Y1 = -3000.0, -1000.0     # 缺口 y 范围(南端 = L1 南边界)
CORRIDOR_HALF_W = NOTCH_HALF_W
CORRIDOR_Y_SOUTH, CORRIDOR_Y_NORTH = NOTCH_Y0, NOTCH_Y1

LANE_W = 600.0                            # 中央地面车道半宽(靠隔板)
RAMP_W = 800.0                            # 坡道宽(单侧)
STAIR_W = 800.0                           # 阶梯宽(单侧)
LANE_CX = LANE_W * 0.5                    # 车道中心 |x| = 300
RAMP_CX = LANE_W + RAMP_W * 0.5           # 坡道中心 |x| = 1000
STAIR_CX = LANE_W + RAMP_W + STAIR_W * 0.5  # 阶梯中心 |x| = 1800

RAMP_LEN = 1000.0                         # 坡道沿 x 的长度(在缺口里横向爬升)
RAMP_Y = 0.0                              # 坡道中心 y
RAMP_X0, RAMP_X1 = LANE_W, LANE_W + RAMP_W
STAIR_STEP_LEN = 300.0                    # 三级阶梯每级长 300
STAIR_STEPS = 3
STAIR_LEN = STAIR_STEP_LEN * STAIR_STEPS  # 阶梯总长 900
STAIR_Y = -2200.0                         # 阶梯中心 y
TRANSFER_SIZE = 1000.0                    # 传递区 1000x1000(一队一半)
TRANSFER_Y = -2100.0                      # 传递区中心 y
RAMP_RISE = L1_HEIGHT                     # 爬升高度 = L1 高度 600(坡度约 9.7 度)

# ---- 储存区 / 启动区位置(规则 V1 2.3 / 2.6) ----
# 储存区: 贴北墙, 在各自半场靠外侧(官方俯视图: 红队储存区左上, 蓝队右上)
STORAGE_CENTER = (HALF - FENCE_THICKNESS - STORAGE_W * 0.5,
                  HALF - FENCE_THICKNESS - STORAGE_H * 0.5)

# 启动/重试区: 官方俯视图里在场地南侧靠各自半场外侧角落, 两个 700x700 并排
START_Y = -(HALF - FENCE_THICKNESS - 900.0)
START_X_OUTER = -(HALF - FENCE_THICKNESS - START_SIZE * 0.5)
START_X_INNER = START_X_OUTER + START_SIZE + START_GAP

LAYER_GROUND = "ground"
LAYER_L1 = "L1"
LAYER_L2 = "L2"

# ---------------------------------------------------------------------------
# tag 分组: 程序依赖这些分类做批量开关与通行性判断, 改名字必须同步改这里
# ---------------------------------------------------------------------------
# 永久固定障碍: 一直都在, 参与避障
FIXED_TAGS = ("fence", "divider", "l1_body", "pedestal")
# 登高结构: **规则上不是平地** —— 必须 enabled=True(不可通行), 见文件头说明
CLIMB_TAGS = ("ramp_stair",)
# 水平交接区: TR 可以停留, 必须 enabled=False
TRANSFER_TAGS = ("transfer",)
# 会变的东西: 位置随比赛进程变化, 默认不建议参与避障
DYNAMIC_TAGS = ("tower_top", "stack")

ALL_TAGS = FIXED_TAGS + CLIMB_TAGS + TRANSFER_TAGS + DYNAMIC_TAGS


@dataclass
class SceneMeta:
    """跟几何无关的元信息, 导出时一起写进注释."""

    name: str = "robocon2027_field"
    note: str = ""
    origin_note: str = "origin = 中央隔板中点"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class Scene:
    """一个可保存/加载的场地场景."""

    meta: SceneMeta = field(default_factory=SceneMeta)
    field_size: float = FIELD_SIZE
    # 栅格覆盖范围(场地系 mm): x_min, y_min, x_max, y_max
    extent: Tuple[float, float, float, float] = (-FIELD_SIZE / 2, -FIELD_SIZE / 2,
                                                 FIELD_SIZE / 2, FIELD_SIZE / 2)
    resolution: float = 25.0          # 栅格分辨率 mm/格
    robot_radius: float = 400.0       # TR 外接圆半径 mm(赛前 700^3 -> 外接圆约 350, 留余量)
    safety_margin: float = 60.0       # 额外安全余量 mm
    obstacles: List[Dict[str, Any]] = field(default_factory=list)
    zones: List[Dict[str, Any]] = field(default_factory=list)
    # 背景图(可选): 相对/绝对路径, 以及标定参数
    background: Dict[str, Any] = field(default_factory=dict)

    # ------------------------------------------------------------------ #
    # 几何访问
    # ------------------------------------------------------------------ #
    def obstacle_polygons(self, tag_filter: Optional[Iterable[str]] = None) -> List[Polygon]:
        """把所有障碍统一成多边形列表(圆/线段在这里被离散化).

        tag_filter 给定时, 只保留 tag 在集合里的障碍; enabled=False 的会被跳过。
        """
        allow = set(tag_filter) if tag_filter is not None else None
        out: List[Polygon] = []
        for item in self.obstacles:
            if not isinstance(item, dict):
                continue
            if item.get("enabled", True) is False:
                continue
            if allow is not None and str(item.get("tag", "")) not in allow:
                continue
            poly = _item_to_polygon(item)
            if poly is not None and poly.is_valid():
                out.append(poly)
        return out

    def zone_polygons(self) -> List[Polygon]:
        out: List[Polygon] = []
        for item in self.zones:
            if isinstance(item, dict) and item.get("enabled", True) is False:
                continue
            poly = _item_to_polygon(item)
            if poly is not None and poly.is_valid():
                out.append(poly)
        return out

    def set_obstacle_enabled(self, tag: str, enabled: bool) -> int:
        """按 tag 批量开关障碍, 返回改动条数(UI 上做"这类障碍算不算"很方便)."""
        count = 0
        for item in self.obstacles:
            if isinstance(item, dict) and str(item.get("tag", "")) == tag:
                item["enabled"] = bool(enabled)
                count += 1
        return count

    def set_group_enabled(self, group: str, enabled: bool) -> int:
        """按语义分组(group)批量开关障碍, 返回改动条数.

        group 取值为 fixed / climb / transfer / dynamic / reference 之一
        (见 AGENT.md 第 4 节); 没有 group 字段的项按 tag 推断, 保证老 JSON 也能用。
        """
        count = 0
        for item in self.obstacles:
            if not isinstance(item, dict):
                continue
            g = str(item.get("group", "")) or self._infer_group(str(item.get("tag", "")))
            if g == group:
                item["enabled"] = bool(enabled)
                count += 1
        return count

    @staticmethod
    def _infer_group(tag: str) -> str:
        if tag in FIXED_TAGS:
            return "fixed"
        if tag in CLIMB_TAGS:
            return "climb"
        if tag in TRANSFER_TAGS:
            return "transfer"
        if tag in DYNAMIC_TAGS:
            return "dynamic"
        return "reference"

    def obstacle_tags(self) -> List[str]:
        tags = []
        for item in self.obstacles:
            if isinstance(item, dict):
                t = str(item.get("tag", ""))
                if t and t not in tags:
                    tags.append(t)
        return tags

    def blocked_radius(self) -> float:
        return float(self.robot_radius + self.safety_margin)

    # ------------------------------------------------------------------ #
    # 增删
    # ------------------------------------------------------------------ #
    def add_obstacle_rect(self, cx: float, cy: float, w: float, h: float,
                          name: str = "rect", layer: str = LAYER_GROUND) -> Dict[str, Any]:
        item = {"type": "rect", "cx": cx, "cy": cy, "w": w, "h": h,
                "name": name, "layer": layer}
        self.obstacles.append(item)
        return item

    def add_obstacle_circle(self, cx: float, cy: float, r: float,
                            name: str = "circle", layer: str = LAYER_GROUND) -> Dict[str, Any]:
        item = {"type": "circle", "cx": cx, "cy": cy, "r": r,
                "name": name, "layer": layer}
        self.obstacles.append(item)
        return item

    def add_obstacle_polygon(self, points: Sequence[Point],
                             name: str = "poly", layer: str = LAYER_GROUND) -> Dict[str, Any]:
        item = {"type": "polygon", "points": [[float(x), float(y)] for x, y in points],
                "name": name, "layer": layer}
        self.obstacles.append(item)
        return item

    # ------------------------------------------------------------------ #
    # 序列化
    # ------------------------------------------------------------------ #
    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": 1,
            "meta": self.meta.to_dict(),
            "field_size": self.field_size,
            "extent": list(self.extent),
            "resolution": self.resolution,
            "robot_radius": self.robot_radius,
            "safety_margin": self.safety_margin,
            "obstacles": self.obstacles,
            "zones": self.zones,
            "background": self.background,
        }

    def save(self, path: str) -> None:
        payload=json.dumps(self.to_dict(),ensure_ascii=False,indent=2)
        json.loads(payload)
        atomic_text(path,payload+"\n")

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> "Scene":
        meta_raw = data.get("meta", {}) or {}
        scene = Scene(meta=SceneMeta(**{k: v for k, v in meta_raw.items()
                                       if k in SceneMeta.__dataclass_fields__}))
        scene.field_size = float(data.get("field_size", FIELD_SIZE))
        ext = data.get("extent")
        if ext and len(ext) == 4:
            scene.extent = tuple(float(v) for v in ext)  # type: ignore[assignment]
        scene.resolution = float(data.get("resolution", 25.0))
        scene.robot_radius = float(data.get("robot_radius", 400.0))
        scene.safety_margin = float(data.get("safety_margin", 60.0))
        scene.obstacles = list(data.get("obstacles", []) or [])
        scene.zones = list(data.get("zones", []) or [])
        scene.background = dict(data.get("background", {}) or {})
        return scene

    @staticmethod
    def load(path: str) -> "Scene":
        with open(path, "r", encoding="utf-8") as fh:
            return Scene.from_dict(json.load(fh))


# --------------------------------------------------------------------------- #
# item -> 多边形
# --------------------------------------------------------------------------- #
def _item_to_polygon(item: Dict[str, Any]) -> Optional[Polygon]:
    if not isinstance(item, dict):
        return None
    kind = str(item.get("type", "")).lower()
    name = str(item.get("name", ""))
    layer = str(item.get("layer", LAYER_GROUND))

    if kind == "rect":
        poly = rect_polygon(float(item["cx"]), float(item["cy"]),
                            float(item["w"]), float(item["h"]))
        return Polygon(poly, name=name, kind=layer)

    if kind == "circle":
        return Circle(float(item["cx"]), float(item["cy"]), float(item["r"]),
                      name=name).to_polygon(int(item.get("segments", 48)))

    if kind == "polygon":
        pts = [(float(p[0]), float(p[1])) for p in item.get("points", [])]
        return Polygon(pts, name=name, kind=layer)

    if kind == "wall":
        wall = Wall(float(item["x1"]), float(item["y1"]),
                    float(item["x2"]), float(item["y2"]),
                    float(item.get("thickness", FENCE_THICKNESS)), name=name)
        return wall.to_polygon()

    return None


# --------------------------------------------------------------------------- #
# 默认模板: 女娲补天 11000x11000
# --------------------------------------------------------------------------- #
def _tower_top_centers_5x5(center: Point, pitch: float = 240.0) -> List[Point]:
    """第一公共区里 12 个塔顶中心的 5x5 棋盘格布局.

    规则 V1 4.1.4: 12 个塔顶放在 5x5 棋盘格里, **正中心格空置**;
    且"每个塔顶朝上的颜色应以棋盘中央列镜像对称"(即布局本身对中央列镜像对称)。

    取法(索引 i = x 方向 0..4, j = y 方向 0..4, 中心为 (2,2)):
      * 中心 (2,2) 空置;
      * 紧邻中心的 8 格(3x3 去掉中心);
      * 中列上下两端 (2,0) 与 (2,4);
      * 再补镜像对称的一对 (+1,-1) 与 (-1,-1)  -> 合计 12 格, 且严格镜像对称。
    """
    cx, cy = center
    # 索引 (i, j): i 是 x 方向, j 是 y 方向, 中心为 (2,2)
    cells: List[Tuple[int, int]] = [
        # 紧邻中心的 8 格(3x3 去掉中心)
        (1, 1), (1, 2), (1, 3),
        (2, 1), (2, 3),
        (3, 1), (3, 2), (3, 3),
        # 中列上下两端
        (2, 0), (2, 4),
        # 再补镜像对称的一对 -> 合计 12 格, 且严格关于中列镜像对称
        (0, 0), (4, 0),
    ]
    return [(cx + (i - 2) * pitch, cy + (j - 2) * pitch) for i, j in cells]



def _notch_x_range() -> Tuple[float, float]:
    """L1 南边界上缺口通道的 x 范围(红队侧; 蓝队镜像到 +x).

    红队侧缺口靠隔板, 中间留出中央地面车道; 蓝队侧完全镜像。
    """
    return (-NOTCH_HALF_W, NOTCH_HALF_W)


def _l1_blocks() -> List[Dict[str, Any]]:
    """把 L1 台体(地面层是墙, TR 不能进入)拆成块, 给换乘通道留出缺口.

    换乘通道(坡道 + 传递区 + 阶梯)是从地面通往传递区的唯一路径, 它的北段嵌在
    L1 里 —— 坡道就是贴着 L1 爬上去的。所以必须在 L1 南边界上开口,
    否则 TR 无论如何都到不了传递区(实测过: 不留缺口时 A* 直接报无可行路径)。

    缺口: x ∈ [NOTCH_X0, NOTCH_X1] (红侧; 蓝侧镜像, 中间靠隔板留出中央车道)
          y ∈ [NOTCH_Y0, NOTCH_Y1]

    切法:
        ┌───────────────┐  y = +3000
        │      A        │
        ├──────┬───┬────┤  y = NOTCH_Y1
        │  B   │缺口│ C  │
        ├──────┴───┴────┤  y = NOTCH_Y0
        │      D        │
        └───────────────┘  y = -3000
    """
    half = L1_HALF                        # 3000
    NOTCH_X0, NOTCH_X1 = _notch_x_range()
    NOTCH_Y0, NOTCH_Y1 = CORRIDOR_Y_SOUTH, CORRIDOR_Y_NORTH

    blocks = [
        # A: 缺口以北的整块
        (0.0, (NOTCH_Y1 + half) * 0.5, L1_SIZE, half - NOTCH_Y1, "L1 台体(北块)"),
        # D: 缺口以南的整块
        (0.0, (NOTCH_Y0 - half) * 0.5, L1_SIZE, NOTCH_Y0 + half, "L1 台体(南块)"),
        # B: 缺口西侧
        ((NOTCH_X0 - half) * 0.5, (NOTCH_Y0 + NOTCH_Y1) * 0.5,
         NOTCH_X0 + half, NOTCH_Y1 - NOTCH_Y0, "L1 台体(西块)"),
        # C: 缺口东侧
        ((NOTCH_X1 + half) * 0.5, (NOTCH_Y0 + NOTCH_Y1) * 0.5,
         half - NOTCH_X1, NOTCH_Y1 - NOTCH_Y0, "L1 台体(东块)"),
    ]
    out: List[Dict[str, Any]] = []
    for cx, cy, w, h, name in blocks:
        if w <= 0.1 or h <= 0.1:
            continue
        out.append({"type": "rect", "cx": cx, "cy": cy, "w": w, "h": h,
                    "name": name, "tag": "l1_body", "layer": LAYER_GROUND,
                    "enabled": True})
    return out


def build_default_scene() -> Scene:
    """按《第二十六届全国大学生机器人大赛女娲补天竞技赛规则V1》建默认场地.

    坐标系
    ------
    原点在场地中心(= 中央隔板中点), **+x 向东(蓝队侧), +y 向北**,
    场地范围 [-5500, 5500]^2, 与规则里 11000x11000 一致。

    布局来源(均已核对)
    ------------------
    * L1 区 6000x6000 居中 -> 地面四周各留 2500mm 宽跑道。官方图 2 上这条跑道
      的链式尺寸是 650 / 1000 / 1100 / 700 mm(沿 +y)。
    * 中央隔板把"地面 + L1"一起等分 -> 红队 -x 半场, 蓝队 +x 半场。
    * 第一公共区 1200x1200 在**南方**, "居中于场地围栏和 L1 围栏之间"
      -> 中心 y = -(3000 + 1250) = -4250。
    * 第二公共区 1000x1000 在**北方**, "边线距北边墙内侧面 1000mm"
      -> 中心 y = +5500 - 1000 - 500 = +4000; 五色石基座立在它中央。
    * 传递区/坡道/阶梯在 L1 边界处, 跨隔板, 两队镜像(官方俯视图)。

    ⚠ 建议现场复核的只有"传递区/坡道/阶梯这一段在 y 上的确切起止",
    它由官方图上的 3500 / 1000 / 300x3 / 1000 这一串尺寸推得;
    实测后改本文件顶部的 RAMP_LEN / TRANSFER_HALF_W / TRANSFER_Y / TRANSITION_X。
    """
    scene = Scene()
    scene.meta = SceneMeta(
        name="nvwa_butian_2027",
        note=("按规则V1与官方图重建; L1/L2/公共区/储存区/启动区位置已核对, "
              "坡道-传递区-阶梯段的确切边长建议现场复核"),
        origin_note="origin = 场地中心(中央隔板中点); +x 东(蓝队), +y 北",
    )
    scene.extent = (-5600.0, -5600.0, 5600.0, 5600.0)
    scene.resolution = 25.0
    # ⚠ 300mm 而不是 350mm: TR 赛前 700x700 -> 外接圆 350mm, 但传递区那一带的
    # 地面通道只有 1000mm 宽, 按 350mm 圆膨胀会把通道堵死。
    # 300mm = 按矩形半宽建模 + 允许贴边微调, 是窄通道的实际做法。
    scene.robot_radius = 300.0
    scene.safety_margin = 5.0

    t = FENCE_THICKNESS
    l1h = L1_HALF

    obs = scene.obstacles

    # ================= 固定障碍 =================
    # ---- 场地围栏(内高 80, 厚 50), 厚度朝内 ----
    obs += [
        {"type": "rect", "cx": 0.0, "cy": HALF - t / 2, "w": FIELD_SIZE, "h": t,
         "name": "场地围栏(北)", "tag": "fence", "layer": LAYER_GROUND, "enabled": True},
        {"type": "rect", "cx": 0.0, "cy": -(HALF - t / 2), "w": FIELD_SIZE, "h": t,
         "name": "场地围栏(南)", "tag": "fence", "layer": LAYER_GROUND, "enabled": True},
        {"type": "rect", "cx": HALF - t / 2, "cy": 0.0, "w": t, "h": FIELD_SIZE,
         "name": "场地围栏(东)", "tag": "fence", "layer": LAYER_GROUND, "enabled": True},
        {"type": "rect", "cx": -(HALF - t / 2), "cy": 0.0, "w": t, "h": FIELD_SIZE,
         "name": "场地围栏(西)", "tag": "fence", "layer": LAYER_GROUND, "enabled": True},
    ]

    # ---- L1 台体(6000x6000, 高出地面 600): 地面层是墙, TR 不能进入 ----
    # 拆成块并在换乘通道处留缺口(否则 TR 到不了传递区)。
    obs += _l1_blocks()

    # ---- 换乘通道里的坡道与阶梯: **不能当作普通地面走** ----
    # 规则 2.16: 坡道、阶梯和它们之间的传递区是"过渡区域", 不属于地面区;
    # 规则 3.2.1/3.4.5: TR 只能"通过坡道或阶梯"到达传递区(爬上去, 不是走过去)。
    # 所以标成不可通行, A* 不会把它们当平地。它们只占通道的"两侧",
    # 靠隔板留出一条中央车道, 让 TR 能走到缺口里(否则通道被堵死, 传递区到不了)。
    for sign, side in ((-1.0, "红"), (1.0, "蓝")):
        obs.append({"type": "rect", "cx": RAMP_CX * sign, "cy": RAMP_Y,
                    "w": RAMP_W, "h": RAMP_LEN, "name": f"坡道({side})",
                    "tag": "ramp_stair", "layer": LAYER_L1, "enabled": True})
        obs.append({"type": "rect", "cx": STAIR_CX * sign, "cy": STAIR_Y,
                    "w": STAIR_W, "h": STAIR_LEN, "name": f"阶梯({side})",
                    "tag": "ramp_stair", "layer": LAYER_L1, "enabled": True})
    # 传递区: 水平交接区, 是 TR 可以停留/行驶的地方(**不是障碍**)。
    # 宽度 2000 = 两队各 1000, 正好落在缺口里; TR 靠中间那条地面车道从南边走进来。
    obs.append({"type": "rect", "cx": 0.0, "cy": TRANSFER_Y,
                "w": 2 * TRANSFER_SIZE, "h": TRANSFER_SIZE,
                "name": "传递区(水平交接区, 可停留)", "tag": "transfer",
                "layer": LAYER_L1, "enabled": False})
    # ---- 中央隔板: 把地面和 L1 一起等分(通长)。
    #      在 L1 区间内与台体侧面共面, 所以地面段只需要补"北跑道 + 南跑道"。 ----
    for sign, label in ((1.0, "北"), (-1.0, "南")):
        obs.append({"type": "rect", "cx": 0.0,
                    "cy": sign * (l1h + GROUND_RUN * 0.5),
                    "w": DIVIDER_THICKNESS, "h": GROUND_RUN,
                    "name": f"中央隔板(地面{label}段)", "tag": "divider",
                    "layer": LAYER_GROUND, "enabled": True})

    # ---- 五色石基座(第二公共区中央) ----
    obs.append({"type": "circle", "cx": PEDESTAL_CENTER[0], "cy": PEDESTAL_CENTER[1],
                "r": PEDESTAL_D * 0.5, "name": "五色石基座", "tag": "pedestal",
                "layer": LAYER_GROUND, "enabled": True})

    # ================= 参考区域(只做可视化/取点参考) =================
    def zone(cx, cy, w, h, name, tag, color="", layer=LAYER_GROUND):
        return {"type": "rect", "cx": cx, "cy": cy, "w": w, "h": h,
                "name": name, "tag": tag, "layer": layer, "color": color}

    scene.zones = [
        zone(-(l1h + GROUND_RUN * 0.5), 0.0, GROUND_RUN, FIELD_SIZE,
             "地面区(红队半场)", "ground_red", "#f0d2d2"),
        zone(l1h + GROUND_RUN * 0.5, 0.0, GROUND_RUN, FIELD_SIZE,
             "地面区(蓝队半场)", "ground_blue", "#aad2e6"),
        zone(PUBLIC1_CENTER[0], PUBLIC1_CENTER[1], PUBLIC1_SIZE, PUBLIC1_SIZE,
             "地面第一公共区", "public1", "#efe9b8"),
        zone(PUBLIC2_CENTER[0], PUBLIC2_CENTER[1], PUBLIC2_SIZE, PUBLIC2_SIZE,
             "地面第二公共区", "public2", "#efe9b8"),
        # 传递区: 贴着 L1 南边界, 跨中央隔板, 一队一半 1000x1000。
        # 这是 TR 的交接区(要爬坡/爬梯才能到), 在地面图里标成"爬升可达", 见 pipeline。
        zone(-CORRIDOR_HALF_W * 0.5, TRANSFER_Y, CORRIDOR_HALF_W, TRANSFER_SIZE,
             "传递区(红)", "transfer_red", "#f5aa3c"),
        zone(CORRIDOR_HALF_W * 0.5, TRANSFER_Y, CORRIDOR_HALF_W, TRANSFER_SIZE,
             "传递区(蓝)", "transfer_blue", "#3caaf5"),
        # 坡道 / 阶梯: 传递区南侧的登高结构(不是平地), 一队一半
        zone(-RAMP_CX, RAMP_Y, RAMP_W, RAMP_LEN,
             "坡道(红)", "ramp_red", "#e8b4b4", LAYER_L1),
        zone(RAMP_CX, RAMP_Y, RAMP_W, RAMP_LEN,
             "坡道(蓝)", "ramp_blue", "#9dc6e0", LAYER_L1),
        zone(-STAIR_CX, STAIR_Y, STAIR_W, STAIR_LEN,
             "阶梯(红)", "stair_red", "#e8b4b4", LAYER_L1),
        zone(STAIR_CX, STAIR_Y, STAIR_W, STAIR_LEN,
             "阶梯(蓝)", "stair_blue", "#9dc6e0", LAYER_L1),
        zone(0.0, 0.0, L1_SIZE, L1_SIZE, "L1 区", "l1", "#c8c8c8", LAYER_L1),
        # L1 自带围栏: 内高 50 / 厚 50(规则 V1 1.1), 这里用一圈参考框示意
        zone(0.0, 0.0, L1_SIZE - L1_FENCE_THICKNESS, L1_SIZE, "L1 围栏(东/西示意)",
             "l1_fence", "#909090", LAYER_L1),
        zone(0.0, 0.0, L1_SIZE, L1_SIZE - L1_FENCE_THICKNESS, "L1 围栏(南/北示意)",
             "l1_fence", "#909090", LAYER_L1),
        zone(0.0, 0.0, L2_SIZE, L2_SIZE, "L2 区", "l2", "#b0b0b0", LAYER_L2),
        zone(-STORAGE_CENTER[0], STORAGE_CENTER[1], STORAGE_W, STORAGE_H,
             "红队储存区", "storage_red", "#f0d2d2"),
        zone(STORAGE_CENTER[0], STORAGE_CENTER[1], STORAGE_W, STORAGE_H,
             "蓝队储存区", "storage_blue", "#aad2e6"),
        zone(START_X_OUTER, START_Y, START_SIZE, START_SIZE,
             "红启动/重试区A", "start_red", "#df2222"),
        zone(START_X_INNER, START_Y, START_SIZE, START_SIZE,
             "红启动/重试区B", "start_red", "#df2222"),
        zone(-START_X_OUTER, START_Y, START_SIZE, START_SIZE,
             "蓝启动/重试区A", "start_blue", "#3200ff"),
        zone(-START_X_INNER, START_Y, START_SIZE, START_SIZE,
             "蓝启动/重试区B", "start_blue", "#3200ff"),
    ]

    # ================= 会变的东西: 默认不算障碍, 可按 tag 打开 =================
    # ---- 12 个塔顶 200x200 摆在第一公共区的 5x5 棋盘格里(正中心空置) ----
    for idx, (tx, ty) in enumerate(_tower_top_centers_5x5(PUBLIC1_CENTER)):
        obs.append({"type": "rect", "cx": tx, "cy": ty,
                    "w": TOWER_TOP_SIZE, "h": TOWER_TOP_SIZE,
                    "name": f"塔顶{idx + 1}", "tag": "tower_top",
                    "layer": LAYER_GROUND, "enabled": False})

    # ---- 储存区料堆: 20 个 350x350, 每列最多 2 个 -> 10 列沿 y 排 ----
    stack = 350.0
    cols = 10
    pitch = STORAGE_H / cols
    for side, sign in (("R", 1.0), ("B", -1.0)):
        cx = sign * STORAGE_CENTER[0]
        cy0 = STORAGE_CENTER[1] - STORAGE_H * 0.5
        for k in range(cols):
            obs.append({"type": "rect", "cx": cx, "cy": cy0 + (k + 0.5) * pitch,
                        "w": stack, "h": stack, "name": f"料堆{side}{k + 1}",
                        "tag": "stack", "layer": LAYER_GROUND, "enabled": False})

    return scene


def set_symmetric_layout(scene: Scene) -> None:
    """把红侧的东西镜像到蓝侧(方便改完一半就同步).

    保留原实现位, 后续 UI 上做"改一半自动镜像"用.
    """
    raise NotImplementedError
