"""场地画布: 底图 + 障碍 + 路径 + 交互.

支持的操作
----------
* 滚轮缩放, 中键/右键拖拽平移
* 左键按当前"工具"工作:
    起终点  : 点一下设起点, 再点一下设终点(可拖动)
    障碍矩形: 拖出一个矩形障碍
    障碍圆  : 按下是圆心, 拖出半径
    障碍多边形: 依次点, 双击/回车闭合
    标定    : 第 1 次点=参考点, 第 2 次点=方向点
* Delete: 删除选中的障碍
* F: 缩放到全部

画布坐标系是**场地毫米**, y 轴向上(和场地系一致), 绘制时自动翻转 y。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from PySide6.QtCore import QPoint, QPointF, QRectF, Qt, Signal
from PySide6.QtGui import (QBrush, QColor, QFont, QImage, QPainter, QPainterPath,
                           QPen, QPixmap, QPolygonF, QTransform)
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..core.field_frame import FieldFrame
from ..core.geometry import Point, Pose, circle_polygon, rect_polygon
from ..core.scene import Scene

# 工具
TOOL_POINTS = "points"
TOOL_RECT = "rect"
TOOL_CIRCLE = "circle"
TOOL_POLY = "poly"
TOOL_CALIB = "calib"
TOOL_PAN = "pan"

_ZONE_COLORS = {
    "start_red": "#df2222", "start_blue": "#3200ff",
    "storage_red": "#e8b4b4", "storage_blue": "#9dc6e0",
    "transfer_red": "#f5aa3c", "transfer_blue": "#3caaf5",
    "public1": "#efe9b8", "public2": "#efe9b8",
}


@dataclass
class CanvasState:
    """画布要画的东西, 由主窗口填充."""

    scene: Optional[Scene] = None
    background: Optional[QImage] = None
    frame: FieldFrame = field(default_factory=FieldFrame)
    show_background: bool = True
    background_opacity: float = 0.55
    show_inflated: bool = True
    show_obstacles: bool = True
    show_zones: bool = True
    show_searched: bool = True
    show_los: bool = True
    show_smooth: bool = True
    show_table: bool = True
    show_colors_by_speed: bool = True

    start: Optional[Pose] = None
    goal: Optional[Pose] = None
    start_yaw_deg: float = 0.0
    goal_yaw_deg: float = 0.0

    calib_ref: Optional[Point] = None       # 图片像素坐标
    calib_dir: Optional[Point] = None

    route_points: List[Tuple[float, float]] = field(default_factory=list)
    route_speeds: List[float] = field(default_factory=list)
    searched_cells: List[Tuple[int, int]] = field(default_factory=list)
    los_points: List[Point] = field(default_factory=list)
    smooth_points: List[Point] = field(default_factory=list)
    grid_spec: Optional[object] = None


class FieldCanvas(QWidget):
    """场地视图."""

    pointChanged = Signal(str, float, float)     # ('start'|'goal', x, y)
    yawChanged = Signal(str, float)
    sceneEdited = Signal(str)                    # 提示词, 主窗口据此刷新
    calibPicked = Signal(str, float, float)      # ('ref'|'dir', px, py)
    cursorMoved = Signal(float, float)
    statusMessage = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(560, 560)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        self.state = CanvasState()
        self.tool = TOOL_POINTS

        # 视图变换: 场地毫米 -> 屏幕像素
        self._scale = 0.08          # px per mm
        self._center = (0.0, 0.0)   # 视口中心对应的场地坐标
        self._panning = False
        self._pan_last = QPoint()
        self._drag_target: Optional[str] = None

        self._rect_start: Optional[Point] = None
        self._rect_now: Optional[Point] = None
        self._circle_start: Optional[Point] = None
        self._circle_now: Optional[Point] = None
        self._poly_points: List[Point] = []
        self._selected_index: Optional[int] = None

        self.costmap = None     # 由主窗口塞进来, 用于画膨胀区

    # ------------------------------------------------------------------ #
    # 坐标变换
    # ------------------------------------------------------------------ #
    def world_to_screen(self, x: float, y: float) -> QPointF:
        sx = self.width() * 0.5 + (x - self._center[0]) * self._scale
        sy = self.height() * 0.5 - (y - self._center[1]) * self._scale
        return QPointF(sx, sy)

    def screen_to_world(self, px: float, py: float) -> Point:
        x = (px - self.width() * 0.5) / self._scale + self._center[0]
        y = -(py - self.height() * 0.5) / self._scale + self._center[1]
        return (x, y)

    def fit_to_extent(self, extent: Optional[Tuple[float, float, float, float]] = None) -> None:
        if extent is None:
            scene = self.state.scene
            extent = scene.extent if scene is not None else (-5500, -5500, 5500, 5500)
        x0, y0, x1, y1 = extent
        w = max(1.0, x1 - x0)
        h = max(1.0, y1 - y0)
        self._center = ((x0 + x1) * 0.5, (y0 + y1) * 0.5)
        self._scale = min(self.width() / w, self.height() / h) * 0.95
        self.update()

    def resizeEvent(self, event):  # noqa: N802
        super().resizeEvent(event)
        if self._scale <= 0:
            self.fit_to_extent()

    # ------------------------------------------------------------------ #
    # 绘制
    # ------------------------------------------------------------------ #
    def paintEvent(self, event):  # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#1e1f22"))

        self._draw_background(painter)
        self._draw_grid(painter)
        self._draw_zones(painter)
        self._draw_costmap(painter)
        self._draw_obstacles(painter)
        self._draw_zone_labels(painter)
        self._draw_entrances(painter)
        self._draw_route(painter)
        self._draw_points(painter)
        self._draw_draft(painter)
        self._draw_hud(painter)
        painter.end()

    def _draw_background(self, painter: QPainter) -> None:
        img = self.state.background
        if img is None or not self.state.show_background:
            return
        frame = self.state.frame
        # 图片四角映射到场地系, 再映射到屏幕
        corners = [(0.0, 0.0), (float(img.width()), 0.0),
                   (float(img.width()), float(img.height())), (0.0, float(img.height()))]
        world = [frame.image_to_field(u, v) for u, v in corners]
        screen = [self.world_to_screen(x, y) for x, y in world]

        # 用仿射变换直接把图片贴上去(两个基向量 + 原点)
        o = screen[0]
        ex = screen[1]
        ey = screen[3]
        painter.save()
        painter.setOpacity(self.state.background_opacity)
        transform = QTransform(ex.x() - o.x(), ex.y() - o.y(),
                               ey.x() - o.x(), ey.y() - o.y(),
                               o.x(), o.y())
        painter.setTransform(transform, True)
        painter.drawImage(QRectF(0, 0, img.width(), img.height()), img)
        painter.restore()

    def _draw_grid(self, painter: QPainter) -> None:
        scene = self.state.scene
        if scene is None:
            return
        # 选一个合适的网格间距
        target_px = 80.0
        step = 500.0
        for cand in (100.0, 200.0, 500.0, 1000.0, 2000.0):
            if cand * self._scale >= target_px:
                step = cand
                break
        else:
            step = 2000.0

        x0, y0, x1, y1 = scene.extent
        pen = QPen(QColor(70, 75, 85), 1.0)
        painter.setPen(pen)
        x = math.ceil(x0 / step) * step
        while x <= x1:
            p1 = self.world_to_screen(x, y0)
            p2 = self.world_to_screen(x, y1)
            painter.drawLine(p1, p2)
            x += step
        y = math.ceil(y0 / step) * step
        while y <= y1:
            p1 = self.world_to_screen(x0, y)
            p2 = self.world_to_screen(x1, y)
            painter.drawLine(p1, p2)
            y += step

    def _draw_zones(self, painter: QPainter) -> None:
        scene = self.state.scene
        if scene is None or not self.state.show_zones:
            return
        for item in scene.zones:
            if not isinstance(item, dict) or item.get("enabled", True) is False:
                continue
            pts = _item_polygon(item)
            if len(pts) < 3:
                continue
            color = QColor(item.get("color") or _ZONE_COLORS.get(str(item.get("tag", "")), "#888"))
            color.setAlpha(70)
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(color.darker(140), 1.0, Qt.DashLine))
            painter.drawPolygon(QPolygonF([self.world_to_screen(x, y) for x, y in pts]))

    def _draw_costmap(self, painter: QPainter) -> None:
        cm = self.costmap
        if cm is None or self.state.scene is None:
            return
        extent = cm.spec.extent()
        tl = self.world_to_screen(extent[0], extent[3])
        br = self.world_to_screen(extent[2], extent[1])
        w = max(1.0, br.x() - tl.x())
        h = max(1.0, br.y() - tl.y())

        if self.state.show_inflated:
            rgba = _mask_to_qimage(cm.inflated, QColor(255, 110, 110, 90))
            painter.drawImage(QRectF(tl.x(), tl.y(), w, h), rgba)
        if self.state.show_searched and self.state.searched_cells:
            img = self._searched_image(cm)
            painter.setOpacity(0.75)
            painter.drawImage(QRectF(tl.x(), tl.y(), w, h), img)
            painter.setOpacity(1.0)

    def _searched_image(self, cm) -> QImage:
        import numpy as np
        spec = cm.spec
        mask = np.zeros((spec.height, spec.width), dtype=bool)
        for row, col in self.state.searched_cells:
            if 0 <= row < spec.height and 0 <= col < spec.width:
                mask[row, col] = True
        return _mask_to_qimage(mask, QColor(120, 180, 245, 120))

    def _draw_obstacles(self, painter: QPainter) -> None:
        scene = self.state.scene
        if scene is None or not self.state.show_obstacles:
            return
        for idx, item in enumerate(scene.obstacles):
            if not isinstance(item, dict):
                continue
            pts = _item_polygon(item)
            if len(pts) < 3:
                continue
            enabled = item.get("enabled", True) is not False
            if getattr(self.state,"ground_only",False) and item.get("tag")=="transfer":
                enabled=True              # ground display; never change scene enabled
            selected = (idx == self._selected_index)
            if enabled:
                fill = QColor(90, 90, 90)
                edge = QColor(230, 230, 230)
                if item.get("tag") == "l1_body":
                    fill = QColor(90, 90, 90, 70)
                elif item.get("tag") == "ramp_stair":
                    fill = QColor("#af6b64" if item.get("cx", 0) < 0 else "#557fa5")
                    fill.setAlpha(100)
            else:
                fill = QColor(90, 90, 90, 60)
                edge = QColor(150, 150, 150, 160)
            painter.setBrush(QBrush(fill))
            painter.setPen(QPen(QColor("#ffd166") if selected else edge,
                                2.0 if selected else 1.0,
                                Qt.SolidLine if enabled else Qt.DotLine))
            painter.drawPolygon(QPolygonF([self.world_to_screen(x, y) for x, y in pts]))

    def _draw_zone_labels(self, painter: QPainter) -> None:
        if not self.state.show_zones or self.state.scene is None:
            return
        painter.setFont(QFont("Microsoft YaHei", 9))
        painter.setPen(QColor("#edf2f7"))
        for z in self.state.scene.zones:
            tag = str(z.get("tag", ""))
            if tag in ("l1", "l2", "public1", "public2", "storage_red", "storage_blue",
                       "start_red", "start_blue", "ramp_red", "ramp_blue",
                       "stair_red", "stair_blue", "transfer_red", "transfer_blue", "l2_step"):
                center = self.world_to_screen(z["cx"], z["cy"])
                text = str(z.get("name", ""))
                if tag.startswith("start_"):
                    text = "启动" + text[-1]
                elif tag.startswith("storage_"):
                    text = "红储存" if tag.endswith("red") else "蓝储存"
                elif tag.startswith("ramp_"):
                    text = "红坡道" if tag.endswith("red") else "蓝坡道"
                    if getattr(self.state,"ground_only",False): text+="（禁入）"
                elif tag.startswith("stair_"):
                    text = "三级阶梯"
                    if getattr(self.state,"ground_only",False): text+="（禁入）"
                elif tag.startswith("transfer_"):
                    text = "平台（禁入）" if getattr(self.state,"ground_only",False) else "传递区"
                elif tag == "l2_step":
                    text = "一级台阶"
                if tag == "l1":
                    center = self.world_to_screen(-2000, 2200)
                painter.drawText(QRectF(center.x()-85, center.y()-10, 170, 20),
                                 Qt.AlignCenter, text)

    def _draw_entrances(self,painter: QPainter) -> None:
        if getattr(self.state,"ground_only",False): return
        if self.state.scene is None: return
        painter.setFont(QFont("Microsoft YaHei",9))
        for item in self.state.scene.obstacles:
            nav=item.get("navigation",{})
            if nav.get("kind") not in ("ramp","stairs"): continue
            x,y=item["cx"],nav["ground_end_y_mm"]
            direction=-1 if nav["kind"]=="ramp" else 1
            painter.setPen(QPen(QColor("#4ade80"),2.5))
            painter.drawLine(self.world_to_screen(x-item["w"]/2,y),self.world_to_screen(x+item["w"]/2,y))
            a=self.world_to_screen(x,y-direction*500)
            b=self.world_to_screen(x,y+direction*180)
            painter.drawLine(a,b)
            painter.drawLine(b,b+QPointF(-5,direction*8))
            painter.drawLine(b,b+QPointF(5,direction*8))
            painter.drawText(QRectF(a.x()-45,a.y()-10,90,20),Qt.AlignCenter,
                "坡道入口" if nav["kind"]=="ramp" else "阶梯入口")

    def _draw_route(self, painter: QPainter) -> None:
        st = self.state
        if st.show_los and len(st.los_points) > 1:
            painter.setPen(QPen(QColor(255, 160, 60), 1.6, Qt.DashLine))
            painter.drawPolyline(QPolygonF([self.world_to_screen(x, y) for x, y in st.los_points]))
        if st.show_smooth and len(st.smooth_points) > 1:
            painter.setPen(QPen(QColor(90, 200, 255), 1.2))
            painter.drawPolyline(QPolygonF([self.world_to_screen(x, y) for x, y in st.smooth_points]))
        if st.show_table and len(st.route_points) > 1:
            if st.show_colors_by_speed and len(st.route_speeds) == len(st.route_points):
                vmax = max(1.0, max(st.route_speeds))
                for i in range(1, len(st.route_points)):
                    t = st.route_speeds[i] / vmax
                    color = QColor.fromHsvF(0.66 * (1.0 - t) * 0.66, 0.9, 1.0)
                    painter.setPen(QPen(color, 3.0))
                    painter.drawLine(self.world_to_screen(*st.route_points[i - 1]),
                                     self.world_to_screen(*st.route_points[i]))
            else:
                painter.setPen(QPen(QColor(230, 60, 60), 2.6))
                painter.drawPolyline(QPolygonF([self.world_to_screen(x, y)
                                                for x, y in st.route_points]))
            if len(st.route_points) > 2:
                painter.setPen(QPen(QColor(255, 255, 255, 120), 0))
                painter.setBrush(QBrush(QColor(255, 255, 255, 170)))
                stride = max(1, len(st.route_points) // 60)
                for i in range(0, len(st.route_points), stride):
                    p = self.world_to_screen(*st.route_points[i])
                    painter.drawEllipse(p, 1.6, 1.6)

    def _draw_points(self, painter: QPainter) -> None:
        st = self.state
        for name, pose, color in (("start", st.start, QColor("#22c55e")),
                                  ("goal", st.goal, QColor("#ef4444"))):
            if pose is None:
                continue
            p = self.world_to_screen(pose.x, pose.y)
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(QColor("white"), 1.5))
            painter.drawEllipse(p, 6.0, 6.0)
            # 朝向箭头
            if name == "start":
                yaw = math.radians(st.start_yaw_deg)
            else:
                yaw = math.radians(st.goal_yaw_deg)
            tip = self.world_to_screen(pose.x + 600.0 * math.cos(yaw),
                                       pose.y + 600.0 * math.sin(yaw))
            painter.setPen(QPen(color, 2.0))
            painter.drawLine(p, tip)
            painter.setPen(QPen(QColor("white")))
            painter.setFont(QFont("Consolas", 9))
            painter.drawText(p + QPointF(8, -8), name.upper())

        # 标定点
        for name, pt, color in (("ref", st.calib_ref, QColor("#facc15")),
                                ("dir", st.calib_dir, QColor("#38bdf8"))):
            if pt is None:
                continue
            p = self.world_to_screen(*self.state.frame.image_to_field(*pt))
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(color, 2.0))
            painter.drawEllipse(p, 7.0, 7.0)
            painter.drawLine(p + QPointF(-10, 0), p + QPointF(10, 0))
            painter.drawLine(p + QPointF(0, -10), p + QPointF(0, 10))
            painter.setPen(QPen(color))
            painter.drawText(p + QPointF(9, 14), name)
        if st.calib_ref is not None and st.calib_dir is not None:
            p1 = self.world_to_screen(*self.state.frame.image_to_field(*st.calib_ref))
            p2 = self.world_to_screen(*self.state.frame.image_to_field(*st.calib_dir))
            painter.setPen(QPen(QColor("#facc15"), 1.2, Qt.DashLine))
            painter.drawLine(p1, p2)

    def _draw_draft(self, painter: QPainter) -> None:
        if self._rect_start and self._rect_now:
            x0, y0 = self._rect_start
            x1, y1 = self._rect_now
            pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
            painter.setBrush(QBrush(QColor(255, 209, 102, 70)))
            painter.setPen(QPen(QColor("#ffd166"), 1.4, Qt.DashLine))
            painter.drawPolygon(QPolygonF([self.world_to_screen(x, y) for x, y in pts]))
        if self._circle_start and self._circle_now:
            r = math.hypot(self._circle_now[0] - self._circle_start[0],
                           self._circle_now[1] - self._circle_start[1])
            pts = circle_polygon(self._circle_start[0], self._circle_start[1], r, 40)
            painter.setBrush(QBrush(QColor(255, 209, 102, 70)))
            painter.setPen(QPen(QColor("#ffd166"), 1.4, Qt.DashLine))
            painter.drawPolygon(QPolygonF([self.world_to_screen(x, y) for x, y in pts]))
        if self._poly_points:
            pts = list(self._poly_points)
            if self._rect_now is not None:
                pts = pts + [self._rect_now]
            painter.setPen(QPen(QColor("#ffd166"), 1.6))
            painter.setBrush(Qt.NoBrush)
            painter.drawPolyline(QPolygonF([self.world_to_screen(x, y) for x, y in pts]))
            for x, y in self._poly_points:
                painter.setBrush(QBrush(QColor("#ffd166")))
                painter.drawEllipse(self.world_to_screen(x, y), 3.0, 3.0)

    def _draw_hud(self, painter: QPainter) -> None:
        painter.setPen(QPen(QColor(200, 205, 215)))
        painter.setFont(QFont("Consolas", 9))
        scale_txt = f"1 格 = {1000.0 / self._scale:.0f}mm?  缩放 {self._scale * 1000:.1f}px/m"
        painter.drawText(10, self.height() - 24, f"缩放 {self._scale:.4f} px/mm")
        painter.drawText(10, self.height() - 10, "滚轮缩放 / 中键平移 / F 适应窗口")

    # ------------------------------------------------------------------ #
    # 交互
    # ------------------------------------------------------------------ #
    def wheelEvent(self, event):  # noqa: N802
        pos = event.position()
        before = self.screen_to_world(pos.x(), pos.y())
        factor = 1.18 if event.angleDelta().y() > 0 else 1.0 / 1.18
        self._scale = max(0.002, min(6.0, self._scale * factor))
        after = self.screen_to_world(pos.x(), pos.y())
        self._center = (self._center[0] + before[0] - after[0],
                        self._center[1] + before[1] - after[1])
        self.update()

    def mousePressEvent(self, event):  # noqa: N802
        pos = event.position()
        world = self.screen_to_world(pos.x(), pos.y())

        if event.button() in (Qt.MiddleButton,) or (event.button() == Qt.RightButton
                                                    and self.tool == TOOL_PAN):
            self._panning = True
            self._pan_last = event.pos()
            return
        if event.button() != Qt.LeftButton:
            return

        if self.tool == TOOL_POINTS:
            hit = self._hit_marker(world)
            if hit is not None:
                self._drag_target = hit
                return
            # 找最近的起/终点, 太远就移动"当前要设的那个"
            if self.state.start is None:
                self._drag_target = "start"
                self.state.start = Pose(world[0], world[1], 0.0)
            elif self.state.goal is None:
                self._drag_target = "goal"
                self.state.goal = Pose(world[0], world[1], 0.0)
            else:
                # 都设过了: 就近移动
                d_start = math.hypot(world[0] - self.state.start.x, world[1] - self.state.start.y)
                d_goal = math.hypot(world[0] - self.state.goal.x, world[1] - self.state.goal.y)
                if d_start <= d_goal:
                    self._drag_target = "start"
                    self.state.start = Pose(world[0], world[1], self.state.start.yaw)
                else:
                    self._drag_target = "goal"
                    self.state.goal = Pose(world[0], world[1], self.state.goal.yaw)
            self._emit_point(self._drag_target)
            self.update()
            return

        if self.tool == TOOL_CALIB:
            which = "ref" if self.state.calib_ref is None else "dir"
            self.calibPicked.emit(which, pos.x(), pos.y())
            return

        if self.tool == TOOL_RECT:
            self._rect_start = world
            self._rect_now = world
            self.update()
            return

        if self.tool == TOOL_CIRCLE:
            self._circle_start = world
            self._circle_now = world
            self.update()
            return

        if self.tool == TOOL_POLY:
            self._poly_points.append(world)
            self._rect_now = world
            self.update()
            return

    def mouseMoveEvent(self, event):  # noqa: N802
        pos = event.position()
        world = self.screen_to_world(pos.x(), pos.y())
        self.cursorMoved.emit(world[0], world[1])

        if self._panning:
            delta = event.pos() - self._pan_last
            self._pan_last = event.pos()
            self._center = (self._center[0] - delta.x() / self._scale,
                            self._center[1] + delta.y() / self._scale)
            self.update()
            return

        if self._drag_target is not None:
            if self._drag_target == "start" and self.state.start is not None:
                self.state.start.x, self.state.start.y = world
            elif self._drag_target == "goal" and self.state.goal is not None:
                self.state.goal.x, self.state.goal.y = world
            self._emit_point(self._drag_target)
            self.update()
            return

        if self._rect_start is not None:
            self._rect_now = world
            self.update()
        elif self._circle_start is not None:
            self._circle_now = world
            self.update()
        elif self._poly_points:
            self._rect_now = world
            self.update()

    def mouseReleaseEvent(self, event):  # noqa: N802
        if event.button() == Qt.MiddleButton or self._panning:
            self._panning = False
            return
        world = self.screen_to_world(event.position().x(), event.position().y())

        if self._drag_target is not None:
            self._drag_target = None
            return

        scene = self.state.scene
        if scene is None:
            return

        if self.tool == TOOL_RECT and self._rect_start is not None:
            x0, y0 = self._rect_start
            x1, y1 = world
            w, h = abs(x1 - x0), abs(y1 - y0)
            if w > 20.0 and h > 20.0:
                scene.add_obstacle_rect((x0 + x1) * 0.5, (y0 + y1) * 0.5, w, h,
                                        name="手动矩形", layer="ground")
                scene.obstacles[-1]["tag"] = "manual"
                self.sceneEdited.emit(f"新增矩形障碍 {w:.0f}x{h:.0f}mm")
            self._rect_start = None
            self._rect_now = None
            self.update()

        elif self.tool == TOOL_CIRCLE and self._circle_start is not None:
            r = math.hypot(world[0] - self._circle_start[0], world[1] - self._circle_start[1])
            if r > 20.0:
                scene.add_obstacle_circle(self._circle_start[0], self._circle_start[1], r,
                                          name="手动圆")
                scene.obstacles[-1]["tag"] = "manual"
                self.sceneEdited.emit(f"新增圆形障碍 R={r:.0f}mm")
            self._circle_start = None
            self._circle_now = None
            self.update()

    def mouseDoubleClickEvent(self, event):  # noqa: N802
        if self.tool == TOOL_POLY and len(self._poly_points) >= 3:
            self._commit_polygon()

    def keyPressEvent(self, event):  # noqa: N802
        key = event.key()
        if key == Qt.Key_F:
            self.fit_to_extent()
        elif key in (Qt.Key_Return, Qt.Key_Enter) and self._poly_points:
            self._commit_polygon()
        elif key == Qt.Key_Escape:
            self._poly_points.clear()
            self._rect_start = None
            self._circle_start = None
            self._rect_now = None
            self._circle_now = None
            self.update()
        elif key == Qt.Key_Delete and self._selected_index is not None:
            scene = self.state.scene
            if scene is not None and 0 <= self._selected_index < len(scene.obstacles):
                scene.obstacles.pop(self._selected_index)
                self._selected_index = None
                self.sceneEdited.emit("已删除一个障碍")
                self.update()
        else:
            super().keyPressEvent(event)

    # ------------------------------------------------------------------ #
    def _commit_polygon(self) -> None:
        scene = self.state.scene
        if scene is not None and len(self._poly_points) >= 3:
            scene.add_obstacle_polygon(self._poly_points, name="手动多边形")
            scene.obstacles[-1]["tag"] = "manual"
            self.sceneEdited.emit(f"新增多边形障碍 {len(self._poly_points)} 顶点")
        self._poly_points = []
        self._rect_now = None
        self.update()

    def _emit_point(self, which: str) -> None:
        pose = self.state.start if which == "start" else self.state.goal
        if pose is not None:
            self.pointChanged.emit(which, pose.x, pose.y)

    def _hit_marker(self, world: Point, tol_px: float = 14.0) -> Optional[str]:
        tol_mm = tol_px / max(1e-6, self._scale)
        for name, pose in (("start", self.state.start), ("goal", self.state.goal)):
            if pose is None:
                continue
            if math.hypot(world[0] - pose.x, world[1] - pose.y) <= tol_mm:
                return name
        return None


# --------------------------------------------------------------------------- #
def _item_polygon(item: Dict) -> List[Point]:
    kind = str(item.get("type", "")).lower()
    if kind == "rect":
        return rect_polygon(float(item["cx"]), float(item["cy"]),
                            float(item["w"]), float(item["h"]))
    if kind == "circle":
        return circle_polygon(float(item["cx"]), float(item["cy"]),
                              float(item["r"]), int(item.get("segments", 40)))
    if kind == "polygon":
        return [(float(p[0]), float(p[1])) for p in item.get("points", [])]
    if kind == "wall":
        from ..core.geometry import Wall
        return Wall(float(item["x1"]), float(item["y1"]),
                    float(item["x2"]), float(item["y2"]),
                    float(item.get("thickness", 50.0))).to_polygon().points
    return []


def _mask_to_qimage(mask, color: QColor) -> QImage:
    """World rows increase with y; image rows increase downward, so flip vertically."""
    import numpy as np
    h, w = mask.shape
    rgba = np.zeros((h, w, 4), dtype=np.uint8)
    rgba[..., 0] = color.red()
    rgba[..., 1] = color.green()
    rgba[..., 2] = color.blue()
    rgba[..., 3] = np.where(mask, color.alpha(), 0).astype(np.uint8)
    rgba = np.ascontiguousarray(rgba[::-1])
    img = QImage(rgba.data, w, h, 4 * w, QImage.Format_RGBA8888)
    return img.copy()
