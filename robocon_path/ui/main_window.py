"""主窗口: 菜单 + 工具栏 + 画布 + 参数面板 + 曲线/代码预览."""

from __future__ import annotations

import json
import math
import os
import sys
import traceback
from typing import Dict, List, Optional, Tuple

import numpy as np
from PySide6.QtCore import QObject, QThread, Qt, Signal, Slot
from PySide6.QtGui import QAction, QImage, QKeySequence, QPixmap
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
                               QFileDialog, QFormLayout, QGridLayout, QGroupBox,
                               QHBoxLayout, QLabel, QLineEdit, QListWidget,
                               QListWidgetItem, QMainWindow, QMessageBox,
                               QPlainTextEdit, QPushButton, QRadioButton,
                               QScrollArea, QSizePolicy, QSpinBox, QSplitter,
                               QStatusBar, QTabWidget, QToolBar, QVBoxLayout,
                               QWidget)

from ..core.field_frame import FieldFrame
from ..core.geometry import Pose
from ..core.scene import Scene, build_default_scene
from ..core.trajectory import Trajectory
from ..export import rows_for_export, trajectory_to_c_code, trajectory_to_csv
from ..export.c_code import (CCodeConfig, STYLE_C_STRUCT, STYLE_FLOAT_ARRAY,
                             STYLE_PATH_POINT)
from ..pipeline import PipelineConfig, build_map, plan_route, resolve_scene
from ..core.surface_navigation import SurfaceMap, required as surface_required
from ..core.file_io import atomic_text
from .canvas import (TOOL_CALIB, TOOL_CIRCLE, TOOL_POINTS, TOOL_POLY, TOOL_RECT,
                     FieldCanvas)
from .chart import CurvePlot

APP_TITLE = "ROBOCON 2027 TR 离线路径点表生成器"


# --------------------------------------------------------------------------- #
# 后台规划线程
# --------------------------------------------------------------------------- #
class PlanWorker(QObject):
    progress = Signal(str)
    finished = Signal(object)      # Trajectory
    failed = Signal(str)

    def __init__(self, scene: Scene, start: Pose, goal: Pose, cfg: PipelineConfig):
        super().__init__()
        self.scene = scene
        self.start = start
        self.goal = goal
        self.cfg = cfg

    @Slot()
    def run(self) -> None:
        try:
            cm = build_map(self.scene, self.cfg)
            traj = plan_route(self.scene, self.start, self.goal, self.cfg,
                              costmap=cm, progress=self.progress.emit)
            self.finished.emit(traj)
        except Exception as exc:  # pragma: no cover
            self.failed.emit(f"{exc}\n{traceback.format_exc()}")


# --------------------------------------------------------------------------- #
class MainWindow(QMainWindow):
    def __init__(self, scene: Optional[Scene] = None):
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(1560, 940)

        sim_scene = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
                                 "scenes", "nvwa_butian_2027_from_sim.json")
        self.scene: Scene = (scene if scene is not None else
                             Scene.load(sim_scene) if os.path.isfile(sim_scene) else
                             build_default_scene())
        self.frame = FieldFrame()
        self.traj: Optional[Trajectory] = None
        self.costmap = None
        self._image: Optional[QImage] = None
        self._image_path: str = ""
        self._image_scale: float = 1.0
        self._thread: Optional[QThread] = None
        self._worker: Optional[PlanWorker] = None

        self._build_ui()
        self._load_scene_into_ui()
        self._refresh_obstacle_list()
        self._log(f"就绪。已载入场景：{self.scene.meta.name} (11000x11000)。")
        self._log("仿真地图：红队在左，蓝队在右，北方向上；灰色/粉色区域不能作为地面路径。")
        self._log("当前仅规划地面区：绕开坡道、阶梯及高平台；仿真场地用生成器更新。")

    # ================================================================== #
    # 界面搭建
    # ================================================================== #
    def _build_ui(self) -> None:
        self._build_actions()

        self.canvas = FieldCanvas()
        self.canvas.state.scene = self.scene
        self.canvas.state.frame = self.frame
        self.canvas.pointChanged.connect(self._on_point_changed)
        self.canvas.sceneEdited.connect(self._on_scene_edited)
        self.canvas.calibPicked.connect(self._on_calib_picked)
        self.canvas.cursorMoved.connect(self._on_cursor_moved)

        self.curves = QTabWidget()
        self.plot_speed = CurvePlot("沿路速度 v(s)", "s [mm]", "mm/s")
        self.plot_curv = CurvePlot("曲率 kappa(s)", "s [mm]", "1/mm")
        self.plot_vt = CurvePlot("速度-时间 v(t)", "t [s]", "mm/s")
        self.curves.addTab(self.plot_speed, "速度-弧长")
        self.curves.addTab(self.plot_curv, "曲率-弧长")
        self.curves.addTab(self.plot_vt, "速度-时间")
        self.code_view = QPlainTextEdit()
        self.code_view.setReadOnly(True)
        self.code_view.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.code_view.setStyleSheet("font-family: Consolas, monospace; font-size: 11px;")
        self.curves.addTab(self.code_view, "点表代码")

        bottom = QWidget()
        bl = QVBoxLayout(bottom)
        bl.setContentsMargins(4, 4, 4, 4)
        bl.addWidget(self.curves)

        right = self._build_right_panel()
        left = self._build_left_panel()

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(self.canvas)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        splitter.setSizes([220, 900, 380])

        v_split = QSplitter(Qt.Vertical)
        v_split.addWidget(splitter)
        v_split.addWidget(bottom)
        v_split.setStretchFactor(0, 1)
        v_split.setStretchFactor(1, 0)
        v_split.setSizes([700, 240])
        self.setCentralWidget(v_split)

        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.lbl_cursor = QLabel("x=0.0  y=0.0 mm")
        self.status.addPermanentWidget(self.lbl_cursor)
        self.lbl_info = QLabel("未规划")
        self.status.addWidget(self.lbl_info)

    def _build_actions(self) -> None:
        tb = QToolBar("主工具栏")
        tb.setMovable(False)
        self.addToolBar(tb)

        def act(text: str, slot, shortcut: str = "", tip: str = "") -> QAction:
            a = QAction(text, self)
            a.triggered.connect(slot)
            if shortcut:
                a.setShortcut(QKeySequence(shortcut))
            if tip:
                a.setToolTip(tip)
            tb.addAction(a)
            return a

        act("导入地图", self.on_import_map, "Ctrl+O", "导入场地俯视图(png/jpg)")
        act("载入场景JSON", self.on_load_scene, "Ctrl+L")
        act("保存场景JSON", self.on_save_scene, "Ctrl+S")
        act("内置场地模板", self.on_default_scene)
        tb.addSeparator()
        act("规划路径", self.on_plan, "F5", "A* -> LOS -> B样条 -> 速度规划")
        act("导出点表", self.on_export, "Ctrl+E", "导出 .c / .csv 点表文件")
        tb.addSeparator()
        act("适应窗口", lambda: self.canvas.fit_to_extent(), "F")

    def _build_left_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(6, 6, 6, 6)

        layout.addWidget(QLabel("<b>工具</b>"))
        self.tool_box = QComboBox()
        for label, value in (("起终点 (拖拽)", TOOL_POINTS),
                             ("矩形障碍", TOOL_RECT),
                             ("圆形障碍", TOOL_CIRCLE),
                             ("多边形障碍", TOOL_POLY),
                             ("地图标定", TOOL_CALIB)):
            self.tool_box.addItem(label, value)
        self.tool_box.currentIndexChanged.connect(
            lambda: setattr(self.canvas, "tool", self.tool_box.currentData()))
        layout.addWidget(self.tool_box)

        layout.addWidget(QLabel("<b>显示</b>"))
        self.chk_bg = QCheckBox("底图")
        self.chk_bg.setChecked(True)
        self.chk_inflated = QCheckBox("膨胀区(不可通行)")
        self.chk_inflated.setChecked(True)
        self.chk_obstacles = QCheckBox("障碍")
        self.chk_obstacles.setChecked(True)
        self.chk_zones = QCheckBox("参考区域")
        self.chk_zones.setChecked(True)
        self.chk_searched = QCheckBox("A* 搜索痕迹")
        self.chk_searched.setChecked(False)
        self.chk_los = QCheckBox("LOS 关键点")
        self.chk_smooth = QCheckBox("平滑曲线")
        self.chk_smooth.setChecked(True)
        self.chk_table = QCheckBox("最终点表")
        self.chk_table.setChecked(True)
        self.chk_speed_color = QCheckBox("按速度着色")
        self.chk_speed_color.setChecked(True)
        checks = [self.chk_bg, self.chk_inflated, self.chk_obstacles, self.chk_zones,
                  self.chk_searched, self.chk_los, self.chk_smooth, self.chk_table,
                  self.chk_speed_color]
        for c in checks:
            layout.addWidget(c)
            c.stateChanged.connect(self._sync_canvas_flags)

        layout.addWidget(QLabel("<b>障碍清单</b>"))
        self.obs_list = QListWidget()
        self.obs_list.setMaximumHeight(220)
        self.obs_list.itemChanged.connect(self._on_obstacle_item_changed)
        self.obs_list.currentRowChanged.connect(self._on_obstacle_selected)
        layout.addWidget(self.obs_list)

        row = QHBoxLayout()
        btn_del = QPushButton("删除选中")
        btn_del.clicked.connect(self._delete_selected_obstacle)
        btn_clear = QPushButton("清空手动")
        btn_clear.clicked.connect(self._clear_manual_obstacles)
        row.addWidget(btn_del)
        row.addWidget(btn_clear)
        layout.addLayout(row)

        hint = QLabel("滚轮缩放 / 中键平移 / F 适应\n"
                      "多边形: 依次点击, 回车或双击闭合\n"
                      "Del 删除选中障碍")
        hint.setStyleSheet("color:#9aa0aa; font-size:11px;")
        layout.addWidget(hint)
        layout.addStretch(1)
        return panel

    def _build_right_panel(self) -> QWidget:
        tabs = QTabWidget()
        tabs.setMinimumWidth(360)

        # ---------------- 地图 / 标定 ----------------
        page = QWidget()
        form = QFormLayout(page)
        self.spin_res = _dspin(1.0, 500.0, 25.0, 1.0, " mm", 1)
        self.spin_robot_r = _dspin(50.0, 2000.0, 350.0, 10.0, " mm", 0)
        self.spin_margin = _dspin(0.0, 1000.0, 50.0, 5.0, " mm", 0)
        form.addRow("栅格分辨率", self.spin_res)
        form.addRow("车体外接半径", self.spin_robot_r)
        form.addRow("安全余量", self.spin_margin)
        self.spin_robot_r.setToolTip("须包住底盘、机械臂和载荷的全部投影；700×700mm方形的外接圆半径约495mm。默认300mm不代表实车尺寸。")
        self.combo_climb=QComboBox()
        self.combo_climb.addItem("仅地面（暂不处理登高）","ground")
        self.combo_climb.setEnabled(False)
        form.addRow("通行范围",self.combo_climb)
        self.combo_team=QComboBox()
        for label,value in (("按起点推断","auto"),("红队","red"),("蓝队","blue")):
            self.combo_team.addItem(label,value)
        form.addRow("所属队伍",self.combo_team)
        self.combo_climb.currentIndexChanged.connect(self._refresh_code_preview)
        self.combo_team.currentIndexChanged.connect(self._refresh_code_preview)
        self.spin_robot_r.valueChanged.connect(self._refresh_code_preview)
        self.spin_margin.valueChanged.connect(self._refresh_code_preview)

        self.spin_bg_opacity = _dspin(0.05, 1.0, 0.55, 0.05, "", 2)
        self.spin_bg_opacity.valueChanged.connect(
            lambda v: (setattr(self.canvas.state, "background_opacity", v),
                       self.canvas.update()))
        form.addRow("底图不透明度", self.spin_bg_opacity)
        self.lbl_image = QLabel("未导入底图")
        self.lbl_image.setWordWrap(True)
        self.lbl_image.setStyleSheet("color:#9aa0aa; font-size:11px;")
        form.addRow("底图", self.lbl_image)

        grp = QGroupBox("地图标定(2 点)")
        gl = QFormLayout(grp)
        self.spin_ref_x = _dspin(-20000, 20000, 0.0, 100.0, " mm", 0)
        self.spin_ref_y = _dspin(-20000, 20000, 0.0, 100.0, " mm", 0)
        self.spin_dir_dist = _dspin(1.0, 20000.0, 1000.0, 100.0, " mm", 0)
        gl.addRow("参考点场地 X", self.spin_ref_x)
        gl.addRow("参考点场地 Y", self.spin_ref_y)
        gl.addRow("参考点->方向点距离", self.spin_dir_dist)
        self.btn_calib_apply = QPushButton("应用标定")
        self.btn_calib_apply.clicked.connect(self._apply_calibration)
        gl.addRow(self.btn_calib_apply)
        self.btn_calib_clear = QPushButton("清除标定点")
        self.btn_calib_clear.clicked.connect(self._clear_calibration)
        gl.addRow(self.btn_calib_clear)
        self.lbl_calib = QLabel("未标定")
        self.lbl_calib.setStyleSheet("color:#9aa0aa; font-size:11px;")
        gl.addRow(self.lbl_calib)

        v = QVBoxLayout()
        v.addLayout(form)
        v.addWidget(grp)
        v.addWidget(QLabel("提示: 先把工具切到“地图标定”,\n"
                           "在图上点参考点, 再点一个已知距离的方向点,\n"
                           "填好表格里的三个数值后点“应用标定”。"))
        v.addStretch(1)
        page.setLayout(v)
        tabs.addTab(_scroll(page), "地图/标定")

        # ---------------- 规划参数 ----------------
        page2 = QWidget()
        f2 = QFormLayout(page2)
        self.spin_goal_yaw = _dspin(-360.0, 360.0, 0.0, 5.0, " deg", 1)
        self.spin_start_yaw = _dspin(-360.0, 360.0, 0.0, 5.0, " deg", 1)
        self.spin_start_yaw.valueChanged.connect(self._on_yaw_changed)
        self.spin_goal_yaw.valueChanged.connect(self._on_yaw_changed)
        f2.addRow("起点航向", self.spin_start_yaw)
        f2.addRow("终点航向", self.spin_goal_yaw)

        self.chk_spline = QCheckBox("启用 B 样条平滑")
        self.chk_spline.setChecked(True)
        self.chk_los_enable = QCheckBox("启用视线法简化")
        self.chk_los_enable.setChecked(True)
        self.chk_prune = QCheckBox("精简直线段冗余点")
        self.chk_prune.setChecked(True)
        f2.addRow(self.chk_los_enable)
        f2.addRow(self.chk_spline)
        f2.addRow(self.chk_prune)

        self.spin_min_radius = _dspin(50.0, 10000.0, 800.0, 50.0, " mm", 0)
        self.spin_spline_step = _dspin(1.0, 100.0, 5.0, 1.0, " mm", 1)
        self.spin_gap = _dspin(5.0, 1000.0, 100.0, 5.0, " mm", 1)
        self.spin_max_points = QSpinBox()
        self.spin_max_points.setRange(2, 4000)
        self.spin_max_points.setValue(512)
        f2.addRow("最小转弯半径", self.spin_min_radius)
        f2.addRow("采样步长", self.spin_spline_step)
        f2.addRow("点间距", self.spin_gap)
        f2.addRow("点数上限", self.spin_max_points)

        self.combo_heading = QComboBox()
        self.combo_heading.addItem("切向航向(给纯跟踪做前馈)", "tangent")
        self.combo_heading.addItem("起止线性插值(同参考工程 B 样条)", "linear")
        f2.addRow("航向模式", self.combo_heading)

        self.chk_tower = QCheckBox("把塔顶阵列算作障碍")
        self.chk_stack = QCheckBox("把储存区料堆算作障碍")
        f2.addRow(self.chk_tower)
        f2.addRow(self.chk_stack)
        tabs.addTab(_scroll(page2), "规划")

        # ---------------- 速度参数 ----------------
        page3 = QWidget()
        f3 = QFormLayout(page3)
        self.spin_vmax = _dspin(0.05, 6.0, 1.5, 0.1, " m/s", 2)
        self.spin_amax = _dspin(0.1, 12.0, 1.5, 0.1, " m/s2", 2)
        self.spin_jmax = _dspin(0.0, 400.0, 25.0, 5.0, " m/s3", 1)
        self.spin_alat = _dspin(0.05, 12.0, 1.2, 0.1, " m/s2", 2)
        self.spin_vmin = _dspin(0.0, 1.0, 0.06, 0.01, " m/s", 2)
        f3.addRow("最大速度 vmax", self.spin_vmax)
        f3.addRow("最大加速度 amax", self.spin_amax)
        f3.addRow("加加速度 jmax(参考)", self.spin_jmax)
        f3.addRow("横向加速度上限", self.spin_alat)
        f3.addRow("最小驱动速度", self.spin_vmin)
        note = QLabel("注: 点表存的是“这个位置最多能开多快”, 与时间无关;\n"
                      "jerk 的最终整形放在板端控制器(与参考工程分工一致)。\n"
                      "横向加速度上限决定过弯限速 v=sqrt(alat/|kappa|)。")
        note.setStyleSheet("color:#9aa0aa; font-size:11px;")
        note.setWordWrap(True)
        f3.addRow(note)
        tabs.addTab(_scroll(page3), "速度")

        # ---------------- 导出 ----------------
        page4 = QWidget()
        f4 = QFormLayout(page4)
        self.edit_name = _line("TR_route_01")
        f4.addRow("数组名", self.edit_name)
        self.combo_style = QComboBox()
        self.combo_style.addItem("float[N][5] (path_data.c 风格)", STYLE_FLOAT_ARRAY)
        self.combo_style.addItem("PathFollower::PathPoint (2026R2_conbat 风格)", STYLE_PATH_POINT)
        self.combo_style.addItem("TR_PathPoint_t (纯 C 结构体)", STYLE_C_STRUCT)
        self.combo_style.currentIndexChanged.connect(self._refresh_code_preview)
        f4.addRow("导出风格", self.combo_style)

        self.radio_origin_start = QRadioButton("以起点为点表原点 (0,0)")
        self.radio_origin_custom = QRadioButton("自定义点表原点")
        self.radio_origin_start.setChecked(True)
        self.radio_origin_start.toggled.connect(self._refresh_code_preview)
        self.radio_origin_custom.toggled.connect(self._refresh_code_preview)
        f4.addRow(self.radio_origin_start)
        f4.addRow(self.radio_origin_custom)
        self.spin_ox = _dspin(-20000, 20000, 0.0, 100.0, " mm", 0)
        self.spin_oy = _dspin(-20000, 20000, 0.0, 100.0, " mm", 0)
        self.spin_ox.valueChanged.connect(self._refresh_code_preview)
        self.spin_oy.valueChanged.connect(self._refresh_code_preview)
        f4.addRow("原点 X", self.spin_ox)
        f4.addRow("原点 Y", self.spin_oy)

        btn_row = QHBoxLayout()
        b1 = QPushButton("导出 .c 文件")
        b1.clicked.connect(lambda: self.on_export(kind="c"))
        b2 = QPushButton("导出 .csv")
        b2.clicked.connect(lambda: self.on_export(kind="csv"))
        btn_row.addWidget(b1)
        btn_row.addWidget(b2)
        f4.addRow(btn_row)

        self.btn_plan2 = QPushButton("开始规划 (F5)")
        self.btn_plan2.setStyleSheet("font-weight:bold; padding:6px;")
        self.btn_plan2.clicked.connect(self.on_plan)
        f4.addRow(self.btn_plan2)

        self.lbl_summary = QLabel("-")
        self.lbl_summary.setWordWrap(True)
        self.lbl_summary.setStyleSheet("font-size:11px;")
        f4.addRow(self.lbl_summary)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(400)
        self.log.setStyleSheet("font-family: Consolas, monospace; font-size: 11px;")
        self.log.setMinimumHeight(150)
        f4.addRow(self.log)
        tabs.addTab(_scroll(page4), "导出")

        self.tabs = tabs
        return tabs

    # ================================================================== #
    # 状态同步
    # ================================================================== #
    def _sync_canvas_flags(self) -> None:
        st = self.canvas.state
        st.show_background = self.chk_bg.isChecked()
        st.show_inflated = self.chk_inflated.isChecked()
        st.show_obstacles = self.chk_obstacles.isChecked()
        st.show_zones = self.chk_zones.isChecked()
        st.show_searched = self.chk_searched.isChecked()
        st.show_los = self.chk_los.isChecked()
        st.show_smooth = self.chk_smooth.isChecked()
        st.show_table = self.chk_table.isChecked()
        st.show_colors_by_speed = self.chk_speed_color.isChecked()
        self.canvas.update()

    def _load_scene_into_ui(self) -> None:
        self.traj=None
        self.canvas.state.route_points=[]
        self.canvas.state.los_points=[]
        self.canvas.state.smooth_points=[]
        self._refresh_code_preview()
        self.spin_res.setValue(self.scene.resolution)
        self.spin_robot_r.setValue(self.scene.robot_radius)
        self.spin_margin.setValue(self.scene.safety_margin)
        self.canvas.state.scene = self.scene
        self.canvas.state.ground_only=True
        self.canvas.fit_to_extent(self.scene.extent)
        self.canvas.update()

    def _config(self) -> PipelineConfig:
        origin = ((self.spin_ox.value(), self.spin_oy.value())
                  if self.radio_origin_custom.isChecked() else (0.0, 0.0))
        return PipelineConfig(
            route_name=self.edit_name.text().strip() or "TR_route_01",
            resolution_mm=self.spin_res.value(),
            robot_radius_mm=self.spin_robot_r.value(),
            safety_margin_mm=self.spin_margin.value(),
            use_spline=self.chk_spline.isChecked(),
            simplify_los=self.chk_los_enable.isChecked(),
            spline_step_mm=self.spin_spline_step.value(),
            min_radius_mm=self.spin_min_radius.value(),
            point_gap_mm=self.spin_gap.value(),
            max_points=self.spin_max_points.value(),
            heading_mode=self.combo_heading.currentData(),
            prune_collinear=self.chk_prune.isChecked(),
            v_max_mps=self.spin_vmax.value(),
            a_max_mps2=self.spin_amax.value(),
            j_max_mps3=self.spin_jmax.value(),
            a_lat_max_mps2=self.spin_alat.value(),
            v_min_mps=self.spin_vmin.value(),
            include_tower_tops=self.chk_tower.isChecked(),
            include_stacks=self.chk_stack.isChecked(),
            keep_searched=True,
            origin_mm=origin,
            climb_mode=self.combo_climb.currentData(),
            team=self.combo_team.currentData(),
        )

    # ================================================================== #
    # 画布回调
    # ================================================================== #
    def _on_point_changed(self, which: str, x: float, y: float) -> None:
        self.status.showMessage(f"{'起点' if which == 'start' else '终点'}: "
                                f"({x:.0f}, {y:.0f}) mm", 3000)

    def _on_cursor_moved(self, x: float, y: float) -> None:
        self.lbl_cursor.setText(f"x={x:.1f}  y={y:.1f} mm")

    def _on_yaw_changed(self) -> None:
        self.canvas.state.start_yaw_deg = self.spin_start_yaw.value()
        self.canvas.state.goal_yaw_deg = self.spin_goal_yaw.value()
        self.canvas.update()

    def _on_scene_edited(self, msg: str) -> None:
        self._log(msg)
        self._refresh_obstacle_list()
        self._refresh_code_preview()

    def _on_calib_picked(self, which: str, px: float, py: float) -> None:
        px /= self._image_scale
        py /= self._image_scale
        if which == "ref":
            self.canvas.state.calib_ref = (px, py)
            self._log(f"标定参考点(图片像素): ({px:.1f}, {py:.1f})")
        else:
            self.canvas.state.calib_dir = (px, py)
            self._log(f"标定方向点(图片像素): ({px:.1f}, {py:.1f})")
        self.canvas.update()

    def _apply_calibration(self) -> None:
        ref = self.canvas.state.calib_ref
        dr = self.canvas.state.calib_dir
        if ref is None or dr is None:
            QMessageBox.warning(self, "标定", "请先在图上点出参考点和方向点。")
            return
        try:
            self.frame.calibrate_full(ref, (self.spin_ref_x.value(), self.spin_ref_y.value()),
                                      dr, self.spin_dir_dist.value())
        except Exception as exc:
            QMessageBox.warning(self, "标定失败", str(exc))
            return
        if self._image is not None:
            self.frame.image_width = self._image.width()
            self.frame.image_height = self._image.height()
            ext = self.frame.field_extent_from_image()
            self._log(f"标定完成: {self.frame.scale_mm_per_px:.3f} mm/px, "
                      f"旋转 {math.degrees(self.frame.theta_rad):.2f} deg, "
                      f"底图覆盖 x[{ext[0]:.0f},{ext[2]:.0f}] y[{ext[1]:.0f},{ext[3]:.0f}]mm")
            self.canvas.fit_to_extent(ext)
        self.lbl_calib.setText(f"scale={self.frame.scale_mm_per_px:.3f} mm/px, "
                               f"theta={math.degrees(self.frame.theta_rad):.2f}°")
        self.canvas.state.frame = self.frame
        self.canvas.update()

    def _clear_calibration(self) -> None:
        self.canvas.state.calib_ref = None
        self.canvas.state.calib_dir = None
        self.lbl_calib.setText("未标定")
        self.canvas.update()

    # ================================================================== #
    # 障碍清单
    # ================================================================== #
    def _refresh_obstacle_list(self) -> None:
        self.obs_list.blockSignals(True)
        self.obs_list.clear()
        for idx, item in enumerate(self.scene.obstacles):
            if not isinstance(item, dict):
                continue
            name = str(item.get("name", f"#{idx}"))
            tag = str(item.get("tag", ""))
            it = QListWidgetItem(f"{idx:>3}  {name}   [{tag}]" if tag else f"{idx:>3}  {name}")
            it.setFlags(it.flags() | Qt.ItemIsUserCheckable)
            it.setCheckState(Qt.Checked if item.get("enabled", True) is not False
                             else Qt.Unchecked)
            it.setData(Qt.UserRole, idx)
            self.obs_list.addItem(it)
        self.obs_list.blockSignals(False)

    def _on_obstacle_item_changed(self, item: QListWidgetItem) -> None:
        idx = item.data(Qt.UserRole)
        if idx is None or not (0 <= idx < len(self.scene.obstacles)):
            return
        self.scene.obstacles[idx]["enabled"] = (item.checkState() == Qt.Checked)
        self.canvas.update()

    def _on_obstacle_selected(self, row: int) -> None:
        item = self.obs_list.item(row)
        idx = item.data(Qt.UserRole) if item is not None else None
        self.canvas._selected_index = idx
        self.canvas.update()

    def _delete_selected_obstacle(self) -> None:
        row = self.obs_list.currentRow()
        item = self.obs_list.item(row)
        if item is None:
            return
        idx = item.data(Qt.UserRole)
        if 0 <= idx < len(self.scene.obstacles):
            name = self.scene.obstacles[idx].get("name", "")
            self.scene.obstacles.pop(idx)
            self.canvas._selected_index = None
            self._refresh_obstacle_list()
            self.canvas.update()
            self._log(f"删除障碍: {name}")

    def _clear_manual_obstacles(self) -> None:
        before = len(self.scene.obstacles)
        self.scene.obstacles = [o for o in self.scene.obstacles
                                if str(o.get("tag", "")) != "manual"]
        self._refresh_obstacle_list()
        self.canvas.update()
        self._log(f"清除了 {before - len(self.scene.obstacles)} 个手动障碍")

    # ================================================================== #
    # 文件
    # ================================================================== #
    def on_import_map(self) -> None:
        start_dir = os.path.dirname(self._image_path) if self._image_path else ""
        path, _ = QFileDialog.getOpenFileName(
            self, "导入场地俯视图", start_dir,
            "图片 (*.png *.jpg *.jpeg *.bmp *.webp *.gif *.tif *.tiff);;所有文件 (*)")
        if not path:
            self._log("导入已取消")
            return
        try:
            img = QImage()
            if not img.load(path):
                raise RuntimeError("Qt 无法解码这张图(格式不支持?)")
            if img.isNull() or img.width() <= 0 or img.height() <= 0:
                raise RuntimeError(f"读到的图片尺寸异常: {img.width()}x{img.height()}")

            raw_w, raw_h = img.width(), img.height()
            max_side = max(raw_w, raw_h)
            if max_side > 1600:
                self._image_scale = 1600.0 / max_side
                img = img.scaled(max(1, int(raw_w * self._image_scale)),
                                 max(1, int(raw_h * self._image_scale)),
                                 Qt.KeepAspectRatio, Qt.SmoothTransformation)
            else:
                self._image_scale = 1.0
            if img.isNull():
                raise RuntimeError("缩放后图片为空")

            self._image = img
            self._image_path = path

            # 给新图一个"居中"的初始变换 —— 否则图会落在视口外, 看着就像没导进来
            self.frame = FieldFrame.default_for_image(img.width(), img.height(),
                                                      target_size_mm=self.scene.field_size)
            self.canvas.state.background = img
            self.canvas.state.frame = self.frame
            self.canvas.state.calib_ref = None
            self.canvas.state.calib_dir = None
            self.lbl_calib.setText("未标定")

            if not self.chk_bg.isChecked():
                self.chk_bg.setChecked(True)
            self._sync_canvas_flags()

            # 自动把视图适配到这张底图, 保证导入后立刻看得见
            ext = self.frame.field_extent_from_image()
            self.canvas.fit_to_extent(ext)
            self.canvas.update()

            self.lbl_image.setText(
                f"底图: {os.path.basename(path)}\n"
                f"{raw_w}x{raw_h}px"
                + (f" → 缩放 {img.width()}x{img.height()}px"
                   if self._image_scale != 1.0 else ""))
            self._log(f"导入底图成功: {os.path.basename(path)}  {raw_w}x{raw_h}px"
                      f"{' (已缩放显示)' if self._image_scale != 1.0 else ''}")
            self._log(f"底图初始范围 x[{ext[0]:.0f},{ext[2]:.0f}] "
                      f"y[{ext[1]:.0f},{ext[3]:.0f}]mm, "
                      f"{self.frame.scale_mm_per_px:.3f} mm/px (未标定, 仅用于显示)")
            self._log("下一步: 工具切到「地图标定」, 在图上点参考点与方向点, "
                      "填好右侧三个数值后点「应用标定」。")
            QMessageBox.information(
                self, "导入成功",
                f"已导入: {os.path.basename(path)}\n原始 {raw_w}x{raw_h}px\n\n"
                "现在只是把图显示出来, 坐标还没对齐。\n"
                "请把左侧工具切到「地图标定」, 在图上点两个已知点完成标定。")
        except Exception as exc:
            self._log(f"导入失败: {exc}")
            QMessageBox.warning(self, "导入失败",
                                f"无法导入这张图片:\n{path}\n\n{exc}\n\n"
                                "可以试试: 另存为 png, 或换一张图。")

    def on_load_scene(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "载入场景", "", "场景 JSON (*.json)")
        if not path:
            return
        try:
            self.scene = Scene.load(path)
        except Exception as exc:
            QMessageBox.warning(self, "载入失败", str(exc))
            return
        self._load_scene_into_ui()
        self._refresh_obstacle_list()
        bg = self.scene.background or {}
        if bg.get("image_path") and os.path.isfile(bg["image_path"]):
            self.frame = FieldFrame.from_dict(bg)
            self.canvas.state.frame = self.frame
            img = QImage(bg["image_path"])
            if not img.isNull():
                self._image = img
                self.canvas.state.background = img
                self._log(f"已恢复底图: {bg['image_path']}")
        self._log(f"已载入场景: {path}")

    def on_save_scene(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "保存场景", "scene.json", "场景 JSON (*.json)")
        if not path:
            return
        generated=os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))),"scenes","nvwa_butian_2027_from_sim.json")
        if os.path.normcase(os.path.realpath(path))==os.path.normcase(os.path.realpath(generated)):
            QMessageBox.warning(self,"场地由仿真生成","请用 build_template_from_sim.py --write 更新源场地；可另存为副本。")
            return
        self.scene.resolution = self.spin_res.value()
        self.scene.robot_radius = self.spin_robot_r.value()
        self.scene.safety_margin = self.spin_margin.value()
        self.scene.background = self.frame.to_dict()
        self.scene.background["image_path"] = self._image_path
        self.scene.save(path)
        self._log(f"场景已保存: {path}")

    def on_default_scene(self) -> None:
        self.scene = build_default_scene()
        self._load_scene_into_ui()
        self._refresh_obstacle_list()
        self._log("已恢复内置女娲补天场地模板")

    # ================================================================== #
    # 规划
    # ================================================================== #
    def on_plan(self) -> None:
        if self._thread is not None:
            self._log("上一次规划还在跑, 请稍等")
            return
        st = self.canvas.state
        if st.start is None or st.goal is None:
            QMessageBox.information(self, "规划", "请先在图上点出起点和终点(工具选“起终点”)。")
            return

        self.scene.resolution = self.spin_res.value()
        self.scene.robot_radius = self.spin_robot_r.value()
        self.scene.safety_margin = self.spin_margin.value()

        cfg = self._config()
        start = Pose(st.start.x, st.start.y, math.radians(self.spin_start_yaw.value()))
        goal = Pose(st.goal.x, st.goal.y, math.radians(self.spin_goal_yaw.value()))

        # 地图先建好, 主线程里画出来(建图很快)
        try:
            self.costmap = build_map(self.scene, cfg)
        except Exception as exc:
            QMessageBox.warning(self, "建图失败", str(exc))
            return
        self.canvas.costmap = self.costmap
        self.canvas.state.grid_spec = self.costmap.spec
        self.canvas.update()
        self._log(f"栅格 {self.costmap.spec.width}x{self.costmap.spec.height} @ "
                  f"{cfg.resolution_mm:.0f}mm, 可行 {self.costmap.free_cell_count()} 格, "
                  f"阻塞半径 {self.costmap.blocked_radius:.0f}mm")

        self.btn_plan2.setEnabled(False)
        self.btn_plan2.setText("规划中...")
        self._thread = QThread(self)
        self._worker = PlanWorker(self.scene, start, goal, cfg)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.progress.connect(self._log)
        self._worker.finished.connect(self._on_plan_done)
        self._worker.failed.connect(self._on_plan_failed)
        self._thread.start()

    @Slot(object)
    def _on_plan_done(self, traj: Trajectory) -> None:
        self._stop_thread()
        self.traj = traj
        st = self.canvas.state
        st.route_points = [(p.x, p.y) for p in traj.points]
        st.route_speeds = [math.hypot(p.vx, p.vy) for p in traj.points]
        st.searched_cells = list(traj.searched_cells)
        st.los_points = list(traj.los_points)
        st.smooth_points = list(traj.smooth_points)
        self.canvas.update()

        for msg in traj.messages:
            self._log(msg)

        if traj.count < 2:
            self.lbl_summary.setText("规划失败, 请看日志")
            self.lbl_info.setText("规划失败")
            self._refresh_code_preview()
            return

        m = traj.meta
        self.lbl_summary.setText(
            f"<b>{traj.count} 点</b>, 长度 {m.length_mm / 1000.0:.3f} m<br>"
            f"预计耗时 {m.total_time_s:.2f} s, 峰值 {m.peak_v_mm_s:.0f} mm/s<br>"
            f"最小转弯半径 {m.min_radius_mm:.0f} mm<br>"
            f"离障 {m.min_clearance_mm:.0f} mm<br>"
            f"A* 展开 {m.astar_expanded} 格, 关键点 {m.los_points} 个"
        )
        if traj.navigation_map is not None:
            states=list(dict.fromkeys(s for s,z in traj.support_samples))
            names={"ground":"地面","ramp_red":"红坡道","ramp_blue":"蓝坡道",
                   "stairs_red":"红阶梯","stairs_blue":"蓝阶梯",
                   "transfer_red":"红传递区","transfer_blue":"蓝传递区"}
            extra="<br>扫掠校验通过："+" → ".join(names[s] for s in states)
            if traj.navigation_map.climb_mode=="ground":
                extra+="<br>仅地面；坡道/阶梯/高平台封闭"
            self.lbl_summary.setText(self.lbl_summary.text()+extra)
        self.lbl_info.setText(f"{traj.count} 点 / {m.length_mm / 1000.0:.2f} m / "
                              f"{m.total_time_s:.1f} s")
        self._update_plots()
        self._refresh_code_preview()
        self._log(f"规划完成: {traj.count} 点, {m.length_mm / 1000.0:.3f} m, "
                  f"{m.total_time_s:.2f} s")

    @Slot(str)
    def _on_plan_failed(self, msg: str) -> None:
        self._stop_thread()
        self.traj=None
        self.canvas.state.route_points=[]
        self._refresh_code_preview()
        self._log("规划异常: " + msg)
        QMessageBox.warning(self, "规划异常", msg.splitlines()[0] if msg else "未知错误")

    def _stop_thread(self) -> None:
        if self._thread is not None:
            self._thread.quit()
            self._thread.wait(2000)
            self._thread = None
        self._worker = None
        self.btn_plan2.setEnabled(True)
        self.btn_plan2.setText("开始规划 (F5)")

    # ================================================================== #
    # 曲线 / 代码预览
    # ================================================================== #
    def _update_plots(self) -> None:
        traj = self.traj
        if traj is None or len(traj.speed_s_mm) < 2:
            return
        from ..speed.profile import trajectory_times
        s = traj.speed_s_mm
        v = traj.speed_v_mm_s
        self.plot_speed.set_data(s, [("v", v, "#ff6b6b")])
        k = np.abs(traj.curvature)
        self.plot_curv.set_data(s[: len(k)], [("|kappa|", k, "#4dd0e1")])
        t = trajectory_times(s, v)
        self.plot_vt.set_data(t, [("v", v, "#ffd166")])

    def _origin_mm(self) -> Tuple[float, float]:
        if self.radio_origin_custom.isChecked():
            return (self.spin_ox.value(), self.spin_oy.value())
        st = self.canvas.state
        if st.start is not None:
            return (st.start.x, st.start.y)
        return (0.0, 0.0)

    def _code_config(self) -> CCodeConfig:
        return CCodeConfig(
            array_name=self.edit_name.text().strip() or "TR_route_01",
            style=self.combo_style.currentData(),
            origin_mm=self._origin_mm(),
        )

    def _refresh_code_preview(self) -> None:
        if self.traj is None or self.traj.count < 2:
            self.code_view.setPlainText("(还没有生成点表, 先点“规划路径”或按 F5)")
            return
        try:
            self._validate_current_traj()
            code = trajectory_to_c_code(self.traj, self._code_config())
        except Exception as exc:
            code = f"生成失败: {exc}"
        self.code_view.setPlainText(code)

    def _validate_current_traj(self) -> None:
        if surface_required(self.scene):
            old=self.traj.navigation_map
            if old is None: raise ValueError("缺少物理通行校验，请重新规划")
            config=self._config()
            current=SurfaceMap(resolve_scene(self.scene,config),team=old.team if config.team=="auto" else config.team,climb_mode=config.climb_mode)
            current.validate(self.traj.points)
            self.traj.navigation_map=current
        elif any(o.get("tag")=="transfer" for o in self.scene.obstacles):
            raise ValueError("旧二维场地缺少爬升端口信息，仅供参考；请载入仿真生成场地再导出。")

    # ================================================================== #
    # 导出
    # ================================================================== #
    def on_export(self, kind: str = "c") -> None:
        if self.traj is None or self.traj.count < 2:
            QMessageBox.information(self, "导出", "还没有生成点表, 先规划一次。")
            return
        name = self.edit_name.text().strip() or "TR_route_01"
        try:
            self._validate_current_traj()
            payload=trajectory_to_csv(self.traj,self._origin_mm()) if kind=="csv" else trajectory_to_c_code(self.traj,self._code_config())
        except ValueError as exc:
            QMessageBox.warning(self,"拒绝导出",str(exc))
            return
        if kind == "csv":
            path, _ = QFileDialog.getSaveFileName(self, "导出 CSV", f"{name}.csv",
                                                  "CSV (*.csv)")
            if not path:
                return
            atomic_text(path,payload,"utf-8-sig")
        else:
            path, _ = QFileDialog.getSaveFileName(self, "导出 C 点表", f"{name}.c",
                                                  "C 源文件 (*.c);;头文件 (*.h)")
            if not path:
                return
            atomic_text(path,payload)
        self._log(f"已导出: {path}")

    # ================================================================== #
    def _log(self, msg: str) -> None:
        self.log.appendPlainText(msg)
        self.status.showMessage(msg, 5000)

    def closeEvent(self, event):  # noqa: N802
        self._stop_thread()
        super().closeEvent(event)


# --------------------------------------------------------------------------- #
def _dspin(lo: float, hi: float, value: float, step: float,
           suffix: str = "", decimals: int = 2) -> QDoubleSpinBox:
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setValue(value)
    s.setSingleStep(step)
    s.setDecimals(decimals)
    if suffix:
        s.setSuffix(suffix)
    s.setKeyboardTracking(False)
    return s


def _line(text: str) -> QLineEdit:
    return QLineEdit(text)


def _scroll(widget: QWidget) -> QScrollArea:
    area = QScrollArea()
    area.setWidgetResizable(True)
    area.setWidget(widget)
    area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    return area
