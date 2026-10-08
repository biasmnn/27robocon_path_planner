"""界面冒烟测试: 离屏创建窗口, 跑一遍规划 + 导出, 保存界面截图.

用法(在没有显示器的环境下也能跑):
    python uitest.py
"""

from __future__ import annotations

import math
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from PySide6.QtCore import QEventLoop, QTimer         # noqa: E402
from PySide6.QtWidgets import QApplication            # noqa: E402
from PySide6.QtGui import QFont, QFontDatabase         # noqa: E402


def pump(ms: int = 300) -> None:
    loop = QEventLoop()
    QTimer.singleShot(ms, loop.quit)
    loop.exec()


def main() -> int:
    app = QApplication(sys.argv)
    # Windows offscreen Qt has no system font database by default.
    if not QFontDatabase.families():
        font_path = os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts", "msyh.ttc")
        if os.path.isfile(font_path):
            QFontDatabase.addApplicationFont(font_path)
            app.setFont(QFont("Microsoft YaHei", 9))

    from robocon_path.core.geometry import Pose
    from robocon_path.export import trajectory_to_c_code
    from robocon_path.ui.main_window import MainWindow

    win = MainWindow()
    win.resize(1560, 940)
    win.show()
    pump(400)
    print("[1] MainWindow 创建成功")

    # ---- 设置起终点(模拟在画布上点两下) ----
    st = win.canvas.state
    st.start = Pose(-4200.,4300.,0.0)
    st.goal = Pose(-1200.,-4100.,0.0)
    assert win.combo_climb.currentData()=="ground" and not win.combo_climb.isEnabled()
    assert win.scene.meta.name == "nvwa_butian_2027_from_sim", "UI未加载仿真地图"
    win.canvas.update()
    print("[2] 起终点已设置")

    # ---- 规划(走后台线程, 等它结束) ----
    win.on_plan()
    timeout = 0
    while win._thread is not None and timeout < 20000:
        pump(100)
        timeout += 100
    print(f"[3] 规划结束, 用时约 {timeout} ms")

    if win.traj is None or win.traj.count < 2:
        print("!! 规划没有产出点表")
        return 1
    print(f"[4] 点表 {win.traj.count} 点, 长度 {win.traj.meta.length_mm:.1f} mm, "
          f"耗时 {win.traj.meta.total_time_s:.2f} s")
    print(f"    离障 {win.traj.meta.min_clearance_mm:.0f} mm, "
          f"最小转弯半径 {win.traj.meta.min_radius_mm:.0f} mm")
    # Raised platform and ramp sides both need footprint clearance. Their
    # centre support strips remain accessible through the end entrances.
    from PySide6.QtGui import QColor
    from robocon_path.ui.canvas import _mask_to_qimage
    image = _mask_to_qimage(win.costmap.inflated, QColor(255, 110, 110, 90))
    # inflated is the safety band outside the raw obstacle, not its interior.
    for label, x, y, blocked in [("传递平台外侧", -4150, -1300, True),
                                  ("坡道外侧", -4150, 1300, True)]:
        row, col = win.costmap.spec.world_to_cell(x, y)
        alpha = image.pixelColor(col, image.height()-1-row).alpha()
        assert (alpha > 0) == blocked, f"{label}避障层显示与源场地不一致"
    print("[4a] 避障显示方向核对 PASS（真实传递平台与坡道）")
    assert win.traj.navigation_map is not None
    win.traj.navigation_map.validate(win.traj.points)
    states=list(dict.fromkeys(s for s,z in win.traj.support_samples))
    assert states==["ground"],states
    print("[4b] 端口通行/整段扫掠 PASS: "+" → ".join(states))
    # Changing the robot envelope must invalidate a previously valid route.
    from PySide6.QtWidgets import QMessageBox,QFileDialog
    warning,dialog=QMessageBox.warning,QFileDialog.getSaveFileName
    rejected=[]
    try:
        QMessageBox.warning=lambda parent,title,message:rejected.append(message)
        def unexpected_dialog(*args,**kwargs):
            raise AssertionError("未经当前约束校验便进入文件导出")
        QFileDialog.getSaveFileName=unexpected_dialog
        old_radius=win.spin_robot_r.value()
        win.spin_robot_r.setValue(900.)
        assert "生成失败" in win.code_view.toPlainText(),"旧点表预览未按当前约束失效"
        win.on_export(kind="c")
        assert rejected,"修改爬升约束后旧点表未被拒绝"
    finally:
        QMessageBox.warning,QFileDialog.getSaveFileName=warning,dialog
        win.spin_robot_r.setValue(old_radius)
    print("[4c] 修改车体包络后，旧点表预览失效/导出被拒绝 PASS")

    # ---- 检查曲线与代码预览 ----
    txt = win.code_view.toPlainText()
    if "float" not in txt or "][" not in txt:
        print("!! 代码预览异常")
        print(txt[:400])
        return 1
    print(f"[5] 点表代码预览 {len(txt.splitlines())} 行")

    if not win.plot_speed.x:
        print("!! 速度曲线没数据")
        return 1
    print(f"[6] 速度曲线采样 {len(win.plot_speed.x)} 点")

    # ---- 导出三种风格 ----
    out = os.path.join(HERE, "out")
    os.makedirs(out, exist_ok=True)
    from robocon_path.core.file_io import atomic_text,atomic_bytes
    for style, name in (("float_array", "ui_route_array.c"),
                        ("path_point", "ui_route_pathpoint.cpp"),
                        ("c_struct", "ui_route_struct.c")):
        cfg = win._code_config()
        cfg.style = style
        cfg.array_name = "TR_ui_test"
        text = trajectory_to_c_code(win.traj, cfg)
        atomic_text(os.path.join(out,name),text)
        print(f"[7] 导出 {name} ({len(text.splitlines())} 行)")

    from robocon_path.export import trajectory_to_csv
    atomic_text(os.path.join(out,"ui_route.csv"),trajectory_to_csv(win.traj,win._origin_mm()),"utf-8-sig")
    print("[8] 导出 ui_route.csv")

    # ---- 截图 ----
    pump(300)
    shot = win.grab()
    shot_path = os.path.join(out, "ui_screenshot.png")
    from PySide6.QtCore import QBuffer,QIODevice
    def save_png(pixmap,path):
        buffer=QBuffer();buffer.open(QIODevice.WriteOnly)
        assert pixmap.save(buffer,"PNG")
        atomic_bytes(path,bytes(buffer.data()))
    save_png(shot,shot_path)
    print(f"[9] 界面截图: {shot_path} ({shot.width()}x{shot.height()})")

    # ---- 单独截画布, 看清楚路径 ----
    save_png(win.canvas.grab(),os.path.join(out, "ui_canvas.png"))
    print(f"[10] 画布截图: {os.path.join(out, 'ui_canvas.png')}")

    print("\n=== 界面冒烟测试 PASS ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
