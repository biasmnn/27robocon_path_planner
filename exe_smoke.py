"""Exercise the actual frozen UI, planner and save/export actions."""
from pathlib import Path
import csv
import hashlib
import json
import os
import sys
import time
import traceback


def run(output: Path) -> int:
    output.mkdir(parents=True, exist_ok=True)
    from robocon_path.core.file_io import atomic_text, atomic_bytes
    notes = []
    try:
        from PySide6.QtCore import QBuffer, QEventLoop, QIODevice, QTimer
        from PySide6.QtGui import QFont, QFontDatabase
        from PySide6.QtWidgets import QApplication, QFileDialog, QMessageBox
        from robocon_path.core.geometry import Pose
        from robocon_path.core.scene import Scene
        from robocon_path.plan.spline import _fit_one_segment
        from robocon_path.ui.main_window import MainWindow
        import numpy as np

        def pump(ms=100):
            loop = QEventLoop()
            QTimer.singleShot(ms, loop.quit)
            loop.exec()

        app = QApplication(sys.argv[:1])
        if not QFontDatabase.families():
            font = Path(os.environ.get("WINDIR", r"C:\Windows")) / "Fonts/msyh.ttc"
            if font.exists():
                QFontDatabase.addApplicationFont(str(font))
                app.setFont(QFont("Microsoft YaHei", 9))
        win = MainWindow()
        win.show()
        pump(400)
        assert win.scene.meta.name == "nvwa_butian_2027_from_sim", win.scene.meta.name
        assert win.combo_climb.currentData() == "ground" and not win.combo_climb.isEnabled()
        notes.append("PASS 内置仿真地图加载、仅地面界面、Qt窗口创建")
        bundle = Path(getattr(sys, "_MEIPASS", Path(__file__).parent))
        scene_path = bundle / "scenes/nvwa_butian_2027_from_sim.json"
        before = hashlib.sha256(scene_path.read_bytes()).hexdigest()
        win.canvas.state.start = Pose(-4200., 4300., 0.)
        win.canvas.state.goal = Pose(-1200., -4100., 0.)
        win.on_plan()
        deadline = time.monotonic() + 60.
        while win._thread is not None and time.monotonic() < deadline:
            pump()
        assert win._thread is None, "后台规划超时"
        assert win.traj is not None and win.traj.count >= 2, win.log.toPlainText()
        win.traj.navigation_map.validate(win.traj.points)
        assert all(state == "ground" and z == 0 for state, z in win.traj.support_samples)
        assert win.plot_speed.x and "float" in win.code_view.toPlainText()
        notes.append(f"PASS 后台A*、圆弧平滑、速度曲线及预览：{win.traj.count}点，{win.traj.meta.length_mm:.1f}mm，全程ground/z0")
        # Exercise the scipy spline extension even if this route uses fillets.
        fitted = _fit_one_segment(np.array([[0.,0.],[100.,50.],[200.,0.],[300.,0.]]), 0., 30)
        assert fitted is not None and fitted.shape == (30,2) and np.isfinite(fitted).all()
        notes.append("PASS scipy样条拟合及原生扩展加载")
        old_dialog = QFileDialog.getSaveFileName
        old_warning = QMessageBox.warning
        warnings = []
        QMessageBox.warning = lambda parent, title, message: warnings.append(message)
        try:
            for kind, filename in (("c", "路线.c"), ("csv", "路线.csv")):
                target = output / filename
                QFileDialog.getSaveFileName = lambda *args, p=target, **kwargs: (str(p), "")
                win.on_export(kind)
                assert target.exists() and target.stat().st_size > 100, filename
            with (output / "路线.csv").open(encoding="utf-8-sig", newline="") as fh:
                rows = list(csv.reader(fh))
            assert len(rows) > win.traj.count
            target = output / "场景副本.json"
            QFileDialog.getSaveFileName = lambda *args, **kwargs: (str(target), "")
            win.on_save_scene()
            copied = Scene.load(str(target))
            assert copied.obstacles == win.scene.obstacles and copied.zones == win.scene.zones
            assert not warnings, warnings
        finally:
            QFileDialog.getSaveFileName = old_dialog
            QMessageBox.warning = old_warning
        notes.append("PASS 界面C/CSV导出及场景另存：中文路径可写，副本几何保持一致")
        assert hashlib.sha256(scene_path.read_bytes()).hexdigest() == before
        notes.append("PASS 内置源地图未被写入")
        pump(300)
        for pixmap, filename in ((win.grab(), "exe_window.png"), (win.canvas.grab(), "exe_canvas.png")):
            buffer = QBuffer()
            buffer.open(QIODevice.WriteOnly)
            assert pixmap.save(buffer, "PNG")
            atomic_bytes(output / filename, bytes(buffer.data()))
        win.close()
        pump()
        report = dict(status="PASS", frozen=bool(getattr(sys,"frozen",False)), executable=sys.executable,
                      cwd=str(Path.cwd()), scene_sha256=before, checks=notes)
        atomic_text(output / "result.json", json.dumps(report, ensure_ascii=False, indent=2))
        atomic_text(output / "output.txt", "\n".join(notes) + "\n=== EXE SMOKE PASS ===\n")
        return 0
    except Exception:
        atomic_text(output / "output.txt", "\n".join(notes) + "\nFAIL\n" + traceback.format_exc())
        return 1
