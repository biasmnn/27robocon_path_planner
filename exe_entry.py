"""Standalone Windows entry; normal launch delegates to the unchanged app."""
from pathlib import Path
import os
import sys
import traceback


def main():
    if "--smoke-test" in sys.argv:
        index = sys.argv.index("--smoke-test")
        if index + 1 >= len(sys.argv):
            return 2
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from exe_smoke import run
        return run(Path(sys.argv[index + 1]).resolve())
    try:
        from main import main as run_app
        return run_app()
    except Exception:
        message = traceback.format_exc()
        log_dir = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "ROBOCON_TR_Planner"
        log_dir.mkdir(parents=True, exist_ok=True)
        from robocon_path.core.file_io import atomic_text
        log = log_dir / "startup_error.txt"
        atomic_text(log, message)
        from PySide6.QtWidgets import QApplication, QMessageBox
        app = QApplication.instance() or QApplication(sys.argv)
        QMessageBox.critical(None, "程序启动失败", f"启动失败，详细记录：\n{log}\n\n{message[-1200:]}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
