"""ROBOCON 2027 TR 离线路径点表生成器 —— 程序入口.

用法:
    python main.py
"""

from __future__ import annotations

import os
import sys

# 允许 python main.py 直接跑(不用先装包)
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)


def main() -> int:
    try:
        from PySide6.QtWidgets import QApplication
    except Exception:
        print("缺少 PySide6。请先执行:\n"
              "    pip install PySide6-Essentials\n"
              "或双击 requirements_install.bat", file=sys.stderr)
        return 2

    from robocon_path.ui.main_window import MainWindow

    app = QApplication(sys.argv)
    app.setApplicationName("ROBOCON2027 TR Path Planner")
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
