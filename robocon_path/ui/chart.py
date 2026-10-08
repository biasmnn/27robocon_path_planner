"""轻量曲线图控件: 画 v-s / kappa-s / v-t, 不依赖 matplotlib.

为什么自己画: 工程要打包成 exe, 少一个依赖少一份麻烦; 而且这里需求很简单
(两条曲线 + 网格 + 数值标注), QPainter 比嵌入 matplotlib 更快更省事。
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget


class CurvePlot(QWidget):
    """一条或多条曲线, 横轴为弧长(或时间)."""

    def __init__(self, title: str = "", x_label: str = "s [mm]",
                 y_label: str = "", parent=None):
        super().__init__(parent)
        self.setMinimumHeight(120)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.title = title
        self.x_label = x_label
        self.y_label = y_label
        self.x: List[float] = []
        self.series: List[Tuple[str, List[float], QColor]] = []

    def set_data(self, x: Sequence[float],
                 series: Sequence[Tuple[str, Sequence[float], str]]) -> None:
        self.x = [float(v) for v in x]
        self.series = [(name, [float(v) for v in vals], QColor(color))
                       for name, vals, color in series]
        self.update()

    # ------------------------------------------------------------------ #
    def paintEvent(self, event):  # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#23262b"))

        margin_l, margin_r, margin_t, margin_b = 58, 12, 22, 26
        plot = QRectF(margin_l, margin_t,
                      max(10.0, self.width() - margin_l - margin_r),
                      max(10.0, self.height() - margin_t - margin_b))

        painter.setPen(QPen(QColor("#cfd4dc")))
        painter.setFont(QFont("Microsoft YaHei", 8))
        painter.drawText(QPointF(8, 14), self.title)

        if not self.x or not self.series:
            painter.setPen(QPen(QColor("#71767f")))
            painter.drawText(plot, Qt.AlignCenter, "暂无数据")
            painter.end()
            return

        x0, x1 = min(self.x), max(self.x)
        if x1 - x0 < 1e-9:
            x1 = x0 + 1.0

        # y 轴范围: 所有序列共用(速度与曲率量级不同, 所以调用方分两个图更清楚)
        y_lo = 0.0
        y_hi = 1e-9
        for _name, vals, _c in self.series:
            if vals:
                y_lo = min(y_lo, min(vals))
                y_hi = max(y_hi, max(vals))
        if y_hi - y_lo < 1e-9:
            y_hi = y_lo + 1.0

        # 网格
        painter.setPen(QPen(QColor("#343941"), 1.0))
        for i in range(5):
            y = plot.top() + plot.height() * i / 4.0
            painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
        for i in range(5):
            x = plot.left() + plot.width() * i / 4.0
            painter.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()))

        # 坐标轴标注
        painter.setPen(QPen(QColor("#9aa0aa")))
        for i in range(5):
            y = plot.top() + plot.height() * i / 4.0
            value = y_hi - (y_hi - y_lo) * i / 4.0
            painter.drawText(QPointF(4, y + 3), _fmt_short(value))
            x = plot.left() + plot.width() * i / 4.0
            xv = x0 + (x1 - x0) * i / 4.0
            painter.drawText(QPointF(x - 16, plot.bottom() + 14), _fmt_short(xv))
        painter.drawText(QPointF(plot.left(), self.height() - 3), self.x_label)

        # 曲线
        n = len(self.x)
        for name, vals, color in self.series:
            if len(vals) != n or n < 2:
                continue
            pts = []
            for i in range(n):
                px = plot.left() + plot.width() * (self.x[i] - x0) / (x1 - x0)
                vv = (vals[i] - y_lo) / (y_hi - y_lo)
                py = plot.bottom() - plot.height() * vv
                pts.append(QPointF(px, py))
            painter.setPen(QPen(color, 1.8))
            painter.drawPolyline(QPolygonF(pts))
            peak = max(vals)
            painter.setPen(QPen(color))
            painter.drawText(QPointF(plot.left() + 4, plot.top() + 12),
                             f"{name} max={_fmt_short(peak)}")

        painter.setPen(QPen(QColor("#4a505a")))
        painter.drawRect(plot)
        painter.end()


def _fmt_short(v: float) -> str:
    av = abs(v)
    if av >= 10000:
        return f"{v / 1000.0:.1f}k"
    if av >= 100:
        return f"{v:.0f}"
    if av >= 1:
        return f"{v:.1f}"
    if av <= 1e-9:
        return "0"
    return f"{v:.2e}"
