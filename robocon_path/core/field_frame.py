"""像素 <-> 场地毫米 的相似变换(标定).

用于把导入的场地俯视图(png/jpg)对齐到场地坐标系.

标定只需要两个点:
  1. 参考点   : 图上某个已知点 + 它在场地系里的坐标 (mm)
  2. 方向点   : 从参考点指向"场地 +X 方向"的另一个图上的点

变换是**相似变换**(等比缩放 + 旋转 + 平移), 因此场地系的长度比例保持正确;
y 轴方向由"参考点 -> 方向点"这一对点自然确定, 不需要额外翻转变换.

图片坐标用左上角为原点的像素坐标 (u 向右, v 向下), 与 Qt/QImage 一致.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from .geometry import Point


@dataclass
class FieldFrame:
    # 图片 -> 场地毫米:  P = s * R(theta) * p + T
    scale_mm_per_px: float = 25.0             # 默认按 25mm/px 估
    theta_rad: float = -math.pi / 2.0        # 默认图 +u 对应场地 +X, +v 对应场地 -Y
    tx: float = 0.0
    ty: float = 0.0
    image_width: int = 0
    image_height: int = 0

    # ------------------------------------------------------------------ #
    # 默认 / 重置
    # ------------------------------------------------------------------ #
    @staticmethod
    def default_for_image(width: int, height: int,
                          target_size_mm: float = 11000.0) -> "FieldFrame":
        """给一张新导入的图一个"居中且大小合理"的初始变换.

        ⚠ 一定要居中: 早期版本 scale=1000/55 且 T=(0,0), 一张 900x900 的图会被映射到
        x[0,-16364] y[16364,0] —— 完全落在视口外面。导入底图后画布上什么都看不见,
        看起来就像"导入没成功"。这里改成按 90% 视场居中, 导入后立刻能看到图,
        再去做标定精调。
        """
        if width <= 0 or height <= 0:
            return FieldFrame()
        frame = FieldFrame()
        frame.scale_mm_per_px = (target_size_mm * 0.9) / max(width, height)
        frame.image_width = int(width)
        frame.image_height = int(height)
        frame.center_on_field(0.0, 0.0)
        return frame

    def center_on_field(self, cx: float = 0.0, cy: float = 0.0) -> None:
        """平移, 使图片中心落在场地坐标 (cx, cy)."""
        if self.image_width <= 0 or self.image_height <= 0:
            self.tx, self.ty = cx, cy
            return
        saved = (self.tx, self.ty)
        self.tx = self.ty = 0.0
        mid = self.image_to_field(self.image_width * 0.5, self.image_height * 0.5)
        self.tx = saved[0] + cx - mid[0]
        self.ty = saved[1] + cy - mid[1]

    # ------------------------------------------------------------------ #
    # 变换
    # ------------------------------------------------------------------ #
    def image_to_field(self, u: float, v: float) -> Point:
        c, s = math.cos(self.theta_rad), math.sin(self.theta_rad)
        return (self.scale_mm_per_px * (c * u - s * v) + self.tx,
                self.scale_mm_per_px * (s * u + c * v) + self.ty)

    def field_to_image(self, x: float, y: float) -> Point:
        dx = (x - self.tx) / self.scale_mm_per_px
        dy = (y - self.ty) / self.scale_mm_per_px
        c, s = math.cos(-self.theta_rad), math.sin(-self.theta_rad)
        return (c * dx - s * dy, s * dx + c * dy)

    def field_to_image_scale(self) -> float:
        """1 毫米等于多少像素(用于按毫米画线宽)."""
        if self.scale_mm_per_px <= 1e-12:
            return 1.0
        return 1.0 / self.scale_mm_per_px

    def field_extent_from_image(self) -> Tuple[float, float, float, float]:
        """图片四角映射到场地系后的包围盒 (x_min, y_min, x_max, y_max)."""
        if self.image_width <= 0 or self.image_height <= 0:
            return (-5500.0, -5500.0, 5500.0, 5500.0)
        corners = [
            self.image_to_field(0.0, 0.0),
            self.image_to_field(float(self.image_width), 0.0),
            self.image_to_field(float(self.image_width), float(self.image_height)),
            self.image_to_field(0.0, float(self.image_height)),
        ]
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        return (min(xs), min(ys), max(xs), max(ys))

    # ------------------------------------------------------------------ #
    # 标定
    # ------------------------------------------------------------------ #
    def calibrate_from_two_points(self,
                                  ref_image: Point, ref_field: Point,
                                  dir_image: Point,
                                  field_plus_x: Tuple[float, float] = (1.0, 0.0),
                                  ) -> float:
        """按"参考点 + 方向点"求 scale / theta / T.

        返回算出来的 scale(mm/px); 两点重合会抛 ValueError.
        """
        du = dir_image[0] - ref_image[0]
        dv = dir_image[1] - ref_image[1]
        pix_len = math.hypot(du, dv)
        if pix_len <= 1e-9:
            raise ValueError("参考点与方向点重合, 无法标定")

        # 图上"参考点->方向点"的方向角
        alpha = math.atan2(dv, du)
        # 场地系里 +X 的方向角
        beta = math.atan2(field_plus_x[1], field_plus_x[0])

        self.theta_rad = beta - alpha
        # 缩放由外部的"方向点距离(mm)"决定, 这里只定方向;
        # 没有距离信息时保持原 scale(调用方可以先 set_scale_from_known_distance)
        c, s = math.cos(self.theta_rad), math.sin(self.theta_rad)
        self.tx = ref_field[0] - self.scale_mm_per_px * (c * ref_image[0] - s * ref_image[1])
        self.ty = ref_field[1] - self.scale_mm_per_px * (s * ref_image[0] + c * ref_image[1])
        return self.scale_mm_per_px

    def calibrate_full(self,
                       ref_image: Point, ref_field: Point,
                       dir_image: Point, dir_field_distance_mm: float) -> None:
        """参考点 + 方向点 + 两点间的真实距离(mm) -> 完整标定."""
        if dir_field_distance_mm <= 0.0:
            raise ValueError("方向点距离必须为正")
        du = dir_image[0] - ref_image[0]
        dv = dir_image[1] - ref_image[1]
        pix_len = math.hypot(du, dv)
        if pix_len <= 1e-9:
            raise ValueError("参考点与方向点重合, 无法标定")

        self.scale_mm_per_px = dir_field_distance_mm / pix_len
        self.calibrate_from_two_points(ref_image, ref_field, dir_image)
        return None

    # ------------------------------------------------------------------ #
    # 序列化
    # ------------------------------------------------------------------ #
    def to_dict(self) -> Dict[str, float]:
        return {
            "scale_mm_per_px": self.scale_mm_per_px,
            "theta_rad": self.theta_rad,
            "tx": self.tx,
            "ty": self.ty,
            "image_width": self.image_width,
            "image_height": self.image_height,
        }

    @staticmethod
    def from_dict(data: Optional[Dict[str, float]]) -> "FieldFrame":
        frame = FieldFrame()
        if not data:
            return frame
        frame.scale_mm_per_px = float(data.get("scale_mm_per_px", frame.scale_mm_per_px))
        frame.theta_rad = float(data.get("theta_rad", frame.theta_rad))
        frame.tx = float(data.get("tx", 0.0))
        frame.ty = float(data.get("ty", 0.0))
        frame.image_width = int(data.get("image_width", 0))
        frame.image_height = int(data.get("image_height", 0))
        return frame
