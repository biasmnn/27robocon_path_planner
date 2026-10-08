"""ROBOCON 2027 TR 离线路径点表生成器.

模块划分:
    core.geometry   几何图元
    core.scene      场地场景(JSON) 与默认模板
    core.field_frame 像素 <-> 场地毫米 的相似变换(标定)
    core.costmap    栅格化 + 膨胀 + 碰撞查询
    plan.astar      A* 搜索
    plan.los        视线法简化
    plan.spline     B 样条平滑(带曲率约束与避障回退)
    plan.curvature  弧长/切向/曲率
    speed.profile   速度剖面(曲率限速 + 前后向扫描 + jerk 平滑)
    export.csv_export / export.c_code  点表导出
"""

__version__ = "0.1.0"
