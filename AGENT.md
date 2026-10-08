# AGENT.md — ROBOCON 2027 TR 离线路径点表生成器

> 给 AI 编码助手（Codex / Claude Code / Cursor 等）的项目约定。
> **动手前先读完本文件**，尤其是「不可改动清单」「写文件铁律」和「验收协议」。

---

## 0. 写文件铁律 ⛔（先看这条）

**这个项目曾经被一次失败的写操作毁过文件**：某个助手在写
`scenes/nvwa_butian_2027_template.json` 时，先把原文件截断成 `{}`（3 字节），
然后写内容的那一步被沙箱拒绝 / 请求中断，结果整份 21KB 的场地模板只剩 3 字节，
而且这个目录**不在版本控制下**，差点永久丢失。

所以：

1. **写任何已存在的文件之前，先复制一份备份**，并且在**写成功且校验通过之后**
   才删除备份。不要用"打开原文件 → 覆盖写 → 失败"的流程。
2. **不要用清空/截断原文件的方式做编辑**（`open(w)`、`Set-Content`、
   `> file` 都会先截断）。要么用原地替换（patch/edit），
   要么写临时文件再原子替换。
3. **写之前先确认有写权限**，使用当前项目实际路径，不依赖原开发机盘符。
   本目录是可独立复制的GitHub工程，用户指定的整理和修改范围优先。
4. **每次写完后必须校验**：

   ```bat
   python integrity.py --check                      :: 与基线 sha256 比对, 看有没有文件被改动/截断
   python -c "import json;json.load(open('scenes/nvwa_butian_2027_template.json',encoding='utf-8'))"
   python selftest.py --verify scenes/nvwa_butian_2027_template.json
   ```

   `integrity.py` 与 `integrity.sha256` 就是为此准备的：
   `--check` 能立刻发现"文件被改过/被截断/被新增"；有意改动后用 `--save` 刷新基线。

---

## 1. 这个项目是干什么的

TR（搬运机器人）的**离线路径点表生成器**：导入场地图 → 点起终点 →
A\* 寻路 → 圆弧倒角/B 样条平滑 → 速度规划 → 导出点表文件（`.c` / `.csv`）。

导出的点表 5 列，与两套参考工程兼容：

```
{ Vx[mm/s], Vy[mm/s], X[mm], Y[mm], yaw[rad] }
```

* `Chassis2026_R1_H723/Path_tracking/path_data.c` → `float name[][5]`
* `2026R2_conbat/MDK-ARM/Route_Plan/path.cpp` → `const PathFollower::PathPoint name[]`
* **首末点速度强制为 0**（触发参考工程跟点器的终点精定位）。

## 2. 目录结构

```
main.py             程序入口(PySide6)          selftest.py    端到端自检 + 场地校验
draw_field.py       只画场地俯视图             field_lab.py   场地几何实验台
add_rules_meta.py   给场景 JSON 补写规则依据
robocon_path/
  core/   geometry.py  scene.py  field_frame.py  costmap.py  trajectory.py
  plan/   astar.py  los.py  fillet.py  spline.py  curvature.py
  speed/  profile.py
  export/ c_code.py  csv_export.py
  ui/     main_window.py  canvas.py  chart.py
scenes/ 场景 JSON          out/ 产物
```

## 3. 坐标系（**不可改动**）

* 单位 mm，右手系，`yaw` 逆时针为正。
* 原点 = **场地中心 = 中央隔板中点**，范围 `[-5500, +5500]^2`。
* **+x 向东（蓝队侧）**，红队在 −x 侧；**+y 向北**。
* 曲率符号：`yaw = atan2(dy,dx)`，`kappa = d(yaw)/ds`，**左转为正**，
  于是 `wz = v·kappa` 左转为正。改这一处就要全链路一起改，不要只改一半。

## 4. 场地 JSON 格式

完整 schema 写在 `robocon_path/core/scene.py` 的**模块 docstring** 里，先读它。
要点：

* 顶层：`meta / field_size / extent / resolution / robot_radius / safety_margin /
  obstacles / zones / background`。
* 几何项 4 种 `type`：`rect{cx,cy,w,h}` / `circle{cx,cy,r}` /
  `polygon{points}` / `wall{x1,y1,x2,y2,thickness}`。
* 元字段：`name`（给人看）、`tag`（**语义分类，程序依赖**）、
  `layer`（ground/L1/L2，仅显示）、`enabled`（**通行性开关**）、`color`（仅 zones）。
* `obstacles` = 不可通行/受限；`zones` = 参考区域，**只做可视化，不参与避障**。
* `meta.rules` 块里抄了规则书全部尺寸依据（程序不解析，供人/工具校验）。

### tag 分类（见 `core/scene.py` 的 `*_TAGS` 常量）

| tag | 含义 | enabled 必须 |
|---|---|---|
| `fence` / `divider` / `l1_body` / `pedestal` | 固定障碍 | `true` |
| `ramp_stair` | 坡道/阶梯，**登高结构不是平地** | **`true`** |
| `transfer` | 传递区，水平交接区，TR 可停留 | **`false`** |
| `tower_top` / `stack` | 塔顶、料堆（位置会变） | 默认 `false` |

## 5. 不可改动清单 ⛔

改场地模板（JSON）或 `core/scene.py` 时，下面这些**动了就静默出错**：

1. **坐标约定**（原点 / 轴向 / 单位）—— 动了导出的点表整体偏移或镜像。
2. **任何几何项的 `tag`** —— 动了 `ramp_stair`/`transfer`/`l1_body` 的分类失效。
3. **任何几何项的 `enabled`** —— 轻则路径绕远，**重则把换乘通道堵死，TR 到不了传递区**。
4. **`obstacles` 与 `zones` 的分工** —— 参考区域误放进 `obstacles` 会变成墙。
5. **导出点表的列顺序与单位** —— 板端跟点器按这 5 列解析。

## 6. 规则要点（改场地前必须理解）

| 规则 | 含义 |
|---|---|
| 2.16 | 坡道、阶梯**和它们之间的传递区**是「地面区与 L1 之间的**过渡区域**」，**坡道/阶梯不属于地面区** |
| 3.2.1 / 4.3.1 | TR 只能在地面区运行，**只能通过坡道或阶梯**到达传递区，严禁进入 L1/L2 |
| 3.4.5 | **在坡道或阶梯上的 TR** 可以直接把比赛用品交给 BR |
| 2.15 / 2.17 | L1 6000×6000 居中（高出地面 600）；L2 3000×3000 居中（高出地面 900） |
| 2.7 | 第一公共区 1200×1200 在**南方**，居中于场地围栏与 L1 围栏之间；第二公共区 1000×1000 在**北方**，边线距北边墙内侧面 1000mm |
| 2.3 / 2.6 | 启动区 2×700×700（间隙 100）；储存区 1000×2000（每队专属） |

**推论**：`ramp_stair` 在地面投影中必须不可横穿；TR须经真实地面端爬上去。
传递区600mm高，不存在侧向地面车道。仿真地图的SurfaceMap通过独立支承层开放
坡道/阶梯及顶面平台，地面端入口以外均不可跨入；不能靠改enabled或挪几何实现。

## 7. 验收协议 ✅（**改完必须跑，全 PASS 才算完成**）

```bat
python integrity.py --check                          :: 先看有没有文件被误改/截断
python selftest.py --verify                          :: 校验内置场地
python selftest.py --verify scenes/xxx.json          :: 校验指定场景 JSON
```

`--verify` 会检查三组条件，全部 OK 才打印 `场地校验 PASS`：

1. **8 条典型路线全部连通**且点表 **0 越界**
   （启动区↔传递区↔第一/第二公共区↔储存区↔环绕半场↔北跑道）
2. **坡道 / 阶梯 / L1 台体必须不可通行**（`free=False`）
3. **红蓝传递区必须可停留**（`free=True`）

以上是旧内置模板的二维回归条件，不能证明物理登高合法。
仿真地图需额外运行 `python rules_selftest.py`：整段圆包络扫掠、端口、敌方专属区域、
L1禁入、双侧登高与返回、指定坡道/阶梯、篡改点表/元数据故障注入。

**当前用户范围（2026-10-06）：暂时只走地面，不处理坡道/阶梯登高。**
PipelineConfig默认ground，UI固定仅地面，不开放爬升选项；所有坡道、阶梯、
600mm传递平台均当作地面障碍投影并膨胀，不改JSON任何tag/enabled。
verify_from_sim按地面路线验收，进入高结构的端点必须拒绝；不能保留旧的
“默认必须登高可达”期望。rules_selftest显式auto仅用于保留代码的历史回归。
ground_selftest验证地面绕行、最终线段、圆弧左右转解析切线连续性和拒绝非法导出。

另外还应跑：

```bat
python selftest.py       :: 端到端: 避障/加速度/首末速度/点间距
python uitest.py         :: 界面冒烟(离屏) + 导出三种风格 + 截图
python draw_field.py     :: 出场地俯视图, 肉眼核对
```

**任何一项 FAIL 都不得提交。** 不要为了让 PASS 而放宽阈值——
特别不要去改 `robot_radius` / `safety_margin` / `--verify` 里的判据。
有意改动完成后，用 `python integrity.py --save` 刷新完整性基线。

## 8. 已知的坑（都踩过，别再踩）

| 坑 | 症状 | 正确做法 |
|---|---|---|
| 把坡道/阶梯当平地 | A\* 给出"顺着坡道穿过去"的路径 | 必须 `enabled=true` |
| 把传递区标成障碍 | TR 永远到不了传递区 | 必须 `enabled=false` |
| 误把传递区当地面空地 | 侧向进入600mm平台 | 使用支承通路和真实入口；不能造地面车道 |
| 端点被吸附跨区 | 终点落在隔板上，被吸附到对方半场 | 吸附后必须**重新校验**，仍在障碍里就报错拒绝出表 |
| 曲率为 0 时算限速 | `sqrt(a_lat/0)` 被当成 0，直线只能跑最低速 | 曲率为 0 时必须给 `vmax` |
| 距离变换差一格 | 离障判断整体偏悲观 | `edt` 结果 `+0.5` 格再乘分辨率 |
| 用弧长做 jerk 受限 | 起步被"龟速化" | jerk 是时间导数，离线只做**加速度**可行包络 |

## 9. 与仿真地图的对接（2026-10-06 按源网格复核）

### 9.1 坐标变换

planner 保持 mm、原点=中心、+x东、+y北、红队x<0、yaw逆时针。
仿真为m、红队y>0、蓝队y<0、地面隔板沿x。

```
sim_x = planner_y / 1000
sim_y = -planner_x / 1000
planner_x = -sim_y * 1000
planner_y = sim_x * 1000
sim_yaw = planner_yaw - pi/2
```

planner→sim是−90°旋转，无镜像；逆向是+90°。变换函数不可另写一套。

### 9.2 真实地图来源

build_template_from_sim.py解析OBJ连通组件和SDF基础box，通过sim_bridge转换，
不再为了连通性人为挪动坡道。GUI默认载入生成的from_sim.json，菜单“内置场地模板”
仍可加载旧规则地图。旧规则模板不由生成器修改。

| 项 | planner范围/中心，mm | 来源 |
|---|---|---|
| 红坡道 | x[-4000,-3000], y[-800,2700] | OBJ009 |
| 红三级阶梯 | x[-4000,-3000], y[-2700,-1800] | OBJ010~012露出顶面 |
| 红传递平台 | x[-4000,-3000], y[-1800,-800] | OBJ013的1m顶面；规则2.18 |
| 蓝侧爬升结构 | 上述x取反、y保持相同 | OBJ014~018 |
| L1→L2单级台阶 | x[-1800,-1500]/[1500,1800], y[-500,500] | OBJ019/020；不是传递区 |
| 五色石基座 | (0,4150)，r135 | OBJ000 |
| 中央基座 | (0,0)，r135，z900~1700 | OBJ001 |
| 第一公共区 | (0,-4250)，1200×1200 | OBJ023 |
| 第二公共区 | (0,4150)，1000×1000 | OBJ005中的地面方片 |
| 启动A/B | 红(-5150,-5150)/(-4350,-5150)，蓝x镜像 | OBJ002/003地面色块 |
| 储存区 | 红(-4500,5000)，蓝x镜像，2000×1000 | OBJ002/003地面色块 |

三级台阶露出高度150/300/450；013/018上方600mm的1m平台是传递区，
不是第四个需要绕开的阶梯。原先把019/020当传递区的注释和复核报告结论已纠正。
12个塔顶由红蓝半色网格配对得到；20个可选料堆标记的位置仍为assumed。
四边围栏读取SDF，中心在±5500，不再向内移25mm。

### 9.3 碰撞与支承通路的边界

Gazebo L1/L2已修为实心并补坡道/阶梯，视觉网格不变；MuJoCo统一方案尚未实施。
地图以mesh场地布局为基准，基座采用OBJ的实际位置，不复制Gazebo错位圆柱。
点表器保持坡道/阶梯enabled=true、传递区enabled=false；navigation标注入口与高度。
SurfaceMap把平台地面投影封闭，仅允许沿1000mm宽结构的支承面从两端进入。
圆包络必须包括机械臂/载荷，默认300mm不代表真实机器人；700×700方形需约495mm。
UI当前固定仅地面，支持红/蓝队，依据起点推断队伍也可手动指定。
保留爬升实现供后续开发，但不在应用默认规划/导出中使用，不当作穿越捷径。
最终所有线段和实际导出舍入后的线段都检查扫掠，失败就拒绝出表。
CSV额外记录支承面与规则标称高度；板端五列单位/顺序不变，不提供z控制。
此校验确认端口、投影净空和区域规则；不证明爬阶能力、牵引力、姿态稳定或实车跟踪误差。
旧二维模板仅供回归/参考，UI禁止将缺少爬升语义的比赛场地导出为实车点表。

### 9.4 生成与独立验收

```bat
python build_template_from_sim.py
python build_template_from_sim.py --write
python verify_from_sim.py
python probe_sim_field.py verify
```

生成器先备份、再临时文件校验和原子替换；遇到已有备份会使用日期备份，
不得覆盖他人备份。验证通过后再清理本轮备份。
probe的独立检查自行读OBJ/SDF，不导入生成器/共同解析器；
按真实露出几何比较坡道、三级阶梯和传递平台，并检查fixed enabled与严格镜像。
不要用原先“每段都必须比宽度长”“坡道南端必须北移300mm”的错误假设替代源几何。


## 10. 验证一次改动的标准流程

### 10.1 完整验收（改场地模板 / 改仿真后必跑）

```bat
:: 1) 坐标变换没被改坏
python robocon_path/core/sim_bridge.py --check

:: 2) 新模板(按仿真重建)的场地校验: 12 点位 + 8 路线 + 2 爬升入口
python verify_from_sim.py

:: 3) 【独立复核】重新解析仿真源文件, 再和模板逐项对拍
python probe_sim_field.py verify

:: 4) 旧模板(按规则文字推的)回归
python selftest.py --verify

:: 5) 端到端 + 出图 + 界面
python selftest.py
python draw_field.py scenes/nvwa_butian_2027_from_sim.json
python uitest.py
python rules_selftest.py
python ground_selftest.py

:: 6) 文件完整性(有没有被截断/误改)
python integrity.py --check
```

**六条全 PASS 才算改完。** `probe_sim_field.py verify` 是**独立复核**口：
它不读 `build_template_from_sim.py` 里的常量，而是**重新解析仿真的
mesh / SDF 源文件**，再和模板逐项对拍。已用故障注入验证过它的有效性：

| 注入的错误 | 是否被抓到 |
|---|---|
| 把坡道标成可通行 | ✅ 抓 |
| 通道改回横穿隔板 | ✅ 抓 |
| L1 台体缩放成 5×5 | ✅ 抓 |
| 坡道位置偏移 / L1 enabled=false | ✅ 按源几何与通行性抓 |

放宽松一点：**`integrity.py --check` 只能发现"文件变了"，`probe_sim_field.py verify`
才能发现"改错了"。** 两者都要跑。

### 10.2 改模板的正确姿势

**不要手改 JSON。** 改 `build_template_from_sim.py` 顶部常量 → 重跑 `--write`：

```bat
python build_template_from_sim.py            :: 先看输出对不对
python build_template_from_sim.py --write    :: 确认后再写盘(自动留 .bak)
python verify_from_sim.py                    :: 验收
python probe_sim_field.py verify             :: 独立复核
```

### 10.3 目视核对

```bat
python draw_field.py scenes/nvwa_butian_2027_from_sim.json
```

出的图和规则书/官方图对得上，才算改完。

## 11. GitHub独立副本（2026-10-08）

- README.md是当前使用说明。源码运行不依赖外部仿真目录；已有生成地图随仓库提供。
- 基础验收：`python run_checks.py`，包含地图、地面绕行、UI和导出验证。
- 独立仿真源复核：设置ROBOCON_SIM_ROOT后执行`python run_checks.py --with-sim`。
  仿真资源不随此仓库复制；无资源时必须说“未运行”，不得认为独立复核通过。
- rules_selftest原始地图基准移至tests/fixtures，禁止用当前地图覆盖基准来追求PASS。
- dist里的EXE与out里的运行结果由.gitignore排除。EXE可作为GitHub Release附件。
- 修改后刷新完整性基线前先备份integrity.sha256；不得放宽任何验收阈值。
