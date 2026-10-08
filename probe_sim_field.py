"""深入解析场地仿真: 色块包围盒 + 关键结构采样 + 两个引擎的碰撞对比.

用途: 把"仿真里场地到底长什么样、碰撞体是否可信"这件事一次性查清楚,
      作为规划工具适配仿真的依据。

用法: python probe_sim_field.py            # 全部检查
      python probe_sim_field.py bbox       # 只看色块包围盒
      python probe_sim_field.py sample     # 采样关键 mesh 的顶点形状
      python probe_sim_field.py collision  # 对比 Gazebo / MuJoCo 碰撞体
"""

from __future__ import annotations

import os
import re
import sys
from typing import Dict, List, Tuple

ROOT = os.path.abspath(os.path.expanduser(os.environ.get(
    "ROBOCON_SIM_ROOT",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                 "27th_gap_gazebo_model", "gazebo_models-main")
)))
MESH_DIR = os.path.join(ROOT, "robocon_mujoco", "meshes")
SDF = os.path.join(ROOT, "robocon_ground", "model.sdf")
MUJOCO_XML = os.path.join(ROOT, "robocon_ground", "mujoco", "robocon_ground.xml")
GAZEBO_WORLD = os.path.join(ROOT, "robocon_track.world")

# 材质 rgab(MuJoCo) 与语义的对照(人工判读, 见 robocon_geoms.xml)
SEMANTIC: Dict[int, str] = {
    0: "五色石基座(圆柱 h=0.5 Φ0.27)",
    1: "中央基座(圆柱 z0.9~1.7 Φ0.27)",
    2: "红色半场地面(x<0)",
    3: "蓝色半场地面(x>0)",
    4: "L1 顶面(红半)薄板",
    5: "L2 台体(实体, 3x3x0.9)",
    6: "场地围栏/隔板(底面薄板)",
    7: "场地围栏/隔板(底面薄板, 另一层)",
    8: "L1 侧面(深绿, 6x6 高0.3?)",
    9: "坡道(红, 高0.597)",
    10: "阶梯第1级(红)",
    11: "阶梯第2级(红)",
    12: "阶梯第3级(红)",
    13: "阶梯第4级(红)",
    14: "坡道(蓝)",
    15: "阶梯第1级(蓝)",
    16: "阶梯第2级(蓝)",
    17: "阶梯第3级(蓝)",
    18: "阶梯第4级(蓝)",
    19: "传递区(红, L1 顶面 +0.15 高差?)",
    20: "传递区(蓝)",
    21: "地面(红半场)薄板 z=0",
    22: "地面(蓝半场)薄板 z=0",
    23: "传递区(黄, 地面 z=0)",
    24: "L1 顶面(红半)y0~3",
    25: "L1 顶面(蓝半)y-3~0",
    26: "小方块(蓝, L1 顶面)",
    27: "隔板顶面(深红, L1 顶面一线)",
    28: "小方块(深红, 地面 z=0.03)",
}


def read_obj(path: str) -> Tuple[List[Tuple[float, float, float]], List[List[int]]]:
    verts: List[Tuple[float, float, float]] = []
    faces: List[List[int]] = []
    with open(path, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            if line.startswith("v "):
                p = line.split()
                if len(p) >= 4:
                    try:
                        verts.append((float(p[1]), float(p[2]), float(p[3])))
                    except ValueError:
                        pass
            elif line.startswith("f "):
                idx = []
                for tok in line.split()[1:]:
                    t = tok.split("/")[0]
                    try:
                        idx.append(int(t))
                    except ValueError:
                        pass
                if len(idx) >= 3:
                    faces.append(idx)
    return verts, faces


def bbox(verts) -> Tuple[List[float], List[float]]:
    lo = [min(v[i] for v in verts) for i in range(3)]
    hi = [max(v[i] for v in verts) for i in range(3)]
    return lo, hi


def cmd_bbox() -> int:
    files = sorted(f for f in os.listdir(MESH_DIR)
                   if re.fullmatch(r"robocon_\d+\.obj", f))
    print(f"{'mesh':<18}{'顶点':>6} {'x范围':>17} {'y范围':>17} {'z范围':>16}  语义")
    print("-" * 118)
    for fn in files:
        idx = int(fn.replace("robocon_", "").replace(".obj", ""))
        verts, faces = read_obj(os.path.join(MESH_DIR, fn))
        if not verts:
            continue
        lo, hi = bbox(verts)
        print(f"{fn:<18}{len(verts):>6} [{lo[0]:6.2f},{hi[0]:6.2f}] "
              f"[{lo[1]:6.2f},{hi[1]:6.2f}] [{lo[2]:6.3f},{hi[2]:6.3f}]  "
              f"{SEMANTIC.get(idx, '')}")
    print("-" * 118)
    print("注: 坐标单位 m; x<0 = 红方半场, x>0 = 蓝方半场; 隔板在 x=0 沿 y 方向")
    return 0


def cmd_sample() -> int:
    """采样关键 mesh 的顶点分布, 判断它是平面片还是立体块."""
    targets = {
        "robocon_009.obj": "坡道(红) 期望从 z0 升到 z0.597",
        "robocon_010.obj": "阶梯1级(红)",
        "robocon_013.obj": "阶梯4级(红)",
        "robocon_019.obj": "传递区(红) 期望是水平面",
        "robocon_023.obj": "传递区(黄) 地面水平面",
        "robocon_005.obj": "L2 台体",
        "robocon_008.obj": "L1 侧面(深绿)",
        "robocon_002.obj": "红色半场地面",
        "robocon_004.obj": "L1 顶面(红半)薄板",
    }
    for fn, desc in targets.items():
        p = os.path.join(MESH_DIR, fn)
        if not os.path.isfile(p):
            continue
        verts, faces = read_obj(p)
        if not verts:
            continue
        lo, hi = bbox(verts)
        print(f"\n=== {fn}  ({desc}) ===")
        print(f"  顶点 {len(verts)}, 面 {len(faces)}")
        print(f"  bbox x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] "
              f"z[{lo[2]:.3f},{hi[2]:.3f}]")
        zs = sorted(set(round(v[2], 4) for v in verts))
        print(f"  z 取值({len(zs)} 个): {zs[:12]}{' ...' if len(zs) > 12 else ''}")
        if len(verts) <= 12:
            for v in sorted(verts, key=lambda t: (t[2], t[0], t[1])):
                print(f"     ({v[0]:7.3f},{v[1]:7.3f},{v[2]:7.4f})")
    return 0


def cmd_collision() -> int:
    print("=" * 100)
    print("Gazebo Classic (model.sdf) 的碰撞体")
    print("=" * 100)
    sdf = open(SDF, encoding="utf-8", errors="ignore").read()
    for m in re.finditer(
            r'<link name="([^"]+)">(.*?)</link>', sdf, re.S):
        name, body = m.group(1), m.group(2)
        geo = re.search(r"<(box|cylinder)>(.*?)</\1>", body, re.S)
        pose = re.search(r"<pose>([^<]+)</pose>", body)
        g = geo.group(1) if geo else "?"
        size = " ".join(geo.group(2).split()) if geo else "?"
        p = " ".join(pose.group(1).split()) if pose else "(默认 0)"
        print(f"  {name:<32} {g:<9} {size:<24} pose={p}")

    print()
    print("=" * 100)
    print("MuJoCo (robocon_ground.xml) 的碰撞体")
    print("=" * 100)
    xml = open(MUJOCO_XML, encoding="utf-8", errors="ignore").read()
    for m in re.finditer(r'<geom name="(collision_[^"]+)"([^/]*)/>', xml):
        name, attrs = m.group(1), m.group(2)
        t = re.search(r'type="(\w+)"', attrs)
        pos = re.search(r'pos="([^"]+)"', attrs)
        size = re.search(r'size="([^"]+)"', attrs)
        print(f"  {name:<32} {t.group(1) if t else '?':<9} "
              f"size={size.group(1) if size else '?':<22} "
              f"pos={pos.group(1) if pos else '(0)'}")

    print()
    print("=" * 100)
    print("差异汇总(关键)")
    print("=" * 100)
    print("""  L1 层:
    Gazebo : box 6x6x0.02, pose z=0.61  ->  只有一块 20mm 薄板悬在 0.61m
             => 地面机器人可以**直接开进 L1 底下**, 与规则"TR 严禁进入 L1"不符!
    MuJoCo : box 6x6x0.602, pos z=0.301 ->  实心台体(0 到 0.602m), 正确

  L2 层:
    Gazebo : box 3x3x0.02, pose z=0.91  ->  同样只是薄板
    MuJoCo : box 3x3x0.299, pos z=0.7515 -> 实心

  坡道/阶梯:
    Gazebo : **完全没有碰撞体**(model.sdf 里没有对应 link)
    MuJoCo : README 称"斜坡、阶梯已有实际碰撞", 但 robocon_ground.xml 里也看不到
             -> 需实测确认

  场地围栏:
    两侧都是 11x0.05x0.08 (高 80mm) —— 这是 2.21 里"场地围栏"的高
    但 L1 区围栏(1.1 说内高 50/厚 50)未见单独建模
""")
    return 0


def cmd_verify_template(scene_path: str) -> int:
    """Independently compare every mapped source footprint, not builder constants."""
    import json
    import xml.etree.ElementTree as E
    if not os.path.isfile(scene_path):
        print(f"找不到场景文件: {scene_path}"); return 1
    with open(scene_path, encoding="utf-8") as f: doc=json.load(f)
    fails=[]
    print(f"独立复核: {os.path.basename(scene_path)} <-> 原始OBJ/SDF")
    def parts(index):
        v,faces=read_obj(os.path.join(MESH_DIR,f"robocon_{index:03}.obj"))
        parent=list(range(len(v)))
        def root(i):
            while parent[i]!=i:
                parent[i]=parent[parent[i]];i=parent[i]
            return i
        for face in faces:
            for i in face[1:]: parent[root(i-1)]=root(face[0]-1)
        groups={}
        for i,p in enumerate(v):groups.setdefault(root(i),[]).append(p)
        return [bbox(g) for g in groups.values()]
    def mesh(index):
        v,_=read_obj(os.path.join(MESH_DIR,f"robocon_{index:03}.obj"))
        return bbox(v)
    sdf=E.parse(SDF)
    def box(link):
        col=sdf.find(f".//link[@name='{link}']/collision")
        size=list(map(float,col.findtext("geometry/box/size").split()))
        pose=list(map(float,col.findtext("pose","0 0 0 0 0 0").split()))
        if any(abs(a)>1e-12 for a in pose[3:]):raise ValueError("Rotated source requires frame resolution")
        return ([pose[k]-size[k]/2 for k in range(3)],[pose[k]+size[k]/2 for k in range(3)])
    def item_bounds(o):
        sx,sy=o["cy"]/1000.,-o["cx"]/1000.
        hx=o.get("h",o.get("r",0)*2)/2000.
        hy=o.get("w",o.get("r",0)*2)/2000.
        return ([sx-hx,sy-hy],[sx+hx,sy+hy])
    def compare(items,name,source,tag,enabled=None,legacy_tol=None):
        found=[o for o in items if o.get("name")==name]
        if len(found)!=1:
            fails.append(f"{name}: 缺失或重名");print(f"BAD {name}: count={len(found)}");return
        o=found[0];a,b=item_bounds(o);c,d=source
        error=max(abs(a[k]-c[k]) for k in (0,1))
        error=max(error,max(abs(b[k]-d[k]) for k in (0,1)))
        # Keep old L1/divider/pedestal tolerances as supplementary checks.
        # Exact source reconstruction is an additional, stricter check.
        ok=error<=1e-8 and o.get("tag")==tag
        if legacy_tol is not None:ok=ok and error<=legacy_tol
        if enabled is not None:ok=ok and o.get("enabled") is enabled
        if not ok:fails.append(f"{name}: 几何/分类/通行性与源文件不符")
        print(f"{'OK' if ok else 'BAD'} {name}: template={a}/{b}, source={c[:2]}/{d[:2]}, error={error*1000:.6f}mm, enabled={o.get('enabled')}")
    obs=doc.get("obstacles",[]);zones=doc.get("zones",[])
    compare(obs,"L1 台体(地面层不可进入)",box("collision_l1"),"l1_body",True,.06)
    compare(obs,"中央隔板",box("collision_center_divider"),"divider",True,.06)
    compare(obs,"五色石基座",mesh(0),"pedestal",True,.15)
    compare(obs,"中央基座",mesh(1),"pedestal",True)
    for label,link in [("北","collision_wall_right"),("南","collision_wall_left"),
                       ("东","collision_wall_front"),("西","collision_wall_back")]:
        compare(obs,f"场地围栏({label})",box(link),"fence",True)
    for side,offset,side_tag in [("红",0,"red"),("蓝",5,"blue")]:
        a,b=mesh(10+offset);c,d=mesh(13+offset)
        sources=[("坡道","ramp_stair","ramp",mesh(9+offset),True),
                 ("阶梯","ramp_stair","stair",(a,[c[0],b[1],mesh(12+offset)[1][2]]),True),
                 ("传递区","transfer","transfer",(c,d),False)]
        for label,tag,ztag,src,on in sources:
            compare(obs,f"{label}({side})",src,tag,on)
            compare(zones,f"{label}({side})",src,f"{ztag}_{side_tag}")
            found=[o for o in obs if o.get("name")==f"{label}({side})"]
            if len(found)==1:
                nav=found[0].get("navigation",{})
                kind={"ramp":"ramp","stair":"stairs","transfer":"transfer"}[ztag]
                # Independent frame derivation: planner y is source sim x.
                end=(src[1][0] if kind=="ramp" else src[0][0])*1000
                ok=(nav.get("kind")==kind and nav.get("side")==side_tag
                    and nav.get("entry_axis")=="y" and nav.get("top_height_mm")==600.)
                if kind!="transfer": ok=ok and nav.get("ground_end_y_mm")==end
                print(f"{'OK' if ok else 'BAD'} {label}({side}) 端口/高度: {nav}")
                if not ok: fails.append(f"{label}({side}): 入口/高度元数据与源几何不符")
        for label in ["坡道","阶梯","传递区"]:
            red=[o for o in obs if o.get("name")==f"{label}(红)"]
            blue=[o for o in obs if o.get("name")==f"{label}(蓝)"]
            if len(red)==len(blue)==1:
                a,b=red[0],blue[0]
                if not (abs(a["cx"]+b["cx"])<1e-6 and abs(a["cy"]-b["cy"])<1e-6 and a["w"]==b["w"] and a["h"]==b["h"]):
                    fails.append(f"{label}: 红蓝未严格镜像")
    compare(zones,"L1 区",box("collision_l1"),"l1")
    compare(zones,"L2 区",box("collision_l2"),"l2")
    compare(zones,"地面第一公共区",mesh(23),"public1")
    squares=[b for b in parts(5) if abs(b[1][0]-b[0][0]-1)<1e-6
             and abs(b[1][1]-b[0][1]-1)<1e-6 and b[1][2]-b[0][2]<1e-5]
    if len(squares)!=1:fails.append("源文件公共区识别失败")
    else:compare(zones,"地面第二公共区",squares[0],"public2")
    for index,label,tag in [(2,"红","red"),(3,"蓝","blue")]:
        patches=[b for b in parts(index) if b[1][2]-b[0][2]<1e-6 and abs(b[0][2]-.001)<1e-6]
        starts=sorted([b for b in patches if abs(b[1][0]-b[0][0]-.7)<1e-6 and abs(b[1][1]-b[0][1]-.7)<1e-6],
                      key=lambda b:abs((b[0][1]+b[1][1])/2),reverse=True)
        for k,src in enumerate(starts):compare(zones,f"{label}启动/重试区{'AB'[k]}",src,f"start_{tag}")
        storage=[b for b in patches if abs(b[1][0]-b[0][0]-1)<1e-6 and abs(b[1][1]-b[0][1]-2)<1e-6]
        if len(starts)!=2 or len(storage)!=1:fails.append("源文件启动/储存区识别失败")
        else:compare(zones,f"{label}队储存区",storage[0],f"storage_{tag}")
    for o in obs:
        if o.get("tag") in ("fence","divider","l1_body","pedestal","ramp_stair") and o.get("enabled") is not True:
            fails.append(f"{o.get('name')}: 固定/登高结构必须 enabled=true")
        if o.get("tag")=="transfer" and o.get("enabled") is not False:
            fails.append(f"{o.get('name')}: 传递区必须 enabled=false")
    print("说明：013/018是1m传递平台；019/020是L1到L2单级台阶。")
    if fails:
        print("=== 独立复核 FAIL ===")
        for f in fails:print("  -",f)
        return 1
    print("=== 独立复核 PASS: 源文件几何、通行性、镜像一致 ===")
    return 0


def main() -> int:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "all"
    if cmd == "verify":
        default = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "scenes", "nvwa_butian_2027_from_sim.json")
        path = sys.argv[2] if len(sys.argv) > 2 else default
        return cmd_verify_template(path)
    if cmd in ("bbox", "all"):
        print("=" * 100)
        print("一、色块包围盒(场地几何的精确来源)")
        print("=" * 100)
        cmd_bbox()
    if cmd in ("sample", "all"):
        print()
        print("=" * 100)
        print("二、关键结构顶点采样")
        print("=" * 100)
        cmd_sample()
    if cmd in ("collision", "all"):
        print()
        cmd_collision()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
