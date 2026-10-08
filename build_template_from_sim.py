"""Generate the planner's map from simulation meshes, keeping planner mm coordinates.
Run python build_template_from_sim.py --write. Existing files are backed up and
replaced atomically; the backup is kept until the acceptance checks pass.
"""
from __future__ import annotations
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
from datetime import date
from robocon_path.core.scene import Scene, SceneMeta
from robocon_path.core.sim_bridge import sim_to_planner
from robocon_path.core.sim_geometry import (SIM_ROOT, obj_bounds, obj_components,
                                           sdf_box, colored_ground_rectangles, tower_bounds)

HERE = Path(__file__).resolve().parent
OUT_NAME = "nvwa_butian_2027_from_sim.json"

def _group(tag):
    if tag in ("fence","divider","l1_body","pedestal"): return "fixed"
    if tag=="ramp_stair": return "climb"
    if tag=="transfer": return "transfer"
    if tag in ("tower_top","stack"): return "dynamic"
    return "reference"

def rect(bounds, name, tag, source, *, layer="ground", enabled=None, color=None):
    lo, hi = bounds
    cx, cy = sim_to_planner((lo[0]+hi[0])/2, (lo[1]+hi[1])/2)
    item = dict(type="rect", cx=cx, cy=cy, w=(hi[1]-lo[1])*1000,
                h=(hi[0]-lo[0])*1000, name=name, tag=tag, layer=layer,
                group=_group(tag), source=source)
    if enabled is not None: item["enabled"]=enabled
    if color: item["color"]=color
    item["sim_z_range_m"]=[lo[2],hi[2]]
    return item

def flat_part(parts, sx, sy):
    matches=[b for b in parts if abs(b[1][0]-b[0][0]-sx)<1e-6
             and abs(b[1][1]-b[0][1]-sy)<1e-6 and b[1][2]-b[0][2]<1e-5]
    if len(matches)!=1: raise ValueError(f"Cannot identify {sx}x{sy} surface")
    return matches[0]

def build() -> Scene:
    sc=Scene(meta=SceneMeta(name="nvwa_butian_2027_from_sim",
        note="源OBJ/SDF生成；TR仅经坡道/阶梯地面端口登高；禁止侧入和进入L1/L2。",
        origin_note="场地中心；+x东(蓝)、+y北；红x<0；mm；yaw逆时针为正"))
    sc.extent=(-5600.,-5600.,5600.,5600.)
    sc.resolution=25.
    sc.robot_radius=300.
    sc.safety_margin=5.
    obs=[]; zones=[]
    for key,label in [("collision_wall_right","北"),("collision_wall_left","南"),
                      ("collision_wall_front","东"),("collision_wall_back","西")]:
        obs.append(rect(sdf_box(key),f"场地围栏({label})","fence",f"sim model.sdf/{key}; rule 2.21",enabled=True))
    l1=sdf_box("collision_l1"); l2=sdf_box("collision_l2")
    obs.append(rect(l1,"L1 台体(地面层不可进入)","l1_body","sim model.sdf/collision_l1; rule 2.15",enabled=True))
    obs.append(rect(sdf_box("collision_center_divider"),"中央隔板","divider",
                    "sim model.sdf/collision_center_divider; rule 2.21",enabled=True))
    for i,name,layer in [(0,"五色石基座","ground"),(1,"中央基座","L2")]:
        lo,hi=obj_bounds(i)
        cx,cy=sim_to_planner((lo[0]+hi[0])/2,(lo[1]+hi[1])/2)
        obs.append(dict(type="circle",cx=cx,cy=cy,r=(hi[0]-lo[0])*500,
                        name=name,tag="pedestal",layer=layer,enabled=True,group="fixed",
                        source=f"sim robocon_{i:03}.obj",sim_z_range_m=[lo[2],hi[2]]))
    for index,label,color in [(21,"红","#f0d2d2"),(22,"蓝","#aad2e6")]:
        zones.append(rect(obj_bounds(index),f"{label}方地面",f"ground_{index}",
                          f"sim robocon_{index:03}.obj",color=color))
    zones += [rect(l1,"L1 区","l1","sim model.sdf/collision_l1",layer="L1",color="#dcb9a4"),
              rect(l2,"L2 区","l2","sim model.sdf/collision_l2",layer="L2",color="#c7cbd1")]
    public1=obj_bounds(23)
    public2=flat_part(obj_components(5),1.,1.)
    zones += [rect(public1,"地面第一公共区","public1","sim robocon_023.obj; rule 2.7",color="#efe9b8"),
              rect(public2,"地面第二公共区","public2","sim robocon_005.obj ground square; rule 2.7",color="#efe9b8")]
    for index,label,color in [(2,"红","#df2222"),(3,"蓝","#3200ff")]:
        patches=colored_ground_rectangles(index)
        starts=sorted([b for b in patches if abs(b[1][0]-b[0][0]-.7)<1e-6
                       and abs(b[1][1]-b[0][1]-.7)<1e-6],key=lambda b:abs((b[0][1]+b[1][1])/2),reverse=True)
        storage=[b for b in patches if abs(b[1][0]-b[0][0]-1.)<1e-6
                 and abs(b[1][1]-b[0][1]-2.)<1e-6]
        if len(starts)!=2 or len(storage)!=1: raise ValueError("Missing start/storage source rectangles")
        side="red" if index==2 else "blue"
        for k,b in enumerate(starts):
            zones.append(rect(b,f"{label}启动/重试区{'AB'[k]}",f"start_{side}",
                         f"sim robocon_{index:03}.obj ground patch; rule 2.3",color=color))
        zone=rect(storage[0],f"{label}队储存区",f"storage_{side}",
                  f"sim robocon_{index:03}.obj ground patch; rule 2.6",color="#e8b4b4" if index==2 else "#9dc6e0")
        zones.append(zone)
        # Keep the 10 existing optional stack markers per team; source has no stack positions.
        for k in range(10):
            obs.append(dict(type="rect",cx=zone["cx"]+(k%5-2)*400,
                    cy=zone["cy"]+(-250 if k<5 else 250),w=350.,h=350.,
                    name=f"料堆{'R' if index==2 else 'B'}{k+1}",tag="stack",layer="ground",
                    enabled=False,group="dynamic",source="assumed positions in measured storage; rule 2.11 size"))
    for side,offset,color in [("红",0,"#e8b4b4"),("蓝",5,"#9dc6e0")]:
        ramp=obj_bounds(9+offset); plateau=obj_bounds(13+offset); bottom=obj_bounds(10+offset)
        # The upper 1x1m landing is transfer, NOT a fourth exposed stair tread.
        stairs=(bottom[0],(plateau[0][0],bottom[1][1],obj_bounds(12+offset)[1][2]))
        for b,name,tag,z_tag,src in [
                (ramp,"坡道","ramp_stair","ramp",f"sim robocon_{9+offset:03}.obj; rule 2.16"),
                (stairs,"阶梯","ramp_stair","stair",f"sim exposed treads robocon_{10+offset:03}~{12+offset:03}.obj; rule 2.16"),
                (plateau,"传递区","transfer","transfer",f"sim robocon_{13+offset:03}.obj top landing; rule 2.18")]:
            obs.append(rect(b,f"{name}({side})",tag,src,layer="L1",enabled=tag!="transfer"))
            item=obs[-1]
            kind={"ramp":"ramp","stair":"stairs","transfer":"transfer"}[z_tag]
            item["navigation"]={"kind":kind,"side":"red" if offset==0 else "blue",
                "entry_axis":"y","top_height_mm":600.,
                "rule":"2.16; 2.18; 3.2.1; 4.3.1; 6.2.1"}
            if kind!="transfer":
                item["navigation"]["ground_end_y_mm"]=item["cy"]+(item["h"]/2 if kind=="ramp" else -item["h"]/2)
            zones.append(rect(b,f"{name}({side})",f"{z_tag}_{'red' if offset==0 else 'blue'}",
                    src,layer="L1",color="#f5aa3c" if tag=="transfer" else color))
        for step in range(3):
            a,b=obj_bounds(10+offset+step)
            end=obj_bounds(11+offset+step)[0][0]
            zones.append(rect((a,(end,b[1],b[2])),f"台阶{step+1}({side})","stair_tread",
                    f"sim exposed top robocon_{10+offset+step:03}.obj",layer="L1",color="#8b6f61"))
        for index in [19+offset//5]:
            zones.append(rect(obj_bounds(index),f"L1→L2台阶({side})","l2_step",
                        f"sim robocon_{index:03}.obj; rule 2.17",layer="L1",color=color))
    for k,b in enumerate(tower_bounds(),1):
        obs.append(rect(b,f"塔顶{k}","tower_top","sim paired red/blue box halves in robocon_002/003.obj; rule 2.13",
                        enabled=False))
    # Green build pads are display references; L1/L2 remain blocked to a ground TR.
    for b in obj_components(8):
        lo,hi=b
        if abs(hi[0]-lo[0]-.5)<1e-6 and abs(hi[1]-lo[1]-.5)<1e-6:
            zones.append(rect(b,"建塔处","build_pad","sim robocon_008.obj",layer="L2" if lo[2]>.8 else "L1",color="#3a885a"))
    sc.obstacles=obs;sc.zones=zones
    return sc

def write_atomic(scene,path):
    if path.exists():
        # Do not overwrite someone else's outstanding backup.
        backup=Path(str(path)+".bak")
        if backup.exists(): backup=Path(str(path)+f".{date.today().isoformat()}.bak")
        if backup.exists(): raise FileExistsError(f"Backup already exists: {backup}")
        shutil.copy2(path,backup)
        print(f"已备份: {backup}")
    payload=scene.to_dict()
    payload["meta"].update(schema_version=2,last_edited=date.today().isoformat(),
                          sim_source=str(SIM_ROOT),assumed_items=["stack marker positions"])
    fd,tmp=tempfile.mkstemp(prefix=path.name+".",suffix=".tmp",dir=path.parent)
    try:
        with os.fdopen(fd,"w",encoding="utf-8",newline="\n") as f:
            json.dump(payload,f,ensure_ascii=False,indent=2)
            f.write("\n");f.flush();os.fsync(f.fileno())
        with open(tmp,encoding="utf-8") as f: json.load(f)
        os.replace(tmp,path)
    finally:
        if os.path.exists(tmp):os.unlink(tmp)

def main():
    sc=build()
    print("按源仿真生成地图（planner mm，+y向上）")
    print(f"obstacles={len(sc.obstacles)}, zones={len(sc.zones)}, radius={sc.robot_radius}+{sc.safety_margin}mm")
    for o in sc.obstacles:
        if o["tag"] not in ("tower_top","stack"):
            print(f"{o['name']}: center=({o['cx']:.0f},{o['cy']:.0f}), "
                  f"size={o.get('w',o.get('r')):.0f}/{o.get('h',o.get('r')):.0f}, enabled={o['enabled']}")
    if "--write" in sys.argv:
        path=HERE/"scenes"/OUT_NAME
        write_atomic(sc,path)
        print(f"已原子写入: {path} ({path.stat().st_size} bytes)")
    else: print("未写入；使用 --write 生成场景")
    return 0
if __name__=="__main__":raise SystemExit(main())
