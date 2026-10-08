"""One-command TR access tests, independent entrance trace and fault injection."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import math

from robocon_path.core.geometry import Pose
from robocon_path.core.scene import Scene
from robocon_path.core.surface_navigation import SurfaceMap
from robocon_path.core.file_io import atomic_text
from robocon_path.pipeline import PipelineConfig, plan_route, resolve_scene
from robocon_path.export import trajectory_to_csv,trajectory_to_c_code

ROOT=Path(__file__).resolve().parent


def independent_trace(points,side,expected):
    # Deliberately does not call support()/validate() or read navigation metadata.
    # Dimensions are the rule/source mesh cross-check: x width 1m, ramp end
    # y2700, stair end y-2700, landing[-1800,-800]. Check all crossings exactly.
    lo,hi=(-4000.,-3000.) if side=="red" else (3000.,4000.)
    entries=[]
    for a,b in zip(points,points[1:]):
        cuts=[0.,1.]
        for axis,v in [(0,lo),(0,hi),(1,-2700.),(1,2700.)]:
            d=(b.x-a.x) if axis==0 else (b.y-a.y)
            start=a.x if axis==0 else a.y
            if d and 0<(f:=(v-start)/d)<1:cuts.append(f)
        cuts=sorted(cuts)
        for f,g in zip(cuts,cuts[1:]):
            x=a.x+(b.x-a.x)*(f+g)/2
            y=a.y+(b.y-a.y)*(f+g)/2
            if lo<x<hi and -2700<y<2700:
                assert lo+305-1e-6<=x<=hi-305+1e-6,"侧边净空不足"
        # Crossings from ground to raised structure must be at its two ends.
        for y,kind in [(2700.,"ramp"),(-2700.,"stairs")]:
            if min(a.y,b.y)<y<=max(a.y,b.y):
                f=(y-a.y)/(b.y-a.y)
                x=a.x+f*(b.x-a.x)
                if lo<x<hi:
                    assert lo+305<=x<=hi-305
                    entries.append(kind)
        for xedge in (lo,hi):
            if min(a.x,b.x)<xedge<=max(a.x,b.x):
                f=(xedge-a.x)/(b.x-a.x)
                y=a.y+f*(b.y-a.y)
                assert not -2700<y<2700,"从结构侧面进入"
    assert expected in entries,(expected,entries)
    return entries


def rejected(label,action):
    try: action()
    except ValueError as exc:
        print(f"OK 故障被拒绝: {label}: {exc}")
        return
    raise AssertionError("错误被漏检: "+label)


def main():
    (ROOT/"out").mkdir(exist_ok=True)
    s=Scene.load(str(ROOT/"scenes/nvwa_butian_2027_from_sim.json"))
    # Historical climb implementation is regression-only, explicitly opt in.
    # The current application/default pipeline is ground-only.
    cfg=PipelineConfig(climb_mode="auto")
    good=None
    for side,sign in (("red",-1),("blue",1)):
        goal=(sign*3500.,-1300.)
        for kind,start in (("stairs",(sign*5150.,-5150.)),("ramp",(sign*4500.,5000.))):
            for reverse in (False,True):
                a,b=(goal,start) if reverse else (start,goal)
                t=plan_route(s,Pose(*a,0),Pose(*b,0),cfg)
                assert t.count>=2,t.messages
                t.navigation_map.validate(t.points)
                entries=independent_trace(t.points,side,kind)
                assert any(z==600. for state,z in t.support_samples)
                assert t.points[0].vx==t.points[0].vy==t.points[-1].vx==t.points[-1].vy==0
                trajectory_to_csv(t);trajectory_to_c_code(t)
                print(f"OK {side} {kind} {'返回' if reverse else '登高'}: {t.count}点, {t.meta.length_mm:.1f}mm, 独立端口轨迹={entries}, z=600mm")
                good=t
    cm=SurfaceMap(resolve_scene(s,cfg),team="red",climb_mode="auto")
    for mode,start in (("ramp",(-5150.,-5150.)),("stairs",(-4500.,5000.))):
        t=plan_route(s,Pose(*start,0),Pose(-3500,-1300,0),PipelineConfig(climb_mode=mode))
        assert t.count>=2,t.messages
        independent_trace(t.points,"red",mode)
        assert not any(state.startswith("stairs" if mode=="ramp" else "ramp") for state,z in t.support_samples)
        print(f"OK 强制{mode}: 从另一端的地面绕行至指定入口，{t.count}点")
    t=plan_route(s,Pose(-4500,5000,0),Pose(-3500,-1300,0),PipelineConfig(heading_mode="linear",climb_mode="auto"))
    assert t.count>=2,t.messages
    for p in t.points:
        if -2700<p.y<2700:
            assert abs(p.vx)<abs(p.vy)+1e-6,"世界速度不应随独立yaw旋到结构侧边"
    print("OK 线性yaw模式: 世界速度沿路径切线，首末速度为0")
    for name,a,b in [("侧进平台",(-4500,-1300),(-3500,-1300)),
                     ("侧进坡道",(-4500,1000),(-3500,1000)),
                     ("侧进阶梯",(-4500,-2200),(-3500,-2200)),
                     ("入口斜切侧边",(-4500,3100),(-3500,2600)),
                     ("从平台跳到地面",(-3500,-1300),(-4500,-1300))]:
        rejected(name,lambda a=a,b=b:cm.validate([a,b]))
    for goal in [(0,0),(-3100,-1300),(3500,-1300),(-3900,1000)]:
        t=plan_route(s,Pose(-4500,5000,0),Pose(*goal,0),cfg)
        assert not t.points,(goal,t.messages)
        print(f"OK 非法终点不吸附: {goal}: {t.messages}")
    # Both valid endpoints, illegal interpolated chord: export must fail closed.
    bad=copy.copy(good)
    bad.points=[copy.copy(good.points[0]),copy.copy(good.points[-1])]
    bad.points[0].x=-4500.;bad.points[0].y=-1300.
    bad.points[-1].x=-3500.;bad.points[-1].y=-1300.
    bad.navigation_map=cm
    rejected("导出被篡改的侧向点表CSV",lambda:trajectory_to_csv(bad))
    rejected("导出被篡改的侧向点表C",lambda:trajectory_to_c_code(bad))
    for radius in (350.,495.,510.):
        cfg2=PipelineConfig(robot_radius_mm=radius,climb_mode="auto")
        t=plan_route(s,Pose(-4500,5000,0),Pose(-3500,-1300,0),cfg2)
        assert (t.count>=2)==(radius<495.),(radius,t.messages)
        print(f"OK 半径{radius}+5mm: {'合法登高' if t.points else '宽度或端点净空不足，拒绝出表'}")
    doc=json.loads((ROOT/"scenes/nvwa_butian_2027_from_sim.json").read_text(encoding="utf-8"))
    before=json.loads((ROOT/"tests/fixtures/map_before_rules_20261006.json").read_text(encoding="utf-8"))
    for section in ("obstacles","zones"):
        assert len(doc[section])==len(before[section])
        for a,b in zip(before[section],doc[section]):
            assert all(b.get(k)==v for k,v in a.items()),(section,a["name"])
    print("OK 原地图全部项、坐标、尺寸、tag/enabled逐字段保持")
    for label,mutate in [("入口移动50mm",lambda d:d["obstacles"][next(i for i,o in enumerate(d["obstacles"]) if o.get("navigation",{}).get("kind")=="ramp")]["navigation"].update(ground_end_y_mm=2750.)),
                          ("高度错误",lambda d:[o["navigation"].update(top_height_mm=0.) for o in d["obstacles"] if "navigation" in o]),
                          ("丢失导航语义",lambda d:[o.pop("navigation",None) for o in d["obstacles"]])]:
        bad=copy.deepcopy(doc);mutate(bad)
        path=ROOT/"out"/("rules_bad_"+label+".json")
        atomic_text(path,json.dumps(bad,ensure_ascii=False,indent=2))
        r=subprocess.run([sys.executable,"probe_sim_field.py","verify",str(path)],cwd=ROOT,capture_output=True,text=True,encoding="utf-8")
        print(f"\n故障副本: {path}; 错误={label}; exit={r.returncode}\n{r.stdout}{r.stderr}")
        assert r.returncode!=0
        rejected("缺陷场景拒绝构图: "+label,lambda bad=bad:SurfaceMap(Scene.from_dict(bad)))
    print("=== 规则/物理通路自检 PASS ===")
    return 0


if __name__=="__main__":raise SystemExit(main())
