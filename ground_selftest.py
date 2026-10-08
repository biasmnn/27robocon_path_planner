"""Ground-only routing and analytic tangent-arc regression; one command."""
from pathlib import Path
import copy
import hashlib
import math
import numpy as np

from robocon_path.core.geometry import Pose,point_segment_distance
from robocon_path.core.scene import Scene
from robocon_path.core.surface_navigation import SurfaceMap
from robocon_path.pipeline import PipelineConfig,build_map,plan_route
from robocon_path.plan.fillet import fillet_polyline
from robocon_path.export import trajectory_to_csv,trajectory_to_c_code

ROOT=Path(__file__).resolve().parent


def max_turn(points):
    p=np.asarray(points,dtype=float)
    d=np.diff(p,axis=0)
    norm=np.linalg.norm(d,axis=1)
    assert np.all(norm>1e-7),"连续点重复"
    return float(np.degrees(np.max(np.arccos(np.clip(
        np.sum(d[:-1]*d[1:],axis=1)/(norm[:-1]*norm[1:]),-1.,1.))))) if len(d)>1 else 0.


def tangent_tests():
    for rotation in (0.,37.,180.):
        a=math.radians(rotation)
        transform=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
        for degrees in (-135.,-90.,-45.,-15.,15.,45.,90.,135.):
            turn=math.radians(degrees)
            pts=np.array([[-2000.,0.],[0.,0.],[2000.*math.cos(turn),2000.*math.sin(turn)]])@transform.T
            radius=200.
            r=fillet_polyline(pts,radius,arc_step_mm=5.)
            assert r.ok and r.corners_rounded==1,r
            arc=np.array(r.points[1:-1])@transform
            tangent_dist=radius*math.tan(abs(turn)/2.)
            center=np.array([-tangent_dist,math.copysign(radius,turn)])
            assert np.allclose(np.linalg.norm(arc-center,axis=1),radius,atol=1e-7),"圆弧不在解析圆上"
            assert np.allclose(arc[0],[-tangent_dist,0.],atol=1e-7)
            assert np.allclose(arc[-1],tangent_dist*np.array([math.cos(turn),math.sin(turn)]),atol=1e-7)
            incoming=(arc[1]-arc[0])/np.linalg.norm(arc[1]-arc[0])
            outgoing=(arc[-1]-arc[-2])/np.linalg.norm(arc[-1]-arc[-2])
            assert np.dot(incoming,[1,0])>math.cos(.02)
            assert np.dot(outgoing,[math.cos(turn),math.sin(turn)])>math.cos(.02)
            assert max_turn(r.points)<1.5
    # Multiple consecutive mixed turns: verify short shared edges cannot fold back.
    r=fillet_polyline([(-2000,0),(0,0),(0,-300),(1000,-300),(1000,-1600)],400.,arc_step_mm=5.)
    assert r.ok and max_turn(r.points)<3.
    print("OK 24组旋转/左右转/不同转角：解析半径200mm，切点/切线连续，无反折")
    print("OK 连续左右转短边衔接，无圆弧重叠或折返")


def independent_ground(points,side,radius):
    # Independent dense physical check against source rectangles, not support()
    # or navigation metadata. Use closed AABBs for L1 and the whole side strip.
    boxes=[(-3000.,-3000.,3000.,3000.),(-4000.,-2700.,-3000.,2700.),(3000.,-2700.,4000.,2700.)]
    sign=-1 if side=="red" else 1
    samples=0
    for a,b in zip(points,points[1:]):
        length=math.hypot(b.x-a.x,b.y-a.y)
        n=max(1,math.ceil(length/5.))
        for f in np.linspace(0.,1.,n+1):
            x,y=a.x+f*(b.x-a.x),a.y+f*(b.y-a.y)
            for x0,y0,x1,y1 in boxes:
                dx=max(x0-x,0.,x-x1);dy=max(y0-y,0.,y-y1)
                assert math.hypot(dx,dy)>=radius-1e-6,"圆包络进入高结构"
            assert sign*x>=radius-1e-6,"越入对方区域"
            samples+=1
    return samples


def main():
    path=ROOT/"scenes/nvwa_butian_2027_from_sim.json"
    before=hashlib.sha256(path.read_bytes()).hexdigest()
    s=Scene.load(str(path))
    cfg=PipelineConfig(min_radius_mm=800.)
    assert cfg.climb_mode=="ground"
    cm=build_map(s,cfg)
    for sign in (-1,1):
        for y in (-2250.,-1300.,950.):
            assert not cm.is_free_world(sign*3500.,y)
    print("OK 默认地面模式：红蓝坡道/阶梯/传递平台均不可进入")
    good=None
    for side,sign in (("red",-1),("blue",1)):
        for label,a,b in [("图示北侧到南侧",(sign*4200.,4300.),(sign*1200.,-4100.)),
                          ("南侧返回北侧",(sign*1200.,-4100.),(sign*4200.,4300.)),
                          ("地面直线",(sign*4900.,-4200.),(sign*4900.,4200.))]:
            t=plan_route(s,Pose(*a,0),Pose(*b,0),cfg)
            assert t.count>=2,t.messages
            assert np.allclose([t.points[0].x,t.points[0].y],a)
            assert np.allclose([t.points[-1].x,t.points[-1].y],b)
            t.navigation_map.validate(t.points)
            assert all(state=="ground" and z==0 for state,z in t.support_samples)
            samples=independent_ground(t.points,side,305.)
            angle=max_turn([(p.x,p.y) for p in t.points])
            assert angle<3.,f"突折{angle}deg"
            gaps=np.linalg.norm(np.diff([(p.x,p.y) for p in t.points],axis=0),axis=1)
            assert gaps.max()<=cfg.max_point_gap_mm
            assert t.points[0].vx==t.points[0].vy==t.points[-1].vx==t.points[-1].vy==0
            assert all(math.isfinite(v) for p in t.points for v in p.as_row())
            trajectory_to_csv(t);trajectory_to_c_code(t)
            print(f"OK {side} {label}: {t.count}点, {t.meta.length_mm:.1f}mm, 最大相邻转角={angle:.3f}deg, 最大间距={gaps.max():.2f}mm, 独立5mm采样={samples}, 全程ground/z0")
            good=t
    for sign in (-1,1):
        start=Pose(sign*4900.,4300.,0)
        for y in (-2250.,-1300.,950.):
            t=plan_route(s,start,Pose(sign*3500.,y,0),cfg)
            assert not t.points,"高结构终点被吸附到地面"
    print("OK 高结构上的起终点直接拒绝，不吸附跨区")
    blocked=copy.copy(good)
    blocked.points=[copy.copy(good.points[0]),copy.copy(good.points[-1])]
    # Endpoints on ground, shortcut intersects stairs/ramp/L1 footprint.
    blocked.points[0].x=-4200.;blocked.points[0].y=4300.
    blocked.points[-1].x=-1200.;blocked.points[-1].y=-4100.
    blocked.navigation_map=SurfaceMap(s,team="red")
    for label,exporter in (("CSV",trajectory_to_csv),("C",trajectory_to_c_code)):
        try: exporter(blocked)
        except ValueError as exc: print(f"OK 地面点表穿越高结构，{label}拒绝导出: {exc}")
        else: raise AssertionError("穿越被漏检")
    tangent_tests()
    assert hashlib.sha256(path.read_bytes()).hexdigest()==before
    print("OK 场景JSON字节完全不变：几何/tag/enabled未改")
    print("=== 地面路径/圆弧连续性自检 PASS ===")
    return 0


if __name__=="__main__":raise SystemExit(main())
