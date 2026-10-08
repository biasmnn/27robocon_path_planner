"""Planner (mm,+x east,+y north) to simulation (m,red y>0).
planner->sim rotates -90 degrees: (sx,sy)=(py,-px)/1000.
The scene map uses OBJ footprints, with SDF base boundaries; no simulator is run.
"""
from __future__ import annotations
import math
import sys
from pathlib import Path
if not __package__:
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from robocon_path.core.sim_geometry import obj_bounds, sdf_box

def planner_to_sim(x_mm: float,y_mm: float):
    return y_mm/1000.,-x_mm/1000.

def sim_to_planner(sim_x: float,sim_y: float):
    return -sim_y*1000.,sim_x*1000.

def planner_yaw_to_sim(yaw: float):
    return yaw-math.pi/2

def sim_yaw_to_planner(yaw: float):
    return yaw+math.pi/2

def _field():
    def ranges(i):
        a,b=obj_bounds(i)
        return dict(x_range=[a[0],b[0]],y_range=[a[1],b[1]],z_range=[a[2],b[2]])
    def pedestal(i):
        a,b=obj_bounds(i)
        return dict(center=[(a[0]+b[0])/2,(a[1]+b[1])/2],
                    diameter=b[0]-a[0],height=b[2]-a[2],z_range=[a[2],b[2]])
    a,b=sdf_box("collision_l1");c,d=sdf_box("collision_l2")
    return {
        "field_size":11.,
        "fence":{"height":.08,"thickness":.05,"center_offset":5.5},
        "divider":{"axis":"x","at":0.,"thickness":.05,"height":.08,"length":11.},
        "half_red":{"y_range":[0.,5.5]},"half_blue":{"y_range":[-5.5,0.]},
        "L1":{"x_range":[a[0],b[0]],"y_range":[a[1],b[1]],"height":b[2],"rail_height":.05,"rail_thickness":.05},
        "L2":{"x_range":[c[0],d[0]],"y_range":[c[1],d[1]],"height":d[2]},
        "pedestal_five_color":pedestal(0),"pedestal_central":pedestal(1),
        "corridor":{"x_range":[-2.7,2.7],"width":1.,"red_y_range":[3.,4.],"blue_y_range":[-4.,-3.]},
        "ramp":ranges(9),"ramp_blue":ranges(14),
        "stairs":{"steps":3,"step_len_x":.3,"step_w_y":1.,"z_steps":[.15,.3,.45],
                  "x_range":[-2.7,-1.8]},
        "transfer":{"red":ranges(13),"blue":ranges(18)},
        "l2_step":{"red":ranges(19),"blue":ranges(20)}
    }

SIM_FIELD=_field()
SIM_PROBE_BLOCKED=[("L1",0.,0.),("五色石",*SIM_FIELD["pedestal_five_color"]["center"])]
SIM_PROBE_FREE=[("红地面",-4.25,4.5),("蓝地面",-4.25,-4.5)]
TEMPLATE_MISMATCH=[]  # The generated scene is checked by probe_sim_field.py.

def check(verbose=True):
    tests=[
        ("+x基向量",planner_to_sim(1000,0),(0.,-1.)),
        ("+y基向量",planner_to_sim(0,1000),(1.,0.)),
        ("红传递区",planner_to_sim(-3500,-1300),(-1.3,3.5)),
        ("蓝传递区",planner_to_sim(3500,-1300),(-1.3,-3.5)),
        ("五色石",planner_to_sim(0,4150),tuple(SIM_FIELD["pedestal_five_color"]["center"])),
        ("中央基座",planner_to_sim(0,0),tuple(SIM_FIELD["pedestal_central"]["center"])),
    ]
    bad=0
    for name,got,want in tests:
        ok=all(abs(a-b)<1e-9 for a,b in zip(got,want));bad+=not ok
        if verbose:print(f"{'OK' if ok else 'BAD'} {name}: got={got}, source/expected={want}")
    for x,y in [(1234.,-5678.),(-5100.,4400.)]:
        back=sim_to_planner(*planner_to_sim(x,y))
        bad+=any(abs(a-b)>1e-6 for a,b in zip(back,(x,y)))
    for yaw in [-math.pi,0.,.7,math.pi]:
        bad+=abs(sim_yaw_to_planner(planner_yaw_to_sim(yaw))-yaw)>1e-12
    red=obj_bounds(21);blue=obj_bounds(22)
    bad+=not (red[0][1]==0. and red[1][1]>0. and blue[1][1]==0. and blue[0][1]<0.)
    if verbose:
        print("planner -> sim: -90°, det=+1; sim -> planner: +90°; mm <-> m")
        print("=== 变换自检: "+("PASS" if not bad else f"FAIL({bad})")+" ===")
    return int(bool(bad))
if __name__=="__main__":raise SystemExit(check())
