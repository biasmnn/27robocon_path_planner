"""TR surface access: ground -> end entrance -> ramp/stairs -> transfer.

Original obstacles retain tag/enabled and geometry. The ground projection of
an elevated landing is blocked; only the connected support strip is traversable.
Distances use the configured circular envelope, including safety margin.
This verifies access geometry, not traction or a chassis's stair capability.
"""
from __future__ import annotations

import math
import numpy as np

from .costmap import CostMap
from .scene import _item_to_polygon


def required(scene):
    return any(o.get("navigation") or (o.get("tag") == "transfer" and
               "sim_z_range_m" in o) for o in scene.obstacles)


def _bounds(o):
    return (o["cx"]-o["w"]/2, o["cy"]-o["h"]/2,
            o["cx"]+o["w"]/2, o["cy"]+o["h"]/2)


def _point_segment(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/
                       max(1e-30, dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def _seg_distance(a, b, c, d):
    cross = lambda p, q, r: (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    # Collinear disjoint edges must not be mistaken for intersections.
    overlap = (max(min(a[0],b[0]),min(c[0],d[0])) <= min(max(a[0],b[0]),max(c[0],d[0]))
               and max(min(a[1],b[1]),min(c[1],d[1])) <= min(max(a[1],b[1]),max(c[1],d[1])))
    if overlap and cross(a,b,c)*cross(a,b,d) <= 0 and cross(c,d,a)*cross(c,d,b) <= 0:
        return 0.
    return min(_point_segment(a,c,d), _point_segment(b,c,d),
               _point_segment(c,a,b), _point_segment(d,a,b))


class SurfaceMap(CostMap):
    """Access map plus exact swept-circle checks for every final segment."""

    def __init__(self, scene, resolution=None, team=None, climb_mode="ground"):
        super().__init__(scene, resolution)
        if climb_mode not in ("ground","auto","ramp","stairs"):
            raise ValueError("通行方式必须为 ground/auto/ramp/stairs")
        if team not in (None,"red","blue"): raise ValueError("未知队伍")
        self.team = team
        self.climb_mode=climb_mode
        self.corridors = []
        for side in ("red", "blue"):
            items = {o.get("navigation", {}).get("kind"): o for o in scene.obstacles
                     if o.get("navigation", {}).get("side") == side}
            if set(items) != {"ramp", "stairs", "transfer"}:
                raise ValueError(f"{side}: 爬升通路元数据缺失，拒绝按地面规划")
            ramp, stair, transfer = (items[k] for k in ("ramp", "stairs", "transfer"))
            r, s, t = map(_bounds, (ramp, stair, transfer))
            if not (r[0] == s[0] == t[0] and r[2] == s[2] == t[2]
                    and abs(s[3]-t[1]) < 1e-6 and abs(t[3]-r[1]) < 1e-6):
                raise ValueError("爬升通路不连续/宽度不一致")
            for o, kind in ((ramp,"ramp"),(stair,"stairs"),(transfer,"transfer")):
                n = o["navigation"]
                if n.get("top_height_mm") != 600. or n.get("entry_axis") != "y":
                    raise ValueError("不支持或损坏的爬升语义")
                if o.get("enabled") is not (kind != "transfer"):
                    raise ValueError("爬升/传递区 enabled 不符合规则")
            if ramp["navigation"].get("ground_end_y_mm") != r[3] or stair["navigation"].get("ground_end_y_mm") != s[1]:
                raise ValueError("爬升入口与几何不一致")
            self.corridors.append(dict(side=side, x0=r[0], x1=r[2],
                y0=s[1], y1=r[3], stair_top=s[3], ramp_top=r[1], height=600.))
        self.static = [o for o in scene.obstacles if o.get("enabled",True)
                       and o.get("tag") not in ("ramp_stair","transfer")]
        self.ground = self.static + [o for o in scene.obstacles
                                    if o.get("tag") in ("ramp_stair","transfer")]
        if self.climb_mode=="ground":
            # A landing may be enabled=false for its top surface. Its raised
            # footprint still blocks ground travel; do not alter source flags.
            self.raw_blocked=self._rasterize([_item_to_polygon(o) for o in self.ground])
        self.public = [_bounds(z) for z in scene.zones if z.get("tag") in ("public1","public2")]
        yy, xx = np.mgrid[:self.spec.height, :self.spec.width]
        xx = self.spec.x_min+(xx+.5)*self.resolution
        yy = self.spec.y_min+(yy+.5)*self.resolution
        self._ground_distance=self._clear(self.ground,xx,yy)
        self._static_distance=self._clear(self.static,xx,yy)
        self.distance_mm = self.clearance_many_mm(xx, yy)
        self.cost = self.distance_mm >= self.blocked_radius
        self.inflated = (~self.raw_blocked) & (~self.cost)

    def _tube(self,c):
        pad=self.blocked_radius+2*self.resolution
        return (c["stair_top"] if self.climb_mode=="ramp" else c["y0"]-pad,
                c["ramp_top"] if self.climb_mode=="stairs" else c["y1"]+pad)

    def _strip_clearance(self,c,xs,ys):
        strip=np.minimum(xs-c["x0"],c["x1"]-xs)
        if self.climb_mode=="ramp": strip=np.minimum(strip,ys-c["stair_top"])
        if self.climb_mode=="stairs": strip=np.minimum(strip,c["ramp_top"]-ys)
        return strip

    def edge_is_free_cells(self,row,col,nr,nc):
        # Distance to a closed obstacle is 1-Lipschitz. End clearances minus
        # half the edge length certify the entire edge, without sampling.
        margin=self.resolution*math.hypot(nr-row,nc-col)/2
        a,b=self.spec.cell_to_world(row,col),self.spec.cell_to_world(nr,nc)
        if self.team:
            sign=-1 if self.team=="red" else 1
            if min(sign*a[0],sign*b[0])<self.blocked_radius+margin:
                return self.segment_is_free(a,b)
        if min(self._ground_distance[row,col],self._ground_distance[nr,nc]) >= self.blocked_radius+margin:
            return True
        if self.climb_mode=="ground": return self.segment_is_free(a,b)
        for c in self.corridors:
            y0,y1=self._tube(c)
            if all(float(self._strip_clearance(c,*p))>=self.blocked_radius
                   and y0<=p[1]<=y1 for p in (a,b)):
                if min(self._static_distance[row,col],self._static_distance[nr,nc])>=self.blocked_radius+margin:
                    return True
        return self.segment_is_free(a,b)

    @staticmethod
    def _distance(o, xs, ys):
        if o["type"] == "rect":
            dx = np.abs(xs-o["cx"])-o["w"]/2
            dy = np.abs(ys-o["cy"])-o["h"]/2
            return np.hypot(np.maximum(dx,0),np.maximum(dy,0))+np.minimum(np.maximum(dx,dy),0)
        if o["type"] == "circle":
            return np.hypot(xs-o["cx"],ys-o["cy"])-o["r"]
        points = _item_to_polygon(o).points
        inside = np.zeros(np.broadcast(xs,ys).shape,dtype=bool)
        distance = np.full(inside.shape,np.inf)
        for a,b in zip(points, points[1:]+points[:1]):
            dx,dy=b[0]-a[0],b[1]-a[1]
            f=np.clip(((xs-a[0])*dx+(ys-a[1])*dy)/max(1e-30,dx*dx+dy*dy),0,1)
            distance=np.minimum(distance,np.hypot(xs-a[0]-f*dx,ys-a[1]-f*dy))
            if dy:
                inside ^= ((a[1]>ys)!=(b[1]>ys)) & (xs < a[0]+(ys-a[1])*dx/dy)
        return np.where(inside,-distance,distance)

    def _clear(self, items, xs, ys):
        out=np.full(np.broadcast(xs,ys).shape,np.inf)
        for o in items:
            out=np.minimum(out,self._distance(o,xs,ys))
        return out

    def clearance_many_mm(self, xs, ys):
        xs,ys=np.asarray(xs,dtype=float),np.asarray(ys,dtype=float)
        out=self._clear(self.ground,xs,ys)
        static=self._clear(self.static,xs,ys)
        for c in ([] if self.climb_mode=="ground" else self.corridors):
            # Axial approach tube bridges the inflated ground end. Its outside
            # extension is ground, not an invented ramp or a moved obstacle.
            y0,y1=self._tube(c)
            in_y=(ys>=y0)&(ys<=y1)
            strip=self._strip_clearance(c,xs,ys)
            out=np.maximum(out,np.where(in_y,np.minimum(strip,static),-np.inf))
        xmin,ymin,xmax,ymax=self.scene.extent
        out=np.minimum(out,np.minimum.reduce(np.broadcast_arrays(xs-xmin,xmax-xs,ys-ymin,ymax-ys)))
        if self.team:
            own=-xs if self.team=="red" else xs
            territory=own
            for x0,y0,x1,y1 in self.public:
                territory=np.maximum(territory,np.minimum.reduce(np.broadcast_arrays(xs-x0,x1-xs,ys-y0,y1-ys)))
            out=np.minimum(out,territory)
        return out

    def clearance_mm(self,x,y):
        return float(self.clearance_many_mm(x,y))

    def is_free_world(self,x,y):
        return math.isfinite(x) and math.isfinite(y) and self.clearance_mm(x,y)>=self.blocked_radius

    def nearest_free_cell(self,x,y,max_radius_mm=2000.):
        # Endpoints cannot be snapped from L1, an edge, or the wrong surface.
        if not self.is_free_world(x,y):
            return None
        return super().nearest_free_cell(x,y,max_radius_mm=self.resolution*1.5)

    def _swept_clear(self,items,a,b):
        for o in items:
            if min(float(self._distance(o,*a)),float(self._distance(o,*b))) < self.blocked_radius:
                return False
            if o["type"]=="circle":
                distance=_point_segment((o["cx"],o["cy"]),a,b)-o["r"]
            else:
                p=_item_to_polygon(o).points
                distance=min(_seg_distance(a,b,c,d) for c,d in zip(p,p[1:]+p[:1]))
            if distance < self.blocked_radius-1e-8:
                return False
        return True

    def segment_is_free(self,a,b,sample_step_mm=None):
        if not self.is_free_world(*a) or not self.is_free_world(*b):
            return False
        # Partition exactly at support-tube boundaries. Each piece must have
        # either ground clearance or support-strip clearance for its WHOLE sweep.
        cuts={0.,1.}
        for c in ([] if self.climb_mode=="ground" else self.corridors):
            y0,y1=self._tube(c)
            for axis,value in [(0,c["x0"]+self.blocked_radius),(0,c["x1"]-self.blocked_radius),
                               (1,y0),(1,y1),
                               (1,c["stair_top"]+self.blocked_radius),
                               (1,c["ramp_top"]-self.blocked_radius)]:
                delta=b[axis]-a[axis]
                if delta:
                    f=(value-a[axis])/delta
                    if 0<f<1: cuts.add(f)
        # Team boundaries/public rectangles are additional convex pieces.
        if self.team:
            for axis,value in [(0,-self.blocked_radius),(0,self.blocked_radius)]+[
                (axis,v) for x0,y0,x1,y1 in self.public for axis,v in
                [(0,x0+self.blocked_radius),(0,x1-self.blocked_radius),
                 (1,y0+self.blocked_radius),(1,y1-self.blocked_radius)]]:
                delta=b[axis]-a[axis]
                if delta and 0<(f:=(value-a[axis])/delta)<1: cuts.add(f)
        def lerp(f): return (a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1]))
        ordered=sorted(cuts)
        for f,g in zip(ordered,ordered[1:]):
            u,v,m=lerp(f),lerp(g),lerp((f+g)/2)
            if not self.is_free_world(*m): return False
            supported=any(float(self._strip_clearance(c,*m))>=self.blocked_radius-1e-8
                          and self._tube(c)[0]<=m[1]<=self._tube(c)[1]
                          for c in ([] if self.climb_mode=="ground" else self.corridors))
            if not self._swept_clear(self.static if supported else self.ground,u,v):
                return False
        return True

    def support(self,x,y):
        if self.climb_mode=="ground": return "ground",0.
        for c in self.corridors:
            if c["x0"]<=x<=c["x1"] and c["y0"]<=y<=c["y1"]:
                if y<c["stair_top"]:
                    z=min(450.,150.*(1+int((y-c["y0"])/300.)))
                    return f"stairs_{c['side']}",z
                if y<=c["ramp_top"]: return f"transfer_{c['side']}",600.
                return f"ramp_{c['side']}",600.*(c["y1"]-y)/(c["y1"]-c["ramp_top"])
        return "ground",0.

    def validate(self, points):
        xy=[(float(p.x),float(p.y)) if hasattr(p,"x") else tuple(p[:2]) for p in points]
        if len(xy)<2 or not all(self.is_free_world(*p) for p in xy):
            raise ValueError("点表含越界/越入L1或对方区域/无支承点，拒绝导出")
        for i,(a,b) in enumerate(zip(xy,xy[1:])):
            if not self.segment_is_free(a,b):
                raise ValueError(f"点表段{i}侧向进入爬升区或扫掠碰撞，拒绝导出")
        return [self.support(*p) for p in xy]
