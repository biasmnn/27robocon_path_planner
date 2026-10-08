"""Read field OBJ components and SDF primitives; all source coordinates are metres."""
import os
from pathlib import Path
import xml.etree.ElementTree as E

SIM_ROOT = Path(os.environ.get(
    "ROBOCON_SIM_ROOT",
    str(Path(__file__).resolve().parents[3] / "27th_gap_gazebo_model/gazebo_models-main")
)).expanduser().resolve()

def obj_components(index, root=SIM_ROOT):
    vertices, faces = [], []
    for line in (Path(root)/"robocon_mujoco/meshes"/f"robocon_{index:03}.obj").read_text(encoding="utf-8").splitlines():
        if line.startswith("v "):
            vertices.append(tuple(map(float, line.split()[1:4])))
        elif line.startswith("f "):
            faces.append([int(t.split("/")[0])-1 for t in line.split()[1:]])
    if not vertices:
        raise ValueError(f"Empty mesh {index}")
    parent = list(range(len(vertices)))
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for face in faces:
        for i in face[1:]:
            parent[find(i)] = find(face[0])
    groups = {}
    for i, point in enumerate(vertices):
        groups.setdefault(find(i), []).append(point)
    return [(tuple(min(p[k] for p in group) for k in range(3)),
             tuple(max(p[k] for p in group) for k in range(3))) for group in groups.values()]

def obj_bounds(index, root=SIM_ROOT):
    parts = obj_components(index, root)
    return (tuple(min(lo[k] for lo, hi in parts) for k in range(3)),
            tuple(max(hi[k] for lo, hi in parts) for k in range(3)))

def sdf_box(name, root=SIM_ROOT):
    model = E.parse(Path(root)/"robocon_ground/model.sdf").getroot().find("model")
    link = model.find(f"link[@name='{name}']")
    collision = link.find("collision")
    if model.find("pose") is not None or link.find("pose") is not None:
        raise ValueError("Resolve model/link frames before generating planner geometry")
    pose_node = collision.find("pose")
    if pose_node is not None and pose_node.get("relative_to"):
        raise ValueError("Relative collision frame is unsupported")
    pose = list(map(float, collision.findtext("pose", "0 0 0 0 0 0").split()))
    if any(abs(v)>1e-12 for v in pose[3:]):
        raise ValueError(f"{name}: expected axis-aligned primitive")
    size = list(map(float, collision.findtext("geometry/box/size").split()))
    return tuple(pose[k]-size[k]/2 for k in range(3)), tuple(pose[k]+size[k]/2 for k in range(3))

def colored_ground_rectangles(index, root=SIM_ROOT):
    return [b for b in obj_components(index, root)
            if b[1][2]-b[0][2]<1e-6 and abs(b[0][2]-.001)<1e-6]

def tower_bounds(root=SIM_ROOT):
    # Each tower is split into adjoining red/blue half-boxes by material.
    red = [b for b in obj_components(2, root) if b[1][2]-b[0][2]>.19]
    blue = [b for b in obj_components(3, root) if b[1][2]-b[0][2]>.19]
    towers = []
    used = set()
    for a, b in red:
        matches = [(i,c,d) for i,(c,d) in enumerate(blue)
                   if abs(a[0]-c[0])<1e-8 and abs(b[0]-d[0])<1e-8
                   and (abs(b[1]-c[1])<1e-8 or abs(d[1]-a[1])<1e-8)]
        if len(matches)!=1:
            raise ValueError("Cannot uniquely pair two-color tower halves")
        i,c,d = matches[0]
        if i in used: raise ValueError("Duplicate tower half")
        used.add(i)
        towers.append((tuple(min(a[k],c[k]) for k in range(3)),
                       tuple(max(b[k],d[k]) for k in range(3))))
    if len(towers)!=12 or len(used)!=len(blue):
        raise ValueError("Expected 12 two-color towers")
    return sorted(towers)

