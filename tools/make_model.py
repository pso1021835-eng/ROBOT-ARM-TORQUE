"""Unitree H2-X URDF → model.json (웹 시뮬레이터용).

사용: python tools/make_model.py <unitree_ros>/robots/h2x_description/h2x.urdf
URDF 해석은 h2x_arm_sim(로컬 MuJoCo 버전)의 urdf_model.py와 동일한 규칙.
"""
import hashlib, json, os, sys
import numpy as np
sys.path.insert(0, os.environ.get("H2X_SIM_DIR", "/home/claude/h2x_arm_sim"))
from h2x_arm_sim.urdf_model import (URDFModel, RIGHT_ARM_JOINTS, HAND_LINK, TORSO_LINK,
                                    rpy_to_matrix, world_mesh_bounds)

urdf = sys.argv[1]
out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "model.json")
m = URDFModel(urdf)
T0 = m.forward_kinematics({})
lo, hi = world_mesh_bounds(m, T0)
base_h = float(-lo[2])
T = m.forward_kinematics({}, base_pos=(0, 0, base_h))
arm_links = [m.joints[j].child for j in RIGHT_ARM_JOINTS]
L = lambda a: [round(float(v), 10) for v in np.asarray(a).ravel()]

static_meshes, arm_meshes = [], []
for ln, link in m.links.items():
    for v in link.visuals:
        if not v.mesh:
            continue
        Rv = rpy_to_matrix(v.rpy)
        entry = dict(file=os.path.basename(v.mesh), rgba=L(v.rgba if v.rgba is not None else [0.8, 0.8, 0.85, 1]))
        if ln in arm_links:
            entry.update(link=arm_links.index(ln), R=L(Rv), p=L(v.xyz))
            arm_meshes.append(entry)
        else:
            R, p = T[ln]
            entry.update(R=L(R @ Rv), p=L(p + R @ v.xyz), link_name=ln)
            static_meshes.append(entry)

joints = []
for jn in RIGHT_ARM_JOINTS:
    j = m.joints[jn]
    li = m.links[j.child].inertial
    joints.append(dict(name=jn, parent=j.parent, child=j.child, xyz=L(j.xyz), rpy=L(j.rpy),
                       R0=L(rpy_to_matrix(j.rpy)), axis=L(j.axis / np.linalg.norm(j.axis)),
                       lower=j.lower, upper=j.upper, effort=j.effort, velocity=j.velocity,
                       mass=li.mass, com=L(li.xyz), inertia=L(li.inertia)))
Rt, pt = T[TORSO_LINK]
with open(urdf, "rb") as f:
    sha = hashlib.sha256(f.read()).hexdigest()
model = dict(
    source=dict(repo="https://github.com/unitreerobotics/unitree_ros",
                commit="157ff950351e5f535e03c5d85a16b7d594b3dbcc",
                file="robots/h2x_description/h2x.urdf", sha256=sha, license="BSD-3-Clause (LICENSE-unitree_ros.txt)"),
    base_height=base_h, gravity=[0, 0, -9.81],
    torso=dict(R=L(Rt), p=L(pt)),
    joints=joints, hand_link=HAND_LINK,
    total_mass=float(sum(l.inertial.mass for l in m.links.values() if l.inertial)),
    static_meshes=static_meshes, arm_meshes=arm_meshes,
)
with open(out, "w", encoding="utf-8") as f:
    json.dump(model, f, ensure_ascii=False, separators=(",", ":"))
print(out, len(static_meshes), len(arm_meshes), base_h)
