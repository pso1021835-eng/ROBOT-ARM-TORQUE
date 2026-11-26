"""Unitree H2-X URDF → model.json (웹 시뮬레이터용)

사용:  python tools/make_model.py <unitree_ros>/robots/h2x_description/h2x.urdf
필요:  numpy, 로컬 MuJoCo 버전 h2x_arm_sim의 urdf_model.py (환경변수 H2X_SIM_DIR)

출력
- joints  : 오른팔 7축 (동역학용: origin, axis, 한계, 링크 관성)
- links   : 전신 링크 트리 (부모가 먼저). 관절 origin/axis/범위, 링크 프레임 기준 메시와 충돌 구(sphere)
- 충돌 구 : 각 링크 메시 정점을 k-means로 묶고, 묶음마다 중심 + 반경(정점 거리 85 백분위)으로 근사
"""
import hashlib
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.environ.get("H2X_SIM_DIR", "/home/claude/h2x_arm_sim"))
from h2x_arm_sim.urdf_model import (HAND_LINK, RIGHT_ARM_JOINTS, TORSO_LINK, URDFModel,  # noqa: E402
                                    load_stl_vertices, rpy_to_matrix, world_mesh_bounds)

L = lambda a, d=7: [round(float(v), d) for v in np.asarray(a).ravel()]


def fit_spheres(V, spacing=0.05, kmax=30, seed=0):
    """메시 정점(삼각형 순서) → 구 집합 [x, y, z, r] (링크 프레임).
    구 개수는 표면적 / spacing² 로 정해 표면을 빈틈 적게 덮는다."""
    T = V[: len(V) // 3 * 3].reshape(-1, 3, 3)
    area = 0.5 * np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1).sum()
    V = np.unique(np.round(V, 4), axis=0)
    if len(V) > 8000:
        V = V[np.random.default_rng(seed).choice(len(V), 8000, replace=False)]
    k = int(np.clip(round(area / spacing ** 2), 1, min(kmax, len(V) // 6)))
    rng = np.random.default_rng(seed)
    C = V[rng.choice(len(V), k, replace=False)]
    for _ in range(40):
        d = ((V[:, None, :] - C[None]) ** 2).sum(-1)
        a = d.argmin(1)
        C = np.array([V[a == i].mean(0) if np.any(a == i) else C[i] for i in range(k)])
    d = ((V[:, None, :] - C[None]) ** 2).sum(-1)
    a = d.argmin(1)
    out = []
    for i in range(k):
        P = V[a == i]
        if len(P) < 4:
            continue
        r = float(np.percentile(np.linalg.norm(P - C[i], axis=1), 85))
        out.append(L([*C[i], max(r, 0.008)], 4))
    return out


def main(urdf):
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "model.json")
    m = URDFModel(urdf)
    lo, _ = world_mesh_bounds(m, m.forward_kinematics({}))
    base_h = float(-lo[2])

    order = []
    stack = [m.root]
    while stack:
        ln = stack.pop(0)
        order.append(ln)
        stack += [j.child for j in m.child_joints[ln]]

    links = []
    for ln in order:
        link = m.links[ln]
        meshes, verts = [], []
        for v in link.visuals:
            if not v.mesh:
                continue
            Rv = rpy_to_matrix(v.rpy)
            meshes.append(dict(file=os.path.basename(v.mesh), R=L(Rv), p=L(v.xyz),
                               rgba=L(v.rgba if v.rgba is not None else [0.8, 0.8, 0.85, 1], 4)))
            Vl = load_stl_vertices(m.mesh_path(v.mesh))
            verts.append((Rv @ Vl.T).T + v.xyz)
        e = dict(name=ln, meshes=meshes)
        if ln in m.parent_joint:
            j = m.parent_joint[ln]
            e["parent"] = j.parent
            e["joint"] = dict(name=j.name, type=j.type, xyz=L(j.xyz), R0=L(rpy_to_matrix(j.rpy)),
                              axis=L(j.axis / max(np.linalg.norm(j.axis), 1e-12)),
                              lower=j.lower, upper=j.upper)
        if verts and ln not in ("left_ankle_pitch_link", "right_ankle_pitch_link"):
            kmax = 16 if ln.startswith("right_") and ln in [m.joints[j].child for j in RIGHT_ARM_JOINTS] else \
                60 if ln == TORSO_LINK else 30
            e["spheres"] = fit_spheres(np.vstack(verts), kmax=kmax)
        links.append(e)

    joints = []
    for jn in RIGHT_ARM_JOINTS:
        j = m.joints[jn]
        li = m.links[j.child].inertial
        joints.append(dict(name=jn, parent=j.parent, child=j.child, xyz=L(j.xyz, 10), rpy=L(j.rpy, 10),
                           R0=L(rpy_to_matrix(j.rpy), 10), axis=L(j.axis / np.linalg.norm(j.axis), 10),
                           lower=j.lower, upper=j.upper, effort=j.effort, velocity=j.velocity,
                           mass=li.mass, com=L(li.xyz, 10), inertia=L(li.inertia, 10)))
    T = m.forward_kinematics({}, base_pos=(0, 0, base_h))
    Rt, pt = T[TORSO_LINK]
    body_joints = [dict(name=j.name, lower=j.lower, upper=j.upper) for j in m.joints.values()
                   if j.type == "revolute" and j.name not in RIGHT_ARM_JOINTS]
    with open(urdf, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    model = dict(
        source=dict(repo="https://github.com/unitreerobotics/unitree_ros",
                    commit="157ff950351e5f535e03c5d85a16b7d594b3dbcc",
                    file="robots/h2x_description/h2x.urdf", sha256=sha,
                    license="BSD-3-Clause (LICENSE-unitree_ros.txt)"),
        base_height=base_h, gravity=[0, 0, -9.81],
        torso=dict(R=L(Rt, 10), p=L(pt, 10)), torso_link=TORSO_LINK,
        joints=joints, hand_link=HAND_LINK, body_joints=body_joints, links=links,
        total_mass=float(sum(l.inertial.mass for l in m.links.values() if l.inertial)),
    )
    with open(out, "w", encoding="utf-8") as f:
        json.dump(model, f, ensure_ascii=False, separators=(",", ":"))
    ns = sum(len(l.get("spheres", [])) for l in links)
    print(out, "links", len(links), "spheres", ns, "base_h", round(base_h, 4))


if __name__ == "__main__":
    main(sys.argv[1])
