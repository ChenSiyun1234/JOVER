"""Export the rover scene to a GLB for the browser replay viewer.

usage: python viz/web/export_glb.py      -> viz/web/rover.glb (not committed: it carries the board meshes)

Node names (z-up, metres; "__" because three.js strips ":" from node names):
  b__<body>__<material> visual geoms of one moving body, merged per material, in that body's frame;
                        the page sets each node's world pose from the trace (pos, quat of <body>)
  static__rocks         the 40 rocks, world frame
  static__terrain       the 16 m x 16 m heightfield, every second sample, sand colour shaded by slope
Collision-only geoms (group 3) are skipped.
"""
from collections import defaultdict
from pathlib import Path

import fast_simplification
import mujoco
import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
SCENE = HERE.parent / "scene" / "rover_scene.xml"
MAX_FACES = 8000
SAND = np.array([0.80, 0.66, 0.48])


def quat_mat(q):
    r = np.zeros(9)
    mujoco.mju_quat2Mat(r, np.asarray(q, float))
    return r.reshape(3, 3)


def geom_mesh(m, gid):
    t, size = int(m.geom_type[gid]), m.geom_size[gid]
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        k = int(m.geom_dataid[gid])
        v0, nv = int(m.mesh_vertadr[k]), int(m.mesh_vertnum[k])
        f0, nf = int(m.mesh_faceadr[k]), int(m.mesh_facenum[k])
        verts = m.mesh_vert[v0:v0 + nv].astype(np.float32)
        faces = m.mesh_face[f0:f0 + nf].astype(np.int32)
        if nf > MAX_FACES:
            verts, faces = fast_simplification.simplify(verts, faces, 1 - MAX_FACES / nf)
        mesh = trimesh.Trimesh(np.asarray(verts, float), np.asarray(faces, np.int64), process=False)
    elif t == mujoco.mjtGeom.mjGEOM_BOX:
        mesh = trimesh.creation.box(extents=2 * size[:3])
    elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        mesh = trimesh.creation.cylinder(radius=size[0], height=2 * size[1], sections=32)
    elif t == mujoco.mjtGeom.mjGEOM_CAPSULE:
        mesh = trimesh.creation.capsule(radius=size[0], height=2 * size[1], count=[16, 16])
    elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
        mesh = trimesh.creation.icosphere(radius=size[0], subdivisions=2)
    else:
        return None
    T = np.eye(4)
    T[:3, :3] = quat_mat(m.geom_quat[gid])
    T[:3, 3] = m.geom_pos[gid]
    mesh.apply_transform(T)
    return mesh


def material(m, gid):
    mat = int(m.geom_matid[gid])
    if mat >= 0:
        return m.mat(mat).name, np.asarray(m.mat_rgba[mat])
    rgba = np.asarray(m.geom_rgba[gid])
    return "rgba_" + "".join(f"{int(c * 15):x}" for c in rgba), rgba


def terrain(m):
    gid = m.geom("terrain").id
    h = int(m.geom_dataid[gid])
    nrow, ncol = int(m.hfield_nrow[h]), int(m.hfield_ncol[h])
    sx, sy, sz, _ = m.hfield_size[h]
    data = m.hfield_data[m.hfield_adr[h]:m.hfield_adr[h] + nrow * ncol].reshape(nrow, ncol)
    rows, cols = np.r_[0:nrow:2, nrow - 1], np.r_[0:ncol:2, ncol - 1]
    z = data[np.ix_(rows, cols)] * sz
    x = cols / (ncol - 1) * 2 * sx - sx
    y = rows / (nrow - 1) * 2 * sy - sy
    X, Y = np.meshgrid(x, y)
    verts = np.column_stack([X.ravel(), Y.ravel(), z.ravel()]) + m.geom_pos[gid]
    nr, nc = len(rows), len(cols)
    idx = np.arange(nr * nc).reshape(nr, nc)
    a, b, c, d = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, :-1].ravel(), idx[1:, 1:].ravel()
    faces = np.concatenate([np.column_stack([a, b, d]), np.column_stack([a, d, c])])
    mesh = trimesh.Trimesh(verts, faces, process=False)
    gy, gx = np.gradient(z, y, x)
    slope = np.clip(np.hypot(gx, gy).ravel() / 0.6, 0, 1)
    rng = np.random.default_rng(7)
    shade = 0.92 + 0.08 * rng.random(len(verts)) - 0.18 * slope
    rgb = np.clip(SAND[None] * shade[:, None], 0, 1)
    mesh.visual.vertex_colors = np.column_stack([(rgb * 255).astype(np.uint8), np.full(len(verts), 255, np.uint8)])
    return mesh


def main():
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    groups = defaultdict(list)
    for gid in range(m.ngeom):
        if m.geom_group[gid] == 3 or m.geom_type[gid] == mujoco.mjtGeom.mjGEOM_HFIELD:
            continue
        mesh = geom_mesh(m, gid)
        if mesh is None or len(mesh.faces) == 0:
            continue
        body = int(m.geom_bodyid[gid])
        name, rgba = material(m, gid)
        key = "static__rocks" if body == 0 else f"b__{m.body(body).name}__{name}"
        mesh.visual.face_colors = np.tile(np.clip(rgba * 255, 0, 255).astype(np.uint8), (len(mesh.faces), 1))
        groups[key].append(mesh)
    scene = trimesh.Scene()
    for key, meshes in sorted(groups.items()):
        merged = trimesh.util.concatenate(meshes)
        scene.add_geometry(merged, node_name=key, geom_name=key)
    scene.add_geometry(terrain(m), node_name="static__terrain", geom_name="static__terrain")
    out = HERE / "rover.glb"
    scene.export(out)
    faces = sum(len(g.faces) for g in scene.geometry.values())
    print(f"wrote {out} {out.stat().st_size // 1024} KB, {len(scene.geometry)} nodes, {faces} faces")


if __name__ == "__main__":
    main()
