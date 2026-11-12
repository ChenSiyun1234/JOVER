"""Turn the KiCad GLB export of the rover control board into a few coloured meshes for MuJoCo.

usage: python board_mesh.py <single_board.glb> <out_dir> [--faces 160000]

KiCad exports Y-up in metres with an arbitrary origin. Output: one STL per colour (merged and
decimated), centred on the board, Z up (board normal), plus board.xml (a MuJoCo <body> fragment) and
a preview PNG. Colours are kept from the KiCad materials.
"""
import argparse
from collections import defaultdict
from pathlib import Path

import fast_simplification
import mujoco
import numpy as np
import trimesh


def colour_of(geom):
    mat = getattr(geom.visual, "material", None)
    c = getattr(mat, "baseColorFactor", None)
    if c is None:
        return (0.5, 0.5, 0.5, 1.0)
    c = np.asarray(c, float)
    if c.max() > 1:
        c = c / 255
    return tuple(np.round(c, 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("glb")
    ap.add_argument("out")
    ap.add_argument("--faces", type=int, default=160000)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    scene = trimesh.load(args.glb)
    groups = defaultdict(list)
    for node in scene.graph.nodes_geometry:
        tf, name = scene.graph[node]
        g = scene.geometry[name].copy()
        g.apply_transform(tf)
        groups[colour_of(g)].append(g)

    merged = {c: trimesh.util.concatenate(gs) for c, gs in groups.items()}
    centre = scene.bounds.mean(axis=0)
    y_up_to_z_up = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
    total = sum(len(m.faces) for m in merged.values())
    budget = args.faces / total
    parts = []
    for k, (c, m) in enumerate(sorted(merged.items(), key=lambda kv: -len(kv[1].faces))):
        v = (m.vertices - centre) @ y_up_to_z_up.T
        f = m.faces
        keep = max(200, int(len(f) * budget))
        if len(f) > keep:
            v, f = fast_simplification.simplify(v.astype(np.float32), f.astype(np.int32), 1 - keep / len(f))
        mesh = trimesh.Trimesh(v, f, process=True)
        if len(mesh.faces) < 4:
            continue
        name = f"board_c{k:02d}"
        mesh.export(out / f"{name}.stl")
        parts.append((name, c, len(mesh.faces)))
        print(f"{name}: rgba {c}  faces {len(m.faces)} -> {len(mesh.faces)}")

    assets = "\n".join(f'    <mesh name="{n}" file="{n}.stl"/>' for n, _, _ in parts)
    geoms = "\n".join(
        f'      <geom type="mesh" mesh="{n}" rgba="{c[0]} {c[1]} {c[2]} {c[3] if len(c) > 3 else 1}" '
        f'contype="0" conaffinity="0" mass="0"/>' for n, c, _ in parts)
    xml = f"""<mujoco model="rover_board">
  <compiler meshdir="."/>
  <visual><global offwidth="1600" offheight="1000"/><quality shadowsize="4096"/></visual>
  <asset>
{assets}
  </asset>
  <worldbody>
    <light pos="0.1 -0.2 0.4" dir="-0.2 0.4 -1" diffuse="0.9 0.9 0.9"/>
    <light pos="-0.2 0.1 0.3" dir="0.5 -0.2 -1" diffuse="0.4 0.4 0.45"/>
    <body name="control_board">
{geoms}
    </body>
  </worldbody>
</mujoco>
"""
    (out / "board.xml").write_text(xml)
    model = mujoco.MjModel.from_xml_path(str(out / "board.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, 1000, 1600) as r:
        cam = mujoco.MjvCamera()
        cam.lookat[:] = (0, 0, 0)
        cam.distance, cam.azimuth, cam.elevation = 0.19, 120, -42
        r.update_scene(data, camera=cam)
        import imageio.v2 as imageio
        imageio.imwrite(out / "board_preview.png", r.render())
    print("wrote", out / "board_preview.png")


if __name__ == "__main__":
    main()
