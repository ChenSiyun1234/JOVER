"""Build the SSS URC rover scene for MuJoCo.

usage: python build_rover.py      -> scene/rover_scene.xml and its meshes, terrain and textures

Parts follow the drivebase BOM and team images (see ROVER_PARAMS.md): paired laser-cut acrylic rocker and
bogie plates on 25 mm standoffs, M20 pivot shafts, a curved acrylic differential bar with tie rods,
DM-J6006-2EC hub motors on printed brackets, printed six-spoke rims with TPU honeycomb tyres, an 8020
aluminium electronics frame with clear panels, and the real control board inside. Sizes the team has not
published are assumed and listed in ROVER_PARAMS.md.
"""
import re
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
import trimesh

import rover_parts as P

HERE = Path(__file__).resolve().parent
SCENE = HERE / "scene"

# ---- layout (metres; x forward, y left, z up; chassis frame unless noted) -----------------------
WHEEL_R, WHEEL_W = 0.100, 0.080
LINK_Y = 0.272                     # centre of each rocker/bogie plate pair
# plate pair half-stack 0.0185 + 3 mm gap, bracket 6 mm, motor 36.5 mm, flange 5 mm, rim plate 5 mm, plate offset 14.4 mm
MOTOR_FACE_DY = 0.0215 + 0.0425
TRACK_Y = LINK_Y + MOTOR_FACE_DY + 0.005 + 0.005 + 0.18 * WHEEL_W   # wheel centre, rim plate on the motor flange
RIDE_Z = 0.36                      # chassis origin above flat ground
AXLE_Z = WHEEL_R - RIDE_Z
PIVOT = np.array([0.0, 0.03])      # rocker pivot (x, z)
FRONT = np.array([0.42, AXLE_Z])
BOGIE = np.array([-0.20, -0.13])
MIDDLE = np.array([0.02, AXLE_Z])
REAR = np.array([-0.42, AXLE_Z])
BOX = np.array([0.60, 0.42, 0.20])  # electronics frame outer size
TIE_UP = np.array([-0.11, 0.135])   # tie-rod attachment on the rocker upright (x, z)
MASS = dict(chassis=38.0, rocker=2.2, bogie=2.6, wheel=0.9)
PEAK_TORQUE = 12.0                  # N m, DM-J6006-2EC
FRICTION = "0.9 0.01 0.0001"

# ---- terrain ----------------------------------------------------------------------------------
N, HALF, Z_MAX = 512, 8.0, 1.0


def smooth_noise(rng, n, scale_px):
    f = np.fft.fft2(rng.normal(size=(n, n)))
    k = np.fft.fftfreq(n)[:, None] ** 2 + np.fft.fftfreq(n)[None, :] ** 2
    out = np.real(np.fft.ifft2(f * np.exp(-k * (scale_px ** 2) * 2 * np.pi ** 2)))
    return out / out.std()


def terrain(seed=11):
    rng = np.random.default_rng(seed)
    x = np.linspace(-HALF, HALF, N)
    X, Y = np.meshgrid(x, x)
    h = 0.10 * smooth_noise(rng, N, 40) + 0.025 * smooth_noise(rng, N, 10) + 0.006 * smooth_noise(rng, N, 2.5)
    # a 15 degree slope up to a 0.45 m bench east of x = 3.2 m, across the driving lane
    bench = np.clip((X - 3.2) * np.tan(np.radians(15)), 0, 0.45) * np.exp(-(Y / 2.6) ** 6)
    h += bench
    # keep the start pad and the first metres of the lane gentle
    pad = np.exp(-((X - 0.8) ** 2 / 2.2 ** 2 + Y ** 2 / 1.2 ** 2))
    h = h * (1 - 0.8 * pad) + h[N // 2, N // 2] * 0.8 * pad
    h -= h.min()
    return h


def height_at(h, x, y):
    i = int(np.clip(round((y + HALF) / (2 * HALF) * (N - 1)), 0, N - 1))
    j = int(np.clip(round((x + HALF) / (2 * HALF) * (N - 1)), 0, N - 1))
    return float(h[i, j])


def write_terrain(h):
    with open(SCENE / "terrain.bin", "wb") as f:
        np.array([N, N], np.int32).tofile(f)
        (h / Z_MAX).astype(np.float32)[::-1].tofile(f)   # custom hfield files store the +y row first
    rng = np.random.default_rng(5)
    n = 1024
    base = 0.55 + 0.45 * (0.5 + 0.5 * np.tanh(smooth_noise(rng, n, 60)))
    grain = 0.06 * smooth_noise(rng, n, 1.2) + 0.05 * smooth_noise(rng, n, 6)
    yy, xx = np.mgrid[0:n, 0:n] / n
    ripples = 0.035 * np.sin(2 * np.pi * (xx * 38 + yy * 9 + 0.6 * smooth_noise(rng, n, 30) * 0.15))
    shade = np.clip(base + grain + ripples, 0, 1.2)
    sand = np.array([214, 160, 112], float)
    dark = np.array([150, 98, 66], float)
    col = dark[None, None] + (sand - dark)[None, None] * shade[..., None]
    imageio.imwrite(SCENE / "sand.png", np.clip(col, 0, 255).astype(np.uint8))


# ---- meshes -----------------------------------------------------------------------------------

def save(mesh, name):
    mesh.export(SCENE / f"{name}.stl")
    return name


def build_meshes(h):
    names = {}
    holes_r = [(PIVOT, 0.0105), (FRONT, 0.008), (BOGIE, 0.0105), (TIE_UP, 0.006)]
    rocker_pts = [FRONT, PIVOT, BOGIE]
    rocker = P.link_pair([tuple(p) for p in rocker_pts], tuple(PIVOT), [(tuple(p), r) for p, r in holes_r], 0.0)
    upright = P.extrude_xz(P.LineString([tuple(PIVOT), tuple(TIE_UP)]).buffer(0.016, quad_segs=12)
                           .union(P.Point(tuple(TIE_UP)).buffer(0.018, 32)).difference(P.Point(tuple(TIE_UP)).buffer(0.006, 24)),
                           0.006, 0.0)
    names["rocker"] = save(P.merge([rocker, upright]), "rocker")
    bm, br = MIDDLE - BOGIE, REAR - BOGIE
    bogie = P.link_pair([tuple(bm), (0.0, 0.0), tuple(br)], (0.0, 0.0),
                        [((0, 0), 0.0105), (tuple(bm), 0.008), (tuple(br), 0.008)], 0.0, boss_r=0.036)
    names["bogie"] = save(bogie, "bogie")
    so = [trimesh.creation.cylinder(0.0045, 0.025, sections=6, transform=P.rot_x(-90) @ np.eye(4)) for _ in range(1)][0]
    names["standoff"] = save(so, "standoff")
    diff, end_l, end_r = P.differential_bar()
    names["diffbar"] = save(diff, "diffbar")
    tyre, rim, screws = P.wheel_meshes(WHEEL_R, WHEEL_W)
    names.update(tyre=save(tyre, "tyre"), rim=save(rim, "rim"), wheel_screws=save(screws, "wheel_screws"))
    housing, cap, flange, plug = P.motor_meshes()
    names.update(motor=save(housing, "motor"), motor_cap=save(cap, "motor_cap"),
                 motor_flange=save(flange, "motor_flange"), motor_plug=save(plug, "motor_plug"))
    names["bracket"] = save(P.motor_bracket(), "bracket")
    rng = np.random.default_rng(23)
    rocks = []
    fixed = [(1.55, 0.385, 0.20), (2.55, -0.385, 0.17), (4.6, 0.6, 0.22)]   # in the driving lane
    while len(rocks) < 40:
        if fixed:
            x, y, s = fixed.pop(0)
        else:
            x, y = rng.uniform(-HALF + 0.6, HALF - 0.6, 2)
            s = rng.uniform(0.06, 0.26) * (1 + 0.8 * (rng.uniform() < 0.12))
            if abs(y) < 1.2 and -1.5 < x < 6.5 or np.hypot(x, y) < 1.4:
                continue
        name = f"rock{len(rocks):02d}"
        save(P.rock(rng, s), name)
        rocks.append((name, x, y, height_at(h, x, y), rng.uniform(0, 360)))
    return names, rocks, (end_l, end_r)


def board_geoms():
    """The control board meshes from board_mesh.py (kept out of git); a plain 100 x 100 mm board if absent."""
    src = HERE.parent / "board3d" / "mj"
    if not (src / "board.xml").exists():
        print("board3d/mj/board.xml not found: using a plain 100 x 100 mm board (see README)")
        return "", '          <geom class="vis" type="box" size="0.05 0.05 0.0008" rgba="0.08 0.30 0.16 1"/>'
    for stl in src.glob("board_c*.stl"):
        (SCENE / stl.name).write_bytes(stl.read_bytes())
    xml = (src / "board.xml").read_text()
    assets = re.findall(r'<mesh name="(board_c\d+)" file="(board_c\d+\.stl)"/>', xml)
    geoms = re.findall(r'<geom type="mesh" mesh="(board_c\d+)" rgba="([^"]+)"', xml)
    return ("\n".join(f'    <mesh name="{n}" file="{f}"/>' for n, f in assets),
            "\n".join(f'          <geom class="vis" type="mesh" mesh="{n}" rgba="{c}"/>' for n, c in geoms))


# ---- XML pieces -------------------------------------------------------------------------------

def g(attrs):
    return "<geom " + " ".join(f'{k}="{v}"' for k, v in attrs.items()) + "/>"


def fmt(v):
    return " ".join(f"{x:.4f}" for x in v)


def cable(pts, rgba, r=0.0028):
    return "\n".join(g({"class": "vis", "type": "capsule", "size": r, "rgba": rgba,
                        "fromto": fmt(list(a) + list(b))}) for a, b in zip(pts[:-1], pts[1:]))


def wheel_unit(side, tag, axle_xz, y_link):
    """Motor + bracket fixed to the link, wheel on its own hinge."""
    s = 1 if side == "l" else -1
    y_out = y_link + s * (0.0155 + 0.006)                      # outer face of the outer plate
    y_motor_face = y_out + s * (0.006 + 0.0365)                # output face of the motor
    y_wheel = s * TRACK_Y
    rot = "0 0 0" if s > 0 else "0 0 180"
    x, z = axle_xz
    hub = f"""
        {g({"class": "vis", "type": "mesh", "mesh": "bracket", "material": "petg", "pos": fmt([x, y_out + s * 0.003, z]), "euler": rot})}
        {g({"class": "vis", "type": "mesh", "mesh": "motor", "material": "motor", "pos": fmt([x, y_motor_face, z]), "euler": rot})}
        {g({"class": "vis", "type": "mesh", "mesh": "motor_cap", "material": "motor_cap", "pos": fmt([x, y_motor_face, z]), "euler": rot})}
        {g({"class": "vis", "type": "mesh", "mesh": "motor_flange", "material": "steel", "pos": fmt([x, y_motor_face, z]), "euler": rot})}
        {g({"class": "vis", "type": "mesh", "mesh": "motor_plug", "material": "plug", "pos": fmt([x, y_motor_face, z]), "euler": rot})}
        <body name="wheel_{side}{tag}" pos="{fmt([x, y_wheel, z])}">
          <inertial pos="0 0 0" mass="{MASS['wheel']}" diaginertia="0.0026 0.0045 0.0026"/>
          <joint name="drive_{side}{tag}" type="hinge" axis="0 1 0" damping="0.05" armature="0.02"/>
          {g({"name": f"tire_{side}{tag}", "class": "tire", "type": "cylinder", "size": f"{WHEEL_R} {WHEEL_W / 2}", "euler": "90 0 0"})}
          {g({"class": "vis", "type": "mesh", "mesh": "tyre", "material": "tpu", "euler": rot})}
          {g({"class": "vis", "type": "mesh", "mesh": "rim", "material": "petg", "euler": rot})}
          {g({"class": "vis", "type": "mesh", "mesh": "wheel_screws", "material": "steel", "euler": rot})}
        </body>"""
    return hub


def standoffs(points, y_link):
    return "\n".join(g({"class": "vis", "type": "mesh", "mesh": "standoff", "material": "brass",
                        "pos": fmt([p[0], y_link, p[1]])}) for p in points)


def side(side):
    s = 1 if side == "l" else -1
    y = s * LINK_Y
    rocker_so = P.standoff_points([tuple(FRONT), tuple(PIVOT), tuple(BOGIE)], tuple(PIVOT))
    bm, br = MIDDLE - BOGIE, REAR - BOGIE
    bogie_so = P.standoff_points([tuple(bm), (0, 0), tuple(br)], (0, 0))
    # cables: CAN (yellow/green pair) and power (red/black) along the rocker to the front motor and bogie
    can_r = [(PIVOT[0] + 0.03, y, PIVOT[1] - 0.02), ((FRONT[0] + PIVOT[0]) / 2, y, (FRONT[1] + PIVOT[1]) / 2 - 0.004),
             (FRONT[0] - 0.05, y + s * 0.03, FRONT[1] + 0.03)]
    can_b = [(PIVOT[0] - 0.03, y, PIVOT[1] - 0.02), (BOGIE[0] + 0.04, y, BOGIE[1] + 0.02)]
    return f"""
      <body name="rocker_{side}" pos="{PIVOT[0]} 0 {PIVOT[1]}">
        <joint name="rocker_{side}" type="hinge" axis="0 1 0" range="-35 35" damping="1.0"/>
        <inertial pos="0.08 {y:.3f} -0.10" mass="{MASS['rocker']}" diaginertia="0.04 0.045 0.008"/>
        <frame pos="{-PIVOT[0]} 0 {-PIVOT[1]}">
          {g({"class": "vis", "type": "mesh", "mesh": "rocker", "material": "acrylic_link", "pos": f"0 {y:.4f} 0"})}
          {standoffs(rocker_so, y)}
          {g({"class": "vis", "type": "cylinder", "size": "0.010 0.062", "material": "steel", "pos": f"{PIVOT[0]} {s * 0.248:.4f} {PIVOT[1]}", "euler": "90 0 0"})}
          {g({"class": "vis", "type": "cylinder", "size": "0.017 0.006", "material": "steel_dark", "pos": f"{PIVOT[0]} {y + s * 0.025:.4f} {PIVOT[1]}", "euler": "90 0 0"})}
          {cable(can_r, "0.95 0.80 0.10 1")}
          {cable([(a, b - s * 0.006, c) for a, b, c in can_r], "0.15 0.65 0.25 1")}
          {cable(can_b, "0.80 0.10 0.10 1", 0.0034)}
{wheel_unit(side, "f", FRONT, y)}
          <body name="bogie_{side}" pos="{BOGIE[0]} 0 {BOGIE[1]}">
            <joint name="bogie_{side}" type="hinge" axis="0 1 0" range="-40 40" damping="0.5"/>
            <inertial pos="-0.01 {y:.3f} -0.08" mass="{MASS['bogie']}" diaginertia="0.03 0.034 0.006"/>
            {g({"class": "vis", "type": "mesh", "mesh": "bogie", "material": "acrylic_link", "pos": f"0 {y:.4f} 0"})}
            {standoffs(bogie_so, y)}
            {g({"class": "vis", "type": "cylinder", "size": "0.010 0.036", "material": "steel", "pos": f"0 {y:.4f} 0", "euler": "90 0 0"})}
            {g({"class": "vis", "type": "cylinder", "size": "0.017 0.006", "material": "steel_dark", "pos": f"0 {y + s * 0.025:.4f} 0", "euler": "90 0 0"})}
            {cable([(0.03, y, -0.01), (bm[0] - 0.05, y + s * 0.03, bm[1] + 0.03)], "0.95 0.80 0.10 1")}
            {cable([(-0.03, y, -0.01), (br[0] + 0.05, y + s * 0.03, br[1] + 0.03)], "0.95 0.80 0.10 1")}
{wheel_unit(side, "m", bm, y)}
{wheel_unit(side, "r", br, y)}
          </body>
        </frame>
      </body>"""


def extrusion(a, b):
    """20 x 20 mm T-slot member from a to b with a dark slot on each face."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    out = [g({"class": "vis", "type": "box", "material": "alu", "pos": fmt((a + b) / 2), "size": fmt(np.where(a == b, 0.010, np.abs(b - a) / 2))})]
    axis = int(np.argmax(np.abs(b - a)))
    for k in range(3):
        if k == axis:
            continue
        for sgn in (-1, 1):
            off = np.zeros(3)
            off[k] = sgn * 0.0101
            size = np.where(a == b, 0.0001, np.abs(b - a) / 2 - 0.004)
            size[[i for i in range(3) if i not in (k, axis)][0]] = 0.0028
            out.append(g({"class": "vis", "type": "box", "material": "slot", "pos": fmt((a + b) / 2 + off), "size": fmt(size)}))
    return "\n      ".join(out)


def chassis_frame():
    hx, hy, hz = BOX / 2 - 0.010
    corners = [(sx * hx, sy * hy, sz * hz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    edges = []
    for i, a in enumerate(corners):
        for b in corners[i + 1:]:
            if sum(x != y for x, y in zip(a, b)) == 1:
                edges.append(extrusion(a, b))
    return "\n      ".join(edges)


def build():
    SCENE.mkdir(exist_ok=True)
    h = terrain()
    write_terrain(h)
    names, rocks, (end_l, end_r) = build_meshes(h)
    board_assets, board_xml = board_geoms()
    start = np.array([0.0, 0.0, height_at(h, 0, 0) + RIDE_Z + 0.03])
    hx, hy, hz = BOX / 2
    mesh_assets = "\n".join(f'    <mesh name="{n}" file="{n}.stl"/>' for n in names.values())
    rock_assets = "\n".join(f'    <mesh name="{n}" file="{n}.stl"/>' for n, *_ in rocks)
    rock_geoms = "\n".join(g({"name": n, "type": "mesh", "mesh": n, "material": "rock", "pos": fmt([x, y, z]),
                              "euler": f"0 0 {a:.0f}", "condim": "3", "friction": FRICTION}) for n, x, y, z, a in rocks)
    k_diff = (TIE_UP[1] - PIVOT[1]) / LINK_Y      # bar yaw per rocker pitch (small-angle lever ratio)
    tie = []
    ends = sorted([end_l, end_r], key=lambda e: e[1])          # (x, y) of the bar ends, -y first
    for side_name, end, s in (("l", ends[1], 1), ("r", ends[0], -1)):
        bar_end = np.array([end[0], end[1], hz + 0.035])
        rocker_end = np.array([TIE_UP[0], s * (LINK_Y - 0.026), TIE_UP[1]])
        tie.append(g({"class": "vis", "type": "capsule", "size": "0.005", "material": "steel", "fromto": fmt(list(bar_end) + list(rocker_end))}))
        tie.append(g({"class": "vis", "type": "sphere", "size": "0.009", "material": "steel_dark", "pos": fmt(bar_end)}))
        tie.append(g({"class": "vis", "type": "sphere", "size": "0.009", "material": "steel_dark", "pos": fmt(rocker_end)}))
    actuators = "\n".join(f'    <motor name="m_{s}{t}" joint="drive_{s}{t}" gear="1" ctrllimited="true" ctrlrange="-{PEAK_TORQUE} {PEAK_TORQUE}"/>'
                          for s in "lr" for t in "fmr")
    sensors = "\n".join(f'    <jointvel name="w_{s}{t}" joint="drive_{s}{t}"/>' for s in "lr" for t in "fmr")

    xml = f"""<mujoco model="sss_rover">
  <!-- Generated by build_rover.py; sources and assumptions in ROVER_PARAMS.md -->
  <compiler angle="degree" meshdir="." texturedir="." inertiafromgeom="false"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="1"/>
  <statistic center="0 0 0.3" extent="4"/>
  <visual>
    <global offwidth="1920" offheight="1080" azimuth="135" elevation="-14" fovy="40"/>
    <quality shadowsize="8192" offsamples="8"/>
    <headlight ambient="0.32 0.30 0.28" diffuse="0.18 0.18 0.18" specular="0.05 0.05 0.05"/>
    <map znear="0.005" zfar="80" fogstart="1.3" fogend="2.9"/>
    <rgba fog="0.86 0.80 0.72 1" haze="0.93 0.86 0.78 1"/>
  </visual>
  <default>
    <default class="vis"><geom contype="0" conaffinity="0" group="1"/></default>
    <default class="tire"><geom contype="1" conaffinity="1" condim="4" friction="{FRICTION}" rgba="0 0 0 0" group="3"
      solref="0.008 1" solimp="0.9 0.95 0.001"/></default>
  </default>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.40 0.60 0.86" rgb2="0.94 0.86 0.76" width="800" height="800"/>
    <texture name="sand" type="2d" file="sand.png"/>
    <material name="sand" texture="sand" texrepeat="10 10" specular="0.02" shininess="0.01" reflectance="0"/>
    <material name="rock" rgba="0.55 0.40 0.31 1" specular="0.08" shininess="0.05"/>
    <material name="acrylic_link" rgba="0.62 0.82 0.96 0.50" specular="0.9" shininess="0.95" reflectance="0.04"/>
    <material name="acrylic_panel" rgba="0.82 0.92 1.0 0.16" specular="0.9" shininess="0.95"/>
    <material name="petg" rgba="0.98 0.42 0.05 1" specular="0.35" shininess="0.4"/>
    <material name="tpu" rgba="0.17 0.17 0.18 1" specular="0.25" shininess="0.15"/>
    <material name="alu" rgba="0.80 0.81 0.83 1" specular="0.7" shininess="0.55"/>
    <material name="slot" rgba="0.28 0.29 0.31 1" specular="0.3"/>
    <material name="steel" rgba="0.70 0.71 0.74 1" specular="0.9" shininess="0.85"/>
    <material name="steel_dark" rgba="0.35 0.36 0.38 1" specular="0.8" shininess="0.7"/>
    <material name="brass" rgba="0.80 0.65 0.30 1" specular="0.8" shininess="0.7"/>
    <material name="motor" rgba="0.17 0.17 0.19 1" specular="0.7" shininess="0.6"/>
    <material name="motor_cap" rgba="0.06 0.06 0.07 1" specular="0.4"/>
    <material name="plug" rgba="0.95 0.78 0.10 1"/>
    <material name="floor_plate" rgba="0.12 0.13 0.15 1" specular="0.2"/>
    <material name="battery" rgba="0.10 0.11 0.13 1" specular="0.4"/>
    <material name="battery_label" rgba="0.96 0.74 0.08 1"/>
    <material name="estop" rgba="0.85 0.06 0.05 1" specular="0.6" shininess="0.6"/>
    <material name="estop_base" rgba="0.98 0.80 0.05 1" specular="0.3"/>
    <material name="camera" rgba="0.12 0.13 0.15 1" specular="0.6"/>
    <material name="lens" rgba="0.02 0.03 0.05 1" specular="1" shininess="1" reflectance="0.3"/>
    <material name="gnss" rgba="0.92 0.92 0.90 1" specular="0.3"/>
    <material name="led" rgba="0.2 1.0 0.3 1" emission="1"/>
    <hfield name="terrain" file="terrain.bin" size="{HALF} {HALF} {Z_MAX} 0.3"/>
{mesh_assets}
{rock_assets}
{board_assets}
  </asset>
  <worldbody>
    <light name="sun" directional="true" pos="0 0 10" dir="0.30 -0.22 -0.93" diffuse="0.95 0.90 0.80" specular="0.35 0.33 0.30" castshadow="true"/>
    <light name="sky_fill" directional="true" pos="0 0 10" dir="-0.3 0.4 -1" diffuse="0.22 0.25 0.30" specular="0 0 0" castshadow="false"/>
    <geom name="terrain" type="hfield" hfield="terrain" material="sand" condim="3" friction="{FRICTION}"/>
{rock_geoms}
    <body name="chassis" pos="{fmt(start)}">
      <freejoint name="chassis"/>
      <inertial pos="0 0 -0.03" mass="{MASS['chassis']}" diaginertia="0.69 1.27 1.70"/>
      <!-- 8020 frame, clear panels, floor -->
      {chassis_frame()}
      {g({"class": "vis", "type": "box", "material": "floor_plate", "pos": f"0 0 {-hz + 0.012}", "size": f"{hx - 0.02} {hy - 0.02} 0.003"})}
      {g({"class": "vis", "type": "box", "material": "acrylic_panel", "pos": f"0 0 {hz - 0.002}", "size": f"{hx - 0.02} {hy - 0.02} 0.003"})}
      {g({"class": "vis", "type": "box", "material": "acrylic_panel", "pos": f"0 {hy - 0.010} 0", "size": f"{hx - 0.02} 0.002 {hz - 0.02}"})}
      {g({"class": "vis", "type": "box", "material": "acrylic_panel", "pos": f"0 {-hy + 0.010} 0", "size": f"{hx - 0.02} 0.002 {hz - 0.02}"})}
      {g({"class": "vis", "type": "box", "material": "acrylic_panel", "pos": f"{hx - 0.010} 0 0", "size": f"0.002 {hy - 0.02} {hz - 0.02}"})}
      {g({"class": "vis", "type": "box", "material": "acrylic_panel", "pos": f"{-hx + 0.010} 0 0", "size": f"0.002 {hy - 0.02} {hz - 0.02}"})}
      <!-- printed bearing holders for the rocker shafts -->
      {g({"class": "vis", "type": "cylinder", "material": "petg", "size": "0.032 0.016", "pos": f"{PIVOT[0]} {hy + 0.016} {PIVOT[1]}", "euler": "90 0 0"})}
      {g({"class": "vis", "type": "cylinder", "material": "petg", "size": "0.032 0.016", "pos": f"{PIVOT[0]} {-hy - 0.016} {PIVOT[1]}", "euler": "90 0 0"})}
      <!-- electronics: 24 V pack, control board on brass standoffs, wiring to the side bulkheads -->
      {g({"class": "vis", "type": "box", "material": "battery", "pos": f"-0.15 0 {-hz + 0.055}", "size": "0.12 0.075 0.040"})}
      {g({"class": "vis", "type": "box", "material": "battery_label", "pos": f"-0.15 0 {-hz + 0.0955}", "size": "0.121 0.076 0.0015"})}
      {cable([(-0.27, 0.03, -hz + 0.07), (-0.05, 0.03, -hz + 0.05), (0.06, 0.04, -hz + 0.035)], "0.75 0.05 0.05 1", 0.004)}
      {cable([(-0.27, -0.03, -hz + 0.07), (-0.05, -0.03, -hz + 0.05), (0.06, -0.04, -hz + 0.035)], "0.05 0.05 0.05 1", 0.004)}
      <body name="control_board" pos="0.12 0 {-hz + 0.040}">
        <inertial pos="0 0 0" mass="0.05" diaginertia="1e-5 1e-5 2e-5"/>
{board_xml}
        {"".join(g({"class": "vis", "type": "cylinder", "material": "brass", "size": "0.003 0.010", "pos": f"{sx * 0.045} {sy * 0.045} -0.010"}) for sx in (-1, 1) for sy in (-1, 1))}
        <site name="imu" pos="0 0 0.002" size="0.003"/>
      </body>
      {cable([(0.17, 0.05, -hz + 0.045), (0.12, 0.17, -hz + 0.06), (0.0, hy - 0.02, PIVOT[1] - 0.01)], "0.95 0.80 0.10 1")}
      {cable([(0.17, -0.05, -hz + 0.045), (0.12, -0.17, -hz + 0.06), (0.0, -hy + 0.02, PIVOT[1] - 0.01)], "0.95 0.80 0.10 1")}
      <!-- differential: bracket on the deck, curved bar on a centre pin, tie rods to the rocker uprights -->
      {g({"class": "vis", "type": "box", "material": "acrylic_link", "pos": f"0.03 0 {hz + 0.012}", "size": "0.04 0.05 0.012"})}
      {g({"class": "vis", "type": "cylinder", "material": "steel", "size": "0.005 0.03", "pos": f"0.07 0 {hz + 0.03}"})}
      <body name="diff_bar" pos="0 0 0">
        <joint name="diff" type="hinge" axis="0 0 1" pos="0.07 0 0" damping="0.1"/>
        <inertial pos="0.04 0 {hz + 0.035}" mass="0.4" diaginertia="0.01 0.0005 0.01"/>
        {g({"class": "vis", "type": "mesh", "mesh": "diffbar", "material": "acrylic_link", "pos": f"0 0 {hz + 0.035}"})}
        {"".join(tie)}
      </body>
      <!-- URC kit: e-stop, camera mast with stereo camera, GNSS puck, antenna, status light -->
      {g({"class": "vis", "type": "box", "material": "estop_base", "pos": f"-0.24 0.13 {hz + 0.022}", "size": "0.032 0.032 0.022"})}
      {g({"class": "vis", "type": "cylinder", "material": "estop", "size": "0.024 0.008", "pos": f"-0.24 0.13 {hz + 0.054}"})}
      {g({"class": "vis", "type": "box", "material": "alu", "pos": f"0.24 0 {hz + 0.11}", "size": "0.010 0.010 0.11"})}
      {g({"class": "vis", "type": "box", "material": "camera", "pos": f"0.255 0 {hz + 0.235}", "size": "0.016 0.075 0.016"})}
      {g({"class": "vis", "type": "cylinder", "material": "camera", "size": "0.016 0.016", "pos": f"0.255 0.075 {hz + 0.235}"})}
      {g({"class": "vis", "type": "cylinder", "material": "camera", "size": "0.016 0.016", "pos": f"0.255 -0.075 {hz + 0.235}"})}
      {g({"class": "vis", "type": "cylinder", "material": "lens", "size": "0.010 0.003", "pos": f"0.272 0.06 {hz + 0.235}", "euler": "0 90 0"})}
      {g({"class": "vis", "type": "cylinder", "material": "lens", "size": "0.010 0.003", "pos": f"0.272 -0.06 {hz + 0.235}", "euler": "0 90 0"})}
      {g({"class": "vis", "type": "cylinder", "material": "alu", "size": "0.006 0.05", "pos": f"-0.22 -0.12 {hz + 0.05}"})}
      {g({"class": "vis", "type": "cylinder", "material": "gnss", "size": "0.035 0.010", "pos": f"-0.22 -0.12 {hz + 0.105}"})}
      {g({"class": "vis", "type": "capsule", "material": "camera", "size": "0.0035", "fromto": f"-0.27 0.0 {hz} -0.27 0.0 {hz + 0.34}"})}
      {g({"class": "vis", "type": "cylinder", "material": "led", "size": "0.008 0.010", "pos": f"-0.20 0.0 {hz + 0.012}"})}
{side("l")}
{side("r")}
    </body>
  </worldbody>
  <equality>
    <joint joint1="rocker_l" joint2="rocker_r" polycoef="0 -1 0 0 0"/>
    <joint joint1="diff" joint2="rocker_l" polycoef="0 {k_diff:.4f} 0 0 0"/>
  </equality>
  <actuator>
{actuators}
  </actuator>
  <sensor>
    <accelerometer name="accel" site="imu"/>
    <gyro name="gyro" site="imu"/>
{sensors}
  </sensor>
</mujoco>
"""
    (SCENE / "rover_scene.xml").write_text(xml)
    return SCENE / "rover_scene.xml"


if __name__ == "__main__":
    path = build()
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(path))
    print(f"{path.name}: nbody {m.nbody}, ngeom {m.ngeom}, nmesh {m.nmesh}, nu {m.nu}, total mass {sum(m.body_mass):.1f} kg")
