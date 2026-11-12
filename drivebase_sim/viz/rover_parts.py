"""Procedural meshes for the SSS rover: laser-cut acrylic links, the differential bar, 3D-printed
wheels (orange PETG rim, black TPU honeycomb tyre), DM-J6006-2EC hub motors, printed motor brackets,
and rocks. Units are metres. Plates drawn in the side (x, z) plane are extruded along y.
"""
import numpy as np
import trimesh
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union


# ---------------------------------------------------------------- helpers

def extrude_xz(poly, thickness, y_center=0.0):
    """Extrude a polygon drawn in the (x, z) plane into a plate of the given thickness centred at y."""
    m = trimesh.creation.extrude_polygon(poly, thickness)
    v = m.vertices
    # (u, v, w) -> (u, w, v) swaps two axes, so reverse the winding to keep normals outward
    return trimesh.Trimesh(np.column_stack([v[:, 0], v[:, 2] - thickness / 2 + y_center, v[:, 1]]),
                           m.faces[:, ::-1], process=True)


def extrude_xy(poly, thickness, z_center=0.0):
    m = trimesh.creation.extrude_polygon(poly, thickness)
    m.apply_translation([0, 0, z_center - thickness / 2])
    return m


def rot_x(deg):
    return trimesh.transformations.rotation_matrix(np.radians(deg), [1, 0, 0])


def rot_y(deg):
    return trimesh.transformations.rotation_matrix(np.radians(deg), [0, 1, 0])


def rot_z(deg):
    return trimesh.transformations.rotation_matrix(np.radians(deg), [0, 0, 1])


def hexagon(c, r, phase=0.0):
    a = phase + np.arange(6) * np.pi / 3
    return Polygon(np.column_stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)]))


def annulus(r_out, r_in, res=96):
    return Point(0, 0).buffer(r_out, res).difference(Point(0, 0).buffer(r_in, res))


def merge(meshes):
    return trimesh.util.concatenate([m for m in meshes if m is not None and len(m.faces)])


# ---------------------------------------------------------------- links

def link_outline(points, width, apex, boss_r, end_r, holes, slot_w=None):
    """Outline of a laser-cut link along a polyline, with a round boss at the apex, round ends,
    the given holes, and lightening slots along each long segment."""
    body = LineString(points).buffer(width / 2, quad_segs=24)
    for p in points:
        body = body.union(Point(p).buffer(end_r, 48))
    if apex is not None:
        body = body.union(Point(apex).buffer(boss_r, 64))
    cuts = [Point(p).buffer(r, 48) for p, r in holes]
    slot_w = slot_w or width * 0.36
    for a, b in zip(points[:-1], points[1:]):
        a, b = np.asarray(a), np.asarray(b)
        length = np.linalg.norm(b - a)
        if length < 0.12:
            continue
        d = (b - a) / length
        n = max(1, int((length - 0.10) / 0.075))
        for s in np.linspace(0.30, 0.72, n) if n > 1 else [0.5]:
            c = a + d * length * s
            cuts.append(LineString([c - d * 0.022, c + d * 0.022]).buffer(slot_w / 2, quad_segs=16))
    return body.difference(unary_union(cuts))


def link_pair(points, apex, holes, y_center, gap=0.025, plate=0.006, width=0.044, boss_r=0.042, end_r=0.030):
    """Two acrylic plates (the BOM's rocker/bogie frames) separated by 25 mm standoffs."""
    outline = link_outline(points, width, apex, boss_r, end_r, holes)
    off = (gap + plate) / 2
    return merge([extrude_xz(outline, plate, y_center + off), extrude_xz(outline, plate, y_center - off)])


def standoff_points(points, apex, every=0.085):
    """Where the 25 mm M6 standoffs go: along each segment, away from the holes."""
    pts = []
    for a, b in zip(points[:-1], points[1:]):
        a, b = np.asarray(a), np.asarray(b)
        length = np.linalg.norm(b - a)
        n = max(1, int(length / every))
        d = (b - a) / length
        normal = np.array([-d[1], d[0]])
        for s in np.linspace(0.18, 0.82, n):
            c = a + (b - a) * s
            pts += [c + normal * 0.013, c - normal * 0.013]
    if apex is not None:
        for ang in (45, 135, 225, 315):
            pts.append(np.asarray(apex) + 0.028 * np.array([np.cos(np.radians(ang)), np.sin(np.radians(ang))]))
    return pts


def differential_bar(span=0.56, bow=0.07, width=0.05, plate=0.006, gap=0.0192):
    """Curved bar (top view, bowed toward +x) with triangular lightening holes; two plates 19.2 mm apart
    (a model value)."""
    y = np.linspace(-span / 2, span / 2, 41)
    x = bow * (1 - (2 * y / span) ** 2)
    path = LineString(np.column_stack([x, y]))
    body = path.buffer(width / 2, quad_segs=24).union(Point(x[20], y[20]).buffer(0.045, 64))
    for end in (0, -1):
        body = body.union(Point(x[end], y[end]).buffer(0.022, 48))
    cuts = [Point(x[20], y[20]).buffer(0.006, 32), Point(x[0], y[0]).buffer(0.006, 32), Point(x[-1], y[-1]).buffer(0.006, 32)]
    for i, k in enumerate(range(4, 37, 3)):
        if 17 <= k <= 23:
            continue
        cx, cy = x[k], y[k]
        t = np.array([x[k + 1] - x[k - 1], y[k + 1] - y[k - 1]])
        t /= np.linalg.norm(t)
        nrm = np.array([-t[1], t[0]])
        flip = 1 if i % 2 else -1
        tri = Polygon([(cx, cy) + t * 0.017 - nrm * 0.013 * flip, (cx, cy) - t * 0.017 - nrm * 0.013 * flip,
                       (cx, cy) + nrm * 0.013 * flip])
        cuts.append(tri.buffer(-0.002).buffer(0.002))
    outline = body.difference(unary_union(cuts))
    off = (gap + plate) / 2
    return merge([extrude_xy(outline, plate, off), extrude_xy(outline, plate, -off)]), (x[0], y[0]), (x[-1], y[-1])


# ---------------------------------------------------------------- wheel

def wheel_meshes(r_out=0.100, width=0.080, r_in=0.071):
    """Returns (tyre, rim, screws); axis along y, outer face toward +y."""
    # TPU tyre: annulus with two rings of honeycomb cells through the width, plus chevron lugs
    tyre_poly = annulus(r_out - 0.006, r_in, 128)
    cells = []
    for ring, (rad, n) in enumerate(((r_in + 0.0075, 40), (r_in + 0.0175, 46))):
        for k in range(n):
            a = 2 * np.pi * (k + 0.5 * ring) / n
            cells.append(hexagon((rad * np.cos(a), rad * np.sin(a)), 0.0047, a))
    tyre_poly = tyre_poly.difference(unary_union(cells))
    tyre = extrude_xz(tyre_poly, width * 0.96)
    lugs = []
    for k in range(22):
        a = 360 * k / 22
        for side in (-1, 1):
            lug = trimesh.creation.box([0.011, width * 0.47, 0.008])
            lug.apply_transform(rot_z(side * 22))
            lug.apply_translation([0, side * width * 0.235, r_out - 0.006 + 0.0035])
            lug.apply_transform(rot_y(a))
            lugs.append(lug)
    tyre = merge([tyre] + lugs)

    # PETG rim: support band under the tyre, six-spoke face plate with hub bosses, retaining ring
    band = extrude_xz(annulus(r_in, r_in - 0.005), width * 0.9)
    spokes = [Point(0, 0).buffer(0.036, 64)]
    for k in range(6):
        a = np.radians(60 * k + 30)
        d = np.array([np.cos(a), np.sin(a)])
        spokes.append(LineString([d * 0.02, d * (r_in - 0.004)]).buffer(0.0065, quad_segs=8))
    face = unary_union(spokes + [annulus(r_in - 0.002, r_in - 0.011)])
    holes = [Point(0, 0).buffer(0.010, 32)] + [Point(0.024 * np.cos(np.radians(60 * k)), 0.024 * np.sin(np.radians(60 * k))).buffer(0.0028, 16)
                                              for k in range(6)]
    face = face.difference(unary_union(holes))
    face_plate = extrude_xz(face, 0.010, -width * 0.18)          # inboard, bolted to the motor flange
    ring = extrude_xz(annulus(r_in + 0.012, r_in - 0.009), 0.004, width / 2 + 0.002)
    rim = merge([band, face_plate, ring])

    screws = []
    for k in range(24):
        a = 2 * np.pi * k / 24
        s = trimesh.creation.cylinder(0.0026, 0.003, sections=12)
        s.apply_transform(rot_x(-90))
        s.apply_translation([(r_in + 0.0015) * np.cos(a), width / 2 + 0.0055, (r_in + 0.0015) * np.sin(a)])
        screws.append(s)
    for k in range(6):
        a = np.radians(60 * k)
        s = trimesh.creation.cylinder(0.0035, 0.004, sections=6)
        s.apply_transform(rot_x(-90))
        s.apply_translation([0.024 * np.cos(a), -width * 0.18 + 0.007, 0.024 * np.sin(a)])
        screws.append(s)
    return tyre, rim, merge(screws)


# ---------------------------------------------------------------- motor and bracket

def motor_meshes(d=0.076, h=0.0365):
    """DM-J6006-2EC outline, axis along y, output face at y = 0, body toward -y."""
    r = d / 2
    profile = [[0, -h], [r - 0.004, -h], [r, -h + 0.004], [r, -0.003], [r - 0.002, 0], [0, 0]]
    housing = trimesh.creation.revolve(np.array(profile)[:, [0, 1]], sections=64)
    housing.apply_transform(rot_x(-90))
    fins = []
    for k in range(24):
        f = trimesh.creation.box([0.0025, h * 0.62, 0.0022])
        f.apply_translation([0, -h * 0.5, r + 0.0006])
        f.apply_transform(rot_y(360 * k / 24))
        fins.append(f)
    housing = merge([housing] + fins)
    cap = trimesh.creation.cylinder(r * 0.78, 0.003, sections=48)
    cap.apply_transform(rot_x(-90))
    cap.apply_translation([0, -h - 0.0015, 0])
    flange = trimesh.creation.cylinder(0.027, 0.005, sections=48)
    flange.apply_transform(rot_x(-90))
    flange.apply_translation([0, 0.0025, 0])
    plug = trimesh.creation.box([0.016, 0.012, 0.009])
    plug.apply_translation([0, -h * 0.55, -r - 0.004])
    return housing, merge([cap]), flange, plug


def motor_bracket(size=0.088, thickness=0.006):
    """Printed bracket plate: rounded square with the motor bore and four screw holes, plus a collar."""
    sq = Polygon([(-size / 2, -size / 2), (size / 2, -size / 2), (size / 2, size / 2), (-size / 2, size / 2)]).buffer(-0.008).buffer(0.008)
    holes = [Point(0, 0).buffer(0.020, 48)] + [Point(sx * 0.031, sz * 0.031).buffer(0.0028, 16) for sx in (-1, 1) for sz in (-1, 1)]
    plate = extrude_xz(sq.difference(unary_union(holes)), thickness)
    collar = extrude_xz(annulus(0.0405, 0.0385), 0.014, -thickness / 2 - 0.007)
    return merge([plate, collar])


# ---------------------------------------------------------------- rocks

def rock(rng, size):
    pts = rng.normal(size=(60, 3))
    pts /= np.linalg.norm(pts, axis=1, keepdims=True)
    pts *= (1 + 0.28 * rng.uniform(-1, 1, (60, 1)))
    pts *= np.array([1.0, rng.uniform(0.6, 1.0), rng.uniform(0.45, 0.8)]) * size
    hull = trimesh.convex.convex_hull(pts)
    hull.apply_translation([0, 0, -hull.bounds[0, 2] - size * 0.25])   # sunk a quarter into the soil
    return hull
