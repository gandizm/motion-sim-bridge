"""Build a self-contained, click-to-check waypoint page. No telemetry input."""

import argparse
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET
from itertools import product

import mujoco as mj
import numpy as np

from field_map import load_map

BASE = Path(__file__).resolve().parent
DEMO = {
    "schema": "field-map-v1",
    "frame": "field",
    "units": "m",
    "bounds_m": [-0.55, -0.52, 0.65, 0.57],
    "features": [
        {
            "id": "fixture-A",
            "layer": "barrier",
            "label": "测试挡条 A",
            "color": "#d4ad67",
            "z_m": [0, 0.018],
            "xy_m": [[0.10, 0.15], [0.26, 0.15], [0.26, 0.19], [0.10, 0.19]],
        },
        {
            "id": "fixture-B",
            "layer": "barrier",
            "label": "测试挡条 B",
            "color": "#d4ad67",
            "z_m": [0, 0.038],
            "xy_m": [[-0.32, -0.28], [-0.28, -0.28], [-0.28, -0.06], [-0.32, -0.06]],
        },
    ],
}
MARKINGS = {"start", "zone", "marking"}
SOFT = {"object", "ridge"}


def hull(points):
    """Monotone convex hull, retaining winding and rejecting degenerate polygons."""
    points = sorted({tuple(x) for x in np.round(points, 7)})

    def chain(seq):
        out = []
        for x, y in seq:
            while len(out) >= 2:
                a, b = out[-2:]
                if (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]) > 1e-9:
                    break
                out.pop()
            out.append((x, y))
        return out

    if len(points) < 3:
        return []
    ring = chain(points)[:-1] + chain(reversed(points))[:-1]
    return ring if len(ring) >= 3 else []


def clipped(vertices, edges, low, high):
    low -= 0.05
    high += 0.05  # Retain tangencies at a band boundary.
    z = vertices[:, 2]
    if z.min() > high + 1e-5 or z.max() < low - 1e-5:
        return []
    points = [vertices[(z >= low - 1e-5) & (z <= high + 1e-5), :2]]
    a, b = vertices[edges[:, 0]], vertices[edges[:, 1]]
    for level in (low, high):
        cross = (a[:, 2] - level) * (b[:, 2] - level) < 0
        aa, bb = a[cross], b[cross]
        mix = (level - aa[:, 2]) / (bb[:, 2] - aa[:, 2])
        points.append(aa[:, :2] + (bb - aa)[:, :2] * mix[:, None])
    return hull(np.concatenate(points))


def simple_polygon(points):
    """Triangulate a simple concave obstacle without filling its notches."""
    p = np.asarray(points, float)
    if len(p) > 1 and np.allclose(p[0], p[-1]):
        p = p[:-1]
    outer = hull(p)

    def area(q):
        q = np.asarray(q)
        return 0.5 * np.sum(q[:, 0] * np.roll(q[:, 1], -1) - q[:, 1] * np.roll(q[:, 0], -1))

    if len(outer) < 3 or abs(area(p)) < 1e-8:
        raise ValueError("Degenerate map polygon")
    if abs(abs(area(p)) - abs(area(outer))) < 1e-5:
        return [outer]
    if area(p) < 0:
        p = p[::-1]

    def turn(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    ring = list(range(len(p)))
    pieces = []
    while len(ring) > 3:
        for j, i in enumerate(ring):
            ia, ic = ring[j - 1], ring[(j + 1) % len(ring)]
            a, b, c = p[[ia, i, ic]]
            if abs(turn(a, b, c)) < 1e-10:
                ring.pop(j)
                break
            if turn(a, b, c) < 0:
                continue
            others = [p[k] for k in ring if k not in (ia, i, ic)]
            if any(min(turn(a, b, o), turn(b, c, o), turn(c, a, o)) >= -1e-9 for o in others):
                continue
            pieces.append([a.tolist(), b.tolist(), c.tolist()])
            ring.pop(j)
            break
        else:
            raise ValueError("Map polygon must be simple")
    pieces.append(p[ring].tolist())
    return pieces


def bands_for(model_path, bands):
    m = mj.MjModel.from_xml_path(str(model_path))
    d = mj.MjData(m)
    mj.mj_forward(m, d)
    result = {k: [] for k in bands}
    for i in range(m.ngeom):
        if not int(m.geom_contype[i]) & 2:
            continue
        mesh = m.geom_dataid[i]
        if m.geom_type[i] != mj.mjtGeom.mjGEOM_MESH:
            raise ValueError("Expected a convex vehicle mesh")
        s = m.mesh_vertadr[mesh]
        v = m.mesh_vert[s : s + m.mesh_vertnum[mesh]]
        v = (v @ d.geom_xmat[i].reshape(3, 3).T + d.geom_xpos[i]) * 1000
        sf = m.mesh_faceadr[mesh]
        f = m.mesh_face[sf : sf + m.mesh_facenum[mesh]]
        edges = np.unique(np.sort(f[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1), axis=0)
        name = m.geom(i).name.rsplit("_", 1)[0]
        for k, (lo, hi) in bands.items():
            ring = clipped(v, edges, lo, hi)
            if ring:
                result[k].append([name, np.round(ring, 4).tolist()])
    return result


def build_data(world=None):
    world = load_map(world) if world else DEMO
    bands = {}
    obstacles = []
    for f in world["features"]:
        if f["layer"] in MARKINGS:
            continue
        low, high = [round(z * 1000, 5) for z in f["z_m"]]
        k = f"{low:g}:{high:g}"
        bands[k] = (low, high)
        obstacles.append(
            dict(
                id=f["id"],
                label=f["label"],
                band=k,
                layer=f["layer"],
                soft=f["layer"] in SOFT,
                faces=simple_polygon(np.asarray(f["xy_m"]) * 1000),
            )
        )
    model = json.loads((BASE / "assets/vehicle_plan.json").read_text())
    payload = dict(
        demo=world is DEMO,
        map=world,
        model=model,
        obstacles=obstacles,
        bands=bands_for(BASE / "assets/vehicle.xml", bands),
    )
    return payload


def build(out, world=None):
    payload = build_data(world)
    html = (BASE / "pose_template.html").read_text(encoding="utf-8-sig")
    html = html.replace(
        "/*__ENGINE__*/", (BASE / "pose_geometry.js").read_text(encoding="utf-8-sig")
    )
    html = html.replace(
        "<!--__DATA__-->",
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")).replace("<", "\\u003c"),
    )
    out.write_text(html, encoding="utf8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=Path, help="local field-map-v1 JSON")
    parser.add_argument("--out", type=Path, default=Path("pose_check.html"))
    args = parser.parse_args()
    build(args.out, args.map)
