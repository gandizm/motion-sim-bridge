"""Build a local two-view map and a millimetre polygon table from a shared manifest."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys


def finite(v):
    return type(v) in (int, float) and math.isfinite(v)


def load_map(path):
    doc = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if (doc.get("schema"), doc.get("frame"), doc.get("units")) != ("field-map-v1", "field", "m"):
        raise ValueError("Expected field-map-v1 with frame=field and units=m")
    bounds = doc["bounds_m"]
    if (
        len(bounds) != 4
        or not all(map(finite, bounds))
        or bounds[2] <= bounds[0]
        or bounds[3] <= bounds[1]
    ):
        raise ValueError("Invalid bounds")
    ids = set()
    for f in doc["features"]:
        if not f["id"] or f["id"] in ids:
            raise ValueError("Duplicate or empty feature ID")
        ids.add(f["id"])
        if not re.fullmatch(r"[a-z][a-z0-9_]*", f["layer"]) or not re.fullmatch(
            r"#[0-9a-fA-F]{6}", f["color"]
        ):
            raise ValueError("Invalid layer or color")
        if len(f["xy_m"]) < 3 or any(len(p) != 2 or not all(map(finite, p)) for p in f["xy_m"]):
            raise ValueError("Invalid polygon")
        if len(f["z_m"]) != 2 or not all(map(finite, f["z_m"])) or f["z_m"][1] < f["z_m"][0]:
            raise ValueError("Invalid heights")
    if not ids:
        raise ValueError("Empty map")
    return doc


def write_header(doc, path, digest):
    layers = sorted({f["layer"] for f in doc["features"]})
    vertices = []
    shapes = []
    for f in doc["features"]:
        xy = [tuple(round(n * 1000) for n in p) for p in f["xy_m"]]
        z = [round(v * 1000000) for v in f["z_m"]]
        shapes.append(
            "{" + f'{len(vertices)}, {len(xy)}, FM_{f["layer"].upper()}, {z[0]}, {z[1]}' + "}"
        )
        vertices.extend(xy)
    content = [
        "/* World frame: +X right, +Y up; z heights in micrometres. */",
        "#ifndef MOTION_FIELD_MAP_H",
        "#define MOTION_FIELD_MAP_H",
        "#include <stdint.h>",
        f'#define FM_SOURCE_SHA256 "{digest}"',
        "enum fm_layer { " + ", ".join("FM_" + l.upper() for l in layers) + " };",
        "typedef struct { int32_t x_mm,y_mm; } fm_point;",
        "typedef struct { uint32_t offset,count,layer; int32_t z_min_um,z_max_um; } fm_feature;",
        "static const fm_point fm_vertices[] = {",
        ",\n".join("{%d,%d}" % p for p in vertices),
        "};",
        "static const fm_feature fm_features[] = {",
        ",\n".join(shapes),
        "};",
        f"#define FM_FEATURE_COUNT {len(shapes)}u",
        "static const int32_t fm_bounds_mm[4] = {"
        + ",".join(str(round(x * 1000)) for x in doc["bounds_m"])
        + "};",
    ]
    starts = doc.get("starts", [])
    if starts:
        content += [
            "typedef struct { int32_t id,upper_x_mm,upper_y_mm,lower_x_mm,lower_y_mm,yaw_mdeg; } fm_start;",
            "static const fm_start fm_starts[] = {",
        ]
        for s in starts:
            nums = [
                s["id"],
                *(round(x * 1000) for x in s["upper_m"] + s["lower_m"]),
                round(s["yaw_deg"] * 1000),
            ]
            content.append("{" + ",".join(map(str, nums)) + "},")
        content += ["};", f"#define FM_START_COUNT {len(starts)}u"]
    content.append(
        "/* Inclusive XY point query. A route still needs the moving body's footprint. */\nstatic inline int fm_contains_xy(const fm_feature *f, int32_t x, int32_t y) {\n    int inside = 0;\n    for (uint32_t i = 0, j = f->count - 1; i < f->count; j = i++) {\n        const fm_point a = fm_vertices[f->offset + j], b = fm_vertices[f->offset + i];\n        const int64_t dx = (int64_t)b.x_mm - a.x_mm, dy = (int64_t)b.y_mm - a.y_mm;\n        const int64_t cross = dx * ((int64_t)y-a.y_mm) - dy * ((int64_t)x-a.x_mm);\n        if (cross == 0 && x >= (a.x_mm<b.x_mm?a.x_mm:b.x_mm) && x <= (a.x_mm>b.x_mm?a.x_mm:b.x_mm)\n                       && y >= (a.y_mm<b.y_mm?a.y_mm:b.y_mm) && y <= (a.y_mm>b.y_mm?a.y_mm:b.y_mm)) return 1;\n        if ((a.y_mm > y) != (b.y_mm > y) && ((dy>0 && cross>0) || (dy<0 && cross<0))) inside = !inside;\n    }\n    return inside;\n}"
    )
    path.write_text("\n".join(content + ["#endif", ""]), encoding="utf8")


def build(source, out):
    doc = load_map(source)
    out.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    encoded = json.dumps(doc, ensure_ascii=False, separators=(",", ":")).replace("<", "\\u003c")
    template = Path(__file__).with_name("map_view.html").read_text(encoding="utf-8-sig")
    pattern = r'(<script\b[^>]*\bid="embedded"[^>]*>)\s*null\s*(</script>)'
    html, count = re.subn(pattern, lambda m: m[1] + encoded + m[2], template, flags=re.I)
    if count != 1:
        raise ValueError("Expected one embedded map slot")
    (out / "viewer.html").write_text(html, encoding="utf8")
    write_header(doc, out / "field_map.h", digest)


def draw_upper(app, canvas, text, doc):
    import cv2
    import numpy as np

    left, top, width = app.map_left, app.map_top, app.map_size
    cv2.rectangle(canvas, (left, top), (left + width, top + width), (244, 242, 240), -1)
    for f in doc["features"]:
        xy = np.array([app.world_to_pixel(x, y) for x, y in f["xy_m"]], dtype=np.int32)
        color = tuple(bytes.fromhex(f["color"][1:])[::-1])
        cv2.fillPoly(canvas, [xy], color)
        if (f.get("start_id") is not None and f["start_id"] == app.zone) or (
            f.get("team") is not None and f["team"] == app.side
        ):
            cv2.polylines(canvas, [xy], True, (0, 220, 250), 3)
    for item in doc.get("labels", []):
        px = app.world_to_pixel(item["x_m"], item["y_m"])
        text.add(item["text"], px, 15, (246, 246, 246), True, "mm")
    cv2.rectangle(canvas, (left, top), (left + width, top + width), (70, 65, 60), 2)


def run_upper(app_path, map_path, options):
    from importlib.util import spec_from_file_location, module_from_spec

    doc = load_map(map_path)
    sys.path.insert(0, str(app_path.resolve().parent))
    spec = spec_from_file_location("motion_upper_app", app_path)
    module = module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    from field_model import FIELD_HALF_M

    if not all(
        abs(a - b) < 1e-7
        for a, b in zip(doc["bounds_m"], [-FIELD_HALF_M, -FIELD_HALF_M, FIELD_HALF_M, FIELD_HALF_M])
    ):
        raise ValueError("Map extent disagrees with upper map projection")
    module.RescueMapApp.draw_field = lambda self, canvas, text: draw_upper(self, canvas, text, doc)
    sys.argv = [str(app_path), *options]
    return module.main()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("map", type=Path)
    b.add_argument("out", type=Path)
    u = sub.add_parser("upper")
    u.add_argument("app", type=Path)
    u.add_argument("map", type=Path)
    u.add_argument("options", nargs=argparse.REMAINDER)
    args = p.parse_args()
    if args.cmd == "build":
        build(args.map, args.out)
    else:
        raise SystemExit(
            run_upper(
                args.app, args.map, args.options[1:] if args.options[:1] == ["--"] else args.options
            )
        )
