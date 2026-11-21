"""Build an offline articulated-motion viewer from vehicle geometry and optional local cases."""

import argparse
import json
import math
from pathlib import Path
from pose_check import BASE, build_data


def geometry_cases(plan):
    p = plan["jaw_pivots"]["jaw_left"]
    j = next(x for x in plan["parts"] if x["id"] == "jaw_left")
    corners = [v for poly in j["polygons"] for v in poly[0]]
    v = max(corners, key=lambda v: math.dist(v, p))
    radius = math.dist(v, p)
    peak = 90 - math.degrees(math.atan2(v[1] - p[1], v[0] - p[0]))
    limit = round(peak, 2)

    def make(label, values, text):
        return dict(
            label=label,
            relative=True,
            note=text,
            keys=[[0, 0, 0, l, r] for l, r in values],
            phases=["机构动作"] * (len(values) - 1),
        )

    cases = [
        make("双爪张开", [(0, 0), (limit, limit)], "从装配合爪零位转至横向最宽处。"),
        make(
            "左侧分离",
            [(0, 0), (limit, 0), (0, 0)],
            "释放左侧，右侧保持合拢，再回收。示意不含物料受力及夹紧变形。",
        ),
        make(
            "右侧分离",
            [(0, 0), (0, limit), (0, 0)],
            "释放右侧，左侧保持合拢，再回收。示意不含物料受力及夹紧变形。",
        ),
        make(
            "单侧大角度核对",
            [(0, 0), (220, 0)],
            "展示纯几何转角；水平装配下经过同侧轮组的角度段会干涉。",
        ),
    ]
    cases[-1]["theoretical"] = True
    return dict(radius=radius, peak=peak, width=2 * (p[1] + radius)), cases


def build(out, world=None, private_cases=None):
    data = build_data(world)
    data["mechanics"], generic = geometry_cases(data["model"])
    data["actions"] = (
        json.loads(private_cases.read_text(encoding="utf-8-sig")) if private_cases else []
    ) + generic
    template = (BASE / "action_template.html").read_text(encoding="utf-8-sig")
    out.write_text(
        template.replace(
            "/*__ENGINE__*/", (BASE / "pose_geometry.js").read_text(encoding="utf-8-sig")
        ).replace(
            "<!--__DATA__-->",
            json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("<", "\\u003c"),
        ),
        encoding="utf8",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--map", type=Path)
    p.add_argument("--cases", type=Path)
    p.add_argument("--out", type=Path, default=Path("action_view.html"))
    a = p.parse_args()
    build(a.out, a.map, a.cases)
