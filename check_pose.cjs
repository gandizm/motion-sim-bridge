/* Query the same self-contained page and geometry as the interactive viewer. */
"use strict";
const fs = require("node:fs");
const path = require("node:path");
const { pathToFileURL } = require("node:url");
const geom = require("./pose_geometry.js");
function query(file, x, y, angle, sx = x, sy = y) {
  const html = fs.readFileSync(file, "utf8");
  const raw = html.match(
    /<script type="application\/json" id="payload">([\s\S]*?)<\/script>/,
  );
  if (!raw) throw Error("Expected a generated pose-check page");
  const data = JSON.parse(raw[1]),
    start = [sx, sy],
    goal = [x, y],
    b = data.map.bounds_m.map((v) => 1000 * v);
  const inside = (p) =>
    p[0] >= b[0] && p[0] <= b[2] && p[1] >= b[1] && p[1] <= b[3];
  const r = geom.engine(data)(start, goal, angle);
  function status(res, ok) {
    return !ok
      ? "out_of_bounds"
      : res.hard
        ? "blocked"
        : res.review || res.soft
          ? "review"
          : "clear";
  }
  const link = pathToFileURL(path.resolve(file));
  link.search = new URLSearchParams({
    x,
    y,
    yaw: angle,
    ...(arguments.length > 4 ? { sx, sy } : {}),
  }).toString();
  return {
    goal: status(r.at, inside(goal)),
    straight:
      arguments.length > 4
        ? status(r.along, inside(start) && inside(goal))
        : null,
    endpoint_contacts: r.at,
    segment_contacts: arguments.length > 4 ? r.along : null,
    viewer: link.href,
  };
}
module.exports = query;
if (require.main === module) {
  const [file, ...args] = process.argv.slice(2);
  if (
    ![3, 5].includes(args.length) ||
    args.some((v) => v.trim() === "" || !Number.isFinite(Number(v)))
  ) {
    console.error(
      "Usage: node check_pose.cjs page.html x_mm y_mm yaw_deg [start_x_mm start_y_mm]",
    );
    process.exit(2);
  }
  console.log(JSON.stringify(query(file, ...args.map(Number)), null, 2));
}
