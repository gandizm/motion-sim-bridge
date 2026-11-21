/* Body geometry is in millimetres: X forward, Y left. Field heading is CCW. */
const VehiclePose = (() => {
  function rotate([x, y], c, s) {
    return [x * c - y * s, x * s + y * c];
  }
  function hingePoint(point, part, plan, left, right) {
    if (part !== "jaw_left" && part !== "jaw_right") return point;
    const [x, y] = plan.jaw_pivots[part];
    const angle = ((part === "jaw_left" ? left : -right) * Math.PI) / 180;
    const [dx, dy] = rotate(
      [point[0] - x, point[1] - y],
      Math.cos(angle),
      Math.sin(angle),
    );
    return [x + dx, y + dy];
  }
  function bbox(points) {
    let l = Infinity,
      b = Infinity,
      r = -Infinity,
      t = -Infinity;
    for (const [x, y] of points) {
      l = Math.min(l, x);
      b = Math.min(b, y);
      r = Math.max(r, x);
      t = Math.max(t, y);
    }
    return [l, b, r, t];
  }
  function axes(points) {
    const out = [];
    for (let i = 0; i < points.length; i++) {
      const a = points[i],
        b = points[(i + 1) % points.length],
        dx = b[0] - a[0],
        dy = b[1] - a[1],
        len = Math.hypot(dx, dy);
      if (len > 1e-8) out.push([-dy / len, dx / len]);
    }
    return out;
  }
  function project(points, [x, y]) {
    let l = Infinity,
      h = -Infinity;
    for (const p of points) {
      const v = p[0] * x + p[1] * y;
      l = Math.min(l, v);
      h = Math.max(h, v);
    }
    return [l, h];
  }
  function shape(p) {
    return { p, box: bbox(p), axes: axes(p) };
  }
  function swept(a, b, start, end, tol = 0.05) {
    // Time interval shared by every separating axis; includes stationary overlap and touching.
    const dx = end[0] - start[0],
      dy = end[1] - start[1],
      A = a.box,
      B = b.box;
    if (
      A[2] + Math.max(start[0], end[0]) < B[0] - tol ||
      A[0] + Math.min(start[0], end[0]) > B[2] + tol ||
      A[3] + Math.max(start[1], end[1]) < B[1] - tol ||
      A[1] + Math.min(start[1], end[1]) > B[3] + tol
    )
      return null;
    let enter = 0,
      exit = 1;
    for (const axis of [...a.axes, ...b.axes]) {
      const [al, ah] = project(a.p, axis),
        [bl, bh] = project(b.p, axis);
      const offset = start[0] * axis[0] + start[1] * axis[1],
        v = dx * axis[0] + dy * axis[1];
      const lo = bl - ah - offset - tol,
        hi = bh - al - offset + tol;
      if (Math.abs(v) < 1e-10) {
        if (lo > 0 || hi < 0) return null;
        continue;
      }
      enter = Math.max(enter, Math.min(lo / v, hi / v));
      exit = Math.min(exit, Math.max(lo / v, hi / v));
      if (enter > exit) return null;
    }
    return [enter, exit];
  }
  function engine(data) {
    const obstacles = data.obstacles.map((o) => ({
      ...o,
      faces: o.faces.map(shape),
    }));
    let lastAngle = "",
      rotated = {};
    return (start, end, deg, left = 0, right = 0) => {
      const key = [deg, left, right].join("/");
      if (key !== lastAngle) {
        const a = (deg * Math.PI) / 180;
        for (const [k, parts] of Object.entries(data.bands))
          rotated[k] = parts.map(([id, p]) => [
            id,
            shape(
              p.map((v) =>
                rotate(
                  hingePoint(v, id, data.model, left, right),
                  Math.cos(a),
                  Math.sin(a),
                ),
              ),
            ),
          ]);
        lastAngle = key;
      }
      const result = { at: {}, along: {} };
      for (const o of obstacles)
        for (const [id, body] of rotated[o.band])
          for (const face of o.faces) {
            const interval = swept(body, face, start, end);
            if (!interval) continue;
            const [t, u] = interval;
            const cat = o.soft ? "soft" : o.layer === "rim" ? "review" : "hard";
            const hit = {
              part: id,
              obstacle: o.id,
              label: o.label,
              t,
              point: [
                start[0] + t * (end[0] - start[0]),
                start[1] + t * (end[1] - start[1]),
              ],
            };
            if (!result.along[cat] || result.along[cat].t > t)
              result.along[cat] = hit;
            if (u >= 1 - 1e-9 && !result.at[cat])
              result.at[cat] = { ...hit, t: 1, point: [...end] };
          }
      return result;
    };
  }
  function jawSweep(data, pose, from, to) {
    const [x, y, yaw] = pose,
      position = [x, y],
      rad = Math.PI / 180;
    const stops = data.obstacles.filter((o) => !o.soft && o.layer !== "rim");
    const c = Math.cos(yaw * rad),
      s = Math.sin(yaw * rad);
    const pairs = [];
    for (const o of stops)
      for (const face of o.faces) {
        const target = shape(face);
        for (const [id, ring] of data.bands[o.band]) {
          const side = id === "jaw_left" ? 0 : id === "jaw_right" ? 1 : -1;
          if (side < 0) continue;
          const pivot = data.model.jaw_pivots[id];
          const radius = Math.max(
            ...ring.map((p) => Math.hypot(p[0] - pivot[0], p[1] - pivot[1])),
          );
          const [u, v] = rotate(pivot, c, s),
            box = target.box;
          if (
            x + u + radius + 0.06 < box[0] ||
            x + u - radius - 0.06 > box[2] ||
            y + v + radius + 0.06 < box[1] ||
            y + v - radius - 0.06 > box[3]
          )
            continue;
          pairs.push({ target, id, ring, radius, o, side });
        }
      }
    function search(lo, hi, candidates, depth) {
      const mid = (lo + hi) / 2,
        angles = from.map((v, k) => v + (to[k] - v) * mid);
      const possible = candidates.filter((z) => {
        const pad =
          0.05 +
          (z.radius * rad * Math.abs(to[z.side] - from[z.side]) * (hi - lo)) /
            2;
        const moving = shape(
          z.ring.map((p) =>
            rotate(hingePoint(p, z.id, data.model, ...angles), c, s),
          ),
        );
        return swept(moving, z.target, position, position, pad) !== null;
      });
      if (!possible.length) return null;
      if (
        depth >= 18 ||
        Math.max(...from.map((v, k) => Math.abs(v - to[k]))) * (hi - lo) < 0.02
      ) {
        const z = possible[0];
        return { t: lo, part: z.id, label: z.o.label, obstacle: z.o.id };
      }
      return (
        search(lo, mid, possible, depth + 1) ||
        search(mid, hi, possible, depth + 1)
      );
    }
    return search(0, 1, pairs, 0);
  }
  function draw(
    ctx,
    plan,
    pose,
    toPixel,
    { layers = null, alpha = 1, outline = false, hit = null } = {},
  ) {
    const { x_mm: x, y_mm: y, heading_mdeg: h } = pose;
    if (![x, y, h].every(Number.isFinite))
      throw Error("需要 x_mm、y_mm、heading_mdeg");
    const angle = (h * Math.PI) / 180000,
      c = Math.cos(angle),
      s = Math.sin(angle);
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.lineJoin = "round";
    for (const part of plan.parts) {
      if (layers && !layers.has(part.layer)) continue;
      const [, fill, edge] = plan.layers[part.layer];
      ctx.beginPath();
      for (const poly of part.polygons)
        for (const ring of poly) {
          ring.forEach((p, i) => {
            const [dx, dy] = rotate(
                hingePoint(
                  p,
                  part.id,
                  plan,
                  pose.left_open_deg || 0,
                  pose.right_open_deg || 0,
                ),
                c,
                s,
              ),
              [px, py] = toPixel(x + dx, y + dy);
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
          });
          ctx.closePath();
        }
      ctx.fillStyle = part.id === hit ? "#f06a56" : fill;
      if (!outline) ctx.fill("evenodd");
      ctx.strokeStyle = part.id === hit ? "#a52820" : edge;
      ctx.lineWidth = part.id === hit ? 2 : 1;
      ctx.stroke();
    }
    ctx.restore();
  }
  return { draw, engine, swept, shape, hingePoint, jawSweep };
})();
if (typeof module !== "undefined" && module.exports)
  module.exports = VehiclePose;
