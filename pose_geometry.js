/* Body geometry is in millimetres: X forward, Y left. Field heading is CCW. */
const VehiclePose = (() => {
  function rotate([x, y], c, s) {
    return [x * c - y * s, x * s + y * c];
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
  function swept(a, b, start, end) {
    // Time interval shared by every separating axis; includes stationary overlap and touching.
    const tol = 0.05,
      dx = end[0] - start[0],
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
    let lastAngle = NaN,
      rotated = {};
    return (start, end, deg) => {
      if (deg !== lastAngle) {
        const a = (deg * Math.PI) / 180;
        for (const [k, parts] of Object.entries(data.bands))
          rotated[k] = parts.map(([id, p]) => [
            id,
            shape(p.map((v) => rotate(v, Math.cos(a), Math.sin(a)))),
          ]);
        lastAngle = deg;
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
            const [dx, dy] = rotate(p, c, s),
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
  return { draw, engine, swept, shape };
})();
if (typeof module !== "undefined" && module.exports)
  module.exports = VehiclePose;
