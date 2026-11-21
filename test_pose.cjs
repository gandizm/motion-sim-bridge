"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const crypto = require("node:crypto");
const geo = require("./pose_geometry.js");
const query = require("./check_pose.cjs");
const page = path.join(__dirname, "pose_check.html");
const encoded = fs
  .readFileSync(page, "utf8")
  .match(
    /<script type="application\/json" id="payload">([\s\S]*?)<\/script>/,
  )[1];
const data = JSON.parse(encoded);
assert.equal(data.demo, true);
assert.equal(
  data.model.step_sha256,
  crypto
    .createHash("sha256")
    .update(fs.readFileSync(path.join(__dirname, "assets/vehicle.step")))
    .digest("hex"),
);
function probe(x, y, band, deg = 0) {
  const g = {
    ...data,
    obstacles: [
      {
        id: "probe",
        band,
        label: "probe",
        layer: "barrier",
        faces: [
          [
            [x - 0.5, y - 0.5],
            [x + 0.5, y - 0.5],
            [x + 0.5, y + 0.5],
            [x - 0.5, y + 0.5],
          ],
        ],
      },
    ],
  };
  return geo.engine(g)([0, 0], [0, 0], deg).at.hard?.part;
}
assert.equal(probe(-45, 0, "0:18"), undefined);
assert.equal(probe(-45, 0, "0:38"), "rear_motor");
assert.equal(probe(0, -45, "0:38", 90), "rear_motor");
assert.equal(probe(-116, 0, "0:38"), "wheel_rear");
const bar = geo.shape([
    [0, 0],
    [1, 0],
    [1, 1],
    [0, 1],
  ]),
  tip = geo.shape([
    [-0.01, -0.01],
    [0.01, -0.01],
    [0.01, 0.01],
    [-0.01, 0.01],
  ]);
const [enter, exit] = geo.swept(tip, bar, [-3, 0.5], [4, 0.5]);
assert(enter > 0 && exit < 1);
assert.equal(geo.swept(tip, bar, [-3, 2], [4, 2]), null);
const result = query(page, 0, 0, 90);
assert.equal(result.goal, "clear");
assert.equal(result.straight, null);
console.log(
  "Approved STEP, motor height, wheel direction, swept crossing and pose-only query passed.",
);

const pivot = data.model.jaw_pivots.jaw_left;
assert.deepEqual(geo.hingePoint(pivot, "jaw_left", data.model, 220, 0), pivot);
const startTip = [pivot[0] + 10, pivot[1]];
const leftTip = geo.hingePoint(startTip, "jaw_left", data.model, 90, 0);
assert(
  Math.abs(leftTip[0] - pivot[0]) < 1e-8 &&
    Math.abs(leftTip[1] - pivot[1] - 10) < 1e-8,
);
const fixture = {
  ...data,
  model: { jaw_pivots: { jaw_left: [0, 0], jaw_right: [0, 0] } },
  bands: {
    h: [
      [
        "jaw_left",
        [
          [9.99, -0.01],
          [10.01, -0.01],
          [10.01, 0.01],
          [9.99, 0.01],
        ],
      ],
    ],
  },
  obstacles: [
    {
      label: "fixture",
      id: "pin",
      layer: "barrier",
      band: "h",
      faces: [
        [
          [-0.02, 9.98],
          [0.02, 9.98],
          [0.02, 10.02],
          [-0.02, 10.02],
        ],
      ],
    },
  ],
};
const crossing = geo.jawSweep(fixture, [0, 0, 0], [0, 0], [180, 0]);
assert(crossing && crossing.t > 0.48 && crossing.t < 0.5);
assert.equal(geo.jawSweep(fixture, [0, 0, 0], [0, 0], [45, 0]), null);
