"""Replay planar commands with swept collision checks and rollback of rejected steps."""

import argparse
import json
import math
from pathlib import Path

import mujoco as mj
import numpy as np
from contact_guard import Guard

REQUIRED = ("t_s", "x_m", "y_m", "yaw_deg", "left_deg", "right_deg")
ATTITUDE = ("z_m", "roll_deg", "pitch_deg", "front_deg")
FIELDS = (*REQUIRED, *ATTITUDE)


def read_trace(path):
    samples = []
    attitude_fields = None
    for line_no, line in enumerate(Path(path).read_text(encoding="utf-8-sig").splitlines(), 1):
        if not line.strip():
            continue
        try:
            obj = json.loads(line)
            present = tuple(k for k in ATTITUDE if k in obj)
            if attitude_fields is not None and present != attitude_fields:
                raise ValueError("optional geometric fields must be consistent across samples")
            attitude_fields = present
            row = [obj[k] for k in REQUIRED] + [obj.get(k, 0.0) for k in ATTITUDE]
            if not all(type(x) in (int, float) and math.isfinite(x) for x in row):
                raise ValueError("expected finite numbers")
            if samples and row[0] <= samples[-1][0]:
                raise ValueError("time must increase")
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid trajectory line {line_no}: {exc}") from exc
        samples.append(row)
    if len(samples) < 2:
        raise ValueError("Need at least two trajectory samples")
    return np.array(samples, float)


def pose_dict(p):
    return dict(zip(FIELDS, p.tolist(), strict=True))


class Replay:
    def __init__(self, scene, path, dynamics, keyframe=None, record=False):
        m = self.m = mj.MjModel.from_xml_path(str(scene))
        self.d = mj.MjData(m)
        self.trial = mj.MjData(m)
        self.dynamics = dynamics
        self.path = path
        self.capture = [] if record else None
        self.base = m.joint("base")
        self.jaws = [m.joint(n + "_angle") for n in ("jaw_left", "jaw_right")]
        idx = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "front_angle")
        self.front = m.joint(idx) if idx >= 0 else None
        if self.front is None and np.any(path[:, 9]):
            raise ValueError("Nonzero front_deg requires a front_angle hinge in the scene")
        if dynamics:
            # Prescribed coordinates act as rigid drivers during the contact solve.
            v = int(self.base.dofadr[0])
            m.dof_armature[v : v + 6] += 1e3
            for j in self.jaws + ([self.front] if self.front is not None else []):
                m.dof_armature[j.dofadr[0]] += 1e2
            mj.mj_setConst(m, self.d)
        if keyframe is not None:
            index = mj.mj_name2id(m, mj.mjtObj.mjOBJ_KEY, keyframe)
            if index < 0:
                raise ValueError(f"Unknown scene keyframe: {keyframe}")
            mj.mj_resetDataKeyframe(m, self.d, index)
        self.place(self.d, path[0], np.zeros(len(FIELDS)))
        mj.mj_forward(m, self.d)
        self.guard = Guard(m, self.d, dynamics)
        self.targets = sorted(set(self.guard.weld[self.guard.free]))
        for i in self.targets:
            if (
                m.body_parentid[i] != 0
                or m.body_jntnum[i] != 1
                or m.jnt_type[m.body_jntadr[i]] != mj.mjtJoint.mjJNT_FREE
            ):
                raise ValueError("Moving scene objects must have a top-level free joint")
        self.inverse_mass = {i: 1.0 / m.body_subtreemass[i] for i in self.targets}
        self.starts = {i: self.d.xpos[i].copy() for i in self.targets}
        self.free_radius = {
            i: max(self.guard.local_radius[self.guard.weld == i]) for i in self.targets
        }
        self.accepted = None
        self.block = None
        self.samples = 0
        self.trials = 0
        self.contacts = {}
        self.active = set()
        self.episodes = []
        self.max_depth = 0.0

    def place(self, data, p, vel):
        q = int(self.base.qposadr[0])
        v = int(self.base.dofadr[0])
        yaw, roll, pitch = np.radians(p[[3, 7, 8]])
        cy, cr, cp = np.cos([yaw / 2, roll / 2, pitch / 2])
        sy, sr, sp = np.sin([yaw / 2, roll / 2, pitch / 2])
        data.qpos[q : q + 7] = [
            p[1],
            p[2],
            p[6],
            cy * cr * cp + sy * sr * sp,
            cy * sr * cp - sy * cr * sp,
            cy * cr * sp + sy * sr * cp,
            sy * cr * cp - cy * sr * sp,
        ]
        yr, rr, pr = np.radians(vel[[3, 7, 8]])
        data.qvel[v : v + 6] = [
            vel[1],
            vel[2],
            vel[6],
            rr - yr * math.sin(pitch),
            pr * math.cos(roll) + yr * math.sin(roll) * math.cos(pitch),
            -pr * math.sin(roll) + yr * math.cos(roll) * math.cos(pitch),
        ]
        if self.front is not None:
            data.qpos[self.front.qposadr[0]] = math.radians(p[9])
            data.qvel[self.front.dofadr[0]] = math.radians(vel[9])
        for i, j in enumerate(self.jaws, 4):
            data.qpos[j.qposadr[0]] = math.radians(p[i])
            data.qvel[j.dofadr[0]] = math.radians(vel[i])

    def record(self, p):
        if self.capture is not None and (
            not self.capture or p[0] - self.capture[-1][0] >= 1.0 / 60
        ):
            self.capture.append((float(p[0]), self.d.qpos.copy()))
        now = set()
        for c in self.d.contact:
            a, b = map(int, c.geom)
            if c.dist > 0 or not (self.guard.robot[a] or self.guard.robot[b]):
                continue
            if self.guard.planes[a] or self.guard.planes[b]:
                continue
            key = self.guard.describe(a, b)
            now.add(key)
            if key not in self.active and key not in self.contacts:
                self.contacts[key] = dict(
                    part=key[0],
                    object=key[1],
                    t_first_s=float(p[0]),
                    first_pose=pose_dict(p),
                    max_penetration_mm=0.0,
                )
            if key not in self.active:
                self.episodes.append(
                    dict(part=key[0], object=key[1], start_s=float(p[0]), end_s=float(p[0]))
                )
                self.active.add(key)
            item = self.contacts[key]
            item["t_last_s"] = float(p[0])
            item["max_penetration_mm"] = max(item["max_penetration_mm"], -float(c.dist) * 1000)
        for ep in reversed(self.episodes):
            key = (ep["part"], ep["object"])
            if key in now:
                ep["end_s"] = float(p[0])
                now.remove(key)
            if not now:
                break
        self.active = {key for key in self.active if self.contacts[key]["t_last_s"] == p[0]}

    def blocked(self, reason, p, bad=None):
        item = dict(
            reason=reason,
            attempted_pose=pose_dict(p),
            last_accepted_time_s=None if self.accepted is None else float(self.accepted[0]),
        )
        if bad is not None:
            a, b, dist, limit = bad
            part, obj = self.guard.describe(a, b)
            item.update(
                part=part, object=obj, clearance_at_check_mm=dist * 1000, tolerance_mm=1000 * limit
            )
            key = part, obj
            if key not in self.contacts:
                self.contacts[key] = dict(
                    part=part,
                    object=obj,
                    t_first_s=float(p[0]),
                    t_last_s=float(p[0]),
                    first_pose=pose_dict(p),
                    max_penetration_mm=0.0,
                    source="sweep_guard",
                )
        self.block = item
        return False

    def project_free_contacts(self, trial):
        # Positional correction acts only on movable objects. A trapped object must
        # still satisfy all wall and driver constraints before the trial is accepted.
        m = self.m
        for _ in range(10):
            shifts = {}
            checked = set()
            for c in trial.contact:
                a, b = sorted(map(int, c.geom))
                wa, wb = int(self.guard.weld[a]), int(self.guard.weld[b])
                ia = self.inverse_mass.get(wa, 0.0)
                ib = self.inverse_mass.get(wb, 0.0)
                if ia + ib == 0:
                    continue
                distance = c.dist
                normal = c.frame[:3] if a == c.geom[0] else -c.frame[:3]
                # The manifold and convex closest-point query can disagree near
                # curved edges. Resolve the deeper measure in the same pair order
                # used for sweep validation, once per pair per projection pass.
                if (a, b) not in checked:
                    checked.add((a, b))
                    points = np.empty(6)
                    exact = mj.mj_geomDistance(m, trial, a, b, 0.001, points)
                    axis = points[3:] - points[:3]
                    if exact < min(distance, -0.000035) and np.linalg.norm(axis) > 1e-9:
                        distance = exact
                        normal = -axis / np.linalg.norm(axis)
                depth = -distance - 0.000035
                if depth <= 0.000005:
                    continue
                given = shifts.get(wb, 0.0) - shifts.get(wa, 0.0)
                remaining = depth - float(np.sum(normal * given))
                if remaining <= 0:
                    continue
                correction = normal * (remaining / (ia + ib))
                if ia:
                    shifts[wa] = shifts.get(wa, 0.0) - ia * correction
                if ib:
                    shifts[wb] = shifts.get(wb, 0.0) + ib * correction
            if not shifts or max(np.linalg.norm(v) for v in shifts.values()) < 0.000002:
                break
            for body, move in shifts.items():
                pos = m.jnt_qposadr[m.body_jntadr[body]]
                trial.qpos[pos : pos + 3] += move
            mj.mj_forward(m, trial)

    def advance(self, a, b, depth=0):
        m = self.m
        d = self.d
        trial = self.trial
        delta = b - a
        dt = delta[0]
        vel = delta / dt
        self.trials += 1
        mj.mj_copyData(trial, m, d)
        free_motion = {}
        if self.dynamics:
            m.opt.timestep = dt
            self.place(trial, a, vel)
            mj.mj_step(m, trial)
        self.place(trial, b, vel)
        mj.mj_forward(m, trial)
        if self.dynamics:
            self.project_free_contacts(trial)
        if (
            not np.isfinite(trial.qpos).all()
            or not np.isfinite(trial.qvel).all()
            or np.any(trial.warning.number > d.warning.number)
        ):
            return self.blocked("solver_warning", b)
        if self.dynamics:
            for i in self.targets:
                linear = np.linalg.norm(trial.xpos[i] - d.xpos[i])
                q0, q1 = d.xquat[i], trial.xquat[i]
                # Use the larger of endpoint rotation and integrated angular speed, including full turns.
                angle = (
                    2
                    * math.atan2(
                        min(np.linalg.norm(q1 - q0), np.linalg.norm(q1 + q0)),
                        max(np.linalg.norm(q1 - q0), np.linalg.norm(q1 + q0)),
                    )
                    * 2
                )
                dof = m.jnt_dofadr[m.body_jntadr[i]]
                angle = max(angle, dt * np.linalg.norm(trial.qvel[dof + 3 : dof + 6]))
                free_motion[i] = linear + angle * self.free_radius[i]
        bad = self.guard.inspect(self.guard.geometry(), delta, free_motion)
        endpoint_bad, depth_m = self.guard.endpoint(trial)
        if bad is None:
            bad = endpoint_bad
        if bad is not None:
            bound = max(self.guard.motion(delta, free_motion).values())
            if depth >= 22 or bound < 0.000002 or dt < 1e-8:
                return self.blocked("sweep_limit", b, bad)
            mid = (a + b) * 0.5
            return self.advance(a, mid, depth + 1) and self.advance(mid, b, depth + 1)
        self.max_depth = max(self.max_depth, depth_m)
        mj.mj_copyData(d, m, trial)
        self.accepted = b.copy()
        self.samples += 1
        self.record(b)
        return True

    def execute(self):
        first = self.path[0]
        invalid = self.guard.inspect(self.guard.geometry(), np.zeros(len(FIELDS)), {}, initial=True)
        endpoint_bad, initial_depth = self.guard.endpoint(self.d)
        if invalid is None:
            invalid = endpoint_bad
        if invalid is not None:
            self.blocked("initial_overlap", first, invalid)
        else:
            self.max_depth = initial_depth
            self.accepted = first.copy()
            self.samples = 1
            self.record(first)
            for a, b in zip(self.path, self.path[1:]):
                delta = b - a
                motion = max(self.guard.motion(delta, {}).values())
                steps = max(
                    1,
                    math.ceil(motion / 0.004),
                    math.ceil(delta[0] / 0.001) if self.dynamics else 1,
                )
                prev = a
                for k in range(1, steps + 1):
                    end = a + delta * (k / steps)
                    if not self.advance(prev, end):
                        break
                    prev = end
                if self.block:
                    break
        return dict(
            status="blocked" if self.block else "completed",
            blocked=self.block,
            final_pose=pose_dict(self.accepted) if self.accepted is not None else None,
            max_accepted_penetration_mm=self.max_depth * 1000,
            requested_end=pose_dict(self.path[-1]),
            samples=self.samples,
            trial_steps=self.trials,
            contacts=list(self.contacts.values()),
            episodes=self.episodes,
            mode="prescribed_pose_with_free_objects" if self.dynamics else "pose_geometry",
            target_delta_m={
                (self.m.body(i).name or f"body_{i}"): (self.d.xpos[i] - p).round(7).tolist()
                for i, p in self.starts.items()
            },
        )


def run(scene, trace, dynamics=False, keyframe=None, record=None):
    import hashlib

    replay = Replay(scene, read_trace(trace), dynamics, keyframe, record is not None)
    report = replay.execute()
    report["inputs"] = {
        name: hashlib.sha256(Path(p).read_bytes()).hexdigest()
        for name, p in [("scene_sha256", scene), ("trace_sha256", trace)]
    }
    report["mujoco_version"] = mj.__version__
    report["keyframe"] = keyframe
    if record is not None:
        samples = replay.capture
        if replay.accepted is not None and samples[-1][0] != replay.accepted[0]:
            samples.append((float(replay.accepted[0]), replay.d.qpos.copy()))
        with Path(record).open("wb") as out:
            np.savez_compressed(
                out,
                times=np.array([t for t, q in samples]),
                qpos=np.array([q for t, q in samples]).reshape(-1, replay.m.nq),
                scene_sha256=report["inputs"]["scene_sha256"],
            )
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("scene", type=Path)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--dynamics", action="store_true")
    parser.add_argument("--fail-on-block", action="store_true")
    parser.add_argument("--keyframe", help="initial state of free scene objects")
    parser.add_argument(
        "--record", type=Path, help="save accepted poses and movable objects for playback"
    )
    args = parser.parse_args()
    report = run(args.scene, args.trace, args.dynamics, args.keyframe, args.record)
    args.out.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf8")
    print(f"{report['status']}, {report['samples']} accepted poses")
    raise SystemExit(2 if args.fail_on_block and report["blocked"] else 0)
