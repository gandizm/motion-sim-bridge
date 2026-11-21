"""Conservative checks for swept rigid convex shapes in a planar vehicle scene."""

from collections import defaultdict
from itertools import combinations
import math

import mujoco as mj
import numpy as np

EPS = 2e-7  # metres, permits coincident support at numerical precision
CONTACT_M = 0.0004  # upper bound for compliant contact with movable bodies
PLANAR = (mj.mjtGeom.mjGEOM_PLANE,)
SUPPORTED = {
    mj.mjtGeom.mjGEOM_BOX,
    mj.mjtGeom.mjGEOM_SPHERE,
    mj.mjtGeom.mjGEOM_CYLINDER,
    mj.mjtGeom.mjGEOM_CAPSULE,
    mj.mjtGeom.mjGEOM_ELLIPSOID,
    mj.mjtGeom.mjGEOM_MESH,
    *PLANAR,
}


class Guard:
    def __init__(self, model, data, dynamics=False):
        self.m, self.d = model, data
        self.base = model.body("vehicle").id
        self.jaws = [model.body(n).id for n in ("jaw_left", "jaw_right")]
        self.names = [model.geom(i).name or f"geom_{i}" for i in range(model.ngeom)]
        self.parts = [
            n.rsplit("_", 1)[0] if n.rsplit("_", 1)[-1].isdigit() else n for n in self.names
        ]
        self.weld = model.body_weldid[model.geom_bodyid]
        self.front = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "front")
        self.links = {self.base, *self.jaws}
        if self.front >= 0:
            self.links.add(self.front)
        self.robot = np.isin(self.weld, list(self.links))
        self.free = (~self.robot) & (self.weld != 0)
        self.planes = model.geom_type == mj.mjtGeom.mjGEOM_PLANE
        # Circumsphere around the joint, not around the centre of mass.
        self.local_radius = (
            np.linalg.norm(data.geom_xpos - data.xpos[self.weld], axis=1) + model.geom_rbound
        )
        base_radius = (
            np.linalg.norm(data.geom_xpos - data.xpos[self.base], axis=1) + model.geom_rbound
        )
        for jaw in self.jaws:
            mask = self.weld == jaw
            base_radius[mask] = self.local_radius[mask] + np.linalg.norm(
                data.xpos[jaw] - data.xpos[self.base]
            )
        self.body_radius = max(base_radius[self.robot])
        self.jaw_radius = max(self.local_radius[np.isin(self.weld, self.jaws)])
        self.front_radius = 0.0
        if self.front >= 0:
            pivot = data.xanchor[model.body_jntadr[self.front]]
            mask = self.weld == self.front
            self.front_radius = max(
                np.linalg.norm(data.geom_xpos[mask] - pivot, axis=1) + model.geom_rbound[mask],
                default=0.0,
            )
            for jaw in self.jaws:
                self.front_radius = max(
                    self.front_radius, np.linalg.norm(data.xpos[jaw] - pivot) + self.jaw_radius
                )
            self.body_radius = max(
                self.body_radius, np.linalg.norm(pivot - data.xpos[self.base]) + self.front_radius
            )
        self.groups = defaultdict(list)
        explicit = {
            tuple(sorted((int(a), int(b)))) for a, b in zip(model.pair_geom1, model.pair_geom2)
        }
        active = sorted(
            set(np.flatnonzero(model.geom_contype | model.geom_conaffinity)).union(*explicit)
        )
        self.limits = {}
        for a, b in combinations(active, 2):
            if self.weld[a] == self.weld[b]:
                continue
            self_pair = self.robot[a] and self.robot[b]
            if self_pair:
                # The carrier includes the fastened hinge seat in its convex proxy.
                if {self.parts[a], self.parts[b]} in (
                    {"jaw_carrier", "jaw_left"},
                    {"jaw_carrier", "jaw_right"},
                    {"tilt_arm", "tilt_mount"},
                ):
                    continue
            elif (a, b) not in explicit and not (
                model.geom_contype[a] & model.geom_conaffinity[b]
                or model.geom_contype[b] & model.geom_conaffinity[a]
            ):
                continue
            if not all(model.geom_type[i] in SUPPORTED for i in (a, b)):
                raise ValueError(
                    f"Unsupported collision geometry: {self.names[a]}, {self.names[b]}"
                )
            flexible = self.free[a] or self.free[b]
            wheel_support = any(
                self.planes[i] and self.parts[j].startswith("wheel_") for i, j in ((a, b), (b, a))
            )
            limit = 0.0
            if dynamics and (flexible or wheel_support):
                thickness = [min(model.geom_aabb[i, 3:]) for i in (a, b) if not self.planes[i]]
                limit = min(CONTACT_M, 0.25 * min(thickness)) if thickness else CONTACT_M
            self.limits[(int(a), int(b))] = limit
            self.groups[(int(self.weld[a]), int(self.weld[b]))].append((int(a), int(b), limit))
        self.groups = {
            key: (
                np.asarray(vals)[:, 0].astype(int),
                np.asarray(vals)[:, 1].astype(int),
                np.asarray(vals)[:, 2],
            )
            for key, vals in self.groups.items()
        }

    def geometry(self):
        d, m = self.d, self.m
        rot = d.geom_xmat.reshape(-1, 3, 3)
        center = d.geom_xpos + np.einsum("ijk,ik->ij", rot, m.geom_aabb[:, :3])
        ext = np.einsum("ijk,ik->ij", np.abs(rot), m.geom_aabb[:, 3:])
        return center - ext, center + ext

    def motion(self, delta, free_motion):
        linear = np.linalg.norm(delta[[1, 2, 6]])
        turn = math.radians(abs(delta[3]) + sum(abs(delta[7:9])))
        moves = {0: 0.0, self.base: linear + turn * self.body_radius}
        if self.front >= 0:
            moves[self.front] = moves[self.base] + math.radians(abs(delta[9])) * self.front_radius
        for i, jaw in enumerate(self.jaws):
            moves[jaw] = (
                moves.get(self.front, moves[self.base])
                + math.radians(abs(delta[4 + i])) * self.jaw_radius
            )
        moves.update(free_motion)
        return moves

    def inspect(self, bounds, delta, free_motion, initial=False):
        lo, hi = bounds
        moves = self.motion(delta, free_motion)
        for (wa, wb), (aa, bb, limits) in self.groups.items():
            travel = moves.get(wa, 0.0) + moves.get(wb, 0.0)
            if wa in self.links and wb in self.links:
                travel = self.jaw_radius * math.radians(
                    sum(abs(delta[4 + i]) for i, j in enumerate(self.jaws) if j in (wa, wb))
                )
                if self.base in (wa, wb):
                    travel += self.front_radius * math.radians(abs(delta[9]))
            if travel == 0 and not initial:
                continue
            # Analytic interval AABBs reject separated pairs before any narrow-phase query.
            gap = np.maximum(lo[aa] - hi[bb], lo[bb] - hi[aa]).max(axis=1)
            for k in np.flatnonzero(gap <= travel - limits + EPS):
                a, b = aa[k], bb[k]
                bound = travel
                # Horizontal rigid motion never changes the gap to a horizontal plane.
                if self.planes[a] or self.planes[b]:
                    plane, other = (a, b) if self.planes[a] else (b, a)
                    if self.robot[other] and abs(self.d.geom_xmat[plane, 8] - 1) < 1e-12:
                        bound = abs(delta[6]) + self.body_radius * math.radians(
                            sum(abs(delta[7:9]))
                        )
                        if self.weld[other] != self.base:
                            bound += self.front_radius * math.radians(abs(delta[9]))
                        if (
                            abs(self.d.xmat[self.weld[other], 8] - 1) > 1e-12 or np.any(delta[7:10])
                        ) and self.weld[other] in self.jaws:
                            bound += self.jaw_radius * math.radians(sum(abs(delta[4:6])))
                cutoff = max(0.001, bound + EPS * 4)
                points = np.zeros(6)
                dist = mj.mj_geomDistance(self.m, self.d, int(a), int(b), cutoff, points)
                translating = (
                    not self.free[a]
                    and not self.free[b]
                    and not np.any(delta[[3, 4, 5, 7, 8, 9]])
                    and self.robot[a] != self.robot[b]
                )
                if translating and dist < cutoff - EPS:
                    # A separating plane of two convex bodies remains valid under translation.
                    normal = points[3:] - points[:3]
                    if dist > EPS and np.linalg.norm(normal) > EPS:
                        normal /= np.linalg.norm(normal)
                    else:
                        normal = None
                        for c in self.d.contact:
                            if tuple(c.geom) == (a, b):
                                normal = c.frame[:3]
                                break
                            if tuple(c.geom) == (b, a):
                                normal = -c.frame[:3]
                                break
                    if normal is not None:
                        rel = delta[[1, 2, 6]] * (1 if self.robot[b] else -1)
                        predicted = dist + min(0.0, float(normal @ rel))
                        if dist >= -limits[k] - EPS and predicted >= -limits[k] - EPS:
                            continue
                if dist - bound < -limits[k] - EPS or (not limits[k] and dist < EPS and bound > 0):
                    return int(a), int(b), float(dist), float(limits[k])
        return None

    def endpoint(self, data):
        max_depth = 0.0
        for c in data.contact:
            if c.dist >= 0:
                continue
            a, b = sorted(map(int, c.geom))
            limit = self.limits.get((a, b))
            if limit is None:
                continue
            max_depth = max(max_depth, -c.dist)
            if c.dist < -limit - EPS:
                return (a, b, float(c.dist), limit), max_depth
        return None, max_depth

    def describe(self, a, b):
        if self.robot[b] and not self.robot[a]:
            a, b = b, a
        return (
            self.parts[a] if self.robot[a] else self.names[a],
            self.parts[b] if self.robot[b] else self.names[b],
        )
