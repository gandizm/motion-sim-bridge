"""View the accepted states of a recorded replay, or verify it without a display."""

import argparse
import hashlib
from pathlib import Path
import time

import mujoco as mj
import numpy as np


def load_recording(scene, record):
    m = mj.MjModel.from_xml_path(str(scene))
    with np.load(record, allow_pickle=False) as saved:
        if str(saved["scene_sha256"]) != hashlib.sha256(Path(scene).read_bytes()).hexdigest():
            raise ValueError("Recording belongs to a different scene file")
        times, poses = saved["times"], saved["qpos"]
    if times.ndim != 1 or poses.shape != (len(times), m.nq) or not np.isfinite(poses).all():
        raise ValueError("Malformed recording")
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Invalid recorded times")
    return m, times, poses


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scene", type=Path)
    parser.add_argument("record", type=Path)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    m, times, poses = load_recording(args.scene, args.record)
    print(f"{len(times)} saved states")
    if args.check or not len(times):
        return
    if not np.isfinite(args.speed) or args.speed <= 0:
        parser.error("speed must be positive")
    import mujoco.viewer

    d = mj.MjData(m)
    with mujoco.viewer.launch_passive(m, d) as viewer:
        start = time.monotonic()
        for t, q in zip(times, poses, strict=True):
            deadline = start + (t - times[0]) / args.speed
            while viewer.is_running() and time.monotonic() < deadline:
                time.sleep(max(0.0, min(0.02, deadline - time.monotonic())))
            if not viewer.is_running():
                break
            with viewer.lock():
                d.time = t
                d.qpos[:] = q
                mj.mj_forward(m, d)
            viewer.sync()
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.05)


if __name__ == "__main__":
    main()
