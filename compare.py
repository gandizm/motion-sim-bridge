"""Compare two time-aligned traces and replay them independently on one scene."""

import argparse
import json
from pathlib import Path

import numpy as np
from replay import FIELDS, read_trace, run


def compare(scene, command, feedback, dynamics=False, command_rev=None, controller_rev=None):
    cmd, fb = read_trace(command), read_trace(feedback)
    start = max(cmd[0, 0], fb[0, 0])
    end = min(cmd[-1, 0], fb[-1, 0])
    if start >= end:
        raise ValueError("Traces need overlapping time on the same clock")
    times = np.unique(np.r_[cmd[:, 0], fb[:, 0], start, end])
    times = times[(times >= start) & (times <= end)]

    def interpolate(rows):
        return np.column_stack(
            [np.interp(times, rows[:, 0], rows[:, i]) for i in range(1, len(FIELDS))]
        )

    diff = interpolate(fb) - interpolate(cmd)
    position_mm = np.linalg.norm(diff[:, [0, 1, 5]], axis=1) * 1000
    peak = int(np.argmax(position_mm))
    planned = run(scene, command, dynamics)
    measured = run(scene, feedback, dynamics)
    key = (bool(planned["blocked"]), bool(measured["blocked"]))
    finding = {
        (False, False): "both_completed",
        (True, False): "command_blocked",
        (False, True): "feedback_blocked",
        (True, True): "both_blocked",
    }[key]
    return dict(
        observation=finding,
        command_revision=command_rev,
        controller_revision=controller_rev,
        common_time_s=[float(start), float(end)],
        peak_position_error_mm=float(position_mm[peak]),
        peak_position_error_t_s=float(times[peak]),
        peak_unwrapped_heading_error_deg=float(max(abs(diff[:, 2]))),
        peak_left_error_deg=float(max(abs(diff[:, 3]))),
        peak_right_error_deg=float(max(abs(diff[:, 4]))),
        peak_roll_error_deg=float(max(abs(diff[:, 6]))),
        peak_pitch_error_deg=float(max(abs(diff[:, 7]))),
        peak_front_error_deg=float(max(abs(diff[:, 8]))),
        command=planned,
        feedback=measured,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for k in ("scene", "command", "feedback"):
        p.add_argument(k, type=Path)
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--dynamics", action="store_true")
    p.add_argument("--command-rev")
    p.add_argument("--controller-rev")
    a = p.parse_args()
    out = compare(a.scene, a.command, a.feedback, a.dynamics, a.command_rev, a.controller_rev)
    a.out.write_text(json.dumps(out, indent=2, allow_nan=False), encoding="utf8")
    print(out["command"]["status"], out["feedback"]["status"])
