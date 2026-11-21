"""Independent synthetic contact regressions. All geometry is generated in a temp directory."""

import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from replay import REQUIRED, read_trace, run
from compare import compare

VEHICLE = Path(__file__).parent / "assets" / "vehicle.xml"


class ContactTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.xml = ET.parse(VEHICLE)
        self.world = self.xml.find("worldbody")

    def fixture(self, name="fixture", **attrs):
        return ET.SubElement(
            self.world, "geom", dict(name=name, contype="1", conaffinity="6", **attrs)
        )

    def trace(self, rows, name="path"):
        p = self.dir / (name + ".jsonl")
        p.write_text(
            "\n".join(
                json.dumps(r if isinstance(r, dict) else dict(zip(REQUIRED, r, strict=True)))
                for r in rows
            )
        )
        return p

    def replay(self, rows, dynamic=False):
        scene = self.dir / "scene.xml"
        self.xml.write(scene)
        return run(scene, self.trace(rows), dynamic)

    def test_thin_wall_and_retreat(self):
        self.fixture(type="box", pos=".38 0 .08", size=".00001 .26 .08")
        r = self.replay([(0, 0, 0, 0, 0, 0), (0.015, 0.8, 0, 0, 0, 0)])
        self.assertEqual(r["status"], "blocked")
        final = r["final_pose"]
        self.assertAlmostEqual(final["x_m"], 0.38 - 0.00001 - 0.223664, delta=0.00008)
        self.assertLess(r["max_accepted_penetration_mm"], 0.001)
        self.assertEqual(self.replay([final, dict(final, t_s=0.1, x_m=0)])["status"], "completed")

    def test_disjoint_endpoints_tiny_tip(self):
        # A complete crossing fits between consecutive 1 mm sample points.
        for b in self.world.iter("body"):
            for g in list(b.findall("geom")):
                b.remove(g)
            ET.SubElement(
                b,
                "geom",
                name=b.get("name") + "_tip",
                type="sphere",
                size=".00006",
                pos="0 -.4 .1" if b.get("name") == "front" else "0 0 .1",
                contype="2",
                conaffinity="1",
            )
        self.fixture(type="sphere", pos=".01251 0 .1", size=".00003")
        r = self.replay([(0, 0, 0, 0, 0, 0), (0.001, 0.03, 0, 0, 0, 0)])
        self.assertEqual(r["status"], "blocked")
        self.assertAlmostEqual(r["final_pose"]["x_m"], 0.01242, delta=0.000004)
        r = self.replay([(0, 0, 0.0001, 0, 0, 0), (0.001, 0.03, 0.0001, 0, 0, 0)])
        self.assertEqual(r["status"], "completed")

    def test_start_inside(self):
        self.fixture(type="box", size=".06 .05 .06", pos="0 0 .08")
        r = self.replay([(0, 0, 0, 0, 0, 0), (1, 0.6, 0, 0, 0, 0)])
        self.assertEqual(r["blocked"]["reason"], "initial_overlap")
        self.assertIsNone(r["final_pose"])
        self.assertEqual(r["samples"], 0)

    def test_full_turn(self):
        self.fixture(type="cylinder", pos=".4 0 .04", size=".018 .04")
        r = self.replay([(0, 0.4, -0.2, 0, 0, 0), (1, 0.4, -0.2, 360, 0, 0)])
        self.assertEqual(r["status"], "blocked")
        self.assertLess(r["final_pose"]["yaw_deg"], 90)

    def test_fingers_sweep_both_sides(self):
        for sign, name in [(1, "jaw_left"), (-1, "jaw_right")]:
            with self.subTest(name=name):
                g = self.fixture(type="cylinder", pos=f".4 {sign*.1} .04", size=".018 .04")
                r = self.replay([(0, 0.2, 0, 0, 0, 0), (1, 0.2, 0, 0, 55, -55)])
                self.assertEqual(r["blocked"]["part"], name)
                self.world.remove(g)

    def test_fingers_cannot_cross(self):
        r = self.replay([(0, 0, 0, 0, 0, 0), (1, 0, 0, 0, -32, 32)])
        self.assertEqual({r["blocked"]["part"], r["blocked"]["object"]}, {"jaw_left", "jaw_right"})
        self.assertGreater(r["final_pose"]["left_deg"], -1)

    def test_front_tilt_clear_up_and_block_down(self):
        self.fixture(name="support", type="plane", size="1 1 .01")
        a = dict(zip(REQUIRED, (0, 0, 0, 0, 0, 0)), front_deg=0)
        up = self.replay([a, dict(a, t_s=1, front_deg=-15)])
        self.assertEqual(up["status"], "completed")
        down = self.replay([a, dict(a, t_s=1, front_deg=8)])
        self.assertEqual(down["blocked"]["part"], "blade")
        self.assertLess(down["final_pose"]["front_deg"], 1)
        self.assertEqual(down["max_accepted_penetration_mm"], 0)

    def test_pitch_into_floor(self):
        self.fixture(name="support", type="plane", size="1 1 .01")
        a = dict(zip(REQUIRED, (0, 0, 0, 0, 0, 0)), z_m=0.009, pitch_deg=0)
        r = self.replay([a, dict(a, t_s=1, pitch_deg=20)])
        self.assertEqual(r["blocked"]["object"], "support")
        self.assertLess(r["final_pose"]["pitch_deg"], 20)

    def test_free_and_trapped_object(self):
        self.fixture(name="support", type="plane", size="1 1 .01")
        body = ET.SubElement(self.world, "body", name="loose", pos=".285 0 .024")
        ET.SubElement(body, "freejoint")
        ET.SubElement(
            body,
            "geom",
            name="payload",
            type="box",
            size=".014 .024 .024",
            mass=".033",
            contype="4",
            conaffinity="7",
        )
        rows = [(0, 0, 0, 0, 0, 0), (1, 0.19, 0, 0, 0, 0)]
        r = self.replay(rows, True)
        self.assertEqual(r["status"], "completed")
        self.assertGreater(r["target_delta_m"]["loose"][0], 0.10)
        self.assertLessEqual(r["max_accepted_penetration_mm"], 0.401)
        self.fixture(name="stop", type="box", pos=".338 0 .036", size=".001 .09 .036")
        r = self.replay(rows, True)
        self.assertEqual(r["status"], "blocked")
        self.assertLess(r["final_pose"]["x_m"], 0.12)
        self.assertLess(0.285 + r["target_delta_m"]["loose"][0], 0.338)
        self.assertLessEqual(r["max_accepted_penetration_mm"], 0.401)

    def test_random_tiny_transits(self):
        import math
        import random

        rng = random.Random(218)
        # A compact probe with analytic sphere/sphere clearance.
        self.xml = ET.ElementTree(
            ET.fromstring(
                """<mujoco><worldbody>
          <body name="vehicle"><freejoint name="base"/><geom name="probe" mass=".1" type="sphere" size=".00008" contype="2" conaffinity="1"/>
          <body name="jaw_left" pos="0 .2 .1"><joint name="jaw_left_angle"/><geom mass=".01" size=".001" contype="2" conaffinity="1"/></body>
          <body name="jaw_right" pos="0 -.2 .1"><joint name="jaw_right_angle"/><geom mass=".01" size=".001" contype="2" conaffinity="1"/></body>
          </body><geom name="bead" size=".00004" contype="1" conaffinity="2"/></worldbody></mujoco>"""
            )
        )
        for i in range(40):
            theta = rng.uniform(-math.pi, math.pi)
            c, s = math.cos(theta), math.sin(theta)
            gap = rng.uniform(0.00002, 0.00010) if i % 2 else rng.uniform(0.00014, 0.00032)
            start = -rng.uniform(0.005, 0.019)
            end = rng.uniform(0.007, 0.024)
            rows = [
                (0, start * c - gap * s, start * s + gap * c, 0, 0, 0),
                (0.001, end * c - gap * s, end * s + gap * c, 0, 0, 0),
            ]
            with self.subTest(i=i):
                result = self.replay(rows)
                self.assertEqual(result["status"], "blocked" if gap < 0.00012 else "completed")
                if result["blocked"]:
                    ref = -math.sqrt(0.00012**2 - gap**2)
                    pose = result["final_pose"]
                    along = pose["x_m"] * c + pose["y_m"] * s
                    self.assertAlmostEqual(along, ref, delta=0.000012)

    def test_fast_free_object(self):
        from mujoco import MjModel

        self.fixture(name="foil", type="box", pos=".56 0 .12", size=".00001 .06 .06")
        body = ET.SubElement(self.world, "body", name="ball", pos=".52 0 .12")
        ET.SubElement(body, "freejoint", name="ball_free")
        ET.SubElement(
            body,
            "geom",
            name="ball_shape",
            type="sphere",
            size=".003",
            mass=".01",
            contype="4",
            conaffinity="7",
        )
        path = self.dir / "scene.xml"
        self.xml.write(path)
        m = MjModel.from_xml_path(str(path))
        velocity = [0.0] * m.nv
        velocity[m.joint("ball_free").dofadr[0]] = 90.0
        keys = ET.SubElement(self.xml.getroot(), "keyframe")
        ET.SubElement(keys, "key", name="launch", qvel=" ".join(map(str, velocity)))
        self.xml.write(path)
        trace = self.trace([(0, 0, 0, 0, 0, 0), (0.006, 0, 0, 0, 0, 0)])
        result = run(path, trace, True, "launch")
        self.assertGreater(result["target_delta_m"]["ball"][0], 0.025)
        self.assertLess(0.52 + result["target_delta_m"]["ball"][0], 0.56)
        self.assertLess(result["max_accepted_penetration_mm"], 0.401)

    def test_record_stops_before_obstacle(self):
        import numpy as np
        from play import load_recording

        self.fixture(type="box", pos=".38 0 .08", size=".00001 .26 .08")
        scene = self.dir / "scene.xml"
        self.xml.write(scene)
        tr = self.trace([(0, 0, 0, 0, 0, 0), (1, 0.8, 0, 0, 0, 0)])
        file = self.dir / "states.npz"
        r = run(scene, tr, record=file)
        model, t, q = load_recording(scene, file)
        self.assertAlmostEqual(t[-1], r["final_pose"]["t_s"])
        self.assertAlmostEqual(q[-1, 0], r["final_pose"]["x_m"])
        self.assertTrue(np.all(q[:, 0] <= r["final_pose"]["x_m"]))
        self.assertEqual(r["blocked"]["part"], run(scene, tr)["blocked"]["part"])
        scene.write_text(scene.read_text() + " ")
        with self.assertRaisesRegex(ValueError, "different scene"):
            load_recording(scene, file)

    def test_time_and_metadata(self):
        rows = [(0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)]
        with self.assertRaisesRegex(ValueError, "time must increase"):
            read_trace(self.trace(rows))
        a = dict(zip(REQUIRED, rows[0]))
        for bad in [
            dict(a, t_s=1, x_m=True),
            dict(a, t_s=1, left_deg=float("nan")),
            dict(a, t_s=1, z_m=0.03),
        ]:
            with self.assertRaises(ValueError):
                read_trace(self.trace([a, bad]))
        scene = self.dir / "scene.xml"
        self.xml.write(scene)
        x = self.trace([(0, 0, 0, 0, 0, 0), (2, 0.04, 0, 0, 0, 0)], "cmd")
        y = self.trace([(0.5, 0.01, 0.01, 0, 0, 0), (1.5, 0.03, 0.01, 0, 0, 0)], "fb")
        r = compare(scene, x, y)
        self.assertAlmostEqual(r["peak_position_error_mm"], 10)
        self.assertEqual(r["common_time_s"], [0.5, 1.5])
        self.assertNotEqual(
            r["command"]["inputs"]["trace_sha256"], r["feedback"]["inputs"]["trace_sha256"]
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
