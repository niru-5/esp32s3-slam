"""Host-side tests for the camera calibration tools, against a simulated device.

    cd software && .venv/bin/python -m unittest tests.test_calibration -v
"""

import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from host_server.calibration import intrinsics as ix
from host_server.calibration.cli import App
from host_server.calibration.flow import solve_offline
from host_server.calibration.link import DeviceLink
from host_server.calibration.liveview import LiveView
from host_server.calibration.registers import RegisterBank, load_dump
from host_server.calibration.session import Session

from .fake_device import DIST_TRUE, H, K_TRUE, W, FakeDevice, pose_sequence, render_board

PORT = 18084
BOARD = ix.Board(9, 6, 25.0)


class SolveOnly(unittest.TestCase):
    """calibrateCamera path, no device: detection + solve recovers the ground-truth intrinsics."""

    def test_recovers_known_intrinsics(self):
        obs = []
        for i, (rv, tv) in enumerate(pose_sequence(14, seed=3)):
            det = ix.detect(render_board(rv, tv), BOARD)
            if det.ok:
                obs.append(ix.Observation(f"s{i}", det.corners, det.center, det.area_frac, det.tilt))
        self.assertGreaterEqual(len(obs), 10)
        cal = ix.solve(obs, BOARD, (W, H))
        K = np.array(cal.camera_matrix)
        self.assertLess(cal.rms, 0.5)
        self.assertAlmostEqual(K[0, 0], K_TRUE[0, 0], delta=0.02 * K_TRUE[0, 0])
        self.assertAlmostEqual(K[1, 1], K_TRUE[1, 1], delta=0.02 * K_TRUE[1, 1])
        self.assertAlmostEqual(K[0, 2], K_TRUE[0, 2], delta=6)
        self.assertAlmostEqual(K[1, 2], K_TRUE[1, 2], delta=6)
        self.assertAlmostEqual(cal.dist_coeffs[0][0], DIST_TRUE[0], delta=0.05)

    def test_blank_frame_rejected(self):
        det = ix.detect(np.full((H, W), 128, np.uint8), BOARD)
        self.assertFalse(det.ok)
        self.assertIn("not found", det.reason)

    def test_make_board_png(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "b.png"
            ix.make_board_png(BOARD, p)
            img = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
            self.assertTrue(ix.detect(img, BOARD, min_area_frac=0.0, min_sharpness=0.0).ok)


class RawGeometry(unittest.TestCase):
    def _scene(self):
        y, x = np.mgrid[0:720, 0:1280]
        return (128 + 90 * np.sin(x / 90.0) * np.cos(y / 70.0)).astype(np.uint8)

    def test_coherent_frame_ok(self):
        from host_server.calibration import analysis as an
        self.assertTrue(an.raw_geometry_ok(self._scene()))

    def test_scrambled_rows_detected(self):
        from host_server.calibration import analysis as an
        rng = np.random.default_rng(0)
        scr = np.stack([np.roll(r, int(rng.integers(0, 1280))) for r in self._scene()])
        self.assertFalse(an.raw_geometry_ok(scr))

    def test_dark_noise_frame_not_flagged(self):
        from host_server.calibration import analysis as an
        dark = np.random.default_rng(1).normal(4, 1.2, (720, 1280)).clip(0, 255).astype(np.uint8)
        self.assertTrue(an.raw_geometry_ok(dark))


class OrientedAnalysis(unittest.TestCase):
    """analysis.py's oriented Bayer-tile helpers -- what actually fixes the "debayer error
    after `orient`" report: bayer_planes/plane_stats/debayer must track the sensor's current
    mirror/flip (RegisterBank.mirror/.flip), not just assume the unmirrored/unflipped default."""

    def test_default_orientation_matches_bayer_names(self):
        from host_server.calibration import analysis as an
        self.assertEqual(an.oriented_bayer_names(False, False), an.BAYER_NAMES)
        self.assertEqual(an.oriented_debayer_code(False, False), cv2.COLOR_BayerRG2BGR)

    def test_all_four_orientations_are_distinct_codes(self):
        from host_server.calibration import analysis as an
        codes = {an.oriented_debayer_code(m, f) for m in (False, True) for f in (False, True)}
        self.assertEqual(len(codes), 4)

    def test_bayer_planes_follows_mirror(self):
        from host_server.calibration import analysis as an
        raw = np.zeros((4, 4), np.uint8)
        raw[0::2, 0::2], raw[0::2, 1::2] = 10, 20   # default (0,0)="B", (0,1)="Gb"
        raw[1::2, 0::2], raw[1::2, 1::2] = 30, 40   # default (1,0)="Gr", (1,1)="R"
        default = an.bayer_planes(raw)
        self.assertTrue((default["B"] == 10).all() and (default["R"] == 40).all())
        # mirroring reverses column order: what the default readout calls "Gb" (col 1) is
        # what a mirrored readout now sees first (col 0), i.e. as "B".
        mirrored = an.bayer_planes(raw, mirror=True)
        self.assertTrue((mirrored["B"] == 20).all() and (mirrored["R"] == 30).all())


class WithFakeDevice(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.link = DeviceLink("127.0.0.1", PORT)          # host listens first ...
        self.dev = FakeDevice(PORT)                        # ... the device dials in
        self.hello = self.link.wait_for_device(5)
        self.session = Session(Path(self.tmp.name), "s")
        self.lines: list[str] = []
        self.app = App(self.link, self.session, LiveView(None), ix.Board(9, 6, 25.0), out=self.lines.append)

    def tearDown(self):
        self.link.close()
        self.dev.close()
        self.tmp.cleanup()

    def test_hello(self):
        self.assertEqual(self.hello["mode"], "camera_calibration")

    def test_startup_backup_and_restore(self):
        self.dev.regs[0x5000] = 0xA7                             # gamma bit (0x20) set at backup time
        self.app.startup_backup()
        snap = load_dump(self.session.dir / "settings" / "00_startup.json")
        self.assertEqual(len(snap), 12288)
        self.app.regs.write([(0x5000, 0x00, 0x20)])            # gamma off
        self.app.regs.write([(0x3A00, 0xFF, 0xFF)])
        diffs, n_vol = self.app.regs.restore(snap, dry_run=True)
        self.assertEqual({a for a, _, _ in diffs}, {0x3A00, 0x5000})
        self.assertEqual(n_vol, 0)
        self.app.regs.restore(snap)
        self.assertEqual(self.dev.regs[0x3A00], snap[0x3A00])
        self.assertEqual(self.dev.regs[0x5000], snap[0x5000])

    def test_restore_ignores_volatile_registers(self):
        self.app.startup_backup()
        snap = load_dump(self.session.dir / "settings" / "00_startup.json")
        # a live-statistics register that ticks on every read (like 0x5691 AEC stats)
        ticks = {"n": 0}
        real = self.dev._handle

        def ticking(conn, req):
            if req["cmd"] == "reg_dump":
                ticks["n"] += 1
                self.dev.regs[0x5691] = ticks["n"] & 0xFF
            return real(conn, req)
        self.dev._handle = ticking
        diffs, n_vol = self.app.regs.restore(snap, dry_run=True)
        self.assertEqual(n_vol, 1)
        self.assertEqual(diffs, [])

    def test_undo_log(self):
        before = dict(self.dev.regs)
        m = self.app.regs.mark()
        self.app.regs.write([(0x5000, 0x00, 0x20), (0x3503, 0x03, 0xFF)])
        self.app.regs.set_orientation(True, True)
        self.assertNotEqual(self.dev.regs[0x3820], before[0x3820])
        self.app.regs.revert_to(m)
        self.assertEqual(self.dev.regs[0x5000], before[0x5000])
        self.assertEqual(self.dev.regs[0x3503], before[0x3503])
        self.assertEqual(self.dev.regs[0x3820], before[0x3820])
        self.assertEqual(self.dev.regs[0x3821], before[0x3821])

    def test_new_session_syncs_orientation_from_device(self):
        """A fresh App (a new calibration session) must not assume mirror=flip=False -- it
        should read whatever the device's registers actually are right now (e.g. NVS-
        persisted orientation from an earlier `save --apply`, re-applied by every
        camera_init()), not silently desync analysis.py's oriented Bayer-plane helpers (and
        the calibration panel's "Current" picture) from the real hardware state. Simulate
        that by setting the mirror/flip bits directly (bypassing this App's own
        set_orientation() tracking) and constructing a second App against the same device."""
        self.app.regs.write([(0x3821, 0x06, 0x06), (0x3820, 0x06, 0x06)])
        fresh = App(self.link, self.session, LiveView(None), ix.Board(9, 6, 25.0), out=self.lines.append)
        self.assertTrue(fresh.regs.mirror)
        self.assertTrue(fresh.regs.flip)

    def test_set_orientation_tracks_mirror_flip(self):
        """RegisterBank.mirror/.flip is the source of truth analysis.py's oriented Bayer
        helpers (bayer_planes/plane_stats/debayer) and App.set_mode()'s reapply-after-reset
        both read -- must reflect the last orientation actually requested."""
        self.assertEqual((self.app.regs.mirror, self.app.regs.flip), (False, False))
        self.app.regs.set_orientation(True, False)
        self.assertEqual((self.app.regs.mirror, self.app.regs.flip), (True, False))
        self.app.regs.set_orientation(False, True)
        self.assertEqual((self.app.regs.mirror, self.app.regs.flip), (False, True))

    def test_set_mode_reapplies_orientation(self):
        """A mode switch resets every register on the device, including orientation -- see
        control_link.c camera_init_ex. App.set_mode() must reissue set_orientation() so it
        doesn't silently revert (this was the bug behind the user's debayer-error report)."""
        self.app.regs.set_orientation(True, True)
        self.dev.received.clear()
        self.app.set_mode("raw8", "hd")
        self.assertIn("set_orientation", self.dev.received)
        # still tracked correctly afterwards, not reset to the (now stale) default
        self.assertEqual((self.app.regs.mirror, self.app.regs.flip), (True, True))

    def test_set_mode_skips_reapply_at_default_orientation(self):
        """No orientation was ever set (still mirror=flip=False) -- nothing to reapply."""
        self.dev.received.clear()
        self.app.set_mode("raw8", "hd")
        self.assertNotIn("set_orientation", self.dev.received)

    def test_freeze_loops_writes_exposure_and_gain(self):
        self.app.regs.freeze_loops(exposure_rows=300, gain_x16=0x40)
        self.assertEqual(self.dev.regs[0x3503], 0x03)
        self.assertEqual(self.app.regs.exposure_rows(), 300)
        self.assertEqual(self.app.regs.gain_x16(), 0x40)
        self.assertEqual(self.dev.regs[0x3406], 0x01)

    def test_calibration_capture_flow_recovers_intrinsics(self):
        self.app.run_line("cal capture 6 0")
        self.app.run_line("cal capture 6 0")
        self.app.run_line("cal capture 6 0")
        flow = self.app.flow
        self.assertGreaterEqual(len(flow.obs), 12, "\n".join(self.lines))
        cal = flow.solve(save=True)
        K = np.array(cal.camera_matrix)
        self.assertAlmostEqual(K[0, 0], K_TRUE[0, 0], delta=0.02 * K_TRUE[0, 0])
        # frames + sidecar metadata stored in the documented layout
        imgs = sorted((self.session.dir / "intrinsics" / "images").glob("img_*.jpg"))
        self.assertEqual(len(imgs), len(flow.obs))
        self.assertTrue(imgs[0].with_suffix(".json").exists())
        self.assertTrue(list((self.session.dir / "intrinsics" / "results").glob("camchain_*.yaml")))
        # ... and the saved session can be re-solved later, offline
        cal2 = solve_offline(self.session.dir, BOARD, out=lambda *_: None, save=False)
        self.assertAlmostEqual(np.array(cal2.camera_matrix)[0, 0], K_TRUE[0, 0], delta=0.02 * K_TRUE[0, 0])

    def test_duplicate_pose_rejected(self):
        self.dev.poses = self.dev.poses[:1]                     # same pose every time
        self.app.run_line("cal capture 3 0")
        self.assertEqual(len(self.app.flow.obs), 1)
        self.assertEqual(self.app.flow.n_rejected, 2)
        self.assertTrue(any("too similar" in l for l in self.lines))

    def test_device_redials_after_link_drop(self):
        self.link.close_conn()                             # simulate a dropped link
        hello = self.link.wait_for_device(5)
        self.assertEqual(hello["mode"], "camera_calibration")
        self.assertEqual(self.link.call("ping").data["ok"], True)

    def test_exit_command(self):
        self.assertFalse(self.app.run_line("exit"))
        self.dev.thread.join(2)
        self.assertTrue(self.dev.exit_requested)


if __name__ == "__main__":
    unittest.main()
