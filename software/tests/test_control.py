"""Host-side tests for the unified host_server.app, against a simulated device.

    cd software && .venv/bin/python -m unittest tests.test_control -v

Needs numpy/cv2 (host_server.app pulls in host_server.calibration.cli), unlike the plain
streaming ingest modules -- run with an interpreter that has them (see
docs/camera_calibration_and_tuning.md "Tests").
"""

from __future__ import annotations

import json
import threading
import time
import unittest
import urllib.error
import urllib.request

from host_server.app import Server, make_server

from .fake_control_device import FakeControlDevice


def _post(http_port: int, path: str, obj: dict) -> dict:
    req = urllib.request.Request(
        f"http://127.0.0.1:{http_port}{path}",
        data=json.dumps(obj).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=10) as resp:
        return json.loads(resp.read())


def _get(http_port: int, path: str) -> dict:
    with urllib.request.urlopen(f"http://127.0.0.1:{http_port}{path}", timeout=10) as resp:
        return json.loads(resp.read())


class ServerNoDevice(unittest.TestCase):
    def setUp(self):
        self.state = Server(device_port=0, calib_root=None, tcp_host="127.0.0.1",
                            tcp_ports=(0, 0, 0))
        self.server = make_server("127.0.0.1", 0, self.state)
        self.http_port = self.server.server_address[1]
        import threading
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.state.close()

    def test_index_served(self):
        with urllib.request.urlopen(f"http://127.0.0.1:{self.http_port}/", timeout=5) as resp:
            self.assertEqual(resp.status, 200)
            self.assertIn(b"<html", resp.read().lower())

    def test_status_json_without_device(self):
        j = _get(self.http_port, "/status.json")
        self.assertFalse(j["device"]["connected"])
        self.assertFalse(j["calibration_active"])

    def test_command_without_device(self):
        j = _post(self.http_port, "/command", {"cmd": "set_state", "state": "idle"})
        self.assertFalse(j["ok"])
        self.assertIn("not connected", j["err"])

    def test_calib_line_without_console(self):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.http_port}/calib/line",
            data=json.dumps({"line": "help"}).encode(),
            headers={"Content-Type": "application/json"})
        try:
            urllib.request.urlopen(req, timeout=5)
            self.fail("expected an error status")
        except urllib.error.HTTPError as exc:
            self.assertEqual(exc.code, 409)


class ServerWithFakeDevice(unittest.TestCase):
    def setUp(self):
        self.state = Server(device_port=0, calib_root=None, tcp_host="127.0.0.1",
                            tcp_ports=(0, 0, 0))
        self.server = make_server("127.0.0.1", 0, self.state)
        self.http_port = self.server.server_address[1]
        import threading
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.dev = FakeControlDevice(self.state.device.port)
        for _ in range(50):
            if self.state.device.connected:
                break
            time.sleep(0.05)
        self.assertTrue(self.state.device.connected)

    def tearDown(self):
        self.dev.close()
        self.server.shutdown()
        self.server.server_close()
        self.state.close()

    def test_status_reflects_device(self):
        j = _get(self.http_port, "/status.json")
        self.assertTrue(j["device"]["connected"])
        self.assertEqual(j["device"]["get_status"]["state"], "idle")

    def test_set_state_round_trip(self):
        j = _post(self.http_port, "/command", {"cmd": "set_state", "state": "stream_wifi"})
        self.assertTrue(j["ok"])
        self.assertEqual(self.dev.state, "stream_wifi")

    def test_set_recording_without_ros_reports_clearly(self):
        j = _post(self.http_port, "/command", {"cmd": "set_recording", "enabled": True})
        # This test environment has no sourced ROS 2 -- expect a clear, non-crashing failure.
        self.assertFalse(j["ok"])
        self.assertIn("ROS", j["err"])

    def test_camera_calibration_lifecycle(self):
        j = _post(self.http_port, "/command", {"cmd": "set_state", "state": "camera_calibration"})
        self.assertTrue(j["ok"])
        self.assertEqual(self.dev.state, "camera_calibration")

        out = _get(self.http_port, "/calib/output.json?since=0")
        self.assertTrue(out["active"])
        self.assertTrue(any("backing up all sensor registers" in line for line in out["lines"]))

        r = _post(self.http_port, "/calib/line", {"line": "info"})
        self.assertTrue(r["ok"])
        for _ in range(50):
            out = _get(self.http_port, f"/calib/output.json?since={out['next']}")
            if out["lines"]:
                break
            time.sleep(0.05)
        self.assertTrue(any("sensor_pid" in line for line in out["lines"]))

        # capture exercises MSG_IMAGE handling through the merged protocol.
        r = _post(self.http_port, "/calib/line", {"line": "capture 1"})
        self.assertTrue(r["ok"])

        # exit (a console command, not /command) -- the background poller should notice the
        # device left camera_calibration and tear the console down on its own.
        _post(self.http_port, "/calib/line", {"line": "exit"})
        for _ in range(40):
            if self.dev.state == "idle" and not _get(self.http_port, "/status.json")["calibration_active"]:
                break
            time.sleep(0.1)
        self.assertEqual(self.dev.state, "idle")
        self.assertFalse(_get(self.http_port, "/status.json")["calibration_active"])

    def test_tcp_ports_open_on_demand(self):
        self.assertFalse(self.state.tcp.running)
        _post(self.http_port, "/command", {"cmd": "set_state", "state": "stream_tcp"})
        self.assertTrue(self.state.tcp.running)
        _post(self.http_port, "/command", {"cmd": "set_state", "state": "idle"})
        self.assertFalse(self.state.tcp.running)

    def test_concurrent_enter_builds_exactly_one_session(self):
        """Regression test for a race found on real hardware: CalibrationManager.enter()'s
        body includes App.startup_backup()'s ~9s reg_dump, so the naive "if active: return"
        guard left a wide check-then-act window where two callers (the HTTP handler thread
        driving a just-issued set_state, and the background poller, which independently
        notices camera_calibration too) could both pass the guard and each build their own
        Session/App -- console commands submitted in between landed on whichever instance
        happened to be `self.calib.console` at that moment, silently splitting one operator
        session across two orphaned ones. Slow the fake device's reg_dump down to widen the
        window reliably instead of relying on real hardware's ~9s to hit it by chance."""
        self.dev.reg_dump_delay = 0.3
        barrier = threading.Barrier(2)
        results: list = []

        def enter_once():
            barrier.wait(timeout=5)
            self.state.calib.enter()
            results.append(self.state.calib.session)

        threads = [threading.Thread(target=enter_once) for _ in range(2)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10)

        self.assertEqual(len(results), 2)
        self.assertIsNotNone(results[0])
        self.assertIs(results[0], results[1], "two concurrent enter() calls built two different sessions")

    def test_imu_event_surfaced_in_status(self):
        self.dev.send_event({"evt": "imu_cal_preview", "prompt": "pick axis",
                             "mean_accel_g": {"x": 0.0, "y": 0.0, "z": 1.0}})
        for _ in range(50):
            j = _get(self.http_port, "/status.json")
            if j["device"]["last_event"] is not None:
                break
            time.sleep(0.05)
        self.assertEqual(j["device"]["last_event"]["evt"], "imu_cal_preview")


if __name__ == "__main__":
    unittest.main()
