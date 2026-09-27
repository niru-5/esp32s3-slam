"""Host-side tests for software/host_server/control, against a simulated device.

    cd software && .venv/bin/python -m unittest tests.test_control -v

Stdlib only, like host_server.control itself -- no numpy/cv2 dependency.
"""

from __future__ import annotations

import json
import time
import unittest
import urllib.error
import urllib.request

from host_server.control.app import make_server
from host_server.control.device_channel import ControlLink, ControlLinkError

from .fake_control_device import FakeControlDevice


def _post(http_port: int, path: str, obj: dict) -> dict:
    req = urllib.request.Request(
        f"http://127.0.0.1:{http_port}{path}",
        data=json.dumps(obj).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=5) as resp:
        return json.loads(resp.read())


def _get(http_port: int, path: str) -> tuple[int, bytes]:
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{http_port}{path}", timeout=5) as resp:
            return resp.status, resp.read()
    except urllib.error.HTTPError as exc:
        return exc.code, b""


class ControlLinkNoDevice(unittest.TestCase):
    """No device connected: calls fail cleanly instead of hanging."""

    def setUp(self):
        self.link = ControlLink("127.0.0.1", 0)   # port 0: OS picks a free one, no rebind races

    def tearDown(self):
        self.link.close()

    def test_call_without_device_raises(self):
        with self.assertRaises(ControlLinkError):
            self.link.call("get_status", timeout=1.0)

    def test_status_reports_disconnected(self):
        s = self.link.status()
        self.assertFalse(s["connected"])
        self.assertIsNone(s["last_event"])


class ControlLinkWithFakeDevice(unittest.TestCase):
    def setUp(self):
        self.link = ControlLink("127.0.0.1", 0)         # host listens first ...
        self.dev = FakeControlDevice(self.link.port)    # ... the device dials in
        for _ in range(50):
            if self.link.connected:
                break
            time.sleep(0.05)
        self.assertTrue(self.link.connected)

    def tearDown(self):
        self.dev.close()
        self.link.close()

    def test_get_status_round_trip(self):
        r = self.link.call("get_status", timeout=3.0)
        self.assertEqual(r["state"], "idle")
        self.assertEqual(r["heap_free"], 111)

    def test_set_state_round_trip(self):
        r = self.link.call("set_state", state="stream_wifi", timeout=3.0)
        self.assertEqual(r["requested"], "stream_wifi")
        self.assertEqual(self.dev.state, "stream_wifi")
        # a fresh get_status reflects it
        r2 = self.link.call("get_status", timeout=3.0)
        self.assertEqual(r2["state"], "stream_wifi")

    def test_unhandled_command_raises(self):
        with self.assertRaises(ControlLinkError):
            self.link.call("no_such_cmd", timeout=3.0)

    def test_event_captured_without_a_pending_call(self):
        self.dev.send_event({"evt": "imu_cal_preview", "prompt": "pick axis",
                             "mean_accel_g": {"x": 0.01, "y": 0.02, "z": 0.99}})
        for _ in range(50):
            if self.link.last_event is not None:
                break
            time.sleep(0.05)
        self.assertIsNotNone(self.link.last_event)
        self.assertEqual(self.link.last_event["evt"], "imu_cal_preview")

    def test_disconnect_fails_pending_calls(self):
        # A call that's waiting when the device vanishes should raise, not hang forever.
        import threading
        result: dict = {}

        def do_call():
            try:
                result["r"] = self.link.call("get_status", timeout=5.0)
            except ControlLinkError as exc:
                result["err"] = str(exc)

        # Monkeypatch the fake device to never answer this one.
        self.dev._handle = lambda req: None
        t = threading.Thread(target=do_call)
        t.start()
        time.sleep(0.2)
        self.dev.close()
        t.join(timeout=5.0)
        self.assertIn("err", result)


class HttpApp(unittest.TestCase):
    def setUp(self):
        self.link = ControlLink("127.0.0.1", 0)
        self.server = make_server("127.0.0.1", 0, self.link)
        self.http_port = self.server.server_address[1]
        import threading
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.link.close()

    def test_index_page_served(self):
        status, body = _get(self.http_port, "/")
        self.assertEqual(status, 200)
        self.assertIn(b"<html", body.lower())

    def test_status_json_without_device(self):
        status, body = _get(self.http_port, "/status.json")
        self.assertEqual(status, 200)
        data = json.loads(body)
        self.assertFalse(data["connected"])

    def test_command_without_device_returns_ok_false(self):
        data = _post(self.http_port, "/command", {"cmd": "set_state", "state": "idle"})
        self.assertFalse(data["ok"])
        self.assertIn("not connected", data["err"])

    def test_command_missing_cmd(self):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.http_port}/command",
            data=json.dumps({}).encode(),
            headers={"Content-Type": "application/json"},
        )
        try:
            urllib.request.urlopen(req, timeout=5)
            self.fail("expected HTTPError for missing cmd")
        except urllib.error.HTTPError as exc:
            self.assertEqual(exc.code, 400)

    def test_full_round_trip_through_http(self):
        dev = FakeControlDevice(self.link.port)
        try:
            for _ in range(50):
                if self.link.connected:
                    break
                time.sleep(0.05)
            self.assertTrue(self.link.connected)
            data = _post(self.http_port, "/command", {"cmd": "set_state", "state": "camera_calibration"})
            self.assertTrue(data["ok"])
            self.assertEqual(dev.state, "camera_calibration")
            status, body = _get(self.http_port, "/status.json")
            snap = json.loads(body)
            self.assertTrue(snap["connected"])
            self.assertEqual(snap["device"]["state"], "camera_calibration")
        finally:
            dev.close()


if __name__ == "__main__":
    unittest.main()
