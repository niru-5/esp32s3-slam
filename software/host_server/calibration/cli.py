"""Terminal client for the calibration console: register access, ISP tuning, checkerboard
intrinsics -- the same session a browser's Calibration panel drives.

    cd software && .venv/bin/python -m host_server.calibration run
    cd software && .venv/bin/python -m host_server.calibration run --server http://otherhost:8080

`run` requires `python -m host_server` to already be running (locally or elsewhere --
--server points at it) -- it puts the device into camera_calibration (if it isn't already)
and talks to the SAME calibration console the browser's Calibration panel uses
(host_server/app.py's CalibrationManager, host_server/calibration/console.py), over HTTP
(/command, /calib/line, /calib/output, /calib/answer) instead of owning a device connection
of its own. The App/RegisterBank/IntrinsicFlow/Tuner classes below are unchanged from when
this *was* the thing owning the connection -- only the transport moved server-side.
"""

from __future__ import annotations

import argparse
import json
import shlex
import sys
import threading
import time
from pathlib import Path

import numpy as np

from . import intrinsics as ix
from .flow import IntrinsicFlow, solve_offline
from .link import LinkError
from .liveview import LiveView
from .registers import RegisterBank, RegisterError, UNSAFE_RESTORE, diff_dumps, load_dump, save_dump
from .session import Session, decode, default_root
from ..device_session import DeviceSessionError

# App.run_line()/do_exit() catch this alongside LinkError (calibration/link.py's DeviceLink,
# used offline/by tests) so they work the same whether `link` is a DeviceLink or a
# DeviceSession (host_server/device_session.py, the live one-server transport `run` -- see
# ConsoleClient below -- talks to indirectly, through the server's own calibration console).
LINK_ERRORS = (LinkError, DeviceSessionError)

HELP = """\
device / sensor
  info                          firmware + current camera mode
  mode <fmt> [size] [quality]   fmt: jpeg|raw8|gray|rgb565|yuv422   size: qvga vga svga xga hd sxga uxga fhd qxga 5mp
                                (raw8 only makes sense at hd 1280x720 or vga 640x480: RAW bypasses the ISP scaler)
  fps [n]                       sensor frame cadence from driver timestamps
                                (re-initialises the sensor: all register changes are lost)
  reg <addr> [count]            read register(s), e.g. `reg 0x3820`
  regw <addr> <val> [mask]      write (read-modify-write with mask), recorded in the undo log
  dump [name]                   full register dump -> settings/<name>.json
  diff <fileA> [fileB]          compare backups (fileB defaults to the live sensor)
  restore <file> [--apply]      show/apply the safe differences vs a backup
  undo                          revert every register write made this session
  orient <mirror> <flip>        set mirror/flip (0|1) and capture a JPEG to look at
  freeze [exposure_rows] [gain_x16]   manual AEC/AGC/AWB (unity gains) for repeatable captures
capture
  capture [n] [interval_ms] [tag]     grab n frames -> captures/<tag>/ (with metadata)
intrinsics (checkerboard)
  board <cols> <rows> <square_mm>     inner corners + square size (default 9 6 25)
  cal capture [n] [interval_ms]       burst of n frames, gated, solve on the fly
  cal auto [target]                   keep sharp+novel poses automatically
  cal solve [--save]                  solve now (--save writes json + Kalibr camchain yaml)
  cal status | cal reset
tuning (see docs/camera_calibration_and_tuning.md)
  tune                          list steps
  tune <step> [--yes] [--keep]  run a step: backup -> instructions -> variants -> analysis
  tune notes <step>             show what a step does and what to expect
persistence (see docs/camera_calibration_and_tuning.md "Persisting tuned registers")
  save [--apply]                show/apply: persist this session's register writes (regw/freeze/
                                 orient's undo log) to the device's NVS -- survives a reboot
  save status                   show what's currently saved on the device
  save clear                    erase saved overrides (defaults return after next camera_init())
session
  view                          print the live-view URL
  exit                          leave calibration mode on the device (restores streaming camera config)
  quit                          leave the tool, device stays in calibration mode
"""


def _num(s: str) -> int:
    return int(s, 0)


class App:
    def __init__(self, link, session: Session, view: LiveView, board: ix.Board, out=print, readline=None):
        self.link, self.session, self.view, self.out = link, session, view, out
        self.regs = RegisterBank(link)
        self.flow = IntrinsicFlow(link, session, board, view, out)
        self.mode = {"fmt": link.hello.get("fmt", "jpeg"), "size": link.hello.get("size", "svga")}
        self.startup_dump: dict[int, int] = {}
        self.tuner = None
        # Overridable per-instance (see host_server/calibration/console.py): the web
        # calibration console blocks on a queue instead of real stdin, so a `tune` step
        # run from the browser can wait for a "ready" click instead of a keypress.
        if readline is not None:
            self.readline = readline

    def readline(self, prompt: str = "") -> str:
        try:
            return input(prompt)
        except EOFError:
            return ""

    # -- helpers -----------------------------------------------------------
    def backup(self, name: str) -> Path:
        t0 = time.monotonic()
        regs = self.regs.dump()
        p = self.session.path("settings", f"{name}.json")
        save_dump(p, regs, mode=self.mode, name=name)
        self.session.log("backup", file=str(p), regs=len(regs))
        self.out(f"  backed up {len(regs)} registers -> {p.relative_to(self.session.dir.parent)} "
                 f"({time.monotonic() - t0:.1f}s)")
        return p

    def startup_backup(self) -> None:
        self.out("backing up all sensor registers before touching anything...")
        self.startup_dump = self.regs.dump()
        p = self.session.path("settings", "00_startup.json")
        save_dump(p, self.startup_dump, mode=self.mode, name="startup")
        self.session.log("startup_backup", file=str(p), regs=len(self.startup_dump))
        self.out(f"  {len(self.startup_dump)} registers -> {p}")

    def set_mode(self, fmt: str, size: str | None = None, quality: int | None = None, fb_count: int | None = None):
        args = {"format": fmt, "framesize": size or self.mode["size"]}
        if quality is not None:
            args["quality"] = quality
        if fb_count is not None:
            args["fb_count"] = fb_count
        r = self.link.call("set_mode", timeout=30, **args)
        if not r.ok:
            raise RegisterError(r.data.get("err", "set_mode failed"))
        self.mode = {"fmt": r.data["fmt"], "size": r.data["size"]}
        self.regs.undo_log.clear()      # a re-init resets every register: the log no longer applies
        if self.regs.mirror or self.regs.flip:
            # camera_init_ex() resets orientation too (0x3820/0x3821/0x4514/0x4520 all go back
            # to the sensor default) -- reapply what `orient` last set so it doesn't silently
            # revert on the next tune/cal step's ensure_mode() call. Without this, analysis.py's
            # oriented Bayer-plane helpers would be given the RegisterBank's still-correct
            # mirror/flip while the sensor itself had quietly gone back to unmirrored/unflipped.
            self.regs.set_orientation(self.regs.mirror, self.regs.flip)
        self.session.log("set_mode", **self.mode)
        return r.data

    # -- dispatcher --------------------------------------------------------
    def run_line(self, line: str) -> bool:
        """Returns False when the tool should quit."""
        try:
            argv = shlex.split(line)
        except ValueError as exc:
            self.out(f"parse error: {exc}")
            return True
        if not argv:
            return True
        cmd, args = argv[0], argv[1:]
        try:
            fn = getattr(self, "do_" + cmd, None)
            if fn is None:
                self.out(f"unknown command {cmd!r} -- try `help`")
                return True
            return fn(args) is not False
        except (*LINK_ERRORS, RegisterError, ValueError) as exc:
            self.out(f"error: {exc}")
            if not self.link.connected:
                self.out("device link lost -- waiting for it to reconnect on its own...")
                self.link.wait_for_device(60)
                self.out("device reconnected")
            return True

    # -- commands ----------------------------------------------------------
    def do_help(self, a):
        self.out(HELP)

    def do_info(self, a):
        r = self.link.call("info")
        self.out(json.dumps(r.data, indent=1))

    def do_mode(self, a):
        if not a:
            self.out(f"current: {self.mode}")
            return
        size = a[1] if len(a) > 1 else None
        if a[0] == "raw8" and size is None:
            size = "hd"
        d = self.set_mode(a[0], size, int(a[2]) if len(a) > 2 else None)
        self.out(f"mode -> {self.mode} (sensor PID 0x{d['sensor_pid']:X})")
        if a[0] == "raw8" and self.mode["size"] not in ("hd", "vga"):
            self.out("  note: RAW bypasses the ISP scaler, so only native readouts give a coherent image: use hd or vga")

    def do_reg(self, a):
        addr = _num(a[0])
        n = int(a[1]) if len(a) > 1 else 1
        data = self.regs.read(addr, n)
        for i in range(0, n, 16):
            chunk = data[i:i + 16]
            self.out(f"0x{addr + i:04X}: " + " ".join(f"{b:02X}" for b in chunk))

    def do_regw(self, a):
        addr, val = _num(a[0]), _num(a[1])
        mask = _num(a[2]) if len(a) > 2 else 0xFF
        for res in self.regs.write([(addr, val, mask)]):
            self.out(f"0x{res['a']:04X}: 0x{res['old']:02X} -> 0x{res['new']:02X}")

    def do_dump(self, a):
        self.backup(a[0] if a else time.strftime("dump_%H%M%S"))

    def do_diff(self, a):
        base = load_dump(Path(a[0]))
        other = load_dump(Path(a[1])) if len(a) > 1 else self.regs.dump()
        d = diff_dumps(base, other)
        for addr, x, y in d:
            self.out(f"0x{addr:04X}: 0x{x:02X} -> 0x{y:02X}")
        self.out(f"{len(d)} registers differ")

    def do_restore(self, a):
        snap = load_dump(Path(a[0]))
        apply = "--apply" in a
        diffs, n_vol = self.regs.restore(snap, dry_run=not apply)
        for addr, want, now in diffs:
            self.out(f"0x{addr:04X}: now 0x{now:02X} -> backup 0x{want:02X}")
        self.out(f"{len(diffs)} differing settings {'restored' if apply else '(dry run: add --apply)'}; "
                 f"ignored {n_vol} live-status registers that change on their own (~18 s: two dumps)")

    def do_undo(self, a):
        n = self.regs.revert_to(0)
        self.out(f"reverted {n} registers")

    def do_freeze(self, a):
        exp = int(a[0]) if a else None
        gain = int(a[1]) if len(a) > 1 else None
        self.regs.freeze_loops(exp, gain)
        self.out(f"manual AEC/AGC/AWB (unity). exposure={self.regs.exposure_rows()} rows, "
                 f"gain={self.regs.gain_x16() / 16:.2f}x")

    def do_capture(self, a):
        n = int(a[0]) if a else 1
        interval = int(a[1]) if len(a) > 1 else 0
        tag = a[2] if len(a) > 2 else time.strftime("cap_%H%M%S")
        r = self.link.call("capture", timeout=30 + n * (interval / 1000 + 2), n=n, interval_ms=interval, tag=tag)
        if not r.ok:
            raise RegisterError(r.data.get("err", "capture failed"))
        folder = self.session.path("captures", tag, "x").parent
        for im in r.images:
            p = self.session.save_image(folder, f"frame_{im.meta['seq']:04d}", im)
            self.out(f"  {p.relative_to(self.session.dir)}  {im.meta['w']}x{im.meta['h']} {im.meta['fmt']} "
                     f"{len(im.data)} B  ts_us={im.meta['ts_us']}")
        if r.images:
            try:
                self.view.update_bgr(decode(r.images[-1]), f"last capture: {tag}")
            except ValueError:
                pass
        self.session.log("capture", tag=tag, n=len(r.images))

    def do_board(self, a):
        b = self.flow.board
        b.cols, b.rows, b.square_mm = int(a[0]), int(a[1]), float(a[2])
        self.session.update_meta(board=vars(b))
        self.out(f"board: {b.cols}x{b.rows} inner corners, {b.square_mm} mm squares")

    def do_cal(self, a):
        sub = a[0] if a else "status"
        if sub == "capture":
            self.flow.burst(int(a[1]) if len(a) > 1 else 5, int(a[2]) if len(a) > 2 else 700)
        elif sub == "auto":
            self.flow.auto(int(a[1]) if len(a) > 1 else 15)
        elif sub == "solve":
            self.flow.solve(save="--save" in a)
        elif sub == "reset":
            self.flow.reset()
        elif sub == "status":
            f = self.flow
            self.out(f"{len(f.obs)} accepted / {f.n_rejected} rejected, board {f.board}, size {f.image_size}")
            if f.image_size:
                self.out("  " + ix.coverage_text(f.obs, *f.image_size))
        else:
            self.out("cal capture|auto|solve|status|reset")

    def do_tune(self, a):
        from .tuning import Tuner
        if self.tuner is None:
            self.tuner = Tuner(self)
        self.tuner.command(a)

    def do_orient(self, a):
        """orient <mirror 0|1> <flip 0|1>: set through the driver, capture one JPEG to look at."""
        d = self.regs.set_orientation(bool(int(a[0])), bool(int(a[1])))
        self.out(f"0x3820 {d['old_3820']:#04x}->{d['new_3820']:#04x}   0x3821 {d['old_3821']:#04x}->{d['new_3821']:#04x}")
        if "old_4514" in d and (d["old_4514"] != d["new_4514"] or d.get("old_4520") != d.get("new_4520")):
            self.out(f"0x4514 {d['old_4514']:#04x}->{d['new_4514']:#04x}   "
                     f"0x4520 {d.get('old_4520', 0):#04x}->{d.get('new_4520', 0):#04x}  (BLC readout-direction fixup)")
        r = self.link.call("capture", n=1, flush=5, regs="none", tag="orient")
        if r.images and r.images[0].meta["fmt"] == "jpeg":
            p = self.session.save_image(self.session.path("captures", "orient", "x").parent,
                                        f"orient_m{a[0]}_f{a[1]}", r.images[0])
            self.view.update_bgr(decode(r.images[0]), f"orientation mirror={a[0]} flip={a[1]}")
            self.out(f"saved {p} (also in the live view)")

    def do_fps(self, a):
        """Sensor frame cadence from driver timestamps (no image transfer)."""
        r = self.link.call("fps_probe", n=int(a[0]) if a else 30)
        ts = np.array(r.data["ts"], dtype=np.float64)
        dt = np.diff(ts)
        self.out(f"{len(ts)} frames; dt us: median {np.median(dt):.0f} min {dt.min():.0f} max {dt.max():.0f} "
                 f"-> {1e6 / np.median(dt):.2f} fps;  zero dts: {(dt == 0).sum()}")

    def do_view(self, a):
        self.out(f"live view: http://localhost:{self.view_port}/")

    def do_save(self, a):
        """save [--apply] | save status | save clear -- see docs/camera_calibration_and_tuning.md
        "Persisting tuned registers". Builds the write-list from this session's own undo log
        (every regw/freeze/orient write, deduped to the last value per address) rather than
        diffing against a baseline -- it's already exactly the set of registers this session
        intentionally changed."""
        if a and a[0] == "status":
            r = self.link.call("get_camera_overrides")
            if not r.ok:
                raise RegisterError(r.data.get("err", "get_camera_overrides failed"))
            regs = r.data.get("regs", [])
            if not regs:
                self.out("no register overrides currently saved on the device")
                return
            self.out(f"{len(regs)} register override(s) currently saved:")
            for reg in sorted(regs, key=lambda d: d["a"]):
                self.out(f"  0x{reg['a']:04X} = 0x{reg['v']:02X}")
            return
        if a and a[0] == "clear":
            r = self.link.call("clear_camera_regs")
            if not r.ok:
                raise RegisterError(r.data.get("err", "clear_camera_regs failed"))
            self.out("cleared saved register overrides -- defaults return after the next "
                     "camera_init() (e.g. `exit` or a reboot)")
            self.session.log("clear_camera_regs")
            return

        last: dict[int, int] = {}
        for addr, _old, new in self.regs.undo_log:
            last[addr] = new
        writes = sorted((addr, v) for addr, v in last.items() if addr not in UNSAFE_RESTORE)
        skipped = len(last) - len(writes)
        if not writes:
            self.out("nothing to save -- no (safe) register writes made this session "
                     "(see `regw`/`freeze`/`orient`)" + (f", {skipped} skipped as unsafe" if skipped else ""))
            return
        if "--apply" not in a:
            self.out(f"would save {len(writes)} register(s) to device NVS"
                     + (f" ({skipped} skipped as unsafe)" if skipped else "") + ":")
            for addr, v in writes:
                self.out(f"  0x{addr:04X} = 0x{v:02X}")
            self.out("re-run as `save --apply` to persist (survives reboot)")
            return
        r = self.link.call("save_camera_regs", writes=[{"a": addr, "v": v} for addr, v in writes])
        if not r.ok:
            raise RegisterError(r.data.get("err", "save_camera_regs failed"))
        self.out(f"saved {r.data['saved_this_call']} register(s) this call, "
                 f"{r.data['total_saved']} total persisted on the device")
        self.session.log("save_camera_regs", writes=len(writes))

    def do_exit(self, a):
        try:
            self.link.call("exit", timeout=10)
        except LINK_ERRORS:
            pass
        self.out("device left calibration mode (streaming camera config restored)")
        return False

    def do_quit(self, a):
        return False


class ConsoleClientError(RuntimeError):
    pass


class ConsoleClient:
    """Thin HTTP client for an already-running `python -m host_server`'s calibration
    console endpoints (/command, /calib/line, /calib/output, /calib/answer) -- this is
    cli.py's whole connection to the device now. The App/Session/RegisterBank/etc. all run
    server-side (host_server/app.py's CalibrationManager), shared with the browser page's
    Calibration panel -- `run` is just another client of the same running session, not a
    second one. No serial cable, no separate port: whatever's already entering/driving
    calibration mode (the browser, or this tool) works the same way.
    """

    def __init__(self, base_url: str):
        self.base = base_url.rstrip("/")
        self._since = 0

    def _request(self, method: str, path: str, obj: dict | None = None) -> dict:
        import json as _json
        import urllib.error
        import urllib.request
        data = _json.dumps(obj).encode() if obj is not None else None
        req = urllib.request.Request(self.base + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"} if data else {})
        try:
            with urllib.request.urlopen(req, timeout=15) as resp:
                return _json.loads(resp.read())
        except (OSError, urllib.error.URLError) as exc:
            raise ConsoleClientError(f"{self.base}: {exc}") from exc

    def enter_calibration(self, timeout: float = 15.0) -> None:
        r = self._request("POST", "/command", {"cmd": "set_state", "state": "camera_calibration"})
        if not r.get("ok"):
            raise ConsoleClientError(r.get("err", "set_state failed"))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._request("GET", f"/calib/output.json?since={self._since}").get("active"):
                return
            time.sleep(0.3)
        raise ConsoleClientError("timed out waiting for the calibration console to start "
                                 "(is the device connected to the server?)")

    def _poll_once(self) -> dict:
        j = self._request("GET", f"/calib/output.json?since={self._since}")
        self._since = j.get("next", self._since)
        for line in j.get("lines", []):
            print(line)
        return j

    def submit(self, line: str) -> None:
        j = self._poll_once()
        endpoint = "/calib/answer" if j.get("waiting_for_answer") else "/calib/line"
        self._request("POST", endpoint, {"line": line})

    def wait_idle(self, timeout: float = 300.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self._poll_once().get("busy"):
                return
            time.sleep(0.2)

    def tail(self, stop) -> None:
        while not stop.is_set():
            try:
                self._poll_once()
            except ConsoleClientError:
                pass
            time.sleep(0.3)


def cmd_run(args) -> int:
    client = ConsoleClient(args.server)
    print(f"connecting to {args.server} ...")
    try:
        client.enter_calibration()
    except ConsoleClientError as exc:
        print(f"error: {exc}")
        return 1
    print("entered camera_calibration (or was already there) -- type `help` for commands, "
         "Ctrl-D/Ctrl-C to leave the terminal (device stays in calibration mode)\n")

    cmds = [c.strip() for c in args.commands.split(";")] if args.commands else None
    if cmds is not None:
        for c in cmds:
            print(f"\n> {c}")
            client.submit(c)
            client.wait_idle()
        if args.then_exit:
            client.submit("exit")
            client.wait_idle()
        return 0

    stop = threading.Event()
    tailer = threading.Thread(target=client.tail, args=(stop,), daemon=True)
    tailer.start()
    try:
        while True:
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if line.strip() == "help":
                print(HELP)
                continue
            client.submit(line)
    finally:
        stop.set()
        tailer.join(timeout=2.0)
    return 0


def cmd_solve(args) -> int:
    board = ix.Board(args.cols, args.rows, args.square_mm)
    cal = solve_offline(Path(args.session), board)
    return 0 if cal else 1


def cmd_make_board(args) -> int:
    board = ix.Board(args.cols, args.rows, args.square_mm)
    ix.make_board_png(board, Path(args.output))
    print(f"wrote {args.output}: print at 100% scale, measure a square to confirm {args.square_mm} mm")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="host_server.calibration")
    sub = ap.add_subparsers(dest="sub", required=True)

    def board_args(p):
        p.add_argument("--cols", type=int, default=9, help="inner corners along x")
        p.add_argument("--rows", type=int, default=6, help="inner corners along y")
        p.add_argument("--square-mm", type=float, default=25.0)

    r = sub.add_parser("run", help="terminal client for the calibration console "
                                   "(requires `python -m host_server` already running)")
    r.add_argument("--server", default="http://localhost:8080",
                   help="host_server base URL (default: %(default)s)")
    r.add_argument("-c", "--commands", help="run ';'-separated commands then quit (scripted use)")
    r.add_argument("--then-exit", action="store_true", help="with -c: send `exit` afterwards "
                                                             "(leaves calibration mode)")
    r.set_defaults(fn=cmd_run)
    # Board dimensions for a `run` session are set live via the console's own `board <cols>
    # <rows> <square_mm>` command (default 9 6 25, same as board_args() below) -- there's no
    # separate connection step anymore to attach flags to.

    s = sub.add_parser("solve", help="solve intrinsics offline from a saved session")
    board_args(s)
    s.add_argument("session", help="session folder")
    s.set_defaults(fn=cmd_solve)

    m = sub.add_parser("make-board", help="write a printable checkerboard PNG (A4)")
    board_args(m)
    m.add_argument("output", nargs="?", default="checkerboard.png")
    m.set_defaults(fn=cmd_make_board)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
