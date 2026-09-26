"""Interactive host side of the camera calibration / ISP tuning mode.

    cd software && .venv/bin/python -m host_server.calibration run --serial /dev/ttyACM0
    cd software && .venv/bin/python -m host_server.calibration run      # device already in mode 5

The tool listens on the control port (8084), (optionally) tells the ESP32-S3 to enter
CAMERA_CALIBRATION over serial, waits for the device to dial in, opens a session folder
and backs up every sensor register before anything is touched.
"""

from __future__ import annotations

import argparse
import json
import re
import shlex
import sys
import time
from pathlib import Path

import numpy as np

from . import intrinsics as ix
from .flow import IntrinsicFlow, solve_offline
from .link import DeviceLink, LinkError
from .liveview import LiveView
from .registers import RegisterBank, RegisterError, diff_dumps, load_dump, save_dump
from .session import Session, decode, default_root

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
session
  view                          print the live-view URL
  exit                          leave calibration mode on the device (restores streaming camera config)
  quit                          leave the tool, device stays in calibration mode
"""


def _num(s: str) -> int:
    return int(s, 0)


class App:
    def __init__(self, link: DeviceLink, session: Session, view: LiveView, board: ix.Board, out=print):
        self.link, self.session, self.view, self.out = link, session, view, out
        self.regs = RegisterBank(link)
        self.flow = IntrinsicFlow(link, session, board, view, out)
        self.mode = {"fmt": link.hello.get("fmt", "jpeg"), "size": link.hello.get("size", "svga")}
        self.startup_dump: dict[int, int] = {}
        self.tuner = None

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
        except (LinkError, RegisterError, ValueError) as exc:
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

    def do_exit(self, a):
        try:
            self.link.call("exit", timeout=10)
        except LinkError:
            pass
        self.out("device left calibration mode (streaming camera config restored)")
        return False

    def do_quit(self, a):
        return False


def _enter_mode_over_serial(port: str, out) -> tuple:
    """Open the console, wait for the firmware's "ready" line, send '5'.

    Opening the port resets the ESP32-S3 (USB-Serial-JTAG), and so does closing
    it again on this setup, so the handle is returned and kept open for the
    whole session -- a command sent before boot finishes would be lost.
    """
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.dtr, s.rts, s.timeout = port, 115200, False, False, 0.2
    s.open()
    out(f"waiting for firmware boot on {port} ...")
    buf, t0 = b"", time.monotonic()
    while b"Returned from app_main" not in buf:
        buf += s.read(4096)
        if time.monotonic() - t0 > 40:
            out("no 'ready' line seen (already running?) -- sending '5' anyway")
            break
    m = re.search(rb"IP: (\d+\.\d+\.\d+\.\d+)", buf)
    time.sleep(0.5)
    s.write(b"5\n")
    s.flush()
    out(f"sent '5' (CAMERA_CALIBRATION) on {port}")
    return s, (m.group(1).decode() if m else None)


def _tee_serial(ser, path: Path) -> None:
    """Keep draining the console into <session>/device.log (firmware ESP_LOG output)."""
    import threading

    def pump():
        with open(path, "ab") as f:
            while ser.is_open:
                try:
                    d = ser.read(4096)
                except Exception:
                    return
                if d:
                    f.write(d)
                    f.flush()
    threading.Thread(target=pump, daemon=True).start()


def cmd_run(args) -> int:
    board = ix.Board(args.cols, args.rows, args.square_mm)
    link = DeviceLink("0.0.0.0", args.port)          # listen before the device is told to connect
    view = LiveView(args.view_port)
    print(f"listening for the device on :{args.port}   live view http://localhost:{args.view_port}/")
    ser = None
    if args.serial:
        ser, _ = _enter_mode_over_serial(args.serial, print)
    try:
        hello = link.wait_for_device(args.wait, on_wait=lambda: print(
            "  ... waiting for the device to connect (put it in camera calibration mode: serial command 5)"))
    except LinkError as exc:
        print(exc)
        if ser is not None:
            ser.close()
        return 1
    session = Session(Path(args.out) if args.out else default_root(), args.name)
    session.update_meta(device_hello=hello, peer=str(link.peer))
    print(f"device connected from {link.peer[0]}: {hello}")
    print(f"session folder: {session.dir}")
    if ser is not None:
        _tee_serial(ser, session.dir / "device.log")
    app = App(link, session, view, board)
    app.view_port = args.view_port
    try:
        app.startup_backup()
        cmds = [c.strip() for c in args.commands.split(";")] if args.commands else None
        if cmds is not None:
            for c in cmds:
                print(f"\n> {c}")
                if not app.run_line(c):
                    break
            if args.then_exit:
                app.run_line("exit")
        else:
            print("\ntype `help` for commands\n")
            while True:
                try:
                    line = input("calib> ")
                except (EOFError, KeyboardInterrupt):
                    print()
                    break
                if line.strip() == "help":
                    app.do_help([])
                    continue
                if not app.run_line(line):
                    break
    finally:
        link.close()
        view.close()
        if ser is not None:
            ser.close()
    print(f"session saved in {session.dir}")
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

    r = sub.add_parser("run", help="interactive calibration/tuning session")
    board_args(r)
    r.add_argument("--port", type=int, default=8084, help="control port to listen on (CONFIG_CAM_CALIB_PORT)")
    r.add_argument("--view-port", type=int, default=8090)
    r.add_argument("--serial", help="serial port: send '5' to put the device in calibration mode")
    r.add_argument("--wait", type=float, default=90.0, help="seconds to wait for the device to start listening")
    r.add_argument("--out", help=f"session root (default {default_root()})")
    r.add_argument("--name", help="session folder name (default: timestamp)")
    r.add_argument("-c", "--commands", help="run ';'-separated commands then quit (scripted use)")
    r.add_argument("--then-exit", action="store_true", help="with -c: send `exit` to the device afterwards")
    r.set_defaults(fn=cmd_run)

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
