"""Interactive camera-intrinsics flow: burst / auto capture, gating, on-the-fly solve."""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from . import intrinsics as ix
from .link import DeviceLink, Image, LinkError
from .liveview import LiveView
from .session import Session, decode, load_saved_image

MIN_VIEWS_FOR_SOLVE = 8


class IntrinsicFlow:
    def __init__(self, link: DeviceLink | None, session: Session, board: ix.Board, view: LiveView, out=print):
        self.link, self.session, self.board, self.view, self.out = link, session, board, view, out
        self.obs: list[ix.Observation] = []
        self.image_size: tuple[int, int] | None = None
        self.n_rejected = 0
        self.last_cal: ix.Calibration | None = None
        self._img_dir = session.path("intrinsics", "images")
        self._rej_dir = session.path("intrinsics", "rejected")
        self._res_dir = session.path("intrinsics", "results")
        session.update_meta(board=vars(board))

    # -- processing one frame ---------------------------------------------
    def process(self, img: Image, save: bool = True) -> tuple[bool, str]:
        arr = decode(img)
        h, w = arr.shape[:2]
        if self.image_size is None:
            self.image_size = (w, h)
        elif self.image_size != (w, h):
            return False, f"image size {w}x{h} differs from session {self.image_size}; intrinsics are per-resolution"
        det = ix.detect(arr, self.board)
        reason = det.reason
        if det.ok:
            reason = ix.is_novel(det, self.obs, w, h)
        accepted = det.ok and not reason
        seq = img.meta.get("seq", 0)
        if accepted:
            name = f"img_{len(self.obs) + 1:04d}"
            if save:
                self.session.save_image(self._img_dir, name, img, {
                    "detection": {"sharpness": det.sharpness, "area_frac": det.area_frac, "tilt": det.tilt}})
            self.obs.append(ix.Observation(name, det.corners, det.center, det.area_frac, det.tilt))
            msg = f"ACCEPT {name} (seq {seq}, sharp {det.sharpness:.0f}, area {det.area_frac:.0%}, tilt {det.tilt:.2f})"
        else:
            self.n_rejected += 1
            if save:
                self.session.save_image(self._rej_dir, f"rej_{self.n_rejected:04d}", img, {"reject_reason": reason})
            msg = f"reject (seq {seq}): {reason}"
        status = f"{len(self.obs)} accepted / {self.n_rejected} rejected\n{msg}\n"
        status += ix.coverage_text(self.obs, w, h) + "\n"
        if self.last_cal:
            status += "last solve: " + self.last_cal.summary().splitlines()[0]
        self.view.update_bgr(ix.annotate(arr, self.board, det, "ACCEPT" if accepted else "reject"), status)
        return accepted, msg

    # -- capture modes -----------------------------------------------------
    def burst(self, n: int, interval_ms: int) -> None:
        """Device grabs n frames back-to-back-ish and streams them all with metadata."""
        r = self.link.call("capture", timeout=30 + n * (interval_ms / 1000 + 1),
                           n=n, interval_ms=interval_ms, flush=2, regs="key", tag="intrinsics")
        if not r.ok:
            self.out(f"capture failed: {r.data.get('err')}")
            return
        acc = 0
        for im in r.images:
            ok, msg = self.process(im)
            acc += ok
            self.out("  " + msg)
        self.out(f"burst done: {acc}/{len(r.images)} accepted, total {len(self.obs)}")
        self._maybe_solve()

    def auto(self, target: int, timeout_s: float = 300.0, period_s: float = 0.6) -> None:
        """Poll the camera; whenever a sharp, novel board pose is in view, keep it."""
        self.out(f"auto-capture until {target} views (Ctrl-C to stop): move/tilt the board slowly, "
                 f"hold still ~1 s at each pose. Live view shows what is seen.")
        t_end = time.monotonic() + timeout_s
        last_msg = ""
        try:
            while len(self.obs) < target and time.monotonic() < t_end:
                r = self.link.call("capture", n=1, flush=0, regs="none", tag="auto")
                if not r.ok or not r.images:
                    self.out(f"capture failed: {r.data.get('err')}")
                    return
                ok, msg = self.process(r.images[0])
                if ok:
                    self.out("  " + msg + f"   [{len(self.obs)}/{target}]")
                    self._maybe_solve()
                elif msg != last_msg and "not found" not in msg:
                    self.out("  " + msg)
                last_msg = msg
                time.sleep(period_s)
        except KeyboardInterrupt:
            self.out("stopped")
        self.out(f"auto done: {len(self.obs)} views")

    # -- solving -------------------------------------------------------------
    def _maybe_solve(self) -> None:
        if len(self.obs) >= MIN_VIEWS_FOR_SOLVE:
            self.solve(quiet=True)
        else:
            self.out(f"  ({len(self.obs)}/{MIN_VIEWS_FOR_SOLVE} views before an on-the-fly solve; "
                     f"{ix.coverage_text(self.obs, *self.image_size)})")

    def solve(self, quiet: bool = False, save: bool = False) -> ix.Calibration | None:
        if len(self.obs) < 4 or self.image_size is None:
            self.out(f"need >= 4 accepted views, have {len(self.obs)}")
            return None
        cal = ix.solve(self.obs, self.board, self.image_size)
        self.last_cal = cal
        self.out(("  on-the-fly solve: " if quiet else "") + cal.summary())
        w, h = self.image_size
        self.out("  " + ix.coverage_text(self.obs, w, h))
        hints = []
        if cal.rms > 1.0:
            hints.append("RMS > 1 px: check board flatness/print scale, refocus, or drop blurry views")
        if len(self.obs) < 15:
            hints.append("collect 15-25 views for a stable result")
        for hnt in hints:
            self.out("  hint: " + hnt)
        if save:
            j, y = ix.save_result(cal, self._res_dir)
            self.out(f"saved {j}\n      {y}")
            self.session.log("intrinsics_saved", rms=cal.rms, views=len(cal.used), json=str(j))
        return cal

    def reset(self) -> None:
        self.obs.clear()
        self.n_rejected = 0
        self.last_cal = None
        self.out("cleared in-memory views (files on disk are kept)")


def solve_offline(session_dir: Path, board: ix.Board, out=print, save: bool = True) -> ix.Calibration | None:
    """Re-detect + solve from the saved intrinsics/images of an earlier session."""
    files = sorted((session_dir / "intrinsics" / "images").glob("img_*.*"))
    files = [f for f in files if f.suffix in (".jpg", ".pgm")]
    if not files:
        out(f"no images under {session_dir}/intrinsics/images")
        return None
    obs, size = [], None
    for f in files:
        arr = load_saved_image(f)
        det = ix.detect(arr, board, min_area_frac=0.0, min_sharpness=0.0)
        if not det.ok:
            out(f"  skip {f.name}: {det.reason}")
            continue
        size = (arr.shape[1], arr.shape[0])
        obs.append(ix.Observation(f.stem, det.corners, det.center, det.area_frac, det.tilt))
    out(f"{len(obs)}/{len(files)} images usable, image size {size}")
    if len(obs) < 4:
        return None
    cal = ix.solve(obs, board, size)
    out(cal.summary())
    out("  " + ix.coverage_text(obs, *size))
    if save:
        j, y = ix.save_result(cal, session_dir / "intrinsics" / "results")
        out(f"saved {j}\n      {y}")
    return cal
