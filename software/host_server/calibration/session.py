"""On-disk layout for one calibration/tuning session.

    <root>/<YYYYmmdd_HHMMSS>/
        session.json             device hello, board config, notes
        timeline.jsonl           one line per action (mode change, capture, backup, step ...)
        settings/                full register dumps (startup backup, before every tuning step)
        captures/<tag>/          ad-hoc `capture` output
        intrinsics/images/       checkerboard frames (img_0001.jpg + img_0001.json)
        intrinsics/rejected/     frames that failed detection/quality gates (kept for review)
        intrinsics/results/      intrinsics_<ts>.json, kalibr camchain yaml, report
        tuning/<step>/<NN_variant>/frame_000.jpg|pgm + .json, regs_full.json, analysis.json
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from .link import Image

_EXT = {"jpeg": ".jpg", "raw8": ".pgm", "gray": ".pgm"}


def default_root() -> Path:
    return Path(__file__).resolve().parents[2] / "calib_data"


def image_ext(fmt: str) -> str:
    return _EXT.get(fmt, ".raw")


def decode(img: Image) -> np.ndarray:
    """Frame -> numpy. jpeg -> BGR uint8; raw8/gray -> HxW uint8 (raw8 is the Bayer mosaic)."""
    import cv2

    w, h, fmt = img.meta["w"], img.meta["h"], img.meta["fmt"]
    if fmt == "jpeg":
        arr = cv2.imdecode(np.frombuffer(img.data, np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            raise ValueError("corrupt JPEG frame")
        return arr
    if fmt in ("raw8", "gray"):
        return np.frombuffer(img.data, np.uint8).reshape(h, w)
    if fmt == "rgb565":
        v = np.frombuffer(img.data, ">u2").reshape(h, w)   # big-endian pairs from the DVP path
        r = ((v >> 11) & 0x1F) << 3
        g = ((v >> 5) & 0x3F) << 2
        b = (v & 0x1F) << 3
        return np.dstack([b, g, r]).astype(np.uint8)
    if fmt == "yuv422":
        return cv2.cvtColor(np.frombuffer(img.data, np.uint8).reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUY2)
    raise ValueError(f"cannot decode format {fmt!r}")


def load_saved_image(path: Path) -> np.ndarray:
    import cv2
    if path.suffix == ".pgm":
        arr = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    else:
        arr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if arr is None:
        raise ValueError(f"cannot read {path}")
    return arr


def write_pgm(path: Path, data: bytes, w: int, h: int) -> None:
    path.write_bytes(f"P5\n{w} {h}\n255\n".encode() + data)


class Session:
    def __init__(self, root: Path | None = None, name: str | None = None):
        root = root or default_root()
        self.name = name or time.strftime("%Y%m%d_%H%M%S")
        self.dir = root / self.name
        self.dir.mkdir(parents=True, exist_ok=True)
        self.meta: dict = {"created": time.strftime("%Y-%m-%dT%H:%M:%S")}
        self._save_meta()

    # -- paths -------------------------------------------------------------
    def path(self, *parts: str) -> Path:
        p = self.dir.joinpath(*parts)
        p.parent.mkdir(parents=True, exist_ok=True)
        return p

    # -- metadata / timeline ------------------------------------------------
    def _save_meta(self) -> None:
        (self.dir / "session.json").write_text(json.dumps(self.meta, indent=1))

    def update_meta(self, **kv) -> None:
        self.meta.update(kv)
        self._save_meta()

    def log(self, event: str, **kv) -> None:
        with open(self.dir / "timeline.jsonl", "a") as f:
            f.write(json.dumps({"t": time.strftime("%H:%M:%S"), "event": event, **kv}) + "\n")

    # -- images ------------------------------------------------------------
    def save_image(self, folder: Path, stem: str, img: Image, extra: dict | None = None) -> Path:
        """Write frame bytes + sidecar JSON (device metadata + ``extra``). Returns the frame path."""
        folder.mkdir(parents=True, exist_ok=True)
        fmt = img.meta["fmt"]
        p = folder / (stem + image_ext(fmt))
        if p.suffix == ".pgm":
            write_pgm(p, img.data, img.meta["w"], img.meta["h"])
        else:
            p.write_bytes(img.data)
        meta = dict(img.meta)
        if extra:
            meta.update(extra)
        p.with_suffix(".json").write_text(json.dumps(meta, indent=1))
        return p
