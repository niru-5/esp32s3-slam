"""Checkerboard intrinsic calibration: detection, frame gating, solve, export."""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np


@dataclass
class Board:
    cols: int = 9            # inner corners along x
    rows: int = 6            # inner corners along y
    square_mm: float = 25.0

    def object_points(self) -> np.ndarray:
        pts = np.zeros((self.rows * self.cols, 3), np.float32)
        pts[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2) * self.square_mm
        return pts

    @property
    def size(self) -> tuple[int, int]:
        return (self.cols, self.rows)


@dataclass
class Detection:
    ok: bool
    corners: np.ndarray | None = None      # (N,1,2) float32
    reason: str = ""
    sharpness: float = 0.0
    area_frac: float = 0.0
    center: tuple[float, float] = (0.0, 0.0)
    tilt: float = 0.0                       # crude foreshortening indicator, 0 = fronto-parallel


def to_gray(img: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img


def detect(img: np.ndarray, board: Board, min_area_frac: float = 0.04,
           min_sharpness: float = 20.0) -> Detection:
    gray = to_gray(img)
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(gray, board.size, flags)
    if not found:
        return Detection(False, reason="board not found")
    corners = corners.reshape(-1, 1, 2).astype(np.float32)
    h, w = gray.shape
    x, y, bw, bh = cv2.boundingRect(corners)
    area = (bw * bh) / float(w * h)
    x0, y0 = max(x, 0), max(y, 0)
    roi = gray[y0:y0 + bh, x0:x0 + bw]
    sharp = float(cv2.Laplacian(roi, cv2.CV_64F).var()) if roi.size else 0.0

    grid = corners.reshape(board.rows, board.cols, 2)
    # foreshortening: ratio of opposite edge lengths of the grid quadrilateral
    top = np.linalg.norm(grid[0, -1] - grid[0, 0]); bot = np.linalg.norm(grid[-1, -1] - grid[-1, 0])
    lef = np.linalg.norm(grid[-1, 0] - grid[0, 0]); rig = np.linalg.norm(grid[-1, -1] - grid[0, -1])
    tilt = max(abs(top - bot) / max(top, bot, 1e-6), abs(lef - rig) / max(lef, rig, 1e-6))
    det = Detection(True, corners, "", sharp, area, (x + bw / 2, y + bh / 2), float(tilt))
    if area < min_area_frac:
        det.ok, det.reason = False, f"board too small in frame ({area:.1%} < {min_area_frac:.0%}) -- move closer"
    elif sharp < min_sharpness:
        det.ok, det.reason = False, f"blurry (Laplacian var {sharp:.0f} < {min_sharpness:.0f}) -- hold still / refocus"
    return det


def annotate(img: np.ndarray, board: Board, det: Detection, label: str = "") -> np.ndarray:
    out = img.copy() if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if det.corners is not None:
        cv2.drawChessboardCorners(out, board.size, det.corners, det.ok)
    if label:
        cv2.putText(out, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(out, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    return out


@dataclass
class Observation:
    name: str
    corners: np.ndarray
    center: tuple[float, float]
    area_frac: float
    tilt: float


def is_novel(det: Detection, obs: list[Observation], w: int, h: int) -> str:
    """'' if this pose adds information, else the reason it doesn't."""
    for o in obs:
        d = np.linalg.norm(o.corners.reshape(-1, 2) - det.corners.reshape(-1, 2), axis=1).mean()
        if d < 0.03 * max(w, h):
            return f"too similar to {o.name} (mean corner shift {d:.1f}px) -- move/tilt the board"
    return ""


def coverage_grid(obs: list[Observation], w: int, h: int, nx: int = 4, ny: int = 3) -> np.ndarray:
    g = np.zeros((ny, nx), int)
    for o in obs:
        for x, y in o.corners.reshape(-1, 2)[::3]:
            g[min(int(y / h * ny), ny - 1), min(int(x / w * nx), nx - 1)] += 1
    return g


def coverage_text(obs: list[Observation], w: int, h: int) -> str:
    g = coverage_grid(obs, w, h)
    rows = ["".join("#" if v > 0 else "." for v in row) for row in g]
    frac = float((g > 0).mean())
    tilted = sum(1 for o in obs if o.tilt > 0.08)
    return f"coverage {frac:.0%} of image cells [{' / '.join(rows)}], {tilted} tilted views"


@dataclass
class Calibration:
    rms: float
    camera_matrix: list
    dist_coeffs: list
    image_size: tuple[int, int]
    per_image_error: dict[str, float]
    board: dict
    used: list[str]
    model: str = "radtan5"
    created: str = field(default_factory=lambda: time.strftime("%Y-%m-%dT%H:%M:%S"))

    def fov_deg(self) -> tuple[float, float]:
        fx, fy = self.camera_matrix[0][0], self.camera_matrix[1][1]
        w, h = self.image_size
        return (2 * np.degrees(np.arctan(w / (2 * fx))), 2 * np.degrees(np.arctan(h / (2 * fy))))

    def summary(self) -> str:
        K = self.camera_matrix
        fh, fv = self.fov_deg()
        d = ", ".join(f"{x:+.4f}" for x in self.dist_coeffs[0])
        worst = sorted(self.per_image_error.items(), key=lambda kv: -kv[1])[:3]
        return (f"RMS reprojection error {self.rms:.3f} px over {len(self.used)} images\n"
                f"  fx={K[0][0]:.2f} fy={K[1][1]:.2f} cx={K[0][2]:.2f} cy={K[1][2]:.2f}  "
                f"FOV {fh:.1f} x {fv:.1f} deg\n  distortion (k1 k2 p1 p2 k3) = [{d}]\n"
                f"  worst images: " + ", ".join(f"{n} {e:.2f}px" for n, e in worst))


def solve(obs: list[Observation], board: Board, image_size: tuple[int, int],
          prune: bool = True, prune_factor: float = 2.0) -> Calibration:
    """OpenCV calibrateCamera; optionally drops images whose error is > prune_factor x median and re-solves."""
    if len(obs) < 4:
        raise ValueError(f"need at least 4 views, have {len(obs)}")
    objp = board.object_points()
    use = list(obs)
    for _ in range(2):
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            [objp] * len(use), [o.corners for o in use], image_size, None, None)
        errs = {}
        for o, rv, tv in zip(use, rvecs, tvecs):
            proj, _ = cv2.projectPoints(objp, rv, tv, K, dist)
            errs[o.name] = float(np.sqrt(np.mean(np.sum((proj - o.corners) ** 2, axis=2))))
        med = float(np.median(list(errs.values())))
        keep = [o for o in use if errs[o.name] <= max(prune_factor * med, 0.5)]
        if not prune or len(keep) == len(use) or len(keep) < 6:
            break
        use = keep
    return Calibration(float(rms), K.tolist(), dist.tolist(), image_size, errs, asdict(board),
                       [o.name for o in use])


def save_result(cal: Calibration, folder: Path) -> tuple[Path, Path]:
    """Writes intrinsics_<ts>.json and a Kalibr-style camchain yaml."""
    folder.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    jpath = folder / f"intrinsics_{stamp}.json"
    jpath.write_text(json.dumps(asdict(cal), indent=1))
    K, d = cal.camera_matrix, cal.dist_coeffs[0]
    ypath = folder / f"camchain_{stamp}.yaml"
    ypath.write_text(
        "cam0:\n"
        "  camera_model: pinhole\n"
        f"  intrinsics: [{K[0][0]:.6f}, {K[1][1]:.6f}, {K[0][2]:.6f}, {K[1][2]:.6f}]\n"
        "  distortion_model: radtan\n"
        f"  distortion_coeffs: [{d[0]:.8f}, {d[1]:.8f}, {d[2]:.8f}, {d[3]:.8f}]\n"
        f"  resolution: [{cal.image_size[0]}, {cal.image_size[1]}]\n"
        "  # k3 (radial, 3rd term) is not part of Kalibr's radtan model: "
        f"{d[4]:.8f}\n"
        f"  # OpenCV RMS reprojection error: {cal.rms:.4f} px\n")
    return jpath, ypath


def make_board_png(board: Board, path: Path, dpi: int = 200) -> None:
    """Printable board on A4 (squares are exactly square_mm at the given dpi)."""
    px_mm = dpi / 25.4
    sq = int(round(board.square_mm * px_mm))
    nsx, nsy = board.cols + 1, board.rows + 1          # squares = inner corners + 1
    page_w, page_h = int(210 * px_mm), int(297 * px_mm)
    if nsx * sq > page_w - 20 * px_mm:                 # too wide for portrait: use landscape
        page_w, page_h = page_h, page_w
    img = np.full((page_h, page_w), 255, np.uint8)
    ox, oy = (page_w - nsx * sq) // 2, (page_h - nsy * sq) // 2
    if ox < 0 or oy < 0:
        raise ValueError("board does not fit on A4 at this square size")
    for j in range(nsy):
        for i in range(nsx):
            if (i + j) % 2 == 0:
                img[oy + j * sq: oy + (j + 1) * sq, ox + i * sq: ox + (i + 1) * sq] = 0
    cv2.putText(img, f"{board.cols}x{board.rows} inner corners, square {board.square_mm} mm  "
                     "(print at 100% scale, no fit-to-page)", (ox, oy - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, 0, 2)
    cv2.imwrite(str(path), img)
