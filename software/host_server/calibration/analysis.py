"""Image statistics used by the tuning steps (numpy/OpenCV only)."""

from __future__ import annotations

import cv2
import numpy as np

# OV5640 RAW default readout ("Normal" row of the playbook's Bayer-phase table):
# first 2x2 tile seen by the ISP is  B G / G R.
BAYER_NAMES = {(0, 0): "B", (0, 1): "Gb", (1, 0): "Gr", (1, 1): "R"}


def debayer(raw: np.ndarray) -> np.ndarray:
    """Preview demosaic for the OV5640's normal readout (B G / G R tile).

    OpenCV names Bayer codes by the *second* row/column, so this tile is COLOR_BayerRG2BGR
    -- verified on this module: BayerBG2BGR swaps red and blue on a warm-lit scene."""
    return cv2.cvtColor(raw, cv2.COLOR_BayerRG2BGR)


def raw_geometry_ok(raw: np.ndarray, min_corr: float = 0.5) -> bool:
    """True if consecutive same-colour rows correlate, i.e. the raw stream really is `w` pixels per line.

    In RAW mode the ISP scaler is bypassed, so a frame size that needs scaling (svga, xga, sxga on
    this driver) gets the sensor's native 1280-wide lines chopped into the wrong width and the rows
    come out scrambled. Native readouts that work here: hd (1280x720) and vga (640x480)."""
    f = raw.astype(np.float64)
    if f.std() < 8.0:
        return True                                   # dark/flat frame (noise only): cannot tell, don't cry wolf
    a, b = f[:-2:2, :].ravel(), f[2::2, :].ravel()
    return float(np.corrcoef(a, b)[0, 1]) > min_corr


def bayer_planes(raw: np.ndarray, names=BAYER_NAMES) -> dict[str, np.ndarray]:
    return {n: raw[i::2, j::2].astype(np.float64) for (i, j), n in names.items()}


def plane_stats(raw: np.ndarray) -> dict[str, dict]:
    out = {}
    for n, p in bayer_planes(raw).items():
        out[n] = {"mean": float(p.mean()), "std": float(p.std()),
                  "zero_frac": float((p == 0).mean()), "sat_frac": float((p >= 255).mean())}
    return out


def average_frames(frames: list[np.ndarray]) -> np.ndarray:
    return np.mean(np.stack([f.astype(np.float64) for f in frames]), axis=0)


def phase_guess(raw: np.ndarray) -> dict:
    """Which 2x2 positions carry the green samples, from channel means.

    Greens are the two positions with the highest mean in a typical scene; if they sit on
    the anti-diagonal ((0,1),(1,0)) the tile is B/R-G-G-R/B ("BGGR" or "RGGB"); on the main
    diagonal it's "GRBG"/"GBRG". Red vs blue is inferred from which of the remaining two is
    brighter, which only holds for a warm illuminant -- treat as a hint.
    """
    m = {(i, j): float(raw[i::2, j::2].mean()) for i in (0, 1) for j in (0, 1)}
    greens = sorted(m, key=lambda k: -m[k])[:2]
    others = [k for k in m if k not in greens]
    anti = set(greens) == {(0, 1), (1, 0)}
    hi, lo = sorted(others, key=lambda k: -m[k])
    layout = {(0, 0): "", (0, 1): "", (1, 0): "", (1, 1): ""}
    for g in greens:
        layout[g] = "G"
    layout[hi], layout[lo] = "R", "B"          # assumes R > B (warm light)
    pattern = layout[(0, 0)] + layout[(0, 1)] + layout[(1, 0)] + layout[(1, 1)]
    return {"means": {f"{k[0]}{k[1]}": round(v, 2) for k, v in m.items()},
            "greens_on_antidiagonal": anti, "pattern_if_warm_light": pattern}


def luma(img: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img


def center_patch(img: np.ndarray, frac: float = 0.2) -> np.ndarray:
    h, w = img.shape[:2]
    dh, dw = int(h * frac / 2), int(w * frac / 2)
    return img[h // 2 - dh: h // 2 + dh, w // 2 - dw: w // 2 + dw]


def rgb_means(img_bgr: np.ndarray, frac: float = 0.2) -> dict:
    p = center_patch(img_bgr, frac).reshape(-1, 3).mean(axis=0)
    b, g, r = (float(x) for x in p)
    return {"R": r, "G": g, "B": b, "R/G": r / max(g, 1e-6), "B/G": b / max(g, 1e-6)}


def sharpness(img: np.ndarray) -> float:
    return float(cv2.Laplacian(luma(img), cv2.CV_64F).var())


def temporal_noise(a: np.ndarray, b: np.ndarray, frac: float = 0.3) -> float:
    """Temporal noise sigma (DN) from two frames of a static scene: std(a-b)/sqrt(2) over the centre."""
    d = center_patch(luma(a).astype(np.float64), frac) - center_patch(luma(b).astype(np.float64), frac)
    return float(d.std() / np.sqrt(2))


def grid_means(plane: np.ndarray, n: int) -> np.ndarray:
    h, w = plane.shape
    return np.array([[plane[int(i * h / n): int((i + 1) * h / n), int(j * w / n): int((j + 1) * w / n)].mean()
                      for j in range(n)] for i in range(n)])


def hot_pixels(mean_frame: np.ndarray, thresh: float = 8.0) -> np.ndarray:
    """Boolean map of pixels exceeding the 3x3-same-colour median (5x5 stride-2 neighbourhood) by thresh DN."""
    f = mean_frame.astype(np.float32)
    med = np.zeros_like(f)
    for i in (0, 1):
        for j in (0, 1):
            plane = f[i::2, j::2]
            med[i::2, j::2] = cv2.medianBlur(plane, 3)
    return (f - med) > thresh


def row_banding(gray: np.ndarray) -> float:
    """Relative amplitude of horizontal banding: std of the row-mean profile after removing a smooth trend."""
    rows = gray.astype(np.float64).mean(axis=1)
    x = np.linspace(-1, 1, rows.size)
    trend = np.polyval(np.polyfit(x, rows, 3), x)
    return float((rows - trend).std() / max(rows.mean(), 1e-6))


def contact_sheet(tiles: list[tuple[str, np.ndarray]], cols: int = 2, tile_w: int = 480) -> np.ndarray:
    out = []
    for label, im in tiles:
        if im.ndim == 2:
            im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
        s = tile_w / im.shape[1]
        im = cv2.resize(im, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        cv2.putText(im, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(im, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 1)
        out.append(im)
    h = max(i.shape[0] for i in out)
    out = [cv2.copyMakeBorder(i, 0, h - i.shape[0], 0, 0, cv2.BORDER_CONSTANT) for i in out]
    while len(out) % cols:
        out.append(np.zeros_like(out[0]))
    rows = [np.hstack(out[i:i + cols]) for i in range(0, len(out), cols)]
    return np.vstack(rows)
