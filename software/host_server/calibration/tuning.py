"""Guided OV5640 ISP tuning: ten steps that follow docs/OV5640_tuning_playbook_bench_procedures.md.

Every step:
  1. backs up ALL sensor registers first  (settings/<step>_before.json -- the "old settings")
  2. tells the operator how to set up the scene and waits for Enter
  3. runs variants: apply register changes, flush, capture, save frames + metadata (+ full
     register dump for raw steps) under tuning/<step>/<NN_variant>/, compute metrics
  4. reverts every register it touched (unless --keep) and prints what to expect
  5. writes tuning/<step>/report.md + analysis.json

Register meanings follow the playbook; when a variant shows *no* change, the register may be
mode-dependent or FAE-locked on this module -- the saved before/after register dumps let you check.
"""

from __future__ import annotations

import contextlib
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from . import analysis as an
from .link import Image
from .registers import RegisterBank, diff_dumps, load_dump, save_dump
from .session import decode


# Frame size for RAW8 steps: a native readout (no ISP scaling) -- see analysis.raw_geometry_ok.
RAW_SIZE = "hd"


@dataclass
class Step:
    key: str
    title: str
    setup: str          # what the operator must do physically before pressing Enter
    expect: str         # what a good result looks like
    fn: Callable


class Ctx:
    """Per-step helper handed to each step function."""

    def __init__(self, tuner: "Tuner", step: Step, opts: dict, keep: bool):
        self.tuner, self.app, self.step, self.opts, self.keep = tuner, tuner.app, step, opts, keep
        self.regs: RegisterBank = tuner.app.regs
        self.dir = tuner.app.session.path("tuning", step.key, "x").parent
        self.variants: list[dict] = []
        self.cur: dict | None = None
        self._orient0: tuple[bool, bool] | None = None
        self.say = tuner.app.out

    # -- options -----------------------------------------------------------
    def opt(self, key: str, default):
        v = self.opts.get(key, default)
        return type(default)(v) if default is not None and not isinstance(default, str) else v

    # -- camera ------------------------------------------------------------
    def ensure_mode(self, fmt: str, size: str | None = None, quality: int | None = None) -> None:
        if fmt == "raw8" and size is None:
            size = RAW_SIZE
        m = self.app.mode
        if fmt != m["fmt"] or (size and size != m["size"]) or quality is not None:
            self.say(f"  switching camera mode -> {fmt} {size or m['size']} (all registers reset to driver defaults)")
            self.app.set_mode(fmt, size, quality)

    def orientation(self) -> tuple[bool, bool]:
        r20, r21 = self.regs.read(0x3820, 2)
        return bool(r21 & 0x06), bool(r20 & 0x06)          # (mirror, flip)

    def set_orientation(self, mirror: bool, flip: bool) -> None:
        if self._orient0 is None:
            self._orient0 = self.orientation()
        self.regs.set_orientation(mirror, flip)

    def freeze_current(self, awb: tuple[int, int, int] | None = (0x400, 0x400, 0x400)) -> tuple[int, float]:
        """Let AEC settle, then lock exposure+gain at whatever it chose."""
        self.capture(n=1, flush=6, regs="none", save=False)       # let the loops run a few frames
        e, g = self.regs.exposure_rows(), self.regs.gain_x16()
        self.regs.freeze_loops(e, g, awb)
        return e, g / 16.0

    @contextlib.contextmanager
    def variant(self, name: str, desc: str = ""):
        idx = len(self.variants) + 1
        v = {"name": name, "desc": desc, "dir": f"{idx:02d}_{name}", "metrics": {}, "files": []}
        self.variants.append(v)
        self.cur = v
        mark = self.regs.mark()
        self.say(f"\n  [{idx}] {name}" + (f" -- {desc}" if desc else ""))
        try:
            yield v
        finally:
            v["reg_changes"] = [{"addr": f"0x{a:04X}", "old": o, "new": n} for a, o, n in self.regs.undo_log[mark:]]
            if not self.keep:
                self.regs.revert_to(mark)
                if self._orient0 is not None:
                    self.regs.set_orientation(*self._orient0)
                    self._orient0 = None
            (self.dir / v["dir"]).mkdir(parents=True, exist_ok=True)
            (self.dir / v["dir"] / "variant.json").write_text(json.dumps(v, indent=1, default=str))
            self.cur = None

    def capture(self, n: int = 1, interval_ms: int = 0, flush: int = 3, regs: str = "key",
                full_regs: bool = False, save: bool = True) -> list[Image]:
        r = self.app.link.call("capture", timeout=30 + n * (interval_ms / 1000 + 3), n=n,
                               interval_ms=interval_ms, flush=flush, regs=regs, tag=self.step.key)
        if not r.ok:
            raise RuntimeError(r.data.get("err", "capture failed"))
        if r.images and r.images[0].fmt == "raw8" and not an.raw_geometry_ok(decode(r.images[0])):
            self.say(f"      WARNING: raw rows look scrambled at {self.app.mode['size']} -- this size needs ISP scaling, "
                     f"which RAW bypasses. Use hd or vga.")
            if self.cur is not None:
                self.cur["raw_geometry_warning"] = True
        if save and self.cur is not None:
            folder = self.dir / self.cur["dir"]
            for im in r.images:
                p = self.app.session.save_image(folder, f"frame_{im.meta['seq']:04d}", im)
                self.cur["files"].append(p.name)
            if full_regs:
                save_dump(folder / "regs_full.json", self.regs.dump(), step=self.step.key,
                          variant=self.cur["name"], mode=self.app.mode)
                self.cur["files"].append("regs_full.json")
        return r.images

    def frames(self, images: list[Image]) -> list[np.ndarray]:
        return [decode(i) for i in images]

    def show(self, img: np.ndarray, text: str) -> None:
        self.app.view.update_bgr(img, f"{self.step.key}: {text}")

    def metrics(self, **kv) -> None:
        assert self.cur is not None
        self.cur["metrics"].update(kv)
        self.say("      " + "  ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in kv.items()))

    def dark_check(self, mean_dn: float, limit: float = 30.0) -> None:
        """Flag a 'dark' frame that isn't dark (lens cap off / light leak): the numbers would be meaningless."""
        if mean_dn > limit:
            self.say(f"      WARNING: frame mean {mean_dn:.0f} DN is far from black -- is the lens cap on / the box light-tight? "
                     f"Results for this variant are scene content, not sensor black level.")
            if self.cur is not None:
                self.cur["not_dark_warning"] = True

    def note(self, msg: str) -> None:
        self.say("      " + msg)


# =====================================================================
# Steps
# =====================================================================

def step_orientation(c: Ctx) -> None:
    sheet: list[tuple[str, np.ndarray]] = []
    for fmt in ("jpeg", "raw8"):
        c.ensure_mode(fmt)
        for name, m, f in (("normal", 0, 0), ("mirror", 1, 0), ("flip", 0, 1), ("mirror_flip", 1, 1)):
            with c.variant(f"{fmt}_{name}", f"mirror={m} flip={f}"):
                c.set_orientation(bool(m), bool(f))
                imgs = c.capture(2, flush=5)
                r20, r21 = c.regs.read(0x3820, 2)
                c.metrics(reg3820=f"0x{r20:02X}", reg3821=f"0x{r21:02X}",
                          mirror_bits=bool(r21 & 6), flip_bits=bool(r20 & 6))
                arr = decode(imgs[-1])
                if fmt == "jpeg":
                    sheet.append((name, arr))
                else:
                    g = an.phase_guess(arr)
                    c.metrics(greens_on_antidiagonal=g["greens_on_antidiagonal"],
                              bayer_if_warm_light=g["pattern_if_warm_light"], means=str(g["means"]))
    if sheet:
        img = an.contact_sheet(sheet)
        import cv2
        cv2.imwrite(str(c.dir / "contact_sheet.jpg"), img)
        c.show(img, "orientation contact sheet (normal / mirror / flip / mirror+flip)")
        c.say(f"\n  contact sheet -> {c.dir / 'contact_sheet.jpg'}  (also in the live view)")


def step_timing(c: Ctx) -> None:
    with c.variant("measure", "HTS/VTS registers vs measured frame cadence"):
        b = c.regs.read(0x3808, 8)
        w, h = (b[0] << 8) | b[1], (b[2] << 8) | b[3]
        hts, vts = (b[4] << 8) | b[5], (b[6] << 8) | b[7]
        r = c.app.link.call("fps_probe", timeout=30, n=40)
        ts = np.array(r.data["ts"], dtype=np.float64)
        dt = np.diff(ts)
        dt = dt[dt > 0]
        period_us = float(np.median(dt))
        fps = 1e6 / period_us
        t_row_us = period_us / vts
        pclk = hts * vts * fps
        b50 = 10000.0 / t_row_us
        b60 = 8333.3 / t_row_us
        rb = c.regs.read(0x3A08, 4)
        b50_reg, b60_reg = ((rb[0] & 3) << 8) | rb[1], ((rb[2] & 3) << 8) | rb[3]
        c.metrics(output=f"{w}x{h}", HTS=hts, VTS=vts, fps=fps, frame_period_ms=period_us / 1000,
                  jitter_us=float(dt.std()), t_row_us=t_row_us, pclk_mhz=pclk / 1e6)
        c.metrics(B50_computed=b50, B50_reg=b50_reg, B60_computed=b60, B60_reg=b60_reg,
                  B50_ok=abs(b50 - b50_reg) / b50 < 0.03, B60_ok=abs(b60 - b60_reg) / b60 < 0.03)
        exp = c.regs.exposure_rows()
        c.metrics(exposure_rows_now=exp, max_exposure_ms=(vts - 4) * t_row_us / 1000)
        (c.dir / "timing.json").write_text(json.dumps(
            {"hts": hts, "vts": vts, "fps": fps, "t_row_us": t_row_us, "pclk_hz": pclk,
             "b50": b50, "b60": b60, "mode": c.app.mode}, indent=1))


def _dark_grid(c: Ctx) -> dict:
    vts = c.regs.read16(0x380E)
    exps = sorted({8, 100, max(vts - 8, 20)})
    gains = [0x10, 0x40, 0xF0]                      # 1x, 4x, 15x
    res: dict = {}
    for e in exps:
        for g in gains:
            with c.variant(f"E{e}_G{g / 16:g}x", f"exposure {e} rows, gain {g / 16:g}x"):
                c.regs.freeze_loops(e, g, awb_gains=None)
                imgs = c.capture(6, flush=4, full_regs=(e == exps[1] and g == gains[0]))
                raw = an.average_frames(c.frames(imgs))
                st = an.plane_stats(raw)
                means = {k: v["mean"] for k, v in st.items()}
                c.dark_check(float(np.mean(list(means.values()))), 30.0 if e <= 100 else 60.0)
                c.metrics(**{f"mean_{k}": round(v, 2) for k, v in means.items()},
                          std_avg=float(np.mean([v["std"] for v in st.values()])),
                          zero_frac=float(np.mean([v["zero_frac"] for v in st.values()])),
                          spread=max(means.values()) - min(means.values()))
                res[(e, g)] = means
    return res


def step_black_level(c: Ctx) -> None:
    c.ensure_mode("raw8")
    target10 = c.regs.read8(0x4009)
    c.say(f"  BLC target register 0x4009 = 0x{target10:02X} ({target10}) at 10 bit  ->  ~{target10 / 4:.1f} DN at 8-bit RAW8")
    res = _dark_grid(c)
    with c.variant("blc_off", "BLC disabled (0x4000[0]=0) at 100 rows / 1x"):
        c.regs.freeze_loops(100, 0x10, awb_gains=None)
        c.regs.set_bits(0x4000, 0x01, 0x00)
        imgs = c.capture(6, flush=4, full_regs=True)
        st = an.plane_stats(an.average_frames(c.frames(imgs)))
        c.metrics(**{f"mean_{k}": round(v["mean"], 2) for k, v in st.items()})
    # dark current slope at 1x: DN per row-time
    xs = [e for (e, g) in res if g == 0x10]
    if len(xs) >= 2:
        ys = [np.mean(list(res[(e, 0x10)].values())) for e in xs]
        slope = np.polyfit(xs, ys, 1)[0]
        c.say(f"\n  mean vs exposure at 1x: slope {slope:.4f} DN per row of exposure (dark-current growth)")
        c.variants[-1]["metrics"]["dark_slope_dn_per_row_1x"] = float(slope)


def step_lens_shading(c: Ctx) -> None:
    c.ensure_mode("raw8")
    with c.variant("flat_field_raw", "frames averaged, AEC/AWB frozen after settling"):
        e, g = c.freeze_current(awb=None)
        c.say(f"      frozen at exposure={e} rows, gain={g:.2f}x")
        imgs = c.capture(int(c.opt("n", 16)), flush=4, full_regs=True)
        raw = an.average_frames(c.frames(imgs))
        planes = an.bayer_planes(raw)
        g_plane = (planes["Gr"] + planes["Gb"]) / 2
        import cv2
        blur = cv2.GaussianBlur(g_plane, (0, 0), max(g_plane.shape) / 40)
        cy, cx = np.unravel_index(int(np.argmax(blur)), blur.shape)
        h, w = g_plane.shape
        grids = {k: an.grid_means(p, 6 if k in ("Gr", "Gb") else 5) for k, p in planes.items()}
        centre = float(an.center_patch(g_plane, 0.1).mean())
        gain_g = centre / np.maximum(grids["Gr"], 1e-6)          # 6x6 correction gains for green
        corner = float(np.mean([grids["Gr"][0, 0], grids["Gr"][0, -1], grids["Gr"][-1, 0], grids["Gr"][-1, -1]]))
        rg = an.grid_means(planes["R"] / np.maximum(g_plane[:planes["R"].shape[0], :planes["R"].shape[1]], 1), 5)
        bg = an.grid_means(planes["B"] / np.maximum(g_plane[:planes["B"].shape[0], :planes["B"].shape[1]], 1), 5)
        c.metrics(centre_dn=centre, centre_pct_full_scale=100 * centre / 255,
                  clipped=bool(centre > 250), corner_over_centre=corner / centre,
                  optical_centre_px=f"({2 * cx},{2 * cy}) of {2 * w}x{2 * h}",
                  RG_corner_over_centre=float(rg[0, 0] / rg[2, 2]), BG_corner_over_centre=float(bg[0, 0] / bg[2, 2]),
                  max_green_gain=float(gain_g.max()))
        (c.dir / "shading_map.json").write_text(json.dumps({
            "mode": c.app.mode, "centre_dn": centre,
            "green_6x6_correction_gain": gain_g.round(4).tolist(),
            "grid_means": {k: v.round(2).tolist() for k, v in grids.items()},
            "R_over_G_5x5": rg.round(4).tolist(), "B_over_G_5x5": bg.round(4).tolist()}, indent=1))
        c.say("      green 6x6 correction gain (centre = 1.0):")
        for row in gain_g:
            c.say("        " + " ".join(f"{x:5.2f}" for x in row))
    c.ensure_mode("jpeg")
    for name, val in (("lenc_on", 0x80), ("lenc_off", 0x00)):
        with c.variant(name, f"0x5000[7]={val >> 7} (JPEG output)"):
            c.freeze_current()
            c.regs.set_bits(0x5000, 0x80, val)
            im = c.frames(c.capture(2, flush=5))[-1]
            y = an.luma(im).astype(np.float64)
            h, w = y.shape
            cen, cor = an.center_patch(y, 0.1).mean(), np.mean([y[:h // 10, :w // 10].mean(), y[:h // 10, -w // 10:].mean(),
                                                            y[-h // 10:, :w // 10].mean(), y[-h // 10:, -w // 10:].mean()])
            c.metrics(centre=float(cen), corner=float(cor), corner_over_centre=float(cor / max(cen, 1e-6)))


def step_awb(c: Ctx) -> None:
    label = str(c.opt("illuminant", "unknown"))
    c.ensure_mode("raw8")
    measured = None
    with c.variant(f"measure_{label}", "grey card, AWB off, RAW8 (linear) -> gains for this illuminant"):
        c.freeze_current(awb=None)
        imgs = c.capture(8, flush=3)
        raw = an.average_frames(c.frames(imgs))
        pl = an.bayer_planes(an.center_patch(raw, 0.3))
        r_m, b_m = pl["R"].mean(), pl["B"].mean()
        g_m = (pl["Gr"].mean() + pl["Gb"].mean()) / 2
        gr, gb = g_m / r_m, g_m / b_m
        measured = (int(round(1024 * gr)), 0x400, int(round(1024 * gb)))
        c.metrics(R=r_m, G=g_m, B=b_m, g_R=gr, g_B=gb, reg_R=f"0x{measured[0]:04X}", reg_B=f"0x{measured[2]:04X}",
                  R_over_B_gain=gr / gb)
        table = c.dir / "awb_table.json"
        tab = json.loads(table.read_text()) if table.exists() else {}
        tab[label] = {"g_R": gr, "g_B": gb, "reg": [f"0x{x:04X}" for x in measured], "R_gain_over_B_gain": gr / gb,
                      "mode": c.app.mode, "time": time.strftime("%H:%M:%S")}
        table.write_text(json.dumps(tab, indent=1))
        c.say(f"      saved to {table.name}: run this step again under each illuminant with illuminant=<name>")
    c.ensure_mode("jpeg")
    for name, gains in (("auto_awb", None), ("manual_unity", (0x400, 0x400, 0x400)), ("manual_measured", measured)):
        with c.variant(name, "JPEG output, grey card centre patch"):
            if gains is not None:
                c.regs.freeze_loops(None, None, awb_gains=gains)
            imgs = c.capture(3, flush=6)
            m = an.rgb_means(c.frames(imgs)[-1])
            rb = c.regs.read(0x519F, 6)
            c.metrics(R_over_G=m["R/G"], B_over_G=m["B/G"], neutral=abs(m["R/G"] - 1) < 0.05 and abs(m["B/G"] - 1) < 0.05,
                      awb_readback=" ".join(f"{x:02X}" for x in rb))


def step_ccm(c: Ctx) -> None:
    c.ensure_mode("jpeg")
    with c.variant("ccm_on_default", "driver-default matrix, AWB frozen at unity"):
        c.freeze_current()
        imgs = c.capture(3, flush=5, full_regs=True)
        im = c.frames(imgs)[-1]
        hsv = __import__("cv2").cvtColor(im, __import__("cv2").COLOR_BGR2HSV)
        c.metrics(mean_saturation=float(hsv[..., 1].mean()), **{k: round(v, 3) for k, v in an.rgb_means(im).items()})
    with c.variant("cmx_bypass", "0x5001[1]=0 -- also bypasses the RGB->YUV conversion, so colours break"):
        c.freeze_current()
        c.regs.set_bits(0x5001, 0x02, 0x00)
        im = c.frames(c.capture(3, flush=5, full_regs=True))[-1]
        hsv = __import__("cv2").cvtColor(im, __import__("cv2").COLOR_BGR2HSV)
        c.metrics(mean_saturation=float(hsv[..., 1].mean()), **{k: round(v, 3) for k, v in an.rgb_means(im).items()})
    c.ensure_mode("raw8")
    with c.variant("chart_raw", "linear RAW8 of the same chart for offline CCM fitting"):
        c.freeze_current(awb=None)
        c.capture(4, flush=4, full_regs=True)
        c.note("saved for the offline fit (24 patch means -> constrained least squares, see playbook step 6)")


def step_gamma(c: Ctx) -> None:
    c.ensure_mode("jpeg")
    for name, on in (("gamma_on", True), ("gamma_off", False)):
        with c.variant(name, f"0x5000[5]={int(on)}"):
            c.freeze_current()
            c.regs.set_bits(0x5000, 0x20, 0x20 if on else 0x00)
            im = c.frames(c.capture(3, flush=6))[-1]
            y = an.luma(im).astype(np.float64)
            p = np.percentile(y, [5, 25, 50, 75, 95])
            c.metrics(mean=float(y.mean()), p5=p[0], p25=p[1], p50=p[2], p75=p[3], p95=p[4],
                      hist_std=float(y.std()))


def step_dpc(c: Ctx) -> None:
    c.ensure_mode("raw8")
    vts = c.regs.read16(0x380E)
    e = int(c.opt("exposure", max(vts - 8, 20)))
    for name, bits in (("dpc_off", 0x00), ("dpc_white", 0x02), ("dpc_black", 0x04), ("dpc_both", 0x06)):
        with c.variant(name, f"0x5000[2:1]={bits:#04x}, {e} rows, 8x gain"):
            c.regs.freeze_loops(e, 0x80, awb_gains=None)
            c.regs.set_bits(0x5000, 0x06, bits)
            imgs = c.capture(8, flush=4, full_regs=(name == "dpc_off"))
            frames = c.frames(imgs)
            mean = an.average_frames(frames)
            cnt = [int(an.hot_pixels(f).sum()) for f in frames]
            static = an.hot_pixels(mean, 8.0)
            c.dark_check(float(mean.mean()), 40.0)
            c.metrics(hot_per_frame=float(np.mean(cnt)), hot_in_mean=int(static.sum()),
                      mean_dn=float(mean.mean()), max_dn=float(mean.max()))
            np.save(c.dir / c.cur["dir"] / "hot_map.npy", static)
            c.cur["files"].append("hot_map.npy")


def step_cip(c: Ctx) -> None:
    c.ensure_mode("jpeg")
    variants = [("default", []),
                ("sharpen_off", [(0x5308, 0x40, 0x40), (0x5302, 0x00, 0xFF), (0x5303, 0x00, 0xFF)]),
                ("sharpen_strong", [(0x5308, 0x40, 0x40), (0x5302, 0x40, 0xFF), (0x5303, 0x40, 0xFF)]),
                ("denoise_strong", [(0x5308, 0x10, 0x10), (0x5306, 0x1F, 0xFF), (0x5307, 0x1F, 0xFF)])]
    for name, writes in variants:
        with c.variant(name, ", ".join(f"0x{a:04X}={v:#04x}" for a, v, _ in writes) or "driver defaults"):
            c.freeze_current()
            if writes:
                c.regs.write(writes)
            imgs = c.frames(c.capture(4, flush=6))
            c.metrics(sharpness_lapvar=an.sharpness(imgs[-1]), temporal_noise_dn=an.temporal_noise(imgs[-1], imgs[-2]),
                      live_530d_530f=" ".join(f"{x:02X}" for x in c.regs.read(0x530D, 3)))


def step_aec_banding(c: Ctx) -> None:
    c.ensure_mode("jpeg")
    fp = c.app.link.call("fps_probe", timeout=30, n=30)
    dt = np.diff(np.array(fp.data["ts"], dtype=np.float64))
    period = float(np.median(dt[dt > 0]))
    vts = c.regs.read16(0x380E)
    t_row = period / vts
    hz = int(c.opt("mains_hz", 50))
    b50, b60 = int(round(10000.0 / t_row)), int(round(8333.3 / t_row))
    c.say(f"  t_row = {t_row:.2f} us (VTS {vts}, frame {period / 1000:.1f} ms)  ->  B50={b50} rows, B60={b60} rows")
    for name, banding in (("banding_off", False), ("banding_on", True)):
        with c.variant(name, f"computed steps written; 0x3A00[5]={int(banding)}, mains {hz} Hz"):
            w = [(0x3A08, (b50 >> 8) & 3, 0x03), (0x3A09, b50 & 0xFF, 0xFF), (0x3A0A, (b60 >> 8) & 3, 0x03),
                 (0x3A0B, b60 & 0xFF, 0xFF), (0x3A0D, min(vts // b60, 0x3F), 0x3F), (0x3A0E, min(vts // b50, 0x3F), 0x3F),
                 (0x3C01, 0x80, 0x80), (0x3C00, 0x04 if hz == 50 else 0x00, 0x04),
                 (0x3A00, 0x20 if banding else 0x00, 0x20)]
            c.regs.write(w)
            imgs = c.capture(int(c.opt("n", 12)), interval_ms=180, flush=5)
            ims = c.frames(imgs)
            amps = [an.row_banding(an.luma(i)) for i in ims]
            means = [float(an.luma(i).mean()) for i in ims]
            exps = [((i.meta["regs"]["3500"] & 15) << 16 | i.meta["regs"]["3501"] << 8 | i.meta["regs"]["3502"]) / 16 for i in imgs]
            band_rows = b50 if hz == 50 else b60
            c.metrics(band_amp_mean=float(np.mean(amps)), band_amp_max=float(np.max(amps)),
                      luma_mean=float(np.mean(means)), luma_std_over_time=float(np.std(means)),
                      exposure_rows_avg=float(np.mean(exps)), exposure_over_band=float(np.mean(exps)) / band_rows)


STEPS: dict[str, Step] = {s.key: s for s in [
    Step("orientation", "1  Orientation (mirror / flip / Bayer phase)",
         "Aim at a scene with obvious left-right and up-down asymmetry (a hand-written letter F) that also contains a "
         "saturated red object against white. Keep it still for the whole step.",
         "The 4 contact-sheet tiles differ only by mirroring/flipping. mirror_bits/flip_bits must follow the variant. "
         "A red object must stay red (not blue) and edges must not show magenta/green fringes in every variant. In RAW8 "
         "the green positions (bayer_if_warm_light) shift with mirror/flip -- that is the Bayer phase the ISP must be told "
         "about. Choose the variant matching your mounting and apply it with `orient <mirror> <flip>`.",
         step_orientation),
    Step("timing", "2  Window and frame timing (HTS / VTS / t_row)",
         "Nothing physical. Any static scene.",
         "fps ~ 1 / frame_period; pclk = HTS*VTS*fps; t_row = period / VTS. B50_computed should equal B50_reg (and B60) "
         "within ~3%: if not, the banding registers of this mode are stale and step 10 recomputes them. Jitter should be "
         "tiny (a few us) -- a large jitter means AEC is changing VTS (night mode) under you.",
         step_timing),
    Step("black_level", "3  Black level (BLC) -- RAW8 dark frames",
         "PUT THE LENS CAP ON and make the module light-tight (closed box). Wait a minute for temperature to settle.",
         "For every (exposure, gain): the four Bayer plane means agree within ~1 DN (a Gr/Gb split points at a readout "
         "problem, not BLC); zero_frac stays ~0 (a clipped left tail means the BLC target is too low); the mean is close to "
         "target/4 DN (0x4009 is a 10-bit target, RAW8 drops 2 bits) and barely moves with exposure and gain. blc_off shows "
         "how far the raw pedestal sits without correction. NB: RAW8 keeps only the top 8 of 10 bits, so DN steps are 4x "
         "coarser than the sensor's own black level resolution.",
         step_black_level),
    Step("lens_shading", "4  Lens shading (flat field)",
         "Fill the WHOLE frame with a uniformly lit featureless white surface (lightbox + diffuser, or a defocused white "
         "wall lit from both sides). Bright but not clipping. Do not touch the camera afterwards.",
         "centre_pct_full_scale 60-70 and clipped=False. corner_over_centre is the falloff (0.5 = corners get half the "
         "light); RG/BG_corner_over_centre far from 1.0 is colour shading, the part that MUST be corrected. "
         "shading_map.json holds the 6x6 green gain grid a LENC table would need. lenc_on vs lenc_off (JPEG) should show "
         "corner_over_centre moving closer to 1 with LENC on.",
         step_lens_shading),
    Step("awb", "5  White balance (measure gains per illuminant)",
         "Fill the frame with a neutral grey card under ONE illuminant. Pass illuminant=<name> (e.g. tungsten, led4000, "
         "daylight) and repeat the step for each light you care about.",
         "measure_* gives g_R / g_B (register = round(1024*g)) and R_over_B_gain, monotonic with colour temperature "
         "(tungsten highest). manual_measured should give R/G and B/G ~1.0 (neutral=True) while manual_unity shows the "
         "cast of the light; auto_awb should land close to manual_measured. awb_table.json accumulates all illuminants.",
         step_awb),
    Step("ccm", "6  Colour matrix (capture for fitting)",
         "Mount a ColorChecker Classic flat, ~60% of the frame, evenly lit by one illuminant (tilt slightly to avoid glare).",
         "Saves JPEG with the on-chip matrix, a bypass variant, and a linear RAW8 with full register dumps. ccm_on_default: "
         "neutral patches give R/G and B/G near 1 (with AWB frozen at unity they show the light's cast). cmx_bypass is expected "
         "to look BROKEN (strong green cast, R/G ~ 0): the OV5640 folds the RGB->YUV conversion into the same matrix, so "
         "disabling it skips the conversion, not just the colour correction -- it only proves the register acts, never ship it. "
         "The 24-patch least-squares fit itself is done offline from chart_raw (playbook step 6) -- it is not automated here.",
         step_ccm),
    Step("gamma", "7  Gamma",
         "Aim at a scene with a broad tonal range (grey wedge or the neutral patches of a chart). Keep it still.",
         "With exposure frozen, gamma_off must look darker and flatter in the mid-tones (lower p25/p50, larger gap "
         "p95-p50) than gamma_on. p5/p95 spread and hist_std tell you how much contrast the curve adds.",
         step_gamma),
    Step("dpc", "8  Defect pixels (DPC)",
         "PUT THE LENS CAP ON (dark). Uses a long exposure (exposure=<rows> to override) so hot pixels show.",
         "hot_in_mean (pixels standing out in the 8-frame average) should DROP going from dpc_off to dpc_white / dpc_both. "
         "If all four are equal, DPC does not act on this RAW path -- compare the JPEG path too. Some residual hot pixels "
         "at long exposure are normal; static ones are the worst for feature trackers.",
         step_dpc),
    Step("cip", "9  Denoise and sharpen (CIP)",
         "Aim at a static scene with fine texture, an edge and a flat area near the centre. Keep it still.",
         "sharpness_lapvar: sharpen_strong > default > sharpen_off; temporal_noise_dn: denoise_strong < default. If a "
         "variant equals default the register is not effective in this mode. For VIO prefer mild sharpening: it adds "
         "overshoot that biases corner localisation.",
         step_cip),
    Step("aec_banding", "10  AEC / AGC and banding",
         "Light the scene with a MAINS-powered lamp (fluorescent or a cheap LED bulb; not a DC lamp). Pass mains_hz=60 "
         "for 60 Hz countries.",
         "banding_on: band_amp_mean clearly lower than banding_off, exposure_over_band close to an integer, luma_std_over_time "
         "small. With banding off and a mains lamp, band_amp is large and the picture shows horizontal stripes.",
         step_aec_banding),
]}


class Tuner:
    def __init__(self, app):
        self.app = app

    def command(self, a: list[str]) -> None:
        out = self.app.out
        if not a or a[0] == "list":
            out("tuning steps (run in this order; each one assumes the earlier ones are right):")
            for s in STEPS.values():
                out(f"  {s.key:<12} {s.title}")
            out("run one:  tune <step> [--yes] [--keep] [key=value ...]     other: tune notes <step> | tune backup [name] | tune revert")
            return
        if a[0] == "notes":
            s = STEPS[a[1]]
            out(f"{s.title}\n\nSET UP: {s.setup}\n\nEXPECT: {s.expect}")
            return
        if a[0] == "backup":
            self.app.backup(a[1] if len(a) > 1 else time.strftime("manual_%H%M%S"))
            return
        if a[0] == "revert":
            out(f"reverted {self.app.regs.revert_to(0)} registers written this session")
            return
        if a[0] not in STEPS:
            out(f"unknown step {a[0]!r}; `tune list`")
            return
        self.run(STEPS[a[0]], a[1:])

    def run(self, step: Step, rest: list[str]) -> None:
        out = self.app.out
        yes, keep = "--yes" in rest, "--keep" in rest
        opts = dict(kv.split("=", 1) for kv in rest if "=" in kv)
        out(f"\n=== {step.title} ===")
        before = self.app.backup(f"{step.key}_before_{time.strftime('%H%M%S')}")
        out(f"\nSET UP: {step.setup}")
        out(f"EXPECT: {step.expect}\n")
        if not yes:
            # self.app.readline(), not bare input(): the default implementation is input()
            # (interactive cli.py terminal use), but the web calibration console (see
            # console.py) overrides it per-instance to block on a queue instead, so a `tune`
            # step run from the browser can wait for a "ready" click instead of a keypress.
            if self.app.readline("press Enter when the scene is ready (q = skip): ").strip().lower() == "q":
                return
        ctx = Ctx(self, step, opts, keep)
        mark = self.app.regs.mark()
        mode0 = dict(self.app.mode)
        t0 = time.monotonic()
        try:
            step.fn(ctx)
        finally:
            if not keep:
                self.app.regs.revert_to(mark)
                if self.app.mode != mode0:      # steps switch between jpeg/raw8; leave the camera as we found it
                    out(f"  restoring camera mode {mode0['fmt']} {mode0['size']}")
                    self.app.set_mode(mode0["fmt"], mode0["size"])
        out(f"\n--- {step.key} finished in {time.monotonic() - t0:.0f}s: {len(ctx.variants)} variants in {ctx.dir}")
        (ctx.dir / "analysis.json").write_text(json.dumps(
            {"step": step.key, "title": step.title, "options": opts, "mode": self.app.mode,
             "backup_before": str(before), "variants": ctx.variants}, indent=1, default=str))
        lines = [f"# {step.title}", "", f"*Backup of all registers before this step:* `{before.name}`", "",
                 f"**Set up:** {step.setup}", "", f"**What to expect:** {step.expect}", "", "## Variants", ""]
        for v in ctx.variants:
            lines.append(f"### {v['name']}  ({v['desc']})")
            lines += [f"- {k}: {val}" for k, val in v["metrics"].items()]
            if v.get("reg_changes"):
                lines.append("- register changes: " + ", ".join(f"{r['addr']}: {r['old']}->{r['new']}" for r in v["reg_changes"]))
            lines.append(f"- files: `{v['dir']}/`")
            lines.append("")
        (ctx.dir / "report.md").write_text("\n".join(lines))
        out(f"report: {ctx.dir / 'report.md'}")
        out(f"EXPECTED: {step.expect}")
        self.app.session.log("tune", step=step.key, variants=len(ctx.variants), kept=keep)
        if keep:
            out("--keep: register changes of the last variant were left applied (backup above restores them: `restore <file> --apply`)")
