"""OV5640 register access over the control link, with undo log and full-dump backups."""

from __future__ import annotations

import json
import time
from pathlib import Path

from .link import DeviceLink, LinkError

# Every register range worth backing up. The chip decodes far more addresses than
# it documents; these cover system/timing (0x3000-0x3FFF), BLC/format/DVP
# (0x4000-0x4FFF) and the ISP (0x5000-0x5FFF).
FULL_DUMP_RANGES = [(0x3000, 0x3FFF), (0x4000, 0x4FFF), (0x5000, 0x5FFF)]

# Registers that must never be replayed by ``restore``: software reset/power,
# clock tree, pad enables, SCCB/system control, and live status/readback that a
# write cannot meaningfully "restore" (AWB/AEC statistics, band-detect readbacks). A restore only writes
# registers that (a) differ from the backup, (b) are not in this list and (c) are not volatile (see
# RegisterBank.volatile: registers that change between two consecutive dumps).
UNSAFE_RESTORE = set(range(0x3000, 0x3040)) | {0x3103, 0x3108} | set(range(0x3810, 0x3814)) \
    | set(range(0x519F, 0x51A7)) | set(range(0x51C0, 0x51E0)) | set(range(0x5690, 0x56B0)) \
    | set(range(0x5300 + 0x0D, 0x5300 + 0x10)) | {0x350C, 0x350D, 0x3C0C} | set(range(0x3C1A, 0x3C20))


class RegisterError(RuntimeError):
    pass


class RegisterBank:
    def __init__(self, link: DeviceLink):
        self.link = link
        # (addr, old_byte, new_byte) in application order.
        self.undo_log: list[tuple[int, int, int]] = []
        # Current sensor readout orientation, as last confirmed by set_orientation() -- the
        # source of truth for analysis.py's oriented Bayer-tile helpers (bayer_planes/
        # plane_stats/debayer) and for re-applying orientation after a mode switch resets it
        # (see cli.py App.set_mode()). Device default on boot/mode-switch is mirror=flip=False.
        self.mirror = False
        self.flip = False

    # -- reads -------------------------------------------------------------
    def read(self, addr: int, count: int = 1) -> bytes:
        r = self.link.call("reg_read", addr=addr, count=count)
        if not r.ok:
            raise RegisterError(r.data.get("err", "reg_read failed"))
        return bytes.fromhex(r.data["hex"])

    def read8(self, addr: int) -> int:
        return self.read(addr, 1)[0]

    def read16(self, addr: int) -> int:
        b = self.read(addr, 2)
        return (b[0] << 8) | b[1]

    def dump(self, ranges=FULL_DUMP_RANGES) -> dict[int, int]:
        r = self.link.call("reg_dump", timeout=60.0, ranges=[list(x) for x in ranges])
        if not r.ok:
            raise RegisterError(r.data.get("err", "reg_dump failed"))
        regs: dict[int, int] = {}
        for ch in r.chunks:
            for i, byte in enumerate(bytes.fromhex(ch["hex"])):
                regs[ch["start"] + i] = byte
        return regs

    # -- writes ------------------------------------------------------------
    def write(self, writes: list[tuple[int, int, int]], delay_ms: int = 0,
              record: bool = True) -> list[dict]:
        """``writes`` = [(addr, value, mask)]; returns per-write old/new bytes."""
        payload = [{"a": a, "v": v, "m": m} for a, v, m in writes]
        r = self.link.call("reg_write", writes=payload, delay_ms=delay_ms)
        if not r.ok:
            raise RegisterError(r.data.get("err", "reg_write failed"))
        results = r.data["results"]
        for res in results:
            if res.get("rc", 0) < 0:
                raise RegisterError(f"write to 0x{res['a']:04X} failed (rc={res['rc']})")
            if record and res["old"] != res["new"]:
                self.undo_log.append((res["a"], res["old"], res["new"]))
        return results

    def set_bits(self, addr: int, mask: int, value: int, **kw) -> list[dict]:
        return self.write([(addr, value & mask, mask)], **kw)

    def set_orientation(self, mirror: bool, flip: bool) -> dict:
        r = self.link.call("set_orientation", mirror=int(mirror), flip=int(flip))
        if not r.ok:
            raise RegisterError(r.data.get("err", "set_orientation failed"))
        # 0x3820/0x3821 (mirror/flip control bits) plus 0x4514/0x4520 (BLC readout-direction
        # fixups the driver also touches, see control_link.c cmd_set_orientation) -- all four
        # need to land in undo_log so `save --apply` persists the full orientation-dependent
        # register set, not just the two direct control bits. Older firmware without
        # old_4514/old_4520 in its reply (pre this fix) just won't have those keys; skip them.
        for a, k in ((0x3820, "3820"), (0x3821, "3821"), (0x4514, "4514"), (0x4520, "4520")):
            if "old_" + k in r.data and r.data["old_" + k] != r.data["new_" + k]:
                self.undo_log.append((a, r.data["old_" + k], r.data["new_" + k]))
        self.mirror, self.flip = bool(mirror), bool(flip)
        return r.data

    # -- undo / restore ----------------------------------------------------
    def mark(self) -> int:
        return len(self.undo_log)

    def revert_to(self, mark: int) -> int:
        """Undo every recorded write after ``mark``, newest first. Returns count."""
        pending = self.undo_log[mark:]
        if not pending:
            return 0
        # Newest first, one write per distinct address is enough (oldest 'old' wins).
        first_old: dict[int, int] = {}
        for addr, old, _new in pending:
            first_old.setdefault(addr, old)
        self.write([(a, v, 0xFF) for a, v in first_old.items()], record=False)
        del self.undo_log[mark:]
        return len(first_old)

    def volatile(self, first: dict[int, int] | None = None) -> set[int]:
        """Registers that change on their own (AEC/AWB statistics, band-detect readbacks, ...):
        those that differ between two consecutive dumps with no writes in between."""
        a = first if first is not None else self.dump()
        b = self.dump()
        return {k for k in a if k in b and a[k] != b[k]}

    def restore(self, snapshot: dict[int, int], dry_run: bool = False) -> tuple[list[tuple[int, int, int]], int]:
        """Write back registers that differ from ``snapshot``.

        Skips the always-unsafe set (resets, clocks, readbacks) and every *volatile* register (see
        ``volatile``) -- those are live statistics, not settings. Returns ([(addr, snap, now)], n_volatile).
        """
        now = self.dump()
        vol = self.volatile(now)
        diffs = [(a, snapshot[a], now.get(a, -1)) for a in sorted(snapshot)
                 if a in now and snapshot[a] != now[a] and a not in UNSAFE_RESTORE and a not in vol]
        if diffs and not dry_run:
            self.write([(a, v, 0xFF) for a, v, _ in diffs], record=False)
        return diffs, len(vol)

    # -- exposure / gain / AWB helpers (see docs playbook "Freeze the loops") ---
    def freeze_loops(self, exposure_rows: int | None = None, gain_x16: int | None = None,
                     awb_gains: tuple[int, int, int] | None = (0x400, 0x400, 0x400)) -> None:
        """Manual AEC+AGC, optionally manual AWB with the given (R,G,B) 12-bit gains."""
        w: list[tuple[int, int, int]] = [(0x3503, 0x03, 0xFF)]
        if exposure_rows is not None:
            e = exposure_rows << 4
            w += [(0x3500, (e >> 16) & 0x0F, 0xFF), (0x3501, (e >> 8) & 0xFF, 0xFF), (0x3502, e & 0xFF, 0xFF)]
        if gain_x16 is not None:
            w += [(0x350A, (gain_x16 >> 8) & 0x03, 0xFF), (0x350B, gain_x16 & 0xFF, 0xFF)]
        if awb_gains is not None:
            r, g, b = awb_gains
            w += [(0x3406, 0x01, 0xFF),
                  (0x3400, r >> 8, 0xFF), (0x3401, r & 0xFF, 0xFF),
                  (0x3402, g >> 8, 0xFF), (0x3403, g & 0xFF, 0xFF),
                  (0x3404, b >> 8, 0xFF), (0x3405, b & 0xFF, 0xFF)]
        self.write(w)

    def exposure_rows(self) -> int:
        b = self.read(0x3500, 3)
        return (((b[0] & 0x0F) << 16) | (b[1] << 8) | b[2]) >> 4

    def gain_x16(self) -> int:
        b = self.read(0x350A, 2)
        return ((b[0] & 0x03) << 8) | b[1]


# -- backup files ------------------------------------------------------------

def save_dump(path: Path, regs: dict[int, int], **meta) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {"saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"), **meta,
           "count": len(regs), "regs": {f"{a:04X}": v for a, v in sorted(regs.items())}}
    path.write_text(json.dumps(doc, indent=1))


def load_dump(path: Path) -> dict[int, int]:
    doc = json.loads(path.read_text())
    return {int(a, 16): v for a, v in doc["regs"].items()}


def diff_dumps(a: dict[int, int], b: dict[int, int]) -> list[tuple[int, int, int]]:
    return [(k, a[k], b[k]) for k in sorted(a) if k in b and a[k] != b[k]]
