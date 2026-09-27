"""Web-driven front end for cli.py's App: this is how the *whole* register/tuning/
intrinsics workflow becomes a browser feature without rewriting any of it.

A background worker thread runs commands from a queue through App.run_line() --
completely unchanged from the interactive terminal REPL. Output goes into a version-counted
buffer instead of being printed (App's `out` callback), and the one blocking call App/
tuning.py makes (Tuner.run()'s "press Enter when the scene is ready") goes through a second
queue instead of real stdin (App's `readline`, see cli.py). The browser (or `cli.py` itself,
now a thin HTTP client of the same endpoints -- see __main__ wiring in host_server/app.py)
is just another caller of submit()/output()/answer().

Exactly one command runs at a time -- same as a real terminal REPL, and matching
DeviceSession's own one-call-in-flight constraint.
"""

from __future__ import annotations

import queue
import threading


class CalibrationConsole:
    def __init__(self, app_factory):
        """`app_factory(out, readline) -> App`. Deferred (not just an App instance) so the
        caller can finish other setup (e.g. the startup register backup) using the same
        `out`/`readline` before the worker thread starts consuming commands."""
        self._lines: list[str] = []
        self._lock = threading.Lock()
        self._cmd_queue: "queue.Queue[str]" = queue.Queue()
        self._answer_queue: "queue.Queue[str]" = queue.Queue()
        self._waiting_for_answer = False
        self._busy = False
        self._stop = False
        self.app = app_factory(out=self._append, readline=self._readline)
        self._thread = threading.Thread(target=self._worker, daemon=True, name="calib-console")
        self._thread.start()

    # -- App callbacks -------------------------------------------------------
    def _append(self, text) -> None:
        with self._lock:
            self._lines.append(str(text))

    def _readline(self, prompt: str = "") -> str:
        self._append(prompt)
        self._waiting_for_answer = True
        try:
            return self._answer_queue.get()
        finally:
            self._waiting_for_answer = False

    # -- driven by the HTTP layer ---------------------------------------------
    def submit(self, line: str) -> None:
        self._cmd_queue.put(line)

    def answer(self, line: str) -> bool:
        """Feed a pending readline() prompt. Returns False if nothing's waiting for one."""
        if not self._waiting_for_answer:
            return False
        self._answer_queue.put(line)
        return True

    def output(self, since: int = 0) -> tuple[list[str], int]:
        """Lines appended after index `since`, and the new total count (pass that back as
        the next call's `since` to keep polling from where you left off)."""
        with self._lock:
            return self._lines[since:], len(self._lines)

    @property
    def busy(self) -> bool:
        return self._busy or self._waiting_for_answer

    @property
    def waiting_for_answer(self) -> bool:
        """True while a running command is blocked on readline() (e.g. a `tune` step's
        "press Enter when ready") -- callers (the browser, cli.py's terminal client) use
        this to route typed input to /calib/answer instead of /calib/line."""
        return self._waiting_for_answer

    def _worker(self) -> None:
        while not self._stop:
            try:
                line = self._cmd_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            self._busy = True
            self._append(f"calib> {line}")
            try:
                self.app.run_line(line)
            except Exception as exc:   # keep the console alive even if a command misbehaves
                self._append(f"error: {exc}")
            finally:
                self._busy = False

    def close(self, timeout: float = 20.0) -> None:
        """Stop accepting new commands and wait for whatever's currently running to actually
        finish, instead of just flagging `_stop` and walking away.

        `_stop = True` alone only stops the worker picking up its *next* queued command --
        the loop is still blocked inside `self.app.run_line(line)` for however long the
        in-flight one takes (a multi-variant `tune` step can run tens of seconds). A caller
        that tears the console down right after this returns (CalibrationManager.leave(),
        via Server._set_state() when the browser switches away from Camera Calibration
        without going through the console's own `exit`) used to set `self.console = None`
        immediately while that orphaned thread kept running against the *same* DeviceSession
        -- once the device itself left camera_calibration, every further reg_read/reg_write/
        capture call from the orphaned step started failing "not in camera_calibration",
        appended to a buffer nothing polls any more, with the step's own register-revert
        `finally` blocks then failing too. That's the "no graceful exit" failure: a step
        started, the operator switched to Streaming mid-run, and things silently half-broke.

        If the worker is blocked on `tune`'s "press Enter when ready" readline(), nothing
        will ever answer it now -- push "q" (tuning.py's own skip-the-step answer) so it
        unblocks instead of hanging until the timeout. Otherwise just wait for the current
        command to run to completion (or fail fast once the device state changes under it,
        which it currently still can if the device.call() itself is what's mid-flight when
        the caller's own set_state lands -- see Server._set_state()'s ordering, which closes
        the console *before* changing device state specifically to avoid that)."""
        self._stop = True
        if self._waiting_for_answer:
            self._answer_queue.put("q")
        self._thread.join(timeout=timeout)
        if self._thread.is_alive():
            print(f"[calib-console] warning: worker still busy after {timeout:.0f}s close() "
                  f"timeout -- abandoning it (daemon thread, won't block process exit, but its "
                  f"device calls may still race whatever runs next)")
