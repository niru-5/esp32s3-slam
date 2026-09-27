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

    def close(self) -> None:
        self._stop = True
