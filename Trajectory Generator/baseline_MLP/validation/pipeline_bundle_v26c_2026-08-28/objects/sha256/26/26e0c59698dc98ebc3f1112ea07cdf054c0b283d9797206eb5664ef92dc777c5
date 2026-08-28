"""Lightweight, cross-platform progress display for training and inference.

Pure-stdlib helpers (no torch/ray import) so they can be used in any process,
including before the heavy training stack is loaded. Two public pieces:

* :func:`format_hms` / :func:`render_bar` - formatting primitives.
* :class:`LiveProgress` - a thread-safe live one-line progress bar that renders
  in place on a TTY (carriage return) and degrades to periodic newline updates
  when the stream is redirected to a file or pipe. ``log()`` prints a permanent
  line *above* the live bar without corrupting it.

Everything is defensive: a rendering error is swallowed so a cosmetic display
bug can never crash a training run. Unicode block/spinner glyphs are used only
when the target stream can encode them (Windows legacy consoles fall back to
ASCII automatically), keeping Windows x86 and macOS arm64 consistent.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from typing import TextIO

_UNICODE_BAR = ("█", "░")  # full block, light shade
_ASCII_BAR = ("#", "-")
_UNICODE_SPINNER = "⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏"
_ASCII_SPINNER = "|/-\\"


def format_hms(seconds: float | None) -> str:
    """Format a duration in seconds as ``H:MM:SS`` (``--:--:--`` if unknown)."""
    if seconds is None or not math.isfinite(seconds) or seconds < 0:
        return "--:--:--"
    total = int(round(seconds))
    hours, rem = divmod(total, 3600)
    minutes, secs = divmod(rem, 60)
    return f"{hours:d}:{minutes:02d}:{secs:02d}"


def _enable_windows_utf8_console() -> None:
    """Best-effort: switch the Windows console output code page to UTF-8 (65001).

    Python already emits UTF-8 to the stream; a legacy console code page
    (1252/850) renders the Unicode bar/spinner as mojibake (``â–ˆ``, ``â ‹``).
    Aligning the console to UTF-8 makes the glyphs render correctly. No-op off
    Windows or when no console is attached (output piped to a file)."""
    if sys.platform != "win32":
        return
    try:
        import ctypes

        ctypes.windll.kernel32.SetConsoleOutputCP(65001)
    except Exception:
        pass


def _windows_console_is_utf8() -> bool:
    """True off Windows; on Windows, whether the console output CP is UTF-8."""
    if sys.platform != "win32":
        return True
    try:
        import ctypes

        return int(ctypes.windll.kernel32.GetConsoleOutputCP()) == 65001
    except Exception:
        return False


def _stream_supports_unicode(stream: TextIO) -> bool:
    encoding = getattr(stream, "encoding", None) or ""
    try:
        (_UNICODE_BAR[0] + _UNICODE_BAR[1] + _UNICODE_SPINNER[0]).encode(encoding)
    except (LookupError, UnicodeError):
        return False
    # A utf-8 *stream* encoding can still front a legacy *console* code page on
    # Windows, which would mojibake the glyphs. Require the real console CP to be
    # UTF-8; otherwise fall back to ASCII (clean, never garbled).
    return _windows_console_is_utf8()


def render_bar(fraction: float | None, width: int = 24, *, unicode: bool = True) -> str:
    """Render a ``[####----]`` style bar for ``fraction`` in ``[0, 1]``."""
    full, empty = _UNICODE_BAR if unicode else _ASCII_BAR
    if fraction is None or not math.isfinite(fraction):
        fraction = 0.0
    fraction = max(0.0, min(1.0, fraction))
    filled = int(round(fraction * width))
    return full * filled + empty * (width - filled)


class LiveProgress:
    """Thread-safe in-place progress line with elapsed / ETA and a spinner.

    The owner updates state via :meth:`update` (and optional :meth:`log` for
    permanent lines); a background caller (or the owning monitor thread) calls
    :meth:`render` periodically. All writes go through a lock so a permanent log
    line and a live re-render never interleave on the same console line.
    """

    def __init__(
        self,
        *,
        total: int | None = None,
        label: str = "",
        stream: TextIO | None = None,
        bar_width: int = 24,
        min_redraw_interval_s: float = 0.0,
        non_tty_interval_s: float = 30.0,
    ) -> None:
        self._stream = stream if stream is not None else sys.stderr
        self._total = int(total) if total else None
        self._label = label
        self._bar_width = int(bar_width)
        self._lock = threading.Lock()
        # Align the Windows console to UTF-8 first, then detect: pretty glyphs on
        # a real console, clean ASCII when piped to a file (never mojibake).
        _enable_windows_utf8_console()
        self._unicode = _stream_supports_unicode(self._stream)
        try:
            self._isatty = bool(self._stream.isatty())
        except Exception:
            self._isatty = False
        self._min_redraw_interval_s = float(min_redraw_interval_s)
        self._non_tty_interval_s = float(non_tty_interval_s)
        self._start = time.monotonic()
        self._eta_start = self._start
        self._eta_completed_base = 0
        self._completed = 0
        self._phase = label or "starting"
        self._phase_started = self._start
        self._suffix = ""
        self._spinner_idx = 0
        self._last_render = 0.0
        self._last_line_len = 0
        self._active = False

    # -- state updates -------------------------------------------------
    def update(
        self,
        *,
        completed: int | None = None,
        phase: str | None = None,
        phase_reset: bool = False,
        suffix: str | None = None,
        eta_reset: bool = False,
    ) -> None:
        with self._lock:
            if completed is not None:
                self._completed = int(completed)
            if eta_reset:
                self._eta_start = time.monotonic()
                self._eta_completed_base = self._completed
            if phase is not None:
                self._phase = phase
            if phase_reset:
                self._phase_started = time.monotonic()
            if suffix is not None:
                self._suffix = suffix

    # -- rendering -----------------------------------------------------
    def _compose(self) -> str:
        now = time.monotonic()
        elapsed = now - self._start
        fraction = None
        counter = ""
        eta = None
        if self._total:
            fraction = self._completed / self._total
            counter = f"{self._completed}/{self._total}"
            completed_since_eta_reset = self._completed - self._eta_completed_base
            if completed_since_eta_reset > 0:
                per = (now - self._eta_start) / completed_since_eta_reset
                eta = per * (self._total - self._completed)
        spinner_set = _UNICODE_SPINNER if self._unicode else _ASCII_SPINNER
        spin = spinner_set[self._spinner_idx % len(spinner_set)]
        parts: list[str] = [spin]
        if self._label:
            parts.append(self._label)
        if fraction is not None:
            parts.append(f"[{render_bar(fraction, self._bar_width, unicode=self._unicode)}]")
            parts.append(f"{fraction * 100:5.1f}%")
        if counter:
            parts.append(f"iter {counter}")
        parts.append(f"elapsed {format_hms(elapsed)}")
        if eta is not None:
            parts.append(f"ETA {format_hms(eta)}")
        if self._phase:
            phase_elapsed = now - self._phase_started
            parts.append(f"{self._phase} {format_hms(phase_elapsed)}")
        if self._suffix:
            parts.append(self._suffix)
        return " | ".join(parts)

    def render(self, *, advance_spinner: bool = True) -> None:
        with self._lock:
            now = time.monotonic()
            if advance_spinner:
                self._spinner_idx += 1
            if self._isatty:
                if (
                    self._min_redraw_interval_s > 0.0
                    and now - self._last_render < self._min_redraw_interval_s
                ):
                    return
            else:
                if now - self._last_render < self._non_tty_interval_s:
                    return
            line = self._compose()
            self._write_line(line)
            self._last_render = now

    def _write_line(self, line: str) -> None:
        try:
            if self._isatty:
                pad = max(0, self._last_line_len - len(line))
                # Emit one carriage-return frame per refresh. Rewriting the
                # same line twice looks identical in a terminal, but IDE PTYs
                # and log collectors can preserve both frames as duplicates.
                self._stream.write("\r" + line + (" " * pad))
            else:
                self._stream.write(line + "\n")
            self._stream.flush()
            self._last_line_len = len(line)
            self._active = self._isatty
        except Exception:
            pass

    def log(self, message: str) -> None:
        """Print a permanent line above the live bar."""
        with self._lock:
            try:
                if self._isatty and self._active:
                    clear = " " * self._last_line_len
                    self._stream.write("\r" + clear + "\r")
                self._stream.write(message.rstrip("\n") + "\n")
                self._stream.flush()
                self._last_line_len = 0
                self._active = False
            except Exception:
                pass

    def finish(self, message: str | None = None) -> None:
        with self._lock:
            try:
                if self._isatty and self._active:
                    self._stream.write("\n")
                if message is not None:
                    self._stream.write(message.rstrip("\n") + "\n")
                self._stream.flush()
                self._active = False
                self._last_line_len = 0
            except Exception:
                pass
