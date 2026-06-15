"""Regression tests for the training progress display."""

from __future__ import annotations

import importlib.util
import io
from pathlib import Path
from unittest import mock
import unittest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "Trajectory Generator" / "baseline_MLP" / "progress_display.py"
SPEC = importlib.util.spec_from_file_location("progress_display_under_test", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
progress_display = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(progress_display)


class FakeTTY(io.StringIO):
    """String stream that exercises the in-place TTY rendering path."""

    def isatty(self) -> bool:
        return True


class LiveProgressTests(unittest.TestCase):
    def test_tty_refresh_emits_each_frame_once_and_clears_old_tail(self) -> None:
        stream = FakeTTY()
        progress = progress_display.LiveProgress(stream=stream)

        progress._write_line("long status")
        progress._write_line("short")

        self.assertEqual(stream.getvalue(), "\rlong status\rshort      ")

    def test_permanent_log_clears_live_line_before_printing(self) -> None:
        stream = FakeTTY()
        progress = progress_display.LiveProgress(stream=stream)

        progress._write_line("live")
        progress.log("[iter 1/2]")
        progress._write_line("next")

        self.assertEqual(stream.getvalue(), "\rlive\r    \r[iter 1/2]\n\rnext")

    def test_eta_uses_only_iterations_completed_after_resume(self) -> None:
        stream = FakeTTY()
        with mock.patch.object(progress_display.time, "monotonic", return_value=100.0):
            progress = progress_display.LiveProgress(total=57, stream=stream)
            progress.update(completed=37, eta_reset=True)

        with mock.patch.object(progress_display.time, "monotonic", return_value=110.0):
            before_first_new_iteration = progress._compose()

        self.assertNotIn("ETA", before_first_new_iteration)

        with mock.patch.object(progress_display.time, "monotonic", return_value=120.0):
            progress.update(completed=38)
            after_first_new_iteration = progress._compose()

        self.assertIn("ETA 0:06:20", after_first_new_iteration)


if __name__ == "__main__":
    unittest.main()
