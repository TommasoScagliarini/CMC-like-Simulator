"""
outer_pid_local_sweep.py
========================

Focused PID sweep around the current best AB06 outer-loop candidate.

This is a thin wrapper around ``outer_pid_gain_sweep.py``. It keeps the same
validation, filtering, output format and ranking, but supplies a smaller local
grid centered on the best candidate from the 2026-05-14 sweep:

    knee:  Kp=300, Kd=26, Ki=80
    ankle: Kp=750, Kd=2,  Ki=240

By default, running this file without ``--dry-run`` or ``--quick-smoke`` launches
the real focused sweep. Use ``--dry-run`` first to inspect the job count.
"""

from __future__ import annotations

import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import outer_pid_gain_sweep  # noqa: E402


LOCAL_DEFAULTS = [
    "--knee-kp-grid", "260,300,340",
    "--knee-kd-grid", "22,26,30",
    "--knee-ki-grid", "40,80,120",
    "--ankle-kp-grid", "650,750,850",
    "--ankle-kd-grid", "1.5,2,2.5",
    "--ankle-ki-grid", "180,240,300",
    "--top-n-per-joint", "5",
]

DEFAULT_ENV_PYTHON = Path("/opt/anaconda3/envs/envCMC-like/bin/python")


def has_option(args: Iterable[str], option: str) -> bool:
    return any(arg == option or arg.startswith(f"{option}=") for arg in args)


def conda_prefix_python() -> Optional[Path]:
    prefix = os.environ.get("CONDA_PREFIX")
    if not prefix:
        return None
    if Path(prefix).name != "envCMC-like":
        return None
    candidate = Path(prefix) / ("python.exe" if os.name == "nt" else "bin/python")
    return candidate if candidate.is_file() else None


def default_python_executable() -> Optional[Path]:
    conda_python = conda_prefix_python()
    if conda_python is not None:
        return conda_python
    return DEFAULT_ENV_PYTHON if DEFAULT_ENV_PYTHON.is_file() else None


def main() -> int:
    user_args = list(sys.argv[1:])
    forwarded = list(LOCAL_DEFAULTS)

    if not has_option(user_args, "--python"):
        default_python = default_python_executable()
        if default_python is not None:
            forwarded.extend(["--python", str(default_python)])

    if not has_option(user_args, "--sweep-root") and not has_option(user_args, "--dry-run"):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        forwarded.extend([
            "--sweep-root",
            str(REPO_ROOT / "results" / f"_outer_pid_local_sweep_{stamp}"),
        ])

    forwarded.extend(user_args)
    original_argv = sys.argv
    try:
        sys.argv = [str(SCRIPT_DIR / "outer_pid_gain_sweep.py"), *forwarded]
        return outer_pid_gain_sweep.main()
    finally:
        sys.argv = original_argv


if __name__ == "__main__":
    raise SystemExit(main())
