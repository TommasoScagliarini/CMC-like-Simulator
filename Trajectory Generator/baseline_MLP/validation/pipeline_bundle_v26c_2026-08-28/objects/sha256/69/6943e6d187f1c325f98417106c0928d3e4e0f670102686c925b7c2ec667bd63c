"""Path bootstrap for the baseline_MLP package.

The CMC-like env (``osim_trj_cmc_like``) lives in ``Trajectory Generator/`` and
imports the simulator modules (``config``, ``model_loader``, ``setup_io``,
``simulation_runner``, ``kinematics_interpolator``) from the repository root.
This helper makes all three roots importable regardless of the current working
directory or whether the modules are run as scripts or via ``python -m``.
"""

from __future__ import annotations

import sys
from pathlib import Path

_THIS = Path(__file__).resolve()
BASELINE_DIR = _THIS.parents[0]            # .../Trajectory Generator/baseline_MLP
TRAJ_GEN_DIR = _THIS.parents[1]            # .../Trajectory Generator
REPO_ROOT = _THIS.parents[2]               # repository root (CMC-like simulator)


def ensure_sim_paths() -> None:
    """Insert baseline_MLP, Trajectory Generator and repo root on sys.path."""
    for path in (BASELINE_DIR, TRAJ_GEN_DIR, REPO_ROOT):
        entry = str(path)
        if entry not in sys.path:
            sys.path.insert(0, entry)
