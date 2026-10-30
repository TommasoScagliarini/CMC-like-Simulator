"""Path bootstrap for the baseline_MLP package.

The CMC-like env (``osim_trj_cmc_like``) lives in ``Trajectory Generator/`` and
imports the simulator modules (``config``, ``model_loader``, ``setup_io``,
``simulation_runner``, ``kinematics_interpolator``) from ``CMC-like Simulator/``.
This helper also exposes phase-tracking, utility, warm-start, adaptation and
run-registry modules under their original flat names. All roots remain
importable regardless of the current working directory or whether modules run
as scripts or via ``python -m``.
"""

from __future__ import annotations

import sys
from pathlib import Path

_THIS = Path(__file__).resolve()
BASELINE_DIR = _THIS.parents[0]            # .../Trajectory Generator/baseline_MLP
UTILITY_DIR = BASELINE_DIR / "utility"
IMITATION_TO_EXNOVO_DIR = BASELINE_DIR / "imitation_to_exnovo"
ADAPTATION_DIR = IMITATION_TO_EXNOVO_DIR / "adaptation"
MORPHOLOGY_CORRIDOR_DIR = BASELINE_DIR / "morphology_corridor"
TRAJ_GEN_DIR = _THIS.parents[1]            # .../Trajectory Generator
PHASE_TRACKING_DIR = TRAJ_GEN_DIR / "phase_tracking"
RUNS_DIR = TRAJ_GEN_DIR / "runs"
REPO_ROOT = _THIS.parents[2]               # shared workspace root
SIMULATOR_DIR = REPO_ROOT / "CMC-like Simulator"


def ensure_sim_paths() -> None:
    """Expose shared modules and simulator roots without changing import names."""
    for path in (
        MORPHOLOGY_CORRIDOR_DIR, ADAPTATION_DIR, IMITATION_TO_EXNOVO_DIR,
        UTILITY_DIR, RUNS_DIR, PHASE_TRACKING_DIR,
        BASELINE_DIR, TRAJ_GEN_DIR, REPO_ROOT,
    ):
        entry = str(path)
        if entry not in sys.path:
            sys.path.insert(0, entry)
    from workspace_bootstrap import ensure_simulator_paths

    ensure_simulator_paths()
