"""Portable imports without modifying the baseline or simulator."""
from pathlib import Path
import sys
UNIFIED_DIR = Path(__file__).resolve().parent
TRAJ_GEN_DIR = UNIFIED_DIR.parent
REPO_ROOT = TRAJ_GEN_DIR.parent

def ensure_sim_paths():
    for path in (TRAJ_GEN_DIR, REPO_ROOT):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    # Legacy utilities still use sibling imports internally. Append only;
    # unified modules always use their fully qualified package name.
    baseline = str(TRAJ_GEN_DIR / "baseline_MLP")
    if baseline not in sys.path:
        sys.path.append(baseline)
