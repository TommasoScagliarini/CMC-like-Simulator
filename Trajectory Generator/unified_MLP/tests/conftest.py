"""Make the documented test command portable from any working directory."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from unified_MLP._bootstrap import ensure_sim_paths

ensure_sim_paths()
from baseline_MLP import win_runtime  # noqa: E402,F401 (Torch before OpenSim)
