"""Expose the two-core workspace without importing numerical or native libraries.

The simulator keeps its historical flat module names for existing consumers and
pickles. Shared assets remain relative to the workspace root, independently of
the caller's working directory. Call this before importing simulator modules.
"""

from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent
SIMULATOR_DIR = REPO_ROOT / "CMC-like Simulator"
TRAJECTORY_GENERATOR_DIR = REPO_ROOT / "Trajectory Generator"


def ensure_simulator_paths() -> None:
    """Make the simulator core and shared workspace packages importable."""
    for directory in (REPO_ROOT, SIMULATOR_DIR):
        entry = str(directory)
        if entry not in sys.path:
            sys.path.insert(0, entry)
