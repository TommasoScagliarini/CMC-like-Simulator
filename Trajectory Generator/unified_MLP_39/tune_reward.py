"""Portable direct entrypoint for the offline reward tuning workbench."""
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from unified_MLP_39.reward_tuning.__main__ import main

if __name__ == "__main__":
    main()
