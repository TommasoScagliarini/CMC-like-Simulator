"""Preparation utilities, with compatibility for former ``tools.<module>`` imports.

New callers should use ``tools.model_preparation_pipeline.<module>``. Extending
the package search path lets existing audit helpers find the relocated sources
without retaining duplicate files in the old directory.
"""

from pathlib import Path

__path__.append(str(Path(__file__).resolve().parent / "model_preparation_pipeline"))
