"""Device selection.

macOS arm64 / MPS support is deferred (see AGENTS.md). The selector keeps a
single CUDA branch with CPU fallback; explicit `mps` requests fall back to
CPU rather than failing, to keep configs from older runs loadable.
"""

from __future__ import annotations

import torch


def select_device(prefer: str | torch.device | None = "auto") -> torch.device:
    """
    Select a PyTorch device.

    `auto` prefers CUDA, then CPU. Explicit `cuda` requests fall back to CPU
    if CUDA is unavailable. `mps` is accepted but silently falls back to CPU.
    """
    if isinstance(prefer, torch.device):
        return prefer

    choice = (prefer or "auto").lower()
    if choice == "cpu":
        return torch.device("cpu")

    if choice.startswith("cuda"):
        return torch.device(choice if torch.cuda.is_available() else "cpu")

    if choice == "mps":
        # Deferred: see AGENTS.md "Vincoli fondamentali".
        return torch.device("cpu")

    if choice != "auto":
        raise ValueError(f"unknown device preference: {prefer!r}")

    if torch.cuda.is_available():
        return torch.device("cuda")

    return torch.device("cpu")
