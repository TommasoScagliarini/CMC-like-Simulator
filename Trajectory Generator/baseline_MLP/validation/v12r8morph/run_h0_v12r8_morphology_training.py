"""Authorized launcher for post-qualification checkpoint-zero resume training."""

from __future__ import annotations

import os
import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent
BASELINE_ROOT = HERE.parents[1]
for _root in (HERE, BASELINE_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_v12r8_morphology_causal_runtime as causal_runtime  # noqa: E402


def configure_process() -> dict[str, object]:
    """Propagate the additive hook to supervisor children and Ray workers."""

    os.environ[causal_runtime.SITE_MARKER_ENV] = causal_runtime.RUNTIME_ID
    current = [item for item in os.environ.get("PYTHONPATH", "").split(os.pathsep) if item]
    if str(HERE) not in current:
        os.environ["PYTHONPATH"] = os.pathsep.join([str(HERE), *current])
    installed = causal_runtime.install()
    return {
        **installed,
        "site_hook_root": str(HERE),
        "marker": os.environ[causal_runtime.SITE_MARKER_ENV],
        "pythonpath_propagated": str(HERE)
        in os.environ.get("PYTHONPATH", "").split(os.pathsep),
    }


def main() -> None:
    configure_process()
    causal_runtime.assert_installed()
    import train_ppo_mlp

    causal_runtime.assert_installed()
    train_ppo_mlp.main()


if __name__ == "__main__":
    main()


__all__ = ["configure_process", "main"]
