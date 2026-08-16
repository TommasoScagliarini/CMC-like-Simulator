"""Materialize float32 noise tapes before primary-split execution unlocks."""

from __future__ import annotations

import hashlib
import json
import os
import sys
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
for root in (VALIDATION_ROOT, BASELINE_ROOT):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

import compare_h0_v25_abc as strict_io  # noqa: E402
import primary_grf_split_adaptation as split_contract  # noqa: E402
from run_h0_primary_grf_split_v1_adaptation import (  # noqa: E402
    EXPECTED_STEPS,
    _write_npz_exclusive,
    source_record,
)


OUTPUT_ROOT = VALIDATION_ROOT / "h0_primary_grf_split_noise_tapes"
SEEDS = {
    "collection": {"02": 123, "04": 124, "08": 125},
    "qualification": {"02": 126, "04": 127, "08": 128},
}


class NoiseTapeError(RuntimeError):
    pass


def prepare() -> dict:
    if OUTPUT_ROOT.exists():
        raise NoiseTapeError(f"refusing to clobber: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    records: dict[str, dict] = {}
    for stage, trials in SEEDS.items():
        records[stage] = {}
        for trial, seed in trials.items():
            standard_normal = np.random.default_rng(seed).standard_normal(
                (EXPECTED_STEPS, 2)
            ).astype(np.float32)
            if not np.all(np.isfinite(standard_normal)):
                raise NoiseTapeError("generated tape is non-finite")
            path = OUTPUT_ROOT / f"{stage}_trial_{trial}_standard_normal.npz"
            _write_npz_exclusive(
                path,
                standard_normal=standard_normal,
                seed=np.asarray([seed], dtype=np.int64),
            )
            records[stage][trial] = {
                "seed": seed,
                "shape": [EXPECTED_STEPS, 2],
                "dtype": "float32",
                "array_sha256": split_contract.array_sha256(standard_normal),
                "artifact": source_record(path),
            }
    zero = np.zeros((EXPECTED_STEPS, 2), dtype=np.float32)
    zero_path = OUTPUT_ROOT / "qualification_deterministic_all_zero.npz"
    _write_npz_exclusive(zero_path, standard_normal=zero)
    manifest = {
        "schema_version": 1,
        "status": "H0_PRIMARY_GRF_SPLIT_NOISE_TAPES_FROZEN",
        "generator": "numpy.random.default_rng(seed).standard_normal",
        "standard_normal_pre_scaling": True,
        "tapes": records,
        "deterministic": {
            "shape": [EXPECTED_STEPS, 2],
            "dtype": "float32",
            "array_sha256": split_contract.array_sha256(zero),
            "artifact": source_record(zero_path),
        },
        "protected_trials_opened": [],
    }
    strict_io.write_json_exclusive(OUTPUT_ROOT / "manifest.json", manifest)
    return manifest


if __name__ == "__main__":
    print(json.dumps(prepare(), indent=2, sort_keys=True, allow_nan=False))

