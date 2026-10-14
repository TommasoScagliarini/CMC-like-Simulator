"""Compare the isolated real-row correction to the pinned official Ray fix.

Reconstruct the official connector flags from original episode boundaries.
The artificial rows intentionally retain different semantics in our isolated
experiment, so only real transition targets are compared. No training or env.
"""
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import pickle
import runpy
import sys

import numpy as np

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
PROBE = PACKAGE / "runs/training/MLP39_LEFT_best80_rl_audit_2026-09-29/ppo_audit"
COMMIT = "1927658ad6a97788bd86fc59310d1ed45a08e60e"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    official_file = HERE / "upstream" / COMMIT / "rllib/utils/postprocessing/value_predictions.py"
    captured = PROBE / "gae_values_and_input.pkl"
    stock_file = PROBE / "batch_post_gae.pkl"
    corrected_file = HERE / "boundary_replay_v2/corrected_boundary/ppo_audit/batch_post_gae.pkl"
    boundaries_file = HERE / "learner_analysis/episode_boundaries.csv"
    paths = [official_file, captured, stock_file, corrected_file, boundaries_file, Path(__file__)]
    hashes = {str(path): sha(path) for path in paths}
    with captured.open("rb") as stream:
        values = pickle.load(stream)["values"]["default_policy"]
    with stock_file.open("rb") as stream:
        stock = pickle.load(stream)["default_policy"]
    with corrected_file.open("rb") as stream:
        corrected = pickle.load(stream)["default_policy"]
    mask = np.asarray(stock["loss_mask"], dtype=bool)
    terminateds = np.zeros(len(mask), dtype=bool)
    truncateds = np.zeros(len(mask), dtype=bool)
    with boundaries_file.open(newline="") as stream:
        boundaries = list(csv.DictReader(stream))
    for row in boundaries:
        bootstrap = int(row["bootstrap_row"])
        assert not mask[bootstrap] and mask[bootstrap - 1]
        terminal = row["terminated"] == "True"
        terminateds[bootstrap - 1] = terminal
        truncateds[bootstrap - 1] = not terminal
    function = runpy.run_path(str(official_file))["compute_value_targets"]
    official = function(values=np.asarray(values), rewards=np.asarray(stock["rewards"]),
                        terminateds=terminateds, truncateds=truncateds,
                        gamma=.99, lambda_=.9)
    isolated = np.asarray(corrected["value_targets"])
    difference = official[mask] - isolated[mask]
    np.testing.assert_allclose(official[mask], isolated[mask], atol=1e-5, rtol=1e-6)
    assert all(sha(path) == expected for path, expected in hashes.items())
    result = {
        "passed": True, "official_commit": COMMIT,
        "real_rows_compared": int(mask.sum()), "fragments": len(boundaries),
        "max_abs_difference_real_targets": float(np.max(np.abs(difference))),
        "mean_abs_difference_real_targets": float(np.mean(np.abs(difference))),
        "atol": 1e-5, "rtol": 1e-6,
        "comparison_scope": "Real-row value targets with the official corrected flags; artificial targets and normalization are intentionally outside this equivalence claim",
        "input_hashes": hashes, "inputs_unchanged": True,
        "training_updates": 0, "physical_simulations": 0,
    }
    output = HERE / "upstream/real_target_equivalence.json"
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "input_hashes"}, indent=2))


if __name__ == "__main__":
    main()
