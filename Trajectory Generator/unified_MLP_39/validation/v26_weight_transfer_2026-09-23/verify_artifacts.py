"""Verify diagnostic provenance, six RIGHT channels and reward normalization."""
from __future__ import annotations

import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe import HERE, REPO, ROLLOUT, HISTORICAL, JOINTS, PERM, read, sha
import numpy as np


def first_where(mask, trace):
    indices = np.flatnonzero(mask)
    return None if not len(indices) else {"step": int(indices[0] + 1), "time": trace[int(indices[0])]["time"]}


def main():
    source_hashes = read(HERE / "input_hashes.json")
    for name, expected in source_hashes.items():
        assert sha(REPO / name) == expected, f"Input changed: {name}"
    rows = read(ROLLOUT / "rollout_policy_trace.json")
    historical = read(HISTORICAL / "rollout_policy_trace.json")[:len(rows)]
    summary = read(ROLLOUT / "rollout_summary.json")
    contract = read(Path(summary["checkpoint"]) / "diagnostic_contract.json")
    actual = np.array([r["actor_observation_vector_before"] for r in rows])
    expected = np.array([r["actor_observation_vector_before"][:39] for r in historical])[:, PERM]
    channels = [0, 1, 35, 36, 37, 38]
    np.testing.assert_allclose(actual[:, channels], expected[:, channels], atol=2e-6, rtol=0)
    assert all(r["target_availability"]["visible"] for r in rows)
    assert all(r["target_availability"]["episode_probability"] == 1. for r in rows)
    errors = {}
    for j in JOINTS:
        for kind in ("position_range_rad", "velocity_range_rad_s"):
            key = j + "_imitation_" + kind
            a = np.array([r["reward_terms"][key] for r in rows])
            b = np.array([r["reward_terms"][key] for r in historical])
            np.testing.assert_allclose(a, b, atol=1e-12, rtol=0)
            errors[key] = {"value": float(a[0]), "max_abs_error": float(abs(a-b).max())}
    names = contract["actor_feature_names"]
    differences = np.abs(actual - expected)
    divergence = {}
    for i, name in enumerate(names):
        divergence[name] = {"max_abs_difference": float(differences[:, i].max()),
                            "first_above_1e-4": first_where(differences[:, i] > 1e-4, rows)}
    for j in JOINTS:
        deg_error = np.degrees(np.abs(np.array([r["prosthetic_state"][j] for r in rows]) -
                                     np.array([r["prosthetic_state"][j] for r in historical])))
        divergence[j + "_after_step_deg"] = {"first_above_0.1deg": first_where(deg_error > .1, rows),
                                             "first_above_1deg": first_where(deg_error > 1., rows)}
    reset = read(ROLLOUT / "rollout_reset_diagnostics.json")
    source_reset = read(HISTORICAL / "rollout_reset_diagnostics.json")
    for group in ("joints", "sea"):
        for item, values in reset[group].items():
            for key, value in values.items():
                np.testing.assert_allclose(value, source_reset[group][item][key], atol=1e-10, rtol=0)
    # Preserve the raw reset artifact; clarify inherited disabled clock metadata
    # in a separate receipt using both target phase and actual actor encoding.
    phase = float((np.arctan2(actual[0, 0], actual[0, 1]) / (2*np.pi)) % 1)
    phase_from_target = (reset["joints"][JOINTS[0]]["target_phase"] + .465) % 1
    np.testing.assert_allclose(phase, phase_from_target, atol=1e-7, rtol=0)
    result = {
        "input_hashes_verified": len(source_hashes), "samples": len(rows),
        "six_right_inputs_max_abs_error": float(abs(actual[:, channels]-expected[:, channels]).max()),
        "reward_normalization": errors, "reset_physical_state_and_targets_match_historical": True,
        "reset_clock_clarification": {"raw_metadata_gait_phase": reset["gait_phase"],
            "effective_actor_right_phase": phase, "phase_from_knee_target_plus_shift": phase_from_target,
            "explanation": "Raw reset metadata reports the disabled inherited clock. The diagnostic wrapper presents the verified RIGHT clock in actor slots 0/1."},
        "closed_loop_feature_differences": divergence,
        "caveat": "State differences alone do not isolate their cause: changed runtime/FSM and floating-point permutation effects remain possible.",
    }
    (HERE / "artifact_verification.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "closed_loop_feature_differences"}, indent=2))


if __name__ == "__main__":
    main()
