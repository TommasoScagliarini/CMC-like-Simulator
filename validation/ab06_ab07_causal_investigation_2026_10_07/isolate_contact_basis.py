"""Algebraic ablation of the empirical contact basis on frozen saved poses.

No integration, native model loading, profile edits or refitting. The existing
verified replay stores each sphere's impulse. Removing one contribution at a
fixed trajectory is exact for this additive vertical force law. It is not a
prediction for a dynamically rerun ablated model.
"""
from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
AUDIT = Path(__file__).resolve().parent


def fingerprint(path):
    return {"path": path.relative_to(ROOT).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    replay_path = ROOT / "validation/ab06_ab07_preparation_vs_model_2026_10_07/contact_replay.json"
    profile_path = ROOT / "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json"
    before = [fingerprint(path) for path in (replay_path, profile_path)]
    replay = json.loads(replay_path.read_text(encoding="utf-8"))
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    assert replay["status"] == "PASS"
    assert profile["ground"]["normal"] == [0.0, 1.0, 0.0]
    left = [sphere for sphere in profile["spheres"] if sphere["side"] == "left"]
    assert all(sphere["residual_force_ratio"][1] == 0.0 for sphere in left)
    assert all(sphere.get("residual_force_penetration_gain_per_m", [0, 0, 0])[1] == 0
               and sphere.get("residual_force_penetration_rate_gain_s_per_m", [0, 0, 0])[1] == 0
               for sphere in left)
    cases = {}
    for case in ("raw_IK", "filtered_IK", "actual"):
        subjects = {}
        for subject in ("AB06", "AB07"):
            original = replay["subjects"][subject]["cases"][case]["aggregate"]
            contributions = original["per_sphere_impulse_ns"]
            assert abs(sum(contributions.values()) - original["impulse_ns"]) < 1e-8
            duration = original["duration_s"]
            basis = contributions["left_basis_07"]
            other = original["impulse_ns"] - basis
            subjects[subject] = {
                "duration_s": duration,
                "all_spheres_impulse_ns": original["impulse_ns"],
                "basis07_impulse_ns": basis, "other_spheres_impulse_ns": other,
                "all_spheres_mean_force_n": original["impulse_ns"] / duration,
                "basis07_mean_force_n": basis / duration,
                "other_spheres_mean_force_n": other / duration,
                "own_BW_n": replay["subjects"][subject]["BW_n"],
                "other_spheres_mean_force_bw": other / duration / replay["subjects"][subject]["BW_n"],
                "other_spheres_over_own_measured_impulse_ratio": other / original["measured_impulse_ns"],
            }
        first, second = subjects["AB06"], subjects["AB07"]
        gap = second["all_spheres_mean_force_n"] - first["all_spheres_mean_force_n"]
        basis_gap = second["basis07_mean_force_n"] - first["basis07_mean_force_n"]
        other_gap = second["other_spheres_mean_force_n"] - first["other_spheres_mean_force_n"]
        assert abs(gap - basis_gap - other_gap) < 1e-10
        cases[case] = {
            "subjects": subjects, "AB07_minus_AB06_all_mean_force_n": gap,
            "AB07_minus_AB06_basis07_mean_force_n": basis_gap,
            "AB07_minus_AB06_other_mean_force_n": other_gap,
            "removing_basis07_reverses_absolute_gap": other_gap < 0,
        }
    after = [fingerprint(path) for path in (replay_path, profile_path)]
    assert before == after
    result = {
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "instruction_check_token": "CMC_AGENT_OK_2026", "status": "PASS",
        "scope": __doc__, "cases": cases,
        "basis07": next(sphere for sphere in left if sphere["name"] == "left_basis_07"),
        "findings": [
            "On frozen poses, the absolute AB07-minus-AB06 mean-force difference is entirely carried by basis07; other sphere contributions offset it.",
            "The scalar vertical contact law is additive; this component attribution is exact for the saved pose replay and its declared time grid.",
            "Removing the sphere does not equalize normalized load and under-supports both subjects relative to their measured references. It is a diagnostic, not a proposed profile correction.",
            "This identifies the contact representation through which the gap occurs; it does not uniquely identify anatomy, mounting or reference motion as the origin of the different sphere trajectories.",
        ],
        "limitations": [
            "No dynamics is rerun. Poses, velocities, sphere engagement, SEA controls and other forces would respond to a real force-component intervention.",
            "The replay's exact-TO grid differs slightly from the canonical paired comparator; canonical actual percentages are unchanged.",
        ],
        "inputs_integrity": {"all_unchanged": True, "before": before, "after": after},
        "helper": fingerprint(Path(__file__).resolve()),
    }
    output = AUDIT / "contact_basis_ablation.json"
    with output.open("x", encoding="utf-8") as handle:
        json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
    print(json.dumps({"output": fingerprint(output), "cases": {
        name: {key: value for key, value in item.items() if key != "subjects"}
        for name, item in cases.items()}}, indent=2))


if __name__ == "__main__":
    main()
