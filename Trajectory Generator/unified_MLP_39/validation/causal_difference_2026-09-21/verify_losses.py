"""Recompute imitation losses from recorded physical state and target, offline."""
from pathlib import Path
import hashlib
import json
import numpy as np

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
TG = PACKAGE.parent
PATHS = {
    "iter25": PACKAGE / "runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25",
    "best29": PACKAGE / "runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_best29",
    "last40": PACKAGE / "runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40",
    "v26_best87": TG / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout",
}


def main():
    result = {}
    for name, folder in PATHS.items():
        path = folder / "rollout_policy_trace.json"
        trace = json.loads(path.read_text())
        differences = []
        for row in trace:
            state, terms = row["prosthetic_state"], row["reward_terms"]
            totals = {"sound_imitation_loss": 0., "served_imitation_loss": 0.}
            for joint in ("pros_knee_angle", "pros_ankle_angle"):
                for served, total in ((False, "sound_imitation_loss"), (True, "served_imitation_loss")):
                    loss_prefix = joint + ("_served_imitation_" if served else "_imitation_")
                    state_prefix = joint + ("_served_ref" if served else "")
                    for kind, target_field, suffix, units in (
                        ("position", "imitation_target_q", "", "rad"),
                        ("velocity", "imitation_target_qdot", "_vel", "rad_s"),
                    ):
                        target = row[target_field][joint]
                        logged_target = terms[joint + "_imitation_target_" + ("q" if kind == "position" else "qdot")]
                        differences.append(abs(target - logged_target))
                        error = target - state[state_prefix + suffix]
                        scale = terms[joint + "_imitation_" + kind + "_range_" + units]
                        loss = min(25., (error / max(1e-9, scale)) ** 2)
                        differences.append(abs(loss - terms[loss_prefix + kind + "_loss"]))
                        totals[total] += terms[joint + "_imitation_" + kind + "_weight"] * loss
            differences.extend(abs(value - terms[key]) for key, value in totals.items())
        max_error = float(max(differences))
        # Physical states in the policy trace come from float32 observations;
        # the environment computes the original loss from double state values.
        assert max_error < 1e-6, (name, max_error)
        result[name] = {"steps": len(trace), "max_absolute_error": max_error,
                        "tolerance_due_to_float32_observations": 1e-6,
                        "trace_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    (HERE / "loss_reconstruction.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
