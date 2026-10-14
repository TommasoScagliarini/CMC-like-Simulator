"""Replay only the recorded policy-to-reference command path, without OpenSim.

No policy is re-evaluated and no physical rollout is performed. The unmodified
ProstheticSegmentKinematics class is extracted from the current source by AST.
The recorded-command replay must match the saved reference before interpreting
the alternative inputs. Teacher endpoint playback is an offline oracle, not a
deployable controller or evidence of physical stability.
"""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.interpolate import BPoly, CubicHermiteSpline, PchipInterpolator


TG = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
COORDS = ("pros_knee_angle", "pros_ankle_angle")
SLEW_LIMITS = np.array([2.5, 2.0])
SOURCE = TG / "osim_trj_cmc_like.py"
ROLLOUTS = {
    "unified39_best54": TG / "unified_MLP_39/runs/rollout/MLP39_left_phase_best_iter54_center_2026-09-18",
    "v26_best87": TG / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout",
}
TRAINING_CONFIGS = {
    "unified39_best54": TG / "unified_MLP_39/runs/training/MLP39_left_phase_visible_best_to100_2026-09-18/training_cfg.resolved.yaml",
    "v26_best87": TG / "runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/training_cfg.resolved.yaml",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reference_class():
    source = SOURCE.read_text()
    node = next(
        node for node in ast.parse(source).body
        if isinstance(node, ast.ClassDef)
        and node.name == "ProstheticSegmentKinematics"
    )
    namespace = {
        "np": np,
        "BPoly": BPoly,
        "CubicHermiteSpline": CubicHermiteSpline,
        "PchipInterpolator": PchipInterpolator,
    }
    exec("from __future__ import annotations\n" + ast.get_source_segment(source, node), namespace)
    return namespace["ProstheticSegmentKinematics"]


class ResetBase:
    coord_names = list(COORDS)

    def __init__(self, reset):
        self.reset = reset

    def get(self, time):
        # Base values are used only to initialise the first segment. Later
        # anchors come from the class's carried, served reference state.
        return tuple(
            {coord: self.reset["joints"][coord][key] for coord in COORDS}
            for key in ("served_q", "served_qdot", "served_qddot")
        )


def replay(cls, rows, reset, simulation, endpoints, slew_limits):
    ref = cls(
        ResetBase(reset), COORDS,
        ref_model=simulation["pros_ref_model"],
        ref_lpf_cutoff_hz=simulation["pros_ref_cutoff_hz"],
        ref_governor_enable=simulation["pros_ref_governor"],
        ref_velocity_limits=[simulation[f"pros_{joint}_ref_velocity_limit_rad_s"] for joint in ("knee", "ankle")],
        ref_acceleration_limits=[simulation[f"pros_{joint}_ref_acceleration_limit_rad_s2"] for joint in ("knee", "ankle")],
        ref_jerk_limits=[simulation[f"pros_{joint}_ref_jerk_limit_rad_s3"] for joint in ("knee", "ankle")],
        control_dt=0.001,
    )
    previous_endpoint = np.array([reset["joints"][coord]["served_q"] for coord in COORDS])
    positions, velocities, clipping, governor = [], [], [], []
    for row, endpoint in zip(rows, endpoints):
        times = np.asarray(row["policy_segment_times"], dtype=float)
        if len(times) != 2:
            raise ValueError("This replay requires policy_knots=1")
        anchor_q, anchor_v, _ = ref.get(times[0])
        target = endpoint.copy()
        if slew_limits is not None:
            max_delta = slew_limits * (times[1] - times[0])
            target = previous_endpoint + np.clip(target - previous_endpoint, -max_delta, max_delta)
        clipping.append(np.abs(target - endpoint) > 1e-10)
        previous_endpoint = target.copy()
        values = np.vstack(([anchor_q[coord] for coord in COORDS], target))
        derivatives = np.gradient(values, times, axis=0)
        derivatives[0] = [anchor_v[coord] for coord in COORDS]
        ref.set_segment(times, values, derivatives)
        q, qdot, _ = ref.get(row["time"])
        positions.append([q[coord] for coord in COORDS])
        velocities.append([qdot[coord] for coord in COORDS])
        governor.append(ref.reference_governor_diagnostics)
    return np.asarray(positions), np.asarray(velocities), np.asarray(clipping), governor


def metrics(positions, velocities, clipping, target, governor):
    per_joint = {}
    for index, coord in enumerate(COORDS):
        per_joint[coord] = {
            "served_to_target_rmse_deg": float(np.rad2deg(np.sqrt(np.mean((positions[:, index] - target[:, index]) ** 2)))),
            "served_min_deg": float(np.rad2deg(np.min(positions[:, index]))),
            "served_max_deg": float(np.rad2deg(np.max(positions[:, index]))),
            "served_excursion_deg": float(np.rad2deg(np.ptp(positions[:, index]))),
            "served_velocity_abs_max_rad_s": float(np.max(np.abs(velocities[:, index]))),
            "endpoint_slew_clipped_fraction": float(np.mean(clipping[:, index])),
        }
    return {
        "joints": per_joint,
        "governor_mean_limit_fraction": {
            key: float(np.mean([row[key] for row in governor]))
            for key in ("velocity_limit_fraction", "acceleration_limit_fraction", "jerk_limit_fraction")
        },
    }


def main():
    cls = reference_class()
    result = {
        "method": "Offline replay of recorded commands through the source reference filter; no policy/environment/physical rollout",
        "limitations": [
            "Alternative commands do not update physical state or the recorded policy observations.",
            "Teacher playback uses the known target at each segment endpoint (10 ms horizon); it is an oracle command experiment.",
            "Target-vs-slew exceedance refers to endpoint rates, not a strict speed bound on the filtered reference.",
            "The control interval is 1 ms; equivalence with recorded references is checked numerically.",
        ],
        "source_sha256": digest(SOURCE),
        "script_sha256": digest(Path(__file__)),
        "runs": {},
    }
    for name, folder in ROLLOUTS.items():
        trace_path = folder / "rollout_policy_trace.json"
        reset_path = folder / "rollout_reset_diagnostics.json"
        rows = json.loads(trace_path.read_text())
        reset = json.loads(reset_path.read_text())
        cfg = yaml.safe_load(TRAINING_CONFIGS[name].read_text())["simulation"]
        target = np.array([[row["imitation_target_q"][coord] for coord in COORDS] for row in rows])
        target_velocity = np.array([[row["imitation_target_qdot"][coord] for coord in COORDS] for row in rows])
        recorded_ref = np.array([[row["prosthetic_state"][coord + "_served_ref"] for coord in COORDS] for row in rows])
        # Absolute actor bounds are [-1.5, 0] rad and [-0.7, 0.7] rad.
        # Use applied actions where saved, otherwise the same environment clip.
        actions = np.array([row.get("applied_policy_action", np.clip(row["raw_policy_action"], -1, 1)) for row in rows])
        endpoints = actions * np.array([0.75, 0.7]) + np.array([-0.75, 0.0])
        recorded_slew = np.array([cfg[f"pros_{joint}_target_slew_rate_limit_rad_s"] for joint in ("knee", "ankle")])
        recorded_slew = recorded_slew if np.all(recorded_slew > 0) else None
        scenarios = {
            "recorded_commands_recorded_slew": (endpoints, recorded_slew),
            "recorded_commands_slew_disabled": (endpoints, None),
            "recorded_commands_slew_2p5_2p0": (endpoints, SLEW_LIMITS),
            "oracle_teacher_endpoints_slew_disabled": (target, None),
            "oracle_teacher_endpoints_slew_2p5_2p0": (target, SLEW_LIMITS),
        }
        run = {
            "trace": str(trace_path.relative_to(TG)),
            "trace_sha256": digest(trace_path),
            "reset_sha256": digest(reset_path),
            "training_config_sha256": digest(TRAINING_CONFIGS[name]),
            "steps": len(rows),
            "target_rate_above_proposed_slew_fraction": dict(zip(COORDS, np.mean(np.abs(target_velocity) > SLEW_LIMITS, axis=0).tolist())),
            "target_velocity_abs_max_rad_s": dict(zip(COORDS, np.max(np.abs(target_velocity), axis=0).tolist())),
            "scenarios": {},
        }
        for scenario, (commands, limits) in scenarios.items():
            positions, velocities, clipping, governor = replay(cls, rows, reset, cfg, commands, limits)
            run["scenarios"][scenario] = metrics(positions, velocities, clipping, target, governor)
            if scenario == "recorded_commands_recorded_slew":
                error = np.max(np.abs(positions - recorded_ref), axis=0)
                run["recorded_replay_max_abs_error_rad"] = dict(zip(COORDS, error.tolist()))
                if np.max(error) > 2e-7:
                    raise AssertionError(f"Recorded replay mismatch: {name}: {error}")
        result["runs"][name] = run
    result["recorded_replay_validation_pass"] = True
    (HERE / "filter_replay.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"recorded_replay_validation_pass": True, "output": str(HERE / "filter_replay.json")}, indent=2))


if __name__ == "__main__":
    main()
