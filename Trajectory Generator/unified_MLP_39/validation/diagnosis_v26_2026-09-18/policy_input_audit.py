"""Offline policy audit; reads existing traces/checkpoints, never runs OpenSim."""
from pathlib import Path
import hashlib
import json
import sys

HERE = Path(__file__).resolve().parent
TRAJ = HERE.parents[2]
sys.path.insert(0, str(TRAJ))
from unified_MLP_39._bootstrap import ensure_sim_paths
ensure_sim_paths()
from baseline_MLP import win_runtime  # noqa: F401,E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from scipy.special import ndtr  # noqa: E402
from ray.rllib.core.rl_module.rl_module import RLModule  # noqa: E402


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rms(values, axis=0):
    return np.sqrt(np.mean(np.asarray(values) ** 2, axis=axis)).tolist()


def audit(label, rollout):
    trace_path = rollout / "rollout_policy_trace.json"
    summary = json.loads((rollout / "rollout_summary.json").read_text())
    trace = json.loads(trace_path.read_text())
    checkpoint = Path(summary["checkpoint"])
    module = RLModule.from_checkpoint(str(checkpoint))
    module.eval()
    names = summary["actor_feature_names"][:module._n_actor]
    obs = np.asarray([row["actor_observation_vector_before"][:module._n_actor] for row in trace], dtype=np.float32)
    raw = np.asarray([row["raw_policy_action"] for row in trace], dtype=np.float32)
    teacher = [i for i, name in enumerate(names) if "imitation_target" in name or name.startswith("prescribed_left_")]
    clock = [names.index("gait_phase_sin"), names.index("gait_phase_cos")]
    teacher_vel = [i for i in teacher if "vel" in names[i] or "qdot" in names[i]]

    def forward(array):
        with torch.no_grad():
            return module.forward_inference({"obs": torch.as_tensor(array, dtype=torch.float32)})["action_dist_inputs"].numpy()

    logits = forward(obs)
    mean = logits[:, :2]
    policy_std = np.exp(logits[:, 2:])
    mutations = {}
    for name, indices, kind in [
        ("teacher_zero", teacher, "zero"),
        ("teacher_mean", teacher, "mean"),
        ("teacher_previous_250ms", teacher, "shift25"),
        ("teacher_next_250ms", teacher, "shift-25"),
        ("teacher_velocity_zero", teacher_vel, "zero"),
        ("clock_zero", clock, "zero"),
        ("clock_previous_250ms", clock, "shift25"),
        ("teacher_and_clock_zero", teacher + clock, "zero"),
    ]:
        changed = obs.copy()
        if kind == "zero":
            changed[:, indices] = 0.0
        elif kind == "mean":
            changed[:, indices] = obs[:, indices].mean(axis=0)
        else:
            changed[:, indices] = np.roll(obs[:, indices], int(kind[5:]), axis=0)
        new = forward(changed)[:, :2]
        delta = new - mean
        mutations[name] = {
            "action_mean_delta_rms": rms(delta),
            "action_mean_delta_abs_p95": np.percentile(np.abs(delta), 95, axis=0).tolist(),
            "action_mean_delta_abs_max": np.abs(delta).max(axis=0).tolist(),
        }

    first = module.pi_encoder[0]
    with torch.no_grad():
        z = first(torch.as_tensor(obs)).numpy()
        weight = first.weight.numpy()
        projections = {}
        for group, indices in [("teacher", teacher), ("clock", clock), ("all_inputs", list(range(module._n_actor)))]:
            values = obs[:, indices] @ weight[:, indices].T
            centered = (obs[:, indices] - obs[:, indices].mean(axis=0)) @ weight[:, indices].T
            projections[group] = {"rms": float(np.sqrt(np.mean(values ** 2))), "centered_rms": float(np.sqrt(np.mean(centered ** 2)))}

    obs_tensor = torch.as_tensor(obs.copy()).requires_grad_(True)
    # RLlib inference wraps its forward in no_grad; use the identical policy tower
    # for local input derivatives and verify its values against the public path.
    out = module.pi(obs_tensor)[:, :2]
    assert np.allclose(out.detach().numpy(), mean, atol=1e-6)
    gradients = []
    for joint in range(2):
        gradient = torch.autograd.grad(out[:, joint].sum(), obs_tensor, retain_graph=True)[0].detach().numpy()
        gradients.append(gradient)
    gradient_array = np.stack(gradients, axis=1)
    scale_aware = np.sqrt(np.mean(gradient_array ** 2, axis=0)) * obs.std(axis=0)[None, :]

    # The n+1 actor observation must expose the target that n just evaluated.
    field_spec = [
        (teacher[0], "imitation_target_q", "pros_knee_angle", 1.0),
        (teacher[1], "imitation_target_qdot", "pros_knee_angle", 10.0 if label == "unified39" else 1.0),
        (teacher[2], "imitation_target_q", "pros_ankle_angle", 1.0),
        (teacher[3], "imitation_target_qdot", "pros_ankle_angle", 10.0 if label == "unified39" else 1.0),
    ]
    wiring = {}
    for index, field, joint, divisor in field_spec:
        expected = np.asarray([row[field][joint] / divisor for row in trace[:-1]], dtype=np.float32)
        actual = obs[1:, index]
        wiring[names[index]] = {"divisor": divisor, "max_absolute_error": float(np.max(np.abs(actual - expected))), "equal_float32": bool(np.array_equal(actual, expected)), "within_float32_tolerance_1e_7": bool(np.allclose(actual, expected, rtol=0.0, atol=1e-7))}

    slew_probability = None
    if label == "unified39":
        # Production absolute map: q = midpoint + half_span * clip(action,-1,1).
        # Slew anchors previous policy endpoint, not the governor's served state.
        half_span = np.asarray([.75, .7])
        midpoint = np.asarray([-.75, 0.])
        rate = np.asarray([2.5, 2.])
        endpoints = np.asarray([row["policy_segment_values"][-1] for row in trace])
        anchors = np.vstack((np.asarray(trace[0]["policy_segment_values"][0]), endpoints[:-1]))
        dt = np.asarray([row["policy_segment_times"][-1] - row["policy_segment_times"][0] for row in trace])[:, None]
        allowed_q_low = anchors - rate * dt
        allowed_q_high = anchors + rate * dt
        action_low = (allowed_q_low - midpoint) / half_span
        action_high = (allowed_q_high - midpoint) / half_span
        # All visited anchors are strictly inside absolute clipping bounds.
        assert np.all(action_low > -1.) and np.all(action_high < 1.)
        no_slew_probability = ndtr((action_high - mean) / policy_std) - ndtr((action_low - mean) / policy_std)
        desired_q = midpoint + half_span * np.clip(raw, -1., 1.)
        reconstructed_endpoint = np.maximum(allowed_q_low, np.minimum(allowed_q_high, desired_q))
        assert np.allclose(reconstructed_endpoint, endpoints, rtol=0., atol=1e-7)
        mean_slew_limited = (desired_q < allowed_q_low - 1e-10) | (desired_q > allowed_q_high + 1e-10)
        slew_probability = {
            "scope": "Gaussian probabilities conditional on deterministic rollout observations; not sampled training frequencies.",
            "absolute_half_spans_rad": half_span.tolist(),
            "slew_rates_rad_s": rate.tolist(),
            "allowed_action_half_width_mean": (rate * dt / half_span).mean(axis=0).tolist(),
            "probability_no_slew_clip_mean": no_slew_probability.mean(axis=0).tolist(),
            "probability_slew_clip_mean": (1. - no_slew_probability).mean(axis=0).tolist(),
            "probability_both_actions_no_slew_clip_mean": float(np.mean(np.prod(no_slew_probability, axis=1))),
            "probability_no_slew_clip_min_max": np.stack((no_slew_probability.min(axis=0), no_slew_probability.max(axis=0))).T.tolist(),
            "deterministic_mean_action_slew_clip_fraction": mean_slew_limited.mean(axis=0).tolist(),
            "trace_endpoint_reconstruction_max_abs_error_rad": float(np.max(np.abs(reconstructed_endpoint - endpoints))),
        }

    return {
        "checkpoint": str(checkpoint),
        "rollout": str(rollout),
        "checkpoint_sha256": digest(checkpoint / "module_state.pkl"),
        "trace_sha256": digest(trace_path),
        "actor_width_checkpoint": module._n_actor,
        "actor_width_rollout_summary": summary["n_actor"],
        "critic_width_checkpoint": module._n_full,
        "actor_feature_names": names,
        "teacher_indices": teacher,
        "teacher_mean_std_min_max": {names[i]: [float(obs[:, i].mean()), float(obs[:, i].std()), float(obs[:, i].min()), float(obs[:, i].max())] for i in teacher + clock},
        "deterministic_trace_reproduction_max_abs_error": float(np.max(np.abs(mean - raw))),
        "action_mean_mean_std_min_max": np.stack([mean.mean(axis=0), mean.std(axis=0), mean.min(axis=0), mean.max(axis=0)]).T.tolist(),
        "policy_std_mean_min_max": np.stack([policy_std.mean(axis=0), policy_std.min(axis=0), policy_std.max(axis=0)]).T.tolist(),
        "first_tanh_fraction_abs_over_0_99": float(np.mean(np.abs(np.tanh(z)) > .99)),
        "first_layer_input_projection": projections,
        "sensitivity_single_step_frozen_other_inputs": mutations,
        "local_derivative_rms_times_observed_feature_std": {name: scale_aware[:, i].tolist() for i, name in enumerate(names)},
        "teacher_pre_step_next_equals_previous_post_step": wiring,
        "conditional_exploration_slew_probability": slew_probability,
    }


def main():
    results = {
        "scope": "Offline checkpoint forward passes on archived rollout observations; no simulation or training.",
        "caveats": [
            "Teacher/clock interventions freeze other inputs, so observations may be outside the training distribution.",
            "Sensitivity proves dependence on an input, not that it is used correctly or causally explains training failure.",
            "The 250ms shifts wrap at trace endpoints; they are input sensitivity probes, not counterfactual physical rollouts.",
            "The archived rollouts have different starts and targets; policy sensitivities are descriptive, not controlled architecture ablations.",
        ],
        "v26": audit("v26", TRAJ / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout"),
        "unified39": audit("unified39", TRAJ / "unified_MLP_39/runs/rollout/MLP39_left_phase_best_iter54_center_2026-09-18"),
    }
    output = HERE / "policy_input_audit.json"
    output.write_text(json.dumps(results, indent=2) + "\n")
    for label in ("v26", "unified39"):
        result = results[label]
        print(label, json.dumps({k: v for k, v in result.items() if k not in {"actor_feature_names", "local_derivative_rms_times_observed_feature_std"}}, indent=2))
    print("OUTPUT", output)


if __name__ == "__main__":
    main()
