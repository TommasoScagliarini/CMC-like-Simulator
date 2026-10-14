"""Offline numerical-order and paired-noise audit. Never starts a simulation.

The historical and best72 artifacts are read-only. Canonical and comparison
figures are saved under the repository root plot/, with separate receipts.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
TG = PACKAGE.parent
REPO = TG.parent
os.environ.setdefault("MPLCONFIGDIR", str(HERE / "mpl_cache"))
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(key, "1")


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


previous = load_module("v26_robustness_previous_analysis", HERE.parent / "v26_weight_transfer_2026-09-23/analyze.py")
helpers = previous.helpers
np, plt, torch, plotter = previous.np, previous.plt, helpers.torch, previous.plotter
torch.set_num_threads(1)
read, dump = helpers.read, helpers.dump
ROOT = PACKAGE / "runs/rollout/V26_robustness_2026-09-23"
ORIGINAL = ROOT / "deterministic_original_order"
CONVERTED = previous.DEFAULT_NEW
HISTORICAL = previous.OLD
BEST72_ROOT = previous.BEST72.parent
FIGURES = REPO / "plot/09_23_2026_v26_robustness_comparison"
JOINTS = tuple(helpers.JOINTS)
LABELS = {"historical": "V26 agosto", "permuted_weights": "39D: colonne pesi riordinate",
          "original_order": "39D: ordine calcoli originale", "v26": "V26 trasferita RIGHT",
          "best72": "Best72 LEFT"}


def valid_physical_rows(trace):
    """A caught hard failure may expose a partial state at the previous time."""
    for index in range(1, len(trace)):
        if trace[index]["time"] <= trace[index - 1]["time"] + 1e-9:
            if index != len(trace) - 1:
                raise ValueError("Non-increasing time before the final partial terminal row")
            return trace[:index]
    return trace


def first_above(values, threshold):
    indices = np.flatnonzero(np.asarray(values) > threshold)
    return int(indices[0] + 1) if len(indices) else None


def compare_traces(a, b, *, historical_b=False):
    a, b = valid_physical_rows(a), valid_physical_rows(b)
    n = min(len(a), len(b))
    a, b = a[:n], b[:n]
    np.testing.assert_allclose([r["time"] for r in a], [r["time"] for r in b], atol=2e-8, rtol=0)
    result = {"common_complete_steps": n, "joints": {}}
    for joint in JOINTS:
        for key in ("imitation_target_q", "imitation_target_qdot"):
            np.testing.assert_allclose([r[key][joint] for r in a], [r[key][joint] for r in b], atol=2e-8, rtol=0)
        qa = np.degrees([r["prosthetic_state"][joint] for r in a])
        qb = np.degrees([r["prosthetic_state"][joint] for r in b])
        result["joints"][joint] = dict(previous.tensor_error(qa, qb),
            first_difference_above_0_001deg=first_above(abs(qa - qb), .001),
            first_difference_above_0_1deg=first_above(abs(qa - qb), .1),
            first_difference_above_1deg=first_above(abs(qa - qb), 1))
    aa = np.asarray([r["raw_policy_action"] for r in a])
    ab = np.asarray([r["raw_policy_action"] for r in b])
    delta = np.max(abs(aa - ab), axis=1)
    result["raw_action"] = dict(previous.tensor_error(aa, ab),
        first_bitwise_difference=first_above(delta, 0),
        first_difference_above_1e_6=first_above(delta, 1e-6),
        first_difference_above_0_001=first_above(delta, .001))
    xa = np.asarray([r["actor_observation_vector_before"] for r in a])
    xb = np.asarray([r["actor_observation_vector_before"] for r in b])
    if historical_b:
        xb = xb[:, previous.ACTOR_PERMUTATION]
    result["actor_observations"] = previous.tensor_error(xa, xb)
    result["initial_actor_observation_max_abs_difference"] = float(abs(xa[0] - xb[0]).max())
    # Compare stable actor-visible state features rather than schema-dependent
    # debug fields introduced between FSM versions.
    names = list(a[0]["actor_observation_before"])
    indexes = [i for i, name in enumerate(names) if name.startswith(("phase_fsm_", "phase_expected_"))]
    result["first_actor_fsm_state_difference"] = first_above(np.max(abs(xa[:, indexes] - xb[:, indexes]), axis=1), 0)
    result["scope"] = "Closed-loop trajectories; equality on identical inputs is audited separately."
    return result


def render_canonical(folder, summary):
    receipt_path = HERE / "canonical_original_order_receipt.json"
    if receipt_path.exists():
        receipt = read(receipt_path)
        if receipt["rollout"] != str(folder) or receipt["trace_sha256"] != previous.file_hash(folder / "rollout_policy_trace.json"):
            raise ValueError("Existing canonical receipt refers to a different rollout")
        for path in receipt["figures"]:
            if not Path(path).is_file():
                raise FileNotFoundError(path)
        return receipt
    context = plotter.valid_mlp_rollout_context(folder)
    if context is None:
        raise ValueError("Canonical plotting requires a valid completed rollout")
    from matplotlib.text import Text
    saved, labels = [], []
    original_save, original_discovery, argv = plotter.save_figure, plotter.latest_mlp_rollout_context, sys.argv

    def save(fig, out_dir, filename):
        if filename.startswith("07_"):
            for text_object in fig.findobj(match=Text):
                text = text_object.get_text().replace("Shifted Sound Leg Target", "Historical Right Phase Target")
                text = text.replace("shifted sound leg target", "historical right phase target")
                text = text.replace("shifted sound leg - policy C2", "right phase target - policy C2")
                text_object.set_text(text)
                if text:
                    labels.append(text)
        original_save(fig, out_dir, filename)
        saved.append(str(out_dir / filename))

    try:
        plotter.latest_mlp_rollout_context = lambda: context
        plotter.save_figure = save
        sys.argv = [str(REPO / "plot/plotter.py"), "--mlp", "--results-dir", str(context.sim_outputs_dir),
                    "--prefix", "rollout_episode", "--setup", str(summary["setup_xml_path"]),
                    "--gait-side", "left", "--out-root", str(REPO / "plot")]
        plotter.main()
    finally:
        sys.argv = argv
        plotter.save_figure, plotter.latest_mlp_rollout_context = original_save, original_discovery
    if len(saved) != 7:
        raise RuntimeError("Expected seven canonical plots")
    for path in saved:
        Path(path).relative_to(REPO / "plot")
        if not Path(path).is_file():
            raise FileNotFoundError(path)
    receipt = {"rollout": str(folder), "checkpoint": summary["checkpoint"],
        "trace_sha256": previous.file_hash(folder / "rollout_policy_trace.json"),
        "inference": "Historical network arithmetic; unified39 inputs inverse-permuted",
        "target": "Recorded prescribed RIGHT phase template, shifts knee 0.465 / ankle 0.452",
        "fsm": "v3", "figures": saved, "figure7_labels": labels}
    dump(receipt_path, receipt)
    dump(Path(saved[0]).parent / "plot_receipt.json", receipt)
    return receipt


def deterministic_analysis():
    metrics, traces, physical = {}, {}, {}
    folders = {"historical": HISTORICAL, "permuted_weights": CONVERTED, "original_order": ORIGINAL}
    for label, folder in folders.items():
        metric, trace, summary = helpers.rollout_metrics(folder, historical=label == "historical")
        extra, raw = previous.physical_metrics(folder, trace)
        metrics[label], traces[label], physical[label] = dict(metric, **extra), trace, raw
    result = {"rollouts": metrics,
        "original_order_vs_historical": compare_traces(traces["original_order"], traces["historical"], historical_b=True),
        "permuted_weights_vs_historical": compare_traces(traces["permuted_weights"], traces["historical"], historical_b=True),
        "original_order_vs_permuted_weights": compare_traces(traces["original_order"], traces["permuted_weights"]),
        "limits": ["One deterministic rollout per execution path: sensitivity evidence, not a robustness estimate.",
                   "FSM v3 is unchanged between current paths; both use identical RIGHT target, clock and reward.",
                   "Matching the initial arithmetic does not isolate every historical runtime change."]}
    FIGURES.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), layout="constrained")
    for label, trace in traces.items():
        elapsed = np.asarray([r["time"] for r in trace]) - trace[0]["policy_segment_times"][0]
        for col, (joint, short) in enumerate(zip(JOINTS, ("knee", "ankle"))):
            axes[0, col].plot(elapsed, plotter.joint_sign(short) * np.degrees([r["prosthetic_state"][joint] for r in trace]), label=LABELS[label], lw=1)
            axes[1, col].plot(physical[label]["elapsed"], physical[label]["torques"][short], label=LABELS[label], lw=.65)
        axes[2, 0].plot(elapsed, [1000 * r["reward_terms"]["grf_penetration_m"] for r in trace], label=LABELS[label])
        axes[2, 1].plot(elapsed, [r["reward_terms"].get("phase_valid_cycle_count", 0) for r in trace], label=LABELS[label])
    trace = traces["historical"]
    elapsed = np.asarray([r["time"] for r in trace]) - trace[0]["policy_segment_times"][0]
    for col, (joint, short) in enumerate(zip(JOINTS, ("knee", "ankle"))):
        axes[0, col].plot(elapsed, plotter.joint_sign(short) * np.degrees([r["imitation_target_q"][joint] for r in trace]), "--", color="black", label="Target RIGHT comune", lw=.8)
    for ax, title, ylabel in zip(axes.flat, ("Ginocchio", "Caviglia", "Coppia SEA ginocchio", "Coppia SEA caviglia", "Penetrazione registrata", "Cicli validati dalla FSM"), ("°", "°", "Nm", "Nm", "mm", "Cicli")):
        ax.set(title=title, ylabel=ylabel, xlabel="Tempo dal reset [s]")
        ax.legend(fontsize=7)
        ax.grid(alpha=.2)
    fig.suptitle("V26 nella 39D: sensibilità all'ordine dei calcoli — nessun nuovo training")
    figure = FIGURES / "deterministic_arithmetic_comparison.png"
    fig.savefig(figure, dpi=165)
    plt.close(fig)
    result["comparison_figure"] = str(figure)
    result["canonical_plots"] = render_canonical(ORIGINAL, read(ORIGINAL / "rollout_summary.json"))
    dump(HERE / "deterministic_metrics.json", result)
    return result


def rmse_rows(trace):
    return {j: helpers.rmse(helpers.array(trace, "prosthetic_state", j), helpers.array(trace, "imitation_target_q", j)) for j in JOINTS}


def sea_metrics(trace):
    """Comparable segment-level counters; no signed torque reconstructed."""
    result = {}
    for joint in JOINTS:
        count = sum(r["reward_terms"].get(joint + "_sea_sample_count", 0) for r in trace)
        rates = sum(r["reward_terms"].get(joint + "_sea_tau_spring_rate_sample_count", 0) for r in trace)
        result[joint] = {"sample_count": count, "rate_sample_count": rates}
        for out, source, divisor in (("spring_torque_rms_nm", "_sea_tau_spring_sum_squares_nm2", count),
                                    ("spring_torque_rate_rms_nm_s", "_sea_tau_spring_rate_sum_squares_nm2_s2", rates),
                                    ("torque_tracking_error_rms_nm", "_sea_torque_error_sum_squares_nm2", count)):
            result[joint][out] = math.sqrt(sum(r["reward_terms"].get(joint + source, 0) for r in trace) / divisor) if divisor else None
    return result


def prefix_action_diagnostics(trace, draws, scale):
    """State-conditional exploration on an explicitly matched time window."""
    assert len(trace) == len(draws) > 0
    learned = np.asarray([draw["learned_std"] for draw in draws], dtype=float)
    raw = np.asarray([row["raw_policy_action"] for row in trace], dtype=float)
    mean = np.asarray([row["policy_action_mean"] for row in trace], dtype=float)
    clipped = np.abs(raw) > 1
    return {
        "samples": len(trace),
        "action_order": list(JOINTS),
        "learned_std_mean_normalized": learned.mean(axis=0).tolist(),
        "effective_std_mean_normalized": (learned * scale).mean(axis=0).tolist(),
        "clipped_component_fraction": float(clipped.mean()),
        "clipped_fraction_by_action": clipped.mean(axis=0).tolist(),
        "preclip_perturbation_rms_normalized": np.sqrt(np.mean((raw - mean) ** 2, axis=0)).tolist(),
        "perturbation_definition": "Recorded raw_policy_action minus policy_action_mean, before clipping to [-1, 1].",
    }


def noise_episode(folder, policy, scale, seed):
    summary, trace = read(folder / "rollout_summary.json"), read(folder / "rollout_policy_trace.json")
    meta, draws = read(folder / "noise_diagnostic.json"), read(folder / "paired_noise_draws.json")
    assert summary["ok"] and summary["action_selection"] == "stochastic"
    assert previous.file_hash(folder / "paired_noise_draws.json") == meta["draws_sha256"]
    watchdog = read(folder / "watchdog_summary.json")
    assert watchdog["returncode"] == 0 and watchdog["timeout_reason"] is None
    assert (meta["noise_scale"], meta["noise_seed"], meta["environment_seed"]) == (scale, seed, 123)
    assert len(trace) == len(draws) == summary["steps"] > 0
    cfg = helpers.reward_function.RewardConfig.from_mapping(summary["reward_config"])
    rewards = [helpers.reward_function.compute_reward(r["reward_terms"], cfg, reference=r["policy_segment_values"]) for r in trace]
    error = abs(sum(r for r, _ in rewards) - summary["episode_return"])
    assert error < 1e-6
    generator = torch.Generator(device="cpu").manual_seed(seed)
    reconstruction_error = 0.
    for index, (row, draw) in enumerate(zip(trace, draws)):
        assert draw["step"] == row["step"] == index + 1
        z = torch.randn((1, 2), generator=generator, dtype=torch.float32).numpy().reshape(-1)
        np.testing.assert_array_equal(draw["standard_normal"], z)
        logits = torch.tensor(draw["action_dist_inputs"], dtype=torch.float32)
        mean, std = logits[:2], torch.exp(logits[2:])
        action = mean + std * torch.from_numpy(z) * scale
        np.testing.assert_array_equal(row["policy_action_mean"], mean.numpy())
        np.testing.assert_array_equal(draw["learned_std"], std.numpy())
        np.testing.assert_array_equal(row["policy_action_std"], (std * scale).numpy())
        np.testing.assert_array_equal(row["raw_policy_action"], action.numpy())
        np.testing.assert_array_equal(row["applied_policy_action"], np.clip(action.numpy(), -1, 1))
        reconstruction_error = max(reconstruction_error, float(np.max(abs(action.numpy() - np.asarray(row["raw_policy_action"])))))
    valid = valid_physical_rows(trace)
    clip = float(np.mean(np.abs(np.asarray([r["raw_policy_action"] for r in trace])) > 1))
    if "action_clipped_fraction" in summary:
        np.testing.assert_allclose(clip, summary["action_clipped_fraction"], atol=1e-12, rtol=0)
    result = {k: summary[k] for k in ("steps", "end_reason", "episode_return", "terminated", "truncated", "grf_penetration_max_m", "phase_valid_cycle_count")}
    result.update(policy=policy, noise_scale=scale, noise_seed=seed, environment_seed=123,
        completed=summary["steps"] == 500 and summary["end_reason"] == "episode_time_limit",
        complete_physical_steps=len(valid), partial_terminal_rows_excluded=len(trace) - len(valid),
        reward_reconstruction_error=error, action_reconstruction_error=reconstruction_error,
        action_clipped_fraction=clip, rmse_degrees=rmse_rows(valid), sea_diagnostics=sea_metrics(valid),
        rollout=str(folder), learned_std_mean=np.mean([r["learned_std"] for r in draws], axis=0).tolist(),
        terminal_failure=trace[-1].get("failure", summary.get("terminal_failure")),
        hard_guard_exceeded=summary["end_reason"] == "grf_penetration_hard")
    if (folder / "terminal_info.json").exists():
        terminal = read(folder / "terminal_info.json")
        assert terminal["end_reason"] == summary["end_reason"]
        result["terminal_failure"] = terminal.get("failure")
    if "completed_full_episode" in summary:
        assert summary["completed_full_episode"] == result["completed"]
    if result["hard_guard_exceeded"]:
        result["penetration_note"] = "Hard guard exceeded; saved reward maximum may omit the failing integration substep."
    # Best72 stochastic runs did not save STO. No invented signed peaks or
    # high-pass chattering estimate is inferred from absolute SEA counters.
    result["signed_torque_sto_available"] = (folder / "sim_outputs/rollout_episode_sea_torques.sto").is_file()
    return result, trace, valid, draws


def wilson(successes, n):
    z = 1.959963984540054
    p, den = successes / n, 1 + z*z/n
    center = (p + z*z/(2*n))/den
    half = z*math.sqrt(p*(1-p)/n + z*z/(4*n*n))/den
    return [center - half, center + half]


def stochastic_analysis():
    episodes, traces, valid, draws = {}, {}, {}, {}
    for policy, root in (("v26", ROOT), ("best72", BEST72_ROOT)):
        for scale in (1., .5):
            for seed in range(123, 128):
                key = policy, scale, seed
                folder = root / f"noise_{scale:g}_seed{seed}"
                episodes[key], traces[key], valid[key], draws[key] = noise_episode(folder, *key)
    common = min(len(rows) for rows in valid.values())
    for key, episode in episodes.items():
        episode["common_prefix_rmse_degrees"] = rmse_rows(valid[key][:common])
        episode["common_prefix_sea_diagnostics"] = sea_metrics(valid[key][:common])
        episode["common_prefix_action_diagnostics"] = prefix_action_diagnostics(
            valid[key][:common], draws[key][:common], key[1])
    paired = []
    for seed in range(123, 128):
        keys = [(p, s, seed) for p in ("v26", "best72") for s in (1., .5)]
        count = min(len(draws[key]) for key in keys)
        reference = [r["standard_normal"] for r in draws[keys[0]][:count]]
        assert all([r["standard_normal"] for r in draws[key][:count]] == reference for key in keys)
        for policy in ("v26", "best72"):
            a, b = (policy, 1., seed), (policy, .5, seed)
            assert traces[a][0]["actor_observation_vector_before"] == traces[b][0]["actor_observation_vector_before"]
            assert traces[a][0]["policy_action_mean"] == traces[b][0]["policy_action_mean"]
        paired.append({"noise_seed": seed, "matched_standard_normal_draws_across_both_policies_and_scales": count})
    paired_policy_comparisons = []
    for scale in (1., .5):
        for seed in range(123, 128):
            keys = {policy: (policy, scale, seed) for policy in ("v26", "best72")}
            count = min(len(valid[key]) for key in keys.values())
            np.testing.assert_allclose(
                [r["time"] for r in valid[keys["v26"]][:count]],
                [r["time"] for r in valid[keys["best72"]][:count]], atol=2e-8, rtol=0)
            pair = {"noise_scale": scale, "noise_seed": seed,
                    "common_complete_physical_steps": count,
                    "matched_nominal_duration_s": count * .01, "policies": {}}
            for policy, key in keys.items():
                episode = episodes[key]
                pair["policies"][policy] = {
                    field: episode[field] for field in (
                        "completed", "steps", "complete_physical_steps", "end_reason",
                        "partial_terminal_rows_excluded", "hard_guard_exceeded")}
                pair["policies"][policy].update(
                    paired_prefix_rmse_degrees=rmse_rows(valid[key][:count]),
                    paired_prefix_sea_diagnostics=sea_metrics(valid[key][:count]),
                    paired_prefix_action_diagnostics=prefix_action_diagnostics(
                        valid[key][:count], draws[key][:count], scale))
            pair["rmse_best72_minus_v26_degrees"] = {
                joint: pair["policies"]["best72"]["paired_prefix_rmse_degrees"][joint]
                       - pair["policies"]["v26"]["paired_prefix_rmse_degrees"][joint]
                for joint in JOINTS}
            paired_policy_comparisons.append(pair)
    groups = []
    for policy in ("v26", "best72"):
        for scale in (1., .5):
            group = [e for e in episodes.values() if e["policy"] == policy and e["noise_scale"] == scale]
            successes = sum(e["completed"] for e in group)
            pooled_trace = [row for seed in range(123, 128) for row in valid[(policy, scale, seed)][:common]]
            pooled_draws = [row for seed in range(123, 128) for row in draws[(policy, scale, seed)][:common]]
            groups.append({"policy": policy, "noise_scale": scale, "episodes": len(group), "complete": successes,
                "completion_fraction": successes / len(group), "wilson95": wilson(successes, len(group)),
                "mean_steps": float(np.mean([e["steps"] for e in group])),
                "mean_return": float(np.mean([e["episode_return"] for e in group])),
                "end_reasons": {reason: sum(e["end_reason"] == reason for e in group) for reason in sorted({e["end_reason"] for e in group})},
                "common_prefix_mean_rmse_degrees": {j: float(np.mean([e["common_prefix_rmse_degrees"][j] for e in group])) for j in JOINTS},
                "common_prefix_action_diagnostics": prefix_action_diagnostics(pooled_trace, pooled_draws, scale),
                "clipped_component_fraction_step_weighted": sum(e["action_clipped_fraction"]*e["steps"] for e in group) / sum(e["steps"] for e in group)})
    result = {"groups": groups, "episodes": list(episodes.values()), "common_prefix_steps": common,
        "paired_noise_checks": paired,
        "paired_policy_comparisons": paired_policy_comparisons,
        "paired_policy_comparison_scope": "Each pair uses its own shared physical horizon and identical Gaussian draws, preserving later samples when other episodes fail earlier. RMSE is measured against each policy's own RIGHT or LEFT target. Differences are descriptive and do not isolate a causal effect of target, policy weights, or arithmetic order.",
        "limits": ["Five paired noise sequences at one fixed start: exploratory screening, not statistical equivalence or independent training replicas.",
            "The V26 RIGHT policy and best72 LEFT policy use different target curves and learned distributions; the comparison does not isolate the target alone.",
            "Sigma scaling changes both noise variance and the mean action after clipping; learned std depends on the visited state.",
            "Non-advancing partial terminal rows are excluded from physical RMSE and SEA metrics, but kept for reward reconstruction and failure counts.",
            "Common-prefix metrics exclude later portions of the gait; full-episode RMSE has different horizons when an episode terminates early.",
            "The saved penetration maximum can understate a caught hard-limit failure; end_reason is retained explicitly.",
            "Best72 stochastic STO files are absent: comparable torque metrics use saved SEA sums and counts, not signed peaks."]}
    FIGURES.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout="constrained", sharex=True, sharey=True)
    for (policy, scale, seed), trace in valid.items():
        ax = axes[0 if scale == 1 else 1, 0 if policy == "v26" else 1]
        elapsed = np.asarray([r["time"] for r in trace]) - trace[0]["policy_segment_times"][0]
        ax.plot(elapsed, [1000*r["reward_terms"]["grf_penetration_m"] for r in trace], label=f"Seed {seed}", lw=1)
    for i, scale in enumerate((1., .5)):
        for j, policy in enumerate(("v26", "best72")):
            ax = axes[i, j]
            group = next(g for g in groups if g["policy"] == policy and g["noise_scale"] == scale)
            ax.axhline(28, color="gray", ls="--", lw=1, label="Soglia terminale")
            ax.set(title=f"{LABELS[policy]} — σ × {scale:g}: {group['complete']}/5 completi",
                   xlabel="Tempo dal reset [s]", ylabel="Penetrazione registrata [mm]")
            ax.grid(alpha=.2)
            ax.legend(fontsize=7, ncol=2)
    fig.suptitle("Screening appaiato: stessi campioni gaussiani, seed ambiente 123\nI terminali parziali non mostrano il picco hard effettivo")
    figure = FIGURES / "paired_noise_penetration.png"
    fig.savefig(figure, dpi=165)
    plt.close(fig)
    result["comparison_figure"] = str(figure)
    dump(HERE / "stochastic_metrics.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--deterministic-only", action="store_true")
    group.add_argument("--all", action="store_true")
    args = parser.parse_args()
    result = {"deterministic": deterministic_analysis()}
    if args.all:
        result["stochastic"] = stochastic_analysis()
    dump(HERE / ("all_metrics.json" if args.all else "intermediate_metrics.json"), result)
    dump(HERE / "analysis_input_hashes.json", helpers.INPUT_HASHES)
    print(json.dumps({"deterministic_metrics": str(HERE / "deterministic_metrics.json"),
        "canonical_plots": result["deterministic"]["canonical_plots"]["figures"],
        "stochastic_groups": result.get("stochastic", {}).get("groups")}, indent=2), flush=True)


if __name__ == "__main__":
    main()
