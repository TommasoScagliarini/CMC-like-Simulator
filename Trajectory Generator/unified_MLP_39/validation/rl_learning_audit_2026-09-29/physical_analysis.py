"""Descriptive before/after audit of one PPO batch; no simulation or new gates.

API: python physical_analysis.py --manifest manifest.json [--output-dir directory]
Manifest schema: {"cases": [case_dict_or_path_to_case_json, ...]}.
Case dictionaries are the evaluator inputs (checkpoint/output_dir/role/scenario/
noise_seed). Missing final receipts remain pending and are never ranked.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / "AGENTS.md").is_file())
PRIOR = HERE.parent / "general_method_diagnosis_2026-09-29"
os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".matplotlib"))
sys.dont_write_bytecode = True


def helper(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


helper("audit_evaluation_helpers", HERE / "evaluate.py").prepare_plot_cache()
q = helper("audit_quality_helpers", PRIOR / "quality/analyze.py")
p = helper("audit_physical_helpers", PRIOR / "physical/analyze.py")
import numpy as np

HASHES = {}
WINDOW = (13.15, 17.95)


def read(path):
    path = Path(path).resolve()
    raw = path.read_bytes()
    HASHES[str(path)] = hashlib.sha256(raw).hexdigest()
    return json.loads(raw)


def write(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def table(path):
    result = p.table(path)
    HASHES[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def load(case):
    folder = Path(case["output_dir"]).resolve()
    name = folder.name
    status = {"name": name, "role": case["role"], "scenario": case["scenario"],
              "noise_seed": case.get("noise_seed", 123), "folder": str(folder)}
    if not (folder / "evaluation_complete.json").exists():
        status["status"] = "pending_or_missing_final_receipt"
        return status, None
    receipt = read(folder / "evaluation_complete.json")
    for rel, expected in receipt["artifact_hashes"].items():
        source = folder / rel
        actual = hashlib.sha256(source.read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f"Finalized input changed: {source}")
        HASHES[str(source)] = actual
    summary = read(folder / "rollout_summary.json")
    trace = read(folder / "rollout_policy_trace.json")
    noise = read(folder / "paired_noise_receipt.json")
    assert receipt["checkpoint_unchanged"] and len(trace) == receipt["steps"] == summary["steps"]
    assert len(noise["records"]) == len(trace)
    assert noise["protocol"]["sigma_mode"] == "own"
    complete = [i for i, row in enumerate(trace) if row.get("segment_completed", True)]
    assert complete == list(range(len(complete))), "Only a terminal incomplete segment is supported"
    cfg = q.RewardConfig.from_mapping(summary["reward_config"])
    rewards = np.array([q.compute_reward(row["reward_terms"], cfg, reference=row["policy_segment_values"])[0] for row in trace])
    error = abs(float(np.sum(rewards)) - float(summary["episode_return"]))
    assert error < 1e-7, (name, error)
    full = bool(receipt["completed_full_episode"])
    status.update(status="full_episode" if full else "early_or_nonstandard_end", steps=len(trace),
                  completed_segments=len(complete), terminal_partial_rows=len(trace) - len(complete),
                  completed_full_episode=full, end_reason=summary["end_reason"],
                  episode_return=float(summary["episode_return"]), original_return_reconstruction_error=error,
                  summary_penetration_max_mm=1000 * summary["grf_penetration_max_m"],
                  action_clipped_fraction=summary.get("action_clipped_fraction"),
                  checkpoint=summary["checkpoint"], noise_protocol=noise["protocol"])
    status["summary_penetration_sampling"] = "Reward diagnostic once per policy step (nominal 100Hz), including a terminal partial-step value when present; native STO peaks are reported separately."
    transitions = folder / "critic_transitions.npz"
    if transitions.exists():
        with np.load(transitions) as transition:
            status["terminal_flags"] = {"terminated": bool(transition["terminated"][-1]),
                                        "truncated": bool(transition["truncated"][-1])}
            status["sum_recorded_transition_rewards"] = float(np.sum(transition["reward"]))
            assert abs(status["sum_recorded_transition_rewards"] - status["episode_return"]) < 1e-7
    else:
        status["terminal_flags"] = {"terminated": bool(summary.get("terminated", False)),
                                    "truncated": bool(summary.get("truncated", False))}
    status["eligible_finite_task_episode_return"] = bool(status["terminal_flags"]["terminated"] or summary["end_reason"] == "episode_time_limit")
    status["externally_censored_or_nonstandard"] = not status["eligible_finite_task_episode_return"]
    status["return_interpretation"] = "Observed undiscounted episode return includes early-termination reward/penalties. Continuing-task value calibration needs an explicit truncation/bootstrap convention."
    tables = {key: table(folder / "sim_outputs" / f"rollout_episode_{key}.sto") for key in
              ("online_grf", "reserve_torques", "sea_torques", "sea_diagnostics")}
    return status, {"name": name, "summary": summary, "trace": trace[:len(complete)],
                    "rewards": rewards[:len(complete)], "noise": noise, "tables": tables,
                    "support": [max(tab["time"][0] for tab in tables.values()), min(tab["time"][-1] for tab in tables.values())]}


def tracking(rows):
    result = {}
    for joint in ("knee", "ankle"):
        key = f"pros_{joint}_angle"
        target = np.degrees([row["imitation_target_q"][key] for row in rows])
        actual = np.degrees([row["prosthetic_state"][key] for row in rows])
        served = np.degrees([row["prosthetic_state"][key + "_served_ref"] for row in rows])
        result[joint] = {"actual_target_deg": q.matching(actual, target),
                         "served_target_deg": q.matching(served, target),
                         "actual_served_deg": q.matching(actual, served)}
    return result


def curve(time, signal, phase_time, phase, window):
    mask = (time >= window[0] - 1e-9) & (time <= window[1] + 1e-9)
    x, t = signal[mask], time[mask]
    robust_range = float(np.percentile(x, 95) - np.percentile(x, 5))
    low, _ = q.filter_split(time, signal, 10)
    _, residual = q.filter_split(time, signal, 20)
    result = {"rms_nm": q.rms(x), "min_nm": float(x.min()), "max_nm": float(x.max()),
              "positive_fraction": float(np.mean(x > 0)),
              "raw_tv_normalized_per_s": q.ratio(np.abs(np.diff(x)).sum(), robust_range * (t[-1] - t[0])),
              "lp10_tv_normalized_per_s": q.ratio(np.abs(np.diff(low[mask])).sum(), robust_range * (t[-1] - t[0])),
              "raw_minus_lp20_rms_nm": q.rms(residual[mask]),
              "raw_minus_lp20_normalized_dynamic_rms": q.ratio(q.rms(residual[mask]), np.std(x)),
              "filter_note": "Fourth-order Butterworth with zero-phase filtfilt; residual is raw minus LP20, not an ideal >20Hz spectral partition."}
    try:
        profile = q.profile(time, low, phase_time, phase, window=window)
    except AssertionError as exc:
        if str(exc) != "No complete prescribed cycle in analysis window":
            raise
        result.update(complete_cycles=0, phase_profile=None)
    else:
        result.update(complete_cycles=len(profile["intervals_s"]), phase_profile={
            "mean_nm": profile["mean"].tolist(), "std_nm": profile["std"].tolist(),
            "intervals_s": profile["intervals_s"],
            "tv_over_range": q.ratio(np.abs(np.diff(profile["mean"])).sum(), np.ptp(profile["mean"])),
            "cycle_shape_variability_normalized": q.ratio(q.rms(profile["std"]), np.std(profile["mean"]))})
    return result


def describe(data, count, window, common_end):
    rows = data["trace"][:count]
    times = np.array([row["time"] for row in rows])
    records = data["noise"]["records"][:count]
    sigma = np.array([r["learned_sigma"] for r in records])
    out = {"complete_prefix_steps": count, "prefix_return_without_terminal_partial_cost": float(np.sum(data["rewards"][:count])),
           "prefix_tracking": tracking(rows), "own_sigma_mean_knee_ankle": np.mean(sigma, axis=0).tolist(),
           "own_sigma_min_knee_ankle": np.min(sigma, axis=0).tolist(), "own_sigma_max_knee_ankle": np.max(sigma, axis=0).tolist(),
           "realized_noise_rms_knee_ankle": np.sqrt(np.mean(np.square([r["applied_noise"] for r in records]), axis=0)).tolist(),
           "analysis_window_s": list(window), "shape_available": bool(window[1] - window[0] >= .2)}
    if not out["shape_available"]:
        return out
    selected = [row for row in rows if window[0] - 1e-9 <= row["time"] <= window[1] + 1e-9]
    out["window_tracking"] = tracking(selected)
    out["window_physical"] = p.window_metrics({**data, "trace": rows}, window)
    out["torque"] = {}
    tab = data["tables"]["sea_torques"]
    support = tab["time"] <= common_end + 1e-9
    for joint in ("knee", "ankle"):
        phase = np.array([row["reward_terms"][f"pros_{joint}_angle_imitation_target_phase"] % 1 for row in rows])
        out["torque"][joint] = curve(tab["time"][support], tab[f"SEA_{joint.title()}_tau_spring"][support], times, phase, window)
    return out


def compare(before, after, statuses):
    bstatus, astatus = statuses[before["name"]], statuses[after["name"]]
    count = min(len(before["trace"]), len(after["trace"]))
    full = bstatus["completed_full_episode"] and astatus["completed_full_episode"]
    result = {"before": before["name"], "after": after["name"], "complete_common_prefix_steps": count,
              "eligible_full_return_comparison": full,
              "eligible_full_return_comparison_semantics": "Both runs cover the full physical horizon; this flag concerns matched-horizon quality, not validity of early-terminated RL episode returns.",
              "observed_episode_return_before": bstatus["episode_return"],
              "observed_episode_return_after": astatus["episode_return"],
              "observed_episode_return_after_minus_before": astatus["episode_return"] - bstatus["episode_return"],
              "both_returns_are_task_episode_samples": bstatus["eligible_finite_task_episode_return"] and astatus["eligible_finite_task_episode_return"],
              "comparison": "full_episode" if full else "matched_completed_prefix_only"}
    if not count:
        return {**result, "data": {}, "limitation": "No completed common physical segment"}
    rows = [data["trace"][:count] for data in (before, after)]
    np.testing.assert_allclose([r["time"] for r in rows[0]], [r["time"] for r in rows[1]], atol=1e-8, rtol=0)
    for joint in ("knee", "ankle"):
        key = f"pros_{joint}_angle"
        for field in ("imitation_target_q", "imitation_target_qdot"):
            np.testing.assert_allclose([r[field][key] for r in rows[0]], [r[field][key] for r in rows[1]], atol=1e-8, rtol=0)
        np.testing.assert_allclose([r["reward_terms"][key + "_imitation_target_phase"] for r in rows[0]],
                                   [r["reward_terms"][key + "_imitation_target_phase"] for r in rows[1]], atol=1e-8, rtol=0)
    assert before["summary"]["reward_config"] == after["summary"]["reward_config"], "Reward changed across physical evaluations"
    fields = ("grf_mode", "online_grf_profile_file", "online_grf_applied_sides", "prescribed_grf_disabled_sides",
              "grf_penetration_penalty_threshold_m", "grf_penetration_termination_m")
    for key in fields:
        assert before["summary"][key] == after["summary"][key], key
    bp, ap = before["noise"]["protocol"], after["noise"]["protocol"]
    for key in ("environment_seed", "noise_seed", "noise_scale", "noise_hold_steps", "sigma_mode", "scenario"):
        assert bp[key] == ap[key], key
    assert bp["action_transform"] == ap["action_transform"]
    assert before["noise"]["full_standard_normal_stream_sha256"] == after["noise"]["full_standard_normal_stream_sha256"]
    np.testing.assert_array_equal([r["standard_normal"] for r in before["noise"]["records"][:count]],
                                  [r["standard_normal"] for r in after["noise"]["records"][:count]])
    result["paired_standard_normals_same_target_same_reward_verified"] = True
    common_end = min(rows[0][-1]["time"], before["support"][1], after["support"][1])
    window = [max(WINDOW[0], before["support"][0], after["support"][0]), min(WINDOW[1], common_end - (0 if full else .15))]
    result["data"] = {data["name"]: describe(data, count, window, common_end) for data in (before, after)}
    if full:
        result["full_return_after_minus_before"] = astatus["episode_return"] - bstatus["episode_return"]
    if all(result["data"][data["name"]]["shape_available"] for data in (before, after)):
        result["torque_phase_mean_after_vs_before"] = {}
        for joint in ("knee", "ankle"):
            profiles = [result["data"][data["name"]]["torque"][joint]["phase_profile"] for data in (before, after)]
            result["torque_phase_mean_after_vs_before"][joint] = (q.matching(np.array(profiles[1]["mean_nm"]), np.array(profiles[0]["mean_nm"]))
                                                                       if all(profiles) else None)
    result["interpretation"] = "One updated checkpoint on one reset and three common-noise streams. Sigma changes are part of the current-policy effect; no attribution to mean-only improvement or PPO generalization."
    return result


def render_comparison_figures(data, statuses, groups):
    """Display recorded support only; short/failed runs never gain a future."""
    import matplotlib.pyplot as plt
    out = REPO / "plot/09_29_2026_rl_learning_audit/comparison"
    out.mkdir(parents=True, exist_ok=True)
    saved = []
    pending = [name for name, item in data.items() if item is None]
    snapshot_note = f"PARTIAL snapshot: {len(data) - len(pending)}/{len(data)} finalized receipts; pending curves omitted.\n" if pending else ""
    colors = {"before": "#4c566a", "first_pass": "#2878b5", "after": "#d45b22"}
    roles = ("before", "first_pass", "after")
    nominal = groups.get("nominal", {})
    available = {role: data[nominal[role]] for role in roles if role in nominal and data[nominal[role]] is not None}
    if available:
        fig, axes = plt.subplots(3, 2, figsize=(15, 11), sharex=True, constrained_layout=True)
        reference = available.get("before", next(iter(available.values())))
        reference_rows = [r for r in reference["trace"] if WINDOW[0] - 1e-9 <= r["time"] <= WINDOW[1] + 1e-9]
        for column, joint in enumerate(("knee", "ankle")):
            key = f"pros_{joint}_angle"
            for row_index in (0, 1):
                axes[row_index, column].plot([r["time"] for r in reference_rows],
                    np.degrees([r["imitation_target_q"][key] for r in reference_rows]),
                    color="black", linestyle="--", linewidth=1.4, label="Prescribed LEFT target")
            for role, case in available.items():
                state = statuses[case["name"]]
                rows = [r for r in case["trace"] if WINDOW[0] - 1e-9 <= r["time"] <= WINDOW[1] + 1e-9]
                label = f"{role}: {state['completed_segments']} steps, {state['end_reason']}"
                if rows:
                    time = [r["time"] for r in rows]
                    axes[0, column].plot(time, np.degrees([r["prosthetic_state"][key] for r in rows]), color=colors[role], label=label)
                    axes[1, column].plot(time, np.degrees([r["prosthetic_state"][key + "_served_ref"] for r in rows]), color=colors[role])
                tab = case["tables"]["sea_torques"]
                time, torque = tab["time"], tab[f"SEA_{joint.title()}_tau_spring"]
                mask = (time >= WINDOW[0] - 1e-9) & (time <= WINDOW[1] + 1e-9)
                if np.sum(mask) >= 2:
                    axes[2, column].plot(time[mask], torque[mask], color=colors[role], alpha=.18, linewidth=.55)
                    try:
                        low, _ = q.filter_split(time, torque, 10)
                    except ValueError:
                        # Very short signals cannot support zero-phase filtering.
                        axes[2, column].plot(time[mask], torque[mask], color=colors[role], linewidth=1.2, label=role + " (raw; too short for LP10)")
                    else:
                        axes[2, column].plot(time[mask], low[mask], color=colors[role], linewidth=1.4, label=role + " LP10")
            for row_index, title, unit in ((0, "actual", "angle (deg)"), (1, "served", "angle (deg)"), (2, "SEA spring: raw faint, LP10 solid", "torque (Nm)")):
                ax = axes[row_index, column]
                ax.set_title(f"{joint.capitalize()} {title}")
                ax.set_ylabel(unit)
                ax.grid(alpha=.22)
                ax.set_xlim(WINDOW)
            axes[0, column].legend(fontsize=8, loc="best")
            axes[2, column].set_xlabel("time (s)")
        fig.suptitle(snapshot_note + "One PPO batch — nominal deployment trajectories\nFirst pass is descriptive; each curve stops at its recorded end. LP10 is an offline display filter.", fontsize=13)
        path = out / "nominal_before_first_pass_after.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        saved.append(path)
    stochastic = {(status["role"], status["noise_seed"]): data[name] for name, status in statuses.items()
                  if status["scenario"] == "noisy" and data[name] is not None}
    if stochastic:
        seeds = (123, 125, 127)
        fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
        for role, offset in (("before", -.09), ("after", .09)):
            for i, seed in enumerate(seeds):
                case = stochastic.get((role, seed))
                if case is None:
                    continue
                status = statuses[case["name"]]
                marker = "x" if status["externally_censored_or_nonstandard"] else "o"
                x = i + offset
                args = {"color": colors[role], "marker": marker, "s": 60, "label": role if i == 0 else None}
                axes[0, 0].scatter(x, status["episode_return"], **args)
                axes[0, 1].scatter(x, status["completed_segments"], **args)
                axes[0, 1].annotate(status["end_reason"], (x, status["completed_segments"]),
                                    xytext=(2, 6 if role == "before" else -13), textcoords="offset points", fontsize=7, rotation=12)
                axes[1, 0].scatter(x, status["summary_penetration_max_mm"], **args)
                sigma = np.asarray([r["learned_sigma"] for r in case["noise"]["records"]])
                if len(sigma):
                    for joint_index, joint_marker in ((0, "o"), (1, "s")):
                        axes[1, 1].scatter(x, np.mean(sigma, axis=0)[joint_index], color=colors[role], marker=joint_marker,
                                          facecolors="none" if joint_index == 1 else colors[role], s=55,
                                          label=f"{role} {'knee' if joint_index == 0 else 'ankle'}" if i == 0 else None)
        for ax, title, ylabel in ((axes[0, 0], "Observed undiscounted return (all recorded ends)", "return"),
                                  (axes[0, 1], "Completed physical segments and end reason", "segments"),
                                  (axes[1, 0], "Maximum penetration, per-step summary", "mm"),
                                  (axes[1, 1], "Own sigma averaged over each observed episode", "normalized action std")):
            ax.set_title(title, fontsize=11)
            ax.set_ylabel(ylabel)
            ax.set_xticks(range(len(seeds)), [str(seed) for seed in seeds])
            ax.set_xlabel("paired Gaussian seed")
            ax.grid(alpha=.22)
        axes[0, 0].legend(fontsize=8)
        axes[1, 1].legend(fontsize=8)
        fig.suptitle(snapshot_note + "One PPO batch — own-sigma stochastic, RLlib training connector\nSame z by step; sigma and states may differ. x = externally censored; shorter episodes are not evidence of lower physical cost.", fontsize=12)
        path = out / "own_sigma_stochastic_outcomes.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        saved.append(path)
    receipt = {"scope": "Descriptive one-batch physical comparison; no new simulations or inferred future support.",
               "status": "partial" if pending else "all_requested_cases_finalized", "pending_cases": pending,
               "figures": {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest() for path in saved}}
    write(out / "plot_receipt.json", receipt)
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=HERE / "physical_comparison")
    args = parser.parse_args()
    manifest = read(args.manifest)
    specs = []
    for entry in manifest["cases"]:
        if isinstance(entry, str):
            path = Path(entry)
            entry = read(path if path.is_absolute() else args.manifest.parent / path)
        specs.append(entry)
    statuses, data, groups = {}, {}, {}
    for case in specs:
        status, loaded = load(case)
        name = status["name"]
        if name in statuses:
            raise ValueError(f"Duplicate case name {name}")
        statuses[name], data[name] = status, loaded
        key = "nominal" if case["scenario"] == "nominal" else f"noisy_seed{case.get('noise_seed', 123)}"
        group = groups.setdefault(key, {})
        if case["role"] in group:
            raise ValueError(f"Duplicate role in {key}")
        group[case["role"]] = name
    pairs, descriptive = {}, {}
    for group, roles in groups.items():
        if all(role in roles and data[roles[role]] is not None for role in ("before", "after")):
            pairs[group] = compare(data[roles["before"]], data[roles["after"]], statuses)
        else:
            pairs[group] = {"comparison": "pending", "available_roles": roles}
        if "first_pass" in roles and data[roles["first_pass"]] is not None:
            item = data[roles["first_pass"]]
            count = len(item["trace"])
            end = min(item["support"][1], item["trace"][-1]["time"]) if count else item["support"][0]
            window = [max(WINDOW[0], item["support"][0]), min(WINDOW[1], end)]
            descriptive[roles["first_pass"]] = {"interpretation": "Descriptive nominal first optimizer-pass snapshot, not necessarily an exact PPO epoch boundary.",
                                               "metrics": describe(item, count, window, end) if count else None}
    results = {"schema": "one_ppo_batch_physical_audit_v1", "cases": statuses, "pairs": pairs,
               "status": "partial" if any(item is None for item in data.values()) else "all_requested_cases_finalized",
               "finalized_case_count": sum(item is not None for item in data.values()),
               "kinematic_shape_definition": "centered_unit_rms_shape_error = RMS(zscore(actual)-zscore(target)) on the sampled 100Hz time sequence in the stated window, with no lag search, phase averaging or time warping; dimensionless. It is distinct from matching phase-mean profiles in the prior governor report.",
               "first_pass_nominal_descriptive": descriptive, "limitations": [
                   "One PPO batch/update and three paired Gaussian seeds do not establish robustness, convergence, or statistical significance.",
                   "Only standardized z is shared. State-dependent own sigma and realized action perturbations may change after PPO.",
                   "All finalized episode returns, including true early terminations and their penalties, remain in own-policy descriptive summaries. The full-horizon flag only governs matched-horizon physical-quality comparison.",
                   "Partial common-prefix rewards exclude terminal partial cost and missing future; they are separate from valid observed episode returns. Truncated 5s returns require a bootstrap convention for continuing-task value calibration.",
                   "Operational wall timeouts, numerical failures, and max-step cutoffs retain their recorded return/status but are excluded from the task-episode mean as externally censored observations; true task terminations remain included.",
                   "Noisy cases reproduce the RLlib training connector (normalize_actions=True, clip_actions=False, unsquash arithmetic); nominal cases retain the deployment evaluator pipeline for exact archived reuse. Nominal/noisy raw clipping penalties and arithmetic are therefore not interchangeable.",
                   "Torque phase similarity to the before checkpoint measures change, not biomechanical correctness or agreement with a V26 torque target.",
                   "Analysis window is 13.15–17.95s on common STO support; early failures use a 150ms filter boundary margin and may lack complete cycles.",
                   "Contact and reserve diagnostics use existing logged contact/threshold definitions, with translation forces in N and rotations in Nm kept separate.",
                   "No target-dependent gates or new reward weights are introduced."]}
    results["stochastic_observed_episode_summary"] = {}
    for role in ("before", "after"):
        selected = [s for s in statuses.values() if s["role"] == role and s["scenario"] == "noisy" and "episode_return" in s]
        if selected:
            eligible = [s for s in selected if s["eligible_finite_task_episode_return"]]
            results["stochastic_observed_episode_summary"][role] = {
                "observed_episodes": len(selected), "full_horizon_episodes": sum(s["completed_full_episode"] for s in selected),
                "returns": [s["episode_return"] for s in selected],
                "eligible_task_episodes": len(eligible), "externally_censored_cases": [s["name"] for s in selected if s["externally_censored_or_nonstandard"]],
                "mean_task_episode_return_including_true_early_terminations": float(np.mean([s["episode_return"] for s in eligible])) if eligible else None,
                "end_reasons": [s["end_reason"] for s in selected],
                "interpretation": "Descriptive three-seed finite sample, not an estimate with validated statistical significance. All raw returns stay visible; only operational/external censoring is excluded from task mean, not true early task termination."}
    for source in (Path(__file__), HERE / "evaluate.py", PRIOR / "quality/analyze.py", PRIOR / "physical/analyze.py"):
        HASHES[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
    assert all(hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected for path, expected in HASHES.items())
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    write(output / "metrics.json", results)
    write(output / "input_hashes.json", HASHES)
    plot_receipt = render_comparison_figures(data, statuses, groups)
    write(output / "plot_receipt.json", plot_receipt)
    lines = ["# Physical comparison after one PPO batch", "", f"Status: **{results['status']}**, {results['finalized_case_count']}/{len(statuses)} requested cases finalized.", "", "Paired standard normals; each policy uses its own state-dependent sigma. Reward and target equality are checked on the shared completed prefix.", "",
             "| Case | Role | Scenario | Completed steps | End | Return | Penetration max, step summary (mm) |", "|---|---|---|---:|---|---:|---:|"]
    for status in statuses.values():
        if "episode_return" in status:
            lines.append(f"| {status['name']} | {status['role']} | {status['scenario']} | {status['completed_segments']} | {status['end_reason']} | {status['episode_return']:.3f} | {status['summary_penetration_max_mm']:.3f} |")
        else:
            lines.append(f"| {status['name']} | {status['role']} | {status['scenario']} | — | pending | — | — |")
    for name, pair in pairs.items():
        if not pair.get("data"):
            continue
        lines += ["", f"## {name}", "", f"{pair['complete_common_prefix_steps']} completed common steps; full-return comparison eligible: {pair['eligible_full_return_comparison']}."]
        before, after = pair["data"][pair["before"]], pair["data"][pair["after"]]
        if not (before["shape_available"] and after["shape_available"]):
            lines += ["", "The recorded shared prefix is too short for the fixed shape window; only prefix tracking and reward are reported in JSON."]
            continue
        lines += ["", f"Common analysis window: {before['analysis_window_s'][0]:.3f}–{before['analysis_window_s'][1]:.3f} s.", "",
                  "| Metric | Before | After |", "|---|---:|---:|"]
        metrics = []
        for joint in ("knee", "ankle"):
            metrics.append((f"{joint} actual-target RMSE (deg)", *[d["window_tracking"][joint]["actual_target_deg"]["rmse"] for d in (before, after)]))
            metrics.append((f"{joint} normalized shape error, time sequence (dimensionless)", *[d["window_tracking"][joint]["actual_target_deg"]["centered_unit_rms_shape_error"] for d in (before, after)]))
            metrics.append((f"{joint} served-target RMSE (deg)", *[d["window_tracking"][joint]["served_target_deg"]["rmse"] for d in (before, after)]))
            metrics.append((f"{joint} torque raw−LP20 RMS (Nm)", *[d["torque"][joint]["raw_minus_lp20_rms_nm"] for d in (before, after)]))
            metrics.append((f"{joint} LP10 normalized total variation (/s)", *[d["torque"][joint]["lp10_tv_normalized_per_s"] for d in (before, after)]))
        for label, key in (("Vertical impulse (Ns)", "vertical_impulse_n_s"), ("Logged contact fraction", "logged_contact_time_fraction")):
            metrics.append((label, *[d["window_physical"]["load_contact_penetration"][key] for d in (before, after)]))
        metrics.append(("Penetration max, native STO (mm)", *[d["window_physical"]["load_contact_penetration"]["penetration_mm"]["max"] for d in (before, after)]))
        for group, unit in (("pelvis_translation", "N"), ("pelvis_rotation", "Nm"), ("biological_rotation", "Nm")):
            metrics.append((f"{group} reserve vector RMS ({unit})", *[d["window_physical"]["reserves"][group]["vector_magnitude"]["rms"] for d in (before, after)]))
        for label, b, a in metrics:
            lines.append(f"| {label} | {'—' if b is None else format(b, '.5g')} | {'—' if a is None else format(a, '.5g')} |")
    lines += ["", "Kinematic shape in these tables uses the sampled time sequence in the stated window: RMS(zscore(actual) − zscore(target)), with no lag search, phase averaging or time warping. It is dimensionless. The earlier governor report also computed phase-mean shape errors; those are a different aggregation and must not be mixed with these values.", "", "Detailed tracking, SEA torque shape, contact impulse and reserve comparisons: `metrics.json`.", "", *[f"- {text}" for text in results["limitations"]], ""]
    (output / "README.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({"cases": len(statuses), "complete_pairs": sum(p.get("comparison") != "pending" for p in pairs.values()), "output": str(output)}))


if __name__ == "__main__":
    main()
