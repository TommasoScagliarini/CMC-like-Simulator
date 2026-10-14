"""Separate physical comparison of corrected same-batch replay vs before/stock.

Read finalized original and corrected evaluation receipts; never run simulations
or overwrite the original comparison. Pair each own-sigma seed independently.
Uses existing physical/quality helpers for identical reward, support, filters
and prescribed-phase conventions. Defaults to waiting for every expected receipt.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

import physical_analysis as physical
from run_audit import HERE, REPO, write_json

ORIGINAL = HERE / "evaluation_manifest.json"
CORRECTED = HERE / "corrected_evaluation_manifest.json"
OUTPUT = HERE / "corrected_physical_comparison"
PLOTS = REPO / "plot/09_29_2026_rl_learning_audit/corrected_comparison"
ROLES = ("before", "after", "corrected")
COLORS = {"before": "#4c566a", "after": "#d45b22", "corrected": "#20805d"}


def scenario_key(case):
    return "nominal" if case["scenario"] == "nominal" else f"noisy_seed{case['noise_seed']}"


def descriptive(values):
    values = np.asarray(values, float)
    return {"n": len(values), "values": values.tolist(),
        "mean": float(np.mean(values)) if len(values) else None,
        "standard_deviation_ddof0": float(np.std(values)) if len(values) else None,
        "minimum": float(np.min(values)) if len(values) else None,
        "maximum": float(np.max(values)) if len(values) else None,
        "interpretation": "Finite-sample dispersion, not uncertainty, significance or independent training replications"}


def load_specs(path, *, corrected=False):
    manifest = physical.read(path)
    result = []
    for entry in manifest["cases"]:
        if isinstance(entry, str):
            source = Path(entry)
            entry = physical.read(source if source.is_absolute() else path.parent / source)
        case = dict(entry)
        role = case.get("comparison_role", case["role"])
        if corrected and role != "corrected":
            raise ValueError("Corrected manifest needs explicit comparison_role=corrected")
        if not corrected and role not in ("before", "after"):
            continue
        case["evaluator_role"] = case["role"]
        case["role"] = role
        result.append(case)
    return result


def compare(reference, candidate, statuses, reference_role):
    # These checks also cover an immediate failure with no completed prefix.
    a, b = reference["noise"]["protocol"], candidate["noise"]["protocol"]
    for key in ("scenario", "noise_seed", "environment_seed", "noise_hold_steps", "noise_scale", "sigma_mode", "action_transform"):
        if a[key] != b[key]:
            raise ValueError(f"Different paired evaluation protocol: {key}")
    if reference["noise"]["full_standard_normal_stream_sha256"] != candidate["noise"]["full_standard_normal_stream_sha256"]:
        raise ValueError("Paired standard-normal streams differ")
    if reference["summary"]["reward_config"] != candidate["summary"]["reward_config"]:
        raise ValueError("Paired reward configurations differ")
    result = physical.compare(reference, candidate, statuses)
    result.update(reference_role=reference_role, candidate_role="corrected",
        comparison_direction="corrected minus reference; scalar differences do not define an overall quality ranking",
        interpretation="Same recorded PPO batch and initial learner state, stock vs corrected update; physical evaluation is one checkpoint per branch on three paired noise streams, not a training convergence/generalization experiment.")
    result["metric_differences"] = {}
    if not result.get("data"):
        return result
    before, after = result["data"][reference["name"]], result["data"][candidate["name"]]
    for scope in ("prefix_tracking", "window_tracking"):
        if scope not in before or scope not in after:
            continue
        result["metric_differences"][scope] = {}
        for joint in ("knee", "ankle"):
            for signal in ("actual_target_deg", "served_target_deg"):
                for metric in ("rmse", "centered_unit_rms_shape_error", "correlation", "range_ratio"):
                    bval, aval = before[scope][joint][signal][metric], after[scope][joint][signal][metric]
                    result["metric_differences"][scope][f"{joint}.{signal}.{metric}"] = aval-bval if aval is not None and bval is not None else None
    if before["shape_available"] and after["shape_available"]:
        result["metric_differences"]["torque"] = {}
        for joint in ("knee", "ankle"):
            for metric in ("rms_nm", "min_nm", "max_nm", "raw_minus_lp20_rms_nm", "raw_minus_lp20_normalized_dynamic_rms", "lp10_tv_normalized_per_s"):
                bval, aval = before["torque"][joint][metric], after["torque"][joint][metric]
                result["metric_differences"]["torque"][f"{joint}.{metric}"] = aval-bval if aval is not None and bval is not None else None
    return result


def summarize_stochastic(statuses, pairs):
    result = {"by_role": {}, "paired_return_changes": {}}
    for role in ROLES:
        observed = sorted([s for s in statuses.values() if s["role"] == role and s["scenario"] == "noisy" and "episode_return" in s], key=lambda s: s["noise_seed"])
        eligible = [s for s in observed if s["eligible_finite_task_episode_return"]]
        result["by_role"][role] = {
            "observed_cases": [s["name"] for s in observed],
            "eligible_task_cases": [s["name"] for s in eligible],
            "externally_censored_cases": [s["name"] for s in observed if not s["eligible_finite_task_episode_return"]],
            "full_horizon_cases": [s["name"] for s in observed if s["completed_full_episode"]],
            "all_observed_raw_returns": [s["episode_return"] for s in observed],
            "task_episode_return_including_true_early_termination": descriptive([s["episode_return"] for s in eligible]),
            "completed_steps_all_observed": descriptive([s["completed_segments"] for s in observed]),
            "end_reasons": {s["name"]: s["end_reason"] for s in observed}}
    for reference_role in ("before", "after"):
        selected = [p for key, p in pairs.items() if key.startswith(f"corrected_vs_{reference_role}/noisy_") and p.get("both_returns_are_task_episode_samples")]
        result["paired_return_changes"][f"corrected_minus_{reference_role}"] = {
            "pairs": [p["after"] + " vs " + p["before"] for p in selected],
            "return_difference": descriptive([p["observed_episode_return_after_minus_before"] for p in selected]),
            "scope": "Paired finite-task episodes including physical early terminations; exclude operationally censored returns. Nominal and stochastic protocols remain separate."}
    return result


def render(data, statuses, groups):
    import matplotlib.pyplot as plt
    PLOTS.mkdir(parents=True, exist_ok=True)
    saved = []
    available = {role: data[name] for role, name in groups.get("nominal", {}).items() if data[name] is not None}
    if available:
        fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True, constrained_layout=True)
        reference = available.get("before", next(iter(available.values())))
        for j, joint in enumerate(("knee", "ankle")):
            coord = f"pros_{joint}_angle"
            rows = reference["trace"]
            for i in (0, 1):
                axes[i, j].plot([r["time"] for r in rows], np.degrees([r["imitation_target_q"][coord] for r in rows]), "k--", lw=1.2, label="Prescribed LEFT target")
            for role in ROLES:
                if role not in available:
                    continue
                case = available[role]
                rows = case["trace"]
                if not rows:
                    continue
                state = statuses[case["name"]]
                time = np.asarray([r["time"] for r in rows])
                label = f"{role}: {len(rows)} steps, {state['end_reason']}"
                for i, suffix in ((0, ""), (1, "_served_ref")):
                    axes[i, j].plot(time, np.degrees([r["prosthetic_state"][coord+suffix] for r in rows]), color=COLORS[role], lw=1.3, label=label)
                tab = case["tables"]["sea_torques"]
                support = tab["time"] <= time[-1]+1e-9
                t, raw = tab["time"][support], tab[f"SEA_{joint.title()}_tau_spring"][support]
                # Filter only the completed support. The last 150ms are hidden
                # on shortened episodes, matching the numerical quality audit.
                tail = 0 if state["completed_full_episode"] else .15
                mask = t <= t[-1]-tail+1e-9
                axes[2, j].plot(t[mask], raw[mask], color=COLORS[role], lw=.5, alpha=.18)
                try:
                    low, _ = physical.q.filter_split(t, raw, 10)
                except (ValueError, AssertionError):
                    axes[2, j].plot(t[mask], raw[mask], color=COLORS[role], lw=1, label=role+" (raw, insufficient filter support)")
                else:
                    axes[2, j].plot(t[mask], low[mask], color=COLORS[role], lw=1.3, label=role+" LP10")
            for i, title, unit in ((0, "actual", "deg"), (1, "served", "deg"), (2, "SEA spring, raw faint / LP10 solid", "Nm")):
                axes[i, j].set(title=f"{joint} {title}", ylabel=unit, xlim=physical.WINDOW)
                axes[i, j].grid(alpha=.22)
            axes[0, j].legend(fontsize=7)
            axes[2, j].set_xlabel("time (s)")
        fig.suptitle("Same-batch correction — nominal deployment comparison\nCurves retain raw OpenSim signs and stop at completed recorded support; no time shift", fontsize=12)
        path = PLOTS / "nominal_before_stock_corrected.png"
        fig.savefig(path, dpi=170)
        plt.close(fig)
        saved.append(path)
    noisy = [s for s in statuses.values() if s["scenario"] == "noisy" and "episode_return" in s]
    if noisy:
        fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), constrained_layout=True)
        for role, offset in zip(ROLES, (-.18, 0, .18)):
            for i, seed in enumerate((123, 125, 127)):
                row = next((s for s in noisy if s["role"] == role and s["noise_seed"] == seed), None)
                if row is None:
                    continue
                marker = "o" if row["eligible_finite_task_episode_return"] else "x"
                for ax, key in zip(axes, ("episode_return", "completed_segments", "summary_penetration_max_mm")):
                    ax.scatter(i+offset, row[key], color=COLORS[role], marker=marker, s=55, label=role if i == 0 else None)
                if row["end_reason"] != "episode_time_limit":
                    axes[1].annotate(row["end_reason"], (i+offset, row["completed_segments"]), xytext=(2, 5), textcoords="offset points", fontsize=7)
        for ax, title, unit in zip(axes, ("Observed return, including physical failures", "Completion (500 = time limit)", "Peak penetration from step diagnostics"), ("undiscounted return", "completed physical steps", "mm")):
            ax.set(title=title, ylabel=unit, xlabel="paired Gaussian seed")
            ax.set_xticks(range(3), ("123", "125", "127"))
            ax.grid(alpha=.2)
        axes[0].legend(fontsize=8)
        fig.suptitle("Own-sigma stochastic, same RLlib connector and paired z; x denotes external censoring\nThree noise streams describe dispersion, not significance or independent training replications", fontsize=11)
        path = PLOTS / "stochastic_before_stock_corrected.png"
        fig.savefig(path, dpi=170)
        plt.close(fig)
        saved.append(path)
    receipt = {"figures": {str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest() for p in saved},
        "scope": "Corrected branch comparison only; original plot and rollout artifacts untouched"}
    write_json(PLOTS / "plot_receipt.json", receipt)
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--original-manifest", type=Path, default=ORIGINAL)
    parser.add_argument("--corrected-manifest", type=Path, default=CORRECTED)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT)
    parser.add_argument("--allow-pending", action="store_true")
    parser.add_argument("--skip-plots", action="store_true")
    args = parser.parse_args()
    physical.HASHES.clear()
    cases = load_specs(args.original_manifest) + load_specs(args.corrected_manifest, corrected=True)
    missing = [c["name"] for c in cases if not (Path(c["output_dir"]) / "evaluation_complete.json").is_file()]
    if missing and not args.allow_pending:
        raise SystemExit("Await finalized receipts; no analysis written: " + ", ".join(missing))
    statuses, data, groups = {}, {}, {}
    for case in cases:
        status, loaded = physical.load(case)
        name = status["name"]
        if name in statuses:
            raise ValueError("Duplicate case name: " + name)
        status["evaluator_role"] = case["evaluator_role"]
        statuses[name], data[name] = status, loaded
        key = scenario_key(case)
        group = groups.setdefault(key, {})
        if case["role"] in group:
            raise ValueError("Duplicate scenario/role")
        group[case["role"]] = name
    pairs = {}
    for scenario, roles in groups.items():
        for reference in ("before", "after"):
            key = f"corrected_vs_{reference}/{scenario}"
            if all(role in roles and data[roles[role]] is not None for role in (reference, "corrected")):
                pairs[key] = compare(data[roles[reference]], data[roles["corrected"]], statuses, reference)
            else:
                pairs[key] = {"comparison": "pending", "available_roles": roles}
    result = {"schema": "same_batch_corrected_physical_comparison_v1", "cases": statuses, "pairs": pairs,
        "stochastic": summarize_stochastic(statuses, pairs),
        "limitations": [
            "One captured PPO batch with exact control replay and one corrected update does not establish convergence, learning from scratch or generalization.",
            "Each policy uses its own state-dependent sigma. Shared standard-normal draws do not hold realized action noise or physical states fixed.",
            "Nominal deployment and stochastic RLlib connector protocols are reported separately.",
            "True physical early terminations remain in finite-task return summaries. Operational/external truncations retain raw outcomes but are excluded from task-return means.",
            "Kinematic and torque metrics compare a common completed prefix/window within each pair. Different seeds can have different supported windows; those shape metrics are not pooled as if support were identical.",
            "Torque similarity measures waveform change relative to before/stock, not biomechanical correctness or V26 torque-target agreement.",
            "Three paired Gaussian seeds describe finite-sample dispersion only; no p-values or significance claim."]}
    sources = [Path(__file__), HERE / "physical_analysis.py", HERE / "evaluate.py",
               physical.PRIOR / "quality/analyze.py", physical.PRIOR / "physical/analyze.py"]
    for path in sources:
        physical.HASHES[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    for path, digest in physical.HASHES.items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise RuntimeError("Finalized input changed: " + path)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_json(args.output_dir / "metrics.json", result)
    write_json(args.output_dir / "input_hashes.json", physical.HASHES)
    if not args.skip_plots:
        write_json(args.output_dir / "plot_receipt.json", render(data, statuses, groups))
    lines = ["# Same-batch corrected physical comparison", "",
        "Corrected update versus both the original best80 and the stock update. Values remain descriptive; one training batch and three paired noise streams.", "",
        "| Case | Role | Completed steps | End | Observed return | Task sample |",
        "|---|---|---:|---|---:|---|"]
    for s in statuses.values():
        lines.append(f"| {s['name']} | {s['role']} | {s.get('completed_segments','—')} | {s.get('end_reason','pending')} | {s.get('episode_return','—')} | {s.get('eligible_finite_task_episode_return','—')} |")
    for name, pair in pairs.items():
        if not pair.get("data"):
            continue
        lines += ["", "## " + name, "", f"{pair['complete_common_prefix_steps']} matched completed steps. Observed return change: {pair['observed_episode_return_after_minus_before']:.6g}; both finite-task samples: {pair['both_returns_are_task_episode_samples']}.", ""]
        for scope in ("prefix_tracking", "window_tracking", "torque"):
            if scope not in pair["metric_differences"]:
                continue
            lines += ["| " + scope + " — corrected minus reference | Difference |", "|---|---:|"]
            for metric, value in pair["metric_differences"][scope].items():
                if scope != "torque" and not (metric.endswith("rmse") or metric.endswith("centered_unit_rms_shape_error")):
                    continue
                lines.append(f"| {metric} | {'—' if value is None else format(value,'.6g')} |")
            lines.append("")
    lines += ["", *["- " + text for text in result["limitations"]], ""]
    (args.output_dir / "README.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({"cases": len(statuses), "ready_pairs": sum(p.get('comparison') != 'pending' for p in pairs.values()), "output": str(args.output_dir)}))


if __name__ == "__main__":
    main()
