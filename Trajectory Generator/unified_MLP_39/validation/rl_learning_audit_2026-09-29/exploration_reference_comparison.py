"""Compare sampled partial trajectories with nominal best80 at identical times.

Consumes exploration_records.jsonl, never samples or updates a policy. Comparison
units are contiguous physical segments within one exported episode fragment. No
concatenation crosses a fragment, missing step, target mismatch or partial step.
Reward comparisons require explicit batch reward configuration, equality to the
nominal configuration, and successful reconstruction from logged reward terms.

The question answered is whether this batch *contains* locally better observed
trajectories on the matched target/time support. This is neither a full-return
ranking nor a causal estimate of an action's advantage. EnvRunner fragments from
one PPO iteration are not independent training replications.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import sys

import numpy as np

from exploration_analysis import JOINTS, _completed, _continuous, jsonable

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
REPO = PACKAGE.parents[1]
BASELINE = PACKAGE / "runs/rollout/general_method_diagnosis_2026-09-29/LEFT_baseline/rollout_policy_trace.json"
PLOT = REPO / "plot/09_29_2026_rl_learning_audit/exploration_reference_comparison.png"


def matching(actual, target):
    actual, target = np.asarray(actual, float), np.asarray(target, float)
    if actual.shape != target.shape or not (np.isfinite(actual).all() and np.isfinite(target).all()):
        raise ValueError("Tracking comparison requires aligned finite samples")
    if not len(actual):
        return None
    a_std, t_std = float(np.std(actual)), float(np.std(target))
    shape = len(actual) >= 3 and min(a_std, t_std) > 1e-8
    return {"n": len(actual), "rmse_deg": float(np.sqrt(np.mean(np.square(actual-target)))),
        "bias_deg": float(np.mean(actual-target)), "actual_range_deg": float(np.ptp(actual)),
        "target_range_deg": float(np.ptp(target)),
        "range_ratio": float(np.ptp(actual)/np.ptp(target)) if np.ptp(target) > 1e-8 else None,
        "correlation": float(np.corrcoef(actual, target)[0, 1]) if shape else None,
        "centered_unit_rms_shape_error": float(np.sqrt(np.mean(np.square(
            (actual-actual.mean())/a_std - (target-target.mean())/t_std)))) if shape else None,
        "shape_scope": "Observed partial time window; no lag fit, cycle completion assumption or uncertainty interval"}


def _target_match(record, nominal, tolerance):
    payload = record["info"].get("imitation_target", {})
    errors = {}
    for joint in JOINTS:
        for field, nominal_key in (("q", "imitation_target_q"), ("qdot", "imitation_target_qdot")):
            try:
                error = abs(float(payload[field][joint])-float(nominal[nominal_key][joint]))
            except (KeyError, TypeError, ValueError):
                return False, {"reason": "missing_target_fields"}
            errors[f"{joint}.{field}"] = error
        try:
            a = float(payload["phase"][joint])
            b = float(nominal["reward_terms"][joint + "_imitation_target_phase"])
            errors[f"{joint}.phase"] = abs((a-b+.5) % 1-.5)
        except (KeyError, TypeError, ValueError):
            return False, {"reason": "missing_prescribed_phase"}
    return all(np.isfinite(v) and v <= tolerance for v in errors.values()), errors


def _reward_context(records, nominal, summary, reward_mapping):
    """Reconstruct both sides; absence/mismatch disables reward claims explicitly."""
    audit = {"available": False, "reason": "No explicit batch reward configuration supplied"}
    if reward_mapping is None:
        return audit, None, None
    sys.path.insert(0, str(PACKAGE.parent))
    from unified_MLP_39.reward_function import RewardConfig, compute_reward
    batch_cfg = RewardConfig.from_mapping(reward_mapping.get("reward", reward_mapping))
    nominal_cfg = RewardConfig.from_mapping(summary["reward_config"])
    if asdict(batch_cfg) != asdict(nominal_cfg):
        audit.update(reason="Reward configurations differ", different_fields=[k for k, v in asdict(batch_cfg).items()
            if v != asdict(nominal_cfg)[k]])
        return audit, None, None
    baseline_rewards, errors, record_errors = [], [], []
    try:
        for row in nominal:
            reward, _ = compute_reward(row["reward_terms"], nominal_cfg, reference=row["policy_segment_values"])
            baseline_rewards.append(float(reward))
        errors.append(abs(sum(baseline_rewards)-float(summary["episode_return"])))
        for record in records:
            info = record["info"]
            reward, _ = compute_reward(info["reward_terms"], batch_cfg, reference=info["policy_segment_values"])
            record_errors.append(abs(float(reward)-float(record["reward"])))
    except (KeyError, TypeError, ValueError) as exc:
        audit.update(reason="Logged terms cannot reconstruct reward", error=f"{type(exc).__name__}: {exc}")
        return audit, None, None
    audit.update(configurations_equal=True, nominal_full_sum_error=max(errors),
                 max_batch_transition_error=max(record_errors, default=0.0),
                 batch_transitions_checked=len(record_errors),
                 action_pipeline_note="Nominal deployment and RLlib sampling differ in action preprocessing. Objective equality and each observed reward are checked; physical trajectories are not assumed equal.")
    if max(errors + record_errors, default=0.0) > 1e-7:
        audit.update(reason="Reward reconstruction exceeds 1e-7 tolerance")
        return audit, None, None
    audit.update(available=True, reason=None,
        scope="Undiscounted sum only on matching completed segments; terminal partial costs retained separately")
    return audit, np.asarray(baseline_rewards), np.asarray(record_errors)


def _block_metrics(pairs, records, baseline, baseline_rewards):
    sampled = [records[r] for r, _ in pairs]
    nominal = [baseline[b] for _, b in pairs]
    times = [float(r["info"]["time"]) for r in sampled]
    out = {"episode_id": sampled[0]["episode_id"], "fragment_index": sampled[0]["fragment_index"],
        "episode_t_first": sampled[0]["episode_t"], "episode_t_last": sampled[-1]["episode_t"],
        "n_completed_segments": len(sampled), "time_first_endpoint_s": times[0], "time_last_endpoint_s": times[-1],
        "time_before_first_s": float(sampled[0]["time_before"]),
        "completed_duration_s": times[-1]-float(sampled[0]["time_before"]),
        "baseline_indices": [b for _, b in pairs],
        "target_source": sampled[0]["info"]["imitation_target"].get("source"),
        "target_visible": sampled[0]["info"].get("target_availability", {}).get("visible"),
        "phase_start_end_prescribed": {j: [sampled[0]["info"]["imitation_target"]["phase"][j],
            sampled[-1]["info"]["imitation_target"]["phase"][j]] for j in JOINTS},
        "kinematics": {}, "reward": {"available": baseline_rewards is not None},
        "scope": "Matched contiguous partial trajectory, not a complete episode/cycle claim"}
    improvement = {}
    for joint in JOINTS:
        target = np.degrees([r["info"]["imitation_target"]["q"][joint] for r in sampled])
        joint_result = {}
        for kind, suffix in (("actual", ""), ("served", "_served_ref")):
            observed = np.degrees([r["observation_after"][joint+suffix] for r in sampled])
            reference = np.degrees([r["prosthetic_state"][joint+suffix] for r in nominal])
            sample_metrics, nominal_metrics = matching(observed, target), matching(reference, target)
            deltas = {}
            for metric in ("rmse_deg", "centered_unit_rms_shape_error", "correlation"):
                a, b = sample_metrics[metric], nominal_metrics[metric]
                deltas[metric] = a-b if a is not None and b is not None else None
            joint_result[kind] = {"sampled": sample_metrics, "nominal_same_times": nominal_metrics,
                "sampled_minus_nominal": deltas}
            improvement[f"{joint}.{kind}.rmse_lower"] = deltas["rmse_deg"] < -1e-9
            value = deltas["centered_unit_rms_shape_error"]
            improvement[f"{joint}.{kind}.shape_error_lower"] = value < -1e-9 if value is not None else None
        out["kinematics"][joint] = joint_result
    out["improvement_flags"] = improvement
    out["both_joint_actual_rmse_lower"] = all(improvement[j+".actual.rmse_lower"] for j in JOINTS)
    out["both_joint_served_rmse_lower"] = all(improvement[j+".served.rmse_lower"] for j in JOINTS)
    actual_shape = [improvement[j+".actual.shape_error_lower"] for j in JOINTS]
    out["both_joint_actual_shape_error_lower"] = all(actual_shape) if None not in actual_shape else None
    if baseline_rewards is not None:
        sampled_sum = sum(r["reward"] for r in sampled)
        nominal_sum = float(sum(baseline_rewards[b] for _, b in pairs))
        out["reward"].update(sampled_completed_prefix_sum=sampled_sum,
            nominal_same_times_sum=nominal_sum, sampled_minus_nominal=sampled_sum-nominal_sum,
            sampled_mean_per_completed_step=sampled_sum/len(sampled),
            nominal_mean_per_completed_step=nominal_sum/len(sampled),
            higher_on_matched_completed_prefix=sampled_sum > nominal_sum+1e-9,
            note="No terminal partial transition or unobserved future enters this paired sum; not a full return or advantage")
    return out


def compare_records(records, baseline, summary, *, reward_mapping=None, time_tolerance=1e-7,
                    target_tolerance=1e-7):
    records = list(records)
    if not baseline:
        raise ValueError("Empty nominal trace")
    keys = [(r["episode_id"], r["episode_t"]) for r in records]
    if len(set(keys)) != len(keys):
        raise ValueError("Duplicate episode/t key in batch records")
    baseline_times = np.array([r["time"] for r in baseline])
    if not np.all(np.diff(baseline_times) > 0):
        raise ValueError("Nominal trace must have strictly increasing completed endpoints")
    if not all(r.get("segment_completed", False) for r in baseline):
        raise ValueError("Nominal reference must contain only completed segments")
    reward_audit, baseline_rewards, _ = _reward_context(records, baseline, summary, reward_mapping)
    fragments = defaultdict(list)
    for index, record in enumerate(records):
        fragments[(str(record["episode_id"]), int(record["fragment_index"]))].append(index)
    output = {"schema": "matched_exploration_reference_v1", "n_batch_records": len(records),
        "n_distinct_episodes": len({r["episode_id"] for r in records}), "n_exported_fragments": len(fragments),
        "matching_tolerances": {"time_s": time_tolerance, "target_q_rad_qdot_rad_s_phase": target_tolerance},
        "reward_reconstruction": reward_audit, "fragments": [], "blocks": [],
        "limitations": [
            "Every comparison uses the nominal samples at exactly the same absolute times, with target q/qdot and joint prescribed phase equality checked.",
            "Fragments and matched blocks are correlated observations from one PPO batch; EnvRunners are not independent training replications.",
            "A better partial RMSE, shape or reward demonstrates a sampled local outcome, not a causal action advantage or full-episode superiority.",
            "Shape uses the observed partial window with no phase/lag fitting; a short target excursion may make shape statistics unstable or unavailable.",
            "Completed-prefix rewards omit terminal partial costs; all original fragment rewards and terminal reasons are retained separately.",
            "Kinematic improvements do not establish torque-shape parity with V26; this comparator has no 1kHz torque data for training fragments.",
        ]}
    for (episode_id, fragment_index), indices in sorted(fragments.items()):
        indices.sort(key=lambda index: records[index]["episode_t"])
        frag_rows = [records[i] for i in indices]
        fragment = {"episode_id": episode_id, "fragment_index": fragment_index,
            "episode_t_first": frag_rows[0]["episode_t"], "episode_t_last": frag_rows[-1]["episode_t"],
            "n_records": len(indices), "observed_fragment_reward_sum": sum(r["reward"] for r in frag_rows),
            "end_reason": frag_rows[-1]["info"].get("end_reason"),
            "terminated": bool(frag_rows[-1].get("terminated")), "truncated": bool(frag_rows[-1].get("truncated")),
            "starts_at_episode_reset": frag_rows[0]["episode_t"] == 0,
            "fragment_is_complete_episode": frag_rows[0]["episode_t"] == 0 and bool(frag_rows[-1].get("terminated") or frag_rows[-1].get("truncated")),
            "incomplete_transition_rewards": [{"episode_t": r["episode_t"], "reward": r["reward"],
                "end_reason": r["info"].get("end_reason")} for r in frag_rows if not _completed(r)],
            "rejected_rows": [], "block_indices": []}
        block = []
        def flush():
            if block:
                fragment["block_indices"].append(len(output["blocks"]))
                output["blocks"].append(_block_metrics(block, records, baseline, baseline_rewards))
                block.clear()
        for index in indices:
            record = records[index]
            reason = None
            if not _completed(record):
                reason = "incomplete_physical_segment_or_missing_time"
            else:
                time = float(record["info"]["time"])
                candidate = int(np.argmin(np.abs(baseline_times-time)))
                if abs(baseline_times[candidate]-time) > time_tolerance:
                    reason = "no_nominal_sample_at_same_absolute_time"
                else:
                    equivalent, errors = _target_match(record, baseline[candidate], target_tolerance)
                    if not equivalent:
                        reason = "target_q_qdot_or_prescribed_phase_mismatch"
            if reason:
                flush()
                fragment["rejected_rows"].append({"episode_t": record["episode_t"], "reason": reason})
                continue
            if block and (not _continuous(records[block[-1][0]], record) or candidate != block[-1][1]+1):
                flush()
            block.append((index, candidate))
        flush()
        fragment["n_matched_completed_segments"] = sum(output["blocks"][i]["n_completed_segments"] for i in fragment["block_indices"])
        output["fragments"].append(fragment)
    blocks = output["blocks"]
    output["observed_counts_not_statistical_tests"] = {
        "matched_blocks": len(blocks), "matched_completed_segments": sum(b["n_completed_segments"] for b in blocks),
        "blocks_with_lower_actual_rmse_both_joints": sum(b["both_joint_actual_rmse_lower"] for b in blocks),
        "blocks_with_lower_served_rmse_both_joints": sum(b["both_joint_served_rmse_lower"] for b in blocks),
        "blocks_with_lower_actual_shape_error_both_joints": sum(b["both_joint_actual_shape_error_lower"] is True for b in blocks),
        "blocks_with_reconstructible_reward_comparison": sum(b["reward"]["available"] for b in blocks),
        "blocks_with_higher_matched_prefix_reward": sum(b["reward"].get("higher_on_matched_completed_prefix", False) for b in blocks),
        "blocks_with_lower_actual_rmse_both_and_higher_prefix_reward": sum(b["both_joint_actual_rmse_lower"] and b["reward"].get("higher_on_matched_completed_prefix", False) for b in blocks)}
    return jsonable(output)


def plot_comparison(result, path=PLOT):
    os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    blocks = result["blocks"]
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.3))
    for ax, kind in zip(axes[:2], ("actual", "served")):
        for i, block in enumerate(blocks):
            delta = [block["kinematics"][j][kind]["sampled_minus_nominal"]["rmse_deg"] for j in JOINTS]
            ax.scatter(*delta, s=30)
            ax.annotate(str(i), delta, fontsize=7, xytext=(3, 3), textcoords="offset points")
        ax.axhline(0, color="gray", lw=.8)
        ax.axvline(0, color="gray", lw=.8)
        ax.set(xlabel="Δ knee RMSE (deg)", ylabel="Δ ankle RMSE (deg)",
               title=f"{kind.title()}: sampled − nominal\nLower left: both RMSEs decrease")
        ax.grid(alpha=.2)
    for i, block in enumerate(blocks):
        if block["reward"]["available"]:
            xy = (block["completed_duration_s"], block["reward"]["sampled_minus_nominal"])
            axes[2].scatter(*xy, s=30)
            axes[2].annotate(str(i), xy, fontsize=7, xytext=(3, 3), textcoords="offset points")
    axes[2].axhline(0, color="gray", lw=.8)
    axes[2].set(xlabel="Matched completed duration (s)", ylabel="Δ reward sum, same completed steps",
                title="Reward on the matched partial window\nNo terminal partial cost or missing future")
    axes[2].grid(alpha=.2)
    fig.suptitle("One PPO batch: separate contiguous fragments vs best80 nominal at identical prescribed target times", fontsize=11)
    fig.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records", type=Path, help="exploration_records.jsonl")
    parser.add_argument("--baseline-trace", type=Path, default=BASELINE)
    parser.add_argument("--reward-config", type=Path, help="Frozen batch JSON/YAML reward or resolved training configuration")
    parser.add_argument("--output-dir", type=Path, default=HERE / "exploration_reference_comparison")
    parser.add_argument("--plot", action="store_true", help="Save standalone plot beneath repository plot/")
    args = parser.parse_args()
    paths = [args.records, args.baseline_trace, args.baseline_trace.with_name("rollout_summary.json"), Path(__file__), HERE / "exploration_analysis.py"]
    def read_json(path):
        return json.loads(Path(path).read_text(encoding="utf-8"))
    records = [json.loads(line) for line in args.records.read_text(encoding="utf-8").splitlines() if line.strip()]
    reward_mapping = None
    if args.reward_config:
        paths.append(args.reward_config)
        paths += [PACKAGE / "reward_function.py", PACKAGE / "tuning_terms.py",
                  PACKAGE.parent / "baseline_MLP/experimental_morphology_corridor.py"]
        import yaml
        reward_mapping = yaml.safe_load(args.reward_config.read_text(encoding="utf-8"))
    before = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    result = compare_records(records, read_json(args.baseline_trace), read_json(paths[2]), reward_mapping=reward_mapping)
    result["input_hashes"] = before
    result["baseline_trace"] = str(args.baseline_trace.resolve())
    if args.plot:
        plot_comparison(result)
        result["plot"] = str(PLOT.resolve())
    if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != digest for p, digest in before.items()):
        raise RuntimeError("Input changed during offline comparison")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "comparison.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps(result["observed_counts_not_statistical_tests"]))


if __name__ == "__main__":
    main()
