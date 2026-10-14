"""Offline exploration diagnostics for native RLlib SingleAgentEpisode exports.

No environment, rollout, learner update, or ray.init is invoked. The only RLlib
dependency is the optional native-state adapter. ``analyze_records`` needs numpy
only. Actions_for_env and applied actions are explicitly *reconstructed* from the
recorded action and the frozen connector flags, never reported as observations.

Alignment: action/reward i uses observations[i], observations[i+1], infos[i+1].
Episode t_started accounts for fragments; lookback is excluded by native getters.
Partial terminal transitions retain their reward but are excluded from physical
shape/lag diagnostics. No lag pair crosses an episode, missing step, partial step,
or time discontinuity. Prescribed target phase and FSM phase remain distinct.

The lag correlations below are observational, confounded by state, gait phase,
and persistent governor state. They neither estimate causal action effects nor
establish whether sampled returns/advantages are ground-truth policy quality.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import pickle
from typing import Any, Mapping, Sequence

import numpy as np

JOINTS = ("pros_knee_angle", "pros_ankle_angle")
INFO_KEYS = (
    "time", "target_time", "segment_completed", "end_reason", "terminal_contact",
    "target_availability", "imitation_target", "phase_fsm",
    "policy_segment_times", "policy_segment_values", "policy_segment_derivatives",
    "reference_governor_diagnostics", "reward_terms", "reward_components",
    "sea_segment_diagnostics", "episode_start_offset_s",
)
LAG_REWARD_KEYS = (
    "sound_imitation_score", "served_imitation_score", "imitation_score",
    "sound_imitation_loss", "served_imitation_loss", "command_rate_loss",
    "grf_penetration_loss", "grf_ankle_moment_flip_loss", "tracking_loss",
)


def jsonable(value: Any) -> Any:
    """Encode diagnostic nonfinite values as null, not nonstandard JSON NaN."""
    if isinstance(value, Mapping):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, np.ndarray):
        return jsonable(value.tolist())
    if isinstance(value, (tuple, list)):
        return [jsonable(v) for v in value]
    if isinstance(value, np.generic):
        return jsonable(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def reconstruct_actions(action, low, high, *, normalize_actions: bool, clip_actions: bool):
    """Mirror RLlib NormalizeAndClipActions and FlattenClipAction for flat Box.

    Preserve input dtype through the connector, whose arithmetic may round even
    with [-1,+1] bounds. The final wrapper casts float32 before clipping.
    This assumes the standard connector pipeline; caller must verify the config.
    """
    raw = np.asarray(action)
    lo, hi = np.asarray(low, dtype=np.float32), np.asarray(high, dtype=np.float32)
    if raw.shape != lo.shape or lo.shape != hi.shape or np.any(hi <= lo):
        raise ValueError("Action and finite Box bounds must have matching flat shapes")
    if not (np.isfinite(raw).all() and np.isfinite(lo).all() and np.isfinite(hi).all()):
        raise ValueError("Nonfinite raw action or bounds")
    if normalize_actions:
        unconstrained = lo + (raw + 1.0) * (hi - lo) / 2.0
        to_env = np.clip(unconstrained, lo, hi)
    else:
        unconstrained = raw.copy()
        to_env = np.clip(raw, lo, hi) if clip_actions else raw.copy()
    applied = np.clip(np.asarray(to_env, dtype=np.float32), lo, hi)
    return {
        "raw": raw, "connector_unclipped": unconstrained,
        "to_env_reconstructed": to_env, "applied_reconstructed": applied,
        "connector_clipped": (unconstrained < lo) | (unconstrained > hi)
        if normalize_actions or clip_actions else np.zeros(raw.shape, bool),
        "wrapper_clipped": (to_env < lo) | (to_env > hi),
        "raw_outside_policy_bounds": (raw < -1) | (raw > 1)
        if normalize_actions else (raw < lo) | (raw > hi),
    }


def records_from_episode_states(states: Sequence[dict], *, normalize_actions: bool,
                               clip_actions: bool) -> list[dict]:
    """Adapt learner_probe episodes_pre_gae.pkl; no simulator or Ray cluster."""
    from ray.rllib.env.single_agent_episode import SingleAgentEpisode

    records = []
    for fragment_index, state in enumerate(states):
        episode = SingleAgentEpisode.from_state(state)
        obs = np.asarray(episode.get_observations())
        infos = list(episode.get_infos())
        actions = np.asarray(episode.get_actions())
        rewards = np.asarray(episode.get_rewards())
        if len(obs) != len(actions) + 1 or len(infos) != len(obs) or len(rewards) != len(actions):
            raise ValueError("Native episode action/observation/info boundary mismatch")
        feature_names = next((x.get("observation_feature_names") for x in infos
                              if isinstance(x, Mapping) and x.get("observation_feature_names")), None)
        extras = episode.extra_model_outputs
        logits = np.asarray(episode.get_extra_model_outputs("action_dist_inputs")) if "action_dist_inputs" in extras else None
        logp = np.asarray(episode.get_extra_model_outputs("action_logp")) if "action_logp" in extras else None
        space = episode.action_space
        for i, action in enumerate(actions):
            before, after = infos[i] or {}, infos[i + 1] or {}
            names = after.get("observation_feature_names", feature_names)
            before_map = dict(zip(names, obs[i].tolist())) if names else before.get("observation", {})
            after_map = dict(zip(names, obs[i + 1].tolist())) if names else after.get("observation", {})
            record = {
                "episode_id": str(episode.id_), "episode_t": int(episode.t_started + i),
                "fragment_index": fragment_index, "module_id": episode.module_id,
                "observation_before": before_map, "observation_after": after_map,
                "observation_before_vector": obs[i], "observation_after_vector": obs[i + 1],
                "feature_names": names, "time_before": before.get("time"),
                "reward": float(rewards[i]),
                "terminated": bool(episode.is_terminated and i == len(actions) - 1),
                "truncated": bool(episode.is_truncated and i == len(actions) - 1),
                "info": {k: after[k] for k in INFO_KEYS if k in after},
                "actions": reconstruct_actions(action, space.low, space.high,
                                                normalize_actions=normalize_actions,
                                                clip_actions=clip_actions),
                "action_bounds": {"low": space.low, "high": space.high},
                "connector_config": {"normalize_actions": normalize_actions, "clip_actions": clip_actions},
                "applied_action_origin": "reconstructed_standard_connector_then_FlattenClipAction",
            }
            if logits is not None:
                if logits.shape != (len(actions), 2 * action.size):
                    raise ValueError("Expected diagonal Gaussian mean+logstd logits")
                mean, logstd = np.split(logits[i], 2)
                sigma = np.exp(logstd)
                mean_actions = reconstruct_actions(mean, space.low, space.high,
                                                   normalize_actions=normalize_actions,
                                                   clip_actions=clip_actions)
                innovation = np.asarray(action, dtype=float) - mean
                record["distribution"] = {
                    "mean": mean, "logstd": logstd, "sigma": sigma,
                    "innovation_raw": innovation,
                    "standardized_innovation": np.divide(innovation, sigma, out=np.full(innovation.shape, np.nan),
                                                         where=np.isfinite(sigma) & (sigma > 0)),
                    "mean_applied_reconstructed": mean_actions["applied_reconstructed"],
                    "sample_minus_mean_applied": record["actions"]["applied_reconstructed"] - mean_actions["applied_reconstructed"],
                }
            if logp is not None:
                record["action_logp"] = float(logp[i])
            records.append(jsonable(record))
    keys = [(r["episode_id"], r["episode_t"]) for r in records]
    if len(set(keys)) != len(keys):
        raise ValueError("Duplicate episode/t transition; do not count lookback twice")
    return records


def _number(value):
    return float(value) if isinstance(value, (int, float, np.number)) and np.isfinite(value) else np.nan


def _summary(values):
    values = np.asarray([_number(v) for v in values], float)
    x = values[np.isfinite(values)]
    if not x.size:
        return {"n": 0}
    return {"n": int(x.size), "mean": float(x.mean()), "std": float(x.std()),
            "rms": float(np.sqrt(np.mean(x*x))), "min": float(x.min()),
            "max": float(x.max()), "sum": float(x.sum())}


def _corr(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    valid = np.isfinite(x) & np.isfinite(y)
    x, y = x[valid], y[valid]
    return {"n": int(len(x)), "correlation": float(np.corrcoef(x, y)[0, 1])
            if len(x) >= 3 and np.std(x) > 1e-12 and np.std(y) > 1e-12 else None}


def _completed(record):
    info = record["info"]
    t0, t1 = _number(record.get("time_before")), _number(info.get("time"))
    return bool(info.get("segment_completed", False) and np.isfinite(t0 + t1) and t1 > t0)


def _stratum(record):
    info = record["info"]
    target = info.get("imitation_target", {})
    return (str(target.get("source", "unknown")), str(target.get("time_alignment", "unknown")),
            str(info.get("target_availability", {}).get("visible", "unknown")))


def _continuous(left, right):
    return (_completed(left) and _completed(right)
            and left["episode_id"] == right["episode_id"]
            and right["episode_t"] == left["episode_t"] + 1
            and not left.get("terminated") and not left.get("truncated")
            and abs(_number(left["info"].get("time")) - _number(right.get("time_before"))) < 1e-7
            and _stratum(left) == _stratum(right))


def _signal(record, path, index=None):
    value: Any = record
    for part in path.split("."):
        value = value.get(part, {}) if isinstance(value, Mapping) else {}
    if index is not None:
        value = value[index] if isinstance(value, (list, np.ndarray)) and len(value) > index else None
    return _number(value)


def _kinematic_metrics(records, phase_bins):
    result = {}
    for joint_index, joint in enumerate(JOINTS):
        target = [_signal(r, "info.imitation_target.q." + joint) for r in records]
        actual = [_signal(r, "observation_after." + joint) for r in records]
        served = [_signal(r, "observation_after." + joint + "_served_ref") for r in records]
        phase = np.asarray([_signal(r, "info.imitation_target.phase." + joint) for r in records])
        target, actual, served = map(np.asarray, (target, actual, served))
        endpoint = []
        for record in records:
            values = record["info"].get("policy_segment_values", [])
            endpoint.append(_number(values[-1][joint_index]) if len(values) and len(values[-1]) > joint_index else np.nan)
        endpoint = np.asarray(endpoint)
        joint_result = {"n": len(records), "units": "rad", "phase_source": "imitation_target.phase (prescribed)",
                        "actual_minus_target": _summary(actual - target),
                        "served_minus_target": _summary(served - target),
                        "actual_minus_served": _summary(actual - served),
                        "command_endpoint_after_slew_minus_served": _summary(endpoint - served),
                        "command_endpoint_after_slew_minus_target": _summary(endpoint - target),
                        "actual_target_correlation": _corr(actual, target),
                        "served_target_correlation": _corr(served, target), "phase_bins": []}
        for b in range(phase_bins):
            mask = np.isfinite(phase) & (np.floor((phase % 1) * phase_bins) == b)
            joint_result["phase_bins"].append({"phase_start": b / phase_bins, "phase_end": (b + 1) / phase_bins,
                "actual_minus_target": _summary((actual - target)[mask]),
                "served_minus_target": _summary((served - target)[mask]),
                "actual": _summary(actual[mask]), "served": _summary(served[mask]), "target": _summary(target[mask])})
        result[joint] = joint_result
    return result


def _lag_diagnostics(records, max_lag):
    by_episode = defaultdict(list)
    for record in records:
        by_episode[record["episode_id"]].append(record)
    runs = []
    for rows in by_episode.values():
        run = []
        for record in sorted(rows, key=lambda r: r["episode_t"]):
            if run and not _continuous(run[-1], record):
                runs.append(run)
                run = []
            if _completed(record):
                run.append(record)
        if run:
            runs.append(run)
    output = []
    for lag in range(max_lag + 1):
        pairs = [(run[i], run[i + lag]) for run in runs for i in range(max(0, len(run) - lag))]
        for j, joint in enumerate(JOINTS):
            # For policy_knots >1 the final knot is the command endpoint.
            innovation = [(_signal(a, "distribution.innovation_raw", len(a["actions"]["raw"]) - 2 + j)) for a, _ in pairs]
            targets = {"reward": [_number(b.get("reward")) for _, b in pairs]}
            targets["served_increment_rad"] = [
                _signal(b, "observation_after." + joint + "_served_ref") -
                _signal(b, "observation_before." + joint + "_served_ref") for _, b in pairs]
            targets["actual_target_abs_error_rad"] = [abs(_signal(b, "observation_after." + joint) -
                _signal(b, "info.imitation_target.q." + joint)) for _, b in pairs]
            for key in LAG_REWARD_KEYS:
                targets["reward_terms." + key] = [_signal(b, "info.reward_terms." + key) for _, b in pairs]
            output.append({"lag_steps": lag, "joint": joint,
                "lag_seconds": _summary([_number(b["time_before"]) - _number(a["time_before"]) for a, b in pairs]),
                "innovation_raw_vs_future": {name: _corr(innovation, vals) for name, vals in targets.items()}})
    return output


def analyze_records(records: Sequence[dict], *, max_lag=20, phase_bins=10,
                    gamma=None, lambda_=None, expected_episode_steps=500) -> dict:
    """Analyze adapter records, keeping different target/visibility strata apart."""
    if max_lag < 0 or phase_bins < 1:
        raise ValueError("Invalid lag or phase bin count")
    records = list(records)
    keys = [(r["episode_id"], r["episode_t"]) for r in records]
    if len(set(keys)) != len(keys):
        raise ValueError("Duplicate episode/t key")
    groups, episodes = defaultdict(list), defaultdict(list)
    for record in records:
        groups[_stratum(record)].append(record)
        episodes[record["episode_id"]].append(record)
    result = {"schema": "ppo_exploration_diagnostics_v1", "n_transitions": len(records),
              "n_completed_physical_segments": sum(_completed(r) for r in records),
              "interpretation": [
                  "Actions sent to env are reconstructed using recorded raw actions, native Box bounds and supplied frozen connector flags.",
                  "sample_minus_mean_applied is a same-observation algebraic clipping comparison, not a counterfactual physical rollout.",
                  "Lag correlations are descriptive, confounded by state, phase and governor memory; no causal credit or true-advantage claim.",
                  "All transition rewards, including partial terminal penalties, enter sampled fragment return; partial segments do not enter kinematics/lag.",
                  "Phase bin statistics pool observed endpoints; they are not independent cycles, confidence intervals or optimized phase alignments.",
              ], "episodes": [], "strata": []}
    for episode_id, rows in episodes.items():
        rows.sort(key=lambda r: r["episode_t"])
        t_indices = [r["episode_t"] for r in rows]
        is_full = (t_indices == list(range(expected_episode_steps))
                   and all(_completed(r) for r in rows)
                   and all(_continuous(a, b) for a, b in zip(rows, rows[1:]))
                   and rows[-1]["info"].get("end_reason") == "episode_time_limit")
        result["episodes"].append({"episode_id": episode_id, "n": len(rows),
            "episode_t_first": t_indices[0], "episode_t_last": t_indices[-1],
            "sampled_fragment_reward_sum": sum(r["reward"] for r in rows),
            "full_horizon_return": sum(r["reward"] for r in rows) if is_full else None,
            "full_horizon_eligible": is_full, "end_reason": rows[-1]["info"].get("end_reason"),
            "terminated": rows[-1].get("terminated"), "truncated": rows[-1].get("truncated"),
            "n_incomplete_segments": sum(not _completed(r) for r in rows)})
    for key, rows in sorted(groups.items()):
        completed = [r for r in rows if _completed(r)]
        group = {"target_source": key[0], "target_time_alignment": key[1], "target_visible": key[2],
                 "n": len(rows), "n_completed": len(completed),
                 "reward": _summary([r["reward"] for r in rows]),
                 "kinematics": _kinematic_metrics(completed, phase_bins), "actions": [],
                 "reward_terms": {}, "reward_components": {}, "governor": {}}
        action_dims = {len(r["actions"]["raw"]) for r in rows}
        if len(action_dims) != 1:
            raise ValueError("Mixed action dimensions in target stratum")
        for j in range(next(iter(action_dims))):
            action_result = {"component": j, "joint": JOINTS[j % 2], "knot": j // 2}
            for field in ("raw", "to_env_reconstructed", "applied_reconstructed", "connector_clipped",
                          "wrapper_clipped", "raw_outside_policy_bounds"):
                action_result[field] = _summary([_signal(r, "actions." + field, j) for r in rows])
            for field in ("mean", "logstd", "sigma", "innovation_raw", "standardized_innovation", "sample_minus_mean_applied"):
                action_result[field] = _summary([_signal(r, "distribution." + field, j) for r in rows])
            group["actions"].append(action_result)
        for category in ("reward_terms", "reward_components", "reference_governor_diagnostics"):
            source_rows = completed if category == "reference_governor_diagnostics" else rows
            all_keys = sorted(set().union(*(r["info"].get(category, {}).keys() for r in source_rows)))
            summaries = {}
            for name in all_keys:
                vals = [r["info"].get(category, {}).get(name) for r in source_rows]
                if any(isinstance(v, (list, tuple)) for v in vals):
                    length = max((len(v) for v in vals if isinstance(v, (list, tuple))), default=0)
                    summaries[name] = [_summary([v[j] if isinstance(v, (list, tuple)) and len(v) > j else None for v in vals]) for j in range(length)]
                else:
                    summaries[name] = _summary(vals)
            group["governor" if category == "reference_governor_diagnostics" else category] = summaries
        group["observational_lags"] = _lag_diagnostics(rows, max_lag)
        result["strata"].append(group)
    if gamma is not None and lambda_ is not None:
        if not (0 <= gamma <= 1 and 0 <= lambda_ <= 1):
            raise ValueError("gamma and lambda must be in [0,1]")
        decay = gamma * lambda_
        result["gae_kernel_only"] = {"gamma": gamma, "lambda": lambda_, "decay": decay,
            "weights_lag_0_to_max": [decay**lag for lag in range(max_lag + 1)],
            "half_life_steps": float(np.log(.5) / np.log(decay)) if 0 < decay < 1 else None,
            "note": "Nominal TD-residual weighting, not measured physical response or validated credit assignment. Real GAE is separately audited."}
    return jsonable(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episodes", type=Path, help="Trusted local episodes_pre_gae.pkl")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--normalize-actions", choices=("true", "false"), required=True)
    parser.add_argument("--clip-actions", choices=("true", "false"), required=True)
    parser.add_argument("--max-lag", type=int, default=20)
    parser.add_argument("--gamma", type=float)
    parser.add_argument("--lambda", dest="lambda_", type=float)
    args = parser.parse_args()
    # Pickles are local audit artefacts; never load an untrusted downloaded pickle.
    states = pickle.loads(args.episodes.read_bytes())
    records = records_from_episode_states(states, normalize_actions=args.normalize_actions == "true",
                                         clip_actions=args.clip_actions == "true")
    metrics = analyze_records(records, max_lag=args.max_lag, gamma=args.gamma, lambda_=args.lambda_)
    metrics["provenance"] = {"input": str(args.episodes.resolve()),
        "input_sha256": hashlib.sha256(args.episodes.read_bytes()).hexdigest(),
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "normalize_actions": args.normalize_actions == "true", "clip_actions": args.clip_actions == "true"}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "exploration_metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    with (args.output_dir / "exploration_records.jsonl").open("w", encoding="utf-8") as out:
        for record in records:
            out.write(json.dumps(record, allow_nan=False) + "\n")
    print(json.dumps({"transitions": len(records), "strata": len(metrics["strata"]),
                      "output": str(args.output_dir.resolve())}))


if __name__ == "__main__":
    main()
