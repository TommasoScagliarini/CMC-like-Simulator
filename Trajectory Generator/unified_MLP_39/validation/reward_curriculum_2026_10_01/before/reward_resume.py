"""Explicit, audited imitation reward fork with full learner state retained."""
from __future__ import annotations

import copy
import hashlib
import math
import pickle
from pathlib import Path

from unified_MLP_39 import actor_transition

ALLOWED_REWARD_KEY = "grf_penetration_weight"
TUNING_REWARD_KEYS = frozenset({
    "ankle_positive_torque_weight", "ankle_positive_torque_scale_nm",
    "knee_torque_rate_weight", "knee_torque_rate_scale_nm_s",
    "knee_kinematic_jerk_weight", "ankle_kinematic_jerk_weight",
    "kinematic_jerk_scale_rad_s3",
})


def _canonical_reward(config):
    """Only migrate newly introduced inactive defaults in old checkpoint JSON."""
    from unified_MLP_39.reward_function import RewardConfig
    defaults = RewardConfig().to_dict()
    return {**{key: defaults[key] for key in TUNING_REWARD_KEYS if key in defaults},
            **config}


def _require_fresh_fork_output(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output or output in source.parents or source in output.parents:
        raise ValueError("Reward fork requires a separate output directory")
    history = output / "train_iterations.jsonl"
    if history.exists() and history.stat().st_size:
        raise ValueError("Reward fork cannot append to an existing training history")
    return source, output


def prepare_resume_state(state, reward_config, *, allow_override, source, output,
                         allow_tuning_fork=False, critic_rewarm_iterations=0):
    """Keep ordinary resume strict; fork only explicitly selected reward families."""
    updated = copy.deepcopy(state)
    old = _canonical_reward(state["reward_config"])
    reward_config = _canonical_reward(reward_config)
    changed = sorted(k for k in old.keys() | reward_config.keys()
                     if old.get(k) != reward_config.get(k))
    if state["reward_mode"] != reward_config["reward_mode"]:
        raise ValueError("Resume cannot change reward mode")
    if (isinstance(critic_rewarm_iterations, bool)
            or not isinstance(critic_rewarm_iterations, int) or critic_rewarm_iterations < 0):
        raise ValueError("Imitation critic rewarm iterations must be a nonnegative integer")
    if allow_override and allow_tuning_fork:
        raise ValueError("Reward fork flags are mutually exclusive")
    if critic_rewarm_iterations and not allow_tuning_fork:
        raise ValueError("Imitation critic rewarm requires an explicit tuning reward fork")
    updated["reward_config"] = copy.deepcopy(reward_config)
    if allow_tuning_fork:
        if state["reward_mode"] != "imitation" or set(changed) - TUNING_REWARD_KEYS:
            raise ValueError("Tuning reward fork permits only imitation torque/kinematic tuning fields")
        for key in TUNING_REWARD_KEYS:
            value = reward_config.get(key)
            if value is None:
                raise ValueError(f"Missing tuning reward field: {key}")
            positive = not key.endswith("_weight")
            if (isinstance(value, bool) or not math.isfinite(float(value))
                    or (float(value) <= 0 if positive else float(value) < 0)):
                raise ValueError(f"Tuning reward field {key} must be finite and {'positive' if positive else 'nonnegative'}")
        # A supervisor or later continuation may carry the original flags. The
        # persisted countdown, not those flags, decides whether rewarm remains.
        previous = state.get("reward_tuning_forks", [])
        if not changed and previous:
            if critic_rewarm_iterations not in (0, previous[-1]["critic_rewarm_iterations"]):
                raise ValueError("Resume cannot change the completed/pending critic rewarm budget")
            return updated, None
        source, output = _require_fresh_fork_output(source, output)
        audit = {
            "schema": "unified39_tuning_reward_fork_v1",
            "source_checkpoint": str(source),
            "source_iteration": int(state["last_completed_iteration"]),
            "changed_reward_fields": {key: {"before": old.get(key), "after": reward_config[key]}
                                      for key in changed},
            "control_reward_unchanged": not changed,
            "critic_rewarm_iterations": critic_rewarm_iterations,
            "critic_warmup_added": critic_rewarm_iterations > 0,
            "actor_reinitialized": False, "critic_reinitialized": False,
            "optimizer_reinitialized": False,
        }
        updated["imitation_critic_rewarm_remaining"] = critic_rewarm_iterations
        updated["imitation_critic_rewarm_completed_iterations"] = 0
        updated.setdefault("reward_tuning_forks", []).append(copy.deepcopy(audit))
        return updated, audit
    if not changed:
        return updated, None
    if not allow_override:
        raise ValueError("Resume must retain its reward; an imitation penetration-weight fork requires --resume-penetration-weight-override")
    if state["reward_mode"] != "imitation" or changed != [ALLOWED_REWARD_KEY]:
        raise ValueError("Reward fork permits only grf_penetration_weight in imitation")
    value = reward_config[ALLOWED_REWARD_KEY]
    if isinstance(value, bool) or not math.isfinite(float(value)) or float(value) < 0:
        raise ValueError("Penetration weight must be finite and nonnegative")
    source, output = _require_fresh_fork_output(source, output)
    audit = {
        "schema": "unified39_penetration_reward_fork_v1",
        "source_checkpoint": str(source),
        "source_iteration": int(state["last_completed_iteration"]),
        "changed_reward_fields": {ALLOWED_REWARD_KEY: {
            "before": old[ALLOWED_REWARD_KEY], "after": value,
        }},
        "critic_warmup_added": False,
        "actor_reinitialized": False, "critic_reinitialized": False,
        "optimizer_reinitialized": False,
    }
    updated["reward_config"] = copy.deepcopy(reward_config)
    updated.setdefault("reward_forks", []).append(copy.deepcopy(audit))
    return updated, audit


def audit_restored_parameters(source, restored, critic_digest):
    path = Path(source) / "learner_group/learner/rl_module/default_policy/module_state.pkl"
    raw = path.read_bytes()
    saved = pickle.loads(raw)
    checks = {
        "actor_exact": actor_transition.actor_state_digest(saved) == actor_transition.actor_state_digest(restored),
        "actor_mean_exact": actor_transition.actor_mean_digest(saved) == actor_transition.actor_mean_digest(restored),
        "critic_exact": critic_digest(saved) == critic_digest(restored),
    }
    if not all(checks.values()):
        raise RuntimeError(f"Reward resume changed checkpoint parameters: {checks}")
    return {**checks, "source_module_sha256": hashlib.sha256(raw).hexdigest()}


def audit_runner_reward(runner, *, expected):
    """Read live wrappers after restore, rather than trusting config metadata."""
    vector = getattr(runner, "env", None)
    if vector is None:
        return {"worker_index": getattr(runner, "worker_index", None), "envs": []}
    vector = getattr(vector, "unwrapped", vector)
    envs = getattr(vector, "envs", [vector])
    rows = []
    for index, env in enumerate(envs):
        current = env
        while current is not None and "reward_config" not in vars(current):
            current = getattr(current, "env", None)
        if current is None or current.reward_config.to_dict() != expected:
            raise RuntimeError("Live EnvRunner reward does not match requested fork")
        base = env.unwrapped
        rows.append({"env_index": index, "reward_exact": True,
                     "penetration_weight": current.reward_config.grf_penetration_weight,
                     "target_reference": base.target_reference,
                     "target_velocity_scale": base.target_velocity_scale,
                     "availability_probability": base._target_availability.probability,
                     "fsm_version": base.env_cfg.binary_phase_actor_fsm_version})
    return {"worker_index": getattr(runner, "worker_index", None), "envs": rows}


def audit_live_rewards(algo, args, reward_config):
    reports = algo.env_runner_group.foreach_env_runner(
        lambda runner: audit_runner_reward(runner, expected=reward_config),
        local_env_runner=True, timeout_seconds=args.startup_timeout_s,
    )
    count = sum(len(report["envs"]) for report in reports)
    if count < max(1, int(args.num_env_runners)):
        raise RuntimeError("Reward audit did not cover every EnvRunner")
    return reports


def clear_old_return_metrics(algo):
    """Keep lifetime counters, discard reward summaries from the old objective."""
    state = algo.metrics.get_state()
    removed = []
    for key in list(state["stats"]):
        if key.startswith("env_runners--") and any(
            token in key for token in ("return", "reward", "RewardShaped")
        ):
            removed.append(key)
            del state["stats"][key]
    algo.metrics.set_state(state)
    return sorted(removed)
