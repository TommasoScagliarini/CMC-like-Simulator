"""Versioned target-channel contract and episode-level availability.

Payload for the deferred unified_MLP_39 package. No learned normalization, no
forward-time randomness, and no raw teacher copy in the privileged suffix.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path

import numpy as np

TARGET_FEATURES = (
    "prescribed_left_knee_q_scaled", "prescribed_left_knee_qdot_scaled",
    "prescribed_left_ankle_q_scaled", "prescribed_left_ankle_qdot_scaled",
)
TARGET_SCALES = (1.0, 10.0, 1.0, 10.0)
TARGET_CONTRACT = {
    "version": "prescribed_left_current_q_qdot_fixed_scale_v1",
    "features": list(TARGET_FEATURES), "actor_indices": [35, 36, 37, 38],
    "units_before_normalization": ["rad", "rad/s", "rad", "rad/s"],
    "normalization_mean": [0.0] * 4,
    "normalization_scale": list(TARGET_SCALES),
    "masked_value_after_normalization": [0.0] * 4,
    "mask_scope": "episode", "availability_indicator": False,
    "critic_teacher_copies": "same four masked actor-prefix channels only",
    "running_normalization": False,
}


def probability(value):
    result = float(value)
    if not np.isfinite(result) or not 0.0 <= result <= 1.0:
        raise ValueError("target availability probability must be finite in [0, 1]")
    return result


def configuration(args):
    levels = [probability(x) for x in getattr(args, "target_curriculum_probabilities", [])]
    block = int(getattr(args, "target_curriculum_block_iterations", 10))
    if block < 1:
        raise ValueError("target curriculum block iterations must be positive")
    return {
        "probability": probability(getattr(args, "target_availability_probability", 0.0)),
        "curriculum_probabilities": levels, "block_iterations": block,
        "mask_seed": int(getattr(args, "target_mask_seed", 123)),
    }


def probability_for_iteration(config, iteration, reward_mode):
    if reward_mode != "imitation":
        return 0.0
    levels = config["curriculum_probabilities"]
    if levels:
        index = max(0, int(iteration) - 1) // config["block_iterations"]
        return levels[min(index, len(levels) - 1)]
    return config["probability"]


def env_options(args, *, evaluation=False):
    """Rollout uses the explicit probability; training uses its fixed schedule."""
    config = configuration(args)
    iteration = 1
    resume = getattr(args, "resume_from", None)
    if resume and not evaluation:
        location = Path(resume).expanduser()
        candidates = [location / "unified_training_state.json"]
        if not location.is_absolute():
            from unified_MLP_39._bootstrap import UNIFIED_DIR
            candidates.append(UNIFIED_DIR / "runs" / "training" / location / "unified_training_state.json")
        source = next((p for p in candidates if p.is_file()), None)
        if source is None:
            raise ValueError("cannot resolve target curriculum resume metadata")
        iteration = int(json.loads(source.read_text())["last_completed_iteration"]) + 1
    # env_factory enforces ex-novo after reward overrides are resolved.
    selected = config["probability"] if evaluation else probability_for_iteration(config, iteration, "imitation")
    return {"target_availability_probability": selected, "target_mask_seed": config["mask_seed"]}


def normalize_targets(raw_targets, visible):
    """Mask AFTER immutable normalization. The reward never calls this helper."""
    raw = np.asarray(raw_targets, dtype=np.float32)
    if raw.shape != (4,) or not np.all(np.isfinite(raw)):
        raise ValueError("four finite same-time prescribed q/qdot values required")
    normalized = raw / np.asarray(TARGET_SCALES, dtype=np.float32)
    return normalized if bool(visible) else np.zeros(4, dtype=np.float32)


def extend_observation(base, raw_targets, visible):
    base = np.asarray(base, dtype=np.float32)
    if base.shape != (84,):
        raise ValueError("39D feasibility requires the V1 35/84 observation contract")
    actor = base[:35].copy()
    actor[:2] = 0.0
    return np.concatenate((actor, normalize_targets(raw_targets, visible), base[35:]))


class EpisodeAvailability:
    """Independent RNG, serialized explicitly because RLlib omits env RNGs."""
    def __init__(self, availability_probability=0.0, seed=123, reward_mode="imitation"):
        self.seed = int(seed)
        self.reward_mode = str(reward_mode)
        self.rng = np.random.default_rng(self.seed)
        self.visible = False
        self.episode_probability = 0.0
        self.draws = 0
        self.total_steps = 0
        self.visible_steps = 0
        self.restored = False
        self.set_probability(availability_probability)

    def set_probability(self, value):
        self.probability = probability(value) if self.reward_mode == "imitation" else 0.0
        if self.reward_mode != "imitation":
            self.visible = False

    def reset_seed(self, seed):
        # The first RLlib reset supplies a seed even after checkpoint restore.
        # Retain the saved mask RNG stream across that one initialization reset.
        if self.restored:
            self.restored = False
        elif seed is not None:
            self.rng = np.random.default_rng(np.random.SeedSequence([self.seed, int(seed)]))
            self.draws = 0

    def begin_episode(self):
        self.episode_probability = self.probability
        self.visible = bool(self.rng.random() < self.probability)
        self.draws += 1
        return self.visible

    def record_transition(self):
        self.total_steps += 1
        self.visible_steps += int(self.visible)

    def transition_counts(self):
        return {"total_steps": self.total_steps, "visible_steps": self.visible_steps}

    def get_state(self):
        return {
            "schema": "episode_target_availability_v1", "seed": self.seed,
            "reward_mode": self.reward_mode, "probability": self.probability,
            "visible": self.visible, "draws": self.draws,
            "episode_probability": self.episode_probability,
            **self.transition_counts(),
            "rng_state": copy.deepcopy(self.rng.bit_generator.state),
        }

    def restore_rng(self, state):
        if state["schema"] != "episode_target_availability_v1" or state["seed"] != self.seed:
            raise ValueError("incompatible target availability RNG checkpoint")
        if state["reward_mode"] != self.reward_mode:
            raise ValueError("target RNG resume cannot change objective")
        self.rng.bit_generator.state = copy.deepcopy(state["rng_state"])
        self.draws = int(state["draws"])
        self.total_steps = int(state["total_steps"])
        self.visible_steps = int(state["visible_steps"])
        self.restored = True
        # New process starts a new physical episode. Its next reset draws a new
        # mask at the CURRENT curriculum probability from the saved RNG stream.


def add_cli_arguments(parser):
    parser.add_argument("--target-availability-probability", type=probability, default=0.0)
    parser.add_argument("--target-curriculum-probabilities", type=probability, nargs="*", default=[])
    parser.add_argument("--target-curriculum-block-iterations", type=int, default=10)
    parser.add_argument("--target-mask-seed", type=int, default=123)


def _runner_environments(runner):
    root = getattr(runner, "env", None)
    if root is None:
        return []
    # RLlib wraps SyncVectorEnv in DictInfoToList. Vector wrappers do not
    # forward `envs`, so unwrap the vector BEFORE looking for its children.
    root = getattr(root, "unwrapped", root)
    environments = getattr(root, "envs", None)
    if environments is None and hasattr(root, "num_envs"):
        raise ValueError("target availability requires locally accessible vector environments")
    environments = [getattr(env, "unwrapped", env)
                    for env in (environments if environments is not None else [root])]
    if len(environments) != getattr(root, "num_envs", len(environments)):
        raise ValueError("target availability EnvRunner vector width disagrees with local environments")
    if any(not hasattr(env, "_target_availability") for env in environments):
        raise ValueError("target availability controller missing from a sampling environment")
    return environments


def _runner_apply(runner, selected, restore):
    worker = str(getattr(runner, "worker_index", 0) or 0)
    states = restore.get(worker, [])
    environments = _runner_environments(runner)
    if states and len(states) != len(environments):
        raise ValueError("mask RNG checkpoint EnvRunner vector width changed")
    for index, env in enumerate(environments):
        env._target_availability.set_probability(selected)
        if states:
            env._target_availability.restore_rng(states[index])
    return {"worker_index": worker, "environments": len(environments), "probability": selected,
            "transition_counts": [env._target_availability.transition_counts() for env in environments]}


def _runner_snapshot(runner):
    return {
        "worker_index": str(getattr(runner, "worker_index", 0) or 0),
        "states": [env._target_availability.get_state() for env in _runner_environments(runner)],
    }


def initialize_runtime(algo, args, resume_path):
    state = algo._unified_training_state
    config = configuration(args)
    saved = state.get("target_availability")
    if resume_path:
        if saved is None or saved["configuration"] != config:
            raise ValueError("target curriculum resume requires the identical schedule and mask seed")
        algo._target_rng_restore = saved.get("runner_rng_states", {})
    else:
        state["target_availability"] = {
            "configuration": config, "last_completed_iteration": 0,
            "runner_rng_states": {}, "physical_resume": "new episode; saved mask RNG stream",
        }
        algo._target_rng_restore = {}


def before_iteration(algo, args):
    state = algo._unified_training_state
    data = state["target_availability"]
    iteration = int(state["last_completed_iteration"]) + 1
    selected = probability_for_iteration(data["configuration"], iteration, state["reward_mode"])
    restore = getattr(algo, "_target_rng_restore", {})
    results = algo.env_runner_group.foreach_env_runner(
        lambda runner: _runner_apply(runner, selected, restore),
        local_env_runner=True, timeout_seconds=args.startup_timeout_s,
    )
    if not results or not any(item["environments"] for item in results):
        raise RuntimeError("target availability was not applied to any sampling environment")
    # Fail closed if a saved remote worker disappeared or was not restored.
    actual = {item["worker_index"] for item in results if item["environments"]}
    missing = {key for key, value in restore.items() if value} - actual
    requested_workers = int(getattr(args, "num_env_runners", 0) or 0)
    if requested_workers:
        missing |= {str(i) for i in range(1, requested_workers + 1)} - actual
    if missing:
        raise ValueError(f"missing target mask RNG workers on resume: {sorted(missing)}")
    algo._target_rng_restore = {}
    algo._target_batch_counts = {item["worker_index"]: item["transition_counts"] for item in results}
    data["current_probability"] = selected
    data["next_iteration"] = iteration
    return {"probability": selected, "next_iteration": iteration, "runners": results}


def after_iteration(algo, iteration, args):
    results = algo.env_runner_group.foreach_env_runner(
        _runner_snapshot, local_env_runner=True, timeout_seconds=args.startup_timeout_s,
    )
    data = algo._unified_training_state["target_availability"]
    actual = {item["worker_index"] for item in results if item["states"]}
    requested_workers = int(getattr(args, "num_env_runners", 0) or 0)
    if requested_workers and not {str(i) for i in range(1, requested_workers + 1)} <= actual:
        raise RuntimeError("incomplete per-worker target mask RNG snapshot")
    data["runner_rng_states"] = {item["worker_index"]: item["states"] for item in results}
    data["last_completed_iteration"] = int(iteration)
    before = algo._target_batch_counts
    workers = {}
    for item in results:
        key = item["worker_index"]
        previous = before.get(key, [])
        if len(previous) != len(item["states"]):
            raise RuntimeError("target availability counter topology changed during sampling")
        deltas = []
        for old, current in zip(previous, item["states"]):
            delta = {field: current[field] - old[field] for field in ("total_steps", "visible_steps")}
            if not 0 <= delta["visible_steps"] <= delta["total_steps"]:
                raise RuntimeError("target availability transition counters moved backwards")
            deltas.append(delta)
        workers[key] = {"total_steps": sum(x["total_steps"] for x in deltas),
                        "visible_steps": sum(x["visible_steps"] for x in deltas),
                        "environments": deltas}
    total = sum(x["total_steps"] for x in workers.values())
    visible = sum(x["visible_steps"] for x in workers.values())
    expected = getattr(args, "train_batch_size", None)
    batch = {"iteration": int(iteration), "total_steps": total, "visible_steps": visible,
             "visible_fraction": visible / total if total else None,
             "expected_train_batch_size": expected,
             "matches_expected_train_batch_size": total == expected if expected is not None else None,
             "workers": workers}
    data["last_batch"] = batch
    return batch
