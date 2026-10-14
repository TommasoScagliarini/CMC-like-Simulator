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
PRESCRIBED_CLOCK_FEATURES = ("gait_phase_sin", "gait_phase_cos")
PRESCRIBED_INDICES = (0, 1, 35, 36, 37, 38)
DEFAULT_TARGET_VELOCITY_SCALE = 10.0
RIGHT_TARGET_FEATURES = (
    "prescribed_right_shifted_knee_q_scaled", "prescribed_right_shifted_knee_qdot_scaled",
    "prescribed_right_shifted_ankle_q_scaled", "prescribed_right_shifted_ankle_qdot_scaled",
)


def reference_name(value):
    if value not in ("left", "right_v26"):
        raise ValueError("target_reference must be 'left' or 'right_v26'")
    return str(value)


def target_feature_names(target_reference="left"):
    return TARGET_FEATURES if reference_name(target_reference) == "left" else RIGHT_TARGET_FEATURES


def reference_from_contract(channels):
    sources = {"prescribed_left_phase_template": "left", "prescribed_right_phase_template": "right_v26"}
    try:
        return sources[channels["teacher_source"]]
    except (KeyError, TypeError) as exc:
        raise ValueError("unrecognized prescribed reference contract") from exc


def velocity_scale(value):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError("target velocity scale must be finite and positive")
    result = float(value)
    if not np.isfinite(result) or result <= 0.0:
        raise ValueError("target velocity scale must be finite and positive")
    return result


def target_contract(target_velocity_scale=DEFAULT_TARGET_VELOCITY_SCALE, *, target_reference="left"):
    scale = velocity_scale(target_velocity_scale)
    reference = reference_name(target_reference)
    result = {
        "version": "prescribed_left_phase_clock_q_qdot_fixed_scale_v3",
        "teacher_source": "prescribed_left_phase_template",
        "teacher_clock": "prescribed_left_grf_heel_strikes",
        "phase_shift_cycles": 0.0,
        "clock_encoding": "sin(2*pi*left_phase),cos(2*pi*left_phase)",
        "features": list(PRESCRIBED_CLOCK_FEATURES + TARGET_FEATURES),
        "actor_indices": list(PRESCRIBED_INDICES),
        "units_before_normalization": ["dimensionless", "dimensionless", "rad", "rad/s", "rad", "rad/s"],
        "target_velocity_scale": scale,
        "normalization_mean": [0.0] * 6,
        "normalization_scale": [1.0, 1.0, 1.0, scale, 1.0, scale],
        "masked_value_after_normalization": [0.0] * 6,
        "mask_scope": "episode", "mask_group": "one shared draw for all six prescribed channels",
        "availability_indicator": False,
        "critic_teacher_copies": "same six masked actor-prefix channels only",
        "running_normalization": False,
    }
    if reference == "right_v26":
        result.update(
            version="prescribed_right_v26_phase_clock_q_qdot_fixed_scale_v1",
            teacher_source="prescribed_right_phase_template",
            teacher_clock="prescribed_right_grf_heel_strikes",
            phase_shift_cycles={"pros_knee_angle": 0.465, "pros_ankle_angle": 0.452},
            clock_encoding="sin(2*pi*right_phase),cos(2*pi*right_phase)",
            clock_phase_offset_cycles=0.0,
            features=list(PRESCRIBED_CLOCK_FEATURES + RIGHT_TARGET_FEATURES),
        )
    return result


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
    result = {
        "probability": probability(getattr(args, "target_availability_probability", 0.0)),
        "curriculum_probabilities": levels, "block_iterations": block,
        "mask_seed": int(getattr(args, "target_mask_seed", 123)),
        "target_velocity_scale": velocity_scale(getattr(args, "target_velocity_scale", DEFAULT_TARGET_VELOCITY_SCALE)),
    }
    reference = reference_name(getattr(args, "target_reference", "left"))
    # Preserve the serialized configuration of existing LEFT checkpoints.
    if reference != "left":
        result["target_reference"] = reference
    offset = int(getattr(args, "target_curriculum_iteration_offset", 0))
    if offset < 0:
        raise ValueError("target curriculum iteration offset must be nonnegative")
    # Omit the default so historical checkpoint configuration equality survives.
    if offset:
        result["iteration_offset"] = offset
    return result


def probability_for_iteration(config, iteration, reward_mode):
    if reward_mode != "imitation":
        return 0.0
    levels = config["curriculum_probabilities"]
    if levels:
        index = max(0, int(iteration) - 1 - config.get("iteration_offset", 0)) // config["block_iterations"]
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
        saved_state = json.loads(source.read_text())
        prepare_curriculum_resume(saved_state, args, resume)
        iteration = int(saved_state["last_completed_iteration"]) + 1
    # env_factory enforces ex-novo after reward overrides are resolved.
    selected = config["probability"] if evaluation else probability_for_iteration(config, iteration, "imitation")
    return {"target_availability_probability": selected, "target_mask_seed": config["mask_seed"],
            "target_velocity_scale": config["target_velocity_scale"],
            "target_reference": config.get("target_reference", "left")}


def normalize_targets(raw_targets, visible, target_velocity_scale=DEFAULT_TARGET_VELOCITY_SCALE):
    """Mask AFTER immutable normalization. The reward never calls this helper."""
    raw = np.asarray(raw_targets, dtype=np.float32)
    if raw.shape != (4,) or not np.all(np.isfinite(raw)):
        raise ValueError("four finite prescribed phase-target q/qdot values required")
    scale = velocity_scale(target_velocity_scale)
    normalized = (raw.astype(float) / np.asarray([1.0, scale, 1.0, scale])).astype(np.float32)
    if not np.all(np.isfinite(normalized)):
        raise ValueError("target velocity scale produces nonfinite normalized observations")
    return normalized if bool(visible) else np.zeros(4, dtype=np.float32)


def extend_observation(base, raw_targets, visible, *, phase,
                       target_velocity_scale=DEFAULT_TARGET_VELOCITY_SCALE):
    base = np.asarray(base, dtype=np.float32)
    if base.shape != (84,):
        raise ValueError("39D feasibility requires the V1 35/84 observation contract")
    actor = base[:35].copy()
    phase = float(phase)
    if not np.isfinite(phase):
        raise ValueError("prescribed-left observation phase must be finite")
    angle = 2.0 * np.pi * phase
    actor[:2] = (np.sin(angle), np.cos(angle)) if bool(visible) else (0.0, 0.0)
    return np.concatenate((actor, normalize_targets(raw_targets, visible, target_velocity_scale), base[35:]))


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
    parser.add_argument("--target-reference", type=reference_name, choices=("left", "right_v26"), default="left",
                        help="Prescribed phase template and clock; right_v26 reproduces historical RIGHT shifts.")
    parser.add_argument("--target-availability-probability", type=probability, default=0.0)
    parser.add_argument("--target-curriculum-probabilities", type=probability, nargs="*", default=[])
    parser.add_argument("--target-curriculum-block-iterations", type=int, default=10)
    parser.add_argument("--target-curriculum-iteration-offset", type=int, default=0,
                        help="Completed global iterations before curriculum level one.")
    parser.add_argument("--resume-target-curriculum-fork", action="store_true",
                        help="Explicitly fork an imitation schedule, retaining learner and mask RNG state.")
    parser.add_argument("--target-mask-seed", type=int, default=123)
    parser.add_argument("--target-velocity-scale", type=velocity_scale, default=DEFAULT_TARGET_VELOCITY_SCALE,
                        help="Positive fixed divisor for both prescribed qdot input channels (reward unchanged).")


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


def prepare_curriculum_resume(state, args, resume_path):
    """Validate before mutation; only schedule fields can change in a new fork.

    Repeated supervisor/stage resumes with the same configuration are ordinary
    resumes: the origin and RNG stream never restart from the CLI flag.
    """
    config = configuration(args)
    saved = state.get("target_availability")
    if saved is None:
        raise ValueError("target curriculum resume requires saved availability state")
    old = saved["configuration"]
    if old == config:
        return None
    if not getattr(args, "resume_target_curriculum_fork", False):
        raise ValueError("target curriculum resume requires the identical schedule and mask seed")
    if state["reward_mode"] != "imitation":
        raise ValueError("target curriculum fork requires imitation reward mode")
    allowed = {"probability", "curriculum_probabilities", "block_iterations", "iteration_offset"}
    changed = {key for key in old.keys() | config.keys() if old.get(key) != config.get(key)}
    if changed - allowed:
        raise ValueError("target curriculum fork cannot change mask seed, scale or reference")
    source_iteration = int(state["last_completed_iteration"])
    if (config.get("iteration_offset", 0) != source_iteration
            or not config["curriculum_probabilities"]):
        raise ValueError("new target curriculum must start immediately after the source checkpoint")
    if saved.get("last_completed_iteration") != source_iteration:
        raise ValueError("target curriculum checkpoint iteration counters disagree")
    return {
        "schema": "target_curriculum_fork_v1", "source_checkpoint": str(resume_path),
        "source_iteration": source_iteration, "configuration_before": copy.deepcopy(old),
        "configuration_after": config, "first_iteration": source_iteration + 1,
        "learner_state_retained": True, "mask_rng_retained": True,
        "physical_resume": "new episode; saved mask RNG stream",
    }


def initialize_runtime(algo, args, resume_path):
    state = algo._unified_training_state
    config = configuration(args)
    saved = state.get("target_availability")
    if resume_path:
        fork = prepare_curriculum_resume(state, args, resume_path)
        if fork is not None:
            saved["configuration"] = config
            saved.setdefault("curriculum_forks", []).append(fork)
        algo._target_rng_restore = saved.get("runner_rng_states", {})
    else:
        if getattr(args, "resume_target_curriculum_fork", False) or config.get("iteration_offset", 0):
            raise ValueError("target curriculum fork/offset requires a full checkpoint resume")
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
