"""Direct actor initialization and checkpointed training-phase bookkeeping."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from unified_MLP import actor_transition

STATE_FILE = "unified_training_state.json"
CONTRACT_FILE = "unified_contract.json"


def _array(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def critic_digest(state):
    h = hashlib.sha256()
    keys = sorted(k for k in state if k.startswith(("vf.", "vf_encoder.")))
    if not keys:
        raise ValueError("Critic state is missing")
    for key in keys:
        value = _array(state[key])
        h.update(key.encode())
        h.update(str(value.dtype).encode())
        h.update(str(value.shape).encode())
        h.update(value.tobytes())
    return h.hexdigest()


def write_json(path, payload):
    path = Path(path)
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding="utf-8")
    temp.replace(path)


def _set_phase_on_learner(learner, *, phase):
    module = learner.module["default_policy"]
    module.set_training_phase(phase)
    return module.get_training_phase_state()


def _set_phase_on_runner(runner, *, phase):
    module = runner.module
    if not hasattr(module, "set_training_phase"):
        module = module["default_policy"]
    module.set_training_phase(phase)
    return module.get_training_phase_state()


def set_phase(algo, phase, driver, args):
    driver._learner_call_results(algo, _set_phase_on_learner, phase=phase)
    algo.get_module("default_policy").set_training_phase(phase)
    algo.env_runner_group.foreach_env_runner(
        lambda runner: _set_phase_on_runner(runner, phase=phase),
        local_env_runner=True, timeout_seconds=args.startup_timeout_s,
    )


def sync_state(algo, state, driver, args):
    from ray.rllib.algorithms.algorithm import (
        COMPONENT_LEARNER, COMPONENT_LEARNER_GROUP, COMPONENT_RL_MODULE,
    )
    algo.get_module("default_policy").set_state(state)
    algo.set_state({COMPONENT_LEARNER_GROUP: {
        COMPONENT_LEARNER: {COMPONENT_RL_MODULE: {"default_policy": state}},
    }})
    driver._learner_call_results(algo, driver._advance_weights_seq_no_on_learner)
    algo.env_runner_group.sync_weights(
        from_worker_or_learner_group=algo.learner_group,
        timeout_seconds=args.startup_timeout_s, inference_only=True,
    )
    expected = actor_transition.actor_state_digest(state)
    if actor_transition.actor_state_digest(driver._learner_module_state(algo)) != expected:
        raise RuntimeError("Direct actor initialization did not reach Learner")
    from baseline_MLP.warm_start import find_actor_state
    states = algo.env_runner_group.foreach_env_runner(
        lambda runner: runner.get_state(components=[COMPONENT_RL_MODULE], inference_only=True),
        local_env_runner=True, timeout_seconds=args.startup_timeout_s,
    )
    if not states:
        raise RuntimeError("No EnvRunner actors available for initialization audit")
    for i, runner_state in enumerate(states):
        state = find_actor_state(runner_state)
        if state is None or actor_transition.actor_state_digest(state) != expected:
            raise RuntimeError(f"Direct actor initialization did not reach EnvRunner {i}")


def initialize(algo, args, output_dir, resume_path, reward_overrides, driver):
    mode = driver.reward_function.RewardConfig.from_mapping(reward_overrides).reward_mode
    reward_config = driver.reward_function.RewardConfig.from_mapping(reward_overrides).to_dict()
    contract = args._unified_contract
    algo._unified_contract = contract
    if resume_path is not None:
        path = Path(resume_path)
        source_contract = json.loads((path / CONTRACT_FILE).read_text())
        actor_transition.validate_contract(source_contract, contract)
        state = json.loads((path / STATE_FILE).read_text())
        if state["reward_mode"] != mode or state["reward_config"] != reward_config:
            raise ValueError("Resume must retain its reward; use --init-actor-from to change objective")
        if state["schema_version"] != "unified_mlp_v1":
            raise ValueError("Not a unified V1 checkpoint")
        restored = driver._learner_module_state(algo)
        write_json(Path(output_dir) / "resume_audit.json", {
            "source_checkpoint": str(path), "actor_reinitialized": False,
            "critic_warmup_completed": state["critic_warmup_completed"],
            "next_phase": mode if state["critic_warmup_completed"] else "critic_warmup",
            "restored_actor_digest": actor_transition.actor_state_digest(restored),
            "restored_mean_digest": actor_transition.actor_mean_digest(restored),
            "restored_critic_digest": critic_digest(restored),
        })
    else:
        state = {
            "schema_version": "unified_mlp_v1", "reward_mode": mode,
            "reward_config": reward_config, "critic_warmup_completed": mode == "imitation",
            "last_completed_phase": None, "last_completed_iteration": 0,
            "actor_source": None,
        }
        if args.init_actor_from:
            source, provenance = actor_transition.load_actor_source(
                args.init_actor_from, contract,
            )
            before = driver._learner_module_state(algo)
            combined, report = actor_transition.copy_actor_state(source, before, contract, contract)
            sync_state(algo, combined, driver, args)
            after = driver._learner_module_state(algo)
            if critic_digest(before) != critic_digest(after):
                raise RuntimeError("Direct actor initialization modified the fresh critic")
            state["actor_source"] = str(args.init_actor_from)
            write_json(Path(output_dir) / "actor_initialization_audit.json", {
                "provenance": provenance, "copy": report,
                "critic_unchanged": True, "optimizer": "fresh",
                "actor_exact_after_sync": True,
            })
    algo._unified_training_state = state
    phase = mode if state["critic_warmup_completed"] else "critic_warmup"
    set_phase(algo, phase, driver, args)
    write_json(Path(output_dir) / CONTRACT_FILE, contract)
    write_json(Path(output_dir) / STATE_FILE, state)


def before_iteration(algo, args, driver):
    state = algo._unified_training_state
    phase = state["reward_mode"] if state["critic_warmup_completed"] else "critic_warmup"
    set_phase(algo, phase, driver, args)
    # Critic learning starts on the new objective with no actor gradients.
    lr = 1e-4 if phase == "critic_warmup" else float(args.lr)
    driver._reapply_optimizer_learning_rate(algo, lr)
    before = driver._learner_module_state(algo)
    return {
        "phase": phase,
        "actor_before": actor_transition.actor_state_digest(before),
        "mean_before": actor_transition.actor_mean_digest(before),
        "critic_before": critic_digest(before),
    }


def after_iteration(algo, iteration, before, args, driver):
    after = driver._learner_module_state(algo)
    audit = {
        **before, "actor_after": actor_transition.actor_state_digest(after),
        "mean_after": actor_transition.actor_mean_digest(after),
        "critic_after": critic_digest(after),
    }
    audit["actor_exact"] = audit["actor_before"] == audit["actor_after"]
    audit["mean_changed"] = audit["mean_before"] != audit["mean_after"]
    audit["critic_changed"] = audit["critic_before"] != audit["critic_after"]
    state = algo._unified_training_state
    if before["phase"] == "critic_warmup":
        if not audit["actor_exact"] or not audit["critic_changed"]:
            raise RuntimeError(f"Critic-only warm-up failed: {audit}")
        state["critic_warmup_completed"] = True
        set_phase(algo, "ex_novo", driver, args)
        driver._reapply_optimizer_learning_rate(algo, float(args.lr))
    state["last_completed_phase"] = before["phase"]
    state["last_completed_iteration"] = int(iteration)
    return audit


def save_metadata(algo, path):
    path = Path(path)
    write_json(path / CONTRACT_FILE, algo._unified_contract)
    write_json(path / STATE_FILE, algo._unified_training_state)
    if hasattr(algo, "_unified_resolved_config"):
        import yaml
        (path / "training_cfg.resolved.yaml").write_text(
            yaml.safe_dump(algo._unified_resolved_config, sort_keys=False), encoding="utf-8",
        )
