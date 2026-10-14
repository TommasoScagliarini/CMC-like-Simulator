"""Direct actor initialization and checkpointed training-phase bookkeeping."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from unified_MLP_39 import actor_transition, target_availability

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


def _actor_optimizer_on_learner(learner):
    """Audit actor Adam slots without including critic slots or learning rate."""
    module = learner.module["default_policy"]
    actor_ids = {id(parameter) for parameter in module.pi.parameters()}
    names = {id(parameter): name for name, parameter in module.named_parameters()
             if id(parameter) in actor_ids}
    reports = {}
    covered = set()
    for optimizer_name, optimizer in sorted(learner._named_optimizers.items()):
        parameters = [parameter for group in optimizer.param_groups for parameter in group["params"]
                      if id(parameter) in actor_ids]
        if not parameters:
            continue
        h = hashlib.sha256()
        for parameter in sorted(parameters, key=lambda p: names[id(p)]):
            covered.add(id(parameter))
            h.update(names[id(parameter)].encode())
            slots = optimizer.state.get(parameter)
            h.update(b"missing" if slots is None else b"present")
            for key, value in sorted((slots or {}).items()):
                array = _array(value)
                h.update(key.encode())
                h.update(str(array.dtype).encode())
                h.update(str(array.shape).encode())
                h.update(array.tobytes())
        reports[optimizer_name] = {"actor_parameter_count": len(parameters),
                                   "state_digest": h.hexdigest()}
    if covered != actor_ids:
        raise RuntimeError("Critic rewarm audit could not locate all actor optimizer parameters")
    return reports


def training_phase(state):
    if state.get("imitation_critic_rewarm_remaining", 0) > 0:
        if state["reward_mode"] != "imitation":
            raise ValueError("Imitation critic rewarm requires imitation reward mode")
        return "imitation_critic_rewarm"
    return state["reward_mode"] if state["critic_warmup_completed"] else "critic_warmup"


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
        from unified_MLP_39 import reward_resume
        state, reward_fork = reward_resume.prepare_resume_state(
            state, reward_config,
            allow_override=getattr(args, "resume_penetration_weight_override", False),
            source=path, output=output_dir,
            allow_tuning_fork=getattr(args, "resume_tuning_reward_fork", False),
            critic_rewarm_iterations=getattr(args, "imitation_critic_rewarm_iterations", 0),
        )
        if state["schema_version"] != actor_transition.SCHEMA_VERSION:
            raise ValueError("Not a unified39 v3 masked prescribed-left phase checkpoint")
        restored = driver._learner_module_state(algo)
        parameter_audit = reward_resume.audit_restored_parameters(path, restored, critic_digest)
        if reward_fork is not None:
            reward_fork.update(parameter_audit)
            reward_fork["live_reward_audit"] = reward_resume.audit_live_rewards(algo, args, reward_config)
            reward_fork["cleared_old_return_metrics"] = reward_resume.clear_old_return_metrics(algo)
            write_json(Path(output_dir) / "reward_fork_audit.json", reward_fork)
        write_json(Path(output_dir) / "resume_audit.json", {
            "source_checkpoint": str(path), "actor_reinitialized": False,
            "critic_warmup_completed": state["critic_warmup_completed"],
            "next_phase": training_phase(state),
            "imitation_critic_rewarm_remaining": state.get("imitation_critic_rewarm_remaining", 0),
            "restored_actor_digest": actor_transition.actor_state_digest(restored),
            "restored_mean_digest": actor_transition.actor_mean_digest(restored),
            "restored_critic_digest": critic_digest(restored),
            "parameter_audit": parameter_audit,
            "reward_changed": bool(reward_fork and reward_fork["changed_reward_fields"]),
            "reward_fork_created": reward_fork is not None,
        })
    else:
        state = {
            "schema_version": actor_transition.SCHEMA_VERSION, "reward_mode": mode,
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
    phase = training_phase(state)
    set_phase(algo, phase, driver, args)
    curriculum_fork = None
    if resume_path is not None:
        curriculum_fork = target_availability.prepare_curriculum_resume(state, args, resume_path)
        if curriculum_fork is not None:
            reward_resume._require_fresh_fork_output(resume_path, output_dir)
    target_availability.initialize_runtime(algo, args, resume_path)
    if curriculum_fork is not None:
        write_json(Path(output_dir) / "curriculum_fork_audit.json", curriculum_fork)
    write_json(Path(output_dir) / CONTRACT_FILE, contract)
    write_json(Path(output_dir) / STATE_FILE, state)


def before_iteration(algo, args, driver):
    availability_audit = target_availability.before_iteration(algo, args)
    state = algo._unified_training_state
    phase = training_phase(state)
    set_phase(algo, phase, driver, args)
    # Critic learning starts on the new objective with no actor gradients.
    lr = 1e-4 if phase == "critic_warmup" else float(args.lr)
    driver._reapply_optimizer_learning_rate(algo, lr)
    before = driver._learner_module_state(algo)
    audit = {
        "phase": phase,
        "target_availability": availability_audit,
        "actor_before": actor_transition.actor_state_digest(before),
        "mean_before": actor_transition.actor_mean_digest(before),
        "critic_before": critic_digest(before),
    }
    if phase == "imitation_critic_rewarm":
        audit["actor_optimizer_before"] = driver._learner_call_results(algo, _actor_optimizer_on_learner)
        if not audit["actor_optimizer_before"]:
            raise RuntimeError("No learner available for critic rewarm optimizer audit")
    return audit


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
    if before["phase"] == "imitation_critic_rewarm":
        audit["actor_optimizer_after"] = driver._learner_call_results(algo, _actor_optimizer_on_learner)
        audit["actor_optimizer_exact"] = audit["actor_optimizer_before"] == audit["actor_optimizer_after"]
        if not audit["actor_exact"] or not audit["critic_changed"] or not audit["actor_optimizer_exact"]:
            raise RuntimeError(f"Imitation critic rewarm failed: {audit}")
        state["imitation_critic_rewarm_remaining"] -= 1
        state["imitation_critic_rewarm_completed_iterations"] = state.get("imitation_critic_rewarm_completed_iterations", 0) + 1
        set_phase(algo, training_phase(state), driver, args)
        audit["imitation_critic_rewarm_remaining"] = state["imitation_critic_rewarm_remaining"]
    if before["phase"] == "critic_warmup":
        if not audit["actor_exact"] or not audit["critic_changed"]:
            raise RuntimeError(f"Critic-only warm-up failed: {audit}")
        state["critic_warmup_completed"] = True
        set_phase(algo, "ex_novo", driver, args)
        driver._reapply_optimizer_learning_rate(algo, float(args.lr))
    state["last_completed_phase"] = before["phase"]
    state["last_completed_iteration"] = int(iteration)
    audit["target_availability_batch"] = target_availability.after_iteration(algo, iteration, args)
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
