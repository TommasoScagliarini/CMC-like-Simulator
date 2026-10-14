"""Validate immutable completed controls before reusing their measured results.

This module only reads files. Reuse never resumes the old training run, edits its
receipts, or regenerates its evaluations. The new pilot freezes every input it
uses and records the narrowly allowed orchestration changes separately.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path


_ORCHESTRATION_FILES = frozenset({
    "Trajectory Generator/unified_MLP_39/reward_tuning/pilot.py",
    "Trajectory Generator/unified_MLP_39/reward_tuning/compare.py",
    "Trajectory Generator/unified_MLP_39/reward_tuning/__main__.py",
})
_CORE_ARTIFACTS = {"rollout_summary.json", "rollout_policy_trace.json", "paired_noise_receipt.json"}


def _require(condition, message):
    if not condition:
        raise ValueError(f"Reference reuse rejected: {message}")


def load_references(config, root, source, source_config, analysis, output):
    """Return eight verified reference cases/control branch, or None if disabled.

    ``pilot.reuse_reference_experiment`` identifies a prior experiment directory
    relative to the repository. Only its best80 and unchanged-reward control are
    reused; candidate measurements are never substituted for a control.
    """
    opts = config.get("pilot", {})
    reference = opts.get("reuse_reference_experiment")
    if not reference:
        return None
    # Runtime import prevents a cycle when pilot.prepare calls this function.
    from . import pilot
    import yaml

    root = Path(root).resolve()
    source = pilot.resolve_path(root, source)
    output = Path(output).resolve()
    experiment = pilot.resolve_path(root, reference)
    _require(not output.is_relative_to(experiment) and not experiment.is_relative_to(output),
             "new output overlaps the reference experiment")
    old_pilot = experiment / "pilot"
    manifest_path = old_pilot / "pilot_manifest.json"
    state_path = old_pilot / "pilot_state.json"
    manifest = pilot.read_json(manifest_path)
    state = pilot.read_json(state_path)
    _require(manifest.get("schema") == state.get("schema") == "reward_tuning_pilot_v1", "unknown prior schema")
    _require(state.get("manifest_sha256") == pilot.sha256(manifest_path), "old manifest SHA does not match its completed state")
    _require(state.get("status") == "completed" and not state.get("active_since_unix"), "old experiment is not completed")
    _require(pilot.resolve_path(root, manifest["repo_root"]) == root, "old repository root differs")
    _require(pilot.resolve_path(root, manifest["source_checkpoint"]) == source, "source checkpoint differs")
    iteration = pilot.checkpoint_iteration(source)
    _require(manifest.get("source_iteration") == iteration, "source iteration differs")
    _require(manifest.get("source_config") == source_config, "source configuration differs")

    hashes = {}
    orchestration_changes = {}

    def key(path):
        return path.relative_to(root).as_posix() if path.is_relative_to(root) else str(path)

    def pin(path, expected=None):
        path = pilot.resolve_path(root, path)
        _require(path.is_file(), f"missing immutable artifact {path}")
        actual = pilot.sha256(path)
        _require(expected is None or actual == expected, f"artifact hash changed: {path}")
        hashes[key(path)] = actual
        return actual

    old_hashes = manifest.get("input_hashes", {})
    _require(bool(old_hashes), "old input hashes are missing")
    for value, expected in old_hashes.items():
        path = pilot.resolve_path(root, value)
        actual = pin(path)
        if actual != expected:
            name = key(path)
            _require(name in _ORCHESTRATION_FILES, f"protected input changed: {name}")
            orchestration_changes[name] = {"before_sha256": expected, "after_sha256": actual}
    pilot.verify_hashes(root, analysis.get("input_hashes", {}))
    pin(manifest_path)
    pin(state_path)

    seed = source_config.get("model", {}).get("seed")
    _require(seed is not None and manifest.get("training_seed") == seed
             and int(opts.get("training_seed", seed)) == seed, "training seed differs")
    for name, expected in (("critic_updates", 1), ("ppo_updates", 3), ("batch_size", 4096)):
        _require(manifest.get(name) == expected and opts.get(name, expected) == expected,
                 f"{name} differs from the 1 critic + 3 PPO / 4096 protocol")
    _require(source_config.get("ppo", {}).get("train_batch_size") == 4096, "source batch size differs")
    evaluation = config.get("evaluation", {})
    seeds = list(evaluation.get("seeds", evaluation.get("noise_seeds", opts.get("noise_seeds", [123, 125, 127]))))
    env_seed = int(evaluation.get("environment_seed", opts.get("environment_seed", 123)))
    noise_scale = float(evaluation.get("noise_scale", 1.0))
    _require(seeds == [123, 125, 127] and math.isfinite(noise_scale) and noise_scale > 0,
             "evaluation seeds or noise scale are invalid")
    scenarios = [("nominal", 123), *((f"noise_{number}", number) for number in seeds)]

    branch = copy.deepcopy(manifest["branches"]["control"])
    run_dir = pilot.resolve_path(root, branch["output_dir"])
    target = iteration + 4
    _require(pilot.resolve_path(root, branch["source_checkpoint"]) == source
             and branch.get("target_iteration") == target, "control source or target iteration differs")
    _require(state.get("stages", {}).get("train_control", {}).get("status") == "completed", "control training was not completed")
    branch_path = pilot.resolve_path(root, branch["config"])
    control_config = yaml.safe_load(branch_path.read_text(encoding="utf-8"))
    expected_config = copy.deepcopy(source_config)
    expected_config.setdefault("simulation", {})["iterations"] = target
    expected_config.setdefault("supervision", {}).update(checkpoint_every=1, retain_iteration_checkpoints=True)
    expected_config.setdefault("run", {}).update(resume_from=str(source), output_dir=str(run_dir),
        resume_tuning_reward_fork=True, imitation_critic_rewarm_iterations=1,
        resume_penetration_weight_override=False)
    _require(control_config == expected_config, "control config is not the unchanged-reward source continuation")
    pin(branch_path)
    phases = ["imitation_critic_rewarm", "imitation", "imitation", "imitation"]
    phase_evidence = []
    for offset, expected_phase in enumerate(phases, start=1):
        logical = iteration + offset
        checkpoint = run_dir / f"milestone_iteration_{logical:06d}" / "checkpoint_last"
        _require(pilot.checkpoint_iteration(checkpoint) == logical, f"control update {logical} is incomplete")
        training = pilot.read_json(checkpoint / "unified_training_state.json")
        _require(training.get("last_completed_phase") == expected_phase, f"control update {logical} has the wrong phase")
        _require(training.get("imitation_critic_rewarm_remaining") == 0
                 and training.get("imitation_critic_rewarm_completed_iterations") == 1,
                 f"control update {logical} has inconsistent rewarm counters")
        pin(checkpoint / "unified_training_state.json")
        pin(checkpoint.parent / "checkpoint_last_meta.json")
        phase_evidence.append({"iteration": logical, "phase": expected_phase, "checkpoint": str(checkpoint)})
    final_checkpoint = run_dir / f"milestone_iteration_{target:06d}" / "checkpoint_last"
    latest = pilot.latest_complete_checkpoint(run_dir)
    _require(latest is not None and latest[0] == target, "control run has moved beyond its evaluated target")
    for path in final_checkpoint.rglob("*"):
        if path.is_file():
            pin(path)

    history_path = run_dir / "train_iterations.jsonl"
    rows = [json.loads(line) for line in history_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    _require([row.get("iteration") for row in rows] == list(range(iteration + 1, target + 1))
             and [row.get("unified_phase") for row in rows] == phases, "control history does not contain exactly four paired updates")
    warm_audit = rows[0].get("unified_phase_audit", {})
    _require(all(warm_audit.get(name) is True for name in ("actor_exact", "actor_optimizer_exact", "critic_changed")),
             "control rewarm does not prove unchanged actor/Adam and changed critic")
    sampled = [float(row["num_env_steps_sampled_lifetime"]) for row in rows]
    _require(all(math.isfinite(value) for value in sampled)
             and all(right - left == 4096 for left, right in zip(sampled, sampled[1:])), "control transition budget differs")
    pin(history_path)
    for name in ("resume_audit.json", "resume_optimizer_audit.json", "reward_fork_audit.json"):
        pin(run_dir / name)
    fork = pilot.read_json(run_dir / "reward_fork_audit.json")
    _require(fork.get("control_reward_unchanged") is True and fork.get("changed_reward_fields") == {}
             and fork.get("critic_rewarm_iterations") == 1
             and pilot.resolve_path(root, fork["source_checkpoint"]) == source,
             "control reward fork audit is inconsistent")
    _require(pilot.read_json(run_dir / "resume_optimizer_audit.json").get("passed") is True,
             "control optimizer restoration audit failed")

    cases_by_id = {}
    for case in manifest["evaluation_cases"]:
        if case["role"] in {"best80", "control"}:
            identifier = (case["role"], case["scenario"])
            _require(identifier not in cases_by_id, f"duplicate reference evaluation {identifier}")
            cases_by_id[identifier] = case
    _require(set(cases_by_id) == {(role, scenario) for role in ("best80", "control") for scenario, _ in scenarios},
             "reference evaluation scenarios differ")
    sigma_path = pilot.resolve_path(root, cases_by_id[("best80", "nominal")]["sigma_path"])
    sigma_digest = pin(sigma_path, state.get("sigma_sha256"))
    _require(state.get("sigma_sha256") == sigma_digest, "old state has no matching frozen sigma")
    frozen = pilot.read_json(sigma_path)
    _require(pilot.resolve_path(root, frozen["checkpoint"]) == source, "sigma came from another source policy")
    sigma = frozen.get("sigma", [])
    _require(len(sigma) == 500 and all(isinstance(row, list) and len(row) == 2
             and all(not isinstance(value, bool) and math.isfinite(float(value)) and float(value) > 0 for value in row)
             for row in sigma), "frozen sigma schedule is incomplete or invalid")

    cases, draw_hashes = [], {}
    for role in ("best80", "control"):
        for scenario, noise_seed in scenarios:
            case = copy.deepcopy(cases_by_id[(role, scenario)])
            _require(case.get("noise_seed") == noise_seed and case.get("environment_seed") == env_seed
                     and case.get("noise_scale") == noise_scale and case.get("maximum_steps") == 500,
                     f"evaluation protocol differs: {role}/{scenario}")
            expected_checkpoint = source if role == "best80" else final_checkpoint
            _require(pilot.resolve_path(root, case["checkpoint"]) == expected_checkpoint,
                     f"evaluation checkpoint differs: {role}/{scenario}")
            _require(pilot.resolve_path(root, case["sigma_path"]) == sigma_path,
                     f"sigma schedule differs: {role}/{scenario}")
            _require(case.get("freeze_sigma") is (role == "best80" and scenario == "nominal"),
                     f"sigma freeze role differs: {role}/{scenario}")
            stage_name = f"eval_{role}_{scenario}"
            _require(state["stages"].get(stage_name, {}).get("status") == "completed", f"evaluation stage incomplete: {stage_name}")
            actual = pilot.resolve_path(root, state["evaluation"][role][scenario])
            case["output_dir"] = str(actual)
            receipt_path = actual / "evaluation_complete.json"
            receipt = pilot.read_json(receipt_path)
            _require(receipt.get("role") == role and receipt.get("scenario") == scenario,
                     f"evaluation receipt identity differs: {actual}")
            artifacts = receipt.get("artifact_hashes", {})
            _require(_CORE_ARTIFACTS <= set(artifacts), f"incomplete evaluation receipt: {actual}")
            for value, digest in artifacts.items():
                artifact = pilot.resolve_path(actual, value)
                _require(artifact.is_relative_to(actual), f"evaluation artifact escapes its folder: {value}")
                pin(artifact, digest)
            pin(receipt_path)
            summary = pilot.read_json(actual / "rollout_summary.json")
            noise = pilot.read_json(actual / "paired_noise_receipt.json")
            _require(noise.get("protocol") == case, f"noise protocol receipt differs: {actual}")
            _require(noise.get("sigma_sha256") == sigma_digest, f"noise sigma receipt differs: {actual}")
            _require(pilot.resolve_path(root, summary["checkpoint"]) == expected_checkpoint,
                     f"rollout summary checkpoint differs: {actual}")
            _require(summary.get("ok") is True and receipt.get("steps") == summary.get("steps") == noise.get("draws_used")
                     and receipt.get("completed_full_episode") == summary.get("completed_full_episode"),
                     f"evaluation completion metadata differs: {actual}")
            _require(all(summary.get("evaluation_protocol", {}).get(field) == value for field, value in case.items()),
                     f"rollout summary protocol differs: {actual}")
            digest = noise.get("full_draws_sha256")
            _require(isinstance(digest, str) and len(digest) == 64, f"missing noise stream digest: {actual}")
            _require(scenario not in draw_hashes or draw_hashes[scenario] == digest, f"unpaired noise stream: {scenario}")
            draw_hashes[scenario] = digest
            if role == "best80" and scenario == "nominal":
                _require(summary.get("completed_full_episode") is True and summary.get("steps") == 500
                         and pilot.resolve_path(root, frozen["source_rollout"]) == actual,
                         "baseline nominal did not produce the full sigma schedule")
            cases.append(case)
    # Recheck the entire frozen set after the reads, including allowed files at
    # their new hashes. No output is written and no training/plot stage executes.
    pilot.verify_hashes(root, hashes)
    return {"cases": cases, "sigma_path": str(sigma_path), "branches": {"control": branch},
            "input_hashes": hashes,
            "provenance": {"schema": "reward_tuning_reference_reuse_v1", "experiment": str(experiment),
                "manifest": str(manifest_path), "manifest_sha256": pilot.sha256(manifest_path),
                "completed_state": str(state_path), "completed_state_sha256": pilot.sha256(state_path),
                "source_checkpoint": str(source), "source_iteration": iteration,
                "reused_roles": ["best80", "control"], "reused_evaluation_count": len(cases),
                "control_training_reused": True, "control_updates": phase_evidence,
                "sigma_sha256": sigma_digest, "orchestration_changes": orchestration_changes,
                "original_files_modified": False, "training_or_evaluation_launched": False}}
