"""Reference reuse verifies frozen evidence without touching prior experiments."""
import copy
import json
from pathlib import Path

import pytest
import yaml

from unified_MLP_39.reward_tuning import pilot
from unified_MLP_39.reward_tuning.references import load_references


@pytest.fixture
def prior(tmp_path, monkeypatch):
    root = tmp_path
    package = root / "Trajectory Generator/unified_MLP_39"
    package.mkdir(parents=True)
    monkeypatch.setattr(pilot, "PACKAGE", package)
    snapshot = {"reward": {"reward_mode": "imitation", "grf_penetration_weight": 2.},
                "simulation": {"episode_duration": 5., "segment_duration": .01},
                "target_availability": {"target_reference": "left", "target_availability_probability": 1.},
                "ppo": {"train_batch_size": 4096}, "model": {"seed": 123}}

    def checkpoint(path, iteration):
        path.mkdir(parents=True, exist_ok=True)
        for name in ("algorithm_state.pkl", "rllib_checkpoint.json", "unified_contract.json", "unified_learner_state.json"):
            (path / name).write_text("{}")
        (path / "training_cfg.resolved.yaml").write_text(yaml.safe_dump(snapshot))
        pilot.write_json(path / "unified_training_state.json", {"last_completed_iteration": iteration,
            "imitation_critic_rewarm_remaining": 0, "imitation_critic_rewarm_completed_iterations": 1,
            "last_completed_phase": "imitation_critic_rewarm" if iteration == 81 else "imitation"})
        pilot.write_json(path.parent / f"{path.name}_meta.json", {"logical_iteration": iteration})
        nested = path / "learner_group/learner/rl_module/default_policy"
        nested.mkdir(parents=True, exist_ok=True)
        (nested / "module_state.pkl").write_bytes(b"actor")
        (path / "learner_group/learner/state.pkl").write_bytes(b"Adam")

    source = package / "runs/training/source/checkpoint_best"
    checkpoint(source, 80)
    simulator = root / "simulation_runner.py"
    simulator.write_text("original simulator")
    orchestration = package / "reward_tuning/pilot.py"
    orchestration.parent.mkdir()
    orchestration.write_text("original orchestration")
    experiment = package / "runs/reward_tuning/old"
    config = {"_repo_root": str(root), "source_checkpoint": str(source),
              "source_config": str(source / "training_cfg.resolved.yaml")}
    analysis = {"input_hashes": {}, "candidates": {"ankle_10": {"overrides": {"ankle_positive_torque_weight": .15}},
                                                      "ankle_5": {"overrides": {"ankle_positive_torque_weight": .075}}}}
    manifest = pilot.prepare(config, analysis, experiment / "pilot")
    state_path = experiment / "pilot/pilot_state.json"
    state = pilot.read_json(state_path)
    state["status"] = "completed"
    state["stages"]["train_control"] = {"status": "completed"}
    run_dir = Path(manifest["branches"]["control"]["output_dir"])
    rows = []
    for iteration in range(81, 85):
        checkpoint(run_dir / f"milestone_iteration_{iteration:06d}/checkpoint_last", iteration)
        rows.append({"iteration": iteration, "unified_phase": "imitation_critic_rewarm" if iteration == 81 else "imitation",
            "unified_phase_audit": {"actor_exact": True, "actor_optimizer_exact": True, "critic_changed": True},
            "num_env_steps_sampled_lifetime": iteration * 4096})
    (run_dir / "train_iterations.jsonl").write_text("\n".join(json.dumps(row) for row in rows))
    pilot.write_json(run_dir / "resume_audit.json", {"actor_reinitialized": False})
    pilot.write_json(run_dir / "resume_optimizer_audit.json", {"passed": True})
    pilot.write_json(run_dir / "reward_fork_audit.json", {"control_reward_unchanged": True,
        "changed_reward_fields": {}, "critic_rewarm_iterations": 1, "source_checkpoint": str(source)})
    sigma_path = Path(manifest["evaluation_cases"][0]["sigma_path"])
    pilot.write_json(sigma_path, {"checkpoint": str(source), "sigma": [[.1, .2]] * 500,
        "source_rollout": manifest["evaluation_cases"][0]["output_dir"]})
    state["sigma_sha256"] = pilot.sha256(sigma_path)

    def evaluation(case, complete=False):
        folder = Path(case["output_dir"])
        steps = 500 if complete else 100
        pilot.write_json(folder / "rollout_summary.json", {"checkpoint": case["checkpoint"], "ok": True,
            "steps": steps, "completed_full_episode": complete, "evaluation_protocol": {**case, "sampler": "fixed"}})
        pilot.write_json(folder / "rollout_policy_trace.json", [])
        pilot.write_json(folder / "paired_noise_receipt.json", {"protocol": case,
            "sigma_sha256": pilot.sha256(Path(case["sigma_path"])), "draws_used": steps,
            "full_draws_sha256": "a" * 64})
        artifacts = {name: pilot.sha256(folder / name) for name in (
            "rollout_summary.json", "rollout_policy_trace.json", "paired_noise_receipt.json")}
        pilot.write_json(folder / "evaluation_complete.json", {"role": case["role"], "scenario": case["scenario"],
            "steps": steps, "completed_full_episode": complete, "artifact_hashes": artifacts})

    for case in manifest["evaluation_cases"]:
        if case["role"] == "ankle_10":
            continue
        evaluation(case, complete=case["freeze_sigma"])
        state["evaluation"].setdefault(case["role"], {})[case["scenario"]] = case["output_dir"]
        state["stages"][f"eval_{case['role']}_{case['scenario']}"] = {"status": "completed"}
    pilot.write_json(state_path, state)
    config["pilot"] = {"candidate": "ankle_5", "reuse_reference_experiment": str(experiment)}
    output = package / "runs/reward_tuning/new/pilot"
    return {"root": root, "package": package, "source": source, "source_config": snapshot,
            "config": config, "analysis": analysis, "output": output, "experiment": experiment,
            "manifest": manifest, "simulator": simulator, "orchestration": orchestration,
            "checkpoint": checkpoint, "evaluation": evaluation}


def load(fixture):
    return load_references(*(fixture[name] for name in ("config", "root", "source", "source_config", "analysis", "output")))


def test_references_are_read_only_and_orchestration_changes_are_recorded(prior, monkeypatch):
    before = {path: pilot.sha256(path) for path in prior["experiment"].rglob("*") if path.is_file()}
    old_hash = pilot.sha256(prior["orchestration"])
    prior["orchestration"].write_text("updated orchestration only")
    monkeypatch.setattr(pilot, "execute_command", lambda *a, **kw: pytest.fail("reference load launched work"))
    result = load(prior)
    assert len(result["cases"]) == 8
    assert result["branches"] == {"control": prior["manifest"]["branches"]["control"]}
    change = result["provenance"]["orchestration_changes"]["Trajectory Generator/unified_MLP_39/reward_tuning/pilot.py"]
    assert change == {"before_sha256": old_hash, "after_sha256": pilot.sha256(prior["orchestration"])}
    assert not prior["output"].exists()
    assert before == {path: pilot.sha256(path) for path in before}


def test_simulator_hash_change_rejects_reuse(prior):
    prior["simulator"].write_text("changed simulator semantics")
    with pytest.raises(ValueError, match="protected input changed"):
        load(prior)


def test_evaluation_receipt_hash_change_rejects_reuse(prior):
    case = prior["manifest"]["evaluation_cases"][0]
    (Path(case["output_dir"]) / "rollout_summary.json").write_text("{}")
    with pytest.raises(ValueError, match="artifact hash changed"):
        load(prior)


def test_prepare_and_run_reuse_control_and_eight_cases_without_reexecution(prior, monkeypatch):
    manifest = pilot.prepare(prior["config"], prior["analysis"], prior["output"])
    assert set(manifest["branches"]) == {"control", "ankle_5"}
    assert manifest["branches"]["control"] == prior["manifest"]["branches"]["control"]
    commands = []

    def fake(command, *_args, **_kwargs):
        commands.append(command)
        if Path(command[1]).name == "train_ppo_mlp.py":
            cfg = yaml.safe_load(Path(command[3]).read_text())
            assert cfg["reward"]["ankle_positive_torque_weight"] == .075
            for iteration in range(81, 85):
                prior["checkpoint"](Path(cfg["run"]["output_dir"]) / f"milestone_iteration_{iteration:06d}/checkpoint_last", iteration)
        elif Path(command[1]).name == "evaluation.py":
            case = pilot.read_json(Path(command[3]))
            assert case["role"] == "ankle_5" and not case["freeze_sigma"]
            prior["evaluation"](case)
        else:
            pytest.fail("reused reference plots must not be regenerated")
        return {"returncode": 0, "elapsed_seconds": 0, "reason": "completed"}

    monkeypatch.setattr(pilot, "execute_command", fake)
    result = pilot.run(prior["output"])
    assert result["status"] == "completed"
    assert len(commands) == 5  # One new training branch and four new evaluations.
    monkeypatch.setattr(pilot, "execute_command", lambda *a, **kw: pytest.fail("completed pilot repeated work"))
    assert pilot.run(prior["output"])["status"] == "completed"


def test_no_reuse_request_performs_no_read():
    assert load_references({}, Path("missing"), Path("missing"), {}, {}, Path("missing")) is None
