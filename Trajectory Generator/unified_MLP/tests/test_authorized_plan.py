"""The long-running controller must reject incomplete or misleading milestones."""
import json

import pytest

from unified_MLP.validation.run_authorized_plan import validate_training


def training_result(tmp_path):
    def write(path, data):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data))
    summary = {"ok": True, "stop_reason": "completed", "iterations_completed": 2, "skipped_iterations": []}
    write(tmp_path / "summary.json", summary)
    rows = [{"iteration": n, "unified_phase": "imitation", "exact_start_balance": {"pass": True}} for n in (1, 2)]
    (tmp_path / "train_iterations.jsonl").write_text("\n".join(json.dumps(row) for row in rows))
    milestone = tmp_path / "milestone_iteration_000002"
    write(milestone / "checkpoint_last_meta.json", {"logical_iteration": 2})
    write(milestone / "checkpoint_last" / "unified_training_state.json", {"last_completed_iteration": 2, "reward_mode": "imitation"})
    return summary, rows


def test_completed_target_requires_consistent_artifacts(tmp_path):
    training_result(tmp_path)
    assert validate_training(tmp_path, 2)["iterations_completed"] == 2
    with pytest.raises(ValueError, match="different iteration target"):
        validate_training(tmp_path, 50)


@pytest.mark.parametrize("change", ["missing", "duplicate", "wrong_phase", "unbalanced", "skipped", "failed", "checkpoint_mismatch"])
def test_controller_rejects_false_completion(tmp_path, change):
    summary, rows = training_result(tmp_path)
    if change == "missing":
        rows.pop(0)
    elif change == "duplicate":
        rows.append(rows[-1])
    elif change == "wrong_phase":
        rows[-1]["unified_phase"] = "ex_novo"
    elif change == "unbalanced":
        rows[-1]["exact_start_balance"]["pass"] = False
    elif change == "skipped":
        summary["skipped_iterations"] = [1]
    elif change == "failed":
        summary["ok"] = False
    elif change == "checkpoint_mismatch":
        (tmp_path / "milestone_iteration_000002/checkpoint_last_meta.json").write_text('{"logical_iteration":1}')
    (tmp_path / "summary.json").write_text(json.dumps(summary))
    (tmp_path / "train_iterations.jsonl").write_text("\n".join(json.dumps(row) for row in rows))
    with pytest.raises(ValueError):
        validate_training(tmp_path, 2)


def test_sigma_evaluation_copy_preserves_source_mean_and_detects_tampering(tmp_path):
    import pickle
    import numpy as np
    from unified_MLP.validation.run_feasibility39 import constant_sigma_export
    from unified_MLP.actor_transition import actor_mean_digest
    variant = tmp_path / "unified_MLP_39"
    run = variant / "runs/training/fixture"
    module = run / "milestone_iteration_000050/rl_module_last"
    module.mkdir(parents=True)
    state = {"pi.1.weight": np.arange(12, dtype=np.float32).reshape(4, 3),
             "pi.1.bias": np.arange(4, dtype=np.float32), "vf.weight": np.ones((1, 3))}
    original_bytes = pickle.dumps(state)
    (module / "module_state.pkl").write_bytes(original_bytes)
    (run / "training_cfg.resolved.yaml").write_text("simulation:\n  iterations: 50\n")
    output = constant_sigma_export(run, 50, variant)
    adjusted = pickle.loads((output / "module_state.pkl").read_bytes())
    assert (module / "module_state.pkl").read_bytes() == original_bytes
    assert actor_mean_digest(state) == actor_mean_digest(adjusted)
    assert np.all(adjusted["pi.1.weight"][2:] == 0)
    assert np.exp(adjusted["pi.1.bias"][2:]) == pytest.approx([.010, .010])
    assert constant_sigma_export(run, 50, variant) == output
    (output / "module_state.pkl").write_bytes(b"tampered")
    with pytest.raises(ValueError, match="weights changed"):
        constant_sigma_export(run, 50, variant)


def test_common_noise_audit_checks_realized_noise_and_sigma():
    import copy
    from unified_MLP.validation.run_feasibility39 import ALL_STARTS, NOISE_SEEDS, verify_common_noise
    rollouts = {(arm, start, seed): {"rows": [{"exploration_action_noise": [.003, -.007], "policy_action_std": [.010, .010]}]}
                for arm in ("A", "B", "C") for start in ALL_STARTS for seed in NOISE_SEEDS}
    assert len(verify_common_noise(rollouts)) == len(ALL_STARTS) * len(NOISE_SEEDS)
    changed = copy.deepcopy(rollouts)
    changed[("B", "early", NOISE_SEEDS[0])]["rows"][0]["exploration_action_noise"][0] += .0001
    with pytest.raises(ValueError, match="noise differs"):
        verify_common_noise(changed)
    changed = copy.deepcopy(rollouts)
    changed[("B", "early", NOISE_SEEDS[0])]["rows"][0]["policy_action_std"][0] = .02
    with pytest.raises(ValueError, match="fixed sigma"):
        verify_common_noise(changed)
