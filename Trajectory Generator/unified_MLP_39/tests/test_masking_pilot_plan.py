"""Prevent accidental launches and advancing from incomplete masking stages."""
import importlib.util
import json
from pathlib import Path
import sys

import pytest


@pytest.fixture
def launcher():
    path = (Path(__file__).resolve().parents[1] /
            "runs/plans/left_best90_masking_100iter_2026-09-30/launch.py")
    spec = importlib.util.spec_from_file_location("masking_pilot_launcher_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_default_command_cannot_launch_training(launcher, monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["launch.py"])
    monkeypatch.setattr(launcher, "verify", lambda: {"training_started": False})
    def forbidden():
        pytest.fail("Default command must not start training")
    monkeypatch.setattr(launcher, "start", forbidden)
    launcher.main()
    assert json.loads(capsys.readouterr().out)["training_started"] is False


@pytest.fixture
def completed_stage(launcher, tmp_path, monkeypatch):
    monkeypatch.setattr(launcher, "ROOT", tmp_path)
    stage = dict(stage=4, output="stage04", start=166, end=190, mask=100)
    output = tmp_path / stage["output"]
    output.mkdir()
    launcher.write_json(output / "summary.json", dict(ok=True, stop_reason="completed", skipped_iterations=[]))
    rows = []
    for i in range(166, 191):
        checkpoint = output / f"milestone_iteration_{i:06d}/checkpoint_last"
        checkpoint.mkdir(parents=True)
        launcher.write_json(checkpoint / "unified_training_state.json",
                            dict(last_completed_iteration=i, target_availability=dict(current_probability=0.0)))
        rows.append(dict(iteration=i, unified_phase="imitation",
                         unified_phase_audit=dict(target_availability_batch=dict(total_steps=4096, visible_steps=0))))
    (output / "train_iterations.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
    return stage, output, rows


def test_complete_hidden_stage_is_accepted(launcher, completed_stage):
    stage, _, _ = completed_stage
    assert launcher.validate_stage(stage)["updates"] == 25


@pytest.mark.parametrize("case", ["missing_update", "visible_transition", "critic_rewarm", "wrong_probability", "skipped_update"])
def test_bad_stage_stops_chain(launcher, completed_stage, case):
    stage, output, rows = completed_stage
    if case == "missing_update":
        rows.pop(5)
    elif case == "visible_transition":
        rows[0]["unified_phase_audit"]["target_availability_batch"]["visible_steps"] = 1
    elif case == "critic_rewarm":
        rows[0]["unified_phase"] = "imitation_critic_rewarm"
    elif case == "wrong_probability":
        launcher.write_json(output / "milestone_iteration_000166/checkpoint_last/unified_training_state.json",
                            dict(last_completed_iteration=166, target_availability=dict(current_probability=0.25)))
    else:
        launcher.write_json(output / "summary.json", dict(ok=True, stop_reason="completed", skipped_iterations=[170]))
    (output / "train_iterations.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
    with pytest.raises(RuntimeError):
        launcher.validate_stage(stage)
