from __future__ import annotations

import inspect
from pathlib import Path

import h0_primary_split_v9_hybrid_teacher_probe_contract as contract
import run_h0_primary_split_v9_hybrid_teacher_probe as runner


def test_runner_binds_fresh_one_shot_paths() -> None:
    assert runner.RUN_ROOT == runner.resolve_relative(contract.RUN_ROOT)
    assert runner.ROLLOUT_ROOT == runner.resolve_relative(contract.ROLLOUT_ROOT)
    assert runner.SOURCE_H0 == runner.resolve_relative(
        contract.SOURCE_H0_MODULE_PATH
    )
    assert "v9_hybrid_teacher_probe" in str(runner.RUN_ROOT)


def test_worker_command_reenters_probe_and_exact_destination() -> None:
    token = "x" * 32
    command = runner._worker_command(token)
    assert Path(command[1]).resolve() == Path(runner.__file__).resolve()
    assert command[2:5] == ["--worker", "--output-dir", str(runner.ROLLOUT_ROOT)]
    assert command[-2:] == ["--execution-token", token]


def test_preflight_is_read_only_and_freezes_expected_routing() -> None:
    payload = runner.build_preflight(require_unoccupied=False)
    assert payload["checks"]["v9r1_terminal_fail_preserved"] is True
    assert payload["checks"]["v9_causal_teacher_pass_preserved"] is True
    assert payload["checks"]["binary_active_v26"] is True
    assert payload["checks"]["left_primary_only"] is True
    assert payload["checks"]["morphology_zero"] is True
    assert payload["teacher"] == {
        "teacher_id": contract.TEACHER_ID,
        "construction": "from_current_shadow",
        "privileged_indices": [10, 11],
        "served_action": "FROZEN_H0_DETERMINISTIC_MEAN",
    }
    assert payload["environment_reset_calls"] == 0
    assert payload["environment_step_calls"] == 0
    assert payload["actor_updates"] == 0
    assert payload["protected_trials_opened"] == []


def test_worker_serves_h0_teacher_mean_not_student_or_candidate() -> None:
    source = inspect.getsource(runner._execute_worker)
    assert "from_current_shadow" in source
    assert "env.step(mean)" in source
    assert "P0_MODULE" not in source
    assert "candidate.eval" not in source
    assert "PRIVILEGED_INDICES" in source


def test_progress_exposes_bar_elapsed_and_eta() -> None:
    source = inspect.getsource(runner._progress)
    assert "bar" in source
    assert "elapsed=" in source
    assert "eta=" in source
