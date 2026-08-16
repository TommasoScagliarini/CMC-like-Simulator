"""Focused source-only orchestration tests for the V12R9 one-shot runner."""

from __future__ import annotations

import copy
import json
import os
import sys
from pathlib import Path, PurePosixPath
from typing import Any

import pytest


HERE = Path(__file__).resolve().parent
if os.fspath(HERE) not in sys.path:
    sys.path.insert(0, os.fspath(HERE))

import h0_v12r9_recovery_contract as contract  # noqa: E402
import run_h0_v12r9_recovery as runner  # noqa: E402


def _candidate_fixture() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    module = {
        "file_count": 5,
        "files": [{"path": name} for name in sorted(runner.EXPECTED_CANDIDATE_FILES)],
    }
    actor_manifest = {
        "status": contract.ACTOR_FEATURE_MANIFEST_STATUS,
        "topology_id": contract.TOPOLOGY_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "actor_feature_count": contract.EXPECTED_ACTOR_FEATURES,
        "fcnet_hiddens": [512, 512],
        "disabled_clock_columns": [0, 1],
        "actor_digest": "actor-digest",
        "module_state_sha256": "module-state-digest",
    }
    build_manifest = {
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "topology_id": contract.TOPOLOGY_ID,
        "architecture": contract.FIT["architecture"],
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "logstd_byte_exact": True,
        "disabled_clock_columns_bit_zero": True,
        "save_reload_exact": True,
        "actor_digest": "actor-digest",
        "module_state_sha256": "module-state-digest",
    }
    return module, actor_manifest, build_manifest


def test_candidate_semantic_freeze_is_five_file_and_type_strict() -> None:
    module, actor_manifest, build_manifest = _candidate_fixture()
    audit = runner._candidate_semantic_audit(module, actor_manifest, build_manifest)
    assert audit["passed"] is True
    assert all(audit["checks"].values())

    build_manifest["actor_updates"] = True
    assert (
        runner._candidate_semantic_audit(module, actor_manifest, build_manifest)[
            "passed"
        ]
        is False
    )

    module["files"] = module["files"][:-1]
    assert (
        runner._candidate_semantic_audit(module, actor_manifest, build_manifest)[
            "checks"
        ]["five_file_tree"]
        is False
    )


def test_activity_and_absolute_path_guards_are_strict(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner.freezer, "REPO_ROOT", tmp_path)
    activity = runner._zero_activity()
    runner._increment(activity, "environment_step_calls", 2)
    assert activity["environment_step_calls"] == 2
    for amount in (True, -1):
        with pytest.raises(runner.V12R9ExecutionError, match="non-negative int"):
            runner._increment(activity, "environment_step_calls", amount)
    with pytest.raises(runner.V12R9ExecutionError, match="escaped repository"):
        runner._path(tmp_path.parent / "outside.json")
    with pytest.raises(runner.V12R9ExecutionError, match="unsafe repository path"):
        runner._path(r"nested\windows-alias.json")


def _patch_ephemeral_contract(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner.freezer, "REPO_ROOT", tmp_path)
    values = {
        "ROOT": PurePosixPath("sandbox"),
        "RUN_ROOT": PurePosixPath("sandbox/run"),
        "CLAIM_PATH": PurePosixPath("sandbox/run/pipeline_claim.json"),
        "LEDGER_PATH": PurePosixPath("sandbox/run/pipeline_ledger.json"),
        "PROTOCOL_FREEZE_PATH": PurePosixPath("sandbox/protocol.json"),
        "EXECUTION_LOCK_PATH": PurePosixPath("sandbox/lock.json"),
        "CANDIDATE_MODULE_PATH": PurePosixPath("sandbox/run/fit/candidate"),
        "CANDIDATE_FREEZE_PATH": PurePosixPath("sandbox/run/candidate_freeze.json"),
        "FINAL_DEVELOPMENT_PATH": PurePosixPath("sandbox/run/final.json"),
        "STAGE_IDS": ("collect_label__synthetic",),
    }
    for name, value in values.items():
        monkeypatch.setattr(contract, name, value)
    sandbox = tmp_path / "sandbox"
    sandbox.mkdir()
    (sandbox / "protocol.json").write_text(
        json.dumps({"kind": "protocol"}), encoding="utf-8"
    )
    (sandbox / "lock.json").write_text(json.dumps({"kind": "lock"}), encoding="utf-8")
    monkeypatch.setattr(
        runner.freezer,
        "verify_protocol_freeze",
        lambda: {"passed": True},
    )
    monkeypatch.setattr(
        runner.freezer,
        "verify_execution_lock",
        lambda **_kwargs: {"passed": True},
    )
    monkeypatch.setattr(
        runner.freezer,
        "attest_locked_inputs",
        lambda: {"passed": True},
    )


def test_failed_first_stage_terminalizes_once_and_never_retries(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _patch_ephemeral_contract(monkeypatch, tmp_path)
    calls = 0

    def fail_stage(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
        nonlocal calls
        calls += 1
        raise RuntimeError("synthetic terminal failure")

    monkeypatch.setattr(runner, "_run_collection_label", fail_stage)
    with pytest.raises(runner.V12R9ExecutionError, match="stopped terminally"):
        runner.execute()
    assert calls == 1

    ledger = runner.verify_terminal_ledger()
    assert ledger["passed"] is False
    assert ledger["terminal"] is True
    assert ledger["attempted_stage"] == "collect_label__synthetic"
    assert ledger["completed_stages"] == []
    assert ledger["candidate_module_observation"] == {
        "state": "ABSENT",
        "tree": None,
    }
    assert ledger["retry_authorized"] is False
    assert ledger["resume_authorized"] is False
    assert ledger["q3_paths_opened"] == []
    assert ledger["next_stage"] == "STOP_TERMINAL_NO_RETRY"

    assert runner.execute() == ledger
    assert calls == 1


@pytest.mark.parametrize("mode", ("--execute", "--verify"))
def test_cli_returns_nonzero_for_preexisting_verified_terminal_fail(
    mode: str,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    _patch_ephemeral_contract(monkeypatch, tmp_path)
    calls = 0

    def fail_stage(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
        nonlocal calls
        calls += 1
        raise RuntimeError("synthetic terminal failure")

    monkeypatch.setattr(runner, "_run_collection_label", fail_stage)
    with pytest.raises(runner.V12R9ExecutionError, match="stopped terminally"):
        runner.execute()
    assert calls == 1

    assert runner.main([mode]) == 1
    output = json.loads(capsys.readouterr().out)
    assert output["passed"] is False
    assert output["terminal"] is True
    assert output["next_stage"] == "STOP_TERMINAL_NO_RETRY"
    assert calls == 1


def _publish_synthetic_failed_ledger(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> Path:
    _patch_ephemeral_contract(monkeypatch, tmp_path)

    def fail_stage(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
        raise RuntimeError("synthetic terminal failure")

    monkeypatch.setattr(runner, "_run_collection_label", fail_stage)
    with pytest.raises(runner.V12R9ExecutionError, match="stopped terminally"):
        runner.execute()
    return runner._path(contract.LEDGER_PATH)


def test_terminal_fail_rejects_mutated_attempted_stage(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    ledger_path = _publish_synthetic_failed_ledger(monkeypatch, tmp_path)
    ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    ledger["attempted_stage"] = "wrong_stage"
    ledger_path.unlink()
    runner.forensic.write_json_exclusive(ledger_path, ledger)
    with pytest.raises(runner.V12R9ExecutionError, match="forensic prefix"):
        runner.verify_terminal_ledger()


def test_terminal_fail_dispatches_semantic_verifier_for_forged_completed_stage(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    ledger_path = _publish_synthetic_failed_ledger(monkeypatch, tmp_path)
    ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    receipt_path = runner._path(contract.PROTOCOL_FREEZE_PATH)
    receipt_record = runner._record(receipt_path)
    ledger["completed_stages"] = [
        {"stage_id": "collect_label__synthetic", "receipt": receipt_record}
    ]
    ledger["completed_stage_count"] = 1
    ledger_path.unlink()
    runner.forensic.write_json_exclusive(ledger_path, ledger)
    monkeypatch.setattr(runner, "_stage_receipt_path", lambda _stage: receipt_path)

    def reject_semantic(_case_id: str) -> dict[str, Any]:
        raise runner.V12R9ExecutionError("synthetic completed-stage semantic drift")

    monkeypatch.setattr(runner, "verify_collection_label_receipt", reject_semantic)
    with pytest.raises(runner.V12R9ExecutionError, match="semantic drift"):
        runner.verify_terminal_ledger()


def test_terminal_verifier_rejects_locked_input_drift_at_exit(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    ledger_path = _publish_synthetic_failed_ledger(monkeypatch, tmp_path)
    assert ledger_path.exists()
    calls = 0

    def drifting_attestation() -> dict[str, Any]:
        nonlocal calls
        calls += 1
        return {"generation": calls}

    monkeypatch.setattr(runner.freezer, "attest_locked_inputs", drifting_attestation)
    with pytest.raises(runner.V12R9ExecutionError, match="locked inputs changed"):
        runner.verify_terminal_ledger()
    assert calls == 2


@pytest.mark.parametrize(
    ("field", "mutation"),
    (
        ("schema_version", -1),
        ("candidate_selection_rule", "forged-selection"),
        ("stage_order", ["forged-stage"]),
        ("next_stage", "CONTINUE_UNSAFELY"),
        ("checkpoint_zero_created", True),
        ("positive_morphology_enabled", True),
        ("candidate_id", "forged-candidate"),
        ("candidate_module", {"tree_sha256": "0" * 64}),
        ("candidate_module_observation", {"state": "FORGED", "tree": None}),
        ("candidate_freeze", {"path": "forged"}),
        ("final_development_receipt", {"path": "forged"}),
    ),
)
def test_terminal_fail_rejects_control_and_candidate_mutations(
    field: str,
    mutation: Any,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    ledger_path = _publish_synthetic_failed_ledger(monkeypatch, tmp_path)
    ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    ledger[field] = mutation
    ledger_path.unlink()
    runner.forensic.write_json_exclusive(ledger_path, ledger)
    with pytest.raises(runner.V12R9ExecutionError):
        runner.verify_terminal_ledger()


def test_terminal_fail_rejects_extra_schema_field(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    ledger_path = _publish_synthetic_failed_ledger(monkeypatch, tmp_path)
    ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    ledger["forged_extra"] = True
    ledger_path.unlink()
    runner.forensic.write_json_exclusive(ledger_path, ledger)
    with pytest.raises(runner.V12R9ExecutionError, match="schema drifted"):
        runner.verify_terminal_ledger()


def test_terminal_rejects_rehashed_semantically_forged_pipeline_claim(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    ledger_path = _publish_synthetic_failed_ledger(monkeypatch, tmp_path)
    claim_path = runner._path(contract.CLAIM_PATH)
    claim = json.loads(claim_path.read_text(encoding="utf-8"))
    claim["retry_authorized"] = True
    claim["stage_order"] = ["FORGED"]
    claim_path.unlink()
    runner.forensic.write_json_exclusive(claim_path, claim)

    ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    ledger["pipeline_claim"] = runner._record(claim_path)
    ledger_path.unlink()
    runner.forensic.write_json_exclusive(ledger_path, ledger)
    with pytest.raises(runner.V12R9ExecutionError, match="pipeline claim semantic"):
        runner.verify_terminal_ledger()


def test_attempted_worker_claim_and_claim_directory_are_semantic_exact(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _publish_synthetic_failed_ledger(monkeypatch, tmp_path)
    stage_id = contract.STAGE_IDS[0]
    worker_path = runner._stage_claim_path(stage_id)
    worker = json.loads(worker_path.read_text(encoding="utf-8"))
    worker["retry_authorized"] = True
    worker_path.unlink()
    runner.forensic.write_json_exclusive(worker_path, worker)
    with pytest.raises(runner.V12R9ExecutionError, match="worker claim semantic"):
        runner.verify_terminal_ledger()

    worker_path.unlink()
    runner.forensic.write_json_exclusive(
        worker_path, runner._expected_worker_claim(stage_id)
    )
    extra = runner._claims_root() / "99_FORGED.json"
    runner.forensic.write_json_exclusive(extra, {"forged": True})
    with pytest.raises(runner.V12R9ExecutionError, match="claims directory"):
        runner.verify_terminal_ledger()


def test_public_receipt_verifier_hooks_are_no_arg_callable() -> None:
    for hook in (
        runner.verify_candidate_freeze_receipt,
        runner.verify_final_development_receipt,
        runner.verify_terminal_ledger,
    ):
        assert callable(hook)


def test_stage_order_and_scoped_activity_distinguish_imported_and_live_env() -> None:
    assert contract.STAGE_IDS[:3] == (
        "adjudicate_r8_terminal_and_minus_prefix",
        "import_r8_plus_labels",
        "label_r8_minus_prefix",
    )
    assert (
        len(
            [
                stage
                for stage in contract.STAGE_IDS
                if stage.startswith("collect_label__")
            ]
        )
        == 4
    )
    activity = runner._zero_activity()
    runner._increment(activity, "imported_observer_label_rows", 179)
    runner._increment(activity, "adjudicated_prefix_label_rows", 252)
    runner._increment(activity, "offline_teacher_label_calls", 252)
    runner._increment_scoped(activity, "new_collection", "environment_reset_calls", 4)
    runner._increment_scoped(activity, "development", "environment_reset_calls", 6)
    assert activity["historical_prefix_environment_reset_calls"] == 0
    assert activity["historical_prefix_environment_step_calls"] == 0
    assert activity["new_collection_environment_reset_calls"] == 4
    assert activity["development_environment_reset_calls"] == 6
    assert activity["environment_reset_calls"] == 10
    assert activity["imported_observer_label_rows"] == 179
    assert activity["adjudicated_prefix_label_rows"] == 252
    assert activity["offline_teacher_label_calls"] == 252


def test_activity_contract_accepts_every_exact_stage_boundary(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    minus_receipt = runner._stage_receipt_path("label_r8_minus_prefix")

    def synthetic_mapping(path: Path) -> dict[str, int]:
        rows = 252 if path == minus_receipt else contract.EXPECTED_STEPS
        return {
            "probe_step_count": contract.EXPECTED_STEPS,
            "labelled_row_count": rows,
            "steps": contract.EXPECTED_STEPS,
        }

    monkeypatch.setattr(
        runner,
        "_mapping",
        synthetic_mapping,
    )

    for prefix_length in range(len(contract.STAGE_IDS) + 1):
        completed = list(contract.STAGE_IDS[:prefix_length])
        attempted = (
            contract.STAGE_IDS[prefix_length]
            if prefix_length < len(contract.STAGE_IDS)
            else None
        )
        completed_labels = sum(
            stage in {"import_r8_plus_labels", "label_r8_minus_prefix"}
            or stage.startswith("collect_label__")
            for stage in completed
        )
        completed_new = sum(stage.startswith("collect_label__") for stage in completed)
        completed_development = sum(
            stage.startswith("development__") for stage in completed
        )
        completed_fit = int("fit_recovery_actor" in completed)
        plus_complete = int("import_r8_plus_labels" in completed)
        minus_complete = int("label_r8_minus_prefix" in completed)
        activity = runner._zero_activity()
        activity.update(
            {
                "collection_rounds_completed": completed_labels,
                "new_collection_environment_reset_calls": completed_new,
                "new_collection_environment_step_calls": completed_new
                * contract.EXPECTED_STEPS,
                "development_environment_reset_calls": completed_development,
                "development_environment_step_calls": completed_development
                * contract.EXPECTED_STEPS,
                "offline_teacher_label_calls": minus_complete * 252
                + completed_new * contract.EXPECTED_STEPS,
                "imported_observer_label_rows": plus_complete * 179,
                "adjudicated_prefix_label_rows": minus_complete * 252,
                "new_collection_label_rows": completed_new * contract.EXPECTED_STEPS,
                "actor_fit_stage_calls": completed_fit,
                "actor_updates": completed_fit,
                "adamw_epochs_completed": completed_fit * 2000,
                "lbfgs_closure_calls": completed_fit,
                "development_rollouts_completed": completed_development,
            }
        )
        activity["environment_reset_calls"] = (
            activity["new_collection_environment_reset_calls"]
            + activity["development_environment_reset_calls"]
        )
        activity["environment_step_calls"] = (
            activity["new_collection_environment_step_calls"]
            + activity["development_environment_step_calls"]
        )
        activity["raw_sensor_sample_count"] = (
            activity["environment_step_calls"] * contract.RAW_SAMPLES_PER_STEP
        )
        ledger = {
            "activity_totals": activity,
            "collection_round_count": completed_labels,
            "actor_fit_count": completed_fit,
            "development_count": completed_development,
            "actor_updates": completed_fit,
            "critic_updates": 0,
            "ppo_updates": 0,
        }
        assert runner._prefix_activity_coherent(
            ledger,
            completed_ids=completed,
            attempted_stage=attempted,
        ), f"activity rejected at prefix boundary {prefix_length}: {attempted}"


def _pure_development_trace(case_id: str) -> list[dict[str, Any]]:
    config = runner._physical_config()
    return [
        {
            "step": step,
            "schema_version": config.schema_version,
            "protocol_id": config.protocol_id,
            "stage_id": f"development__{case_id}",
            "case_id": case_id,
            "candidate_mean": [0.25, -0.5],
            "single_noise": [0.01, -0.02],
            "raw_action": [0.26, -0.52],
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            **{name: 0 for name in runner.physical.PURE_POLICY_COUNTER_FIELDS},
            "raw_sensor_sample_count": 10,
            "observer_raw_sensor_journal": {"samples": [{} for _ in range(10)]},
        }
        for step in range(1, runner.physical.EXPECTED_STEPS + 1)
    ]


def test_development_runtime_evidence_is_recomputed_from_trace_and_summary() -> None:
    case_id = contract.DEVELOPMENT_CASE_IDS[0]
    trace = _pure_development_trace(case_id)
    summary = {
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "morphology_weight": 0.0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    audit, checks = runner._recompute_development_runtime_evidence(
        summary=summary, trace=trace, case_id=case_id
    )
    assert audit["passed"] is True
    assert all(checks.values())

    forged_trace = copy.deepcopy(trace)
    forged_trace[7]["teacher_mean"] = [0.0, 0.0]
    forged_summary = {**summary, "morphology_weight": 1.0, "actor_updates": 7}
    forged_audit, forged_checks = runner._recompute_development_runtime_evidence(
        summary=forged_summary,
        trace=forged_trace,
        case_id=case_id,
    )
    assert forged_audit["passed"] is False
    assert forged_checks["pure_trace"] is False
    assert forged_checks["morphology_zero"] is False
    assert forged_checks["zero_updates"] is False


def test_write_surface_rejects_every_non_r9_namespace(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner.freezer, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(contract, "ROOT", PurePosixPath("validation/v12r9"))
    with pytest.raises(runner.V12R9ExecutionError, match="exclusive R9 namespace"):
        runner._write(
            PurePosixPath("validation/v12r7/forbidden.json"), {"forbidden": True}
        )
    assert not (tmp_path / "validation/v12r7/forbidden.json").exists()
