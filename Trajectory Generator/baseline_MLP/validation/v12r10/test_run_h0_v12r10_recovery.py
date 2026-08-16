"""Source-only orchestration and mutation tests for the V12R10 runner."""

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

import h0_v12r10_recovery_contract as contract  # noqa: E402
import run_h0_v12r10_recovery as runner  # noqa: E402


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
        "fcnet_hiddens": [1024, 1024],
        "disabled_clock_columns": list(contract.DISABLED_CLOCK_COLUMNS),
        "actor_digest": "actor-digest",
        "module_state_sha256": "module-state-digest",
        "standard_rlmodule": True,
        "legacy_shadow_runtime_dependency": False,
    }
    build_manifest = {
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "topology_id": contract.TOPOLOGY_ID,
        "architecture": copy.deepcopy(contract.FIT["architecture"]),
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "logstd_byte_exact": True,
        "disabled_clock_columns_bit_zero": True,
        "save_reload_exact": True,
        "tower_a_r6_byte_exact": True,
        "cross_blocks_positive_zero": True,
        "actor_digest": "actor-digest",
        "module_state_sha256": "module-state-digest",
        "r6_functional_predecessor": True,
        "r9_hidden_initialization_only": True,
        "r9_terminal_candidate_promoted": False,
        "source_provenance": {
            "r6_functional_predecessor": True,
            "r9_hidden_initialization_only": True,
            "r9_promoted": False,
        },
        "no_legacy_shadow_runtime_dependency": True,
        "offline_h0_queries": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
    }
    return module, actor_manifest, build_manifest


def test_candidate_semantic_freeze_requires_standard_w1024_and_provenance() -> None:
    module, actor_manifest, build_manifest = _candidate_fixture()
    audit = runner._candidate_semantic_audit(module, actor_manifest, build_manifest)
    assert audit["passed"] is True
    assert all(audit["checks"].values())

    actor_manifest["fcnet_hiddens"] = [512, 512]
    assert (
        runner._candidate_semantic_audit(module, actor_manifest, build_manifest)[
            "checks"
        ]["actor_manifest"]
        is False
    )
    actor_manifest["fcnet_hiddens"] = [1024, 1024]
    build_manifest["r9_hidden_initialization_only"] = False
    build_manifest["source_provenance"]["r9_hidden_initialization_only"] = False
    assert (
        runner._candidate_semantic_audit(module, actor_manifest, build_manifest)[
            "checks"
        ]["provenance"]
        is False
    )


def test_activity_and_path_guards_are_type_strict(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner.freezer, "REPO_ROOT", tmp_path)
    activity = runner._zero_activity()
    runner._increment(activity, "environment_step_calls", 2)
    assert activity["environment_step_calls"] == 2
    for amount in (True, -1):
        with pytest.raises(runner.V12R10ExecutionError, match="non-negative int"):
            runner._increment(activity, "environment_step_calls", amount)
    with pytest.raises(runner.V12R10ExecutionError, match="escaped repository"):
        runner._path(tmp_path.parent / "outside.json")
    with pytest.raises(runner.V12R10ExecutionError, match="unsafe repository path"):
        runner._path(r"nested\windows-alias.json")


def _patch_ephemeral_contract(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner.freezer, "REPO_ROOT", tmp_path)
    values = {
        "ROOT": PurePosixPath("sandbox"),
        "RUN_ROOT": PurePosixPath("sandbox/run"),
        "CLAIM_PATH": PurePosixPath("sandbox/run/pipeline_claim.json"),
        "LEDGER_PATH": PurePosixPath("sandbox/run/pipeline_ledger.json"),
        "R9_IMPORT_ATTESTATION_PATH": PurePosixPath("sandbox/run/r9_import.json"),
        "FIT_RECEIPT_PATH": PurePosixPath("sandbox/run/fit/receipt.json"),
        "CANDIDATE_MODULE_PATH": PurePosixPath("sandbox/run/fit/candidate"),
        "CANDIDATE_FREEZE_PATH": PurePosixPath("sandbox/run/candidate_freeze.json"),
        "DEVELOPMENT_ROOT": PurePosixPath("sandbox/run/development"),
        "FINAL_DEVELOPMENT_PATH": PurePosixPath("sandbox/run/final.json"),
        "PROTOCOL_FREEZE_PATH": PurePosixPath("sandbox/protocol.json"),
        "EXECUTION_LOCK_PATH": PurePosixPath("sandbox/lock.json"),
        "STAGE_IDS": ("attest_r9_terminal_imports",),
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
        runner.freezer, "verify_protocol_freeze", lambda: {"passed": True}
    )
    monkeypatch.setattr(
        runner.freezer,
        "verify_execution_lock",
        lambda **_kwargs: {"passed": True, "runtime": {}},
    )
    monkeypatch.setattr(
        runner.freezer, "attest_locked_inputs", lambda: {"passed": True}
    )


def _failing_adapters(
    callback: Any,
) -> runner.RunnerAdapters:
    return runner.RunnerAdapters(
        verify_r9_inputs=callback,
        run_fit=lambda **_kwargs: {},
        verify_fit=lambda: {},
        run_physical=lambda **_kwargs: {},
    )


def test_failed_first_stage_terminalizes_once_and_never_retries(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _patch_ephemeral_contract(monkeypatch, tmp_path)
    calls = 0

    def fail_source() -> dict[str, Any]:
        nonlocal calls
        calls += 1
        raise RuntimeError("synthetic terminal failure")

    adapters = _failing_adapters(fail_source)
    with pytest.raises(runner.V12R10ExecutionError, match="stopped terminally"):
        runner.execute(adapters=adapters)
    assert calls == 1

    ledger = runner.verify_terminal_ledger(
        fit_verifier=adapters.verify_fit,
        source_verifier=adapters.verify_r9_inputs,
    )
    assert ledger["passed"] is False
    assert ledger["terminal"] is True
    assert ledger["attempted_stage"] == "attest_r9_terminal_imports"
    assert ledger["completed_stages"] == []
    assert ledger["candidate_module_observation"] == {
        "state": "ABSENT",
        "tree": None,
    }
    assert ledger["collection_round_count"] == 0
    assert ledger["retry_authorized"] is False
    assert ledger["resume_authorized"] is False
    assert ledger["fit_sweep_authorized"] is False
    assert ledger["q3_paths_opened"] == []
    assert ledger["next_stage"] == "STOP_TERMINAL_NO_RETRY"

    assert runner.execute(adapters=adapters) == ledger
    assert calls == 1


def _publish_failed_ledger(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    *,
    create_candidate: bool = False,
) -> tuple[Path, runner.RunnerAdapters]:
    _patch_ephemeral_contract(monkeypatch, tmp_path)

    def fail_source() -> dict[str, Any]:
        if create_candidate:
            candidate = runner._path(contract.CANDIDATE_MODULE_PATH)
            candidate.mkdir(parents=True)
            for name in sorted(runner.EXPECTED_CANDIDATE_FILES):
                (candidate / name).write_bytes(f"fixture:{name}".encode())
        raise RuntimeError("synthetic terminal failure")

    adapters = _failing_adapters(fail_source)
    with pytest.raises(runner.V12R10ExecutionError, match="stopped terminally"):
        runner.execute(adapters=adapters)
    return runner._path(contract.LEDGER_PATH), adapters


@pytest.mark.parametrize(
    ("field", "mutation"),
    (
        ("schema_version", -1),
        ("stage_order", ["forged"]),
        ("next_stage", "CONTINUE_UNSAFELY"),
        ("checkpoint_zero_created", True),
        ("positive_morphology_enabled", True),
        ("retry_authorized", True),
        ("candidate_id", "forged"),
        ("candidate_module_observation", {"state": "FORGED", "tree": None}),
    ),
)
def test_terminal_fail_rejects_control_mutations(
    field: str,
    mutation: Any,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    ledger_path, adapters = _publish_failed_ledger(monkeypatch, tmp_path)
    ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    ledger[field] = mutation
    ledger_path.unlink()
    runner.forensic.write_json_exclusive(ledger_path, ledger)
    with pytest.raises(runner.V12R10ExecutionError):
        runner.verify_terminal_ledger(
            fit_verifier=adapters.verify_fit,
            source_verifier=adapters.verify_r9_inputs,
        )


def test_terminal_fail_rejects_extra_schema_field(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    ledger_path, adapters = _publish_failed_ledger(monkeypatch, tmp_path)
    ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    ledger["forged_extra"] = True
    ledger_path.unlink()
    runner.forensic.write_json_exclusive(ledger_path, ledger)
    with pytest.raises(runner.V12R10ExecutionError, match="schema drifted"):
        runner.verify_terminal_ledger(
            fit_verifier=adapters.verify_fit,
            source_verifier=adapters.verify_r9_inputs,
        )


def test_terminal_ledger_keeps_complete_candidate_tree_not_projection(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    ledger_path, adapters = _publish_failed_ledger(
        monkeypatch, tmp_path, create_candidate=True
    )
    ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    module = ledger["candidate_module"]
    assert set(module) == {"path", "tree_sha256", "file_count", "files"}
    assert module["file_count"] == 5
    assert len(module["files"]) == 5
    assert ledger["candidate_module_observation"] == {
        "state": "VALID_COMPLETE_TREE",
        "tree": module,
    }

    ledger["candidate_module"] = {
        key: module[key] for key in ("path", "tree_sha256", "file_count")
    }
    ledger_path.unlink()
    runner.forensic.write_json_exclusive(ledger_path, ledger)
    with pytest.raises(runner.V12R10ExecutionError, match="candidate_full_tree"):
        runner.verify_terminal_ledger(
            fit_verifier=adapters.verify_fit,
            source_verifier=adapters.verify_r9_inputs,
        )


def test_semantically_forged_pipeline_claim_is_rejected(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    ledger_path, adapters = _publish_failed_ledger(monkeypatch, tmp_path)
    claim_path = runner._path(contract.CLAIM_PATH)
    claim = json.loads(claim_path.read_text(encoding="utf-8"))
    claim["retry_authorized"] = True
    claim_path.unlink()
    runner.forensic.write_json_exclusive(claim_path, claim)
    ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    ledger["pipeline_claim"] = runner._record(claim_path)
    ledger_path.unlink()
    runner.forensic.write_json_exclusive(ledger_path, ledger)
    with pytest.raises(runner.V12R10ExecutionError, match="pipeline claim semantic"):
        runner.verify_terminal_ledger(
            fit_verifier=adapters.verify_fit,
            source_verifier=adapters.verify_r9_inputs,
        )


def test_claim_directory_rejects_extra_worker(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _ledger_path, adapters = _publish_failed_ledger(monkeypatch, tmp_path)
    runner.forensic.write_json_exclusive(
        runner._claims_root() / "99_forged.json", {"forged": True}
    )
    with pytest.raises(runner.V12R10ExecutionError, match="claims directory"):
        runner.verify_terminal_ledger(
            fit_verifier=adapters.verify_fit,
            source_verifier=adapters.verify_r9_inputs,
        )


def test_terminal_verifier_redispatches_completed_receipt_semantics(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    ledger_path, adapters = _publish_failed_ledger(monkeypatch, tmp_path)
    ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    receipt_path = runner._path(contract.PROTOCOL_FREEZE_PATH)
    ledger["completed_stages"] = [
        {
            "stage_id": "attest_r9_terminal_imports",
            "receipt": runner._record(receipt_path),
        }
    ]
    ledger["completed_stage_count"] = 1
    ledger_path.unlink()
    runner.forensic.write_json_exclusive(ledger_path, ledger)
    monkeypatch.setattr(runner, "_stage_receipt_path", lambda _stage: receipt_path)

    def reject_semantics(**_kwargs: Any) -> dict[str, Any]:
        raise runner.V12R10ExecutionError("synthetic completed semantic drift")

    monkeypatch.setattr(runner, "verify_r9_attestation_receipt", reject_semantics)
    with pytest.raises(runner.V12R10ExecutionError, match="semantic drift"):
        runner.verify_terminal_ledger(
            fit_verifier=adapters.verify_fit,
            source_verifier=adapters.verify_r9_inputs,
        )


def test_execute_stops_at_first_physical_development_failure(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _patch_ephemeral_contract(monkeypatch, tmp_path)
    first, second = contract.DEVELOPMENT_CASE_IDS[:2]
    monkeypatch.setattr(
        contract,
        "STAGE_IDS",
        (f"development__{first}", f"development__{second}"),
    )
    calls: list[str] = []

    def fail_first(
        case_id: str,
        _activity: dict[str, int],
        _adapters: runner.RunnerAdapters,
    ) -> dict[str, Any]:
        calls.append(case_id)
        raise runner.V12R10ExecutionError(f"R10 development gate failed: {case_id}")

    monkeypatch.setattr(runner, "_run_development", fail_first)
    adapters = _failing_adapters(lambda: {})
    with pytest.raises(runner.V12R10ExecutionError, match="stopped terminally"):
        runner.execute(adapters=adapters)
    assert calls == [first]
    ledger = runner.verify_terminal_ledger(
        fit_verifier=adapters.verify_fit,
        source_verifier=adapters.verify_r9_inputs,
    )
    assert ledger["attempted_stage"] == f"development__{first}"
    assert ledger["completed_stages"] == []
    assert ledger["next_stage"] == "STOP_TERMINAL_NO_RETRY"


def test_stage_order_is_ten_stage_import_only_hardest_first() -> None:
    assert len(contract.STAGE_IDS) == 10
    assert contract.STAGE_IDS[:3] == (
        "attest_r9_terminal_imports",
        "fit_recovery_actor",
        "freeze_recovery_actor",
    )
    assert contract.STAGE_IDS[3:9] == tuple(
        f"development__{case_id}" for case_id in contract.DEVELOPMENT_CASE_IDS
    )
    assert contract.DEVELOPMENT_CASE_IDS == (
        "deterministic_offset_plus_0p20",
        "deterministic_offset_minus_0p20",
        "deterministic_offset_nominal",
        "stochastic_nominal_seed_126",
        "stochastic_nominal_seed_127",
        "stochastic_nominal_seed_128",
    )
    assert all(
        not stage.startswith(("collect", "label")) for stage in contract.STAGE_IDS
    )


def test_activity_contract_accepts_every_exact_stage_boundary() -> None:
    for prefix_length in range(len(contract.STAGE_IDS) + 1):
        completed = list(contract.STAGE_IDS[:prefix_length])
        attempted = (
            contract.STAGE_IDS[prefix_length]
            if prefix_length < len(contract.STAGE_IDS)
            else None
        )
        imported = int("attest_r9_terminal_imports" in completed)
        fit = int("fit_recovery_actor" in completed)
        developments = sum(stage.startswith("development__") for stage in completed)
        activity = runner._zero_activity()
        activity["r9_import_attestation_calls"] = imported
        activity["actor_fit_stage_calls"] = fit
        if fit:
            activity.update(
                {
                    "actor_updates": 1,
                    "uniform_adamw_epochs_completed": runner.fitter.UNIFORM_ADAMW_EPOCHS,
                    "uniform_lbfgs_closure_calls": runner.fitter.EXPECTED_UNIFORM_CLOSURES,
                    "gate_adamw_epochs_completed": runner.fitter.GATE_ADAMW_EPOCHS,
                    "gate_lbfgs_closure_calls": runner.fitter.EXPECTED_GATE_CLOSURES,
                }
            )
        activity["development_rollouts_completed"] = developments
        activity["environment_reset_calls"] = developments
        activity["development_environment_reset_calls"] = developments
        activity["environment_step_calls"] = developments * contract.EXPECTED_STEPS
        activity["development_environment_step_calls"] = (
            developments * contract.EXPECTED_STEPS
        )
        activity["raw_sensor_sample_count"] = (
            developments * contract.EXPECTED_STEPS * contract.RAW_SAMPLES_PER_STEP
        )
        ledger = {
            "activity_totals": activity,
            "collection_round_count": 0,
            "import_attestation_count": imported,
            "actor_fit_count": fit,
            "development_count": developments,
            **{
                name: activity[name]
                for name in (
                    "actor_updates",
                    "critic_updates",
                    "ppo_updates",
                    "environment_reset_calls",
                    "environment_step_calls",
                    "development_environment_reset_calls",
                    "development_environment_step_calls",
                    "raw_sensor_sample_count",
                    "teacher_query_count",
                    "served_action_teacher_dependency_count",
                    "mean_blend_count",
                    "safety_intervention_count",
                    "safety_latch_activation_count",
                    "safety_latch_release_count",
                )
            },
        }
        assert runner._prefix_activity_coherent(
            ledger,
            completed_ids=completed,
            attempted_stage=attempted,
        ), f"activity rejected at prefix {prefix_length}: {attempted}"


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


def test_development_runtime_evidence_recomputes_binary_pure_500_step_closure() -> None:
    case_id = contract.DEVELOPMENT_CASE_IDS[0]
    trace = _pure_development_trace(case_id)
    summary = {
        "steps": contract.EXPECTED_STEPS,
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "morphology_weight": 0.0,
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        **{name: 0 for name in runner.physical.PURE_POLICY_COUNTER_FIELDS},
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
    forged_summary = {**summary, "morphology_weight": 1.0, "actor_updates": 1}
    forged_audit, forged_checks = runner._recompute_development_runtime_evidence(
        summary=forged_summary,
        trace=forged_trace,
        case_id=case_id,
    )
    assert forged_audit["passed"] is False
    assert forged_checks["pure_trace"] is False
    assert forged_checks["morphology_zero"] is False
    assert forged_checks["zero_updates"] is False


def test_r9_attestation_source_contains_no_teacher_or_environment_runner() -> None:
    import inspect

    source = inspect.getsource(runner._verify_r9_inputs)
    assert "load_locked_r9_corpus" in source
    assert "r9_runner" not in source
    assert "verify_observer_label_closure" not in source


def test_write_surface_rejects_every_non_r10_namespace(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner.freezer, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(contract, "ROOT", PurePosixPath("validation/v12r10"))
    with pytest.raises(runner.V12R10ExecutionError, match="exclusive R10 namespace"):
        runner._write(
            PurePosixPath("validation/v12r9/forbidden.json"), {"forbidden": True}
        )
    assert not (tmp_path / "validation/v12r9/forbidden.json").exists()


def test_cli_returns_nonzero_for_verified_terminal_fail(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    fail = {"passed": False, "terminal": True, "next_stage": "STOP_TERMINAL_NO_RETRY"}
    monkeypatch.setattr(runner, "verify_terminal_ledger", lambda: fail)
    assert runner.main(["--verify"]) == 1
    assert json.loads(capsys.readouterr().out) == fail
