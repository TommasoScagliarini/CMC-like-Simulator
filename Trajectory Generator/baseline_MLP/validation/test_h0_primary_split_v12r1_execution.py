"""Execution-surface tests for the additive, one-shot V12R1 pipeline.

These tests exercise orchestration only.  They never load a policy checkpoint,
fit an actor, reset an environment, or create a canonical V12R1 artifact.
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
V12R1_ROOT = Path(__file__).resolve().parent
for _root in (REPO_ROOT, V12R1_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_primary_split_v12r1_autonomy_recovery_contract as contract
import run_h0_primary_split_v12r1_autonomy_recovery as runner
import run_h0_primary_split_v12r1_design_audit as audit_runner


def test_execution_sources_authority_and_stage_topology_are_exact() -> None:
    assert runner.EXECUTION_SOURCE_RELATIVE_PATHS == dict(
        contract.FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS
    )
    assert set(runner._execution_source_records()) == set(
        contract.FUTURE_EXECUTION_SOURCES_REQUIRED
    )
    assert runner.EXECUTION_AUTHORITY == {
        "authority_date": contract.REVISION,
        "authority_text": "esegui",
        "authority_scope": "V12R1_ONE_SHOT_EXECUTION",
        "one_shot": True,
    }
    assert len(contract.STAGE_IDS) == 26
    assert contract.STAGE_IDS[0] == "fit_p0"
    assert contract.STAGE_IDS[-1] == "finalize_development"
    assert sum(
        contract.stage_descriptor(stage)["kind"] == "fit"
        for stage in contract.STAGE_IDS
    ) == 4
    assert sum(
        contract.stage_descriptor(stage)["kind"] == "probe"
        for stage in contract.STAGE_IDS
    ) == 4
    assert sum(
        contract.stage_descriptor(stage)["kind"] == "label"
        for stage in contract.STAGE_IDS
    ) == 4
    manifest = contract.declared_mutation_paths()
    assert manifest["candidate_freeze_summary"] == (
        contract.FIT_ROOT / "candidate_freeze_summary.json"
    )
    assert manifest["candidate_freeze_gate"] == (
        contract.FIT_ROOT / "candidate_freeze_gate.json"
    )
    assert sum(name.startswith("probe_failure_") for name in manifest) == 4
    assert sum(name.endswith("_failure") and name.startswith("collect_") for name in manifest) == 6
    assert sum(name.endswith("_failure") and name.startswith("final_") for name in manifest) == 6


def test_stage_dispatch_is_exact_and_side_effect_free_when_stubbed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[object, ...]] = []
    monkeypatch.setattr(
        runner,
        "_run_fit",
        lambda stage: calls.append(("fit", stage)) or {"passed": True},
    )
    monkeypatch.setattr(
        runner,
        "_run_probe",
        lambda stage: calls.append(("probe", stage)) or {"passed": True},
    )
    monkeypatch.setattr(
        runner,
        "_run_label",
        lambda stage: calls.append(("label", stage)) or {"passed": True},
    )
    monkeypatch.setattr(
        runner,
        "_run_collection",
        lambda round_index, case_id: calls.append(
            ("collection", round_index, case_id)
        )
        or {"passed": True},
    )
    monkeypatch.setattr(
        runner,
        "_run_candidate_freeze",
        lambda: calls.append(("freeze",)) or {"passed": True},
    )
    monkeypatch.setattr(
        runner,
        "_run_final",
        lambda case_id: calls.append(("final", case_id)) or {"passed": True},
    )
    monkeypatch.setattr(
        runner,
        "_run_finalize_development",
        lambda: calls.append(("finalize",)) or {"passed": True},
    )

    for stage_id in contract.STAGE_IDS:
        assert runner._run_stage(stage_id)["passed"] is True

    assert len(calls) == len(contract.STAGE_IDS)
    assert calls[:3] == [("fit", "p0"), ("probe", "p0"), ("label", "p0")]
    assert calls[-2:] == [
        ("final", contract.FINAL_CASE_IDS[-1]),
        ("finalize",),
    ]


def _stub_pipeline_io(
    monkeypatch: pytest.MonkeyPatch,
) -> tuple[list[str], list[tuple[Path, dict[str, object]]]]:
    stage_calls: list[str] = []
    writes: list[tuple[Path, dict[str, object]]] = []
    stored: dict[Path, dict[str, object]] = {}

    monkeypatch.setattr(
        runner, "verify_execution_lock", lambda **_kwargs: {"passed": True}
    )
    monkeypatch.setattr(runner.os.path, "lexists", lambda _path: False)
    monkeypatch.setattr(runner, "_claim_run_root", lambda: None)
    monkeypatch.setattr(
        runner,
        "_record",
        lambda path: {
            "path": Path(path).name,
            "sha256": "b" * 64,
            "size_bytes": 1,
        },
    )
    monkeypatch.setattr(runner, "_write_worker_claim", lambda *_args: None)
    monkeypatch.setattr(
        runner, "_verify_worker_claim", lambda stage: {"stage_id": stage}
    )
    monkeypatch.setattr(
        runner, "_verify_stage_receipt", lambda stage: {"stage_id": stage}
    )
    monkeypatch.setattr(
        runner,
        "_run_stage",
        lambda stage: stage_calls.append(stage) or {"passed": True},
    )
    monkeypatch.setattr(
        runner,
        "_ledger_payload",
        lambda **kwargs: {
            "status": kwargs["status"],
            "passed": kwargs["passed"],
            "completed_stages": list(kwargs["completed_stages"]),
            "attempted_stage": kwargs["attempted_stage"],
        },
    )

    def write(path: Path, payload: dict[str, object]) -> Path:
        snapshot = copy.deepcopy(payload)
        writes.append((Path(path), snapshot))
        stored[Path(path)] = snapshot
        return Path(path)

    monkeypatch.setattr(runner.forensic, "write_json_exclusive", write)
    monkeypatch.setattr(
        runner,
        "_mapping",
        lambda path: copy.deepcopy(stored[Path(path)]),
    )
    return stage_calls, writes


def test_execute_pipeline_once_visits_each_stage_once_in_frozen_order(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stage_calls, writes = _stub_pipeline_io(monkeypatch)
    ledger = runner.execute_pipeline_once()

    assert tuple(stage_calls) == tuple(contract.STAGE_IDS)
    assert ledger["passed"] is True
    assert ledger["completed_stages"] == list(contract.STAGE_IDS)
    assert [path for path, _payload in writes] == [
        runner.PIPELINE_CLAIM_PATH,
        runner.PIPELINE_LEDGER_PATH,
    ]
    claim = writes[0][1]
    assert claim["authority"] == runner.EXECUTION_AUTHORITY
    assert claim["retry_authorized"] is False
    assert claim["sweep_authorized"] is False
    assert claim["rescue_authorized"] is False


def test_execute_pipeline_once_publishes_one_terminal_failure_and_stops(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stage_calls, writes = _stub_pipeline_io(monkeypatch)
    failed_stage = contract.STAGE_IDS[2]

    def fail_on_third(stage: str) -> dict[str, bool]:
        stage_calls.append(stage)
        if stage == failed_stage:
            raise runner.V12R1ExecutionError("synthetic terminal failure")
        return {"passed": True}

    monkeypatch.setattr(runner, "_run_stage", fail_on_third)
    with pytest.raises(runner.V12R1ExecutionError, match="synthetic terminal"):
        runner.execute_pipeline_once()

    assert stage_calls == list(contract.STAGE_IDS[:3])
    assert [path for path, _payload in writes] == [
        runner.PIPELINE_CLAIM_PATH,
        runner.PIPELINE_LEDGER_PATH,
    ]
    failure = writes[-1][1]
    assert failure["passed"] is False
    assert failure["attempted_stage"] == failed_stage
    assert failure["completed_stages"] == list(contract.STAGE_IDS[:2])


def test_pipeline_claim_publication_failure_writes_zero_activity_ledger(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    writes: list[tuple[Path, dict[str, object]]] = []
    monkeypatch.setattr(
        runner, "verify_execution_lock", lambda **_kwargs: {"passed": True}
    )
    monkeypatch.setattr(runner.os.path, "lexists", lambda _path: False)
    monkeypatch.setattr(runner, "_claim_run_root", lambda: None)
    monkeypatch.setattr(
        runner,
        "_record",
        lambda path: {
            "path": Path(path).name,
            "sha256": "d" * 64,
            "size_bytes": 1,
        },
    )

    def write(path: Path, payload: dict[str, object]) -> Path:
        if Path(path) == runner.PIPELINE_CLAIM_PATH:
            raise OSError("synthetic claim publication failure")
        writes.append((Path(path), copy.deepcopy(payload)))
        return Path(path)

    monkeypatch.setattr(runner.forensic, "write_json_exclusive", write)
    with pytest.raises(OSError, match="synthetic claim"):
        runner.execute_pipeline_once()

    assert [path for path, _payload in writes] == [runner.PIPELINE_LEDGER_PATH]
    ledger = writes[0][1]
    assert ledger["status"] == (
        "FAIL_H0_PRIMARY_SPLIT_V12R1_PIPELINE_CLAIM_TERMINAL"
    )
    assert ledger["completed_stages"] == []
    assert ledger["actor_fit_stage_calls_attempted"] == 0
    assert ledger["environment_reset_calls"] == 0
    assert ledger["environment_step_calls"] == 0


def test_execution_lock_gate_rejects_retry_even_if_runner_payload_is_pass_like() -> None:
    record = lambda path: {  # noqa: E731 - compact fixture factory
        "path": path.as_posix(),
        "sha256": "a" * 64,
        "size_bytes": 1,
    }
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.EXECUTION_LOCK_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "protocol_freeze": record(contract.PROTOCOL_FREEZE_PATH),
        "v12_protocol_freeze": {
            "path": contract.V12_PROTOCOL_FREEZE_PATH.as_posix(),
            "sha256": contract.V12_PROTOCOL_FREEZE_SHA256,
            "size_bytes": contract.V12_PROTOCOL_FREEZE_SIZE_BYTES,
        },
        "design_audit": record(contract.DESIGN_AUDIT_RECEIPT_PATH),
        "design_audit_gate_passed": True,
        "execution_sources": {
            name: record(Path(path))
            for name, path in contract.FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS.items()
        },
        "inherited_runtime_evidence": {
            "passed": True,
            "v11_execution_lock": copy.deepcopy(
                contract.V11_EXECUTION_LOCK_ARTIFACT
            ),
            "source_count": 66,
            "input_count": 54,
            "sources_exact": True,
            "inputs_exact": True,
        },
        "execution_authority": copy.deepcopy(runner.EXECUTION_AUTHORITY),
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "post_hoc_retuning_authorized": False,
        "declared_mutation_paths": {
            name: path.as_posix()
            for name, path in contract.declared_mutation_paths().items()
        },
        "pipeline_claim_preexisting": False,
        "actor_fit_executions": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "offline_teacher_label_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "p0_reproduction_tolerance": copy.deepcopy(
            contract.P0_REPRODUCTION_TOLERANCE
        ),
    }
    assert contract.execution_lock_gate(payload)["passed"] is True
    payload["retry_authorized"] = True
    assert contract.execution_lock_gate(payload)["passed"] is False


def test_run_root_claim_is_atomic_and_no_clobber(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    run_root = tmp_path / "claimed_run"
    monkeypatch.setattr(runner, "RUN_ROOT", run_root)
    original_resolve = runner.resolve_relative

    def resolve(value: object) -> Path:
        if value == contract.RUN_ROOT.parent:
            return run_root.parent
        return original_resolve(value)  # pragma: no cover - defensive fallback

    monkeypatch.setattr(runner, "resolve_relative", resolve)
    runner._claim_run_root()
    assert run_root.is_dir()
    with pytest.raises(runner.V12R1ExecutionError, match="already claimed"):
        runner._claim_run_root()


def test_design_audit_build_calls_the_in_memory_fitter_exactly_once(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    record = lambda path: {  # noqa: E731 - compact fixture factory
        "path": path.as_posix(),
        "sha256": "c" * 64,
        "size_bytes": 1,
    }
    protocol_record = record(contract.PROTOCOL_FREEZE_PATH)
    v12_record = {
        "path": contract.V12_PROTOCOL_FREEZE_PATH.as_posix(),
        "sha256": contract.V12_PROTOCOL_FREEZE_SHA256,
        "size_bytes": contract.V12_PROTOCOL_FREEZE_SIZE_BYTES,
    }
    source_records = {
        name: record(Path(path))
        for name, path in contract.FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS.items()
    }
    runtime = {"system": "test", "machine": "test", "python": "test"}
    calls = 0

    def fit_once() -> dict[str, object]:
        nonlocal calls
        calls += 1
        metrics = {
            "rmse": 0.001,
            "max_abs_error": 0.01,
            "reset_max_abs_error": 0.001,
        }
        return {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.DESIGN_AUDIT_PASS_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "contract_id": contract.FIT_CONTRACT_ID,
            "design_audit_id": contract.DESIGN_AUDIT_ID,
            "fit_design": copy.deepcopy(contract.FIT),
            "observed_metrics": metrics,
            "p0_reproduction_reference_metrics": copy.deepcopy(metrics),
            "p0_reproduction_tolerance": copy.deepcopy(
                contract.P0_REPRODUCTION_TOLERANCE
            ),
            "dry_run": True,
            "no_candidate_checkpoint": True,
            "actor_fit_executions": 1,
            "actor_updates": 1,
            "candidate_checkpoints_persisted": 0,
            "candidate_checkpoint_paths": [],
            "environment_reset_calls": 0,
            "environment_step_calls": 0,
            "offline_teacher_label_calls": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "retry_authorized": False,
            "sweep_authorized": False,
            "rescue_authorized": False,
        }

    monkeypatch.setattr(
        audit_runner,
        "_assert_prepare_occupancy",
        lambda: {
            "design_audit_receipt_absent": True,
            "execution_lock_absent": True,
            "run_root_absent": True,
            "pipeline_claim_absent": True,
            "pipeline_ledger_absent": True,
        },
    )
    monkeypatch.setattr(
        audit_runner,
        "_forbidden_execution_occupancy",
        lambda: {
            "execution_lock_absent": True,
            "run_root_absent": True,
            "pipeline_claim_absent": True,
            "pipeline_ledger_absent": True,
        },
    )
    monkeypatch.setattr(audit_runner, "_verify_protocol_freeze", lambda: {})
    monkeypatch.setattr(
        audit_runner,
        "_record",
        lambda path: (
            v12_record
            if path == contract.V12_PROTOCOL_FREEZE_PATH
            else protocol_record
        ),
    )
    monkeypatch.setattr(
        audit_runner, "_execution_source_records", lambda: source_records
    )
    monkeypatch.setattr(audit_runner, "_runtime_record", lambda: runtime)
    monkeypatch.setattr(audit_runner.os.path, "lexists", lambda _path: False)
    monkeypatch.setattr(audit_runner.fitter, "run_design_audit_in_memory", fit_once)

    payload = audit_runner.build_design_audit()
    assert calls == 1
    assert payload["contract_gate"]["passed"] is True
    assert payload["execution_sources"] == source_records
    assert payload["artifacts_written"] == [
        contract.DESIGN_AUDIT_RECEIPT_PATH.as_posix()
    ]
