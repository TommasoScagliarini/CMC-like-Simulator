from __future__ import annotations

import ast
import copy
import os
from pathlib import Path

import pytest

import freeze_h0_v12r5_case_balanced as freezer
import h0_v12r5_case_balanced_contract as contract


def _redirect_publication_outputs(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> tuple[Path, Path, Path]:
    freeze_path = tmp_path / "protocol_freeze.json"
    audit_path = tmp_path / "design_audit.json"
    failure_path = tmp_path / "protocol_publication_failure.json"
    monkeypatch.setattr(freezer, "PROTOCOL_FREEZE_FILE", freeze_path)
    monkeypatch.setattr(freezer, "DESIGN_AUDIT_FILE", audit_path)
    monkeypatch.setattr(freezer, "PROTOCOL_PUBLICATION_FAILURE_FILE", failure_path)
    return freeze_path, audit_path, failure_path


def test_resolver_rejects_noncanonical_paths() -> None:
    for value in ("../escape", "/absolute", "a/../b", ""):
        with pytest.raises(freezer.V12R5ProtocolFreezeError):
            freezer.resolve_relative(value)


def test_all_locked_records_and_semantics_are_current() -> None:
    assert all(freezer._locked_record_checks().values())
    assert all(freezer._locked_semantics().values())
    assert all(freezer._q2_design_semantics().values())
    assert all(freezer._q3_design_semantics().values())
    assert freezer.artifact_record(contract.Q3_DESIGN_FREEZE_PATH) == (
        contract.Q3_DESIGN_FREEZE_ARTIFACT
    )
    assert (
        freezer.artifact_record(contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT["path"])
        == contract.SAFE_V8R1P1_EXECUTION_LOCK_ARTIFACT
    )
    assert len(freezer._safe_environment_source_closure()) == 29
    assert len(contract.FROZEN_EXTERNAL_RUNTIME_SOURCES) == 64
    assert all(
        path == record["path"]
        for path, record in contract.FROZEN_EXTERNAL_RUNTIME_SOURCES.items()
    )
    assert not any(
        Path(path).name.startswith("test_")
        for path in contract.FROZEN_EXTERNAL_RUNTIME_SOURCES
    )
    assert {
        "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py",
        "Trajectory Generator/baseline_MLP/target_domain_imitation.py",
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "run_h0_primary_split_v12r3_autonomy_recovery.py",
        "Trajectory Generator/osim_trj_cmc_like.py",
        "simulation_runner.py",
    }.issubset(contract.FROZEN_EXTERNAL_RUNTIME_SOURCES)


def test_q2_q3_designs_are_allowed_but_execution_outputs_are_absent() -> None:
    absence = freezer._output_absence()
    assert "q2_design_freeze" not in absence
    assert "q3_design_freeze" not in absence
    assert set(
        name.removeprefix("q2_") for name in absence if name.startswith("q2_")
    ) == set(contract.Q2_UNOPENED_PATHS)
    assert set(
        name.removeprefix("q3_") for name in absence if name.startswith("q3_")
    ) == set(contract.Q3_UNOPENED_PATHS)
    assert absence["protocol_publication_failure"] is True
    assert all(absence.values())


def test_source_gate_and_in_memory_protocol_payload_pass_without_writes() -> None:
    freeze_path = freezer.resolve_relative(contract.PROTOCOL_FREEZE_PATH)
    audit_path = freezer.resolve_relative(contract.DESIGN_AUDIT_PATH)
    failure_path = freezer.resolve_relative(contract.PROTOCOL_PUBLICATION_FAILURE_PATH)
    assert not os.path.lexists(freeze_path)
    assert not os.path.lexists(audit_path)
    assert not os.path.lexists(failure_path)
    gate = freezer.source_precondition_gate()
    assert gate["passed"] is True
    assert gate["q2_paths_opened"] == []
    assert gate["q3_paths_opened"] == []
    payload = freezer.build_protocol_freeze_payload()
    assert payload["passed"] is True
    assert payload["new_collection_stages"] == []
    assert payload["new_collection_count"] == 0
    assert payload["corpus_counts"]["sample_count"] == 9232
    assert payload["q3_design_freeze"] == contract.Q3_DESIGN_FREEZE_ARTIFACT
    assert len(payload["locked_inputs"]["safe_environment_source_closure"]) == 29
    assert payload["locked_inputs"]["external_runtime_sources"] == (
        contract.FROZEN_EXTERNAL_RUNTIME_SOURCES
    )
    assert payload["locked_inputs"]["production_source_closure"] == (
        freezer._production_source_closure()
    )
    assert payload["candidate_selection"]["module_path"] == (
        contract.CANDIDATE_MODULE_PATH.as_posix()
    )
    freezer._validate_protocol_freeze_payload(payload)
    expected_freeze = freezer._expected_artifact_record(
        contract.PROTOCOL_FREEZE_PATH, payload
    )
    audit = freezer.build_design_audit_payload(payload, expected_freeze)
    freezer._validate_design_audit_payload(
        audit,
        protocol_payload=payload,
        expected_protocol_record=expected_freeze,
    )
    expected_audit = freezer._expected_artifact_record(
        contract.DESIGN_AUDIT_PATH, audit
    )
    assert expected_freeze["path"] == contract.PROTOCOL_FREEZE_PATH.as_posix()
    assert expected_audit["path"] == contract.DESIGN_AUDIT_PATH.as_posix()
    assert audit["protocol_freeze"] == expected_freeze
    assert not os.path.lexists(freeze_path)
    assert not os.path.lexists(audit_path)
    assert not os.path.lexists(failure_path)


def test_protocol_transaction_publishes_audit_as_commit_marker_in_tmp(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    freeze_path, audit_path, failure_path = _redirect_publication_outputs(
        tmp_path, monkeypatch
    )

    assert freezer.publish_protocol_freeze() == freeze_path
    payload = freezer.verify_protocol_freeze()
    expected_freeze = freezer._expected_artifact_record(
        contract.PROTOCOL_FREEZE_PATH, payload
    )
    assert (
        freezer._publication_observation(freeze_path, expected_record=expected_freeze)[
            "state"
        ]
        == "VALID_REGULAR"
    )
    audit = freezer._load_mapping_file(audit_path)
    expected_audit = freezer._expected_artifact_record(
        contract.DESIGN_AUDIT_PATH, audit
    )
    assert (
        freezer._publication_observation(audit_path, expected_record=expected_audit)[
            "state"
        ]
        == "VALID_REGULAR"
    )
    assert audit["protocol_freeze"] == expected_freeze
    assert not os.path.lexists(failure_path)


def test_protocol_freeze_without_design_commit_marker_is_partial_and_blocked(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    freeze_path, audit_path, failure_path = _redirect_publication_outputs(
        tmp_path, monkeypatch
    )
    payload = freezer.build_protocol_freeze_payload()
    freezer._validate_protocol_freeze_payload(payload)
    freezer._write_json_exclusive(freeze_path, payload)
    assert freeze_path.is_file()
    assert not os.path.lexists(audit_path)
    assert not os.path.lexists(failure_path)
    with pytest.raises(
        freezer.V12R5ProtocolFreezeError,
        match="unsafe/nonregular JSON",
    ):
        freezer.verify_protocol_freeze()
    assert freezer.source_precondition_gate()["passed"] is False


@pytest.mark.parametrize("leave_partial_audit", [False, True])
def test_audit_write_failure_publishes_terminal_bound_receipt(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    leave_partial_audit: bool,
) -> None:
    freeze_path, audit_path, failure_path = _redirect_publication_outputs(
        tmp_path, monkeypatch
    )
    original_write = freezer._write_json_exclusive

    def injected_write(path: Path, payload: object) -> None:
        if path == audit_path:
            if leave_partial_audit:
                path.write_bytes(b"")
            raise OSError("injected audit publication failure")
        original_write(path, payload)

    monkeypatch.setattr(freezer, "_write_json_exclusive", injected_write)
    with pytest.raises(OSError, match="injected audit publication failure"):
        freezer.publish_protocol_freeze()

    failure = freezer.verify_protocol_publication_failure()
    failure_bytes = failure_path.read_bytes()
    with pytest.raises(freezer.V12R5ProtocolFreezeError, match="refusing to clobber"):
        original_write(failure_path, {"tampered": True})
    assert failure_path.read_bytes() == failure_bytes
    assert failure["protocol_freeze_observation"]["state"] == "VALID_REGULAR"
    assert failure["design_audit_observation"]["state"] == (
        "PARTIAL_OR_INVALID_REGULAR" if leave_partial_audit else "ABSENT"
    )
    if leave_partial_audit:
        assert failure["design_audit_observation"]["artifact"]["size_bytes"] == 0
    else:
        assert failure["design_audit_observation"]["artifact"] is None
    assert failure["error"] == {
        "type": "OSError",
        "message": "injected audit publication failure",
    }
    assert failure["next_stage"] == "STOP_TERMINAL"
    assert os.path.lexists(freeze_path)
    assert os.path.lexists(failure_path)
    with pytest.raises(freezer.V12R5ProtocolFreezeError, match="terminal and dominant"):
        freezer.verify_protocol_freeze()
    source_gate = freezer.source_precondition_gate()
    assert source_gate["passed"] is False
    assert source_gate["absence_checks"]["protocol_publication_failure"] is False


@pytest.mark.parametrize(
    "mutation",
    [
        "schema_version_float",
        "new_collection_count_bool",
        "environment_reset_calls_bool",
        "actor_updates_bool",
        "qualification_bool_drift",
        "top_level_extra",
        "top_level_missing",
        "error_extra",
        "expected_record_size_float",
        "observation_record_size_float",
    ],
)
def test_protocol_publication_failure_receipt_is_deep_type_strict(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
) -> None:
    _freeze_path, audit_path, failure_path = _redirect_publication_outputs(
        tmp_path, monkeypatch
    )
    original_write = freezer._write_json_exclusive

    def injected_write(path: Path, payload: object) -> None:
        if path == audit_path:
            raise OSError("injected audit publication failure")
        original_write(path, payload)

    monkeypatch.setattr(freezer, "_write_json_exclusive", injected_write)
    with pytest.raises(OSError, match="injected audit publication failure"):
        freezer.publish_protocol_freeze()
    valid = freezer.verify_protocol_publication_failure()
    tampered = copy.deepcopy(valid)
    if mutation == "schema_version_float":
        tampered["schema_version"] = float(tampered["schema_version"])
    elif mutation == "new_collection_count_bool":
        tampered["new_collection_count"] = False
    elif mutation == "environment_reset_calls_bool":
        tampered["environment_reset_calls"] = False
    elif mutation == "actor_updates_bool":
        tampered["actor_updates"] = False
    elif mutation == "qualification_bool_drift":
        tampered["qualification_violation_detected"] = True
    elif mutation == "top_level_extra":
        tampered["unregistered_field"] = None
    elif mutation == "top_level_missing":
        tampered.pop("phase")
    elif mutation == "error_extra":
        tampered["error"]["unregistered_field"] = None
    elif mutation == "expected_record_size_float":
        record = tampered["expected_protocol_freeze"]
        record["size_bytes"] = float(record["size_bytes"])
    else:
        record = tampered["protocol_freeze_observation"]["artifact"]
        record["size_bytes"] = float(record["size_bytes"])
    failure_path.write_bytes(freezer._canonical_json_bytes(tampered))
    with pytest.raises(
        freezer.V12R5ProtocolFreezeError,
        match="protocol publication failure receipt drifted",
    ):
        freezer.verify_protocol_publication_failure()


def test_protocol_failure_qualification_snapshot_is_historical(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _freeze_path, audit_path, _failure_path = _redirect_publication_outputs(
        tmp_path, monkeypatch
    )
    original_write = freezer._write_json_exclusive
    q3_name = next(iter(contract.Q3_UNOPENED_PATHS))

    def injected_write(path: Path, payload: object) -> None:
        if path == audit_path:
            raise OSError("injected audit publication failure")
        original_write(path, payload)

    monkeypatch.setattr(freezer, "_write_json_exclusive", injected_write)
    monkeypatch.setattr(
        freezer,
        "_opened_qualification_paths",
        lambda: {"q2": [], "q3": [q3_name]},
    )
    with pytest.raises(OSError, match="injected audit publication failure"):
        freezer.publish_protocol_freeze()
    monkeypatch.setattr(
        freezer,
        "_opened_qualification_paths",
        lambda: {"q2": [], "q3": []},
    )
    failure = freezer.verify_protocol_publication_failure()
    assert failure["q3_paths_opened"] == [q3_name]
    assert failure["qualification_violation_detected"] is True


def test_unreadable_regular_protocol_prefix_gets_terminal_failure_receipt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    freeze_path, audit_path, failure_path = _redirect_publication_outputs(
        tmp_path, monkeypatch
    )
    original_record = freezer._raw_artifact_record

    def unreadable_protocol(path: Path, *, logical_path: str) -> dict[str, object]:
        if path == freeze_path:
            raise PermissionError("injected unreadable regular protocol")
        return original_record(path, logical_path=logical_path)

    monkeypatch.setattr(freezer, "_raw_artifact_record", unreadable_protocol)
    with pytest.raises(
        freezer.V12R5ProtocolFreezeError,
        match="protocol freeze post-publication record mismatch",
    ):
        freezer.publish_protocol_freeze()

    assert freeze_path.is_file()
    assert not audit_path.exists()
    assert failure_path.is_file()
    failure = freezer.verify_protocol_publication_failure()
    assert failure["protocol_freeze_observation"] == {
        "state": "UNSAFE_UNREADABLE_REGULAR",
        "artifact": None,
    }
    assert failure["design_audit_observation"] == {
        "state": "ABSENT",
        "artifact": None,
    }


def test_post_verify_failure_receipt_dominates_two_valid_members(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    freeze_path, audit_path, failure_path = _redirect_publication_outputs(
        tmp_path, monkeypatch
    )
    original_verify = freezer.verify_protocol_freeze

    def fail_post_verify() -> dict[str, object]:
        raise freezer.V12R5ProtocolFreezeError("injected post-verify failure")

    monkeypatch.setattr(freezer, "verify_protocol_freeze", fail_post_verify)
    with pytest.raises(
        freezer.V12R5ProtocolFreezeError, match="injected post-verify failure"
    ):
        freezer.publish_protocol_freeze()

    failure = freezer.verify_protocol_publication_failure()
    assert failure["protocol_freeze_observation"]["state"] == "VALID_REGULAR"
    assert failure["design_audit_observation"]["state"] == "VALID_REGULAR"
    assert all(
        os.path.lexists(path) for path in (freeze_path, audit_path, failure_path)
    )
    monkeypatch.setattr(freezer, "verify_protocol_freeze", original_verify)
    with pytest.raises(freezer.V12R5ProtocolFreezeError, match="terminal and dominant"):
        freezer.verify_protocol_freeze()


def test_canonical_link_publication_never_leaves_zero_byte_final(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    destination = tmp_path / "canonical.json"

    def fail_link(_source: object, _destination: object) -> None:
        raise OSError("injected link failure")

    monkeypatch.setattr(freezer.os, "link", fail_link)
    with pytest.raises(OSError, match="injected link failure"):
        freezer._write_json_exclusive(destination, {"value": 1})
    assert not os.path.lexists(destination)
    assert list(tmp_path.glob(".canonical.json.*.tmp")) == []


def test_production_source_closure_is_exact_and_tamper_closed() -> None:
    closure = freezer._production_source_closure()
    assert len(closure) == contract.EXPECTED_PRODUCTION_SOURCE_COUNT == 68
    assert freezer._production_source_closure_exact(closure) is True
    assert not any(Path(path).name.startswith("test_") for path in closure)
    assert {
        (contract.VALIDATION_ROOT / name).as_posix()
        for name in contract.LOCAL_R5_PRODUCTION_SOURCE_NAMES
    }.issubset(closure)

    missing = copy.deepcopy(closure)
    missing.pop(next(iter(missing)))
    assert freezer._production_source_closure_exact(missing) is False

    extra = copy.deepcopy(closure)
    extra["validation/test_surrogate.py"] = {
        "path": "validation/test_surrogate.py",
        "sha256": "0" * 64,
        "size_bytes": 0,
    }
    assert freezer._production_source_closure_exact(extra) is False

    tampered = copy.deepcopy(closure)
    first = next(iter(tampered))
    tampered[first]["sha256"] = "0" * 64
    assert freezer._production_source_closure_exact(tampered) is False


def test_freezer_source_has_no_rllib_opensim_or_execution_surface() -> None:
    source = Path(freezer.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    assert not any(name.startswith("ray") for name in imported)
    assert not any(name.startswith("opensim") for name in imported)
    assert "make_cmc_env" not in source
    assert ".reset(" not in source
    assert ".step(" not in source
    assert "run_fit_stage" not in source


def test_source_closure_includes_cross_platform_attributes() -> None:
    records = freezer._source_records()
    assert set(records) == {
        ".gitattributes",
        "__init__.py",
        "conftest.py",
        "h0_v12r5_case_balanced_contract.py",
        "freeze_h0_v12r5_case_balanced.py",
        "h0_v12r5_case_balanced_fitter.py",
        "run_h0_v12r5_case_balanced.py",
        "test_h0_v12r5_case_balanced_contract.py",
        "test_freeze_h0_v12r5_case_balanced.py",
        "test_h0_v12r5_case_balanced_fitter.py",
        "test_run_h0_v12r5_case_balanced.py",
    }
    attributes = (freezer.REVISION_ROOT / ".gitattributes").read_text(encoding="utf-8")
    assert "*.py text eol=lf" in attributes
    assert "*.json text eol=lf" in attributes
    assert "*.npz binary" in attributes
    assert "*.pkl binary" in attributes
