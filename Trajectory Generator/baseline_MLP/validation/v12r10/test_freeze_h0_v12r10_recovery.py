"""Focused source-only tests for the V12R10 protocol freezer."""

from __future__ import annotations

import ast
import copy
import json
import os
import sys
from pathlib import Path, PurePosixPath

import pytest


HERE = Path(__file__).resolve().parent
if os.fspath(HERE) not in sys.path:
    sys.path.insert(0, os.fspath(HERE))

import freeze_h0_v12r10_recovery as freezer  # noqa: E402
import h0_v12r10_recovery_contract as contract  # noqa: E402


def test_canonical_path_parser_rejects_escape_and_platform_aliases(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(freezer, "REPO_ROOT", tmp_path)
    assert freezer.resolve_relative("safe/path.json") == tmp_path / "safe/path.json"
    for unsafe in (
        "",
        "/absolute/path.json",
        "../escape.json",
        "safe/../alias.json",
        "safe//alias.json",
        "safe/./alias.json",
        r"safe\windows-alias.json",
        "C:/windows-drive-alias.json",
        "safe/alternate:data.json",
    ):
        with pytest.raises(freezer.V12R10FreezeError, match="non-canonical"):
            freezer.resolve_relative(unsafe)


def test_artifact_and_tree_records_reject_symlink_entries(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(freezer, "REPO_ROOT", tmp_path)
    safe = tmp_path / "safe"
    safe.mkdir()
    (safe / "payload.bin").write_bytes(b"locked")
    assert freezer.artifact_record("safe/payload.bin")["size_bytes"] == 6
    assert freezer.tree_record("safe")["file_count"] == 1

    outside = tmp_path / "outside.bin"
    outside.write_bytes(b"outside")
    linked = safe / "linked.bin"
    try:
        linked.symlink_to(outside)
    except (NotImplementedError, OSError):
        pytest.skip("symlink creation unavailable")
    with pytest.raises(freezer.V12R10FreezeError, match="symlink|unsafe"):
        freezer.artifact_record("safe/linked.bin")
    with pytest.raises(freezer.V12R10FreezeError, match="unsafe tree"):
        freezer.tree_record("safe")


def test_strict_json_rejects_duplicate_keys_and_nonfinite_values(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(freezer, "REPO_ROOT", tmp_path)
    duplicate = tmp_path / "duplicate.json"
    duplicate.write_text('{"x": 1, "x": 2}\n', encoding="utf-8")
    nonfinite = tmp_path / "nonfinite.json"
    nonfinite.write_text('{"x": NaN}\n', encoding="utf-8")
    for name in ("duplicate.json", "nonfinite.json"):
        with pytest.raises(freezer.V12R10FreezeError, match="strict JSON"):
            freezer._mapping(name)  # noqa: SLF001


def test_locked_r9_corpus_labels_trees_and_diagnostics_are_exact() -> None:
    attestation = freezer.attest_locked_inputs()
    assert attestation["passed"] is True
    assert attestation["r9_terminal"]["passed"] is True
    assert all(attestation["r9_terminal"]["checks"].values())
    assert attestation["r9_terminal"]["corpus"]["rows"] == 11_875
    assert attestation["r9_candidate_role"] == "INITIALIZATION_ONLY_NOT_PROMOTED"
    assert attestation["imported_labels"]["total_rows"] == 2_431
    assert set(attestation["imported_labels"]["records"]) == set(
        contract.COLLECTION_CASE_IDS
    )
    assert attestation["r6_candidate"] == contract.FULL_R6_CANDIDATE_TREE
    assert attestation["source_h0_teacher"] == contract.LOCKED_SOURCE_H0_TREE
    diagnostics = attestation["diagnostics"]
    assert diagnostics["passed"] is True
    assert all(diagnostics["uniform_checks"].values())
    assert all(diagnostics["gate_aligned_checks"].values())
    assert all(diagnostics["observability_checks"].values())
    assert diagnostics["attestation"] == contract.DIAGNOSTIC_ATTESTATION
    assert attestation["collection_round_count"] == 0
    assert attestation["label_generation_count"] == 0
    assert attestation["offline_h0_teacher_queries"] == 0


def test_production_sources_are_complete_and_do_not_import_diagnostics() -> None:
    paths = freezer.production_source_paths()
    closure = freezer.production_source_closure()
    assert len(paths) == len(set(paths)) == len(closure)
    assert set(paths) == set(closure)
    for required in freezer.LOCAL_PRODUCTION_SOURCES:
        assert required in closure

    for relative in freezer.LOCAL_PRODUCTION_SOURCES:
        if not relative.endswith(".py"):
            continue
        tree = ast.parse(freezer.resolve_relative(relative).read_text(encoding="utf-8"))
        imported: list[str] = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imported.append(node.module or "")
        assert all("diagnostic" not in name for name in imported)


def test_protocol_payload_is_execution_free_and_binds_all_inputs() -> None:
    payload = freezer.expected_protocol_payload()
    assert payload["passed"] is True
    assert payload["status"] == contract.PROTOCOL_FREEZE_PASS_STATUS
    assert payload["schema_version"] == 1300
    assert payload["stage_order"] == list(contract.STAGE_IDS)
    assert len(payload["stage_order"]) == 10
    assert payload["fit_contract"] == contract.FIT
    assert payload["offline_thresholds"] == contract.v12r9.OFFLINE_THRESHOLDS
    assert payload["candidate_module_path"] == contract.CANDIDATE_MODULE_PATH.as_posix()
    assert payload["single_actor_fit"] is True
    assert payload["collection_round_count"] == 0
    assert payload["label_generation_count"] == 0
    assert payload["environment_reset_calls"] == 0
    assert payload["offline_h0_teacher_queries"] == 0
    assert payload["actor_updates"] == 0
    assert payload["retry_authorized"] is False
    assert payload["production_source_count"] == len(
        payload["production_source_closure"]
    )
    assert payload["limitations"] == contract.LIMITATIONS


def test_runtime_boolean_contracts_are_type_strict() -> None:
    ready = {
        "inference_stack_ready": True,
        "platform_plugin_readiness": {"passed": True},
    }
    assert freezer._strict_runtime(lambda: ready) == ready  # noqa: SLF001
    for drifted in (
        {
            "inference_stack_ready": 1,
            "platform_plugin_readiness": {"passed": True},
        },
        {
            "inference_stack_ready": True,
            "platform_plugin_readiness": {"passed": 1},
        },
    ):
        with pytest.raises(freezer.V12R10FreezeError, match="readiness"):
            freezer._strict_runtime(lambda drifted=drifted: drifted)  # noqa: SLF001

    with pytest.raises(freezer.V12R10FreezeError, match="strict bool"):
        freezer.verify_execution_lock(require_pristine=1)  # type: ignore[arg-type]
    with pytest.raises(freezer.V12R10FreezeError, match="strict bool"):
        freezer.safe_repository_path("safe.json", include_leaf=1)  # type: ignore[arg-type]


def test_publication_refuses_nonpristine_run_root_without_writing(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(freezer, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(contract, "RUN_ROOT", PurePosixPath("run"))
    monkeypatch.setattr(contract, "PROTOCOL_FREEZE_PATH", PurePosixPath("freeze.json"))
    (tmp_path / "run").mkdir()
    with pytest.raises(freezer.V12R10FreezeError, match="run root"):
        freezer.publish_protocol_freeze()
    assert not (tmp_path / "freeze.json").exists()


def test_existing_protocol_is_verified_never_overwritten(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(freezer, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(contract, "PROTOCOL_FREEZE_PATH", PurePosixPath("freeze.json"))
    destination = tmp_path / "freeze.json"
    destination.write_bytes(b"immutable")
    sentinel = {"passed": True}
    monkeypatch.setattr(freezer, "verify_protocol_freeze", lambda: sentinel)
    assert freezer.publish_protocol_freeze() is sentinel
    assert destination.read_bytes() == b"immutable"


def _install_fake_execution_lock(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> tuple[dict[str, object], dict[str, object]]:
    monkeypatch.setattr(freezer, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(
        contract, "PROTOCOL_FREEZE_PATH", PurePosixPath("protocol_freeze.json")
    )
    monkeypatch.setattr(
        contract, "EXECUTION_LOCK_PATH", PurePosixPath("execution_lock.json")
    )
    monkeypatch.setattr(contract, "RUN_ROOT", PurePosixPath("run"))
    monkeypatch.setattr(contract, "CLAIM_PATH", PurePosixPath("run/claim.json"))
    monkeypatch.setattr(contract, "LEDGER_PATH", PurePosixPath("run/ledger.json"))
    monkeypatch.setattr(
        contract,
        "R9_IMPORT_ATTESTATION_PATH",
        PurePosixPath("run/r9_import.json"),
    )

    (tmp_path / "protocol_freeze.json").write_text("{}\n", encoding="utf-8")
    sources = {
        "source.py": {
            "path": "source.py",
            "sha256": "0" * 64,
            "size_bytes": 1,
        }
    }
    protocol = {
        "passed": True,
        "production_source_closure": copy.deepcopy(sources),
    }
    locked = {"passed": True, "evidence": "locked"}
    runtime: dict[str, object] = {
        "inference_stack_ready": True,
        "platform_plugin_readiness": {"passed": True},
        "runtime_id": "test-runtime",
    }
    monkeypatch.setattr(
        freezer, "verify_protocol_freeze", lambda: copy.deepcopy(protocol)
    )
    monkeypatch.setattr(
        freezer, "expected_protocol_payload", lambda: copy.deepcopy(protocol)
    )
    monkeypatch.setattr(
        freezer, "production_source_closure", lambda: copy.deepcopy(sources)
    )
    monkeypatch.setattr(freezer, "attest_locked_inputs", lambda: copy.deepcopy(locked))

    payload = freezer.expected_execution_lock_payload(
        runtime_attestor=lambda: copy.deepcopy(runtime)
    )
    assert payload["passed"] is True
    assert payload["positive_morphology_authorized"] is False
    (tmp_path / "execution_lock.json").write_text(
        json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8"
    )
    return payload, runtime


def _mutate_json_value(value: object) -> object:
    if type(value) is bool:
        return int(value)
    if type(value) is int:
        return str(value)
    if isinstance(value, str):
        return value + "__MUTATED"
    if isinstance(value, list):
        return [*value, "__MUTATED"]
    if isinstance(value, dict):
        return {**value, "__unexpected_nested_key__": True}
    raise AssertionError(f"unhandled JSON value type: {type(value)!r}")


def test_execution_lock_rejects_extra_top_level_key(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    payload, runtime = _install_fake_execution_lock(monkeypatch, tmp_path)
    mutated = copy.deepcopy(payload)
    mutated["unexpected_top_level_key"] = True
    (tmp_path / "execution_lock.json").write_text(
        json.dumps(mutated, sort_keys=True) + "\n", encoding="utf-8"
    )
    with pytest.raises(freezer.V12R10FreezeError, match="extra"):
        freezer.verify_execution_lock(
            require_pristine=False,
            runtime_attestor=lambda: copy.deepcopy(runtime),
        )


def test_execution_lock_rejects_mutation_of_every_expected_field(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    payload, runtime = _install_fake_execution_lock(monkeypatch, tmp_path)
    destination = tmp_path / "execution_lock.json"
    for field in payload:
        mutated = copy.deepcopy(payload)
        mutated[field] = _mutate_json_value(mutated[field])
        destination.write_text(
            json.dumps(mutated, sort_keys=True) + "\n", encoding="utf-8"
        )
        with pytest.raises(
            freezer.V12R10FreezeError,
            match=rf"fields=.*{field}",
        ):
            freezer.verify_execution_lock(
                require_pristine=False,
                runtime_attestor=lambda: copy.deepcopy(runtime),
            )


def test_execution_lock_preserves_historical_occupancy_after_run(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    payload, runtime = _install_fake_execution_lock(monkeypatch, tmp_path)
    (tmp_path / "run").mkdir()
    (tmp_path / "run" / "claim.json").write_text("{}\n", encoding="utf-8")

    observed = freezer.verify_execution_lock(
        require_pristine=False,
        runtime_attestor=lambda: copy.deepcopy(runtime),
    )
    assert observed == payload
    assert all(value is True for value in observed["occupancy"].values())

    with pytest.raises(freezer.V12R10FreezeError, match="no longer pristine"):
        freezer.verify_execution_lock(
            require_pristine=True,
            runtime_attestor=lambda: copy.deepcopy(runtime),
        )
