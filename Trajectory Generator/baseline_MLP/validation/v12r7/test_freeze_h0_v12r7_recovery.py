"""Focused source-only tests for the V12R7 protocol freezer."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest


HERE = Path(__file__).resolve().parent
if os.fspath(HERE) not in sys.path:
    sys.path.insert(0, os.fspath(HERE))

import freeze_h0_v12r7_recovery as freezer  # noqa: E402
import h0_v12r7_recovery_contract as contract  # noqa: E402


def test_canonical_path_parser_rejects_escape_and_cross_platform_aliases(
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
        with pytest.raises(freezer.V12R7FreezeError, match="non-canonical"):
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
        pytest.skip("symlink creation is unavailable on this platform")
    with pytest.raises(freezer.V12R7FreezeError, match="symlink|unsafe"):
        freezer.artifact_record("safe/linked.bin")
    with pytest.raises(freezer.V12R7FreezeError, match="unsafe tree"):
        freezer.tree_record("safe")


def test_locked_r6_inputs_and_complete_source_closure_are_exact() -> None:
    attestation = freezer.attest_locked_inputs()
    assert attestation["passed"] is True
    assert all(attestation["semantic_checks"].values())
    assert (
        attestation["r6_candidate"]["tree_sha256"]
        == contract.LOCKED_INPUTS["r6_candidate"]["tree_sha256"]
    )

    paths = freezer.production_source_paths()
    closure = freezer.production_source_closure()
    assert len(paths) == len(set(paths)) == len(closure)
    assert set(paths) == set(closure)
    assert all(record["path"] == path for path, record in closure.items())
    for required in freezer.LOCAL_PRODUCTION_SOURCES:
        assert required in closure
    for required in freezer.ADDITIONAL_TRANSITIVE_SOURCES:
        assert required in closure


def test_protocol_payload_is_execution_free_and_binds_one_shot_contract() -> None:
    payload = freezer.expected_protocol_payload()
    assert payload["passed"] is True
    assert payload["status"] == contract.PROTOCOL_FREEZE_PASS_STATUS
    assert payload["stage_order"] == list(contract.STAGE_IDS)
    assert payload["candidate_selection_rule"] == contract.CANDIDATE_SELECTION_RULE
    assert payload["candidate_module_path"] == contract.CANDIDATE_MODULE_PATH.as_posix()
    assert payload["single_actor_fit"] is True
    assert payload["retry_authorized"] is False
    assert payload["qualification_execution_authorized"] is False
    assert payload["environment_reset_calls"] == 0
    assert type(payload["environment_reset_calls"]) is int
    assert payload["actor_updates"] == 0
    assert payload["production_source_count"] == len(
        payload["production_source_closure"]
    )


def test_runtime_and_verifier_boolean_contracts_are_type_strict() -> None:
    ready = {
        "inference_stack_ready": True,
        "platform_plugin_readiness": {"passed": True},
    }
    assert freezer._strict_runtime(lambda: ready) == ready

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
        with pytest.raises(freezer.V12R7FreezeError, match="readiness"):
            freezer._strict_runtime(lambda drifted=drifted: drifted)

    with pytest.raises(freezer.V12R7FreezeError, match="strict bool"):
        freezer.verify_execution_lock(require_pristine=1)  # type: ignore[arg-type]
    with pytest.raises(freezer.V12R7FreezeError, match="strict bool"):
        freezer.safe_repository_path("safe.json", include_leaf=1)  # type: ignore[arg-type]
