"""Focused source-only orchestration tests for the V12R7 one-shot runner."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path, PurePosixPath
from typing import Any

import pytest


HERE = Path(__file__).resolve().parent
if os.fspath(HERE) not in sys.path:
    sys.path.insert(0, os.fspath(HERE))

import h0_v12r7_recovery_contract as contract  # noqa: E402
import run_h0_v12r7_recovery as runner  # noqa: E402


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
        with pytest.raises(runner.V12R7ExecutionError, match="non-negative int"):
            runner._increment(activity, "environment_step_calls", amount)
    with pytest.raises(runner.V12R7ExecutionError, match="escaped repository"):
        runner._path(tmp_path.parent / "outside.json")
    with pytest.raises(runner.V12R7ExecutionError, match="unsafe repository path"):
        runner._path(r"nested\windows-alias.json")


def _patch_ephemeral_contract(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr(runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(runner.freezer, "REPO_ROOT", tmp_path)
    values = {
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
    with pytest.raises(runner.V12R7ExecutionError, match="stopped terminally"):
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


def test_public_receipt_verifier_hooks_are_no_arg_callable() -> None:
    for hook in (
        runner.verify_candidate_freeze_receipt,
        runner.verify_final_development_receipt,
        runner.verify_terminal_ledger,
    ):
        assert callable(hook)
