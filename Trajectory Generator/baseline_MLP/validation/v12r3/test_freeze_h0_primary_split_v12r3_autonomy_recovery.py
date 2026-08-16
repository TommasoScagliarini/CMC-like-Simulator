from __future__ import annotations

import copy
import json
import os
from pathlib import Path

import pytest

import freeze_h0_primary_split_v12r3_autonomy_recovery as freeze


def _temporary_occupancy(tmp_path: Path) -> dict[str, Path]:
    return {
        "protocol_freeze": tmp_path / "protocol_freeze.json",
        "execution_lock": tmp_path / "execution_lock.json",
        "design_audit": tmp_path / "design_audit.json",
        "run_root": tmp_path / "run_root",
    }


def test_parent_v12_freeze_is_immutable_and_semantically_verified() -> None:
    gate = freeze._v12_protocol_freeze_gate()
    assert gate["passed"] is True
    assert all(gate["checks"].values())
    assert gate["artifact"] == {
        "path": freeze.contract.V12_PROTOCOL_FREEZE_PATH.as_posix(),
        "sha256": freeze.contract.V12_PROTOCOL_FREEZE_SHA256,
        "size_bytes": freeze.contract.V12_PROTOCOL_FREEZE_SIZE_BYTES,
    }


def test_sources_bind_every_r3_design_file_and_nested_attributes() -> None:
    records = freeze._source_records()
    assert set(records) == set(freeze.contract.SOURCE_RELATIVE_PATHS)
    for name in ("contract", "freeze", "contract_tests", "freeze_tests"):
        assert "v12r3" in str(records[name]["path"])
        assert len(str(records[name]["sha256"])) == 64
        assert int(records[name]["size_bytes"]) > 0
    assert records["line_endings_r1"]["path"] == (
        "Trajectory Generator/baseline_MLP/validation/.gitattributes"
    )
    assert records["line_endings_r2"]["path"] == (
        "Trajectory Generator/baseline_MLP/validation/v12r2/.gitattributes"
    )
    assert records["line_endings_r3"]["path"] == (
        "Trajectory Generator/baseline_MLP/validation/v12r3/.gitattributes"
    )


def test_nested_attributes_cover_r3_text_and_binary_artifacts() -> None:
    path = freeze.resolve_relative(
        freeze.contract.SOURCE_RELATIVE_PATHS["line_endings_r3"]
    )
    attributes = path.read_text(encoding="utf-8")
    assert "*.py text eol=lf" in attributes
    assert "*.json text eol=lf" in attributes
    for extension in (
        "npz",
        "npy",
        "pkl",
        "zip",
        "bin",
        "pt",
        "pth",
        "msgpack",
        "safetensors",
    ):
        assert f"h0_v12r3_run_*/**/*.{extension} -text" in attributes


def test_authority_is_design_only_and_rejects_execution_flag(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    gate = freeze._authority_gate()
    assert gate["passed"] is True
    assert all(gate["checks"].values())
    monkeypatch.setitem(freeze.contract.AUTHORITY, "environment_reset_authorized", True)
    broken = freeze._authority_gate()
    assert broken["passed"] is False
    assert broken["checks"]["all_execution_and_relaxation_flags_false"] is False


def test_path_isolation_is_additive_and_complete() -> None:
    audit = freeze._path_isolation_audit()
    assert audit["passed"] is True
    assert audit["violations"] == []
    assert audit["worker_claim_count"] == 26
    assert audit["stage_receipt_count"] == 26
    assert audit["maximum_relative_path_length"] < 160
    assert audit["maximum_temp_relative_path_length"] < 160
    assert audit["checks"]["exclusive_temp_relative_paths_short"] is True
    assert all(
        "h0_v12r3_run" in path
        for name, path in audit["declared_mutation_paths"].items()
        if name not in {"protocol_freeze", "execution_lock", "design_audit"}
    )


def test_build_passes_without_opening_execution_scope(tmp_path: Path) -> None:
    payload = freeze.build_protocol_freeze(
        require_unoccupied=True,
        occupancy_paths=_temporary_occupancy(tmp_path),
    )
    assert payload["schema_version"] == 123
    assert payload["passed"] is True
    assert all(payload["checks"].values())
    assert payload["immutable_v12_protocol_freeze"]["passed"] is True
    assert payload["probe_replay_schema_gate"]["passed"] is True
    assert payload["fit_contract_self_check"]["passed"] is True
    assert payload["v12r1_terminal_failure_lineage"]["passed"] is True
    assert payload["v12r2_terminal_failure_lineage"]["passed"] is True
    assert payload["probe_replay_schema"]["teacher_mutable_actor_columns"] == list(
        range(10, 25)
    )
    assert payload["execution_lock"] is None
    assert payload["pipeline_claim"] is None
    assert payload["actor_fit_executions"] == 0
    assert payload["environment_reset_calls"] == 0
    assert payload["environment_step_calls"] == 0
    assert payload["offline_teacher_label_calls"] == 0
    assert payload["actor_updates"] == 0
    assert payload["critic_updates"] == 0
    assert payload["ppo_updates"] == 0
    assert payload["next_stage"] == ("REQUIRE_SEPARATE_V12R3_NO_CLOBBER_EXECUTION_LOCK")
    assert not any(tmp_path.iterdir())


def test_build_adversarially_rejects_range_regression(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    broken = copy.deepcopy(freeze.contract.PROBE_REPLAY_SCHEMA)
    broken["teacher_mutable_actor_columns"] = list(range(10, 24))
    monkeypatch.setattr(freeze.contract, "PROBE_REPLAY_SCHEMA", broken)
    payload = freeze.build_protocol_freeze(
        require_unoccupied=False,
        occupancy_paths=_temporary_occupancy(tmp_path),
    )
    assert payload["passed"] is False
    assert payload["checks"]["corrected_teacher_columns_10_through_24"] is False
    assert payload["checks"]["self_contained_replay_schema"] is False


def test_build_adversarially_rejects_missing_event_or_config_field(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    broken = copy.deepcopy(freeze.contract.PROBE_REPLAY_SCHEMA)
    del broken["left_event_journal_arrays"]["legacy_left_event_startup_contact_present"]
    del broken["legacy_fsm_config_arrays"]["legacy_fsm_config_sha256_ascii"]
    monkeypatch.setattr(freeze.contract, "PROBE_REPLAY_SCHEMA", broken)
    payload = freeze.build_protocol_freeze(
        require_unoccupied=False,
        occupancy_paths=_temporary_occupancy(tmp_path),
    )
    assert payload["passed"] is False
    assert payload["checks"]["self_contained_replay_schema"] is False


def test_noncanonical_prepare_is_exact_no_clobber_and_tamper_evident(
    tmp_path: Path,
) -> None:
    destination = tmp_path / "protocol_freeze.json"
    payload = freeze.prepare_protocol_freeze(
        output_path=destination,
        enforce_canonical_destination=False,
    )
    assert payload["passed"] is True
    assert [path.name for path in tmp_path.iterdir()] == ["protocol_freeze.json"]
    assert (
        freeze.verify_protocol_freeze(
            input_path=destination,
            enforce_canonical_destination=False,
        )
        == payload
    )

    tampered = json.loads(destination.read_text(encoding="utf-8"))
    tampered["probe_replay_schema"]["teacher_mutable_actor_columns"] = list(
        range(10, 24)
    )
    destination.write_text(json.dumps(tampered), encoding="utf-8")
    with pytest.raises(freeze.V12R3ProtocolFreezeError, match="drifted"):
        freeze.verify_protocol_freeze(
            input_path=destination,
            enforce_canonical_destination=False,
        )
    with pytest.raises(freeze.V12R3ProtocolFreezeError, match="exists/no-clobber"):
        freeze.prepare_protocol_freeze(
            output_path=destination,
            enforce_canonical_destination=False,
        )


def test_canonical_outputs_are_in_a_valid_readiness_phase() -> None:
    occupied = tuple(
        os.path.lexists(freeze._lexical_absolute(path))
        for path in (
            freeze.contract.PROTOCOL_FREEZE_PATH,
            freeze.contract.DESIGN_AUDIT_RECEIPT_PATH,
            freeze.contract.EXECUTION_LOCK_PATH,
        )
    )
    assert occupied in {
        (False, False, False),
        (True, False, False),
        (True, True, False),
        (True, True, True),
    }
    run_exists = os.path.lexists(freeze._lexical_absolute(freeze.contract.RUN_ROOT))
    if run_exists:
        assert occupied == (True, True, True)
        assert os.path.lexists(
            freeze._lexical_absolute(freeze.contract.PIPELINE_CLAIM_PATH)
        )
        assert os.path.lexists(
            freeze._lexical_absolute(freeze.contract.PIPELINE_LEDGER_PATH)
        )


def test_v12r1_terminal_failure_lineage_is_complete_and_non_reusable() -> None:
    gate = freeze._v12r1_terminal_failure_gate()
    assert gate["passed"] is True
    assert all(gate["checks"].values())
    assert len(gate["artifacts"]) == 12
    assert gate["pipeline_ledger"]["attempted_stage"] == "fit_p0"


def test_v12r2_terminal_failure_lineage_is_complete_and_non_reusable() -> None:
    gate = freeze._v12r2_terminal_failure_gate()
    assert gate["passed"] is True
    assert all(gate["checks"].values())
    assert len(gate["lineage"]["artifacts"]) == 7
    assert gate["lineage"]["run_tree"] == (freeze.contract.V12R2_TERMINAL_RUN_TREE)
    assert gate["pipeline_ledger"]["attempted_stage"] == "probe_p0"
    assert gate["pipeline_ledger"]["completed_stages"] == ["fit_p0"]
    assert gate["pipeline_ledger"]["environment_step_calls"] == 232
