"""Read-only tests for the prospective V12R3-P1 qualification design freeze."""

from __future__ import annotations

import copy
import sys
from pathlib import Path

import pytest


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "validation").is_dir()
            and (candidate / "Trajectory Generator").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
ROOT_VALIDATION = REPO_ROOT / "validation"
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
V12P1Q_ROOT = Path(__file__).resolve().parent
for _root in (REPO_ROOT, ROOT_VALIDATION, LOCAL_VALIDATION, V12P1Q_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r3_p1_qualification_design as freeze  # noqa: E402
import h0_v12r3_p1_qualification_contract as contract  # noqa: E402


def _canonical_path() -> Path:
    return freeze.resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)


def test_build_preregisters_design_without_publication_or_execution() -> None:
    assert not _canonical_path().exists()
    assert not freeze.resolve_relative(contract.NOISE_ROOT).exists()
    assert not freeze.resolve_relative(contract.RUN_ROOT).exists()
    assert not freeze.resolve_relative(contract.SALVAGE_RUN_ROOT).exists()

    payload = freeze.build_design_freeze()

    assert payload["schema_version"] == 125
    assert payload["status"] == contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS
    assert payload["passed"] is True
    assert payload["freeze_kind"] == ("PRE_SALVAGE_INDEPENDENT_QUALIFICATION_DESIGN")
    assert payload["protocol_id"] == contract.PROTOCOL_ID
    assert payload["publication_destination"] == (
        contract.QUALIFICATION_DESIGN_FREEZE_PATH.as_posix()
    )
    assert payload["qualification_protocol_freeze"] is None
    assert payload["qualification_execution_lock"] is None
    assert payload["noise_manifest"] is None
    assert payload["salvage_artifact_hashes"] is None
    assert payload["runtime_promoted"] is False
    assert all(value == 0 for value in payload["zero_design_activity"].values())
    assert not _canonical_path().exists()
    assert not freeze.resolve_relative(contract.NOISE_ROOT).exists()
    assert not freeze.resolve_relative(contract.RUN_ROOT).exists()


def test_closed_source_list_binds_original_v6_and_current_design_sources() -> None:
    payload = freeze.build_design_freeze()
    assert payload["source_gate"]["passed"] is True
    assert list(payload["source_hashes"]) == list(contract.DESIGN_SOURCE_RELATIVE_PATHS)
    assert len(payload["source_hashes"]) == 8
    assert {
        name: payload["source_hashes"][name]
        for name in contract.V6_HOLDOUT_SOURCE_ARTIFACTS
    } == contract.V6_HOLDOUT_SOURCE_ARTIFACTS
    for name, path in contract.DESIGN_SOURCE_RELATIVE_PATHS.items():
        record = payload["source_hashes"][name]
        assert record["path"] == path
        assert len(record["sha256"]) == 64
        assert record["size_bytes"] > 0


def test_snapshot_binds_six_holdouts_matrix_tolerances_and_exact_p1() -> None:
    payload = freeze.build_design_freeze()
    snapshot = payload["design_snapshot"]
    assert payload["design_gate"]["passed"] is True
    assert snapshot["candidate_id"] == contract.P1_CANDIDATE_ID
    assert snapshot["candidate_module"] == contract.P1_CANDIDATE_MODULE
    assert len(snapshot["holdout_cases"]) == 6
    assert snapshot["holdout_cases"] == list(contract.HOLDOUT_CASES)
    assert len(snapshot["rollout_matrix"]) == 12
    assert snapshot["rollout_matrix"] == list(contract.ROLLOUT_MATRIX)
    assert snapshot["matrix_order"] == "BASELINE_SIX_THEN_CANDIDATE_SIX"
    assert snapshot["roles"]["baseline"]["actor_id"] == "original_h0"
    assert snapshot["roles"]["baseline"]["event_contract_id"] == "legacy_events"
    assert snapshot["roles"]["candidate"]["event_contract_id"] == (
        "binary_point_v25+heel_qualified_fsm_v2"
    )
    assert snapshot["roles"]["candidate"]["target_contract_id"] == (
        "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2"
    )
    assert snapshot["absolute_rollout_gates"]["morphology_weight"] == 0.0
    assert snapshot["pairwise_noninferiority"]["reserve_tolerances"] == [
        list(row) for row in contract.RESERVE_TOLERANCES
    ]
    assert snapshot["pairwise_noninferiority"]["sea_tolerances"] == [
        list(row) for row in contract.SEA_TOLERANCES
    ]
    assert snapshot["aggregate"]["required_passing_pair_count"] == 6
    assert snapshot["aggregate"]["compensation_or_averaging_allowed"] is False


def test_access_remains_locked_pending_same_candidate_salvage_six_of_six() -> None:
    payload = freeze.build_design_freeze()
    access = payload["qualification_access"]
    assert access["status"] == freeze.LOCKED_ACCESS_STATUS
    assert access["qualification_protocol_freeze"] is None
    assert access["qualification_execution_lock"] is None
    assert access["future_prerequisite_requirements"] == list(
        contract.FUTURE_PREREQUISITE_REQUIREMENTS
    )
    assert access["future_prerequisite_hashes"] is None
    assert access["hash_binding_deferred_until_salvage_terminal_pass"] is True
    assert access["required_salvage_rollout_count"] == 6
    assert access["required_salvage_passing_rollout_count"] == 6
    assert access["required_salvage_failed_rollout_count"] == 0
    assert access["required_salvage_terminal_status"] == (
        "PASS_H0_V12R3_P1_SALVAGE_DEVELOPMENT_TERMINAL"
    )
    assert access["noise_materialization_authorized"] is False
    assert access["qualification_execution_authorized"] is False


def test_design_and_protocol_freezes_have_distinct_paths_and_statuses() -> None:
    assert contract.QUALIFICATION_DESIGN_FREEZE_PATH != contract.PROTOCOL_FREEZE_PATH
    assert contract.QUALIFICATION_DESIGN_FREEZE_PATH != contract.EXECUTION_LOCK_PATH
    assert contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS != (
        contract.PROTOCOL_FREEZE_PASS_STATUS
    )
    assert contract.QUALIFICATION_DESIGN_FREEZE_FAIL_STATUS != (
        contract.PROTOCOL_FREEZE_FAIL_STATUS
    )


def test_original_v6_hash_drift_fails_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    drifted = copy.deepcopy(contract.V6_HOLDOUT_SOURCE_ARTIFACTS)
    drifted["v6_qualification_contract"]["sha256"] = "0" * 64
    monkeypatch.setattr(contract, "V6_HOLDOUT_SOURCE_ARTIFACTS", drifted)

    payload = freeze.build_design_freeze()

    assert payload["passed"] is False
    assert payload["checks"]["closed_source_hashes_bound"] is False
    assert payload["source_gate"]["checks"]["original_v6_sources_byte_exact"] is False


def test_p1_v26_design_drift_fails_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(contract, "EVENT_CONTRACT_ID", "older_event_contract")

    payload = freeze.build_design_freeze()

    assert payload["passed"] is False
    assert payload["checks"]["exact_design_preregistered"] is False
    assert (
        payload["design_gate"]["checks"]["candidate_v26_primary_split_exact"] is False
    )


def test_pre_salvage_or_qualification_occupancy_is_fail_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    occupied = freeze._occupancy_snapshot()
    occupied["salvage_run_root_absent"] = False
    monkeypatch.setattr(freeze, "_occupancy_snapshot", lambda: occupied)

    payload = freeze.build_design_freeze()

    assert payload["passed"] is False
    assert payload["checks"]["salvage_run_root_absent"] is False


def test_temp_publish_verify_is_atomic_no_clobber_and_noncanonical_only_for_tests(
    tmp_path: Path,
) -> None:
    destination = tmp_path / "qualification_design_freeze.json"

    published = freeze.publish_design_freeze(
        output_path=destination,
        enforce_canonical_destination=False,
    )

    assert published["passed"] is True
    assert (
        freeze.verify_design_freeze(
            input_path=destination,
            enforce_canonical_destination=False,
        )
        == published
    )
    with pytest.raises(freeze.V12R3P1QualificationDesignFreezeError, match="clobber"):
        freeze.publish_design_freeze(
            output_path=destination,
            enforce_canonical_destination=False,
        )
    assert not _canonical_path().exists()


def test_noncanonical_publish_requires_explicit_test_override(tmp_path: Path) -> None:
    destination = tmp_path / "not_canonical.json"
    with pytest.raises(
        freeze.V12R3P1QualificationDesignFreezeError, match="non-canonical"
    ):
        freeze.publish_design_freeze(output_path=destination)
    assert not destination.exists()
    assert not _canonical_path().exists()


def test_verify_rejects_byte_drift(tmp_path: Path) -> None:
    destination = tmp_path / "qualification_design_freeze.json"
    freeze.publish_design_freeze(
        output_path=destination,
        enforce_canonical_destination=False,
    )
    destination.write_bytes(b"{}\n")

    with pytest.raises(freeze.V12R3P1QualificationDesignFreezeError, match="drifted"):
        freeze.verify_design_freeze(
            input_path=destination,
            enforce_canonical_destination=False,
        )


def test_verify_rejects_windows_reparse_or_junction_semantics(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    destination = tmp_path / "qualification_design_freeze.json"
    freeze.publish_design_freeze(
        output_path=destination,
        enforce_canonical_destination=False,
    )
    original = freeze._is_link_or_reparse
    monkeypatch.setattr(
        freeze,
        "_is_link_or_reparse",
        lambda path: Path(path) == destination or original(Path(path)),
    )

    with pytest.raises(
        freeze.V12R3P1QualificationDesignFreezeError, match="link/reparse"
    ):
        freeze.verify_design_freeze(
            input_path=destination,
            enforce_canonical_destination=False,
        )


def test_source_only_cli_build_does_not_publish(
    capsys: pytest.CaptureFixture[str],
) -> None:
    assert freeze.main(["--build-only"]) == 0
    output = capsys.readouterr().out
    assert contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS in output
    assert not _canonical_path().exists()
