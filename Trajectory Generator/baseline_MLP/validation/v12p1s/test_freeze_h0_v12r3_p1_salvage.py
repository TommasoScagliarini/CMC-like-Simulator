"""Read-only tests for the additive V12R3-P1 selection freeze."""

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
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
V12P1S_ROOT = Path(__file__).resolve().parent
for _root in (REPO_ROOT, REPO_ROOT / "validation", LOCAL_VALIDATION, V12P1S_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r3_p1_salvage as freeze  # noqa: E402
import h0_v12r3_p1_salvage_contract as contract  # noqa: E402


def _canonical_path() -> Path:
    return freeze.resolve_relative(contract.PROTOCOL_FREEZE_PATH)


def _qualification_occupancy() -> dict[str, bool]:
    return {
        "design_freeze_unoccupied": True,
        "qualification_protocol_freeze_absent": True,
        "qualification_execution_lock_absent": True,
        "qualification_noise_root_absent": True,
        "qualification_run_root_absent": True,
        "salvage_run_root_absent": True,
    }


def _bind_temp_q_design_fixture(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> Path:
    q_fixture = tmp_path / "h0_v12r3_p1_qualification_design_freeze.json"
    monkeypatch.setattr(contract, "QUALIFICATION_DESIGN_FREEZE_PATH", q_fixture)
    monkeypatch.setattr(
        freeze.q_contract,
        "QUALIFICATION_DESIGN_FREEZE_PATH",
        q_fixture,
    )
    payload = freeze.q_design_freezer._assemble_design_freeze(
        _qualification_occupancy()
    )
    q_fixture.write_bytes(freeze.forensic.canonical_json_bytes(payload))

    original_record = freeze._record

    def fixture_aware_record(path: object) -> dict[str, object]:
        source = freeze.resolve_relative(path)
        if source == q_fixture:
            return {
                "path": q_fixture.as_posix(),
                "sha256": freeze.forensic.sha256_file(q_fixture),
                "size_bytes": q_fixture.stat().st_size,
            }
        return original_record(path)

    monkeypatch.setattr(freeze, "_record", fixture_aware_record)
    return q_fixture


def test_build_freezes_unique_p1_without_execution_or_publication() -> None:
    assert not _canonical_path().exists()
    before = freeze._tree_summary(
        freeze._tree_record(contract.R3_TERMINAL_RUN_TREE["path"])
    )

    payload = freeze.build_protocol_freeze()

    after = freeze._tree_summary(
        freeze._tree_record(contract.R3_TERMINAL_RUN_TREE["path"])
    )
    assert payload["schema_version"] == 124
    assert payload["status"] == contract.PROTOCOL_FREEZE_PASS_STATUS
    assert payload["passed"] is True
    assert payload["protocol_id"] == contract.PROTOCOL_ID
    assert payload["lineage_kind"] == "ADDITIVE_NEW_LINEAGE_NOT_V12R3_RETRY"
    assert payload["selected_candidate_id"] == contract.P1_CANDIDATE_ID
    assert payload["selected_candidate"] == contract.P1_CANDIDATE_MODULE
    assert payload["selected_candidate_fit_stage"] == "p1"
    assert payload["candidate_selection_gate"]["passed"] is True
    assert payload["r3_terminal_lineage"] == contract.R3_TERMINAL_LINEAGE
    assert payload["r3_selected_evidence"] == contract.R3_SELECTED_ARTIFACTS
    assert payload["zero_freeze_activity"] == freeze.ZERO_FREEZE_ACTIVITY
    assert all(value == 0 for value in payload["zero_freeze_activity"].values())
    assert payload["execution_lock"] is None
    assert payload["pipeline_claim"] is None
    assert payload["v12r3_reopened"] is False
    assert payload["runtime_promoted"] is False
    assert before == after == contract.R3_TERMINAL_RUN_TREE
    assert not _canonical_path().exists()


def test_eleven_selected_records_and_complete_terminal_tree_are_exact() -> None:
    payload = freeze.build_protocol_freeze()
    assert len(payload["r3_selected_evidence"]) == 11
    assert set(payload["r3_selected_evidence"]) == {
        "r3_protocol_freeze",
        "r3_design_audit",
        "r3_execution_lock",
        "r3_pipeline_claim",
        "r3_pipeline_ledger",
        "r3_fit_p1_receipt",
        "r3_fit_p1_gate",
        "r3_fit_p1_summary",
        "r3_probe_p1_receipt",
        "r3_probe_p1_gate",
        "r3_probe_p1_summary",
    }
    assert payload["r3_selected_evidence_gate"]["passed"] is True
    assert payload["r3_terminal_lineage_gate"]["passed"] is True
    assert payload["r3_terminal_lineage"]["run_tree"] == {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/v12r3/h0_v12r3_run_20260809"
        ),
        "tree_sha256": (
            "1248381ad63d990c59b9aaff812385b606bdbf1516814c497f32a53a0d1ee50e"
        ),
        "file_count": 2819,
        "total_size_bytes": 79261095,
    }
    assert payload["r3_terminal_lineage"]["semantics"] == (
        contract.R3_TERMINAL_SEMANTICS
    )


def test_p2_is_exhaustively_bound_as_terminal_history_and_never_selected() -> None:
    payload = freeze.build_protocol_freeze()
    exclusion = payload["excluded_p2_terminal_history"]
    assert exclusion["passed"] is True
    assert exclusion["selection_allowed"] is False
    assert exclusion["input_allowed"] is False
    assert exclusion["module_loaded"] is False
    assert exclusion["corpus_loaded"] is False
    assert exclusion["excluded_paths"] == [
        path.as_posix() for path in freeze.EXCLUDED_P2_HISTORY_PATHS
    ]
    assert len(exclusion["excluded_paths"]) == 7
    assert payload["candidate_selection_payload"]["p2_artifacts_used"] == []
    assert payload["candidate_selection_payload"]["p2_module_loaded"] is False
    assert payload["candidate_selection_payload"]["p2_corpus_loaded"] is False
    assert all(
        "/fit/p2/" not in artifact["path"]
        for artifact in payload["r3_selected_evidence"].values()
    )


def test_p1_selection_is_unique_and_uses_no_label_p1_receipt_or_gate() -> None:
    payload = freeze.build_protocol_freeze()
    eligibility = payload["unique_eligibility_audit"]
    selection = payload["candidate_selection_payload"]
    assert eligibility["passed"] is True
    assert all(eligibility["p1_offline_metric_checks"].values())
    assert eligibility["candidate_module"] == contract.P1_CANDIDATE_MODULE
    assert selection["unique_eligible_candidate"] is True
    assert selection["p1_fit_gate_passed"] is True
    assert selection["p1_probe_integrity_passed"] is True
    assert selection["p1_probe_autonomy_passed"] is True
    assert selection["label_p1_used_for_candidate_selection"] is False
    selected_paths = [
        artifact["path"] for artifact in payload["r3_selected_evidence"].values()
    ]
    assert not any(path.endswith("/label/p1/receipt.json") for path in selected_paths)
    assert not any(path.endswith("/label/p1/gate.json") for path in selected_paths)


def test_source_hashes_and_all_mutation_paths_are_isolated() -> None:
    payload = freeze.build_protocol_freeze()
    assert payload["source_gate"]["passed"] is True
    assert {
        "h0_v12r3_p1_salvage_contract.py",
        "freeze_h0_v12r3_p1_salvage.py",
        "test_freeze_h0_v12r3_p1_salvage.py",
    } <= set(payload["source_hashes"])
    assert payload["write_path_isolation"]["passed"] is True
    root = f"{contract.VALIDATION_ROOT.as_posix()}/"
    old = f"{contract.R3_TERMINAL_RUN_TREE['path']}/"
    for path in payload["write_path_isolation"]["declared_mutation_paths"].values():
        assert path == contract.VALIDATION_ROOT.as_posix() or path.startswith(root)
        assert not path.startswith(old)


def test_selected_hash_drift_fails_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    drifted = copy.deepcopy(contract.R3_SELECTED_ARTIFACTS)
    drifted["r3_fit_p1_receipt"]["sha256"] = "0" * 64
    monkeypatch.setattr(contract, "R3_SELECTED_ARTIFACTS", drifted)

    payload = freeze.build_protocol_freeze()

    assert payload["passed"] is False
    assert payload["checks"]["eleven_selected_r3_records"] is False
    assert payload["r3_selected_evidence_gate"]["checks"]["records_byte_exact"] is False


def test_terminal_tree_drift_fails_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    drifted = copy.deepcopy(contract.R3_TERMINAL_RUN_TREE)
    drifted["tree_sha256"] = "0" * 64
    monkeypatch.setattr(contract, "R3_TERMINAL_RUN_TREE", drifted)

    payload = freeze.build_protocol_freeze()

    assert payload["passed"] is False
    assert payload["checks"]["complete_terminal_r3_tree"] is False


def test_incomplete_p2_exclusion_fails_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        freeze,
        "EXCLUDED_P2_HISTORY_PATHS",
        freeze.EXCLUDED_P2_HISTORY_PATHS[:-1],
    )

    payload = freeze.build_protocol_freeze()

    assert payload["passed"] is False
    assert payload["checks"]["p2_explicitly_excluded"] is False
    assert (
        payload["excluded_p2_terminal_history"]["checks"][
            "all_seven_p2_history_paths_enumerated"
        ]
        is False
    )


def test_path_escape_fails_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    original = contract.declared_mutation_paths

    def escaped() -> dict[str, object]:
        paths = dict(original())
        paths["illegal_r3_write"] = contract.R3_TERMINAL_RUN_TREE["path"]
        return paths

    monkeypatch.setattr(contract, "declared_mutation_paths", escaped)

    payload = freeze.build_protocol_freeze()

    assert payload["passed"] is False
    assert payload["checks"]["write_paths_isolated"] is False
    assert (
        payload["write_path_isolation"]["checks"]["all_mutations_inside_new_lineage"]
        is False
    )


def test_missing_q_design_freeze_verifier_fails_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def missing_design_freeze() -> dict[str, object]:
        raise FileNotFoundError("qualification design freeze fixture is absent")

    monkeypatch.setattr(
        freeze.q_design_freezer,
        "verify_design_freeze",
        missing_design_freeze,
    )

    with pytest.raises(
        freeze.V12R3P1SalvageFreezeError,
        match="canonical qualification design freeze verification failed",
    ):
        freeze.build_protocol_freeze()


def test_fail_q_design_freeze_payload_fails_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    failed = copy.deepcopy(freeze.q_design_freezer.verify_design_freeze())
    failed["status"] = freeze.q_contract.QUALIFICATION_DESIGN_FREEZE_FAIL_STATUS
    failed["passed"] = False
    failed["checks"]["salvage_run_root_absent"] = False
    monkeypatch.setattr(
        freeze.q_design_freezer,
        "verify_design_freeze",
        lambda: copy.deepcopy(failed),
    )

    with pytest.raises(
        freeze.V12R3P1SalvageFreezeError,
        match="qualification design semantics are not PASS",
    ):
        freeze.build_protocol_freeze()


def test_noncanonical_q_design_freeze_path_fails_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    noncanonical = (
        freeze.q_contract.QUALIFICATION_DESIGN_FREEZE_PATH.parent
        / "noncanonical_qualification_design_freeze.json"
    )
    monkeypatch.setattr(
        freeze.q_contract,
        "QUALIFICATION_DESIGN_FREEZE_PATH",
        noncanonical,
    )

    with pytest.raises(
        freeze.V12R3P1SalvageFreezeError,
        match="qualification design canonical identity drifted",
    ):
        freeze.build_protocol_freeze()


def test_q_fixture_byte_mutation_after_s_freeze_fails_s_verification(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    q_fixture = _bind_temp_q_design_fixture(monkeypatch, tmp_path)
    s_freeze = tmp_path / "p1_salvage_freeze.json"
    published = freeze.publish_protocol_freeze(
        output_path=s_freeze,
        enforce_canonical_destination=False,
    )
    assert published["qualification_design_freeze"]["sha256"] == (
        freeze.forensic.sha256_file(q_fixture)
    )

    q_fixture.write_bytes(q_fixture.read_bytes() + b" ")

    with pytest.raises(
        freeze.V12R3P1SalvageFreezeError,
        match="qualification design freeze verification failed",
    ):
        freeze.verify_protocol_freeze(
            input_path=s_freeze,
            enforce_canonical_destination=False,
        )


def test_publish_rejects_symlinked_destination_component(tmp_path: Path) -> None:
    real_root = tmp_path / "real"
    real_root.mkdir()
    linked_root = tmp_path / "linked"
    try:
        linked_root.symlink_to(real_root, target_is_directory=True)
    except OSError as exc:  # pragma: no cover - restricted Windows test host.
        pytest.skip(f"symlink creation is unavailable: {exc}")

    with pytest.raises(
        freeze.V12R3P1SalvageFreezeError,
        match="link/reparse path rejected",
    ):
        freeze.publish_protocol_freeze(
            output_path=linked_root / "p1_salvage_freeze.json",
            enforce_canonical_destination=False,
        )
    assert not (real_root / "p1_salvage_freeze.json").exists()


def test_publish_rejects_simulated_windows_reparse_component(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    junction = tmp_path / "junction"
    junction.mkdir()
    original_lstat = freeze.os.lstat

    class ReparseMetadata:
        def __init__(self, metadata: object) -> None:
            self.st_mode = metadata.st_mode
            self.st_file_attributes = (
                getattr(metadata, "st_file_attributes", 0)
                | freeze.stat.FILE_ATTRIBUTE_REPARSE_POINT
            )

    def simulated_lstat(path: object) -> object:
        metadata = original_lstat(path)
        if Path(path) == junction:
            return ReparseMetadata(metadata)
        return metadata

    monkeypatch.setattr(freeze.os, "lstat", simulated_lstat)

    with pytest.raises(
        freeze.V12R3P1SalvageFreezeError,
        match="link/reparse path rejected",
    ):
        freeze.publish_protocol_freeze(
            output_path=junction / "p1_salvage_freeze.json",
            enforce_canonical_destination=False,
        )
    assert not (junction / "p1_salvage_freeze.json").exists()


def test_temp_publish_verify_is_atomic_and_no_clobber(tmp_path: Path) -> None:
    destination = tmp_path / "p1_salvage_freeze.json"

    published = freeze.publish_protocol_freeze(
        output_path=destination,
        enforce_canonical_destination=False,
    )

    assert published["passed"] is True
    assert (
        freeze.verify_protocol_freeze(
            input_path=destination,
            enforce_canonical_destination=False,
        )
        == published
    )
    with pytest.raises(freeze.V12R3P1SalvageFreezeError, match="clobber"):
        freeze.publish_protocol_freeze(
            output_path=destination,
            enforce_canonical_destination=False,
        )
    assert not _canonical_path().exists()


def test_noncanonical_publish_requires_explicit_test_override(tmp_path: Path) -> None:
    destination = tmp_path / "not_canonical.json"
    with pytest.raises(freeze.V12R3P1SalvageFreezeError, match="non-canonical"):
        freeze.publish_protocol_freeze(output_path=destination)
    assert not destination.exists()
    assert not _canonical_path().exists()


def test_verify_rejects_byte_drift(tmp_path: Path) -> None:
    destination = tmp_path / "p1_salvage_freeze.json"
    freeze.publish_protocol_freeze(
        output_path=destination,
        enforce_canonical_destination=False,
    )
    destination.write_bytes(b"{}\n")

    with pytest.raises(freeze.V12R3P1SalvageFreezeError, match="drifted"):
        freeze.verify_protocol_freeze(
            input_path=destination,
            enforce_canonical_destination=False,
        )
    assert not _canonical_path().exists()
