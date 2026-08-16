from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

import freeze_h0_primary_split_v12_autonomy_recovery as freeze


def _temporary_occupancy(tmp_path: Path) -> dict[str, Path]:
    return {
        "protocol_freeze": tmp_path / "protocol_freeze.json",
        "execution_lock": tmp_path / "execution_lock.json",
        "design_audit": tmp_path / "design_audit.json",
        "run_root": tmp_path / "run_root",
    }


def test_artifact_snapshot_is_type_aware_and_repeatable(tmp_path: Path) -> None:
    missing = tmp_path / "missing"
    assert freeze.artifact_snapshot(missing) == {
        "path": str(missing.resolve()),
        "lexists": False,
    }
    regular = tmp_path / "value.json"
    regular.write_bytes(b"{}\n")
    first = freeze.artifact_snapshot(regular)
    second = freeze.artifact_snapshot(regular)
    assert first == second
    assert first["kind"] == "regular"
    assert first["size_bytes"] == 3
    assert len(first["sha256"]) == 64

    link = tmp_path / "link"
    try:
        link.symlink_to(regular.name)
    except OSError:
        return
    linked = freeze.artifact_snapshot(link)
    assert linked["lexists"] is True
    assert linked["kind"] == "symlink"
    assert linked["target"] == regular.name


def test_v11_terminal_hashes_and_failure_are_authoritative() -> None:
    sources = freeze._source_records()
    inputs = freeze._input_records()
    hashes = freeze._authoritative_hash_gate(sources, inputs)
    assert hashes["passed"] is True
    assert all(hashes["checks"].values())
    terminal = freeze._v11_terminal_gate()
    assert terminal["passed"] is True
    assert all(terminal["checks"].values())
    assert len(terminal["failure_artifact_chain"]["steps"]) == 259
    provenance = freeze._v11_fit_and_freeze_provenance_audit()
    assert provenance["passed"] is True
    assert all(provenance["checks"].values())
    assert len(provenance["fit_receipts"]) == 4
    assert all(row["passed"] for row in provenance["fit_receipts"])
    authority = freeze._authority_gate()
    assert authority["passed"] is True
    assert all(authority["checks"].values())


def test_v11_shielded_recovery_audit_exposes_teacher_dependence() -> None:
    audit = freeze._v11_collection_recovery_audit()
    assert audit["passed"] is True
    assert len(audit["cases"]) == 6
    aggregate = audit["aggregate"]
    assert aggregate["rows"] == 3000
    assert aggregate["latch_active_rows"] == 1597
    assert aggregate["latch_fraction"] == pytest.approx(1597 / 3000)
    assert aggregate["teacher_dependent_rows"] == 3000
    assert aggregate["near_latch_rows"] > 0
    assert aggregate["current_at_or_above_24mm_rows"] == 8
    assert aggregate["current_at_or_above_25mm_rows"] == 0
    assert audit["checks"]["candidate_freeze_to_receipts_bound"] is True
    assert audit["checks"]["receipts_to_artifacts_bound"] is True
    assert audit["checks"]["receipts_to_claims_bound"] is True
    assert all(row["receipt_artifact_binding_passed"] for row in audit["cases"])
    assert all(row["receipt_claim_binding_passed"] for row in audit["cases"])
    deterministic_r3 = next(
        row
        for row in audit["cases"]
        if row["round_index"] == 3
        and row["case_id"] == "deterministic_offset_minus_0p20"
    )
    assert deterministic_r3["latch_intervals_inclusive"] == [
        [47, 135],
        [214, 293],
        [364, 456],
    ]
    assert deterministic_r3["max_consecutive_latch_steps"] == 93


def test_v11_corpus_and_float32_coverage_receipt_are_recomputed() -> None:
    audit = freeze._v11_corpus_audit()
    assert audit["passed"] is True
    assert all(audit["checks"].values())
    assert audit["seed_rows"] == 6000
    assert audit["risk_reweighted_seed_rows"] == 0
    coverage = audit["coverage_audit"]
    assert coverage["feature_count"] == 33
    assert coverage["loo_p95"] == pytest.approx(0.07945888479650812, abs=1.0e-12)
    assert coverage["hashes"]["nearest_indices"] == (
        "aa79fa7e9ab5c4145caa332f9984612d009af2fee7771550ab68db908ce42639"
    )
    assert coverage["tie_audit"] == {
        "extended_query_k": 64,
        "unique_observation_count": 5990,
        "maximum_minimum_distance_tie_count": 6,
        "query_k_matches_extended_query": True,
        "query_and_extended_tie_counts_match": True,
    }


def test_v11_pure_failure_branch_is_bound_and_out_of_coverage() -> None:
    audit = freeze._v11_pure_failure_coverage_audit()
    assert audit["passed"] is True
    assert all(audit["checks"].values())
    assert audit["row_count"] == 259
    assert audit["ood_row_count"] == 164
    assert audit["all_steps_201_through_259_ood"] is True
    assert audit["step_259_distance"] == pytest.approx(0.4934425626710174)
    assert audit["hashes"]["nearest_distances"] == (
        "2be58bb58dbb70af4754874be96847a44b579ac4ea4eb5dafdedc8282a5da75b"
    )
    assert audit["minimum_global_certificate_margin_sum"] > 0.69


def test_path_isolation_covers_every_declared_future_write() -> None:
    audit = freeze._path_isolation_audit()
    assert audit["passed"] is True
    assert audit["violations"] == []
    assert audit["maximum_relative_path_length"] < 160
    assert audit["maximum_checkout_root_length_for_legacy_windows"] > 0
    assert audit["maximum_checkout_root_length_for_preferred_windows"] > 0
    assert audit["worker_claim_count"] == 26
    assert audit["stage_receipt_count"] == 26
    assert "protocol_freeze" in audit["declared_mutation_paths"]
    for stage in freeze.contract.FIT_STAGES:
        assert f"fit_root_{stage}" in audit["declared_mutation_paths"]
        assert f"probe_root_{stage}" in audit["declared_mutation_paths"]
        assert f"label_root_{stage}" in audit["declared_mutation_paths"]

    escaped = dict(freeze.contract.declared_mutation_paths())
    escaped["escape"] = "reports/not_allowed.json"
    assert freeze._path_isolation_audit(escaped)["passed"] is False
    duplicated = dict(freeze.contract.declared_mutation_paths())
    duplicated["duplicate"] = freeze.contract.RUN_ROOT
    duplicate_audit = freeze._path_isolation_audit(duplicated)
    assert duplicate_audit["passed"] is False
    assert duplicate_audit["duplicate_values"] == [freeze.contract.RUN_ROOT.as_posix()]


def test_authority_gate_rejects_any_execution_flag(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setitem(freeze.contract.AUTHORITY, "ppo_updates_authorized", True)
    gate = freeze._authority_gate()
    assert gate["passed"] is False
    assert gate["checks"]["all_execution_and_relaxation_flags_false"] is False


def test_line_endings_do_not_classify_binary_artifacts_as_text() -> None:
    attributes = (freeze.REPO_ROOT / ".gitattributes").read_text(encoding="utf-8")
    prefix = "Trajectory Generator/baseline_MLP/validation/h0_v12_runs"
    assert f'"{prefix}/**" text eol=lf' not in attributes
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
        assert f'"{prefix}/**/*.{extension}" -text' in attributes


def test_build_is_idempotent_wrt_historical_v11_receipt(tmp_path: Path) -> None:
    historical = freeze.resolve_relative(
        freeze.contract.prior.DESIGN_AUDIT_RECEIPT_PATH
    )
    before = freeze.artifact_snapshot(historical)
    payload = freeze.build_protocol_freeze(
        require_unoccupied=True,
        occupancy_paths=_temporary_occupancy(tmp_path),
    )
    after = freeze.artifact_snapshot(historical)
    assert before == after
    assert before["lexists"] is True
    assert payload["passed"] is True
    assert payload["checks"]["v11_design_receipt_idempotent"] is True
    assert payload["v11_design_receipt_snapshot_before"]["path"] == (
        freeze.contract.prior.DESIGN_AUDIT_RECEIPT_PATH.as_posix()
    )
    assert payload["actor_fit_executions"] == 0
    assert payload["environment_reset_calls"] == 0
    assert payload["environment_step_calls"] == 0
    assert payload["next_stage"] == (
        "WAIT_EXPLICIT_V12_ONE_SHOT_EXECUTION_AUTHORIZATION"
    )


def test_noncanonical_prepare_writes_one_freeze_and_never_clobbers(
    tmp_path: Path,
) -> None:
    destination = tmp_path / "protocol_freeze.json"
    payload = freeze.prepare_protocol_freeze(
        output_path=destination,
        enforce_canonical_destination=False,
    )
    assert payload["passed"] is True
    assert [path.name for path in tmp_path.iterdir()] == ["protocol_freeze.json"]
    verified = freeze.verify_protocol_freeze(
        input_path=destination,
        enforce_canonical_destination=False,
    )
    assert verified == payload

    tampered = json.loads(destination.read_text(encoding="utf-8"))
    tampered["coverage_weighting"]["loo_p95"] = 0.5
    destination.write_text(json.dumps(tampered), encoding="utf-8")
    with pytest.raises(freeze.V12ProtocolFreezeError, match="drifted"):
        freeze.verify_protocol_freeze(
            input_path=destination,
            enforce_canonical_destination=False,
        )
    with pytest.raises(freeze.V12ProtocolFreezeError, match="exists/no-clobber"):
        freeze.prepare_protocol_freeze(
            output_path=destination,
            enforce_canonical_destination=False,
        )


def test_prepare_rejects_a_broken_destination_symlink(tmp_path: Path) -> None:
    destination = tmp_path / "protocol_freeze.json"
    try:
        destination.symlink_to("missing-target.json")
    except OSError:
        return
    with pytest.raises(freeze.V12ProtocolFreezeError, match="exists/no-clobber"):
        freeze.prepare_protocol_freeze(
            output_path=destination,
            enforce_canonical_destination=False,
        )


def test_prepare_rejects_missing_or_symlinked_parent(tmp_path: Path) -> None:
    missing_destination = tmp_path / "missing" / "protocol_freeze.json"
    with pytest.raises(freeze.V12ProtocolFreezeError, match="must already exist"):
        freeze.prepare_protocol_freeze(
            output_path=missing_destination,
            enforce_canonical_destination=False,
        )
    assert not missing_destination.parent.exists()

    real_parent = tmp_path / "real"
    real_parent.mkdir()
    linked_parent = tmp_path / "linked"
    try:
        linked_parent.symlink_to(real_parent, target_is_directory=True)
    except OSError:
        return
    with pytest.raises(freeze.V12ProtocolFreezeError, match="symlink"):
        freeze.prepare_protocol_freeze(
            output_path=linked_parent / "protocol_freeze.json",
            enforce_canonical_destination=False,
        )
    assert not (real_parent / "protocol_freeze.json").exists()


def test_canonical_freeze_is_verifiable_if_already_published(tmp_path: Path) -> None:
    canonical = freeze.resolve_relative(freeze.contract.PROTOCOL_FREEZE_PATH)
    if os.path.lexists(canonical):
        payload = freeze.verify_protocol_freeze()
        assert payload["passed"] is True
    else:
        payload = freeze.build_protocol_freeze(
            require_unoccupied=True,
            occupancy_paths=_temporary_occupancy(tmp_path),
        )
        assert payload["checks"]["protocol_freeze_unoccupied"] is True
