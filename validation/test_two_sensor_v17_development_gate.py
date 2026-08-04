"""Tests for the fail-closed V17 development gate."""

from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path

import pytest


VALIDATION_ROOT = Path(__file__).resolve().parent
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import evaluate_two_sensor_v17_development_gate as subject  # noqa: E402


def _actual_receipt() -> dict:
    return subject.build_receipt()


def test_actual_pinned_development_evidence_closes_v17_stage() -> None:
    receipt = _actual_receipt()
    assert receipt["status"] == "FAIL"
    assert receipt["decision"] == "TERMINAL_SEQUENTIAL_FAIL_NO_RETUNING"
    assert receipt["data_access"]["selected_trials"] == ["02", "04", "08"]
    assert receipt["data_access"]["selected_row_count"] == 12
    assert receipt["data_access"]["protected_trials_opened"] == []
    assert receipt["data_access"]["reserve_trials_opened"] == []
    assert receipt["stage_controls"]["next_gate_allowed"] is False
    assert receipt["stage_controls"]["protected_validation_open_allowed"] is False


def test_profile_physics_and_detector_contract_are_exact_v13() -> None:
    identity = _actual_receipt()["profile_and_contract_identity"]
    assert identity == {
        "ground_exact_v13": True,
        "material_exact_v13": True,
        "geometry_exact_v13": True,
        "sensor_count": 2,
        "roles": ["left_heel", "left_toe"],
        "applies_force": False,
        "threshold_and_dwell_exact_v14_2": True,
    }


def test_canonical_oracle_builder_is_hash_bound_and_has_no_protected_dispatch() -> None:
    record = _actual_receipt()["source_records"]["canonical_oracle_builder"]
    assert subject.gate.is_sha256(record["sha256"])
    assert record["event_contract_id"] == subject.EVENT_CONTRACT_ID
    assert record["authorized_development_trials"] == ["02", "04", "08"]
    assert record["forbidden_trials"] == ["01", "03", "05", "06", "07"]


def test_sequential_gate_reports_the_observed_failures_and_passes() -> None:
    sequential = _actual_receipt()["sequential_1ms"]
    by_id = {check["id"]: check for check in sequential["checks"]}
    assert by_id["precision_exact_one"]["status"] == "FAIL"
    assert by_id["recall_exact_one"]["status"] == "FAIL"
    assert by_id["f1_at_least_0p95"]["status"] == "PASS"
    assert by_id["iou_at_least_0p90"]["status"] == "PASS"
    assert by_id["confirmed_hs_error_at_most_0p05s"]["status"] == "FAIL"
    assert by_id["confirmed_to_error_at_most_0p08s"]["status"] == "PASS"
    assert by_id["zero_incomplete_heel_to_forefoot_transfers"]["status"] == "PASS"
    assert by_id["minimum_toe_clear_at_least_0p03s"]["status"] == "FAIL"
    assert by_id["mesh_geometry_pre_gate"]["status"] == "PASS"
    assert by_id["exact_event_counts_order_and_cycles"]["status"] == "PASS"
    assert by_id["required_metrics_finite"]["status"] == "PASS"


def test_terminal_failure_survives_the_canonical_one_ms_reference_shift() -> None:
    direct = _actual_receipt()["sequential_1ms"]["canonical_direct_rescore"]
    proof = direct["terminal_failure_proof"]
    assert proof["status"] == "FAIL"
    assert proof["unit"] == "02:3"
    assert proof["canonical_event_time_s"] == 81.722
    assert proof["absolute_error_s"] > proof["maximum_allowed_s"]
    assert direct["maximum_legacy_to_canonical_reference_shift_s"] <= 0.001
    assert direct["failed_checks"] == [
        "canonical_precision_exact_one",
        "canonical_recall_exact_one",
        "canonical_confirmed_hs_error_at_most_0p05s",
    ]


def test_batch_and_delivery_evidence_are_never_fabricated() -> None:
    receipt = _actual_receipt()
    not_executed = {item["id"]: item for item in receipt["not_executed"]}
    assert not_executed["batched_10ms_same_samples_parity"]["status"] == "NOT_EXECUTED"
    assert not_executed["policy_visible_hs_to_latency"]["status"] == "NOT_EXECUTED"
    assert (
        receipt["oracle_lineage"]["canonical_ledger_rescore"]["status"]
        == "FAIL"
    )
    assert receipt["oracle_lineage"]["canonical_ledger_consumed"] is True
    serialized = json.dumps(receipt, allow_nan=False)
    assert "batch parity is made" in receipt["interpretation"]
    assert "NaN" not in serialized and "Infinity" not in serialized


def test_row_selector_rejects_any_protected_or_reserve_trial() -> None:
    rows = subject._read_csv(subject.DEFAULT_V14_METRICS)
    injected = copy.deepcopy(rows[0])
    injected["trial_id"] = "05"
    with pytest.raises(subject.V17DevelopmentGateError, match="non-development"):
        subject._selected_rows([*rows, injected])


def test_profile_equivalence_rejects_material_or_geometry_drift() -> None:
    v13 = subject._strict_object(
        subject.REPO_ROOT
        / "validation/experimental_detector_profiles/"
        "two_sensor_v13_development_toe_down_p0p75mm_heel_x_p3p5mm.json",
        label="V13",
    )
    v17 = subject._strict_object(
        subject.REPO_ROOT
        / "validation/experimental_detector_profiles/"
        "two_sensor_v17_high_rate_v13_geometry.json",
        label="V17",
    )
    material_drift = copy.deepcopy(v17)
    material_drift["material"]["stiffness"] += 1.0
    with pytest.raises(subject.V17DevelopmentGateError, match="material"):
        subject._assert_profile_equivalence(material_drift, v13)
    geometry_drift = copy.deepcopy(v17)
    geometry_drift["spheres"][0]["radius"] += 1.0e-6
    with pytest.raises(subject.V17DevelopmentGateError, match="spheres"):
        subject._assert_profile_equivalence(geometry_drift, v13)


def test_nonfinite_required_metric_fails_closed() -> None:
    rows = subject._selected_rows(subject._read_csv(subject.DEFAULT_V14_METRICS))
    corrupted = copy.deepcopy(rows)
    corrupted[0]["precision"] = str(math.nan)
    with pytest.raises(subject.V17DevelopmentGateError, match="not finite"):
        subject.evaluate_sequential_rows(corrupted)


def test_evidence_hash_mismatch_fails_before_evaluation() -> None:
    with pytest.raises(subject.V17DevelopmentGateError, match="hash mismatch"):
        subject._require_sha256(
            subject.DEFAULT_V14_METRICS,
            "0" * 64,
            label="mutated expected evidence binding",
        )


def test_receipt_writer_is_atomic_strict_and_no_clobber(tmp_path: Path) -> None:
    destination = tmp_path / "receipt.json"
    receipt = _actual_receipt()
    subject.gate.write_json_no_clobber(destination, receipt)
    assert subject.gate.load_json_strict(destination) == receipt
    before = destination.read_bytes()
    with pytest.raises(subject.gate.NoClobberError):
        subject.gate.write_json_no_clobber(destination, receipt)
    assert destination.read_bytes() == before


def test_published_receipt_exactly_matches_a_fresh_evaluation() -> None:
    assert subject.DEFAULT_OUTPUT.is_file()
    assert subject.gate.load_json_strict(subject.DEFAULT_OUTPUT) == _actual_receipt()
