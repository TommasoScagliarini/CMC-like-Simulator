from __future__ import annotations

import copy
import sys
from pathlib import Path


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r8_prefix_adjudicator as adjudicator  # noqa: E402
import h0_v12r8_recovery_contract as contract  # noqa: E402


def _fresh_evidence() -> dict[str, object]:
    return adjudicator.load_r7_plus_evidence()


def test_real_terminal_r7_prefix_is_labelable_without_rerun_or_rewrite() -> None:
    result = adjudicator.adjudicate_r7_plus_prefix(**_fresh_evidence())
    assert result["status"] == adjudicator.ADJUDICATION_PASS_STATUS
    assert result["passed"] is True
    assert result["offline_label_authorized"] is True
    assert result["autonomy_passed"] is False
    assert result["r7_artifacts_modified"] is False
    assert result["r7_probe_rerun"] is False
    assert result["normalization"]["projected_fields"] == [
        "duplicate_event_count",
        "out_of_order_event_count",
        "left_non_v26_source_count",
        "target_contract_id",
    ]
    assert result["normalization"]["nested_gate_passed_preserved"] is False
    assert result["counterfactual_r7_gate"]["passed"] is True
    assert result["counterfactual_r7_gate"]["autonomy_passed"] is False


def test_target_contract_is_derived_only_after_all_attestation_prechecks() -> None:
    evidence = _fresh_evidence()
    evidence["artifact_records"] = copy.deepcopy(evidence["artifact_records"])
    evidence["artifact_records"]["plus_summary"]["sha256"] = "0" * 64
    result = adjudicator.adjudicate_r7_plus_prefix(**evidence)
    assert result["passed"] is False
    assert result["checks"]["artifact__records_exact"] is False
    assert result["target_contract_derivation"] is None
    assert result["normalization"] is None


def test_reproduction_drift_fails_closed() -> None:
    evidence = _fresh_evidence()
    evidence["reproduction_audit"] = copy.deepcopy(evidence["reproduction_audit"])
    evidence["reproduction_audit"]["shared_runtime_fields_exact"] = False
    result = adjudicator.adjudicate_r7_plus_prefix(**evidence)
    assert result["passed"] is False
    assert result["checks"]["reproduction__source_and_fields_exact"] is False
    assert result["offline_label_authorized"] is False


def test_event_contract_drift_fails_closed() -> None:
    evidence = _fresh_evidence()
    evidence["summary"] = copy.deepcopy(evidence["summary"])
    evidence["summary"]["event_contract_id"] = "drifted_event_contract"
    result = adjudicator.adjudicate_r7_plus_prefix(**evidence)
    assert result["passed"] is False
    assert result["normalization"] is None
    assert result["offline_label_authorized"] is False


def test_nested_prefix_integrity_drift_fails_closed() -> None:
    evidence = _fresh_evidence()
    evidence["summary"] = copy.deepcopy(evidence["summary"])
    evidence["summary"]["binary_event_prefix_integrity"]["sample_count"] = 1780
    result = adjudicator.adjudicate_r7_plus_prefix(**evidence)
    assert result["passed"] is False
    assert result["normalization"] is None
    assert result["offline_label_authorized"] is False


def test_normalizer_never_overwrites_an_existing_top_level_projection() -> None:
    evidence = _fresh_evidence()
    summary = copy.deepcopy(evidence["summary"])
    summary["duplicate_event_count"] = 1
    try:
        adjudicator.normalize_v26_prefix_summary(
            summary,
            frozen_target_contract_id=contract.TARGET_CONTRACT_ID,
            attested_contract_source=contract.R7_CONTRACT_SOURCE_RECORD,
        )
    except adjudicator.V12R8AdjudicationError:
        pass
    else:  # pragma: no cover - defensive assertion.
        raise AssertionError("normalizer accepted a conflicting projection")
