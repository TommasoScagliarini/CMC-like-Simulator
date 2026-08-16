from __future__ import annotations

import copy
import sys
from pathlib import Path

import pytest


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r9_prefix_adjudicator as adjudicator  # noqa: E402
import h0_v12r9_recovery_contract as contract  # noqa: E402


def test_real_terminal_r8_minus_is_uniquely_adjudicated_without_rerun() -> None:
    result = adjudicator.load_and_adjudicate_r8_terminal()
    assert result["status"] == adjudicator.ADJUDICATION_STATUS
    assert result["passed"] is True
    assert result["offline_label_authorized"] is True
    assert result["autonomy_passed"] is False
    assert result["r8_artifacts_modified"] is False
    assert result["r8_probe_rerun"] is False
    assert result["r8_retry_authorized"] is False
    assert result["r8_resume_authorized"] is False
    assert result["next_stage"] == "import_r8_plus_labels"
    assert not hasattr(adjudicator, "load_r7_plus_evidence")
    assert not hasattr(adjudicator, "load_and_adjudicate_r7_plus")
    assert result["candidate_identity_adjudication"] == {
        "mismatch_fields": ["candidate_module"],
        "actual_extra_fields": ["files"],
        "locked_projection_exact": True,
        "full_tree_files_exact": True,
        "tree_sha256_exact": True,
        "file_count_exact": True,
        "semantic_identity_equivalent": True,
        "root_cause": adjudicator.ROOT_CAUSE,
    }
    assert result["counterfactual_probe_closure"] == {
        "passed": True,
        "identity_mismatch_fields": [],
        "gate_exact": True,
        "receipt_exact": True,
    }


def test_real_prefix_replay_journal_gate_and_v26_are_closed() -> None:
    result = adjudicator.load_and_adjudicate_r8_terminal()
    assert result["journal_attestation"]["step_count"] == 252
    assert result["trace_audit"]["passed"] is True
    assert result["replay_audit"] == {
        "passed": True,
        "step_count": 252,
        "boundary_count": 253,
        "event_count": 3,
        "actor_observations_trace_byte_exact": True,
        "previous_penetration_trace_byte_exact": True,
    }
    assert result["v26_audit"]["passed"] is True
    assert result["v26_audit"]["raw_sensor_sample_count"] == 2_520
    assert result["v26_audit"]["detector_anomaly_counters_zero"] is True
    assert result["v26_audit"]["runtime_anomaly_counters_zero"] is True


def test_locked_r8_digest_mutation_fails_before_admission(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    drifted = copy.deepcopy(contract.LOCKED_R8_EVIDENCE)
    drifted["minus_summary"]["sha256"] = "0" * 64
    monkeypatch.setattr(contract, "LOCKED_R8_EVIDENCE", drifted)
    with pytest.raises(adjudicator.V12R9AdjudicationError, match="drifted"):
        adjudicator.load_and_adjudicate_r8_terminal()


def test_full_candidate_lock_is_not_the_legacy_projection() -> None:
    full = contract.FULL_R6_CANDIDATE_TREE
    projection = {name: full[name] for name in ("path", "tree_sha256", "file_count")}
    assert projection == contract.v12r8.LOCKED_INPUTS["r6_candidate"]
    assert full == contract.LOCKED_INPUTS["r6_candidate"]
    assert full != projection
    assert set(full) == {"path", "tree_sha256", "file_count", "files"}
    assert len(full["files"]) == full["file_count"] == 5


def test_normalizer_never_overwrites_conflicting_top_level_projection() -> None:
    summary = adjudicator._strict_mapping(
        adjudicator.REPO_ROOT.joinpath(*contract.R8_MINUS_SUMMARY_PATH.parts)
    )
    summary["duplicate_event_count"] = 1
    with pytest.raises(adjudicator.V12R9AdjudicationError):
        adjudicator.normalize_v26_prefix_summary(
            summary,
            expected_steps=252,
        )


def test_plus_import_is_direct_and_byte_locked_without_copy() -> None:
    result = adjudicator.verify_r8_plus_label_import(semantic_verify=False)
    assert result["passed"] is True
    assert result["direct_immutable_reference"] is True
    assert result["labels_copied"] is False
    assert result["r8_artifacts_modified"] is False
    assert result["labelled_row_count"] == 179
    assert result["teacher_query_count"] == 0
