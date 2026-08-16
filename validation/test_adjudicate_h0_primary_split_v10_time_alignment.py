from __future__ import annotations

import copy
import json

import pytest

import adjudicate_h0_primary_split_v10_time_alignment as adjudicator


def test_frozen_v10_evidence_supports_narrow_offline_adjudication() -> None:
    payload = adjudicator.build_adjudication()
    assert payload["passed"] is True
    assert all(payload["checks"].values())
    assert payload["original_protocol"]["preserved_as_fail"] is True
    assert payload["counterfactual_gate"]["not_written_back"] is True
    assert payload["rollout_rerun_count"] == 0
    assert payload["actor_updates"] == 0
    assert payload["protected_trials_opened"] == []


def test_row_audit_rejects_any_non_time_failure() -> None:
    rows = json.loads(adjudicator.TRACE.read_text(encoding="utf-8"))
    mutated = copy.deepcopy(rows)
    mutated[0]["checks"]["finite"] = False
    result = adjudicator._row_audit(mutated)
    assert result["non_time_failed_checks"] == {"finite": 1}


def test_row_audit_rejects_partial_trace() -> None:
    rows = json.loads(adjudicator.TRACE.read_text(encoding="utf-8"))
    with pytest.raises(adjudicator.V10TimeAdjudicationError, match="not 500"):
        adjudicator._row_audit(rows[:-1])


def test_write_is_exclusive(tmp_path, monkeypatch) -> None:
    destination = tmp_path / "receipt.json"
    monkeypatch.setattr(adjudicator, "RECEIPT", destination)
    first = adjudicator.write_receipt()
    assert destination.is_file()
    assert first["passed"] is True
    with pytest.raises(adjudicator.V10TimeAdjudicationError, match="already exists"):
        adjudicator.write_receipt()
