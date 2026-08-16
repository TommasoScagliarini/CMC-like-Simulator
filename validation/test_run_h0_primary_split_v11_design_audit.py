from __future__ import annotations

from pathlib import Path

import pytest

import h0_forensic_rollout as forensic
import h0_primary_split_v11_weighted_full_mean_contract as contract
import run_h0_primary_split_v11_design_audit as audit


def test_success_writes_only_one_strict_no_clobber_receipt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    destination = tmp_path / "receipt.json"
    gate = {"passed": True, "checks": {"synthetic": True}}
    payload = {
        "status": contract.DESIGN_AUDIT_PASS_STATUS,
        "gate": gate,
        "artifacts_written": [contract.DESIGN_AUDIT_RECEIPT_PATH.as_posix()],
        "candidate_checkpoint_paths": [],
    }
    calls = 0

    def fake_run():
        nonlocal calls
        calls += 1
        return payload

    monkeypatch.setattr(audit.fit_engine, "run_design_audit_in_memory", fake_run)
    monkeypatch.setattr(audit.contract, "design_audit_gate", lambda value: gate)
    result = audit.execute_design_audit(
        output_path=destination,
        enforce_canonical_destination=False,
    )
    assert result == payload
    assert calls == 1
    assert [path.name for path in tmp_path.iterdir()] == ["receipt.json"]
    assert forensic.strict_json_load(destination) == payload
    with pytest.raises(audit.V11DesignAuditError, match="already exists/no-clobber"):
        audit.execute_design_audit(
            output_path=destination,
            enforce_canonical_destination=False,
        )
    assert calls == 1


def test_numerical_failure_consumes_stage_with_failure_receipt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    destination = tmp_path / "failure.json"
    calls = 0

    def fail():
        nonlocal calls
        calls += 1
        raise ArithmeticError("synthetic non-finite loss")

    monkeypatch.setattr(audit.fit_engine, "run_design_audit_in_memory", fail)
    with pytest.raises(audit.V11DesignAuditConsumedFailure, match="is consumed"):
        audit.execute_design_audit(
            output_path=destination,
            enforce_canonical_destination=False,
        )
    receipt = forensic.strict_json_load(destination)
    assert receipt["status"] == contract.DESIGN_AUDIT_FAIL_STATUS
    assert receipt["passed"] is False
    assert receipt["audit_consumed"] is True
    assert receipt["no_candidate_checkpoint"] is True
    assert receipt["candidate_checkpoints_persisted"] == 0
    assert receipt["actor_fit_execution_attempted"] is True
    assert receipt["actor_update_attempted_or_unknown"] is True
    assert receipt["confirmed_actor_updates"] == 0
    assert "actor_updates" not in receipt
    assert receipt["critic_updates"] == 0
    assert receipt["ppo_updates"] == 0
    assert receipt["protected_trials_opened"] == []
    assert receipt["reserve_trials_opened"] == []
    assert receipt["retry_authorized"] is False
    assert receipt["next_stage"] == "STOP_V11_DESIGN_AUDIT_CONSUMED_NO_RETRY"
    assert receipt["error"] == {
        "type": "ArithmeticError",
        "message": "synthetic non-finite loss",
    }
    with pytest.raises(audit.V11DesignAuditError, match="already exists/no-clobber"):
        audit.execute_design_audit(
            output_path=destination,
            enforce_canonical_destination=False,
        )
    assert calls == 1


def test_noncanonical_destination_is_rejected_before_fit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        audit.fit_engine,
        "run_design_audit_in_memory",
        lambda: pytest.fail("fit must not start"),
    )
    with pytest.raises(audit.V11DesignAuditError, match="non-canonical"):
        audit.execute_design_audit(output_path=tmp_path / "wrong.json")


def test_main_requires_explicit_execute() -> None:
    with pytest.raises(audit.V11DesignAuditError, match="explicit --execute"):
        audit.main([])


def test_main_returns_nonzero_for_consumed_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        audit,
        "execute_design_audit",
        lambda: (_ for _ in ()).throw(
            audit.V11DesignAuditConsumedFailure("consumed")
        ),
    )
    assert audit.main(["--execute"]) == 1
