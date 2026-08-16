from __future__ import annotations

import os

import pytest

import freeze_h0_primary_grf_split_v5_execution as freezer
import h0_primary_grf_split_v5_freeze_contract as contract


def test_build_payload_is_complete_and_does_not_publish_lock_or_run(
    monkeypatch,
) -> None:
    lock_existed = os.path.lexists(freezer.LOCK)
    run_existed = os.path.lexists(freezer.RUN_ROOT)
    assert not lock_existed
    assert not run_existed
    failure = freezer._validate_v4_preexecution_lineage()
    assert failure["status"] == "FAIL_H0_PRIMARY_SPLIT_V4_PREEXECUTION_INTEGRITY"
    assert failure["actor_update_candidates"] == 0
    if freezer.PREFLIGHT_RECEIPT.is_file():
        payload = freezer._build_lock_payload()
        assert set(payload) == freezer.runner.LOCK_TOP_LEVEL_KEYS
        assert payload["status"] == "H0_PRIMARY_GRF_SPLIT_V5_EXECUTION_FROZEN"
        assert payload["so_policy_id"] == contract.SO_POLICY_ID
        assert payload["fit"] == contract.FIT
        assert payload["trainable_scope"] == "full_mean_network"
        assert payload["inputs"]["preflight_receipt"]["path"] == (
            contract.PREFLIGHT_RECEIPT_RELATIVE
        )
        assert payload["inputs"]["v4_preexecution_failure"]["path"] == (
            contract.V4_PREEXECUTION_FAILURE_RELATIVE
        )
        assert payload["inputs"]["execution_ledger"]["path"].endswith(
            "2026-08-06_h0_primary_split_v3_semantic_replay/execution_ledger.json"
        )
        assert payload["v3_terminal_expectation"] == contract.V3_TERMINAL_EXPECTATION
        assert payload["v4_preexecution_expectation"] == (
            contract.V4_PREEXECUTION_EXPECTATION
        )
    else:
        with pytest.raises(freezer.V5FreezeError, match="preflight receipt"):
            freezer._build_lock_payload()
        original_record = freezer._record

        def record_with_unpublished_preflight(path):
            if path == freezer.PREFLIGHT_RECEIPT:
                return {
                    "path": contract.PREFLIGHT_RECEIPT_RELATIVE,
                    "sha256": "0" * 64,
                    "size_bytes": 0,
                }
            return original_record(path)

        monkeypatch.setattr(freezer, "_validate_preflight_receipt", lambda: {})
        monkeypatch.setattr(freezer, "_record", record_with_unpublished_preflight)
        payload = freezer._build_lock_payload()
        assert set(payload) == freezer.runner.LOCK_TOP_LEVEL_KEYS
        assert payload["schema_version"] == 5
        assert payload["inputs"]["v4_preexecution_failure"]["path"] == (
            contract.V4_PREEXECUTION_FAILURE_RELATIVE
        )
        assert payload["v4_preexecution_expectation"] == (
            contract.V4_PREEXECUTION_EXPECTATION
        )
    assert not os.path.lexists(freezer.LOCK)
    assert not os.path.lexists(freezer.RUN_ROOT)


def test_holdout_freezer_records_bytes_without_semantic_loader(monkeypatch) -> None:
    def forbidden(*_args, **_kwargs):
        raise AssertionError("seed-125 semantic loader was called")

    monkeypatch.setattr(freezer.v3, "historical_inputs", forbidden)
    records = freezer._opaque_holdout_byte_records()
    assert set(records) == {"trace", "summary"}
    for record in records.values():
        assert set(record) == {"path", "sha256", "size_bytes"}
        assert len(record["sha256"]) == 64
        assert record["size_bytes"] > 0


def test_freeze_refuses_existing_lock_before_any_other_action(
    tmp_path, monkeypatch
) -> None:
    occupied = tmp_path / "occupied-lock.json"
    occupied.write_text("{}", encoding="utf-8")
    monkeypatch.setattr(freezer, "LOCK", occupied)
    monkeypatch.setattr(freezer, "RUN_ROOT", tmp_path / "absent-run-root")
    with pytest.raises(freezer.V5FreezeError, match="refusing to clobber"):
        freezer.freeze()
