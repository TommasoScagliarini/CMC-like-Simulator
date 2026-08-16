from __future__ import annotations

import sys
from pathlib import Path


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r8_recovery_contract as contract  # noqa: E402


def test_contract_is_additive_and_preserves_frozen_r7_training_shape() -> None:
    assert contract.contract_self_check()["passed"] is True
    assert contract.ROOT != contract.v12r7.ROOT
    assert contract.PROTOCOL_ID != contract.v12r7.PROTOCOL_ID
    assert contract.PIPELINE_ID != contract.v12r7.PIPELINE_ID
    assert contract.HISTORICAL_CASE_ID == "deterministic_offset_plus_0p20"
    assert len(contract.NEW_COLLECTION_CASE_IDS) == 5
    assert contract.DEVELOPMENT_CASE_IDS == contract.v12r7.DEVELOPMENT_CASE_IDS
    assert contract.FIT["architecture"] == contract.v12r7.FIT["architecture"]
    assert contract.FIT["stratum_count"] == 13
    assert contract.FIT["actor_fit_count"] == 1
    assert contract.OFFLINE_THRESHOLDS == contract.v12r7.OFFLINE_THRESHOLDS
    assert contract.AUTHORITY["historical_r7_prefix_rerun_authorized"] is False


def test_stage_order_adjudicates_and_labels_history_before_five_new_probes() -> None:
    assert contract.STAGE_IDS[:2] == (
        "adjudicate_r7_plus_prefix",
        "label_r7_plus_prefix",
    )
    assert contract.STAGE_IDS[2:7] == tuple(
        f"collect_label__{case_id}" for case_id in contract.NEW_COLLECTION_CASE_IDS
    )
    assert "collect_label__deterministic_offset_plus_0p20" not in contract.STAGE_IDS
    assert contract.STAGE_IDS[7:9] == (
        "fit_recovery_actor",
        "freeze_recovery_actor",
    )
    assert contract.STAGE_IDS[9:-1] == tuple(
        f"development__{case_id}" for case_id in contract.DEVELOPMENT_CASE_IDS
    )


def test_all_mutable_outputs_are_in_r8_namespace() -> None:
    output_paths = (
        contract.PROTOCOL_FREEZE_PATH,
        contract.EXECUTION_LOCK_PATH,
        contract.RUN_ROOT,
        contract.CLAIM_PATH,
        contract.LEDGER_PATH,
        contract.COLLECTION_ROOT,
        contract.FIT_ROOT,
        contract.CORPUS_PATH,
        contract.CANDIDATE_MODULE_PATH,
        contract.CANDIDATE_FREEZE_PATH,
        contract.DEVELOPMENT_ROOT,
        contract.FINAL_DEVELOPMENT_PATH,
    )
    assert all(
        path == contract.ROOT or contract.ROOT in path.parents for path in output_paths
    )
    assert all(contract.v12r7.ROOT not in path.parents for path in output_paths)


def test_q3_candidate_interface_has_r8_identity() -> None:
    digest = "a" * 64
    assert contract.candidate_id(digest) == f"AB06_H0_V12R8_RECOVERY_W512:{digest}"
    assert contract.CANDIDATE_MODULE_PATH.name == "rl_module_recovery"
    assert contract.CANDIDATE_SELECTION_RULE == (
        "SOLE_R8_SINGLE_FIT_OUTPUT_IF_ALL_LOCKED_GATES_PASS"
    )
