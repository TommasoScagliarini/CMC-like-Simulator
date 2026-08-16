from __future__ import annotations

import sys
from pathlib import Path


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r9_recovery_contract as contract  # noqa: E402


def test_contract_is_additive_and_preserves_the_frozen_training_shape() -> None:
    result = contract.contract_self_check()
    assert result["passed"] is True
    assert all(result["checks"].values())
    assert contract.ROOT != contract.v12r8.ROOT
    assert contract.PROTOCOL_ID != contract.v12r8.PROTOCOL_ID
    assert contract.PIPELINE_ID != contract.v12r8.PIPELINE_ID
    assert contract.IMPORTED_CASE_IDS == (
        "deterministic_offset_plus_0p20",
        "deterministic_offset_minus_0p20",
    )
    assert len(contract.NEW_COLLECTION_CASE_IDS) == 4
    assert contract.DEVELOPMENT_CASE_IDS == contract.v12r8.DEVELOPMENT_CASE_IDS
    assert contract.FIT["architecture"] == contract.v12r8.FIT["architecture"]
    assert contract.FIT["stratum_count"] == 13
    assert contract.FIT["actor_fit_count"] == 1
    assert contract.OFFLINE_THRESHOLDS == contract.v12r8.OFFLINE_THRESHOLDS
    assert contract.AUTHORITY["terminal_r8_retry_authorized"] is False
    assert contract.AUTHORITY["terminal_r8_resume_authorized"] is False


def test_stage_order_has_three_import_stages_then_four_new_probes() -> None:
    assert contract.STAGE_IDS[:3] == (
        "adjudicate_r8_terminal_and_minus_prefix",
        "import_r8_plus_labels",
        "label_r8_minus_prefix",
    )
    assert contract.STAGE_IDS[3:7] == tuple(
        f"collect_label__{case_id}" for case_id in contract.NEW_COLLECTION_CASE_IDS
    )
    assert all(
        f"collect_label__{case_id}" not in contract.STAGE_IDS
        for case_id in contract.IMPORTED_CASE_IDS
    )
    assert contract.STAGE_IDS[7:9] == (
        "fit_recovery_actor",
        "freeze_recovery_actor",
    )
    assert contract.STAGE_IDS[9:-1] == tuple(
        f"development__{case_id}" for case_id in contract.DEVELOPMENT_CASE_IDS
    )
    assert len(contract.STAGE_IDS) == len(set(contract.STAGE_IDS)) == 16
    assert contract.STAGE_IDS.count("freeze_recovery_actor") == 1


def test_mutable_outputs_are_r9_only_but_plus_label_path_is_direct_r8_reference() -> (
    None
):
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
    assert all(contract.v12r8.ROOT not in path.parents for path in output_paths)
    assert (
        contract.observer_label_path(contract.HISTORICAL_CASE_ID)
        == contract.R8_PLUS_LABELS_PATH
    )
    assert contract.v12r8.ROOT in contract.R8_PLUS_LABELS_PATH.parents
    assert (
        contract.observer_label_path(contract.IMPORTED_MINUS_CASE_ID)
        == contract.collection_case_root(contract.IMPORTED_MINUS_CASE_ID)
        / "observer_labels"
        / "labels.npz"
    )


def test_full_candidate_tree_is_the_runtime_and_fit_identity() -> None:
    full = contract.LOCKED_INPUTS["r6_candidate"]
    assert full == contract.FULL_R6_CANDIDATE_TREE
    assert set(full) == {"path", "tree_sha256", "file_count", "files"}
    assert full["file_count"] == len(full["files"]) == 5
    assert {row["path"] for row in full["files"]} == {
        "actor_feature_manifest.json",
        "class_and_ctor_args.pkl",
        "composite_build_manifest.json",
        "metadata.json",
        "module_state.pkl",
    }


def test_q3_candidate_interface_has_r9_identity() -> None:
    digest = "a" * 64
    assert contract.candidate_id(digest) == f"AB06_H0_V12R9_RECOVERY_W512:{digest}"
    assert contract.CANDIDATE_MODULE_PATH.name == "rl_module_recovery"
    assert contract.CANDIDATE_SELECTION_RULE == (
        "SOLE_R9_SINGLE_FIT_OUTPUT_IF_ALL_LOCKED_GATES_PASS"
    )
