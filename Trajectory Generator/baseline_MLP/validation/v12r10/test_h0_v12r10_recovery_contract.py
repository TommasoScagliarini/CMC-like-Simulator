from __future__ import annotations

import copy
import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import h0_v12r10_recovery_contract as contract  # noqa: E402


def _passing_fit_summary() -> dict[str, object]:
    metric = {"rmse": 0.001, "max_abs_error": 0.002}
    return {
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "hidden_dims": [1024, 1024],
        "actor_feature_count": 35,
        "logstd_byte_exact": True,
        "disabled_clock_columns_bit_zero": True,
        "save_reload_exact": True,
        "runtime_save_reload_exact": True,
        "warm_start_actor_exact": True,
        "warm_start_critic_preserved": True,
        "tower_a_r6_byte_exact": True,
        "cross_blocks_positive_zero": True,
        "no_legacy_shadow_runtime_dependency": True,
        "candidate_state_digest": contract.EXPECTED_FINAL_STATE_DIGEST,
        "candidate_predictions_sha256": contract.EXPECTED_FINAL_PREDICTION_DIGEST,
        "uniform_state_digest": contract.EXPECTED_UNIFORM_STATE_DIGEST,
        "uniform_predictions_sha256": contract.EXPECTED_UNIFORM_PREDICTION_DIGEST,
        "uniform_lbfgs_closure_calls": contract.EXPECTED_UNIFORM_LBFGS_CLOSURES,
        "uniform_terminal_loss": contract.EXPECTED_UNIFORM_TERMINAL_LOSS,
        "gate_lbfgs_closure_calls": contract.EXPECTED_GATE_LBFGS_CLOSURES,
        "gate_terminal_loss": contract.EXPECTED_GATE_TERMINAL_LOSS,
        "global_metrics": copy.deepcopy(metric),
        "reset_max_abs_error": 0.001,
        "per_case_metrics": {
            case_id: copy.deepcopy(metric) for case_id in contract.COLLECTION_CASE_IDS
        },
        "observer_case_metrics": {
            case_id: copy.deepcopy(metric) for case_id in contract.COLLECTION_CASE_IDS
        },
        "r4_failed_plus_metrics": copy.deepcopy(metric),
        "observer_plus_late_metrics": copy.deepcopy(metric),
    }


def test_contract_is_schema_1300_import_only_and_self_consistent() -> None:
    result = contract.contract_self_check()
    assert result["passed"] is True
    assert all(result["checks"].values())
    assert contract.SCHEMA_VERSION == 1300
    assert contract.ROOT != contract.v12r9.ROOT
    assert contract.AUTHORITY["new_environment_collection_authorized"] is False
    assert contract.AUTHORITY["observer_only_teacher_queries_authorized"] is False
    assert contract.AUTHORITY["r9_terminal_retry_authorized"] is False
    assert contract.AUTHORITY["r9_terminal_resume_authorized"] is False
    assert contract.AUTHORITY["r9_candidate_promotion_authorized"] is False


def test_stage_order_is_exactly_ten_and_hardest_first_without_collection() -> None:
    expected_cases = (
        "deterministic_offset_plus_0p20",
        "deterministic_offset_minus_0p20",
        "deterministic_offset_nominal",
        "stochastic_nominal_seed_126",
        "stochastic_nominal_seed_127",
        "stochastic_nominal_seed_128",
    )
    assert contract.DEVELOPMENT_CASE_IDS == expected_cases
    assert contract.STAGE_IDS[:3] == (
        "attest_r9_terminal_imports",
        "fit_recovery_actor",
        "freeze_recovery_actor",
    )
    assert contract.STAGE_IDS[3:9] == tuple(
        f"development__{case_id}" for case_id in expected_cases
    )
    assert contract.STAGE_IDS[-1] == "finalize_development"
    assert len(contract.STAGE_IDS) == len(set(contract.STAGE_IDS)) == 10
    assert all(
        not stage.startswith(("collect", "label")) for stage in contract.STAGE_IDS
    )


def test_all_mutable_paths_are_r10_but_imports_remain_immutable_r9() -> None:
    outputs = (
        contract.PROTOCOL_FREEZE_PATH,
        contract.EXECUTION_LOCK_PATH,
        contract.RUN_ROOT,
        contract.CLAIM_PATH,
        contract.LEDGER_PATH,
        contract.R9_IMPORT_ATTESTATION_PATH,
        contract.FIT_ROOT,
        contract.CANDIDATE_MODULE_PATH,
        contract.CANDIDATE_FREEZE_PATH,
        contract.DEVELOPMENT_ROOT,
        contract.FINAL_DEVELOPMENT_PATH,
    )
    assert all(
        path == contract.ROOT or contract.ROOT in path.parents for path in outputs
    )
    assert all(contract.v12r9.ROOT not in path.parents for path in outputs)
    assert contract.v12r9.ROOT in contract.R9_CORPUS_PATH.parents
    assert contract.v12r9.ROOT in contract.R9_CANDIDATE_MODULE_PATH.parents
    assert contract.R9_TERMINAL_CANDIDATE_TREE["role"] == (
        "INITIALIZATION_ONLY_NOT_PROMOTED"
    )


def test_w1024_fit_and_unchanged_thresholds_are_fully_bound() -> None:
    assert contract.FIT["architecture"]["hidden_dims"] == [1024, 1024]
    assert contract.FIT["architecture"]["residual_actor"] is False
    assert contract.FIT["actor_fit_count"] == 1
    assert contract.FIT["offline_h0_teacher_query_count"] == 0
    assert contract.FIT["retry"] is False
    assert contract.FIT["sweep"] is False
    assert contract.OFFLINE_THRESHOLDS == contract.v12r9.OFFLINE_THRESHOLDS
    assert contract.FIT["expected_digests"] == contract.DIAGNOSTIC_ATTESTATION
    assert contract.EXPECTED_UNIFORM_LBFGS_CLOSURES == 3072
    assert contract.EXPECTED_GATE_LBFGS_CLOSURES == 3020


def test_fit_gate_requires_w1024_exact_digests_and_all_unchanged_metrics() -> None:
    summary = _passing_fit_summary()
    gate = contract.fit_gate(summary)
    assert gate["passed"] is True
    assert all(gate["checks"].values())

    for key, drift in (
        ("hidden_dims", [512, 512]),
        ("candidate_state_digest", "0" * 64),
        ("reset_max_abs_error", 0.0030000001),
        ("actor_fit_count", True),
    ):
        changed = copy.deepcopy(summary)
        changed[key] = drift
        assert contract.fit_gate(changed)["passed"] is False

    changed = copy.deepcopy(summary)
    changed["observer_case_metrics"]["stochastic_nominal_seed_128"]["max_abs_error"] = (
        0.0600001
    )
    assert contract.fit_gate(changed)["passed"] is False


def test_semantic_alias_is_a_fail_closed_physical_q3_risk_not_runtime_input() -> None:
    assert contract.LIMITATIONS["legacy_shadow_label_transition_alias"] is True
    assert contract.LIMITATIONS["offline_fit_proves_training_readiness"] is False
    assert contract.LIMITATIONS["runtime_legacy_shadow_dependency_authorized"] is False
    assert contract.AUTHORITY["qualification_execution_authorized"] is False
    assert contract.AUTHORITY["positive_morphology_authorized"] is False


def test_candidate_identity_is_w1024_and_digest_strict() -> None:
    digest = "a" * 64
    assert contract.candidate_id(digest) == ("AB06_H0_V12R10_RECOVERY_W1024:" + digest)
    for bad in ("A" * 64, "a" * 63, 0):
        try:
            contract.candidate_id(bad)  # type: ignore[arg-type]
        except ValueError:
            pass
        else:
            raise AssertionError(f"invalid candidate digest accepted: {bad!r}")
