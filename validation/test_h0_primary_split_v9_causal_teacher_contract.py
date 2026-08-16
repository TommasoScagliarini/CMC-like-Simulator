from __future__ import annotations

import copy

import h0_primary_split_v9_causal_teacher_contract as contract


def _valid_summary() -> dict[str, object]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "target_id": contract.TARGET_ID,
        "case_id": contract.CASE_IDS[0],
        "sample_count": 500,
        "observation_shape": [35],
        "action_shape": [2],
        "dtype": "float32",
        "causal_column_mismatch_count": 0,
        "teacher_privileged_indices": [10, 11],
        "nonfinite_count": 0,
        "out_of_bounds_target_count": 0,
        "teacher_query_mismatch_count": 0,
        "target_equals_base_count": 10,
        "teacher_sigma": 0.005,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def test_v9_contract_is_fresh_and_event_causal() -> None:
    assert contract.SCHEMA_VERSION == 9
    assert "PRIMARY_SPLIT_V9_V26" in contract.PROTOCOL_ID
    assert contract.PRIVILEGED_INDICES == (10, 11)
    assert set(contract.CAUSAL_INDICES) == set(range(35)) - {10, 11}
    assert contract.AUTHORITY["legacy_event_or_fsm_label_authorized"] is False
    assert contract.EVENT_CONTRACT_ID == (
        "binary_point_v25+heel_qualified_fsm_v2"
    )
    assert len(contract.CASE_IDS) == 6


def test_case_gate_passes_only_complete_non_degenerate_relabel() -> None:
    summary = _valid_summary()
    assert contract.case_gate(summary)["passed"] is True

    for key, bad in (
        ("causal_column_mismatch_count", 1),
        ("target_equals_base_count", 500),
        ("teacher_privileged_indices", [10, 11, 12]),
        ("protected_trials_opened", ["05"]),
    ):
        mutated = copy.deepcopy(summary)
        mutated[key] = bad
        assert contract.case_gate(mutated)["passed"] is False


def test_case_copy_and_paths_are_no_clobber_lineage() -> None:
    case = contract.canonical_case(contract.CASE_IDS[0])
    case["case_id"] = "changed"
    assert contract.canonical_case(contract.CASE_IDS[0])["case_id"] != "changed"
    assert "v9_v26_causal_residual" in contract.RUN_ROOT.as_posix()
    assert contract.SOURCE_ROOT != contract.RELABEL_ROOT
