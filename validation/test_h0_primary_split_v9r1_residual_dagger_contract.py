from __future__ import annotations

import h0_primary_split_v9_residual_dagger_contract as v9
import h0_primary_split_v9r1_residual_dagger_contract as contract
import h0_primary_split_v9r1_teacher_compat_contract as teacher


def test_v9r1_is_procedural_alias_only() -> None:
    assert contract.SCHEMA_VERSION == 91
    assert "V9R1" in contract.PROTOCOL_ID
    assert contract.P0_FIT == v9.P0_FIT
    assert contract.P1_FIT == v9.P1_FIT
    assert contract.OFFLINE_THRESHOLDS == v9.OFFLINE_THRESHOLDS
    assert contract.RESIDUAL_LIMITS == v9.RESIDUAL_LIMITS
    assert contract.TEACHER_REPLAY_ROOT == v9.TEACHER_REPLAY_ROOT
    assert contract.AUTHORITY["scientific_target_changed_from_v9"] is False
    assert contract.AUTHORITY["fit_or_gate_changed_from_v9"] is False


def test_teacher_compat_exports_exact_35_84_layout() -> None:
    assert len(teacher.EXPECTED_ACTOR_FEATURE_NAMES) == 35
    assert len(teacher.EXPECTED_OBSERVATION_FEATURE_NAMES) == 84
    assert teacher.PROTOCOL_ID == (
        "AB06_H0_PRIMARY_SPLIT_V9_V26_EVENT_CAUSAL_RESIDUAL"
    )


def test_v9r1_paths_are_fresh_but_corpus_is_reused() -> None:
    assert "v9r1_v26_causal_residual" in contract.RUN_ROOT.as_posix()
    assert contract.RUN_ROOT != v9.RUN_ROOT
    assert contract.TEACHER_REPLAY_LEDGER_PATH == v9.TEACHER_REPLAY_LEDGER_PATH
    assert contract.PIPELINE_LEDGER_PATH != v9.PIPELINE_LEDGER_PATH
