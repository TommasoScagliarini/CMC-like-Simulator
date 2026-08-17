from __future__ import annotations

import ast
import copy
import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r17_masked_teacher_fitter as fitter  # noqa: E402
import run_h0_v12r17_masked_teacher_fit as runner  # noqa: E402


@pytest.fixture(scope="module")
def corpus() -> fitter.LockedCorpus:
    return fitter.load_locked_corpus()


@pytest.fixture(scope="module")
def source() -> tuple[object, dict[str, np.ndarray]]:
    return fitter._load_source_module_and_state()  # noqa: SLF001


def _masked_h0_state(
    source_state: dict[str, np.ndarray], corpus: fitter.LockedCorpus
) -> tuple[dict[str, np.ndarray], fitter.MaskedNormalization]:
    normalization = fitter.build_masked_normalization(corpus.observations)
    model = fitter._new_normalized_model(  # noqa: SLF001
        source_state, normalization
    )
    state, _fold = fitter._fold_model_into_state(  # noqa: SLF001
        model, source_state, normalization
    )
    return state, normalization


def _passing_metrics() -> dict[str, object]:
    case_metric = {
        "rmse": 1.0e-4,
        "max_abs_error": 1.0e-3,
        "per_action_rmse": [1.0e-4, 1.0e-4],
    }
    return {
        "global": copy.deepcopy(case_metric),
        "per_case": {
            case_id: copy.deepcopy(case_metric) for case_id in fitter.CASE_IDS
        },
        "per_trajectory": {
            case_id: copy.deepcopy(case_metric) for case_id in fitter.CASE_IDS
        },
        "per_action_rmse": [1.0e-4, 1.0e-4],
        "worst_per_case_or_action_rmse": 1.0e-4,
        "worst_per_trajectory_or_action_rmse": 1.0e-4,
        "reset_max_abs_error": 5.0e-6,
        "transition_window_rows": 180,
        "transition_window_max_abs_error": 1.0e-3,
        "temporal_first_difference_pair_count": 2994,
        "temporal_first_difference_max_abs_error": 1.0e-3,
        "predicted_mean_max_abs": 0.90,
        "prediction_sha256": "0" * 64,
    }


def _passing_state_audit() -> dict[str, object]:
    return {
        "passed": True,
        "checks": {
            "masked_first_layer_columns_positive_zero": True,
            "logstd_source_byte_exact": True,
            "logstd_bias_sigma_0p005": True,
        },
    }


def test_protocol_is_exact_invariant18_w256_one_shot() -> None:
    description = fitter.describe_protocol()
    assert fitter.MASKED_COLUMNS == (0, 1, *range(10, 25))
    assert fitter.ACTIVE_COLUMNS == (*range(2, 10), *range(25, 35))
    assert set(fitter.MASKED_COLUMNS).isdisjoint(fitter.ACTIVE_COLUMNS)
    assert sorted((*fitter.MASKED_COLUMNS, *fitter.ACTIVE_COLUMNS)) == list(range(35))
    assert len(fitter.MASKED_COLUMNS) == 17
    assert len(fitter.ACTIVE_COLUMNS) == 18
    assert description["architecture"] == {
        "input_dim": 35,
        "hidden_dims": [256, 256],
        "logits_dim": 4,
        "activation": "tanh",
    }
    assert description["counts"] == {
        "actor_fits": 1,
        "actor_updates": 1,
        "offline_teacher_queries": 0,
        "environment_resets": 0,
        "environment_steps": 0,
        "policy_rollouts": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    assert description["fit"]["sweep"] is False
    assert description["fit"]["retry"] is False
    assert description["fit"]["replica"] is False
    assert description["production_fit_executed"] is False


def test_high_fidelity_gates_match_review_contract() -> None:
    assert fitter.GLOBAL_RMSE_LIMIT == 5.0e-3
    assert fitter.GLOBAL_MAX_ABS_LIMIT == 6.0e-2
    assert fitter.PER_CASE_ACTION_RMSE_LIMIT == 6.5e-3
    assert fitter.PER_TRAJECTORY_ACTION_RMSE_LIMIT == 8.0e-3
    assert fitter.RESET_MAX_ABS_LIMIT == 3.0e-3
    assert fitter.TRANSITION_MAX_ABS_LIMIT == 3.0e-2
    assert fitter.TEMPORAL_FIRST_DIFFERENCE_MAX_ABS_LIMIT == 8.0e-2
    assert fitter.PREDICTED_MEAN_ABS_LIMIT == 0.95
    assert fitter.TRANSITION_RADIUS_STEPS == 2
    assert fitter.ADAMW_EPOCHS == 3_000
    assert fitter.ADAMW_BOUNDARIES == (1_500, 2_500, 3_000)
    assert fitter.ADAMW_RATES == (3.0e-4, 1.0e-4, 3.0e-5)
    assert fitter.ADAMW_WEIGHT_DECAY == 1.0e-7
    assert fitter.LBFGS_MAX_ITER == 3_000
    assert fitter.LBFGS_MAX_EVAL == 4_500
    assert fitter.RESET_SAMPLE_WEIGHT == np.float32(100.0)
    assert fitter.NONRESET_SAMPLE_WEIGHT == np.float32(1.0)


def test_transitive_v8r1p1_hash_closure_and_exact_corpus(
    corpus: fitter.LockedCorpus,
) -> None:
    assert corpus.observations.shape == (3000, 35)
    assert corpus.observations.dtype == np.float32
    assert corpus.actions.shape == (3000, 2)
    assert corpus.actions.dtype == np.float32
    assert corpus.case_ids.dtype == np.dtype("<U40")
    assert corpus.trajectory_ids.dtype == np.dtype("<U128")
    assert corpus.step_indices.dtype == np.int64
    assert corpus.transition_mask.dtype == np.bool_
    assert int(np.count_nonzero(corpus.transition_mask)) == 180
    assert corpus.audit["array_hashes"] == fitter.EXPECTED_ARRAY_HASHES
    assert corpus.audit["binary_active_v26_rows"] == 3000
    assert corpus.audit["invariant_columns_exact_rows"] == 3000
    assert corpus.audit["unique_active_rows"] == 2996
    assert corpus.audit["duplicate_active_rows"] == 4
    assert corpus.audit["conflicting_duplicate_targets"] == 0
    assert corpus.source_records["source_h0"] == fitter.SOURCE_H0_TREE
    assert corpus.source_records["transitive_step_files_verified"] == 3000


def test_masked_normalization_uses_only_active_columns(
    corpus: fitter.LockedCorpus,
) -> None:
    normalization = fitter.build_masked_normalization(corpus.observations)
    normalized = fitter.normalized_masked_observations(
        corpus.observations, normalization
    )
    masked = np.asarray(fitter.MASKED_COLUMNS)
    active = np.asarray(fitter.ACTIVE_COLUMNS)
    assert normalization.mean[masked].tobytes() == bytes(4 * len(masked))
    assert np.array_equal(normalization.std[masked], np.ones(len(masked), np.float32))
    assert normalized[:, masked].tobytes() == bytes(normalized[:, masked].nbytes)
    assert np.max(np.abs(normalized[:, active].mean(axis=0))) < 2.0e-5
    assert normalization.record()["runtime_normalization_wrapper"] is False


def test_masked_h0_initialization_folds_and_ignores_all_masked_inputs(
    corpus: fitter.LockedCorpus,
    source: tuple[object, dict[str, np.ndarray]],
) -> None:
    _module, source_state = source
    state, normalization = _masked_h0_state(source_state, corpus)
    audit = fitter.validate_standard_state(state, source_state=source_state)
    assert audit["passed"] is True
    raw = corpus.observations[:128].copy()
    changed = raw.copy()
    changed[:, np.asarray(fitter.MASKED_COLUMNS)] = np.float32(1234.5)
    original_logits = fitter._state_logits(state, raw)  # noqa: SLF001
    changed_logits = fitter._state_logits(state, changed)  # noqa: SLF001
    assert original_logits.tobytes() == changed_logits.tobytes()
    model = fitter._new_normalized_model(  # noqa: SLF001
        source_state, normalization
    )
    normalized = fitter.normalized_masked_observations(raw, normalization)
    import torch

    with torch.no_grad():
        normalized_mean = model(torch.as_tensor(normalized)).numpy()
    assert np.max(np.abs(normalized_mean - original_logits[:, :2])) <= 2.0e-6
    assert np.array_equal(
        np.exp(original_logits[:, 2:]),
        np.full((len(raw), 2), fitter.TEACHER_SIGMA, dtype=np.float32),
    )


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("global.rmse", np.nextafter(fitter.GLOBAL_RMSE_LIMIT, np.inf)),
        ("global.max_abs_error", np.nextafter(fitter.GLOBAL_MAX_ABS_LIMIT, np.inf)),
        (
            "worst_per_case_or_action_rmse",
            np.nextafter(fitter.PER_CASE_ACTION_RMSE_LIMIT, np.inf),
        ),
        (
            "worst_per_trajectory_or_action_rmse",
            np.nextafter(fitter.PER_TRAJECTORY_ACTION_RMSE_LIMIT, np.inf),
        ),
        ("reset_max_abs_error", np.nextafter(fitter.RESET_MAX_ABS_LIMIT, np.inf)),
        (
            "transition_window_max_abs_error",
            np.nextafter(fitter.TRANSITION_MAX_ABS_LIMIT, np.inf),
        ),
        (
            "temporal_first_difference_max_abs_error",
            np.nextafter(fitter.TEMPORAL_FIRST_DIFFERENCE_MAX_ABS_LIMIT, np.inf),
        ),
        ("predicted_mean_max_abs", 0.95),
    ],
)
def test_fit_gate_rejects_each_independent_metric_blocker(
    field: str, value: float
) -> None:
    metrics = _passing_metrics()
    if field.startswith("global."):
        metrics["global"][field.split(".", 1)[1]] = value  # type: ignore[index]
    else:
        metrics[field] = value
    gate = fitter.fit_gate(
        metrics=metrics,
        state_audit=_passing_state_audit(),
        fold_audit={"passed": True},
    )
    assert gate["passed"] is False


def test_fit_gate_requires_mask_logstd_fold_and_accepts_only_full_pass() -> None:
    gate = fitter.fit_gate(
        metrics=_passing_metrics(),
        state_audit=_passing_state_audit(),
        fold_audit={"passed": True},
    )
    assert gate["passed"] is True
    for field in (
        "masked_first_layer_columns_positive_zero",
        "logstd_source_byte_exact",
        "logstd_bias_sigma_0p005",
    ):
        state = _passing_state_audit()
        state["checks"][field] = False  # type: ignore[index]
        assert (
            fitter.fit_gate(
                metrics=_passing_metrics(),
                state_audit=state,
                fold_audit={"passed": True},
            )["passed"]
            is False
        )
    assert (
        fitter.fit_gate(
            metrics=_passing_metrics(),
            state_audit=_passing_state_audit(),
            fold_audit={"passed": False},
        )["passed"]
        is False
    )


def test_real_save_reload_and_fresh_critic_transplant_without_fit(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    corpus: fitter.LockedCorpus,
    source: tuple[object, dict[str, np.ndarray]],
) -> None:
    source_module, source_state = source
    state, normalization = _masked_h0_state(source_state, corpus)
    monkeypatch.setattr(fitter, "REPO_ROOT", tmp_path)
    destination = tmp_path / "candidate"
    record, actor_manifest, build_manifest = fitter.save_candidate_and_manifests(
        source_module=source_module,
        candidate_state=state,
        destination=destination,
        corpus=corpus,
        normalization=normalization,
    )
    assert record["file_count"] == 5
    assert actor_manifest["masked_input_columns"] == list(fitter.MASKED_COLUMNS)
    assert build_manifest["input_contract"]["only_invariant_columns_active"] is True
    audit = fitter.runtime_and_transplant_audit(
        candidate_path=destination,
        intended_state=state,
        source_module=source_module,
        observations=corpus.observations[:128],
    )
    assert audit["passed"] is True
    assert audit["checks"]["warm_start_actor_exact"] is True
    assert audit["checks"]["warm_start_fresh_critic_preserved"] is True
    assert audit["checks"]["masked_features_rezeroed"] is True
    with pytest.raises(fitter.V12R17MaskedTeacherFitError, match="no-clobber"):
        fitter.save_candidate_and_manifests(
            source_module=source_module,
            candidate_state=state,
            destination=destination,
            corpus=corpus,
            normalization=normalization,
        )


def test_preflight_is_read_only_and_reports_representability_risk() -> None:
    assert not os.path.lexists(fitter.DEFAULT_OUTPUT)
    payload = fitter.preflight()
    assert payload["passed"] is True
    assert payload["production_fit_executed"] is False
    assert payload["candidate_created"] is False
    assert payload["checks"]["no_fit_executed"] is True
    evidence = payload["representability_evidence"]
    assert evidence["trainable_mean_parameter_count"] == 71170
    assert evidence["scalar_supervision_constraints"] == 6000
    assert evidence["conflicting_duplicate_targets"] == 0
    assert evidence["masked_h0_initial_metrics"]["global"]["rmse"] > 0.10
    assert not os.path.lexists(fitter.DEFAULT_OUTPUT)


def test_fixed_schedule_and_reset_weighted_objective_only() -> None:
    assert fitter.adamw_rate(1) == 3.0e-4
    assert fitter.adamw_rate(1_500) == 3.0e-4
    assert fitter.adamw_rate(1_501) == 1.0e-4
    assert fitter.adamw_rate(2_500) == 1.0e-4
    assert fitter.adamw_rate(2_501) == 3.0e-5
    assert fitter.adamw_rate(3_000) == 3.0e-5
    import torch

    prediction = torch.tensor([[1.0, 0.0], [0.0, 2.0]])
    target = torch.zeros_like(prediction)
    reset = torch.tensor([True, False])
    objective, mse, reset_mse = fitter._fit_objective(  # noqa: SLF001
        prediction, target, reset
    )
    assert objective.item() == pytest.approx(52.0 / 101.0)
    assert mse.item() == pytest.approx(1.25)
    assert reset_mse.item() == pytest.approx(0.5)


def test_temporal_and_trajectory_metrics_do_not_cross_trajectory_boundaries() -> None:
    targets = np.zeros((6, 2), dtype=np.float32)
    predictions = targets.copy()
    predictions[4, 0] = np.float32(0.001)
    metrics = fitter.prediction_metrics(
        predictions,
        targets,
        np.asarray(["same"] * 6),
        np.asarray(["trajectory_a"] * 3 + ["trajectory_b"] * 3),
        np.asarray([1, 2, 3, 1, 2, 3], dtype=np.int64),
        np.asarray([False, True, False, False, True, False]),
    )
    assert metrics["temporal_first_difference_pair_count"] == 4
    assert set(metrics["per_trajectory"]) == {"trajectory_a", "trajectory_b"}
    assert metrics["per_trajectory"]["trajectory_a"]["rmse"] == 0.0
    assert metrics["per_trajectory"]["trajectory_b"]["rmse"] > 0.0


def _synthetic_p1_from_p0(p0: fitter.LockedCorpus) -> fitter.LockedCorpus:
    observations = np.tile(p0.observations, (3, 1))
    observations[:, np.asarray(fitter.ACTIVE_COLUMNS)] += np.float32(10.0)
    trajectories = np.repeat(
        np.asarray(
            [
                fitter._p1_trajectory_id(alpha, case_id)  # noqa: SLF001
                for alpha in fitter.P1_ALPHAS
                for case_id in fitter.CASE_IDS
            ],
            dtype="U128",
        ),
        fitter.ROWS_PER_CASE,
    )
    return fitter.LockedCorpus(
        stage="p1",
        observations=np.ascontiguousarray(observations, dtype=np.float32),
        actions=np.ascontiguousarray(np.tile(p0.actions, (3, 1)), dtype=np.float32),
        case_ids=np.ascontiguousarray(np.tile(p0.case_ids, 3), dtype="U40"),
        trajectory_ids=np.ascontiguousarray(trajectories, dtype="U128"),
        step_indices=np.ascontiguousarray(np.tile(p0.step_indices, 3), dtype=np.int64),
        transition_mask=np.ascontiguousarray(
            np.tile(p0.transition_mask, 3), dtype=np.bool_
        ),
        source_records={"synthetic_test_only": True},
        audit={"rows": 9_000, "transition_window_rows": 540},
    )


def test_p1_journals_independently_reconstruct_supervision_and_transitions(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(fitter, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(fitter, "P1_ROOT", tmp_path / "p1")
    monkeypatch.setattr(fitter, "CASE_IDS", ("tiny_case",))
    monkeypatch.setattr(fitter, "P1_ALPHAS", (0.25,))
    monkeypatch.setattr(fitter, "ROWS_PER_CASE", 2)
    candidate_tree = {
        "path": "p0_candidate",
        "tree_sha256": "a" * 64,
        "file_count": 1,
        "files": [],
    }
    p0 = fitter.LockedCorpus(
        stage="p0",
        observations=np.zeros((2, 35), dtype=np.float32),
        actions=np.asarray([[0.1, -0.1], [0.2, -0.2]], dtype=np.float32),
        case_ids=np.asarray(["tiny_case", "tiny_case"]),
        trajectory_ids=np.asarray(["tiny_case", "tiny_case"]),
        step_indices=np.asarray([1, 2], dtype=np.int64),
        transition_mask=np.asarray([False, True]),
        source_records={},
        audit={},
    )
    trajectory_id = "alpha_0p25__tiny_case"
    root = tmp_path / "p1" / "collections" / "alpha_0p25" / "tiny_case"
    rows = []
    for step in (1, 2):
        row = {
            "step": step,
            "trajectory_id": trajectory_id,
            "case_id": "tiny_case",
            "requested_alpha": 0.25,
            "effective_alpha": 0.25,
            "v26_observation": [float(step)] * 35,
            "tape_target_mean": p0.actions[step - 1].tolist(),
            "target_provenance": {
                "source_case_id": "tiny_case",
                "source_step": step,
                "source_field": "frozen_teacher_mean",
                "teacher_model_query": False,
                "legacy_gait_shadow_query": False,
            },
            "support_distance_rms_z": 0.1,
            "support_within_p99": True,
            "support_intervened": False,
            "safety_intervened": False,
            "recovery": False,
            "fallback_reasons": [],
            "phase_fsm": {
                "accepted_transitions_this_step": [] if step == 1 else ["HS"]
            },
            "checks": {"semantic": True},
            "teacher_model_query_count": 0,
            "legacy_shadow_query_count": 0,
        }
        fitter._write_json_exclusive(  # noqa: SLF001
            root / "steps" / f"{step:06d}.json", row
        )
        rows.append(row)
    snapshot = {"same": True}
    fitter._write_json_exclusive(  # noqa: SLF001
        root / "run_start.json",
        {
            "source_candidate_id": "candidate-id",
            "source_p0_candidate": candidate_tree,
        },
    )
    fitter._write_json_exclusive(root / "trace.json", rows)  # noqa: SLF001
    fitter._write_json_exclusive(root / "partial_summary.json", {})  # noqa: SLF001
    fitter._write_json_exclusive(  # noqa: SLF001
        root / "summary.json",
        {
            "trajectory_id": trajectory_id,
            "case_id": "tiny_case",
            "requested_alpha": 0.25,
            "source_candidate_id": "candidate-id",
            "source_p0_candidate": candidate_tree,
            "source_snapshot_before": snapshot,
            "source_snapshot_after": snapshot,
            "steps": 2,
            "sample_count": 2,
            "persisted_label_count": 2,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "candidate_update_count_before": 0,
            "candidate_update_count_after": 0,
            "accepted_transition_steps": [2],
            "recovery_row_count": 0,
            "support_fallback_count": 0,
            "safety_fallback_count": 0,
        },
    )
    fitter._write_json_exclusive(  # noqa: SLF001
        root / "gate.json",
        {
            "passed": True,
            "status": "PASS_H0_V12R17_P1_CANDIDATE_EXPOSED_COLLECTION",
            "trajectory_id": trajectory_id,
            "checks": {"physical": True},
        },
    )
    artifacts = {
        "run_start": fitter._record(root / "run_start.json"),  # noqa: SLF001
        "steps": [
            fitter._record(root / "steps" / f"{step:06d}.json")  # noqa: SLF001
            for step in (1, 2)
        ],
        "trace": fitter._record(root / "trace.json"),  # noqa: SLF001
        "partial_summary": fitter._record(root / "partial_summary.json"),  # noqa: SLF001
        "summary": fitter._record(root / "summary.json"),  # noqa: SLF001
        "gate": fitter._record(root / "gate.json"),  # noqa: SLF001
    }
    fitter._write_json_exclusive(  # noqa: SLF001
        root / "receipt.json",
        {
            "passed": True,
            "status": "PASS_H0_V12R17_P1_CANDIDATE_EXPOSED_COLLECTION",
            "protocol_id": fitter.PROTOCOL_ID,
            "trajectory_id": trajectory_id,
            "case_id": "tiny_case",
            "requested_alpha": 0.25,
            "source_candidate_id": "candidate-id",
            "source_p0_candidate": candidate_tree,
            "row_count": 2,
            "accepted_transition_steps": [2],
            "artifacts": artifacts,
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        },
    )
    receipt_record = fitter._record(root / "receipt.json")  # noqa: SLF001
    rebuilt, provenance = fitter._reconstruct_p1_journals(  # noqa: SLF001
        case_receipt_records=[receipt_record],
        p0_terminal={
            "candidate_id": "candidate-id",
            "candidate_module": candidate_tree,
        },
        p0_corpus=p0,
    )
    assert rebuilt["observations"].shape == (2, 35)
    assert rebuilt["targets"].tobytes() == p0.actions.tobytes()
    assert rebuilt["transition_mask"].tolist() == [True, True]
    assert provenance[0]["accepted_event_steps"] == [2]
    (root / "steps" / "000002.json").write_text("{}\n", encoding="utf-8")
    with pytest.raises(fitter.V12R17MaskedTeacherFitError, match="artifact record"):
        fitter._reconstruct_p1_journals(  # noqa: SLF001
            case_receipt_records=[receipt_record],
            p0_terminal={
                "candidate_id": "candidate-id",
                "candidate_module": candidate_tree,
            },
            p0_corpus=p0,
        )


@pytest.mark.parametrize(
    "journal_key",
    [
        "observations",
        "targets",
        "requested_alpha",
        "effective_alpha",
        "recovery_mask",
        "support_distance",
        "transition_mask",
    ],
)
def test_p1_npz_rejects_each_reconstructed_journal_field_drift(
    journal_key: str,
) -> None:
    """Each named trace semantic must bind byte-exactly into the final NPZ."""

    arrays: dict[str, np.ndarray] = {
        "observations": np.zeros((2, 35), dtype=np.float32),
        "targets": np.zeros((2, 2), dtype=np.float32),
        "case_ids": np.asarray(["case", "case"], dtype="U40"),
        "trajectory_ids": np.asarray(["trajectory", "trajectory"], dtype="U80"),
        "step_indices": np.asarray([1, 2], dtype=np.int64),
        "requested_alpha": np.asarray([0.25, 0.25], dtype=np.float32),
        "effective_alpha": np.asarray([0.25, 0.25], dtype=np.float32),
        "recovery_mask": np.asarray([False, False], dtype=np.bool_),
        "support_distance": np.asarray([0.1, 0.1], dtype=np.float64),
        "transition_mask": np.asarray([False, True], dtype=np.bool_),
    }
    journals = {name: value.copy() for name, value in arrays.items()}
    if journals[journal_key].dtype == np.bool_:
        journals[journal_key][0] = ~journals[journal_key][0]
    elif journals[journal_key].dtype.kind in "f":
        journals[journal_key].flat[0] += 0.01
    else:
        pytest.fail(f"unhandled journal mutation dtype: {journals[journal_key].dtype}")
    with pytest.raises(
        fitter.V12R17MaskedTeacherFitError,
        match=f"immutable step journals:.*{journal_key}",
    ):
        fitter._assert_p1_npz_matches_journals(arrays, journals)  # noqa: SLF001


def test_p2_is_p0_then_p1_and_normalizes_over_all_12000_rows(
    monkeypatch: pytest.MonkeyPatch, corpus: fitter.LockedCorpus
) -> None:
    p1 = _synthetic_p1_from_p0(corpus)
    monkeypatch.setattr(fitter, "load_locked_corpus", lambda: corpus)
    monkeypatch.setattr(fitter, "_load_p1_candidate_exposed", lambda *, p0_corpus: p1)
    p2 = fitter.load_fit_corpus("p2")
    assert p2.stage == "p2"
    assert p2.observations.shape == (12_000, 35)
    assert p2.trajectory_ids.shape == (12_000,)
    assert len(set(p2.trajectory_ids.astype(str))) == 24
    assert int(np.count_nonzero(p2.step_indices == 1)) == 24
    assert p2.observations[:3_000].tobytes() == corpus.observations.tobytes()
    assert p2.observations[3_000:].tobytes() == p1.observations.tobytes()
    normalization = fitter.build_masked_normalization(p2.observations)
    active = np.asarray(fitter.ACTIVE_COLUMNS)
    expected = (
        p2.observations[:, active].mean(axis=0, dtype=np.float64).astype(np.float32)
    )
    assert normalization.source_rows == 12_000
    assert normalization.mean[active].tobytes() == expected.tobytes()


def test_governance_freeze_and_lock_are_exact_and_no_clobber(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    freeze = tmp_path / "protocol_freeze.json"
    lock = tmp_path / "execution_lock.json"
    monkeypatch.setattr(fitter, "PROTOCOL_FREEZE_PATH", freeze)
    monkeypatch.setattr(fitter, "EXECUTION_LOCK_PATH", lock)
    monkeypatch.setattr(fitter, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(
        fitter,
        "STAGE_OUTPUTS",
        {"p0": tmp_path / "p0_fit", "p2": tmp_path / "p2_fit"},
    )
    source_records = {
        name: {"path": f"{name}.py", "sha256": name * 16, "size_bytes": 1}
        for name in ("builder", "runner", "tests")
    }
    monkeypatch.setattr(fitter, "_governance_source_records", lambda: source_records)
    frozen = fitter.write_protocol_freeze()
    assert frozen["passed"] is True
    locked = fitter.write_execution_lock()
    assert locked["passed"] is True
    assert fitter.verify_execution_governance("p0")["execution_lock"]["path"]
    with pytest.raises(fitter.V12R17MaskedTeacherFitError, match="no-clobber"):
        fitter.write_protocol_freeze()
    monkeypatch.setattr(
        fitter,
        "_governance_source_records",
        lambda: {
            **source_records,
            "mutation": {"path": "x", "sha256": "0", "size_bytes": 0},
        },
    )
    with pytest.raises(fitter.V12R17MaskedTeacherFitError, match="freeze"):
        fitter.verify_execution_governance("p0")


def test_execute_is_blocked_before_loading_without_frozen_governance(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(
        fitter, "PROTOCOL_FREEZE_PATH", tmp_path / "missing_freeze.json"
    )
    monkeypatch.setattr(fitter, "EXECUTION_LOCK_PATH", tmp_path / "missing_lock.json")
    monkeypatch.setattr(fitter, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(
        fitter,
        "STAGE_OUTPUTS",
        {"p0": tmp_path / "p0_fit", "p2": tmp_path / "p2_fit"},
    )
    monkeypatch.setattr(
        fitter,
        "load_fit_corpus",
        lambda _stage: pytest.fail("corpus must not load before governance"),
    )
    with pytest.raises(fitter.V12R17MaskedTeacherFitError, match="strict JSON"):
        fitter.run_production_fit(
            stage="p0", destination=tmp_path / "candidate", enforce_canonical=False
        )


def test_cli_requires_explicit_one_shot_acknowledgement(
    capsys: pytest.CaptureFixture[str],
) -> None:
    assert runner.main(["--execute"]) == 2
    error = json.loads(capsys.readouterr().err)
    assert error["passed"] is False
    assert "acknowledgement" in error["error"]
    assert not os.path.lexists(fitter.DEFAULT_OUTPUT)


def test_occupied_destination_fails_before_loading_or_fitting(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    occupied = tmp_path / "occupied"
    occupied.mkdir()
    monkeypatch.setattr(fitter, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(
        fitter,
        "load_locked_corpus",
        lambda: pytest.fail("occupied destination must fail before corpus loading"),
    )
    with pytest.raises(fitter.V12R17MaskedTeacherFitError, match="no-clobber"):
        fitter.run_production_fit(destination=occupied, enforce_canonical=False)


def test_source_has_one_fixed_optimizer_path_and_no_runtime_surface() -> None:
    source = Path(fitter.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    adamw_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "AdamW"
    ]
    lbfgs_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "LBFGS"
    ]
    assert len(adamw_calls) == 1
    assert len(lbfgs_calls) == 1
    assert "make_cmc_env" not in source
    assert "env.reset(" not in source
    assert "env.step(" not in source
    assert "policy_rollout(" not in source
    assert "teacher_query(" not in source
    assert "fit_function" not in source
    assert "verify_execution_governance(stage)" in source
    assert '"sweep": False' in source
    assert '"retry": False' in source
    assert '"critic_updates": 0' in source
    assert '"ppo_updates": 0' in source


@pytest.mark.parametrize(
    "identity_key",
    ["case_ids", "trajectory_ids", "step_indices"],
)
def test_p1_npz_rejects_identity_journal_field_drift(identity_key: str) -> None:
    """Identity columns must also bind byte-exactly into the final NPZ."""

    arrays: dict[str, np.ndarray] = {
        "observations": np.zeros((2, 35), dtype=np.float32),
        "targets": np.zeros((2, 2), dtype=np.float32),
        "case_ids": np.asarray(["case", "case"], dtype="U40"),
        "trajectory_ids": np.asarray(["trajectory", "trajectory"], dtype="U80"),
        "step_indices": np.asarray([1, 2], dtype=np.int64),
        "requested_alpha": np.asarray([0.25, 0.25], dtype=np.float32),
        "effective_alpha": np.asarray([0.25, 0.25], dtype=np.float32),
        "recovery_mask": np.asarray([False, False], dtype=np.bool_),
        "support_distance": np.asarray([0.1, 0.1], dtype=np.float64),
        "transition_mask": np.asarray([False, True], dtype=np.bool_),
    }
    journals = {name: value.copy() for name, value in arrays.items()}
    if journals[identity_key].dtype.kind == "U":
        journals[identity_key][0] = "drifted"
    elif journals[identity_key].dtype.kind == "i":
        journals[identity_key][0] += 1
    else:
        pytest.fail(f"unhandled identity dtype: {journals[identity_key].dtype}")
    with pytest.raises(
        fitter.V12R17MaskedTeacherFitError,
        match=f"immutable step journals:.*{identity_key}",
    ):
        fitter._assert_p1_npz_matches_journals(arrays, journals)  # noqa: SLF001


def test_p1_real_builders_roundtrip_through_p2_loader(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """V12R8 lesson closed: the REAL P1 producers (corpus, manifest, ledger,
    receipt builders and forensic records) must satisfy the REAL P2 loader,
    end to end, without any verifier-shaped fixture in between."""

    from types import SimpleNamespace

    import h0_v12r17_tape_dagger as p1_contract  # noqa: PLC0415
    import run_h0_v12r17_tape_dagger as p1_runner  # noqa: PLC0415

    case_ids = tuple(f"case_{index}" for index in range(6))
    alphas = (0.25, 0.50, 0.75)
    rows_per_case = 2
    p0_rows = rows_per_case * len(case_ids)
    p1_rows = p0_rows * len(alphas)

    p1_root = tmp_path / "run" / "p1_candidate_exposed"
    freeze_path = tmp_path / "p1_freeze.json"
    lock_path = tmp_path / "p1_lock.json"
    freeze_path.write_text('{"stub": "freeze"}\n', encoding="utf-8")
    lock_path.write_text('{"stub": "lock"}\n', encoding="utf-8")
    candidate_dir = tmp_path / "p0_candidate"
    candidate_dir.mkdir()
    (candidate_dir / "module_state.pkl").write_bytes(b"stub-state")

    monkeypatch.setattr(p1_runner, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(p1_contract, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(p1_contract, "P1_ROOT", p1_root)
    monkeypatch.setattr(p1_contract, "P1_PROTOCOL_FREEZE", freeze_path)
    monkeypatch.setattr(p1_contract, "P1_EXECUTION_LOCK", lock_path)
    monkeypatch.setattr(
        p1_contract, "P1_CORPUS", p1_root / "corpus_candidate_exposed.npz"
    )
    monkeypatch.setattr(
        p1_contract, "P1_CORPUS_MANIFEST", p1_root / "corpus_manifest.json"
    )
    monkeypatch.setattr(p1_contract, "P1_LEDGER", p1_root / "pipeline_ledger.json")
    monkeypatch.setattr(p1_contract, "P1_RECEIPT", p1_root / "receipt.json")
    monkeypatch.setattr(p1_contract, "EXPECTED_STEPS", rows_per_case)
    monkeypatch.setattr(p1_contract, "EXPECTED_P1_ROWS", p1_rows)
    monkeypatch.setattr(p1_contract, "EXPECTED_BASE_ROWS", p0_rows)
    monkeypatch.setattr(p1_contract, "EXPECTED_P2_ROWS", p0_rows + p1_rows)

    monkeypatch.setattr(fitter, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(fitter, "CASE_IDS", case_ids)
    monkeypatch.setattr(fitter, "P1_ALPHAS", alphas)
    monkeypatch.setattr(fitter, "ROWS_PER_CASE", rows_per_case)
    monkeypatch.setattr(fitter, "P0_ROW_COUNT", p0_rows)
    monkeypatch.setattr(fitter, "P1_ROW_COUNT", p1_rows)
    monkeypatch.setattr(fitter, "P2_ROW_COUNT", p0_rows + p1_rows)

    rng = np.random.default_rng(20260815)
    p0_actions = np.ascontiguousarray(
        rng.uniform(-0.5, 0.5, size=(p0_rows, 2)), dtype=np.float32
    )
    candidate_tree = p1_runner._tree_record(candidate_dir)  # noqa: SLF001

    plan_rows = []
    for alpha in alphas:
        tag = fitter._p1_alpha_tag(alpha)  # noqa: SLF001
        for case_id in case_ids:
            plan_rows.append(
                {
                    "trajectory_id": fitter._p1_trajectory_id(  # noqa: SLF001
                        alpha, case_id
                    ),
                    "case_id": case_id,
                    "requested_alpha": alpha,
                    "destination": (
                        f"run/p1_candidate_exposed/collections/alpha_{tag}/{case_id}"
                    ),
                }
            )
    monkeypatch.setattr(p1_contract, "collection_plan", lambda: list(plan_rows))

    snapshot = {"sha": "same"}
    case_receipts = []
    # The LAST trajectory is a coherent physically-truncated prefix (1 row):
    # the V12R17 variable-length path must survive the real mixed geometry.
    truncated_trajectory = plan_rows[-1]["trajectory_id"]
    for plan_row in plan_rows:
        case_index = case_ids.index(plan_row["case_id"])
        case_steps = (
            1 if plan_row["trajectory_id"] == truncated_trajectory else rows_per_case
        )
        root = tmp_path / plan_row["destination"]
        rows = []
        for step in range(1, case_steps + 1):
            target = p0_actions[case_index * rows_per_case + step - 1]
            row = {
                "step": step,
                "trajectory_id": plan_row["trajectory_id"],
                "case_id": plan_row["case_id"],
                "requested_alpha": plan_row["requested_alpha"],
                "effective_alpha": plan_row["requested_alpha"],
                "v26_observation": [float(case_index * 10 + step)] * 35,
                "tape_target_mean": target.tolist(),
                "target_provenance": {
                    "source_case_id": plan_row["case_id"],
                    "source_step": step,
                    "source_field": "frozen_teacher_mean",
                    "teacher_model_query": False,
                    "legacy_gait_shadow_query": False,
                },
                "support_distance_rms_z": 0.1,
                "support_within_p99": True,
                "support_intervened": False,
                "safety_intervened": False,
                "recovery": False,
                "fallback_reasons": [],
                "phase_fsm": {
                    "accepted_transitions_this_step": [] if step == 1 else ["HS"]
                },
                "checks": {"semantic": True},
                "teacher_model_query_count": 0,
                "legacy_shadow_query_count": 0,
            }
            fitter._write_json_exclusive(  # noqa: SLF001
                root / "steps" / f"{step:06d}.json", row
            )
            rows.append(row)
        fitter._write_json_exclusive(  # noqa: SLF001
            root / "run_start.json",
            {
                "source_candidate_id": "candidate-id",
                "source_p0_candidate": candidate_tree,
            },
        )
        fitter._write_json_exclusive(root / "trace.json", rows)  # noqa: SLF001
        fitter._write_json_exclusive(  # noqa: SLF001
            root / "partial_summary.json", {}
        )
        fitter._write_json_exclusive(  # noqa: SLF001
            root / "summary.json",
            {
                "trajectory_id": plan_row["trajectory_id"],
                "case_id": plan_row["case_id"],
                "requested_alpha": plan_row["requested_alpha"],
                "source_candidate_id": "candidate-id",
                "source_p0_candidate": candidate_tree,
                "source_snapshot_before": snapshot,
                "source_snapshot_after": snapshot,
                "steps": case_steps,
                "sample_count": case_steps,
                "persisted_label_count": case_steps,
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
                "candidate_update_count_before": 0,
                "candidate_update_count_after": 0,
                "accepted_transition_steps": [2] if case_steps == 2 else [],
                "recovery_row_count": 0,
                "support_fallback_count": 0,
                "safety_fallback_count": 0,
            },
        )
        fitter._write_json_exclusive(  # noqa: SLF001
            root / "gate.json",
            {
                "passed": True,
                "status": p1_contract.COLLECTION_PASS_STATUS,
                "trajectory_id": plan_row["trajectory_id"],
                "checks": {"physical": True},
            },
        )
        artifacts = {
            "run_start": p1_runner._record(root / "run_start.json"),  # noqa: SLF001
            "steps": [
                p1_runner._record(  # noqa: SLF001
                    root / "steps" / f"{step:06d}.json"
                )
                for step in range(1, case_steps + 1)
            ],
            "trace": p1_runner._record(root / "trace.json"),  # noqa: SLF001
            "partial_summary": p1_runner._record(  # noqa: SLF001
                root / "partial_summary.json"
            ),
            "summary": p1_runner._record(root / "summary.json"),  # noqa: SLF001
            "gate": p1_runner._record(root / "gate.json"),  # noqa: SLF001
        }
        receipt_payload = {
            "passed": True,
            "status": p1_contract.COLLECTION_PASS_STATUS,
            "protocol_id": fitter.PROTOCOL_ID,
            "trajectory_id": plan_row["trajectory_id"],
            "case_id": plan_row["case_id"],
            "requested_alpha": plan_row["requested_alpha"],
            "source_candidate_id": "candidate-id",
            "source_p0_candidate": candidate_tree,
            "row_count": case_steps,
            "accepted_transition_steps": [2] if case_steps == 2 else [],
            "artifacts": artifacts,
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        }
        fitter._write_json_exclusive(  # noqa: SLF001
            root / "receipt.json", receipt_payload
        )
        case_receipts.append(
            {
                **receipt_payload,
                "receipt": p1_runner._record(root / "receipt.json"),  # noqa: SLF001
            }
        )

    prepared = SimpleNamespace(
        source_candidate_id="candidate-id",
        source_candidate_tree=candidate_tree,
        source_snapshot=snapshot,
        tape=SimpleNamespace(
            reference=lambda case_id, step: SimpleNamespace(
                target_mean=p0_actions[
                    case_ids.index(case_id) * rows_per_case + step - 1
                ]
            )
        ),
    )
    arrays, manifest = p1_runner.build_p1_corpus(case_receipts, prepared=prepared)
    corpus_path = p1_runner._write_npz_exclusive(  # noqa: SLF001
        p1_contract.P1_CORPUS, arrays
    )
    manifest = {
        **manifest,
        "corpus_artifact": p1_runner._record(corpus_path),  # noqa: SLF001
    }
    manifest_path = p1_runner.forensic.write_json_exclusive(
        p1_contract.P1_CORPUS_MANIFEST, manifest
    )
    receipt_records = [dict(item["receipt"]) for item in case_receipts]
    ledger = p1_runner.build_p1_ledger_payload(
        prepared=prepared,
        source_after=dict(snapshot),
        trajectory_ids=[row["trajectory_id"] for row in plan_rows],
        receipt_records=receipt_records,
        manifest=manifest,
        corpus_record=p1_runner._record(corpus_path),  # noqa: SLF001
        manifest_record=p1_runner._record(manifest_path),  # noqa: SLF001
    )
    ledger_path = p1_runner.forensic.write_json_exclusive(p1_contract.P1_LEDGER, ledger)
    receipt = p1_runner.build_p1_receipt_payload(
        prepared=prepared,
        receipt_records=receipt_records,
        manifest=manifest,
        corpus_record=p1_runner._record(corpus_path),  # noqa: SLF001
        manifest_record=p1_runner._record(manifest_path),  # noqa: SLF001
        ledger_record=p1_runner._record(ledger_path),  # noqa: SLF001
    )
    p1_runner.forensic.write_json_exclusive(p1_contract.P1_RECEIPT, receipt)

    monkeypatch.setattr(fitter, "P1_ROOT", p1_root)
    monkeypatch.setattr(fitter, "P1_CORPUS_PATH", p1_contract.P1_CORPUS)
    monkeypatch.setattr(
        fitter, "P1_CORPUS_MANIFEST_PATH", p1_contract.P1_CORPUS_MANIFEST
    )
    monkeypatch.setattr(fitter, "P1_RECEIPT_PATH", p1_contract.P1_RECEIPT)
    monkeypatch.setattr(fitter, "P1_LEDGER_PATH", p1_contract.P1_LEDGER)
    monkeypatch.setattr(fitter, "P1_PROTOCOL_FREEZE_PATH", freeze_path)
    monkeypatch.setattr(fitter, "P1_EXECUTION_LOCK_PATH", lock_path)
    monkeypatch.setattr(
        fitter,
        "_validated_p0_terminal",
        lambda: {"candidate_id": "candidate-id", "candidate_module": candidate_tree},
    )
    p0_corpus = fitter.LockedCorpus(
        stage="p0",
        observations=np.zeros((p0_rows, 35), dtype=np.float32),
        actions=p0_actions,
        case_ids=np.repeat(np.asarray(case_ids, dtype="U40"), rows_per_case),
        trajectory_ids=np.repeat(np.asarray(case_ids, dtype="U80"), rows_per_case),
        step_indices=np.tile(
            np.arange(1, rows_per_case + 1, dtype=np.int64), len(case_ids)
        ),
        transition_mask=np.zeros(p0_rows, dtype=np.bool_),
        source_records={},
        audit={},
    )

    loaded = fitter._load_p1_candidate_exposed(  # noqa: SLF001
        p0_corpus=p0_corpus
    )
    actual_rows = p1_rows - (rows_per_case - 1)
    expected_actions = np.concatenate(
        [
            p0_actions[
                case_ids.index(row["case_id"]) * rows_per_case : case_ids.index(
                    row["case_id"]
                )
                * rows_per_case
                + (1 if row["trajectory_id"] == truncated_trajectory else 2)
            ]
            for row in plan_rows
        ]
    )
    assert loaded.stage == "p1"
    assert loaded.observations.shape == (actual_rows, 35)
    assert (
        loaded.actions.tobytes()
        == np.ascontiguousarray(expected_actions, dtype=np.float32).tobytes()
    )
    assert loaded.audit["rows"] == actual_rows
    assert loaded.audit["per_trajectory_row_counts"][-1] == 1
    assert loaded.audit["trajectory_count"] == len(plan_rows)
    assert loaded.audit["reset_rows"] == len(plan_rows)
    assert loaded.source_records["p1_receipt"]["path"].endswith("receipt.json")

    lock_path.write_text('{"stub": "tampered"}\n', encoding="utf-8")
    with pytest.raises(fitter.V12R17MaskedTeacherFitError, match="artifact record"):
        fitter._load_p1_candidate_exposed(p0_corpus=p0_corpus)  # noqa: SLF001

    lock_path.write_text('{"stub": "lock"}\n', encoding="utf-8")
    assert (
        fitter._load_p1_candidate_exposed(p0_corpus=p0_corpus).stage  # noqa: SLF001
        == "p1"
    )
    freeze_path.write_text('{"stub": "tampered-freeze"}\n', encoding="utf-8")
    with pytest.raises(fitter.V12R17MaskedTeacherFitError, match="artifact record"):
        fitter._load_p1_candidate_exposed(p0_corpus=p0_corpus)  # noqa: SLF001
    freeze_path.write_text('{"stub": "freeze"}\n', encoding="utf-8")

    tampered_ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    tampered_ledger["protocol_freeze"] = {
        **tampered_ledger["protocol_freeze"],
        "sha256": "0" * 64,
    }
    ledger_path.unlink()
    new_ledger_path = p1_runner.forensic.write_json_exclusive(
        p1_contract.P1_LEDGER, tampered_ledger
    )
    realigned_receipt = json.loads(
        (p1_root / "receipt.json").read_text(encoding="utf-8")
    )
    realigned_receipt["artifacts"]["pipeline_ledger"] = p1_runner._record(  # noqa: SLF001
        new_ledger_path
    )
    (p1_root / "receipt.json").unlink()
    p1_runner.forensic.write_json_exclusive(p1_contract.P1_RECEIPT, realigned_receipt)
    with pytest.raises(fitter.V12R17MaskedTeacherFitError, match="artifact record"):
        fitter._load_p1_candidate_exposed(p0_corpus=p0_corpus)  # noqa: SLF001


def test_p1_naming_and_layout_lockstep_between_contract_and_fitter() -> None:
    """The P1 producer (contract) and the P2 verifier (fitter) derive names,
    row counts, and paths independently: bind them on the REAL values so a
    one-sided rename can never survive the suite."""

    import h0_v12r17_tape_dagger as p1_contract  # noqa: PLC0415

    assert tuple(p1_contract.CASE_IDS) == tuple(fitter.CASE_IDS)
    assert tuple(p1_contract.ROUND_ALPHAS) == tuple(fitter.P1_ALPHAS)
    assert p1_contract.EXPECTED_STEPS == fitter.ROWS_PER_CASE
    assert p1_contract.EXPECTED_P1_ROWS == fitter.P1_ROW_COUNT
    assert p1_contract.EXPECTED_BASE_ROWS == fitter.P0_ROW_COUNT
    assert p1_contract.EXPECTED_P2_ROWS == fitter.P2_ROW_COUNT
    for alpha in p1_contract.ROUND_ALPHAS:
        assert p1_contract.alpha_tag(alpha) == fitter._p1_alpha_tag(  # noqa: SLF001
            alpha
        )
        for case_id in p1_contract.CASE_IDS:
            assert p1_contract.trajectory_id(
                alpha, case_id
            ) == fitter._p1_trajectory_id(alpha, case_id)  # noqa: SLF001
    plan = p1_contract.collection_plan()
    assert [row["trajectory_id"] for row in plan] == [
        fitter._p1_trajectory_id(alpha, case_id)  # noqa: SLF001
        for alpha in fitter.P1_ALPHAS
        for case_id in fitter.CASE_IDS
    ]
    for row in plan:
        fitter_receipt_path = (
            fitter.P1_ROOT
            / "collections"
            / f"alpha_{fitter._p1_alpha_tag(row['requested_alpha'])}"  # noqa: SLF001
            / str(row["case_id"])
            / "receipt.json"
        )
        contract_receipt_path = (
            p1_contract.REPO_ROOT / str(row["destination"]) / "receipt.json"
        )
        assert contract_receipt_path == fitter_receipt_path
    assert p1_contract.P1_RECEIPT == fitter.P1_RECEIPT_PATH
    assert p1_contract.P1_LEDGER == fitter.P1_LEDGER_PATH
    assert p1_contract.P1_CORPUS == fitter.P1_CORPUS_PATH
    assert p1_contract.P1_CORPUS_MANIFEST == fitter.P1_CORPUS_MANIFEST_PATH
    assert p1_contract.P1_PROTOCOL_FREEZE == fitter.P1_PROTOCOL_FREEZE_PATH
    assert p1_contract.P1_EXECUTION_LOCK == fitter.P1_EXECUTION_LOCK_PATH
