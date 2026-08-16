from __future__ import annotations

import ast
import json
from pathlib import Path

import numpy as np
import pytest

import h0_primary_split_v10s_fit as fit
import h0_primary_split_v10s_safe_dagger_contract as contract


def _write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _actor_state() -> dict[str, np.ndarray]:
    first_weight = np.zeros((3, 35), dtype=np.float32)
    first_bias = np.zeros(3, dtype=np.float32)
    second_weight = np.zeros((3, 3), dtype=np.float32)
    second_bias = np.zeros(3, dtype=np.float32)
    output_weight = np.zeros((4, 3), dtype=np.float32)
    output_bias = np.zeros(4, dtype=np.float32)
    return {
        "pi_encoder.0.weight": first_weight.copy(),
        "pi_encoder.0.bias": first_bias.copy(),
        "pi_encoder.2.weight": second_weight.copy(),
        "pi_encoder.2.bias": second_bias.copy(),
        "pi.0.0.weight": first_weight.copy(),
        "pi.0.0.bias": first_bias.copy(),
        "pi.0.2.weight": second_weight.copy(),
        "pi.0.2.bias": second_bias.copy(),
        "pi.1.weight": output_weight.copy(),
        "pi.1.bias": output_bias.copy(),
    }


def _candidate_state() -> dict[str, np.ndarray]:
    candidate = _actor_state()
    candidate["pi_encoder.0.weight"][0, 10] = 0.2
    candidate["pi.0.0.weight"][0, 10] = 0.2
    candidate["pi_encoder.2.bias"][0] = 0.1
    candidate["pi.0.2.bias"][0] = 0.1
    candidate["pi.1.weight"][0, 0] = 0.3
    candidate["pi.1.bias"][1] = -0.1
    return candidate


@pytest.fixture(scope="module")
def frozen_base() -> fit.FitCorpus:
    return fit.load_frozen_v8_corpus()


def test_frozen_base_is_exact_six_by_500_and_excludes_failed_v9(
    frozen_base: fit.FitCorpus,
) -> None:
    assert frozen_base.observations.shape == (3000, 35)
    assert frozen_base.observations.dtype == np.float32
    assert frozen_base.actions.shape == (3000, 2)
    assert frozen_base.actions.dtype == np.float32
    assert frozen_base.reset_mask.dtype == np.bool_
    assert np.count_nonzero(frozen_base.reset_mask) == 6
    assert frozen_base.audit["queried_equals_frozen_teacher_count"] == 3000
    assert frozen_base.audit["teacher_view_changes_only_10_24_count"] == 3000
    assert frozen_base.audit["failed_v9_rows_used"] == 0
    histogram = np.asarray(frozen_base.audit["changed_column_histogram"])
    assert np.any(histogram[10:25] > 0)
    assert np.count_nonzero(np.concatenate((histogram[:10], histogram[25:]))) == 0
    assert set(frozen_base.case_ids.astype(str)) == set(contract.FINAL_CASE_IDS)


def test_full_mean_audit_accepts_mean_change_and_frozen_logstd() -> None:
    audit = fit.full_mean_update_audit(_actor_state(), _candidate_state())
    assert audit["changes_confined_to_full_mean_network"] is True
    assert audit["hidden_mean_network_changed"] is True
    assert audit["mean_output_changed"] is True
    assert audit["logstd_parameter_rows_bit_exact"] is True
    assert audit["disabled_clock_columns_zero"] is True
    assert audit["non_actor_exact_or_absent"] is True


def test_full_mean_audit_rejects_logstd_change() -> None:
    source = _actor_state()
    candidate = _candidate_state()
    candidate["pi.1.weight"][2, 0] = 0.01
    audit = fit.full_mean_update_audit(source, candidate)
    assert audit["logstd_parameter_rows_bit_exact"] is False
    assert audit["changes_confined_to_full_mean_network"] is False


def test_full_mean_audit_rejects_reenabled_disabled_clock_columns() -> None:
    source = _actor_state()
    candidate = _candidate_state()
    candidate["pi_encoder.0.weight"][0, 0] = 0.01
    candidate["pi.0.0.weight"][0, 0] = 0.01
    audit = fit.full_mean_update_audit(source, candidate)
    assert audit["encoder_aliases_bit_exact"] is True
    assert audit["disabled_clock_columns_zero"] is False
    assert audit["changes_confined_to_full_mean_network"] is False


def _dagger_row(step: int, case_id: str) -> dict:
    student = np.linspace(-0.2, 0.2, 35, dtype=np.float32)
    teacher = student.copy()
    teacher[10:25] += np.float32(0.05)
    return {
        "step": step,
        "case_id": case_id,
        "v25_observation": student.tolist(),
        "counterfactual_teacher_observation": teacher.tolist(),
        "counterfactual_teacher_mean": [0.1, -0.2],
        "candidate_selected_before_teacher": True,
        "teacher_queried_on_same_state": True,
    }


def _artifact_record(root: Path, path: Path) -> dict:
    return {
        "path": path.relative_to(root).as_posix(),
        "sha256": fit.forensic.sha256_file(path),
        "size_bytes": path.stat().st_size,
    }


def _passing_collection_summary(case_id: str) -> dict:
    case = contract.canonical_collection_case(case_id, 1)
    zero_fields = (
        "action_clipped_values",
        "fallback_count",
        "timeout_count",
        "safety_stop_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
        "hard_invalid_count",
        "invalid_event_count",
        "nonfinite_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.COLLECTION_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "round_index": 1,
        "dagger_round": 1,
        "requested_alpha": case["requested_alpha"],
        "candidate_fit_stage": case["candidate_fit_stage"],
        "behavior": contract.COLLECTION_BEHAVIOR,
        "teacher_id": contract.TEACHER_ID,
        "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
        "sample_count": 500,
        "teacher_query_count": 500,
        "persisted_label_count": 500,
        "same_state_teacher_label_count": 500,
        "candidate_mean_query_count": 500,
        "candidate_selected_before_teacher_count": 500,
        "served_action_teacher_dependency_count": 500,
        "mean_blend_count": 500,
        "blend_before_noise_count": 500,
        "noise_before_blend_count": 0,
        "multiple_noise_application_count": 0,
        "safety_latch_activation_m": contract.SAFETY_LATCH_ACTIVATION_M,
        "safety_latch_release_m": contract.SAFETY_LATCH_RELEASE_M,
        "safety_latch_release_phase": contract.SAFETY_LATCH_RELEASE_PHASE,
        "safety_signal_lag_steps": 1,
        "safety_intervention_diagnostic_only": True,
        "safety_latch_activation_count": 0,
        "safety_latch_release_count": 0,
        "safety_intervention_count": 0,
        "safety_latch_rule_violation_count": 0,
        "alpha_mismatch_count": 0,
        "mean_blend_mismatch_count": 0,
        "noise_application_mismatch_count": 0,
        "physical_gate_bypass_count": 0,
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "sigma": case["sigma"],
        "steps": 500,
        "control_window_count": 5000,
        "raw_sensor_sample_count": 5000,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.02,
        **{name: 0 for name in zero_fields},
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "morphology_weight": 0.0,
        "random_noise_draw_count": 0,
        "single_noise_application_count": 500,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _dagger_receipt(
    root: Path,
    *,
    mutate_row=None,
) -> Path:
    case_id = "deterministic_offset_minus_0p20"
    case = contract.canonical_collection_case(case_id, 1)
    case_root = root / case["destination"]
    trace = case_root / "trace.json"
    rows = [
        _dagger_row(step, case_id)
        for step in range(1, contract.COLLECTION_SAMPLES_PER_CASE + 1)
    ]
    if mutate_row is not None:
        mutate_row(rows[0])
    _write_json(trace, rows)
    record = _artifact_record(root, trace)
    partial = case_root / "partial_summary.json"
    summary = case_root / "summary.json"
    _write_json(partial, {"steps": 500, "gate_evaluated": False})
    _write_json(summary, _passing_collection_summary(case_id))
    persisted = {
        "trace": record,
        "partial_summary": _artifact_record(root, partial),
        "summary": _artifact_record(root, summary),
    }
    gate = case_root / "gate.json"
    gate_payload = contract.collection_gate(
        _passing_collection_summary(case_id), round_index=1
    )
    assert gate_payload["passed"] is True
    gate_payload["persisted_before_gate"] = persisted
    _write_json(gate, gate_payload)
    gate_record = _artifact_record(root, gate)
    pipeline_claim = root / contract.PIPELINE_CLAIM_PATH
    worker_claim = root / contract.worker_claim_path(f"collect_r1__{case_id}")
    _write_json(pipeline_claim, {})
    _write_json(worker_claim, {})
    receipt = root / contract.stage_receipt_path(f"collect_r1__{case_id}")
    _write_json(
        receipt,
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.COLLECTION_PASS_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": f"collect_r1__{case_id}",
            "case_id": case_id,
            "round_index": 1,
            "dagger_round": 1,
            "candidate_fit_stage": case["candidate_fit_stage"],
            "requested_alpha": case["requested_alpha"],
            "sample_count": contract.COLLECTION_SAMPLES_PER_CASE,
            "same_state_teacher_label_count": contract.COLLECTION_SAMPLES_PER_CASE,
            "candidate_selected_before_teacher_count": contract.COLLECTION_SAMPLES_PER_CASE,
            "teacher_query_count": contract.COLLECTION_SAMPLES_PER_CASE,
            "persisted_label_count": contract.COLLECTION_SAMPLES_PER_CASE,
            "candidate_mean_query_count": contract.COLLECTION_SAMPLES_PER_CASE,
            "served_action_teacher_dependency_count": contract.COLLECTION_SAMPLES_PER_CASE,
            "artifacts": {**persisted, "gate": gate_record},
            "pipeline_claim": _artifact_record(root, pipeline_claim),
            "worker_claim": _artifact_record(root, worker_claim),
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        },
    )
    return receipt


def test_dagger_loader_accepts_only_same_state_block_replacement(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(fit, "REPO_ROOT", tmp_path)
    receipt = _dagger_receipt(tmp_path)
    tranche = fit.load_dagger_tranche(receipt)
    assert tranche.observations.shape == (500, 35)
    assert tranche.actions.shape == (500, 2)
    assert tranche.audit["same_state_dagger_sample_count"] == 500
    assert tranche.audit["failed_v9_rows_used"] == 0


def test_dagger_loader_fails_if_teacher_changes_outside_10_to_24(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(fit, "REPO_ROOT", tmp_path)

    def corrupt(row: dict) -> None:
        row["counterfactual_teacher_observation"][9] += 0.1

    receipt = _dagger_receipt(tmp_path, mutate_row=corrupt)
    with pytest.raises(fit.V10SFitError, match="outside 10:24"):
        fit.load_dagger_tranche(receipt)


def test_cumulative_count_contract_is_3000_then_plus_1000_per_round() -> None:
    assert [fit._expected_counts(stage) for stage in contract.FIT_STAGES] == [
        (3000, 6, 0),
        (4000, 8, 2),
        (5000, 10, 4),
        (6000, 12, 6),
    ]


def test_run_fit_stage_emits_a_contract_passing_gate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(fit, "REPO_ROOT", tmp_path)
    source = tmp_path / "source_h0"
    source.mkdir()
    (source / "module_state.pkl").write_bytes(b"source")
    output = tmp_path / "fit_p0"
    pipeline_claim = tmp_path / "pipeline_claim.json"
    worker_claim = tmp_path / "worker_claim.json"
    claim_base = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PIPELINE_CLAIM_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "retry_authorized": False,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    _write_json(pipeline_claim, claim_base)
    _write_json(
        worker_claim,
        {
            **claim_base,
            "status": contract.WORKER_CLAIM_STATUS,
            "stage_id": "fit_p0",
        },
    )

    count = contract.BASE_CORPUS_SAMPLE_COUNT
    observations = np.zeros((count, 35), dtype=np.float32)
    actions = np.zeros((count, 2), dtype=np.float32)
    reset = np.zeros(count, dtype=np.bool_)
    reset[::500] = True
    corpus = fit._make_corpus(
        observations=observations,
        actions=actions,
        reset_mask=reset,
        case_ids=np.repeat(np.asarray(contract.FINAL_CASE_IDS, dtype="U64"), 500),
        step_indices=np.tile(np.arange(1, 501, dtype=np.int64), 6),
        tranche_ids=np.repeat("v8r1p1_base", count),
        origins=[f"base:{index}" for index in range(count)],
        source_records={},
        audit={
            "base_sample_count": count,
            "queried_equals_frozen_teacher_count": count,
            "teacher_view_changes_only_10_24_count": count,
            "failed_v9_rows_used": 0,
            "dagger_sample_count": 0,
            "same_state_dagger_sample_count": 0,
            "dagger_receipt_count": 0,
            "dagger_identities": [],
            "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
            "teacher_evidence_passed": True,
        },
    )
    source_state = _actor_state()
    candidate_state = _candidate_state()

    class FakeImitation:
        @staticmethod
        def adapt_actor(checkpoint, dataset, destination, **kwargs):
            module = destination / "rl_module_target_adapted"
            module.mkdir()
            (module / "module_state.pkl").write_bytes(b"candidate")
            _write_json(destination / "adaptation_history.json", [])
            report = {
                "source_checkpoint": str(checkpoint),
                "selection_mode": "fixed_final_epoch",
                "epochs_run": 400,
                "best_epoch": 400,
                "training_samples": count,
                "validation_samples": 0,
                "hyperparameters": {
                    "seed": 123,
                    "batch_size": 128,
                    "learning_rate": 5.0e-5,
                    "anchor_weight": 1.0e-2,
                    "freeze_logstd_head": True,
                    "trainable_first_layer_features": None,
                },
                "save_reload": {"exact": True},
            }
            _write_json(destination / "adaptation_report.json", report)
            return report

    def module_state(path):
        return (
            source_state
            if Path(path).resolve() == source.resolve()
            else candidate_state
        )

    def logits(path, rows):
        result = np.zeros((len(rows), 4), dtype=np.float32)
        return result

    monkeypatch.setattr(fit, "load_fit_corpus", lambda *_args, **_kwargs: corpus)
    monkeypatch.setattr(fit, "_source_h0_path", lambda: source)
    monkeypatch.setattr(fit, "_imitation_engine", lambda: FakeImitation)
    monkeypatch.setattr(fit.warm_start, "load_module_state", module_state)
    monkeypatch.setattr(fit, "_module_logits", logits)
    receipt = fit.run_fit_stage(
        stage="p0",
        output_dir=output,
        pipeline_claim_path=pipeline_claim,
        worker_claim_path=worker_claim,
        enforce_canonical_destination=False,
    )
    assert receipt["passed"] is True
    gate = fit._mapping(output / "gate.json")
    assert gate["passed"] is True
    assert all(gate["checks"].values())
    summary = fit._mapping(output / "summary.json")
    assert summary["initial_checkpoint_id"] == contract.SOURCE_H0_ID
    assert summary["continued_from_previous_candidate"] is False
    assert summary["duplicate_sample_count"] == 0
    assert summary["source_h0_byte_exact"] is True
    assert summary["critic_byte_exact"] is True
    assert summary["logstd_byte_exact"] is True


def test_fit_call_hardcodes_the_fixed_v5_full_mean_design() -> None:
    source = Path(fit.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "adapt_actor"
    ]
    assert len(calls) == 1
    keyword = {item.arg: item.value for item in calls[0].keywords if item.arg}
    assert isinstance(keyword["freeze_logstd_head"], ast.Constant)
    assert keyword["freeze_logstd_head"].value is True
    assert isinstance(keyword["trainable_first_layer_features"], ast.Constant)
    assert keyword["trainable_first_layer_features"].value is None
    assert isinstance(keyword["selection_mode"], ast.Constant)
    assert keyword["selection_mode"].value == "fixed_final_epoch"


def test_no_clobber_rejects_an_existing_stage(tmp_path: Path) -> None:
    occupied = tmp_path / "occupied"
    occupied.mkdir()
    with pytest.raises(fit.V10SFitError, match="already exists/no-clobber"):
        fit._canonical_fit_destination("p0", occupied, enforce=False)


def test_canonical_claim_check_rejects_a_valid_claim_at_the_wrong_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(fit, "REPO_ROOT", tmp_path)
    lock = tmp_path / contract.LOCK_PATH
    _write_json(lock, {"status": contract.LOCK_STATUS})
    wrong = tmp_path / "wrong_pipeline_claim.json"
    _write_json(
        wrong,
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.PIPELINE_CLAIM_STATUS,
            "protocol_id": contract.PROTOCOL_ID,
            "pipeline_id": contract.PIPELINE_ID,
            "retry_authorized": False,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "execution_lock": _artifact_record(tmp_path, lock),
        },
    )
    with pytest.raises(fit.V10SFitError, match="claim path is not canonical"):
        fit._validate_claim(
            wrong,
            stage="p0",
            worker=False,
            enforce_canonical=True,
        )
