from __future__ import annotations

import ast
import json
from pathlib import Path

import numpy as np
import pytest

import h0_primary_split_v11_weighted_fit as fit
import h0_primary_split_v11_weighted_full_mean_contract as contract


def _write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _source_state() -> dict[str, np.ndarray]:
    rng = np.random.default_rng(42)
    first_weight = rng.normal(0.0, 0.01, (256, 35)).astype(np.float32)
    first_weight[:, :2] = np.float32(0.0)
    first_bias = rng.normal(0.0, 0.01, 256).astype(np.float32)
    second_weight = rng.normal(0.0, 0.01, (256, 256)).astype(np.float32)
    second_bias = rng.normal(0.0, 0.01, 256).astype(np.float32)
    output_weight = rng.normal(0.0, 0.01, (4, 256)).astype(np.float32)
    output_weight[2:] = np.float32(0.0)
    output_bias = rng.normal(0.0, 0.01, 4).astype(np.float32)
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


def _synthetic_corpus() -> fit.FitCorpus:
    rows = 3000
    observations = np.zeros((rows, 35), dtype=np.float32)
    observations[:, 1] = np.float32(1.0)
    actions = np.zeros((rows, 2), dtype=np.float32)
    reset = np.zeros(rows, dtype=np.bool_)
    reset[::500] = True
    case_ids = np.repeat(np.asarray(contract.FINAL_CASE_IDS, dtype="U64"), 500)
    return fit.FitCorpus(
        observations=observations,
        actions=actions,
        reset_mask=reset,
        actor_feature_names=np.asarray([f"feature_{index}" for index in range(35)]),
        case_ids=case_ids,
        step_indices=np.tile(np.arange(1, 501, dtype=np.int64), 6),
        tranche_ids=np.repeat("v8r1p1_base", rows),
        origins=np.asarray([f"base:{index}" for index in range(rows)]),
        source_records={},
        audit={
            "base_sample_count": rows,
            "dagger_sample_count": 0,
            "same_state_dagger_sample_count": 0,
            "failed_v9_rows_used": 0,
            "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
            "teacher_evidence_passed": True,
        },
    )


@pytest.fixture(scope="module")
def real_base() -> fit.FitCorpus:
    return fit.load_frozen_v8_corpus()


def test_fit_spec_is_exact_contract_design() -> None:
    assert fit._fit_spec() == contract.FIT


def test_adamw_schedule_boundaries_have_no_early_transition() -> None:
    assert fit.adamw_learning_rate(1) == 3.0e-4
    assert fit.adamw_learning_rate(1500) == 3.0e-4
    assert fit.adamw_learning_rate(1501) == 1.0e-4
    assert fit.adamw_learning_rate(2500) == 1.0e-4
    assert fit.adamw_learning_rate(2501) == 3.0e-5
    assert fit.adamw_learning_rate(3000) == 3.0e-5
    for invalid in (0, 3001, 1.0, True):
        with pytest.raises(fit.V11WeightedFitError):
            fit.adamw_learning_rate(invalid)  # type: ignore[arg-type]


def test_normalization_and_reset_weights_are_exact() -> None:
    corpus = _synthetic_corpus()
    normalization = fit.frozen_base_normalization(corpus.observations)
    normalized = fit.normalized_observations(corpus.observations, normalization)
    weights = fit.reset_sample_weights(corpus.reset_mask, rows=3000)
    assert normalized[:, :2].tobytes() == bytes(normalized[:, :2].nbytes)
    assert normalization.mean[0].tobytes() == np.float32(0.0).tobytes()
    assert normalization.mean[1].tobytes() == np.float32(1.0).tobytes()
    assert normalization.std[0].tobytes() == np.float32(1.0e-4).tobytes()
    assert normalization.std[1].tobytes() == np.float32(1.0e-4).tobytes()
    assert np.count_nonzero(weights == np.float32(100.0)) == 6
    assert np.count_nonzero(weights == np.float32(1.0)) == 2994


def test_fold_equivalence_is_explicit_and_fail_closed() -> None:
    reference = np.zeros((4, 2), dtype=np.float32)
    within = reference.copy()
    within[0, 0] = np.float32(5.0e-7)
    audit = fit.fold_equivalence_audit(reference, within)
    assert audit["fold_equivalence_passed"] is True
    outside = reference.copy()
    outside[0, 0] = np.float32(2.0e-6)
    with pytest.raises(fit.V11WeightedFitError, match="fold equivalence failed"):
        fit.fold_equivalence_audit(reference, outside)


def test_folded_state_preserves_clocks_logstd_and_nonactor() -> None:
    source = _source_state()
    source["vf.weight"] = np.arange(8, dtype=np.float32).reshape(2, 4)
    corpus = _synthetic_corpus()
    normalization = fit.frozen_base_normalization(corpus.observations)
    model = fit._new_normalized_model(source, normalization)
    with pytest.importorskip("torch").no_grad():
        model[2].bias.add_(0.001)
        model[4].bias.add_(0.001)
    candidate, fold_audit = fit._fold_normalization_into_state(
        model, source, normalization
    )
    audit = fit.full_mean_update_audit(source, candidate)
    assert fold_audit["runtime_normalization_wrapper"] is False
    assert audit["passed"] is True
    assert audit["disabled_clock_columns_bit_zero"] is True
    assert audit["logstd_parameter_rows_bit_exact"] is True
    assert audit["critic_byte_exact"] is True
    assert audit["critic_present"] is True
    assert audit["critic_parameter_count"] == 8
    assert candidate["vf.weight"].tobytes() == source["vf.weight"].tobytes()


def test_real_source_scope_is_reported_as_actor_only_not_compared_critic() -> None:
    source = _source_state()
    candidate = {key: value.copy() for key, value in source.items()}
    candidate["pi_encoder.2.bias"][0] += np.float32(0.001)
    candidate["pi.0.2.bias"][0] += np.float32(0.001)
    candidate["pi.1.bias"][0] += np.float32(0.001)
    audit = fit.full_mean_update_audit(source, candidate)
    assert audit["passed"] is True
    assert audit["source_checkpoint_scope"] == "actor_only_rl_module"
    assert audit["critic_present"] is False
    assert type(audit["critic_parameter_count"]) is int
    assert audit["critic_parameter_count"] == 0


def test_every_public_contract_reference_exists() -> None:
    tree = ast.parse(Path(fit.__file__).read_text(encoding="utf-8"))
    names = {
        node.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == "contract"
    }
    missing = sorted(name for name in names if not hasattr(contract, name))
    assert missing == []


def test_run_fit_stage_emits_summary_accepted_by_real_contract(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(fit, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(fit.v10s_fit, "REPO_ROOT", tmp_path)
    corpus = _synthetic_corpus()
    normalization = fit.frozen_base_normalization(corpus.observations)

    source = tmp_path / contract.SOURCE_H0_MODULE_PATH
    source.mkdir(parents=True)
    for name in ("class_and_ctor_args.pkl", "metadata.json", "module_state.pkl"):
        (source / name).write_bytes(f"source:{name}".encode())
    for relative in (
        contract.V10S_TERMINAL_LEDGER_PATH,
        contract.V10S_P0_GATE_PATH,
        contract.V10S_P0_SUMMARY_PATH,
    ):
        _write_json(tmp_path / relative, {})
    design_receipt = tmp_path / contract.DESIGN_AUDIT_RECEIPT_PATH
    _write_json(design_receipt, {"passed": True})
    pipeline_claim = tmp_path / "pipeline_claim.json"
    worker_claim = tmp_path / "worker_claim.json"
    _write_json(pipeline_claim, {"execution_token_sha256": "a" * 64})
    _write_json(worker_claim, {"execution_token_sha256": "a" * 64})

    candidate_state = _source_state()
    candidate_state["pi_encoder.2.bias"][0] += np.float32(0.001)
    candidate_state["pi.0.2.bias"][0] += np.float32(0.001)
    candidate_state["pi.1.bias"][0] += np.float32(0.001)
    preservation = fit.full_mean_update_audit(_source_state(), candidate_state)
    preservation["logstd_outputs_bit_exact"] = True
    preservation["passed"] = True
    result = fit.InMemoryFitResult(
        candidate_state=candidate_state,
        predictions=np.zeros((3000, 2), dtype=np.float32),
        metrics={
            "rmse": 0.001,
            "max_abs_error": 0.002,
            "reset_max_abs_error": 0.001,
        },
        normalization=normalization,
        normalization_audit={
            "normalization_folded_into_first_layer": True,
            "runtime_normalization_wrapper": False,
            "fold_equivalence_passed": True,
        },
        preservation_audit=preservation,
        history=({"optimizer": "adamw", "index": 3000, "loss": 0.0},),
        optimizer_audit={"deterministic_algorithms_enabled": True},
    )

    monkeypatch.setattr(fit, "_validate_claim", lambda *args, **kwargs: {"execution_token_sha256": "a" * 64})
    monkeypatch.setattr(fit, "load_fit_corpus", lambda *args, **kwargs: corpus)
    monkeypatch.setattr(fit, "load_frozen_v8_corpus", lambda: corpus)
    monkeypatch.setattr(fit, "_load_source_module_and_state", lambda: (object(), _source_state()))
    monkeypatch.setattr(fit, "fit_weighted_full_mean_in_memory", lambda **kwargs: result)
    monkeypatch.setattr(
        fit,
        "_validate_design_audit_for_p0",
        lambda **kwargs: {
            "passed": True,
            "receipt": fit._record(design_receipt),
            "metric_match": {"passed": True},
        },
    )
    monkeypatch.setattr(
        contract,
        "v10s_terminal_failure_gate",
        lambda *args: {"passed": True, "checks": {"terminal": True}},
    )

    def fake_save(*, source_module, candidate_state, destination):
        destination.mkdir()
        for name in ("class_and_ctor_args.pkl", "metadata.json", "module_state.pkl"):
            (destination / name).write_bytes(f"candidate:{name}".encode())
        return {
            "exact": True,
            "clock_columns_bit_zero": True,
            "logstd_parameter_rows_bit_exact": True,
            "critic_byte_exact": True,
            "nonactor_byte_exact": True,
        }

    monkeypatch.setattr(fit, "_save_candidate_exact", fake_save)
    output = tmp_path / "fit_p0"
    receipt = fit.run_fit_stage(
        stage="p0",
        output_dir=output,
        pipeline_claim_path=pipeline_claim,
        worker_claim_path=worker_claim,
        enforce_canonical_destination=False,
    )
    assert receipt["passed"] is True
    summary = fit._mapping(output / "summary.json")
    gate = contract.fit_gate(summary, stage="p0")
    assert gate["passed"] is True, [
        name for name, value in gate["checks"].items() if value is not True
    ]


def test_design_audit_payload_passes_real_contract_without_writing_checkpoint(
    real_base: fit.FitCorpus, monkeypatch: pytest.MonkeyPatch
) -> None:
    normalization = fit.frozen_base_normalization(real_base.observations)
    metrics = {
        "rmse": 0.005,
        "max_abs_error": 0.050,
        "reset_max_abs_error": 0.001,
    }
    result = fit.InMemoryFitResult(
        candidate_state={},
        predictions=np.zeros((3000, 2), dtype=np.float32),
        metrics=metrics,
        normalization=normalization,
        normalization_audit={
            "normalization_folded_into_first_layer": True,
            "runtime_normalization_wrapper": False,
            "fold_equivalence_passed": True,
        },
        preservation_audit={
            "source_checkpoint_scope": "actor_only_rl_module",
            "critic_present": False,
            "critic_parameter_count": 0,
            "critic_byte_exact": True,
            "logstd_parameter_rows_bit_exact": True,
            "logstd_outputs_bit_exact": True,
            "disabled_clock_columns_bit_zero": True,
        },
        history=(),
        optimizer_audit={"deterministic_algorithms_enabled": True},
    )
    monkeypatch.setattr(fit, "load_frozen_v8_corpus", lambda: real_base)
    monkeypatch.setattr(fit, "_load_source_module_and_state", lambda: (object(), {}))
    monkeypatch.setattr(
        fit, "fit_weighted_full_mean_in_memory", lambda **kwargs: result
    )
    receipt_path = fit.REPO_ROOT / contract.DESIGN_AUDIT_RECEIPT_PATH
    assert not receipt_path.exists()
    payload = fit.run_design_audit_in_memory()
    assert payload["observed_metrics"] == metrics
    assert payload["p0_reproduction_reference_metrics"] == metrics
    assert payload["actor_fit_executions"] == 1
    assert payload["actor_updates"] == 1
    assert payload["candidate_checkpoints_persisted"] == 0
    assert contract.design_audit_gate(payload)["passed"] is True
    assert contract.design_audit_current_binding_gate(
        payload, fit.current_design_audit_bindings()
    )["passed"] is True
    assert not receipt_path.exists()


def test_frozen_validator_reconstructs_exact_v10s_p0_arrays(
    real_base: fit.FitCorpus,
) -> None:
    base = real_base
    path = fit.REPO_ROOT / contract.V10S_P0_CORPUS_PATH
    with np.load(path, allow_pickle=False) as frozen:
        assert frozen["observations"].tobytes() == base.observations.tobytes()
        assert frozen["actions"].tobytes() == base.actions.tobytes()
        assert frozen["reset_mask"].tobytes() == base.reset_mask.tobytes()
    assert fit.array_sha256(base.observations) == (
        "d367a4697606f7c5d721823c973aabbbc86fb314e9b74e1529afc22e45a4d9ad"
    )
