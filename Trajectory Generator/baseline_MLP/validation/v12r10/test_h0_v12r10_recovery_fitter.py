from __future__ import annotations

import ast
import copy
import inspect
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r10_recovery_contract as contract  # noqa: E402
import h0_v12r10_recovery_fitter as fitter  # noqa: E402


def _initial_w1024_state() -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    import torch

    bundle = fitter.load_locked_r9_corpus()
    sources = fitter._load_source_modules()
    torch.manual_seed(fitter.SEED)
    residual = fitter._new_normalized_residual_model(
        sources["r9_state"], bundle.normalization
    )
    state = fitter._pack_w1024_state(
        residual=residual,
        r6_state=sources["r6_state"],
        normalization=bundle.normalization,
    )
    return state, sources["r6_state"]


def _passing_gate_summary() -> dict[str, object]:
    result_path = (
        LOCAL_ROOT / "diagnostics" / "results" / "w1024_gate_aligned_dry_fit.json"
    )
    primary = json.loads(result_path.read_text(encoding="utf-8"))["primary"]
    metrics = primary["metrics"]
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
        "candidate_state_digest": fitter.EXPECTED_FINAL_STATE_DIGEST,
        "candidate_predictions_sha256": fitter.EXPECTED_FINAL_PREDICTION_DIGEST,
        "uniform_state_digest": fitter.EXPECTED_UNIFORM_STATE_DIGEST,
        "uniform_predictions_sha256": fitter.EXPECTED_UNIFORM_PREDICTION_DIGEST,
        "uniform_lbfgs_closure_calls": fitter.EXPECTED_UNIFORM_CLOSURES,
        "uniform_terminal_loss": fitter.EXPECTED_UNIFORM_TERMINAL_LOSS,
        "gate_lbfgs_closure_calls": fitter.EXPECTED_GATE_CLOSURES,
        "gate_terminal_loss": fitter.EXPECTED_GATE_TERMINAL_LOSS,
        **metrics,
    }


def test_fixed_schedule_and_attestations_match_contract() -> None:
    assert fitter.UNIFORM_ADAMW_EPOCHS == 2500
    assert fitter.UNIFORM_LBFGS_MAX_ITER == 3000
    assert fitter.UNIFORM_LBFGS_MAX_EVAL == 4500
    assert fitter.EXPECTED_UNIFORM_CLOSURES == 3072
    assert fitter.GATE_ADAMW_EPOCHS == 1500
    assert fitter.GATE_LBFGS_MAX_ITER == 3000
    assert fitter.GATE_LBFGS_MAX_EVAL == 4500
    assert fitter.EXPECTED_GATE_CLOSURES == 3020
    assert fitter.RMSE_LIMIT == 0.006
    assert fitter.MAX_ABS_LIMIT == 0.060
    assert fitter.RESET_MAX_ABS_LIMIT == 0.003
    assert fitter.SMOOTH_MAX_TEMPERATURE == 0.05
    assert fitter.SAFETY_MARGIN_FRACTION == 0.90
    assert set(fitter.LOSS_COEFFICIENTS.values()) == {1.0}
    assert fitter.EXPECTED_FINAL_STATE_DIGEST == contract.EXPECTED_FINAL_STATE_DIGEST
    assert (
        fitter.EXPECTED_FINAL_PREDICTION_DIGEST
        == contract.EXPECTED_FINAL_PREDICTION_DIGEST
    )


def test_both_adamw_schedules_have_exact_boundaries() -> None:
    assert fitter.uniform_adamw_rate(1) == 3.0e-4
    assert fitter.uniform_adamw_rate(1000) == 3.0e-4
    assert fitter.uniform_adamw_rate(1001) == 1.0e-4
    assert fitter.uniform_adamw_rate(2000) == 1.0e-4
    assert fitter.uniform_adamw_rate(2001) == 3.0e-5
    assert fitter.uniform_adamw_rate(2500) == 3.0e-5
    assert fitter.gate_adamw_rate(1) == 3.0e-5
    assert fitter.gate_adamw_rate(500) == 3.0e-5
    assert fitter.gate_adamw_rate(501) == 1.0e-5
    assert fitter.gate_adamw_rate(1000) == 1.0e-5
    assert fitter.gate_adamw_rate(1001) == 3.0e-6
    assert fitter.gate_adamw_rate(1500) == 3.0e-6
    for function, invalid in (
        (fitter.uniform_adamw_rate, (0, 2501, True)),
        (fitter.gate_adamw_rate, (0, 1501, True)),
    ):
        for value in invalid:
            with pytest.raises(fitter.V12R10RecoveryFitError):
                function(value)


def test_locked_r9_terminal_corpus_and_r6_source_close_without_query() -> None:
    inputs = fitter.attest_locked_inputs()
    assert inputs["r9_initialization_only"] is True
    assert inputs["r9_promoted"] is False
    assert inputs["offline_h0_queries"] == 0
    assert inputs["r9_corpus"] == fitter.R9_CORPUS_RECORD
    assert inputs["r9_terminal_candidate"] == fitter.R9_CANDIDATE_TREE
    bundle = fitter.load_locked_r9_corpus()
    assert bundle.arrays["observations"].shape == (11875, 35)
    assert bundle.arrays["actions"].shape == (11875, 2)
    assert int(np.count_nonzero(bundle.arrays["reset_mask"])) == 26
    assert len(set(bundle.arrays["stratum_ids"].astype(str))) == 13
    assert bundle.weight_audit["reset_multiplier"] == 3.0
    assert bundle.weight_audit["total_mass"] == 6500.0


def test_reset3_weights_are_symmetric_and_reject_source_weight_mutation() -> None:
    names = fitter.expected_stratum_ids()
    strata = np.concatenate(
        [
            np.repeat(np.asarray([name], dtype="U96"), index + 2)
            for index, name in enumerate(names)
        ]
    )
    reset = np.zeros(len(strata), dtype=np.bool_)
    cursor = 0
    for index in range(len(names)):
        reset[cursor] = True
        cursor += index + 2
    original, _ = fitter.r9.compute_equal_stratum_weights(strata)
    weights, audit = fitter.compute_reset3_equal_stratum_weights(
        strata, reset, original
    )
    assert audit["stratum_count"] == 13
    assert math.isclose(math.fsum(weights), 6500.0, abs_tol=1.0e-8)
    for name in names:
        selected = strata == name
        assert math.isclose(math.fsum(weights[selected]), 500.0, abs_tol=1.0e-9)
        assert np.max(weights[selected]) == weights[selected][reset[selected]][0]
    mutated = original.copy()
    mutated[0] = np.nextafter(mutated[0], np.inf)
    with pytest.raises(fitter.V12R10RecoveryFitError, match="weights drifted"):
        fitter.compute_reset3_equal_stratum_weights(strata, reset, mutated)


def test_gate_groups_are_exactly_fifteen_and_symmetric() -> None:
    bundle = fitter.load_locked_r9_corpus()
    groups = fitter._gate_group_indices(bundle.arrays)
    expected = {
        "global",
        "r4_failure::deterministic_offset_plus_0p20",
        "observer_plus_late",
        *(f"base::{case_id}" for case_id in contract.COLLECTION_CASE_IDS),
        *(f"observer::{case_id}" for case_id in contract.COLLECTION_CASE_IDS),
    }
    assert len(groups) == 15
    assert set(groups) == expected
    assert all(len(indices) > 0 for indices in groups.values())


def test_initial_w1024_is_exact_r6_function_with_isolated_towers() -> None:
    state, r6_state = _initial_w1024_state()
    bundle = fitter.load_locked_r9_corpus()
    audit = fitter.validate_w1024_state(state)
    isolation = fitter.tower_isolation_audit(state, r6_state)
    candidate = fitter.r9.v11._state_logits(  # noqa: SLF001
        state, bundle.arrays["observations"][:256]
    )
    source = fitter.r9.v11._state_logits(  # noqa: SLF001
        r6_state, bundle.arrays["observations"][:256]
    )
    assert audit["hidden_dims"] == [1024, 1024]
    assert isolation["passed"] is True
    assert candidate.tobytes(order="C") == source.tobytes(order="C")


def test_tower_isolation_rejects_cross_block_and_r6_mutations() -> None:
    state, r6_state = _initial_w1024_state()
    cross = {name: value.copy() for name, value in state.items()}
    cross["pi_encoder.2.weight"][0, 512] = np.float32(1.0)
    cross["pi.0.2.weight"][0, 512] = np.float32(1.0)
    assert fitter.tower_isolation_audit(cross, r6_state)["passed"] is False

    tower = {name: value.copy() for name, value in state.items()}
    tower["pi_encoder.0.weight"][0, 2] += np.float32(1.0e-3)
    tower["pi.0.0.weight"][0, 2] += np.float32(1.0e-3)
    assert fitter.tower_isolation_audit(tower, r6_state)["passed"] is False


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("uniform_lbfgs_closure_calls", 3071),
        ("uniform_terminal_loss", 0.0),
        ("candidate_state_digest", "0" * 64),
        ("candidate_predictions_sha256", "f" * 64),
        ("gate_lbfgs_closure_calls", 3019),
        ("gate_terminal_loss", np.nextafter(0.7536344049605196, np.inf)),
    ],
)
def test_frozen_terminal_attestation_rejects_any_mutation(
    field: str, value: object
) -> None:
    payload = {
        "uniform_state_digest": fitter.EXPECTED_UNIFORM_STATE_DIGEST,
        "uniform_predictions_sha256": fitter.EXPECTED_UNIFORM_PREDICTION_DIGEST,
        "uniform_lbfgs_closure_calls": fitter.EXPECTED_UNIFORM_CLOSURES,
        "uniform_terminal_loss": fitter.EXPECTED_UNIFORM_TERMINAL_LOSS,
        "candidate_state_digest": fitter.EXPECTED_FINAL_STATE_DIGEST,
        "candidate_predictions_sha256": fitter.EXPECTED_FINAL_PREDICTION_DIGEST,
        "gate_lbfgs_closure_calls": fitter.EXPECTED_GATE_CLOSURES,
        "gate_terminal_loss": fitter.EXPECTED_GATE_TERMINAL_LOSS,
    }
    fitter._assert_expected_fit_attestation(payload)
    payload[field] = value
    with pytest.raises(fitter.V12R10RecoveryFitError, match="attestation drifted"):
        fitter._assert_expected_fit_attestation(payload)


def test_contract_gate_accepts_only_exact_locked_pass_projection() -> None:
    summary = _passing_gate_summary()
    assert contract.fit_gate(summary)["passed"] is True
    for field, value in (
        ("candidate_state_digest", "0" * 64),
        ("gate_lbfgs_closure_calls", 3019),
        ("tower_a_r6_byte_exact", False),
        ("warm_start_critic_preserved", False),
        ("no_legacy_shadow_runtime_dependency", False),
    ):
        mutated = copy.deepcopy(summary)
        mutated[field] = value
        assert contract.fit_gate(mutated)["passed"] is False


def test_actor_and_build_manifests_bind_standard_runtime_and_limitation(
    tmp_path: Path,
) -> None:
    state, r6_state = _initial_w1024_state()
    bundle = fitter.load_locked_r9_corpus()
    module_state = tmp_path / "module_state.pkl"
    module_state.write_bytes(b"synthetic")
    actor = fitter._candidate_actor_manifest(
        state=state,
        feature_names=bundle.arrays["actor_feature_names"].astype(str).tolist(),
        module_state=module_state,
    )
    build = fitter._candidate_build_manifest(
        state=state,
        actor_manifest=actor,
        bundle=bundle,
        isolation=fitter.tower_isolation_audit(state, r6_state),
    )
    assert actor["status"] == contract.ACTOR_FEATURE_MANIFEST_STATUS
    assert actor["fcnet_hiddens"] == [1024, 1024]
    assert actor["legacy_shadow_runtime_dependency"] is False
    assert build["r6_functional_predecessor"] is True
    assert build["r9_hidden_initialization_only"] is True
    assert build["r9_terminal_candidate_promoted"] is False
    assert build["no_legacy_shadow_runtime_dependency"] is True
    assert build["limitations"] == [fitter.TRANSITION_ALIAS_LIMITATION]


def test_real_standard_w1024_module_executes_packed_state() -> None:
    state, _r6_state = _initial_w1024_state()
    sources = fitter._load_source_modules()
    bundle = fitter.load_locked_r9_corpus()
    module = fitter._new_w1024_module(
        source_module=sources["r6_module"], state=state, inference_only=True
    )
    observations = bundle.arrays["observations"][:64]
    runtime = fitter._runtime_logits(module, observations)
    direct = fitter.r9.v11._state_logits(state, observations)  # noqa: SLF001
    assert type(module) is fitter.AsymmetricActorCriticTorchRLModule
    assert module.model_config["fcnet_hiddens"] == [1024, 1024]
    assert runtime.tobytes(order="C") == direct.tobytes(order="C")


def test_real_save_reload_and_warm_start_preserve_fresh_critic(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    state, r6_state = _initial_w1024_state()
    sources = fitter._load_source_modules()
    bundle = fitter.load_locked_r9_corpus()
    feature_names = bundle.arrays["actor_feature_names"].astype(str).tolist()
    monkeypatch.setattr(fitter, "REPO_ROOT", tmp_path)
    candidate = tmp_path / "candidate"
    _record, _actor, _build = fitter._save_candidate_exact(
        candidate_state=state,
        feature_names=feature_names,
        source_module=sources["r6_module"],
        destination=candidate,
        bundle=bundle,
        isolation=fitter.tower_isolation_audit(state, r6_state),
    )
    audit = fitter.runtime_and_warm_start_audit(
        candidate_path=candidate,
        intended_state=state,
        source_module=sources["r6_module"],
        observations=bundle.arrays["observations"][:128],
        feature_names=feature_names,
    )
    assert audit["passed"] is True
    assert audit["checks"]["checkpoint_state_byte_exact"] is True
    assert audit["checks"]["temporary_resave_state_byte_exact"] is True
    assert audit["checks"]["temporary_resave_logits_byte_exact"] is True
    assert audit["checks"]["warm_start_actor_exact"] is True
    assert audit["checks"]["warm_start_critic_preserved"] is True
    assert audit["optimizer_invocations"] == 0


def test_source_contains_two_fixed_optimizers_and_no_runtime_collection_surface() -> (
    None
):
    source = Path(fitter.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    lbfgs_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "LBFGS"
    ]
    assert len(lbfgs_calls) == 2
    assert "diagnostics" not in "\n".join(
        line
        for line in source.splitlines()
        if line.lstrip().startswith(("import ", "from "))
    )
    assert "make_cmc_env" not in source
    assert "env.reset(" not in source
    assert "env.step(" not in source
    assert "teacher_query" not in source
    assert '"critic_updates": 0' in source
    assert '"ppo_updates": 0' in source
    assert '"legacy_shadow_runtime_dependency": False' in source


def test_published_summary_and_receipt_schemas_are_explicit_and_exact() -> None:
    tree = ast.parse(Path(fitter.__file__).read_text(encoding="utf-8"))
    summary_keys: set[str] | None = None
    receipt_keys: set[str] | None = None
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "_fit_summary":
            returns = [
                child
                for child in ast.walk(node)
                if isinstance(child, ast.Return) and isinstance(child.value, ast.Dict)
            ]
            assert len(returns) == 1
            summary_keys = {
                ast.literal_eval(key)
                for key in returns[0].value.keys
                if key is not None
            }
        if isinstance(node, ast.FunctionDef) and node.name == "run_fit_stage":
            assignments = [
                child
                for child in ast.walk(node)
                if isinstance(child, ast.Assign)
                and any(
                    isinstance(target, ast.Name) and target.id == "receipt"
                    for target in child.targets
                )
                and isinstance(child.value, ast.Dict)
            ]
            assert len(assignments) == 1
            receipt_keys = {
                ast.literal_eval(key)
                for key in assignments[0].value.keys
                if key is not None
            }
    assert summary_keys == fitter.FIT_SUMMARY_FIELDS
    assert receipt_keys == fitter.FIT_RECEIPT_FIELDS


def test_semantic_verifier_source_has_no_optimizer_call() -> None:
    source = inspect.getsource(fitter.verify_fit_receipt)
    assert "torch.optim" not in source
    assert "fit_recovery_actor" not in source
    assert "_reproduce_uniform_terminal" not in source
    assert "runtime_and_warm_start_audit" in source
    assert "recompute_fit_metric_payload" in source


def test_run_fit_is_no_clobber_before_any_input_read(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(fitter, "REPO_ROOT", tmp_path)
    destination = fitter._resolve(contract.FIT_ROOT)
    destination.mkdir(parents=True)
    with pytest.raises(fitter.V12R10RecoveryFitError, match="exists/no-clobber"):
        fitter.run_fit_stage(
            pipeline_claim_path="missing_claim.json",
            worker_claim_path="missing_worker.json",
            protocol_freeze_path="missing_freeze.json",
            execution_lock_path="missing_lock.json",
        )


def test_import_does_not_create_canonical_fit() -> None:
    assert not os.path.lexists(fitter._resolve(contract.FIT_ROOT))
