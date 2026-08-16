"""Adversarial mutations for the V12R8 pre-freeze semantic closure."""

from __future__ import annotations

import copy
import ast
import inspect
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r8_recovery_contract as contract  # noqa: E402
import h0_v12r8_recovery_fitter as fitter  # noqa: E402
import h0_v12r8_recovery_probe as probe  # noqa: E402


def _metric_bundle() -> fitter.RecoveryCorpusBundle:
    cases = tuple(contract.COLLECTION_CASE_IDS)
    rows = 13
    observations = np.zeros((rows, contract.EXPECTED_ACTOR_FEATURES), dtype=np.float32)
    observations[:, 1] = np.float32(1.0)
    observations[:, 2] = np.linspace(-0.2, 0.2, rows, dtype=np.float32)
    actions = np.zeros((rows, contract.EXPECTED_ACTION_DIM), dtype=np.float32)
    reset = np.zeros(rows, dtype=np.bool_)
    reset[0] = True
    case_ids = np.asarray([*cases, cases[0], *cases], dtype="U64")
    step_indices = np.arange(1, rows + 1, dtype=np.int64)
    step_indices[7] = 140
    corpus = SimpleNamespace(
        observations=observations,
        actions=actions,
        reset_mask=reset,
        case_ids=case_ids,
        step_indices=step_indices,
        tranche_ids=np.asarray(["mutation"] * rows, dtype="U64"),
    )
    return fitter.RecoveryCorpusBundle(
        corpus=corpus,
        stratum_ids=np.asarray(fitter.expected_stratum_ids(), dtype="U96"),
        base_indices={
            case_id: np.asarray([index]) for index, case_id in enumerate(cases)
        },
        r4_indices=np.asarray([6], dtype=np.int64),
        observer_indices={
            case_id: np.asarray([7 + index], dtype=np.int64)
            for index, case_id in enumerate(cases)
        },
        observer_plus_late_indices=np.asarray([7], dtype=np.int64),
    )


def test_mutated_recomputed_label_array_fails_byte_exact_closure() -> None:
    expected = {
        "observations": np.zeros((2, 35), dtype=np.float32),
        "actions": np.zeros((2, 2), dtype=np.float32),
    }
    mutated = {name: value.copy() for name, value in expected.items()}
    mutated["actions"][1, 0] = np.nextafter(
        np.float32(0.0), np.float32(1.0), dtype=np.float32
    )
    with pytest.raises(probe.V12R8RecoveryProbeError, match="array bytes"):
        probe._assert_array_mapping_byte_exact(
            expected, mutated, label="mutated observer label"
        )


def test_mutated_assembled_corpus_array_fails_byte_exact_closure() -> None:
    expected = {
        "observations": np.zeros((2, 35), dtype=np.float32),
        "stratum_ids": np.asarray(["base::a", "observer::a"], dtype="U96"),
    }
    mutated = {name: value.copy() for name, value in expected.items()}
    mutated["observations"][0, 2] = np.float32(0.125)
    with pytest.raises(fitter.V12R8RecoveryFitError, match="array bytes"):
        fitter._assert_array_mapping_byte_exact(
            expected, mutated, label="mutated assembled corpus"
        )


def test_mutated_summary_metric_fails_recomputed_metric_closure() -> None:
    _module, state, _manifest = fitter._load_source_module_and_state()
    metrics = fitter.recompute_fit_metric_payload(_metric_bundle(), state)
    summary = {name: copy.deepcopy(metrics[name]) for name in fitter.FIT_METRIC_FIELDS}
    summary["global_metrics"]["rmse"] += 1.0e-6
    with pytest.raises(fitter.V12R8RecoveryFitError, match="metrics drifted"):
        fitter._assert_fit_metric_payload(
            summary, metrics, include_worst_row=True, label="mutated summary"
        )


def test_mutated_actor_mean_fails_recomputed_metric_closure() -> None:
    _module, state, _manifest = fitter._load_source_module_and_state()
    bundle = _metric_bundle()
    original = fitter.recompute_fit_metric_payload(bundle, state)
    expected_summary = {
        name: copy.deepcopy(original[name]) for name in fitter.FIT_METRIC_FIELDS
    }
    mutated_state = fitter.v11._clone_state(state)
    mutated_state["pi.1.bias"][0] = np.float32(
        mutated_state["pi.1.bias"][0] + np.float32(0.25)
    )
    recomputed = fitter.recompute_fit_metric_payload(bundle, mutated_state)
    assert not np.array_equal(original["predictions"], recomputed["predictions"])
    with pytest.raises(fitter.V12R8RecoveryFitError, match="metrics drifted"):
        fitter._assert_fit_metric_payload(
            expected_summary,
            recomputed,
            include_worst_row=True,
            label="mutated actor mean",
        )


def test_reloaded_module_logits_must_match_audited_state_tensors() -> None:
    module, state, _manifest = fitter._load_source_module_and_state()
    observations = np.zeros((3, contract.EXPECTED_ACTOR_FEATURES), dtype=np.float32)
    observed = fitter._runtime_state_logits_exact(module, state, observations)
    assert observed.shape[0] == len(observations)

    mutated_state = fitter.v11._clone_state(state)
    mutated_state["pi.1.bias"][0] = np.float32(
        mutated_state["pi.1.bias"][0] + np.float32(0.25)
    )
    with pytest.raises(fitter.V12R8RecoveryFitError, match="RLModule logits"):
        fitter._runtime_state_logits_exact(module, mutated_state, observations)


def test_mutated_teacher_tree_fails_before_label_query(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    drifted = copy.deepcopy(contract.LOCKED_SOURCE_H0_TREE)
    drifted["tree_sha256"] = "0" * 64
    monkeypatch.setattr(probe, "_tree_artifact", lambda *_args, **_kwargs: drifted)
    source = probe.REPO_ROOT.joinpath(*contract.SOURCE_H0_MODULE_PATH.parts)
    with pytest.raises(probe.V12R8RecoveryProbeError, match="teacher tree drifted"):
        probe.attest_offline_label_inputs(source, artifact_root=probe.REPO_ROOT)


def test_mutated_coverage_reference_fails_before_label_query(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    drifted = {
        key: contract.LOCKED_COVERAGE_REFERENCE[key]
        for key in ("path", "sha256", "size_bytes")
    }
    drifted["sha256"] = "0" * 64
    monkeypatch.setattr(probe, "_artifact", lambda *_args, **_kwargs: drifted)
    source = probe.REPO_ROOT.joinpath(*contract.SOURCE_H0_MODULE_PATH.parts)
    with pytest.raises(probe.V12R8RecoveryProbeError, match="coverage reference"):
        probe.attest_offline_label_inputs(source, artifact_root=probe.REPO_ROOT)


@pytest.mark.parametrize("function", (fitter.run_fit_stage, fitter.verify_fit_stage))
def test_fit_surfaces_recheck_locked_inputs_at_exit(function: object) -> None:
    tree = ast.parse(inspect.getsource(function))
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "attest_locked_inputs"
    ]
    returns = [node for node in ast.walk(tree) if isinstance(node, ast.Return)]
    assert len(calls) >= 2
    assert returns
    assert max(call.lineno for call in calls) < max(node.lineno for node in returns)
