from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

import h0_primary_split_v12r2_autonomy_recovery_contract as contract
import h0_primary_split_v12r2_recovery_weighted_fitter as fit
import h0_forensic_rollout as forensic


def test_recovery_ramp_endpoints_and_midpoint_are_frozen() -> None:
    assert fit.recovery_ramp_weight(0.0) == 1.0
    assert fit.recovery_ramp_weight(0.010) == 1.0
    assert fit.recovery_ramp_weight(0.0125) == pytest.approx(50.5)
    assert fit.recovery_ramp_weight(0.015) == 100.0
    assert fit.recovery_ramp_weight(0.100) == 100.0
    for invalid in (-1.0e-9, math.nan, math.inf, -math.inf):
        with pytest.raises(fit.V12R2RecoveryWeightedFitError):
            fit.recovery_ramp_weight(invalid)


def test_each_episode_normalizes_to_exact_float64_mass_500() -> None:
    episodes = (
        [100.0] + [1.0] * 499,
        [100.0, 100.0] + [1.0] * 257,
        [100.0],
        np.linspace(1.0, 100.0, 500, dtype=np.float64).tolist(),
    )
    for raw in episodes:
        normalized = fit.normalized_episode_sample_weights(raw)
        assert normalized.dtype == np.dtype("float64")
        assert normalized.shape == (len(raw),)
        assert math.fsum(normalized.tolist()) == 500.0
        assert np.all(normalized > 0.0)


def test_observer_weights_use_max_of_reset_recovery_and_coverage() -> None:
    penetration = np.asarray([0.0, 0.010, 0.0125, 0.015], dtype=np.float64)
    reset = np.asarray([True, False, False, False], dtype=np.bool_)
    ood = np.asarray([False, True, False, False], dtype=np.bool_)
    raw, normalized = fit.observer_episode_weights(penetration, reset, ood)
    assert raw[[0, 1, 3]].tolist() == [100.0, 100.0, 100.0]
    assert raw[2] == pytest.approx(50.5)
    assert math.fsum(normalized.tolist()) == 500.0


def test_corpus_component_order_is_seed_then_label_then_matching_round() -> None:
    minus, stochastic = contract.COLLECTION_CASE_IDS
    assert fit.expected_corpus_component_order("p0") == (("v11_seed",),)
    assert fit.expected_corpus_component_order("p1") == (
        ("v11_seed",),
        ("pure_observer", "p0"),
        ("v12r2_shielded", 1, minus),
        ("v12r2_shielded", 1, stochastic),
    )
    assert fit.expected_corpus_component_order("p3") == (
        ("v11_seed",),
        ("pure_observer", "p0"),
        ("v12r2_shielded", 1, minus),
        ("v12r2_shielded", 1, stochastic),
        ("pure_observer", "p1"),
        ("v12r2_shielded", 2, minus),
        ("v12r2_shielded", 2, stochastic),
        ("pure_observer", "p2"),
        ("v12r2_shielded", 3, minus),
        ("v12r2_shielded", 3, stochastic),
    )


def test_real_coverage_reference_hashes_and_p95_are_exact() -> None:
    audit = fit.coverage_reference_audit()
    hashes = audit["hashes"]
    assert audit["passed"] is True
    assert (
        hashes["observations"]
        == contract.COVERAGE_WEIGHTING["reference_observations_sha256"]
    )
    assert (
        hashes["normalization_mean"]
        == contract.COVERAGE_WEIGHTING["normalization_mean_sha256"]
    )
    assert (
        hashes["normalization_std"]
        == contract.COVERAGE_WEIGHTING["normalization_std_sha256"]
    )
    assert (
        hashes["normalized_features"]
        == contract.COVERAGE_WEIGHTING["normalized_feature_matrix_sha256"]
    )
    assert (
        hashes["nearest_indices"]
        == contract.COVERAGE_WEIGHTING["loo_nearest_indices_sha256"]
    )
    assert (
        hashes["loo_distances"] == contract.COVERAGE_WEIGHTING["loo_distances_sha256"]
    )
    assert audit["loo_p95"] == contract.COVERAGE_WEIGHTING["loo_p95"]


def test_new_row_query_reproduces_frozen_v11_failure_certificate() -> None:
    trace_path = fit.REPO_ROOT / contract.V11_FINAL_FAILURE_TRACE_PATH
    rows = forensic.strict_json_load(trace_path)
    observations = np.ascontiguousarray(
        [row["v26_observation"] for row in rows], dtype=np.float32
    )
    result = fit.evaluate_observer_coverage(observations)
    expected = contract.V11_PURE_FAILURE_COVERAGE
    assert fit.array_sha256(observations) == expected["trace_observations_sha256"]
    assert (
        result["normalized_features_sha256"] == expected["normalized_features_sha256"]
    )
    assert result["nearest_indices_sha256"] == expected["nearest_indices_sha256"]
    assert result["distances_sha256"] == expected["nearest_distances_sha256"]
    assert int(np.count_nonzero(result["ood_mask"])) == expected["ood_row_count"]
    assert (
        bool(np.all(result["ood_mask"][200:]))
        is expected["all_steps_201_through_259_ood"]
    )
    assert result["distance_rms_z"][200] == expected["step_201_distance"]
    assert result["distance_rms_z"][258] == expected["step_259_distance"]


def test_p0_reproduction_tolerance_is_taken_from_r1_contract() -> None:
    reference = {
        "rmse": 0.01,
        "max_abs_error": 0.02,
        "reset_max_abs_error": 0.03,
    }
    observed = dict(reference)
    match = fit._metric_match(observed, reference)
    assert match["passed"] is True
    assert match["absolute_tolerance"] == contract.P0_REPRODUCTION_TOLERANCE["absolute"]
    assert match["relative_tolerance"] == contract.P0_REPRODUCTION_TOLERANCE["relative"]


def test_no_unit_test_executes_the_3000_epoch_fit() -> None:
    source = Path(fit.__file__).read_text(encoding="utf-8")
    assert "for epoch in range(1, 3001)" in source
    assert "sample_weights" in source
    assert "dtype=torch.float64" in source
    assert "h0_v12r1_run_20260809" not in source


@pytest.mark.parametrize(
    ("stage", "labelled"),
    [
        ("p0", {}),
        ("p1", {"p0": 259}),
        ("p2", {"p0": 259, "p1": 500}),
        ("p3", {"p0": 259, "p1": 500, "p2": 411}),
    ],
)
def test_fitter_audit_builder_matches_real_contract_wire_schema(
    stage: str, labelled: dict[str, int]
) -> None:
    expected = contract.expected_fit_counts(stage, labelled_probe_rows=labelled)
    audit = fit.build_fit_corpus_audit(
        v11_seed_rows=expected["v11_seed_sample_count"],
        dagger_rows=expected["v12_dagger_sample_count"],
        pure_rows=expected["pure_probe_label_sample_count"],
        sample_count=expected["sample_count"],
        reset_row_count=expected["reset_row_count"],
        duplicate_sample_count=0,
        all_finite=True,
    )
    assert contract._fit_corpus_audit_matches(audit, expected) is True
    assert set(audit) == {
        "v11_seed_sample_count",
        "v12_dagger_sample_count",
        "same_state_v12_dagger_sample_count",
        "pure_probe_label_sample_count",
        "same_state_pure_probe_label_sample_count",
        "sample_count",
        "reset_row_count",
        "duplicate_sample_count",
        "all_finite",
    }
