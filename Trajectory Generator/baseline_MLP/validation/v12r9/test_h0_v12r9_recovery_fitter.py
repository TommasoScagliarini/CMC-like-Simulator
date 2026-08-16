from __future__ import annotations

import ast
import math
import sys
from pathlib import Path

import numpy as np
import pytest


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r9_recovery_contract as contract  # noqa: E402
import h0_v12r9_recovery_fitter as fitter  # noqa: E402


def _observer_arrays(case_id: str, *, rows: int = 140) -> dict[str, np.ndarray]:
    observations = np.zeros((rows, 35), dtype=np.float32)
    observations[:, 1] = np.float32(1.0)
    reset = np.zeros(rows, dtype=np.bool_)
    reset[0] = True
    return {
        "observations": observations,
        "actions": np.zeros((rows, 2), dtype=np.float32),
        "reset_mask": reset,
        "previous_penetration_m": np.linspace(0.0, 0.02, rows, dtype=np.float64),
        "coverage_distance_rms_z": np.linspace(0.0, 2.0, rows, dtype=np.float64),
        "coverage_nearest_reference_index": np.arange(rows, dtype=np.int64),
        "coverage_ood_mask": np.zeros(rows, dtype=np.bool_),
        "raw_sample_weights": np.ones(rows, dtype=np.float64),
        "normalized_sample_weights": np.ones(rows, dtype=np.float64),
        "actor_feature_names": np.asarray(
            [f"feature_{index}" for index in range(35)], dtype="U64"
        ),
        "case_ids": np.repeat(np.asarray([case_id], dtype="U64"), rows),
        "step_indices": np.arange(1, rows + 1, dtype=np.int64),
        "tranche_ids": np.repeat(np.asarray(["observer_probe_p0"], dtype="U64"), rows),
        "origins": np.asarray(
            [f"pure_observer:p0:{case_id}:{step}" for step in range(1, rows + 1)],
            dtype="U160",
        ),
    }


def _write_observer_npz(path: Path, arrays: dict[str, np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, **arrays)


def test_adamw_schedule_matches_contract_boundaries() -> None:
    assert fitter.adamw_learning_rate(1) == 3.0e-4
    assert fitter.adamw_learning_rate(1000) == 3.0e-4
    assert fitter.adamw_learning_rate(1001) == 1.0e-4
    assert fitter.adamw_learning_rate(1700) == 1.0e-4
    assert fitter.adamw_learning_rate(1701) == 3.0e-5
    assert fitter.adamw_learning_rate(2000) == 3.0e-5
    for value in (0, 2001, True):
        with pytest.raises(fitter.V12R9RecoveryFitError):
            fitter.adamw_learning_rate(value)


def test_thirteen_equal_mass_strata_are_uniform() -> None:
    expected = fitter.expected_stratum_ids()
    assert len(expected) == contract.FIT["stratum_count"] == 13
    assert len(set(expected)) == 13
    counts = {name: index + 1 for index, name in enumerate(expected)}
    strata = np.concatenate(
        [
            np.repeat(np.asarray([name], dtype="U96"), count)
            for name, count in counts.items()
        ]
    )
    weights, audit = fitter.compute_equal_stratum_weights(strata)
    assert audit["stratum_order"] == list(expected)
    assert audit["within_stratum_weighting"] == "UNIFORM"
    assert math.isclose(audit["total_sample_mass"], 6500.0, abs_tol=1.0e-9)
    for name in expected:
        selected = strata == name
        assert len(np.unique(weights[selected])) == 1
        assert math.isclose(
            math.fsum(weights[selected].tolist()), 500.0, abs_tol=1.0e-9
        )


def test_stratum_weighting_rejects_missing_or_extra_stratum() -> None:
    expected = np.asarray(fitter.expected_stratum_ids(), dtype="U96")
    with pytest.raises(fitter.V12R9RecoveryFitError, match="stratum id set"):
        fitter.compute_equal_stratum_weights(expected[:-1])
    extra = np.concatenate((expected, np.asarray(["extra"], dtype="U96")))
    with pytest.raises(fitter.V12R9RecoveryFitError, match="stratum id set"):
        fitter.compute_equal_stratum_weights(extra)


def test_locked_base_r4_and_r6_source_are_exact() -> None:
    attestation = fitter.attest_locked_inputs()
    assert (
        attestation["r6_candidate"]["tree_sha256"]
        == (contract.LOCKED_INPUTS["r6_candidate"]["tree_sha256"])
    )
    base = fitter._load_base_piece()
    r4 = fitter._load_r4_piece(base["actor_feature_names"])
    assert base["observations"].shape == (9232, 35)
    assert base["actions"].shape == (9232, 2)
    assert int(np.count_nonzero(base["reset_mask"])) == 19
    assert r4["observations"].shape == (212, 35)
    assert int(np.count_nonzero(r4["reset_mask"])) == 1
    _module, state, manifest = fitter._load_source_module_and_state()
    audit = fitter.validate_source_r6_state(state)
    assert audit["hidden_dims"] == [512, 512]
    assert audit["disabled_clock_columns_bit_zero"] is True
    assert audit["logstd_weight_bit_zero"] is True
    assert audit["actor_digest"] == manifest["actor_digest"]


def test_observer_label_npz_exact_schema_and_literals(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    case_id = contract.COLLECTION_CASE_IDS[0]
    arrays = _observer_arrays(case_id)
    source = tmp_path / "labels.npz"
    _write_observer_npz(source, arrays)
    monkeypatch.setattr(fitter, "REPO_ROOT", tmp_path)
    loaded = fitter._load_observer_piece(
        case_id, arrays["actor_feature_names"], path=source
    )
    assert loaded["observations"].shape == (140, 35)
    assert loaded["actions"].shape == (140, 2)
    assert loaded["episode_ids"].tolist() == [f"v12r9_observer:{case_id}"] * 140


@pytest.mark.parametrize(
    ("drift", "match"),
    [
        ("extra_key", "key set drifted"),
        ("wrong_dtype", "observer label NPZ drifted"),
        ("wrong_origin", "observer label NPZ drifted"),
        ("wrong_tranche", "observer label NPZ drifted"),
    ],
)
def test_observer_label_npz_rejects_schema_or_provenance_drift(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    drift: str,
    match: str,
) -> None:
    case_id = contract.COLLECTION_CASE_IDS[0]
    arrays = _observer_arrays(case_id)
    if drift == "extra_key":
        arrays["extra"] = np.zeros(140, dtype=np.float32)
    elif drift == "wrong_dtype":
        arrays["actions"] = arrays["actions"].astype(np.float64)
    elif drift == "wrong_origin":
        arrays["origins"][0] = "wrong"
    elif drift == "wrong_tranche":
        arrays["tranche_ids"][0] = "wrong"
    source = tmp_path / "labels.npz"
    _write_observer_npz(source, arrays)
    monkeypatch.setattr(fitter, "REPO_ROOT", tmp_path)
    with pytest.raises(fitter.V12R9RecoveryFitError, match=match):
        fitter._load_observer_piece(
            case_id, _observer_arrays(case_id)["actor_feature_names"], path=source
        )


def test_512_normalization_fold_preserves_logstd_and_clock_contract() -> None:
    import torch

    _module, source_state, _manifest = fitter._load_source_module_and_state()
    base = fitter._load_base_piece()
    selected = base["tranche_ids"].astype(str) == "v8r1p1_base"
    normalization = fitter.v11.frozen_base_normalization(base["observations"][selected])
    model = fitter._new_normalized_model(source_state, normalization)
    with torch.no_grad():
        model[2].bias[0].add_(np.float32(1.0e-4))
        model[4].bias[0].add_(np.float32(1.0e-4))
    candidate, fold = fitter._fold_normalization_into_state(
        model, source_state, normalization
    )
    probe = base["observations"][:256]
    normalized_probe = fitter.v11.normalized_observations(probe, normalization)
    with torch.no_grad():
        normalized_prediction = model(
            torch.as_tensor(normalized_probe, dtype=torch.float32)
        ).numpy()
    runtime_prediction = fitter.v11._state_logits(candidate, probe)[:, :2]
    equivalence = fitter.v11.fold_equivalence_audit(
        normalized_prediction, runtime_prediction
    )
    audit = fitter.full_mean_update_audit(source_state, candidate)
    assert fold["folded_clock_columns_bit_zero"] is True
    assert equivalence["fold_equivalence_passed"] is True
    assert audit["passed"] is True
    assert audit["logstd_parameter_rows_byte_exact"] is True
    assert audit["critic_present"] is False


def test_actor_manifest_exact_interface(tmp_path: Path) -> None:
    _module, state, source_manifest = fitter._load_source_module_and_state()
    module_state = tmp_path / "module_state.pkl"
    module_state.write_bytes(b"synthetic")
    manifest = fitter._candidate_actor_manifest(
        state=state,
        feature_names=source_manifest["actor_feature_names"],
        module_state=module_state,
    )
    assert set(manifest) == fitter.ACTOR_MANIFEST_FIELDS
    assert manifest["schema_version"] == 1
    assert manifest["status"] == "H0_V12R9_RECOVERY_ACTOR_FEATURE_CONTRACT"
    assert manifest["topology_id"] == "V12R9_STANDARD_RECOVERY_W512_V1"
    assert manifest["fcnet_hiddens"] == [512, 512]
    assert manifest["disabled_clock_columns"] == [0, 1]


def test_fitter_source_has_one_fixed_fit_and_no_environment_surface() -> None:
    source = Path(fitter.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    lbfgs_keywords = [
        {
            keyword.arg: ast.literal_eval(keyword.value)
            for keyword in node.keywords
            if keyword.arg in {"max_iter", "max_eval"}
        }
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "LBFGS"
    ]
    assert lbfgs_keywords == [{"max_iter": 300, "max_eval": 600}]
    assert "range(1, 2001)" in source
    assert "make_cmc_env" not in source
    assert "env.reset" not in source
    assert "env.step" not in source
    assert "hard_polish" not in source
    assert '"critic_updates": 0' in source
    assert '"ppo_updates": 0' in source


def test_run_fit_is_no_clobber_before_any_input_read(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(fitter, "REPO_ROOT", tmp_path)
    destination = fitter._resolve(contract.FIT_ROOT)
    destination.mkdir(parents=True)
    with pytest.raises(fitter.V12R9RecoveryFitError, match="exists/no-clobber"):
        fitter.run_fit_stage(
            pipeline_claim_path="missing_claim.json",
            worker_claim_path="missing_worker.json",
            protocol_freeze_path="missing_freeze.json",
            execution_lock_path="missing_lock.json",
        )


def test_import_does_not_create_canonical_fit() -> None:
    assert not fitter.os.path.lexists(fitter._resolve(contract.FIT_ROOT))
