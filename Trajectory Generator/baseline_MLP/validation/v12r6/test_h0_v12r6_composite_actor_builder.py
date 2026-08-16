"""Tests for the source-only V12R6 functional-composite builder."""

from __future__ import annotations

import inspect
import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest


V12R6_ROOT = Path(__file__).resolve().parent
if os.fspath(V12R6_ROOT) not in sys.path:
    sys.path.insert(0, os.fspath(V12R6_ROOT))

import build_h0_v12r6_composite_actor as builder  # noqa: E402


@pytest.fixture(scope="module")
def source_modules():
    p2 = builder.RLModule.from_checkpoint(builder.DEFAULT_P2_CHECKPOINT)
    r5 = builder.RLModule.from_checkpoint(builder.DEFAULT_R5_CHECKPOINT)
    p2.eval()
    r5.eval()
    return p2, r5


@pytest.fixture(scope="module")
def corpus() -> tuple[np.ndarray, tuple[str, ...]]:
    return builder._load_corpus(builder.DEFAULT_CORPUS_PATH)  # noqa: SLF001


@pytest.fixture(scope="module")
def composite(source_modules):
    p2, r5 = source_modules
    p2_state, r5_state, _audit = builder._validate_source_pair(  # noqa: SLF001
        p2, r5
    )
    config = dict(p2.model_config)
    config["fcnet_hiddens"] = list(builder.TARGET_HIDDENS)
    config["freeze_actor"] = True
    module = builder.AsymmetricActorCriticTorchRLModule(
        observation_space=p2.observation_space,
        action_space=p2.action_space,
        inference_only=True,
        learner_only=False,
        model_config=config,
        catalog_class=None,
    )
    module.set_state(builder.build_composite_state(p2_state, r5_state))
    module.eval()
    return module


def _mutable_state(module) -> dict[str, np.ndarray]:
    return {
        name: np.ascontiguousarray(builder._array(value)).copy()  # noqa: SLF001
        for name, value in module.get_state().items()
    }


def test_builder_is_inert_and_requires_an_explicit_output_path(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    signature = inspect.signature(builder.build_verify_save_composite)

    assert signature.parameters["output_path"].default is inspect.Parameter.empty
    assert not hasattr(builder, "DEFAULT_OUTPUT_PATH")
    assert builder.P2_WEIGHT == 0.70
    assert builder.R5_WEIGHT == 0.30
    assert builder.P2_WEIGHT + builder.R5_WEIGHT == 1.0
    assert builder.TARGET_HIDDENS == (512, 512)

    monkeypatch.setattr(sys, "argv", [builder.__file__])
    with pytest.raises(SystemExit):
        builder._parse_args()  # noqa: SLF001


def test_source_pair_is_exact_standard_256_clock_zero_and_constant_sigma(
    source_modules,
) -> None:
    p2, r5 = source_modules

    _p2_state, _r5_state, audit = builder._validate_source_pair(  # noqa: SLF001
        p2, r5
    )

    assert audit["spaces_byte_exact"] is True
    assert audit["logstd_sources_byte_exact"] is True
    assert audit["p2"]["standard_module_class"] is True
    assert audit["r5"]["standard_module_class"] is True
    assert audit["p2"]["fcnet_hiddens"] == [256, 256]
    assert audit["r5"]["fcnet_hiddens"] == [256, 256]
    assert audit["p2"]["disabled_clock_columns_positive_zero"] is True
    assert audit["r5"]["disabled_clock_columns_positive_zero"] is True
    assert audit["logstd_bias"] == pytest.approx([-5.2983174324] * 2, abs=1.0e-9)
    assert audit["sigma"] == pytest.approx([0.005] * 2, abs=1.0e-9)


def test_composite_state_is_concatenated_block_diagonal_and_scaled(
    source_modules,
) -> None:
    p2, r5 = source_modules
    p2_state = _mutable_state(p2)
    r5_state = _mutable_state(r5)

    state = builder.build_composite_state(p2_state, r5_state)

    assert set(state) == builder.EXPECTED_ACTOR_STATE_KEYS
    assert np.array_equal(
        state["pi_encoder.0.weight"][:256], p2_state["pi_encoder.0.weight"]
    )
    assert np.array_equal(
        state["pi_encoder.0.weight"][256:], r5_state["pi_encoder.0.weight"]
    )
    assert np.array_equal(
        state["pi_encoder.0.bias"][:256], p2_state["pi_encoder.0.bias"]
    )
    assert np.array_equal(
        state["pi_encoder.0.bias"][256:], r5_state["pi_encoder.0.bias"]
    )
    assert np.array_equal(
        state["pi_encoder.2.weight"][:256, :256],
        p2_state["pi_encoder.2.weight"],
    )
    assert np.array_equal(
        state["pi_encoder.2.weight"][256:, 256:],
        r5_state["pi_encoder.2.weight"],
    )
    assert builder._positive_zero_bits(  # noqa: SLF001
        state["pi_encoder.2.weight"][:256, 256:]
    )
    assert builder._positive_zero_bits(  # noqa: SLF001
        state["pi_encoder.2.weight"][256:, :256]
    )
    assert np.array_equal(
        state["pi.1.weight"][:2, :256],
        builder.P2_WEIGHT * p2_state["pi.1.weight"][:2],
    )
    assert np.array_equal(
        state["pi.1.weight"][:2, 256:],
        builder.R5_WEIGHT * r5_state["pi.1.weight"][:2],
    )
    expected_bias = (
        builder.P2_WEIGHT * p2_state["pi.1.bias"][:2]
        + builder.R5_WEIGHT * r5_state["pi.1.bias"][:2]
    )
    assert np.array_equal(state["pi.1.bias"][:2], expected_bias)
    assert builder._positive_zero_bits(state["pi.1.weight"][2:])  # noqa: SLF001
    assert np.array_equal(state["pi.1.bias"][2:], p2_state["pi.1.bias"][2:])
    for canonical, alias in builder.ENCODER_ALIAS_PAIRS:
        assert np.array_equal(state[canonical], state[alias])


@pytest.mark.parametrize(
    "mutation,error",
    (
        (
            lambda p2, _r5: p2["pi.0.0.weight"].__setitem__((0, 2), 99.0),
            "encoder alias drifted",
        ),
        (
            lambda p2, _r5: (
                p2["pi_encoder.0.weight"].__setitem__((slice(None), 0), -0.0),
                p2["pi.0.0.weight"].__setitem__((slice(None), 0), -0.0),
            ),
            "clock columns are not positive zero",
        ),
        (
            lambda p2, _r5: p2["pi.1.weight"].__setitem__((2, 0), 1.0e-4),
            "logstd weight is not positive zero",
        ),
        (
            lambda _p2, r5: r5["pi.1.bias"].__setitem__(2, -4.0),
            "source logstd biases differ",
        ),
    ),
)
def test_state_builder_fails_closed_on_source_invariant_drift(
    source_modules, mutation, error: str
) -> None:
    p2, r5 = source_modules
    p2_state = _mutable_state(p2)
    r5_state = _mutable_state(r5)
    mutation(p2_state, r5_state)

    with pytest.raises(builder.CompositeActorBuildError, match=error):
        builder.build_composite_state(p2_state, r5_state)


def test_module_is_standard_512_with_live_encoder_alias_and_clock_zero(
    composite,
) -> None:
    audit = builder._topology_audit(composite)  # noqa: SLF001

    assert audit["passed"] is True
    assert audit["topology_id"] == builder.TOPOLOGY_ID
    assert audit["fcnet_hiddens"] == [512, 512]
    assert audit["checks"]["standard_module_class"] is True
    assert audit["checks"]["actor_only_inference"] is True
    assert audit["checks"]["encoder_object_alias"] is True
    assert audit["checks"]["encoder_state_alias_values"] is True
    assert audit["checks"]["disabled_clock_columns_positive_zero"] is True
    assert audit["checks"]["logstd_weight_positive_zero"] is True
    assert composite.pi[0] is composite.pi_encoder


def test_direct_equivalence_on_complete_corpus_and_deterministic_inputs(
    source_modules, corpus, composite
) -> None:
    p2, r5 = source_modules
    observations, _feature_names = corpus

    audit = builder.verify_composite_actor(composite, p2, r5, observations)

    assert audit["passed"] is True
    assert audit["corpus_direct_equivalence"]["rows"] == 9232
    assert audit["corpus_direct_equivalence"]["mean_equivalent"] is True
    assert audit["corpus_direct_equivalence"]["mean_max_abs_error"] <= 2.0e-6
    assert audit["corpus_direct_equivalence"]["composite_logstd_byte_exact"] is True
    assert audit["deterministic_direct_equivalence"]["rows"] == 16
    assert audit["deterministic_direct_equivalence"]["mean_equivalent"] is True
    assert (
        audit["deterministic_direct_equivalence"]["composite_logstd_byte_exact"] is True
    )
    assert (
        audit["clock_invariance"]["output_byte_exact_under_clock_perturbation"] is True
    )
    assert audit["corpus_direct_equivalence"]["sigma"] == pytest.approx(
        [0.005, 0.005], abs=1.0e-9
    )


def test_direct_equivalence_rejects_a_changed_composite_head(
    source_modules, corpus, composite
) -> None:
    p2, r5 = source_modules
    observations, _feature_names = corpus
    state = _mutable_state(composite)
    state["pi.1.bias"][0] += np.float32(1.0e-3)
    changed = builder.AsymmetricActorCriticTorchRLModule(
        observation_space=composite.observation_space,
        action_space=composite.action_space,
        inference_only=True,
        learner_only=False,
        model_config=dict(composite.model_config),
        catalog_class=None,
    )
    changed.set_state(state)

    with pytest.raises(builder.CompositeActorBuildError, match="direct equivalence"):
        builder.verify_composite_actor(changed, p2, r5, observations)


def test_save_reload_manifests_and_512_transplants_are_exact(
    tmp_path: Path,
) -> None:
    output = tmp_path / "rl_module_composite"

    result = builder.build_verify_save_composite(output_path=output)

    assert result["passed"] is True
    assert result["status"] == "H0_V12R6_COMPOSITE_BUILD_PASS"
    assert result["mean_formula"] == "0.70*P2+0.30*R5"
    assert result["weights"] == {"p2": 0.70, "r5": 0.30}
    assert result["construction"]["target_hiddens"] == [512, 512]
    assert result["before_save"]["passed"] is True
    assert result["after_reload"]["passed"] is True
    assert result["final_reload"]["passed"] is True
    transplant = result["warm_start_and_checkpoint_zero_512"]
    assert transplant["passed"] is True
    assert transplant["required_target_fcnet_hiddens"] == [512, 512]
    assert transplant["warm_start_transplant_actor_exact"] is True
    assert transplant["checkpoint_zero_standard_actor_transplant_exact"] is True
    assert transplant["fresh_critic_preserved_exact"] is True
    assert transplant["forward_surface_byte_exact"] is True
    assert set(path.name for path in output.iterdir()) == {
        *builder.RL_MODULE_FILES,
        builder.ACTOR_FEATURE_MANIFEST,
        builder.BUILD_MANIFEST,
    }
    actor_manifest = json.loads(
        (output / builder.ACTOR_FEATURE_MANIFEST).read_text(encoding="utf-8")
    )
    build_manifest = json.loads(
        (output / builder.BUILD_MANIFEST).read_text(encoding="utf-8")
    )
    assert actor_manifest["actor_feature_count"] == 35
    assert actor_manifest["actor_feature_names"][:2] == [
        "gait_phase_sin",
        "gait_phase_cos",
    ]
    assert actor_manifest["fcnet_hiddens"] == [512, 512]
    assert build_manifest["actor_digest"] == actor_manifest["actor_digest"]
    assert (
        build_manifest["module_state_sha256"] == actor_manifest["module_state_sha256"]
    )
    reloaded = builder.RLModule.from_checkpoint(output)
    assert type(reloaded) is builder.AsymmetricActorCriticTorchRLModule
    assert reloaded.pi[0] is reloaded.pi_encoder
    assert list(reloaded.model_config["fcnet_hiddens"]) == [512, 512]

    with pytest.raises(builder.CompositeActorBuildError, match="clobber"):
        builder.save_composite_checkpoint_no_clobber(reloaded, output)


def test_no_clobber_rejects_file_dangling_symlink_and_missing_parent(
    tmp_path: Path, composite
) -> None:
    occupied = tmp_path / "occupied"
    occupied.write_text("do not replace", encoding="utf-8")
    with pytest.raises(builder.CompositeActorBuildError, match="clobber"):
        builder.save_composite_checkpoint_no_clobber(composite, occupied)
    assert occupied.read_text(encoding="utf-8") == "do not replace"

    missing_parent = tmp_path / "missing" / "checkpoint"
    with pytest.raises(builder.CompositeActorBuildError, match="output parent"):
        builder.save_composite_checkpoint_no_clobber(composite, missing_parent)

    if hasattr(os, "symlink"):
        dangling_target = tmp_path / "absent-target"
        dangling = tmp_path / "dangling"
        dangling.symlink_to(dangling_target)
        with pytest.raises(
            builder.CompositeActorBuildError,
            match="symlink/junction",
        ):
            builder.save_composite_checkpoint_no_clobber(composite, dangling)
        assert dangling.is_symlink()


def test_public_build_module_api_writes_nothing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)
    before = set(tmp_path.iterdir())

    module = builder.build_composite_module()

    assert set(tmp_path.iterdir()) == before
    assert type(module) is builder.AsymmetricActorCriticTorchRLModule
    assert builder._topology_audit(module)["passed"] is True  # noqa: SLF001
