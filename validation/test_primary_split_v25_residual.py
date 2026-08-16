"""Tests for the PPO-compatible H0 + V25 residual RLModule."""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest
import torch
from gymnasium import spaces


REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_DIR = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

from asymmetric_rl_module import (  # noqa: E402
    AsymmetricActorCriticTorchRLModule,
)
from primary_split_v25_residual import (  # noqa: E402
    DEFAULT_RESIDUAL_LIMITS,
    RESET_BYPASS_SUPPORTED,
    RESET_BYPASS_UNAVAILABLE_REASON,
    RESIDUAL_HIDDEN_DIMS,
    RESIDUAL_INPUT_COUNT,
    RESIDUAL_INPUT_FEATURE_NAMES,
    RESIDUAL_INPUT_INDICES,
    PrimarySplitV25ResidualTorchRLModule,
    base_state_keys,
    build_v25_residual_module_from_checkpoint,
)
from ray.rllib.core.columns import Columns  # noqa: E402
from ray.rllib.core.rl_module.rl_module import RLModule  # noqa: E402


ACTOR_FEATURE_COUNT = 35


def _spaces() -> tuple[spaces.Box, spaces.Box]:
    observation = spaces.Box(
        low=-np.inf,
        high=np.inf,
        shape=(84,),
        dtype=np.float32,
    )
    action = spaces.Box(
        low=-1.0,
        high=1.0,
        shape=(2,),
        dtype=np.float32,
    )
    return observation, action


def _model_config(**updates) -> dict:
    config = {
        "n_actor": ACTOR_FEATURE_COUNT,
        "n_full": 84,
        "fcnet_hiddens": [8],
        "fcnet_activation": "tanh",
        "freeze_logstd": False,
        "freeze_actor": False,
        "primary_split_v25_residual_input_mean": [0.0] * RESIDUAL_INPUT_COUNT,
        "primary_split_v25_residual_input_std": [1.0] * RESIDUAL_INPUT_COUNT,
        "primary_split_v25_residual_limits": list(DEFAULT_RESIDUAL_LIMITS),
        "primary_split_v25_residual_reset_bypass": False,
    }
    config.update(updates)
    return config


def _module(*, inference_only: bool = False, **updates):
    observation, action = _spaces()
    return PrimarySplitV25ResidualTorchRLModule(
        observation_space=observation,
        action_space=action,
        inference_only=inference_only,
        learner_only=False,
        model_config=_model_config(**updates),
        catalog_class=None,
    )


def _array(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def _state_subset(module, keys) -> dict[str, np.ndarray]:
    state = module.get_state()
    return {key: _array(state[key]).copy() for key in keys}


def _full_source_module() -> AsymmetricActorCriticTorchRLModule:
    observation, action = _spaces()
    return AsymmetricActorCriticTorchRLModule(
        observation_space=observation,
        action_space=action,
        inference_only=False,
        learner_only=False,
        model_config={
            "n_actor": ACTOR_FEATURE_COUNT,
            "n_full": 84,
            "fcnet_hiddens": [8],
            "fcnet_activation": "tanh",
            "freeze_logstd": False,
            "freeze_actor": False,
        },
        catalog_class=None,
    )


def test_v25_residual_contract_and_zero_initialization() -> None:
    assert RESIDUAL_INPUT_INDICES == tuple(range(2, 35))
    assert RESIDUAL_INPUT_COUNT == 33
    assert RESIDUAL_HIDDEN_DIMS == (128, 128)
    assert len(RESIDUAL_INPUT_FEATURE_NAMES) == RESIDUAL_INPUT_COUNT
    assert not any("detector" in name.lower() for name in RESIDUAL_INPUT_FEATURE_NAMES)
    assert not RESET_BYPASS_SUPPORTED

    module = _module(inference_only=True)
    assert sum(parameter.numel() for parameter in module.residual_parameters()) == 21122
    assert module.primary_split_v25_residual[0].in_features == 33
    assert module.primary_split_v25_residual[0].out_features == 128
    assert module.primary_split_v25_residual[2].in_features == 128
    assert module.primary_split_v25_residual[2].out_features == 128
    assert module.primary_split_v25_residual[4].out_features == 2
    observations = torch.randn((7, ACTOR_FEATURE_COUNT))
    with torch.no_grad():
        assert torch.count_nonzero(module.residual_delta(observations)) == 0


def test_residual_does_not_read_excluded_clock_columns() -> None:
    module = _module(inference_only=True)
    module.eval()
    with torch.no_grad():
        module.primary_split_v25_residual[-1].weight.fill_(0.01)
    generator = torch.Generator().manual_seed(5)
    original = torch.randn((4, ACTOR_FEATURE_COUNT), generator=generator)
    changed = original.clone()
    changed[:, :2] = torch.randn((4, 2), generator=generator) * 100.0
    with torch.no_grad():
        original_delta = module.residual_delta(original)
        changed_delta = module.residual_delta(changed)
    assert torch.equal(original_delta, changed_delta)


def test_zero_residual_preserves_clipped_raw_h0_mean_and_exact_logstd() -> None:
    module = _module(inference_only=True)
    module.eval()
    observations = torch.randn((6, ACTOR_FEATURE_COUNT))
    with torch.no_grad():
        raw = module.pi(observations)
        output = module.forward_inference({Columns.OBS: observations})[
            Columns.ACTION_DIST_INPUTS
        ]
    assert torch.equal(output[:, :2], raw[:, :2].clamp(-1.0, 1.0))
    assert torch.equal(output[:, 2:], raw[:, 2:])


def test_forward_scales_residual_then_clips_means_and_preserves_logstd() -> None:
    module = _module(
        inference_only=False,
        primary_split_v25_residual_limits=[0.175, 0.12],
    )
    module.eval()
    with torch.no_grad():
        for parameter in module.pi.parameters():
            parameter.zero_()
        module.pi[-1].bias[:] = torch.tensor([0.9, -0.95, -5.0, -4.0])
        module.primary_split_v25_residual[-1].weight.zero_()
        module.primary_split_v25_residual[-1].bias[:] = torch.tensor([10.0, -10.0])

    observations = torch.zeros((3, 84), dtype=torch.float32)
    actor_observations = observations[:, :ACTOR_FEATURE_COUNT]
    with torch.no_grad():
        raw = module.pi(actor_observations)
        output = module.forward_inference({Columns.OBS: observations})[
            Columns.ACTION_DIST_INPUTS
        ]
        delta = module.residual_delta(actor_observations)

    assert torch.all(torch.abs(delta[:, 0]) <= 0.175)
    assert torch.all(torch.abs(delta[:, 1]) <= 0.12)
    torch.testing.assert_close(
        output[:, :2],
        torch.tensor([[1.0, -1.0]]).expand(3, -1),
        rtol=0.0,
        atol=1.0e-7,
    )
    assert torch.equal(output[:, 2:], raw[:, 2:])

    train_output = module.forward_train({Columns.OBS: observations})
    assert train_output[Columns.ACTION_DIST_INPUTS].shape == (3, 4)
    assert train_output[Columns.EMBEDDINGS].shape == (3, 8)
    assert module.compute_values({Columns.OBS: observations}).shape == (3,)


def test_reset_bypass_request_fails_without_causal_marker() -> None:
    assert "no causal" in RESET_BYPASS_UNAVAILABLE_REASON
    with pytest.raises(ValueError, match="no causal, unambiguous reset marker"):
        _module(primary_split_v25_residual_reset_bypass=True)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        (
            "primary_split_v25_residual_input_mean",
            [0.0] * 32,
            "exactly 33",
        ),
        (
            "primary_split_v25_residual_input_std",
            [1.0] * 32 + [0.0],
            "strictly positive",
        ),
        (
            "primary_split_v25_residual_limits",
            [0.175, float("nan")],
            "finite",
        ),
    ],
)
def test_invalid_serialized_contract_fails_closed(field, value, message) -> None:
    with pytest.raises(ValueError, match=message):
        _module(**{field: value})


def test_wrap_full_checkpoint_preserves_actor_critic_and_save_reload() -> None:
    source = _full_source_module()
    generator = torch.Generator().manual_seed(19)
    observations = torch.randn((5, 84), generator=generator)
    with tempfile.TemporaryDirectory() as raw_directory:
        root = Path(raw_directory)
        source_path = root / "source"
        candidate_path = root / "candidate"
        source.save_to_path(source_path)
        module = build_v25_residual_module_from_checkpoint(
            source_path,
            input_mean=[0.0] * RESIDUAL_INPUT_COUNT,
            input_std=[1.0] * RESIDUAL_INPUT_COUNT,
        )

        source_state = source.get_state()
        module_state = module.get_state()
        for key, value in source_state.items():
            np.testing.assert_array_equal(_array(value), _array(module_state[key]))
        assert "vf.weight" in module_state
        assert "vf_encoder.0.weight" in module_state

        module.eval()
        with torch.no_grad():
            before_logits = module.forward_inference({Columns.OBS: observations})[
                Columns.ACTION_DIST_INPUTS
            ]
            before_values = module.compute_values({Columns.OBS: observations})
        module.save_to_path(candidate_path)
        reloaded = RLModule.from_checkpoint(candidate_path)
        assert isinstance(reloaded, PrimarySplitV25ResidualTorchRLModule)
        reloaded.eval()
        with torch.no_grad():
            after_logits = reloaded.forward_inference({Columns.OBS: observations})[
                Columns.ACTION_DIST_INPUTS
            ]
            after_values = reloaded.compute_values({Columns.OBS: observations})
        assert torch.equal(before_logits, after_logits)
        assert torch.equal(before_values, after_values)
        for key, value in module.get_state().items():
            np.testing.assert_array_equal(
                _array(value),
                _array(reloaded.get_state()[key]),
            )


def test_official_h0_wrap_is_identity_and_reloads_exactly() -> None:
    checkpoint = (
        REPO_ROOT
        / "validation"
        / "critic_warmup"
        / "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry"
        / "rl_module_last"
    )
    source = RLModule.from_checkpoint(checkpoint)
    module = build_v25_residual_module_from_checkpoint(
        checkpoint,
        input_mean=[0.0] * RESIDUAL_INPUT_COUNT,
        input_std=[1.0] * RESIDUAL_INPUT_COUNT,
    )
    assert module.inference_only
    for key, value in source.get_state().items():
        np.testing.assert_array_equal(_array(value), _array(module.get_state()[key]))

    observations = torch.zeros((2, ACTOR_FEATURE_COUNT), dtype=torch.float32)
    source.eval()
    module.eval()
    with torch.no_grad():
        source_logits = source.forward_inference({Columns.OBS: observations})[
            Columns.ACTION_DIST_INPUTS
        ]
        module_logits = module.forward_inference({Columns.OBS: observations})[
            Columns.ACTION_DIST_INPUTS
        ]
    assert torch.equal(source_logits[:, :2].clamp(-1.0, 1.0), module_logits[:, :2])
    assert torch.equal(source_logits[:, 2:], module_logits[:, 2:])

    with tempfile.TemporaryDirectory() as raw_directory:
        candidate = Path(raw_directory) / "candidate"
        module.save_to_path(candidate)
        reloaded = RLModule.from_checkpoint(candidate)
        reloaded.eval()
        with torch.no_grad():
            reloaded_logits = reloaded.forward_inference({Columns.OBS: observations})[
                Columns.ACTION_DIST_INPUTS
            ]
        assert torch.equal(module_logits, reloaded_logits)


def test_residual_only_fit_keeps_h0_critic_and_logstd_exact_then_allows_ppo() -> None:
    source = _full_source_module()
    with tempfile.TemporaryDirectory() as raw_directory:
        source_path = Path(raw_directory) / "source"
        source.save_to_path(source_path)
        module = build_v25_residual_module_from_checkpoint(
            source_path,
            input_mean=[0.0] * RESIDUAL_INPUT_COUNT,
            input_std=[1.0] * RESIDUAL_INPUT_COUNT,
        )

    keys = base_state_keys(module)
    base_before = _state_subset(module, keys)
    residual_before = {
        name: parameter.detach().clone()
        for name, parameter in module.primary_split_v25_residual.named_parameters()
    }
    generator = torch.Generator().manual_seed(37)
    observations = torch.randn((16, 84), generator=generator)
    module.eval()
    with torch.no_grad():
        logstd_before = module.forward_inference({Columns.OBS: observations})[
            Columns.ACTION_DIST_INPUTS
        ][:, 2:].clone()

    trainable = module.prepare_residual_fit()
    trainable_ids = {id(parameter) for parameter in trainable}
    assert trainable_ids
    for parameter in module.parameters():
        assert parameter.requires_grad == (id(parameter) in trainable_ids)
    optimizer = torch.optim.Adam(trainable, lr=1.0e-3)
    logits = module.forward_train({Columns.OBS: observations})[
        Columns.ACTION_DIST_INPUTS
    ]
    target = logits[:, :2].detach() + torch.tensor([0.01, -0.01])
    loss = torch.square(logits[:, :2] - target).mean()
    optimizer.zero_grad(set_to_none=True)
    loss.backward()
    optimizer.step()

    base_after = _state_subset(module, keys)
    for key in keys:
        np.testing.assert_array_equal(base_before[key], base_after[key])
    assert any(
        not torch.equal(residual_before[name], parameter.detach())
        for name, parameter in module.primary_split_v25_residual.named_parameters()
    )
    module.eval()
    with torch.no_grad():
        logstd_after = module.forward_inference({Columns.OBS: observations})[
            Columns.ACTION_DIST_INPUTS
        ][:, 2:]
    assert torch.equal(logstd_before, logstd_after)

    module.prepare_for_ppo()
    assert all(parameter.requires_grad for parameter in module.parameters())
    train_output = module.forward_train({Columns.OBS: observations})
    assert Columns.ACTION_DIST_INPUTS in train_output
    assert Columns.EMBEDDINGS in train_output
    assert module.compute_values(
        {Columns.OBS: observations},
        embeddings=train_output[Columns.EMBEDDINGS],
    ).shape == (16,)
