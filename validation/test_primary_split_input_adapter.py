"""Unit tests for the PPO-compatible primary-split H0 input adapter."""

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
from primary_split_input_adapter import (  # noqa: E402
    ACTOR_FEATURE_COUNT,
    ADAPTER_HIDDEN_DIMS,
    ADAPTER_INPUT_COUNT,
    ADAPTER_INPUT_INDICES,
    CONTACT_FEATURE_INDEX,
    EVENT_AND_FSM_FEATURE_INDICES,
    LOAD_FEATURE_INDEX,
    RESET_BYPASS_SUPPORTED,
    RESET_BYPASS_UNAVAILABLE_REASON,
    PrimarySplitInputAdapterTorchRLModule,
    base_state_keys,
    build_adapter_module_from_checkpoint,
)
from ray.rllib.core.columns import Columns  # noqa: E402
from ray.rllib.core.rl_module.rl_module import RLModule  # noqa: E402


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
        "primary_split_adapter_input_mean": [0.0] * ADAPTER_INPUT_COUNT,
        "primary_split_adapter_input_std": [1.0] * ADAPTER_INPUT_COUNT,
        "primary_split_adapter_residual_limits": [0.05, 0.13],
        "primary_split_adapter_reset_bypass": False,
    }
    config.update(updates)
    return config


def _module(*, inference_only: bool = False, **updates):
    observation, action = _spaces()
    return PrimarySplitInputAdapterTorchRLModule(
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


def test_adapter_contract_excludes_every_event_and_fsm_feature() -> None:
    assert ADAPTER_INPUT_COUNT == 20
    assert ADAPTER_HIDDEN_DIMS == (64, 64)
    assert ADAPTER_INPUT_INDICES == (
        *range(2, 12),
        *range(25, 35),
    )
    assert set(ADAPTER_INPUT_INDICES).isdisjoint(EVENT_AND_FSM_FEATURE_INDICES)
    assert not RESET_BYPASS_SUPPORTED

    module = _module(inference_only=True)
    assert sum(parameter.numel() for parameter in module.adapter_parameters()) == 5634
    assert module.primary_split_input_adapter[0].in_features == 20
    assert module.primary_split_input_adapter[0].out_features == 64
    assert module.primary_split_input_adapter[2].in_features == 64
    assert module.primary_split_input_adapter[2].out_features == 64
    assert module.primary_split_input_adapter[4].out_features == 2


def test_proxy_is_independent_of_event_and_fsm_columns() -> None:
    module = _module(inference_only=True)
    module.eval()
    generator = torch.Generator().manual_seed(9)
    original = torch.randn((4, ACTOR_FEATURE_COUNT), generator=generator)
    changed = original.clone()
    changed[:, EVENT_AND_FSM_FEATURE_INDICES] = torch.randn(
        (4, len(EVENT_AND_FSM_FEATURE_INDICES)),
        generator=generator,
    )

    with torch.no_grad():
        translated_original, proxy_original = module.adapt_actor_observation(original)
        translated_changed, proxy_changed = module.adapt_actor_observation(changed)

    assert torch.equal(proxy_original, proxy_changed)
    keep = torch.ones(ACTOR_FEATURE_COUNT, dtype=torch.bool)
    keep[[LOAD_FEATURE_INDEX, CONTACT_FEATURE_INDEX]] = False
    assert torch.equal(translated_original[:, keep], original[:, keep])
    assert torch.equal(translated_changed[:, keep], changed[:, keep])
    assert torch.all(proxy_original[:, 0] >= 0.0)
    assert torch.all(proxy_original[:, 0] <= 2.0)
    assert set(proxy_original[:, 1].tolist()) <= {0.0, 1.0}


def test_forward_bounds_mean_residual_and_preserves_raw_logstd() -> None:
    module = _module(
        inference_only=False,
        fcnet_hiddens=[2],
        primary_split_adapter_residual_limits=[0.05, 0.13],
    )
    module.eval()
    with torch.no_grad():
        for parameter in module.pi.parameters():
            parameter.zero_()
        module.pi_encoder[0].weight[0, LOAD_FEATURE_INDEX] = 1.0
        module.pi_encoder[0].weight[1, CONTACT_FEATURE_INDEX] = 1.0
        module.pi[-1].weight[0, 0] = 1.0
        module.pi[-1].weight[1, 1] = 1.0
        module.pi[-1].bias[2:] = torch.tensor([-5.0, -4.0])
        module.primary_split_input_adapter[-1].weight.zero_()
        module.primary_split_input_adapter[-1].bias[:] = torch.tensor([10.0, 10.0])

    full_observation = torch.zeros((3, 84), dtype=torch.float32)
    actor_observation = full_observation[:, :ACTOR_FEATURE_COUNT]
    with torch.no_grad():
        raw_logits = module.pi(actor_observation)
        output = module.forward_inference({Columns.OBS: full_observation})[
            Columns.ACTION_DIST_INPUTS
        ]

    torch.testing.assert_close(
        output[:, :2],
        torch.tensor([[0.05, 0.13]]).expand(3, -1),
        rtol=0.0,
        atol=1.0e-7,
    )
    assert torch.equal(output[:, 2:], raw_logits[:, 2:])

    train_output = module.forward_train({Columns.OBS: full_observation})
    assert train_output[Columns.ACTION_DIST_INPUTS].shape == (3, 4)
    assert train_output[Columns.EMBEDDINGS].shape == (3, 2)
    assert module.compute_values({Columns.OBS: full_observation}).shape == (3,)


def test_reset_bypass_request_fails_instead_of_inferring_from_observation() -> None:
    assert "no causal" in RESET_BYPASS_UNAVAILABLE_REASON
    with pytest.raises(ValueError, match="no causal, unambiguous reset marker"):
        _module(primary_split_adapter_reset_bypass=True)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        (
            "primary_split_adapter_input_mean",
            [0.0] * 19,
            "exactly 20",
        ),
        (
            "primary_split_adapter_input_std",
            [1.0] * 19 + [0.0],
            "strictly positive",
        ),
        (
            "primary_split_adapter_residual_limits",
            [0.05, float("nan")],
            "finite",
        ),
    ],
)
def test_invalid_serialized_contract_fails_closed(field, value, message) -> None:
    with pytest.raises(ValueError, match=message):
        _module(**{field: value})


def test_wrap_checkpoint_preserves_actor_critic_and_save_reload() -> None:
    observation, action = _spaces()
    source = AsymmetricActorCriticTorchRLModule(
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
    generator = torch.Generator().manual_seed(17)
    observation_batch = torch.randn((5, 84), generator=generator)

    with tempfile.TemporaryDirectory() as raw_directory:
        root = Path(raw_directory)
        source_path = root / "source"
        candidate_path = root / "candidate"
        source.save_to_path(source_path)
        wrapped = build_adapter_module_from_checkpoint(
            source_path,
            input_mean=[0.0] * ADAPTER_INPUT_COUNT,
            input_std=[1.0] * ADAPTER_INPUT_COUNT,
        )

        source_state = source.get_state()
        wrapped_state = wrapped.get_state()
        for key, value in source_state.items():
            np.testing.assert_array_equal(_array(value), _array(wrapped_state[key]))
        assert "vf.weight" in wrapped_state
        assert "vf_encoder.0.weight" in wrapped_state

        wrapped.eval()
        with torch.no_grad():
            before_logits = wrapped.forward_inference({Columns.OBS: observation_batch})[
                Columns.ACTION_DIST_INPUTS
            ]
            before_values = wrapped.compute_values({Columns.OBS: observation_batch})
        wrapped.save_to_path(candidate_path)
        reloaded = RLModule.from_checkpoint(candidate_path)
        assert isinstance(reloaded, PrimarySplitInputAdapterTorchRLModule)
        reloaded.eval()
        with torch.no_grad():
            after_logits = reloaded.forward_inference({Columns.OBS: observation_batch})[
                Columns.ACTION_DIST_INPUTS
            ]
            after_values = reloaded.compute_values({Columns.OBS: observation_batch})
        assert torch.equal(before_logits, after_logits)
        assert torch.equal(before_values, after_values)
        for key, value in wrapped.get_state().items():
            np.testing.assert_array_equal(
                _array(value),
                _array(reloaded.get_state()[key]),
            )


def test_official_h0_inference_checkpoint_wraps_with_exact_base_and_logstd() -> None:
    checkpoint = (
        REPO_ROOT
        / "validation"
        / "critic_warmup"
        / "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry"
        / "rl_module_last"
    )
    source = RLModule.from_checkpoint(checkpoint)
    module = build_adapter_module_from_checkpoint(
        checkpoint,
        input_mean=[0.0] * ADAPTER_INPUT_COUNT,
        input_std=[1.0] * ADAPTER_INPUT_COUNT,
    )
    assert module.inference_only
    source_state = source.get_state()
    module_state = module.get_state()
    for key, value in source_state.items():
        np.testing.assert_array_equal(_array(value), _array(module_state[key]))

    observations = torch.zeros((2, ACTOR_FEATURE_COUNT), dtype=torch.float32)
    source.eval()
    module.eval()
    with torch.no_grad():
        source_logits = source.forward_inference({Columns.OBS: observations})[
            Columns.ACTION_DIST_INPUTS
        ]
        adapter_logits = module.forward_inference({Columns.OBS: observations})[
            Columns.ACTION_DIST_INPUTS
        ]
    assert torch.equal(source_logits[:, 2:], adapter_logits[:, 2:])

    with tempfile.TemporaryDirectory() as raw_directory:
        checkpoint_copy = Path(raw_directory) / "adapter"
        module.save_to_path(checkpoint_copy)
        reloaded = RLModule.from_checkpoint(checkpoint_copy)
        reloaded.eval()
        with torch.no_grad():
            reloaded_logits = reloaded.forward_inference({Columns.OBS: observations})[
                Columns.ACTION_DIST_INPUTS
            ]
        assert torch.equal(adapter_logits, reloaded_logits)


def test_adapter_only_fit_keeps_base_and_logstd_bit_exact_then_allows_ppo() -> None:
    observation, action = _spaces()
    source = AsymmetricActorCriticTorchRLModule(
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
    with tempfile.TemporaryDirectory() as raw_directory:
        source_path = Path(raw_directory) / "source"
        source.save_to_path(source_path)
        module = build_adapter_module_from_checkpoint(
            source_path,
            input_mean=[0.0] * ADAPTER_INPUT_COUNT,
            input_std=[1.0] * ADAPTER_INPUT_COUNT,
        )

    keys = base_state_keys(module)
    base_before = _state_subset(module, keys)
    adapter_before = {
        name: parameter.detach().clone()
        for name, parameter in module.primary_split_input_adapter.named_parameters()
    }
    generator = torch.Generator().manual_seed(31)
    observations = torch.randn((16, 84), generator=generator)
    module.eval()
    with torch.no_grad():
        logstd_before = module.forward_inference({Columns.OBS: observations})[
            Columns.ACTION_DIST_INPUTS
        ][:, 2:].clone()

    trainable = module.prepare_adapter_fit()
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
        not torch.equal(
            adapter_before[name],
            parameter.detach(),
        )
        for name, parameter in module.primary_split_input_adapter.named_parameters()
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
