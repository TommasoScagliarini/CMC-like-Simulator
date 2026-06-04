"""Smoke tests for the portable prosthesis SNN scaffold."""

from __future__ import annotations

import pathlib
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import torch

from prosthesis_snn import (
    DictReferenceProvider,
    HybridReferenceProvider,
    ProsthesisReferenceSNN,
    ReferenceGenerator,
    SNNConfig,
    SNNProsthesisReferenceProvider,
)
from prosthesis_snn.training import (
    LinearEntropyDecay,
    PPO_SNN,
    PPO_SNN_DEFAULT_CONFIG,
    ProsthesisSNNActorCritic,
    REFERENCE_CHECKPOINT_FORMAT,
    build_reference_checkpoint,
    save_reference_checkpoint,
)


def test_model_shapes() -> None:
    cfg = SNNConfig(hidden_size=16)
    model = ProsthesisReferenceSNN(cfg)

    expected_size = len(cfg.output_coords) * len(cfg.output_channels)
    assert cfg.output_size == expected_size

    out_one, mem_one = model(torch.zeros(1, cfg.input_size))
    assert out_one.shape == (1, expected_size)
    assert len(mem_one) == cfg.num_layers + 2

    out_batch, mem_batch = model(torch.zeros(4, cfg.input_size))
    assert out_batch.shape == (4, expected_size)
    assert mem_batch[0].shape[0] == 4


def test_generator_and_hybrid_provider() -> None:
    cfg = SNNConfig(hidden_size=16)
    model = ProsthesisReferenceSNN(cfg)
    generator = ReferenceGenerator(model, cfg=cfg, device="cpu")

    predicted = generator.predict({"phase_sin": 0.0, "phase_cos": 1.0})
    assert set(predicted) == set(cfg.output_channels)
    for channel in cfg.output_channels:
        assert set(predicted[channel]) == set(cfg.output_coords)

    base = DictReferenceProvider(
        q_ref={
            "hip_flexion_r": 0.25,
            "pros_knee_angle": 99.0,
            "pros_ankle_angle": 99.0,
        },
        qdot_ref={
            "hip_flexion_r": 0.0,
            "pros_knee_angle": 99.0,
            "pros_ankle_angle": 99.0,
        },
        qddot_ref={
            "hip_flexion_r": 0.0,
            "pros_knee_angle": 99.0,
            "pros_ankle_angle": 99.0,
        },
    )
    prosthesis = SNNProsthesisReferenceProvider(
        generator,
        feature_builder=lambda t, state: {"phase_sin": 0.0, "phase_cos": 1.0},
    )
    hybrid = HybridReferenceProvider(base, prosthesis)
    q_ref, qdot_ref, qddot_ref = hybrid.get(0.0)

    assert q_ref["hip_flexion_r"] == 0.25
    for coord in ("pros_knee_angle", "pros_ankle_angle"):
        assert q_ref[coord] != 99.0, f"q for {coord} was not overridden"
        assert qdot_ref[coord] != 99.0, f"qdot for {coord} was not overridden"
        assert qddot_ref[coord] != 99.0, f"qddot for {coord} was not overridden"

    assert qdot_ref["hip_flexion_r"] == 0.0
    assert qddot_ref["hip_flexion_r"] == 0.0


def test_training_helpers_import_without_skrl() -> None:
    schedule = LinearEntropyDecay(start=0.01, end=0.0, total_steps=100)
    assert schedule.get(0) == 0.01
    assert schedule.get(100) == 0.0
    assert PPO_SNN_DEFAULT_CONFIG["rollouts"] > 0


def test_actor_critic_contract() -> None:
    try:
        from gymnasium import spaces
    except ModuleNotFoundError:
        return

    cfg = SNNConfig(hidden_size=16)
    observation_space = spaces.Box(
        low=-1.0,
        high=1.0,
        shape=(cfg.input_size,),
        dtype="float32",
    )
    action_space = spaces.Box(
        low=-1.0,
        high=1.0,
        shape=(cfg.output_size,),
        dtype="float32",
    )
    try:
        model = ProsthesisSNNActorCritic(
            observation_space=observation_space,
            action_space=action_space,
            device="cpu",
            cfg=cfg,
            num_envs=2,
            sequence_length=4,
        )
    except ModuleNotFoundError:
        return

    rnn_sizes = model.get_specification()["rnn"]["sizes"]
    rnn = [torch.zeros(size) for size in rnn_sizes]
    states = torch.zeros(2, cfg.input_size)

    actions, log_prob, policy_outputs = model.act(
        {"states": states, "rnn": rnn},
        role="policy",
    )
    values, _, value_outputs = model.act(
        {"states": states, "rnn": rnn},
        role="value",
    )

    assert actions.shape == (2, cfg.output_size)
    assert log_prob is not None and log_prob.shape == (2, 1)
    assert values.shape == (2, 1)
    assert len(policy_outputs["rnn"]) == len(rnn_sizes)
    assert len(value_outputs["rnn"]) == len(rnn_sizes)
    assert model.get_entropy(role="policy").shape == (2, 1)

    reference_model = ProsthesisReferenceSNN(cfg)
    reference_model.load_state_dict(model.reference_state_dict())

    checkpoint = build_reference_checkpoint(
        model,
        metadata={"test": "actor_critic_contract"},
    )
    assert checkpoint["format"] == REFERENCE_CHECKPOINT_FORMAT
    assert checkpoint["config"]["output_channels"] == list(cfg.output_channels)
    assert "value_lif.fc.weight" not in checkpoint["model_state_dict"]
    assert "log_std_parameter" not in checkpoint["model_state_dict"]

    with tempfile.TemporaryDirectory() as tmp_dir:
        checkpoint_path = pathlib.Path(tmp_dir) / "reference.pt"
        save_reference_checkpoint(
            checkpoint_path,
            model,
            output_transform="identity",
            metadata={"test": "reload"},
        )
        reloaded = ReferenceGenerator.from_checkpoint(checkpoint_path, device="cpu")
        predicted = reloaded.predict({"phase_sin": 0.0, "phase_cos": 1.0})
        flat_values = [
            value
            for channel_values in predicted.values()
            for value in channel_values.values()
        ]
        assert len(flat_values) == cfg.output_size
        assert torch.isfinite(torch.tensor(flat_values)).all()

    try:
        from skrl.memories.torch import RandomMemory
    except ModuleNotFoundError:
        return

    memory = RandomMemory(memory_size=4, num_envs=2, device="cpu")
    agent = PPO_SNN(
        models={"policy": model, "value": model},
        memory=memory,
        observation_space=observation_space,
        action_space=action_space,
        device="cpu",
        cfg={"rollouts": 4, "learning_starts": 100},
        num_envs=2,
    )
    agent.init()
    agent_actions, agent_outputs = agent.act(
        states,
        timestep=0,
        timesteps=10,
    )
    assert agent_actions.shape == (2, cfg.output_size)
    assert agent_outputs["log_prob"].shape == (2, 1)
    assert len(agent_outputs["rnn"]) == len(rnn_sizes)

    agent.record_transition(
        observations=states,
        states=None,
        actions=agent_actions,
        rewards=torch.zeros(2, 1),
        next_observations=states,
        next_states=None,
        terminated=torch.zeros(2, 1, dtype=torch.bool),
        truncated=torch.zeros(2, 1, dtype=torch.bool),
        infos={},
        timestep=0,
        timesteps=10,
    )


def main() -> None:
    test_model_shapes()
    test_generator_and_hybrid_provider()
    test_training_helpers_import_without_skrl()
    test_actor_critic_contract()
    print("smoke tests passed")


if __name__ == "__main__":
    main()
