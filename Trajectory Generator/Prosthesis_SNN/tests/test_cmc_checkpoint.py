"""Prevent legacy SNN actions from being interpreted as absolute targets."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from prosthesis_snn import ProsthesisReferenceSNN, ReferenceGenerator, SNNConfig
from prosthesis_snn.cmc_checkpoint import (
    require_absolute_agent, require_absolute_reference, write_agent_contract,
)
from prosthesis_snn.training.checkpoint import save_reference_checkpoint


@pytest.mark.parametrize("mode", ["absolute", "delta", None])
def test_reference_checkpoint_keeps_action_provenance(tmp_path, mode):
    cfg = SNNConfig(hidden_size=8)
    path = tmp_path / "reference.pt"
    save_reference_checkpoint(
        path, ProsthesisReferenceSNN(cfg),
        metadata={"env_config": {"action_mode": mode}},
    )
    generator = ReferenceGenerator.from_checkpoint(path, device="cpu")
    if mode == "absolute":
        require_absolute_reference(generator)
    else:
        with pytest.raises(ValueError, match="absolute"):
            require_absolute_reference(generator)


def test_agent_resume_requires_saved_absolute_contract(tmp_path):
    path = tmp_path / "agent.pt"
    with pytest.raises(ValueError, match="absolute"):
        require_absolute_agent(path)
    write_agent_contract(path)
    require_absolute_agent(path)
