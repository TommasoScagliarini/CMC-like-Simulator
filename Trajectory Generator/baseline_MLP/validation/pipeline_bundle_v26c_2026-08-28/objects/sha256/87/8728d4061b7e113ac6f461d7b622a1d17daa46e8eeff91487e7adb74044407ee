"""Focused gradient tests for the asymmetric Gaussian actor."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

BASELINE_DIR = (
    Path(__file__).resolve().parents[1] / "Trajectory Generator" / "baseline_MLP"
)
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

import torch

from asymmetric_rl_module import _detach_actor_gradient, _detach_logstd_gradient


class AsymmetricRLModuleTests(unittest.TestCase):
    def test_freeze_logstd_preserves_values_and_blocks_only_its_gradient(self) -> None:
        logits = torch.tensor([[1.0, 2.0, -3.0, -4.0]], requires_grad=True)
        output = _detach_logstd_gradient(logits, action_dim=2, freeze=True)

        torch.testing.assert_close(output, logits)
        output.sum().backward()
        torch.testing.assert_close(
            logits.grad,
            torch.tensor([[1.0, 1.0, 0.0, 0.0]]),
        )

    def test_disabled_freeze_preserves_all_gradients(self) -> None:
        logits = torch.tensor([[1.0, 2.0, -3.0, -4.0]], requires_grad=True)
        output = _detach_logstd_gradient(logits, action_dim=2, freeze=False)

        self.assertIs(output, logits)
        output.sum().backward()
        torch.testing.assert_close(logits.grad, torch.ones_like(logits))

    def test_freeze_actor_preserves_values_and_blocks_every_gradient(self) -> None:
        logits = torch.tensor([[1.0, 2.0, -3.0, -4.0]], requires_grad=True)
        output = _detach_actor_gradient(logits, freeze=True)

        torch.testing.assert_close(output, logits)
        self.assertFalse(output.requires_grad)

    def test_disabled_actor_freeze_keeps_gradient_path(self) -> None:
        logits = torch.tensor([[1.0, 2.0]], requires_grad=True)
        output = _detach_actor_gradient(logits, freeze=False)

        self.assertIs(output, logits)


if __name__ == "__main__":
    unittest.main(verbosity=2)
