"""Focused checks for rollout checkpoint/observation compatibility guards."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


BASELINE_DIR = (
    Path(__file__).resolve().parents[1] / "Trajectory Generator" / "baseline_MLP"
)
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

import rollout_eval


class _Module:
    _n_actor = 31
    _n_full = 80


class RolloutObservationContractTests(unittest.TestCase):
    def test_matching_contract_is_accepted(self) -> None:
        rollout_eval._validate_module_observation_contract(
            _Module(),
            tuple(f"actor_{index}" for index in range(31)),
            tuple(f"obs_{index}" for index in range(80)),
        )

    def test_schema_drift_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "Prefix slicing"):
            rollout_eval._validate_module_observation_contract(
                _Module(),
                tuple(f"actor_{index}" for index in range(43)),
                tuple(f"obs_{index}" for index in range(88)),
            )

    def test_action_selection_cli_is_explicit(self) -> None:
        source = (BASELINE_DIR / "rollout_eval.py").read_text(encoding="utf-8")
        self.assertIn(
            'choices=("deterministic", "stochastic", "stochastic_held")',
            source,
        )
        self.assertIn("module.forward_exploration", source)
        self.assertIn("get_exploration_action_dist_cls", source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
