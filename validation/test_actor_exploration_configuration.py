"""Focused tests for variance-only warm-start actor configuration."""

from __future__ import annotations

import math
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

BASELINE_DIR = (
    Path(__file__).resolve().parents[1] / "Trajectory Generator" / "baseline_MLP"
)
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

import configure_actor_exploration


class ActorExplorationConfigurationTests(unittest.TestCase):
    def test_explicit_feature_manifest_must_match_actor_input_width(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "actor_feature_manifest.json"
            path.write_text(
                json.dumps(
                    {
                        "actor_feature_count": 2,
                        "actor_feature_names": ["feature_a", "feature_b"],
                    }
                ),
                encoding="utf-8",
            )

            manifest = configure_actor_exploration._load_actor_feature_manifest(
                path,
                expected_feature_count=2,
            )
            self.assertEqual(manifest["actor_feature_names"], ["feature_a", "feature_b"])
            with self.assertRaisesRegex(ValueError, "input width"):
                configure_actor_exploration._load_actor_feature_manifest(
                    path,
                    expected_feature_count=3,
                )

    def test_constant_std_changes_only_logstd_output_rows(self) -> None:
        rng = np.random.default_rng(123)
        state = {
            "pi.0.0.weight": rng.normal(size=(3, 2)).astype(np.float32),
            "pi.0.0.bias": rng.normal(size=3).astype(np.float32),
            "pi.1.weight": rng.normal(size=(4, 3)).astype(np.float32),
            "pi.1.bias": rng.normal(size=4).astype(np.float32),
            "unrelated": rng.normal(size=2).astype(np.float32),
        }
        configured, report = configure_actor_exploration.configure_constant_std(
            state,
            sigma=0.05,
            action_dim=2,
        )

        np.testing.assert_array_equal(
            configured["pi.1.weight"][:2], state["pi.1.weight"][:2]
        )
        np.testing.assert_array_equal(
            configured["pi.1.bias"][:2], state["pi.1.bias"][:2]
        )
        np.testing.assert_array_equal(configured["unrelated"], state["unrelated"])
        np.testing.assert_array_equal(configured["pi.1.weight"][2:], 0.0)
        np.testing.assert_allclose(configured["pi.1.bias"][2:], math.log(0.05))
        self.assertEqual(report["sigma"], [0.05, 0.05])
        self.assertTrue(report["mean_parameters_exact"])
        self.assertEqual(report["configured_logstd_weight_l2"], 0.0)

    def test_per_action_sigma_is_supported(self) -> None:
        state = {
            "pi.1.weight": np.ones((4, 3), dtype=np.float32),
            "pi.1.bias": np.ones(4, dtype=np.float32),
        }
        configured, report = configure_actor_exploration.configure_constant_std(
            state,
            sigma=(0.02, 0.015),
            action_dim=2,
        )
        np.testing.assert_allclose(
            configured["pi.1.bias"][2:],
            np.log([0.02, 0.015]),
        )
        self.assertEqual(report["sigma"], [0.02, 0.015])

    def test_single_cli_style_sigma_is_broadcast(self) -> None:
        state = {
            "pi.1.weight": np.ones((4, 3), dtype=np.float32),
            "pi.1.bias": np.ones(4, dtype=np.float32),
        }
        configured, report = configure_actor_exploration.configure_constant_std(
            state,
            sigma=[0.03],
            action_dim=2,
        )
        np.testing.assert_allclose(configured["pi.1.bias"][2:], math.log(0.03))
        self.assertEqual(report["sigma"], [0.03, 0.03])

    def test_invalid_sigma_is_rejected(self) -> None:
        state = {
            "pi.1.weight": np.zeros((4, 3), dtype=np.float32),
            "pi.1.bias": np.zeros(4, dtype=np.float32),
        }
        with self.assertRaisesRegex(ValueError, "sigma"):
            configure_actor_exploration.configure_constant_std(
                state,
                sigma=0.0,
                action_dim=2,
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
