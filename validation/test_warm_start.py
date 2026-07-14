"""Focused tests for the actor-only imitation warm-start transplant."""

from __future__ import annotations

import pickle
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_DIR = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

import warm_start


def _actor_state(input_width: int, *, hidden: int = 8) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(1000 + input_width)
    first = rng.normal(size=(hidden, input_width)).astype(np.float32)
    second = rng.normal(size=(hidden, hidden)).astype(np.float32)
    return {
        "pi_encoder.0.weight": first.copy(),
        "pi_encoder.0.bias": rng.normal(size=hidden).astype(np.float32),
        "pi_encoder.2.weight": second.copy(),
        "pi_encoder.2.bias": rng.normal(size=hidden).astype(np.float32),
        "pi.0.0.weight": first.copy(),
        "pi.0.0.bias": rng.normal(size=hidden).astype(np.float32),
        "pi.0.2.weight": second.copy(),
        "pi.0.2.bias": rng.normal(size=hidden).astype(np.float32),
        "pi.1.weight": rng.normal(size=(4, hidden)).astype(np.float32),
        "pi.1.bias": rng.normal(size=4).astype(np.float32),
    }


class WarmStartTransplantTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.source_dir = Path(self.temp_dir.name) / "rl_module_best"
        self.source_dir.mkdir()
        self.source_state = _actor_state(
            len(warm_start.ASYM100_GRF_PENALTY_LOWERED_ACTOR_FEATURES)
        )
        with (self.source_dir / "module_state.pkl").open("wb") as handle:
            pickle.dump(self.source_state, handle)

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def test_actor_copy_zeroes_new_and_disabled_features(self) -> None:
        source_features = warm_start.ASYM100_GRF_PENALTY_LOWERED_ACTOR_FEATURES
        target_features = (*source_features, "phase_fsm_wait_hs")
        target_state = _actor_state(len(target_features))
        target_state["vf.weight"] = np.arange(8, dtype=np.float32).reshape(1, 8)
        target_state["vf.bias"] = np.asarray([3.0], dtype=np.float32)
        critic_before = {
            key: value.copy()
            for key, value in target_state.items()
            if key.startswith("vf.")
        }

        transplanted, report = warm_start.transplant_actor_state(
            target_state=target_state,
            target_actor_feature_names=target_features,
            source_checkpoint=self.source_dir,
            zero_target_features=warm_start.DISABLED_GAIT_CLOCK_FEATURES,
        )

        source_index = {name: i for i, name in enumerate(source_features)}
        target_index = {name: i for i, name in enumerate(target_features)}
        copied = set(report["copied_features"])
        for key in warm_start._FIRST_LAYER_WEIGHT_KEYS:
            for name in copied:
                np.testing.assert_array_equal(
                    transplanted[key][:, target_index[name]],
                    self.source_state[key][:, source_index[name]],
                )
            for name in (
                *warm_start.DISABLED_GAIT_CLOCK_FEATURES,
                "phase_fsm_wait_hs",
            ):
                np.testing.assert_array_equal(
                    transplanted[key][:, target_index[name]],
                    0.0,
                )
        for key in warm_start._DIRECT_ACTOR_KEYS:
            np.testing.assert_array_equal(transplanted[key], self.source_state[key])
        for key, before in critic_before.items():
            np.testing.assert_array_equal(transplanted[key], before)

        self.assertTrue(report["source_state_is_actor_only"])
        self.assertTrue(report["target_non_actor_state_unchanged"])
        self.assertEqual(
            report["shared_features_zeroed"],
            list(warm_start.DISABLED_GAIT_CLOCK_FEATURES),
        )

    def test_compare_actor_states_detects_single_value_change(self) -> None:
        left = self.source_state
        right = {key: value.copy() for key, value in left.items()}
        self.assertTrue(warm_start.compare_actor_states(left, right)["exact"])
        right["pi.1.bias"][0] += 0.5
        comparison = warm_start.compare_actor_states(left, right)
        self.assertFalse(comparison["exact"])
        self.assertAlmostEqual(comparison["max_abs_diff"], 0.5)

    def test_compare_non_actor_states_requires_unchanged_critic(self) -> None:
        before = {
            **self.source_state,
            "vf.weight": np.arange(8, dtype=np.float32).reshape(1, 8),
            "vf.bias": np.asarray([3.0], dtype=np.float32),
        }
        after = {key: value.copy() for key, value in before.items()}
        comparison = warm_start.compare_non_actor_states(before, after)
        self.assertTrue(comparison["exact"])
        self.assertEqual(comparison["keys"], ["vf.bias", "vf.weight"])

        after["vf.bias"][0] += 0.25
        comparison = warm_start.compare_non_actor_states(before, after)
        self.assertFalse(comparison["exact"])
        self.assertAlmostEqual(comparison["max_abs_diff"], 0.25)

    def test_unknown_forced_zero_feature_is_rejected(self) -> None:
        source_features = warm_start.ASYM100_GRF_PENALTY_LOWERED_ACTOR_FEATURES
        with self.assertRaisesRegex(ValueError, "absent from the target"):
            warm_start.transplant_actor_state(
                target_state=_actor_state(len(source_features)),
                target_actor_feature_names=source_features,
                source_checkpoint=self.source_dir,
                zero_target_features=("not_a_feature",),
            )

    def test_explicit_manifest_supports_same_schema_target_actor(self) -> None:
        source_features = tuple(f"target_feature_{index}" for index in range(39))
        source_state = _actor_state(len(source_features))
        with (self.source_dir / "module_state.pkl").open("wb") as handle:
            pickle.dump(source_state, handle)
        manifest_path = Path(self.temp_dir.name) / "actor_feature_manifest.json"
        manifest_path.write_text(
            json.dumps(
                {
                    "actor_feature_count": len(source_features),
                    "actor_feature_names": list(source_features),
                    "actor_digest": warm_start.actor_state_digest(source_state),
                }
            ),
            encoding="utf-8",
        )

        target_state = _actor_state(len(source_features))
        target_state["vf.bias"] = np.asarray([7.0], dtype=np.float32)
        transplanted, report = warm_start.transplant_actor_state(
            target_state=target_state,
            target_actor_feature_names=source_features,
            source_checkpoint=self.source_dir,
            source_actor_feature_manifest=manifest_path,
        )

        self.assertTrue(
            warm_start.compare_actor_states(source_state, transplanted)["exact"]
        )
        np.testing.assert_array_equal(transplanted["vf.bias"], [7.0])
        self.assertEqual(
            report["source_actor_feature_manifest"]["resolution"],
            "explicit_manifest",
        )

    def test_manifest_digest_mismatch_is_rejected(self) -> None:
        source_features = warm_start.ASYM100_GRF_PENALTY_LOWERED_ACTOR_FEATURES
        manifest_path = Path(self.temp_dir.name) / "actor_feature_manifest.json"
        manifest_path.write_text(
            json.dumps(
                {
                    "actor_feature_names": list(source_features),
                    "actor_digest": "0" * 64,
                }
            ),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(ValueError, "digest does not match"):
            warm_start.transplant_actor_state(
                target_state=_actor_state(len(source_features)),
                target_actor_feature_names=source_features,
                source_checkpoint=self.source_dir,
                source_actor_feature_manifest=manifest_path,
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
