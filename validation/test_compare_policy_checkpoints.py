from __future__ import annotations

import json
import pickle
import tempfile
import unittest
from pathlib import Path

import numpy as np

from validation.audit_policy_milestones import (
    DEFAULT_DEVELOPMENT_TRACE_SPECS,
    DEFAULT_DEVELOPMENT_TRACES,
    _completed_retained_milestones,
    _development_trace_descriptor,
    _validated_development_traces,
    audit_policy_milestones,
)
from validation.compare_policy_checkpoints import (
    _actor_digest,
    _fixed_observation_metrics,
    _load_state,
    _logstd_head_comparison,
    _parameter_comparison,
)


ROOT = Path(__file__).resolve().parents[1]
CANONICAL_H0_RL_MODULE = (
    ROOT
    / "validation/critic_warmup/"
    "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last"
)


def _state() -> dict[str, np.ndarray]:
    return {
        "pi.0.0.weight": np.asarray([[0.2, -0.1, 0.3], [-0.4, 0.2, 0.1]]),
        "pi.0.0.bias": np.asarray([0.0, 0.1]),
        "pi.0.2.weight": np.asarray([[0.3, -0.2], [0.1, 0.4]]),
        "pi.0.2.bias": np.asarray([0.0, -0.1]),
        "pi.1.weight": np.asarray(
            [[0.2, 0.1], [-0.1, 0.3], [0.0, 0.0], [0.0, 0.0]]
        ),
        "pi.1.bias": np.asarray([0.0, 0.0, np.log(0.005), np.log(0.005)]),
    }


class ComparePolicyCheckpointsTest(unittest.TestCase):
    def test_identical_states_have_zero_shift(self) -> None:
        state = _state()
        observations = np.asarray([[0.1, 0.2, -0.1], [0.0, -0.2, 0.3]])
        metrics = _fixed_observation_metrics(state, state, observations, 2)

        self.assertEqual(metrics["mean_delta_abs_max"], 0.0)
        self.assertTrue(metrics["logstd_bit_exact"])
        self.assertEqual(metrics["logstd_delta_abs_max"], 0.0)
        self.assertAlmostEqual(
            metrics["empirical_kl_reference_to_candidate_mean"], 0.0
        )
        self.assertTrue(_parameter_comparison(state, state)["exact"])

    def test_mean_shift_produces_expected_equal_variance_kl(self) -> None:
        reference = _state()
        candidate = {key: value.copy() for key, value in reference.items()}
        candidate["pi.1.bias"][:2] += np.asarray([0.001, -0.002])
        observations = np.asarray([[0.1, 0.2, -0.1], [0.0, -0.2, 0.3]])

        metrics = _fixed_observation_metrics(
            reference, candidate, observations, action_dim=2
        )
        expected_kl = (0.001**2 + 0.002**2) / (2.0 * 0.005**2)

        self.assertAlmostEqual(
            metrics["empirical_kl_reference_to_candidate_mean"], expected_kl
        )
        self.assertEqual(metrics["logstd_delta_abs_max"], 0.0)
        self.assertAlmostEqual(
            _parameter_comparison(reference, candidate)["max_abs_diff"], 0.002
        )
        self.assertTrue(
            _logstd_head_comparison(reference, candidate, action_dim=2)[
                "bit_exact"
            ]
        )

    def test_logstd_comparison_is_bit_exact_without_tolerance(self) -> None:
        reference = _state()
        candidate = {key: value.copy() for key, value in reference.items()}
        candidate["pi.1.bias"][2] = np.nextafter(
            candidate["pi.1.bias"][2], np.inf
        )
        observations = np.asarray([[0.1, 0.2, -0.1]])

        metrics = _fixed_observation_metrics(
            reference, candidate, observations, action_dim=2
        )
        head = _logstd_head_comparison(reference, candidate, action_dim=2)

        self.assertFalse(metrics["logstd_bit_exact"])
        self.assertFalse(head["bit_exact"])
        self.assertGreater(head["max_abs_diff"], 0.0)

    @staticmethod
    def _write_module(path: Path, state: dict[str, np.ndarray]) -> None:
        path.mkdir(parents=True)
        with (path / "module_state.pkl").open("wb") as handle:
            pickle.dump(state, handle)

    @staticmethod
    def _write_trace(path: Path, *, action_seed: int = 123) -> None:
        path.parent.mkdir(parents=True)
        rows = [
            {
                "actor_observation_vector_before": [0.1, 0.2, -0.1],
                "raw_policy_action": [0.0, 0.0],
            },
            {
                "actor_observation_vector_before": [0.0, -0.2, 0.3],
                "raw_policy_action": [0.0, 0.0],
            },
        ]
        path.write_text(json.dumps(rows), encoding="utf-8")
        (path.parent / "rollout_summary.json").write_text(
            json.dumps(
                {
                    "ok": True,
                    "action_seed": action_seed,
                    "action_selection": "stochastic",
                    "checkpoint": "synthetic-reference",
                }
            ),
            encoding="utf-8",
        )

    def test_post_training_audit_covers_every_registered_milestone(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            reference = root / "h0"
            self._write_module(reference, _state())
            trace = root / "development_seed123" / "rollout_policy_trace.json"
            self._write_trace(trace)
            run = root / "run"
            run.mkdir()

            milestone_paths = []
            for iteration in (2, 3):
                milestone = run / f"milestone_iteration_{iteration:06d}"
                candidate = {key: value.copy() for key, value in _state().items()}
                candidate["pi.1.bias"][:2] += iteration * 1.0e-4
                if iteration == 3:
                    candidate["pi.1.bias"][2] = np.nextafter(
                        candidate["pi.1.bias"][2], np.inf
                    )
                self._write_module(milestone / "rl_module_last", candidate)
                (milestone / "rl_module_last_meta.json").write_text(
                    json.dumps(
                        {
                            "logical_iteration": iteration,
                            "rl_module": str(milestone / "rl_module_last"),
                        }
                    ),
                    encoding="utf-8",
                )
                milestone_paths.append(str(milestone))
            (run / "summary.json").write_text(
                json.dumps(
                    {
                        "ok": True,
                        "stop_reason": "completed",
                        "iteration_checkpoint_retention": {
                            "enabled": True,
                            "milestones": milestone_paths,
                        },
                    }
                ),
                encoding="utf-8",
            )

            report = audit_policy_milestones(reference, run, [trace])

        self.assertEqual(report["milestone_count"], 2)
        self.assertEqual(
            [row["logical_iteration"] for row in report["milestones"]],
            [2, 3],
        )
        self.assertEqual(report["milestones"][0]["status"], "PASS")
        self.assertGreater(report["milestones"][0]["action_mean_rmse"], 0.0)
        self.assertGreater(
            report["milestones"][0]["kl_reference_to_candidate_mean"], 0.0
        )
        self.assertEqual(report["milestones"][1]["status"], "FAIL")
        self.assertFalse(report["milestones"][1]["logstd_bit_exact"])
        self.assertEqual(report["failed_logical_iterations"], [3])
        self.assertFalse(report["ok"])

    def test_audit_rejects_heldout_trace_and_incomplete_run(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            heldout = root / "stochastic_seed_126" / "rollout_policy_trace.json"
            self._write_trace(heldout)
            with self.assertRaisesRegex(ValueError, "held-out seeds 126-128"):
                _validated_development_traces([heldout])

            run = root / "run"
            run.mkdir()
            (run / "summary.json").write_text(
                json.dumps({"ok": False, "stop_reason": "error"}),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "active or incomplete"):
                _completed_retained_milestones(run)

    def test_audit_rejects_heldout_seed_from_innocuous_summary(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            disguised = root / "ordinary_development" / "rollout_policy_trace.json"
            self._write_trace(disguised, action_seed=126)
            with self.assertRaisesRegex(ValueError, "records action_seed=126"):
                _validated_development_traces([disguised])

            allowed = root / "explicit_development" / "rollout_policy_trace.json"
            self._write_trace(allowed, action_seed=124)
            self.assertEqual(
                _validated_development_traces([allowed]),
                [allowed.resolve()],
            )

            missing = root / "missing_provenance" / "rollout_policy_trace.json"
            missing.parent.mkdir()
            missing.write_text("[]", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "provenance is missing"):
                _validated_development_traces([missing])

    def test_default_protocol_uses_h0_three_start_and_seed123_only(self) -> None:
        self.assertEqual(
            [spec["condition"] for spec in DEFAULT_DEVELOPMENT_TRACE_SPECS],
            [
                "deterministic_minus020",
                "deterministic_nominal",
                "deterministic_plus020",
                "stochastic_plus020_seed123",
            ],
        )
        rendered = [Path(path).as_posix() for path in DEFAULT_DEVELOPMENT_TRACES]
        self.assertEqual(len(rendered), 4)
        self.assertTrue(any("seed123" in path for path in rendered))
        self.assertFalse(any("seed124" in path for path in rendered))
        self.assertFalse(any("seed125" in path for path in rendered))
        self.assertEqual(
            _validated_development_traces(DEFAULT_DEVELOPMENT_TRACES),
            [Path(path).resolve() for path in DEFAULT_DEVELOPMENT_TRACES],
        )

    def test_default_trace_sources_are_bit_exact_to_canonical_h0(self) -> None:
        h0_digest = _actor_digest(_load_state(CANONICAL_H0_RL_MODULE))
        descriptors = [
            _development_trace_descriptor(Path(path).resolve(), h0_digest)
            for path in DEFAULT_DEVELOPMENT_TRACES
        ]

        self.assertEqual(
            [item["protocol_role"] for item in descriptors],
            [spec["condition"] for spec in DEFAULT_DEVELOPMENT_TRACE_SPECS],
        )
        self.assertTrue(
            all(
                item["source_actor_bit_exact_to_reference_h0"]
                for item in descriptors
            )
        )
        self.assertTrue(
            all(item["source_actor_digest"] == h0_digest for item in descriptors)
        )


if __name__ == "__main__":
    unittest.main()
