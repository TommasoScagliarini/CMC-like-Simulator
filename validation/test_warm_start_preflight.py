"""Tests for warm-start stochastic preflight classification."""

from __future__ import annotations

import unittest

from validation.summarize_warm_start_preflight import (
    classify_relative_h1_gate,
    classify_stochastic_preflight,
)


def _probe(*, reached_to: bool, penetration: bool, clipping: float) -> dict:
    return {
        "ok": True,
        "action_selection": "stochastic",
        "phase_valid_to_count": int(reached_to),
        "phase_valid_cycle_count": 0,
        "end_reason": "grf_penetration" if penetration else "episode_duration",
        "action_clipped_fraction": clipping,
    }


class WarmStartPreflightTests(unittest.TestCase):
    def test_three_early_penetrations_fail(self) -> None:
        report = classify_stochastic_preflight(
            [_probe(reached_to=False, penetration=True, clipping=0.3)] * 3
        )
        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(report["valid_to_fraction"], 0.0)
        self.assertEqual(report["penetration_termination_fraction"], 1.0)

    def test_two_useful_low_clip_probes_pass(self) -> None:
        report = classify_stochastic_preflight(
            [
                _probe(reached_to=True, penetration=False, clipping=0.05),
                _probe(reached_to=True, penetration=False, clipping=0.08),
                _probe(reached_to=False, penetration=True, clipping=0.09),
            ]
        )
        self.assertEqual(report["status"], "PASS")

    def test_relative_gate_requires_cycle_and_survival(self) -> None:
        probes = [
            {
                **_probe(reached_to=True, penetration=True, clipping=0.0),
                "steps": steps,
                "episode_return": -10.0,
                "grf_penetration_max_m": 0.025,
            }
            for steps in (205, 205, 216)
        ]
        report = classify_relative_h1_gate(
            probes,
            deterministic_steps=356,
            deterministic_unchanged=True,
        )
        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(report["median_steps"], 205.0)
        self.assertFalse(report["checks"]["at_least_one_valid_cycle"])
        self.assertFalse(report["checks"]["median_survival"])

    def test_relative_gate_accepts_informative_small_noise_probes(self) -> None:
        probes = []
        for seed, steps, cycle in ((123, 220, 0), (124, 230, 1), (125, 240, 0)):
            probes.append(
                {
                    **_probe(reached_to=True, penetration=True, clipping=0.001),
                    "action_seed": seed,
                    "steps": steps,
                    "episode_return": 1.0,
                    "grf_penetration_max_m": 0.025,
                    "phase_valid_cycle_count": cycle,
                }
            )
        report = classify_relative_h1_gate(
            probes,
            deterministic_steps=356,
            deterministic_unchanged=True,
        )
        self.assertEqual(report["status"], "PASS")


if __name__ == "__main__":
    unittest.main(verbosity=2)
