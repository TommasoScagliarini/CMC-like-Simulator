"""Focused behavioral checks for baseline_MLP/reward_function.py."""

from __future__ import annotations

import math
import sys
from pathlib import Path


BASELINE_DIR = (
    Path(__file__).resolve().parents[1] / "Trajectory Generator" / "baseline_MLP"
)
sys.path.insert(0, str(BASELINE_DIR))

import reward_function  # noqa: E402
import training_config  # noqa: E402


TESTS = []


def _test(func):
    TESTS.append(func)
    return func


@_test
def test_v4_three_objective_blend() -> None:
    cfg = reward_function.RewardConfig.from_mapping(
        {
            "reward_mode": "imitation",
            "blend_served_imitation": 0.65,
            "blend_imitation": 0.20,
            "blend_imitation_tracking": 0.15,
            "oob_weight": 0.0,
        }
    )
    terms = {
        "served_imitation_loss": 0.25,
        "sound_imitation_loss": 0.50,
        "tracking_loss": 0.125,
    }

    reward, components = reward_function.compute_reward(terms, cfg)

    expected = (
        0.65 / (1.0 + 8.0 * 0.25)
        + 0.20 / (1.0 + 8.0 * 0.50)
        + 0.15 / (1.0 + 8.0 * 0.125)
    )
    assert math.isclose(reward, expected, rel_tol=0.0, abs_tol=1e-12)
    assert math.isclose(components["reward_base"], expected, abs_tol=1e-12)


@_test
def test_penetration_is_applied_after_clip() -> None:
    cfg = reward_function.RewardConfig(
        reward_mode="imitation",
        blend_served_imitation=1.0,
        blend_imitation=0.0,
        blend_imitation_tracking=0.0,
        grf_penetration_weight=5.0,
        oob_weight=0.0,
    )

    reward, components = reward_function.compute_reward(
        {"served_imitation_loss": 0.0, "grf_penetration_loss": 0.004},
        cfg,
    )

    assert math.isclose(reward, 0.98, rel_tol=0.0, abs_tol=1e-12)
    assert math.isclose(
        components["grf_penetration_term"], 0.02, rel_tol=0.0, abs_tol=1e-12
    )


@_test
def test_component_level_command_penalties_are_explicit() -> None:
    cfg = reward_function.RewardConfig(
        reward_mode="imitation",
        blend_served_imitation=1.0,
        blend_imitation=0.0,
        blend_imitation_tracking=0.0,
        jerk_ref_weight=0.25,
        segment_delta_weight=0.5,
        oob_weight=0.0,
    )

    reward, components = reward_function.compute_reward(
        {
            "served_imitation_loss": 0.0,
            "jerk_ref_loss": 0.4,
            "segment_delta_loss": 0.2,
        },
        cfg,
    )

    assert math.isclose(reward, 0.8, rel_tol=0.0, abs_tol=1e-12)
    assert math.isclose(components["penalty"], 0.2, rel_tol=0.0, abs_tol=1e-12)
    assert components["jerk_ref_loss"] == 0.4
    assert components["segment_delta_loss"] == 0.2


@_test
def test_v4_config_enables_central_penetration_penalty() -> None:
    cfg = training_config.load(BASELINE_DIR / "training_cfg.v4_imitation.yaml")
    _, reward = training_config.to_argparse_defaults(cfg)
    resolved = reward_function.RewardConfig.from_mapping(reward)

    assert resolved.reward_mode == "imitation"
    assert resolved.blend_served_imitation == 0.65
    assert resolved.grf_penetration_weight == 5.0


if __name__ == "__main__":
    for test in TESTS:
        test()
        print(f"[PASS] {test.__name__}")
    print(f"ALL REWARD TESTS PASSED ({len(TESTS)})")
