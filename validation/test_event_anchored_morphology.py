"""Standalone regression checks for event-anchored morphology phase mapping.

Run directly from the repository root::

    python validation/test_event_anchored_morphology.py

The suite deliberately uses only the runtime reward/FSM APIs.  In particular,
it does not import the morphology-profile builder, so it can also validate a
deployment checkout that contains only the generated JSON asset.
"""

from __future__ import annotations

import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_DIR = REPO_ROOT / "Trajectory Generator"
BASELINE_DIR = TRAJECTORY_DIR / "baseline_MLP"
MORPHOLOGY_DIR = BASELINE_DIR / "morphology_profiles"
LEGACY_MORPHOLOGY_PROFILE = (
    MORPHOLOGY_DIR / "ab06_prosthetic_mean_std_corridor.json"
)
EVENT_MORPHOLOGY_PROFILE = (
    MORPHOLOGY_DIR / "ab06_prosthetic_event_warped_mean_std_corridor.json"
)

sys.path.insert(0, str(TRAJECTORY_DIR))
sys.path.insert(0, str(BASELINE_DIR))

from prosthetic_phase_fsm import (  # noqa: E402
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)
import reward_function  # noqa: E402
import training_config  # noqa: E402


TESTS = []


def _test(func):
    TESTS.append(func)
    return func


def _assert_close(actual: float, expected: float, *, atol: float = 1e-12) -> None:
    assert math.isclose(
        float(actual),
        float(expected),
        rel_tol=0.0,
        abs_tol=atol,
    ), f"expected {expected!r}, got {actual!r}"


def _assert_raises(error_type, callback, *, contains: str = "") -> None:
    try:
        callback()
    except error_type as exc:
        if contains:
            assert contains in str(exc), str(exc)
        return
    except Exception as exc:  # pragma: no cover - improves standalone diagnostics.
        raise AssertionError(
            f"expected {error_type.__name__}, got {type(exc).__name__}: {exc}"
        ) from exc
    raise AssertionError(f"expected {error_type.__name__} to be raised")


class _DummyEnv(reward_function.gym.Env):
    action_space = reward_function.gym.spaces.Box(
        low=-1.0,
        high=1.0,
        shape=(2,),
        dtype=np.float32,
    )


def _phase_payload(
    *,
    state_id: int,
    stance_elapsed_s: float = 0.0,
    swing_elapsed_s: float = 0.0,
    valid_cycle_count: float = 0.0,
    last_period_s: float = 0.0,
    last_stance_fraction: float = 0.0,
    robust_stance_duration_s: float = 0.0,
    robust_swing_duration_s: float = 0.0,
    duration_history_count: float = 0.0,
) -> dict[str, object]:
    return {
        "phase_fsm": {
            "state_id": float(state_id),
            "stance_elapsed_s": float(stance_elapsed_s),
            "swing_elapsed_s": float(swing_elapsed_s),
            "valid_cycle_count": float(valid_cycle_count),
            "last_period_s": float(last_period_s),
            "last_stance_fraction": float(last_stance_fraction),
            "robust_stance_duration_s": float(robust_stance_duration_s),
            "robust_swing_duration_s": float(robust_swing_duration_s),
            "duration_history_count": float(duration_history_count),
        }
    }


@_test
def test_default_mode_is_legacy_and_bit_compatible() -> None:
    cfg_default = reward_function.RewardConfig(
        phase_period_nominal_s=1.58,
        prosthetic_stance_phase_end=0.68,
    )
    cfg_explicit = reward_function.RewardConfig(
        phase_period_nominal_s=1.58,
        prosthetic_stance_phase_end=0.68,
        morphology_phase_mode="legacy_cycle_fraction",
    )
    assert cfg_default.morphology_phase_mode == "legacy_cycle_fraction"

    # Robust-duration fields must be inert in legacy mode.  This payload also
    # exercises the historical measured-cycle branch exactly as older runs did.
    info = _phase_payload(
        state_id=1,
        stance_elapsed_s=0.30,
        valid_cycle_count=2.0,
        last_period_s=1.20,
        last_stance_fraction=0.50,
        robust_stance_duration_s=0.90,
        robust_swing_duration_s=0.30,
        duration_history_count=2.0,
    )
    default_result = reward_function._fsm_morphology_phase(info, cfg_default)
    explicit_result = reward_function._fsm_morphology_phase(info, cfg_explicit)

    assert default_result == explicit_result
    phase, available, source_id = default_result
    assert phase is not None
    _assert_close(phase, 0.30 / 1.20)
    assert available == 1.0
    assert source_id == 2.0


@_test
def test_event_bootstrap_maps_half_stance_and_half_swing_to_profile_anchors() -> None:
    alpha = 0.62
    cfg = reward_function.RewardConfig(
        phase_period_nominal_s=1.58,
        prosthetic_stance_phase_end=0.68,
        morphology_phase_mode="event_anchored",
    )
    nominal_stance_s = 1.58 * 0.68
    nominal_swing_s = 1.58 * (1.0 - 0.68)

    stance = reward_function._fsm_morphology_phase(
        _phase_payload(state_id=1, stance_elapsed_s=0.5 * nominal_stance_s),
        cfg,
        canonical_to_phase=alpha,
    )
    swing = reward_function._fsm_morphology_phase(
        _phase_payload(state_id=2, swing_elapsed_s=0.5 * nominal_swing_s),
        cfg,
        canonical_to_phase=alpha,
    )

    assert stance[0] is not None and swing[0] is not None
    _assert_close(stance[0], alpha / 2.0)
    _assert_close(swing[0], alpha + (1.0 - alpha) / 2.0)
    assert stance[1:] == (1.0, 1.0)
    assert swing[1:] == (1.0, 1.0)


@_test
def test_event_segments_clip_at_hs_to_and_cycle_end() -> None:
    alpha = 0.625
    cfg = reward_function.RewardConfig(
        phase_period_nominal_s=1.0,
        prosthetic_stance_phase_end=0.60,
        morphology_phase_mode="event_anchored",
    )

    early_stance = reward_function._fsm_morphology_phase(
        _phase_payload(state_id=1, stance_elapsed_s=-10.0),
        cfg,
        canonical_to_phase=alpha,
    )[0]
    late_stance = reward_function._fsm_morphology_phase(
        _phase_payload(state_id=1, stance_elapsed_s=10.0),
        cfg,
        canonical_to_phase=alpha,
    )[0]
    early_swing = reward_function._fsm_morphology_phase(
        _phase_payload(state_id=2, swing_elapsed_s=-10.0),
        cfg,
        canonical_to_phase=alpha,
    )[0]
    late_swing = reward_function._fsm_morphology_phase(
        _phase_payload(state_id=2, swing_elapsed_s=10.0),
        cfg,
        canonical_to_phase=alpha,
    )[0]

    assert None not in (early_stance, late_stance, early_swing, late_swing)
    _assert_close(early_stance, 0.0)
    _assert_close(late_stance, alpha)
    _assert_close(early_swing, alpha)
    _assert_close(late_swing, 1.0)


@_test
def test_event_mode_uses_last_measured_cycle_before_robust_history() -> None:
    alpha = 0.60
    cfg = reward_function.RewardConfig(morphology_phase_mode="event_anchored")
    common = {
        "valid_cycle_count": 1.0,
        "last_period_s": 1.20,
        "last_stance_fraction": 0.50,
    }

    stance = reward_function._fsm_morphology_phase(
        _phase_payload(state_id=1, stance_elapsed_s=0.30, **common),
        cfg,
        canonical_to_phase=alpha,
    )
    swing = reward_function._fsm_morphology_phase(
        _phase_payload(state_id=2, swing_elapsed_s=0.30, **common),
        cfg,
        canonical_to_phase=alpha,
    )

    assert stance[0] is not None and swing[0] is not None
    _assert_close(stance[0], alpha / 2.0)
    _assert_close(swing[0], alpha + (1.0 - alpha) / 2.0)
    assert stance[2] == 2.0
    assert swing[2] == 2.0


@_test
def test_event_mode_prefers_robust_durations_and_reports_source_four() -> None:
    alpha = 0.64
    cfg = reward_function.RewardConfig(morphology_phase_mode="event_anchored")
    common = {
        "valid_cycle_count": 3.0,
        "last_period_s": 1.00,
        "last_stance_fraction": 0.50,
        "robust_stance_duration_s": 0.80,
        "robust_swing_duration_s": 0.40,
        "duration_history_count": 3.0,
    }

    stance = reward_function._fsm_morphology_phase(
        _phase_payload(state_id=1, stance_elapsed_s=0.40, **common),
        cfg,
        canonical_to_phase=alpha,
    )
    swing = reward_function._fsm_morphology_phase(
        _phase_payload(state_id=2, swing_elapsed_s=0.20, **common),
        cfg,
        canonical_to_phase=alpha,
    )

    assert stance[0] is not None and swing[0] is not None
    _assert_close(stance[0], alpha / 2.0)
    _assert_close(swing[0], alpha + (1.0 - alpha) / 2.0)
    assert stance[1:] == (1.0, 4.0)
    assert swing[1:] == (1.0, 4.0)


@_test
def test_morphology_phase_mode_and_reward_config_roundtrip() -> None:
    assert (
        reward_function.RewardConfig.from_mapping({}).morphology_phase_mode
        == "legacy_cycle_fraction"
    )
    assert (
        reward_function._normalize_morphology_phase_mode("event")
        == "event_anchored"
    )
    assert (
        reward_function._normalize_morphology_phase_mode("hs-to-anchored")
        == "event_anchored"
    )

    source = {
        "reward": {
            "morphology_profile": str(LEGACY_MORPHOLOGY_PROFILE),
            "morphology_phase_mode": "event_anchored",
            "morphology_weight": 0.125,
        }
    }
    flat, reward_overrides = training_config.to_argparse_defaults(source)
    assert flat == {}
    first = reward_function.RewardConfig.from_mapping(reward_overrides)
    snapshot = first.to_dict()
    restored = reward_function.RewardConfig.from_mapping(snapshot)

    assert snapshot["morphology_phase_mode"] == "event_anchored"
    assert restored.morphology_phase_mode == first.morphology_phase_mode
    assert restored.morphology_profile == first.morphology_profile
    assert restored.morphology_weight == first.morphology_weight


@_test
def test_wrapper_uses_profile_to_anchor_for_served_trajectories() -> None:
    raw_profile = json.loads(EVENT_MORPHOLOGY_PROFILE.read_text(encoding="utf-8"))
    alpha = float(raw_profile["metadata"]["canonical_to_phase"])
    cfg = reward_function.RewardConfig(
        morphology_profile=str(EVENT_MORPHOLOGY_PROFILE),
        morphology_phase_mode="event_anchored",
        phase_period_nominal_s=1.58,
        prosthetic_stance_phase_end=0.68,
    )
    wrapper = reward_function.RewardShapingWrapper(_DummyEnv(), cfg)
    nominal_stance_s = 1.58 * 0.68

    terms = wrapper._morphology_terms(
        {
            **_phase_payload(
                state_id=1,
                stance_elapsed_s=0.5 * nominal_stance_s,
            ),
            "observation": {
                "pros_knee_angle_served_ref": -0.45,
                "pros_ankle_angle_served_ref": 0.05,
            },
        }
    )

    _assert_close(wrapper._morphology_canonical_to_phase, alpha)
    assert terms["morphology_available"] == 1.0
    _assert_close(terms["morphology_phase"], alpha / 2.0)
    _assert_close(terms["fsm_morphology_phase"], alpha / 2.0)
    _assert_close(terms["morphology_canonical_to_phase"], alpha)
    assert terms["morphology_phase_mode_id"] == 2.0
    assert terms["morphology_phase_source_id"] == 1.0
    assert terms["morphology_phase_fsm_available"] == 1.0


@_test
def test_wrapper_rejects_invalid_mode_and_event_metadata_at_construction() -> None:
    _assert_raises(
        ValueError,
        lambda: reward_function.RewardShapingWrapper(
            _DummyEnv(),
            reward_function.RewardConfig(morphology_phase_mode="not-a-mode"),
        ),
        contains="unsupported morphology_phase_mode",
    )

    legacy_cfg = reward_function.RewardConfig(
        morphology_profile=str(LEGACY_MORPHOLOGY_PROFILE),
        morphology_phase_mode="event_anchored",
    )
    _assert_raises(
        ValueError,
        lambda: reward_function.RewardShapingWrapper(_DummyEnv(), legacy_cfg),
        contains="phase_parameterization",
    )

    raw_profile = json.loads(EVENT_MORPHOLOGY_PROFILE.read_text(encoding="utf-8"))
    event_contract = {
        "phase_parameterization": "event_warped_hs_to_to_to_hs_v1",
    }
    invalid_metadata = (
        ({}, "phase_parameterization"),
        ({**event_contract, "canonical_to_phase": 0.0}, "canonical_to_phase"),
        ({**event_contract, "canonical_to_phase": 1.0}, "canonical_to_phase"),
        ({**event_contract, "canonical_to_phase": "nan"}, "canonical_to_phase"),
    )
    with tempfile.TemporaryDirectory(prefix="morphology-invalid-") as temp_dir:
        for index, (metadata, expected_error) in enumerate(invalid_metadata):
            malformed = dict(raw_profile)
            malformed["metadata"] = metadata
            profile_path = Path(temp_dir) / f"invalid_{index}.json"
            profile_path.write_text(json.dumps(malformed), encoding="utf-8")
            cfg = reward_function.RewardConfig(
                morphology_profile=str(profile_path),
                morphology_phase_mode="event_anchored",
            )
            _assert_raises(
                ValueError,
                lambda cfg=cfg: reward_function.RewardShapingWrapper(
                    _DummyEnv(), cfg
                ),
                contains=expected_error,
            )


@_test
def test_zero_morphology_weight_is_reward_identical_in_both_phase_modes() -> None:
    common = {
        "reward_mode": "ex_novo",
        "blend_tracking": 0.75,
        "blend_contact_load": 0.10,
        "morphology_weight": 0.0,
        "oob_weight": 0.0,
    }
    legacy = reward_function.RewardConfig.from_mapping(
        {**common, "morphology_phase_mode": "legacy_cycle_fraction"}
    )
    event = reward_function.RewardConfig.from_mapping(
        {**common, "morphology_phase_mode": "event_anchored"}
    )
    without_morphology = {
        "tracking_loss": 0.25,
        "contact_load_score": 0.6,
    }
    with_morphology = {
        **without_morphology,
        "morphology_loss": 1.0e9,
    }

    legacy_reward, legacy_components = reward_function.compute_reward(
        with_morphology, legacy
    )
    event_reward, event_components = reward_function.compute_reward(
        with_morphology, event
    )
    reference_reward, _ = reward_function.compute_reward(without_morphology, event)

    assert legacy_reward == event_reward == reference_reward
    assert legacy_components["morphology_term"] == 0.0
    assert event_components["morphology_term"] == 0.0


def _left_event(name: str, time_s: float) -> dict[str, object]:
    return {"side": "left", "event": name, "time": float(time_s)}


@_test
def test_fsm_robust_duration_median_rolls_and_reset_clears_it() -> None:
    fsm = ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(
            min_stance_duration_s=0.01,
            min_swing_duration_s=0.01,
            stance_hard_timeout_s=20.0,
            swing_hard_timeout_s=20.0,
            min_stance_contact_fraction=0.0,
            min_stance_load_bw_s=0.0,
            min_cycle_knee_excursion_rad=0.0,
            duration_history_window_cycles=3,
        )
    )
    time_s = 0.0
    fsm.update(
        time_s=time_s,
        events=[_left_event("heel_strike", time_s)],
        normal_force_bw=1.0,
        in_contact=True,
    )

    durations = ((0.60, 0.40), (0.80, 0.20), (1.00, 0.60), (1.20, 0.80))
    for stance_s, swing_s in durations:
        toe_off_s = time_s + stance_s
        fsm.update(
            time_s=toe_off_s,
            events=[_left_event("toe_off", toe_off_s)],
            normal_force_bw=1.0,
            in_contact=True,
        )
        time_s = toe_off_s + swing_s
        payload = fsm.update(
            time_s=time_s,
            events=[_left_event("heel_strike", time_s)],
            normal_force_bw=0.0,
            in_contact=False,
        )

    # Window 3 retains cycles 2-4: stance [0.8, 1.0, 1.2], swing [0.2, 0.6, 0.8].
    assert payload["valid_cycle_count"] == 4.0
    assert payload["duration_history_count"] == 3.0
    _assert_close(payload["robust_stance_duration_s"], 1.0)
    _assert_close(payload["robust_swing_duration_s"], 0.6)

    fsm.reset()
    reset_payload = fsm.payload()
    assert reset_payload["duration_history_count"] == 0.0
    assert reset_payload["robust_stance_duration_s"] == 0.0
    assert reset_payload["robust_swing_duration_s"] == 0.0
    assert reset_payload["valid_cycle_count"] == 0.0


@_test
def test_event_warped_profile_or_synthetic_anchor_helper() -> None:
    candidates = sorted(
        {
            *MORPHOLOGY_DIR.glob("*event*.json"),
            *MORPHOLOGY_DIR.glob("*warp*.json"),
        }
    )
    if candidates:
        profile = reward_function._load_morphology_profile(candidates[0])
        assert profile is not None
        alpha = reward_function._morphology_canonical_to_phase(
            profile,
            require_event_contract=True,
        )
        assert 0.0 < alpha < 1.0
        assert profile["phase_grid"][0] == 0.0
        assert profile["phase_grid"][-1] == 1.0
        return

    # A checkout may intentionally omit the derived event-warped asset.  Keep
    # this regression meaningful by exercising the runtime helper directly,
    # without importing or depending on the offline profile builder.
    synthetic_profile = {
        "metadata": {
            "canonical_to_phase": 0.625,
            "mean_to_phase": 0.70,
            "phase_parameterization": "event_warped_hs_to_to_to_hs_v1",
        }
    }
    _assert_close(
        reward_function._morphology_canonical_to_phase(
            synthetic_profile,
            require_event_contract=True,
        ),
        0.625,
    )


if __name__ == "__main__":
    for test in TESTS:
        test()
        print(f"[PASS] {test.__name__}")
    print(f"ALL EVENT-ANCHORED MORPHOLOGY TESTS PASSED ({len(TESTS)})")
