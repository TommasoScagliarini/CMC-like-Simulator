"""Unit and live tests for ``binary_phase_invalid_event_policy``.

Training semantics agreed for the native-V26 imitation retraining: a
behaviour-dependent actor-FSM rejection must terminate the episode as a true
MDP termination (like the penetration guard), never kill the worker; the
default ``raise`` preserves the fail-closed qualification behaviour byte for
byte. The live tests build the REAL environment from the frozen June recipe
plus the canonical V26 flags and inject a deterministic rejection at the
adapter boundary.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
TG = REPO_ROOT / "Trajectory Generator"
for entry in (str(TG), str(TG / "baseline_MLP")):
    if entry not in sys.path:
        sys.path.insert(0, entry)

import binary_phase_adapter  # noqa: E402

JUNE_RESOLVED_CFG = (
    TG
    / "runs"
    / "training"
    / "MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter"
    / "training_cfg.resolved.yaml"
)
V25_PROFILE = (
    "validation/binary_phase_detector_v25_geometry_runs/"
    "2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json"
)


def test_transfer_error_is_a_value_error_subclass() -> None:
    assert issubclass(
        binary_phase_adapter.BinaryPhaseTransferError, ValueError
    )


def test_fsm_rejection_raises_the_typed_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    adapter = binary_phase_adapter.BinaryPhaseActiveAdapter
    monkeypatch.setattr(
        adapter,
        "_validate_phase_runtime_state",
        classmethod(lambda cls, payload: None),
    )
    payload = {
        "event_source": "binary_active",
        "invalid_event_this_step": 1.0,
        "invalid_event_type": "to_too_early_after_hs",
        "state_name": "STANCE_AFTER_HS",
    }
    event = {
        "side": "left",
        "event": "toe_off",
        "source": "v25_fsm",
        "event_contract_id": "binary_point_v25+functional_contact_fsm_v1",
        "event_time_s": 1.0,
        "confirmed_time_s": 1.005,
        "delivered_time_s": 1.010,
    }
    with pytest.raises(binary_phase_adapter.BinaryPhaseTransferError):
        adapter._validate_transfer([event], payload)  # noqa: SLF001


def test_training_config_exposes_the_policy_key() -> None:
    import training_config

    source = Path(training_config.__file__).read_text(encoding="utf-8")
    assert '"binary_phase_invalid_event_policy": str,' in source


def _make_env(policy: str, monkeypatch: pytest.MonkeyPatch):
    import env_factory
    import train_ppo_mlp

    train_ppo_mlp._load_training_stack()  # noqa: SLF001

    argv = [
        "train_ppo_mlp.py",
        "--config",
        str(JUNE_RESOLVED_CFG),
        "--iterations",
        "1",
        "--phase-fsm-input-mode",
        "legacy_events",
        "--event-contract-id",
        "legacy_events_v1",
        "--binary-phase-fsm-mode",
        "binary_active",
        "--binary-phase-detector-profile",
        V25_PROFILE,
        "--online-grf-detector-profile",
        "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json",
        "--online-grf-profile",
        "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json",
        "--detector-sample-dt-s",
        "0.001",
        "--binary-phase-debounce-s",
        "0.005",
        "--binary-phase-event-contract-id",
        "binary_point_v25+heel_qualified_fsm_v2",
        "--binary-phase-invalid-event-policy",
        policy,
    ]
    monkeypatch.setattr(sys, "argv", argv)
    args = train_ppo_mlp.parse_args()
    config = train_ppo_mlp.build_config(args)
    env_config = dict(config.env_config)
    assert env_config["binary_phase_invalid_event_policy"] == policy
    return env_factory.make_cmc_env(env_config)


def _base_env(env):
    return env.unwrapped


@pytest.mark.parametrize("policy", ["terminate"])
def test_injected_rejection_terminates_the_episode_cleanly(
    policy: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    env = _make_env(policy, monkeypatch)
    try:
        env.reset()
        action = np.zeros(env.action_space.shape, dtype=np.float32)
        for _ in range(5):
            _obs, _reward, terminated, truncated, _info = env.step(action)
            assert not terminated and not truncated
        base = _base_env(env)

        def injected(**_kwargs):
            raise binary_phase_adapter.BinaryPhaseTransferError(
                "injected deterministic rejection"
            )

        base._binary_phase_active_adapter.advance = injected  # noqa: SLF001
        _obs, _reward, terminated, truncated, info = env.step(action)
        assert terminated is True
        assert truncated is False
        assert info.get("end_reason") == "invalid_binary_event"
        assert base._binary_phase_invalid_event_count == 1  # noqa: SLF001
    finally:
        env.close()


def test_injected_rejection_with_default_raise_stays_fail_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    env = _make_env("raise", monkeypatch)
    try:
        env.reset()
        action = np.zeros(env.action_space.shape, dtype=np.float32)
        _obs, _reward, terminated, truncated, _info = env.step(action)
        assert not terminated and not truncated
        base = _base_env(env)

        def injected(**_kwargs):
            raise binary_phase_adapter.BinaryPhaseTransferError(
                "injected deterministic rejection"
            )

        base._binary_phase_active_adapter.advance = injected  # noqa: SLF001
        with pytest.raises(binary_phase_adapter.BinaryPhaseTransferError):
            env.step(action)
    finally:
        env.close()
