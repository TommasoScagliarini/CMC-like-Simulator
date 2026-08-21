"""Live tests: process guards must terminate the episode, never the worker.

Covers the two guards promoted from worker-fatal exceptions to clean MDP
terminations (user decision 2026-08-20):

* ``OnlineGRFPenetrationLimitExceeded`` (30 mm hard physics guard) →
  ``end_reason="grf_penetration_hard"``;
* V26 latch/phase coherence rejections (``BinaryPhaseTransferError`` raised
  outside the droppable event-transfer stage) → terminal
  ``invalid_binary_event``.

Both tests build the REAL environment from the fresh june-equivalent
baseline recipe and assert the env survives (reset + further steps) after
the terminal event.
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
import simulation_runner  # noqa: E402

FRESH_RESOLVED_CFG = (
    TG
    / "runs"
    / "training"
    / "MLP_imitation_native_v26_08-20-2026_june_equiv_100iter"
    / "training_cfg.resolved.yaml"
)


def _make_env(monkeypatch: pytest.MonkeyPatch):
    import env_factory
    import train_ppo_mlp

    train_ppo_mlp._load_training_stack()  # noqa: SLF001
    argv = [
        "train_ppo_mlp.py",
        "--config",
        str(FRESH_RESOLVED_CFG),
        "--iterations",
        "1",
    ]
    monkeypatch.setattr(sys, "argv", argv)
    args = train_ppo_mlp.parse_args()
    reward_overrides = dict(getattr(args, "_cfg_reward", None) or {})
    config = train_ppo_mlp.build_config(args, reward_overrides)
    env_config = dict(config.env_config)
    assert env_config["binary_phase_invalid_event_policy"] == "reject_continue"
    return env_factory.make_cmc_env(env_config)


def test_penetration_limit_exception_is_a_floating_point_error() -> None:
    assert issubclass(
        simulation_runner.OnlineGRFPenetrationLimitExceeded, FloatingPointError
    )


def test_hard_penetration_terminates_episode_and_env_survives(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    env = _make_env(monkeypatch)
    try:
        env.reset()
        base = env.unwrapped
        action = np.zeros(env.action_space.shape, dtype=np.float32)
        _obs, _reward, terminated, truncated, _info = env.step(action)
        assert not terminated and not truncated
        # Shrink the hard limit below any loaded-contact penetration so the
        # guard fires on the next contact-bearing step through the REAL path.
        monkeypatch.setattr(
            base.runner._cfg, "online_grf_max_penetration_m", 1e-6  # noqa: SLF001
        )
        terminated = truncated = False
        info: dict = {}
        for _ in range(100):
            _obs, _reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                break
        assert terminated is True
        assert truncated is False
        assert info.get("end_reason") == "grf_penetration_hard"
        # The worker (this process) must survive: full reset + steps.
        monkeypatch.setattr(
            base.runner._cfg, "online_grf_max_penetration_m", 0.03  # noqa: SLF001
        )
        env.reset()
        for _ in range(3):
            _obs, _reward, terminated, truncated, _info = env.step(action)
            assert not terminated and not truncated
    finally:
        env.close()


def test_state_level_rejection_terminates_cleanly_under_reject_continue(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    env = _make_env(monkeypatch)
    try:
        env.reset()
        base = env.unwrapped
        action = np.zeros(env.action_space.shape, dtype=np.float32)
        _obs, _reward, terminated, truncated, _info = env.step(action)
        assert not terminated and not truncated
        adapter_cls = type(base._binary_phase_active_adapter)  # noqa: SLF001
        calls = {"n": 0}
        original = adapter_cls.advance

        def raising_advance(self, **kwargs):
            if calls["n"] == 0:
                calls["n"] += 1
                raise binary_phase_adapter.BinaryPhaseTransferError(
                    "injected state-level rejection (latch coherence)"
                )
            return original(self, **kwargs)

        monkeypatch.setattr(adapter_cls, "advance", raising_advance)
        _obs, _reward, terminated, truncated, info = env.step(action)
        assert calls["n"] == 1
        assert terminated is True
        assert truncated is False
        assert info.get("end_reason") == "invalid_binary_event"
        assert base._binary_phase_invalid_event_count == 1  # noqa: SLF001
        env.reset()
        for _ in range(3):
            _obs, _reward, terminated, truncated, _info = env.step(action)
            assert not terminated and not truncated
    finally:
        env.close()
