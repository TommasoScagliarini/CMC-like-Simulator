"""Absolute action mapping and rejection of obsolete action contracts."""

import importlib
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest
import yaml

from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv
from prescribed_action import prescribed_action


class Reference:
    def get(self, t):
        return {"knee": -t, "ankle": 2 * t}, {"knee": -1., "ankle": 2.}, {}


def mapping_env(mode="absolute"):
    env = object.__new__(CMCLikeProsthesisTrajectoryEnv)
    env.env_cfg = SimpleNamespace(
        action_mode=mode, policy_knots=3, segment_duration=.1,
        absolute_bounds_rad={"knee": (-1.3, 0.), "ankle": (-.6, .6)},
    )
    env.cfg = SimpleNamespace(pros_coords=("knee", "ankle"), dt=.001)
    env.t, env._episode_end = .5, .575
    env.kin = Reference()
    env.base_kin = Reference()
    env._last_policy_endpoint = np.array([-.5, 1.])
    env._target_slew_rate_limit = np.full(2, np.inf)
    return env


def test_absolute_endpoints_and_midpoints_do_not_read_prescribed_ik():
    env = mapping_env()
    del env.base_kin
    actions = np.array([[-1., 1.], [0., 0.], [1., -1.]])
    times, values, _ = env._action_to_segment(actions, .575)
    np.testing.assert_allclose(times, [.5, .525, .55, .575])
    np.testing.assert_allclose(values[1:], [[-1.3, .6], [-.65, 0.], [0., -.6]])
    assert env._build_action_space().shape == (3, 2)


def test_removed_mode_is_rejected_by_environment():
    with pytest.raises(ValueError, match="action_mode"):
        mapping_env("delta")._build_action_space()
    with pytest.raises(TypeError, match="max_delta_rad"):
        CMCEnvConfig(max_delta_rad=.35)


def test_prescribed_replay_keeps_exact_knots_including_short_final_step():
    env = mapping_env("raw")
    actions = prescribed_action(env)
    expected = np.array([[-.525, 1.05], [-.55, 1.1], [-.575, 1.15]])
    np.testing.assert_allclose(actions, expected, rtol=0., atol=3e-16)
    _, values, _ = env._action_to_segment(actions, env._episode_end)
    np.testing.assert_array_equal(values[1:], actions)


def test_prescribed_oracle_cannot_be_used_as_normalized_policy_actions():
    with pytest.raises(ValueError, match="raw"):
        prescribed_action(mapping_env())


@pytest.mark.parametrize("package", ["baseline_MLP", "unified_MLP", "unified_MLP_39"])
@pytest.mark.parametrize("entrypoint", ["train_ppo_mlp", "rollout_eval"])
def test_cli_and_yaml_reject_delta_and_old_absolute_snapshots_load(
    package, entrypoint, tmp_path
):
    if not (Path(__file__).resolve().parents[2] / package).is_dir():
        pytest.skip("Optional materialized variant is not present in this checkout")
    module = importlib.import_module(f"{package}.{entrypoint}")
    # Use a current complete preset as if loaded from an old checkpoint.
    source = Path(__file__).resolve().parents[1] / "configs" / "smoke_pretraining.yaml"
    cfg = yaml.safe_load(source.read_text())
    cfg["simulation"]["max_delta_rad"] = .35
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(cfg))
    argv = [entrypoint, "--config", str(path)]
    if entrypoint == "rollout_eval":
        argv += ["--checkpoint", str(tmp_path), "--no-auto-config"]
    with patch.object(sys, "argv", argv):
        args = module.parse_args()
    assert args.action_mode == "absolute"
    assert not hasattr(args, "max_delta_rad")
    for option in (["--action-mode", "delta"], ["--max-delta-rad", ".35"]):
        with patch.object(sys, "argv", argv + option), pytest.raises(SystemExit):
            module.parse_args()
    cfg["simulation"]["action_mode"] = "delta"
    path.write_text(yaml.safe_dump(cfg))
    with patch.object(sys, "argv", argv), pytest.raises(SystemExit):
        module.parse_args()
