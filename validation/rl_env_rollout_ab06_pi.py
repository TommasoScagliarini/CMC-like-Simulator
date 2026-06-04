"""
Longer-rollout, residual-state, and reset-cost validation for the trajectory RL
environment (osim_trj_cmc_like.py) on the AB06 SEA PI setup.

This extends validation/rl_env_smoke_ab06_pi.py (which only exercises a single
2 ms episode) to the open TODOs from the 2026-05-29 daily report:

  - longer rollouts (tens to hundreds of ms),
  - reusable reset leaves no residual state across episodes,
  - cost of reset() with rebuild_model_on_reset True vs False.

Run:
    conda run --no-capture-output -n envCMC-like \
        python validation/rl_env_rollout_ab06_pi.py

The script is deterministic (random_init=False, fixed seeds, scripted actions)
so the residual-state comparison is meaningful.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
# The RL adapter was relocated under "Trajectory Generator/" alongside the SNN
# trajectory-generator code; keep it importable from there too. Repo root still
# takes precedence so config/simulation_runner resolve against the canonical tree.
_TRAJ_GEN_DIR = REPO_ROOT / "Trajectory Generator"
if _TRAJ_GEN_DIR.is_dir() and str(_TRAJ_GEN_DIR) not in sys.path:
    sys.path.append(str(_TRAJ_GEN_DIR))

from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv


SETUP_XML = "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"


def _make_env(
    *,
    segment_duration: float,
    episode_duration: float,
    rebuild_model_on_reset: bool = False,
) -> CMCLikeProsthesisTrajectoryEnv:
    return CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            setup_xml_path=SETUP_XML,
            segment_duration=segment_duration,
            episode_duration=episode_duration,
            policy_knots=1,
            action_mode="delta",
            random_init=False,
            rebuild_model_on_reset=rebuild_model_on_reset,
            record_outputs=False,
            fail_fast=True,
        )
    )


def _scripted_actions(env, n_steps: int, *, amplitude: float, seed: int) -> np.ndarray:
    """Deterministic, smooth, small action sequence to excite the controller."""
    rng = np.random.default_rng(seed)
    shape = tuple(env.action_space.shape)
    # Smooth per-channel sinusoids with seeded phase/freq, scaled to `amplitude`.
    phases = rng.uniform(0.0, 2.0 * np.pi, size=shape)
    freqs = rng.uniform(0.5, 2.5, size=shape)
    actions = np.empty((n_steps, *shape), dtype=np.float32)
    for k in range(n_steps):
        actions[k] = amplitude * np.sin(freqs * (k + 1) + phases)
    return actions


def _rollout(env, actions: np.ndarray):
    """Run one episode applying `actions` row by row. Returns per-step records."""
    # _last_u_sea holds the full SEA diagnostics dict; the true normalized
    # command u is keyed by the prosthetic coordinate names exactly.
    pros_coords = list(env.cfg.pros_coords)
    rewards: list[float] = []
    times: list[float] = []
    obs_norms: list[float] = []
    max_u: dict[str, float] = {c: 0.0 for c in pros_coords}
    sat_steps: dict[str, int] = {c: 0 for c in pros_coords}
    nonfinite = 0
    terminated = truncated = False
    for k in range(len(actions)):
        obs, reward, terminated, truncated, info = env.step(actions[k])
        rewards.append(float(reward))
        times.append(float(info["time"]))
        obs_norms.append(float(np.linalg.norm(obs)))
        if not np.all(np.isfinite(obs)) or not np.isfinite(reward):
            nonfinite += 1
        for coord in pros_coords:
            u = abs(float(env._last_u_sea.get(coord, 0.0)))  # noqa: SLF001
            max_u[coord] = max(max_u[coord], u)
            if u >= 0.999:
                sat_steps[coord] += 1
        if terminated or truncated:
            break
    return {
        "rewards": np.asarray(rewards),
        "times": np.asarray(times),
        "obs_norms": np.asarray(obs_norms),
        "max_u": max_u,
        "sat_steps": sat_steps,
        "nonfinite": nonfinite,
        "terminated": terminated,
        "truncated": truncated,
        "n_steps": len(rewards),
    }


# ---------------------------------------------------------------------------
# Test L - longer rollout stability
# ---------------------------------------------------------------------------
def test_longer_rollout(segment_duration: float, episode_duration: float) -> None:
    print("\n=== Test L: longer rollout ===")
    n_steps = int(round(episode_duration / segment_duration)) + 2
    env = _make_env(
        segment_duration=segment_duration, episode_duration=episode_duration
    )
    try:
        obs, info = env.reset(seed=0)
        actions = _scripted_actions(env, n_steps, amplitude=0.10, seed=7)
        t0 = time.perf_counter()
        rec = _rollout(env, actions)
        wall = time.perf_counter() - t0
    finally:
        env.close()

    dt = np.diff(rec["times"])
    monotonic = bool(np.all(dt > 0.0)) if len(dt) else True
    print(f"start_t           {info['time']:.4f}")
    print(f"n_steps           {rec['n_steps']}")
    print(f"final_t           {rec['times'][-1]:.4f}")
    print(f"sim_duration_s    {rec['times'][-1] - info['time']:.4f}")
    print(f"wall_s            {wall:.1f}")
    print(f"time_monotonic    {monotonic}")
    print(f"terminated        {rec['terminated']}")
    print(f"truncated         {rec['truncated']}")
    print(f"nonfinite_steps   {rec['nonfinite']}")
    print(f"reward[min,max]   [{rec['rewards'].min():.4f}, {rec['rewards'].max():.4f}]")
    print(f"obs_norm[min,max] [{rec['obs_norms'].min():.4f}, {rec['obs_norms'].max():.4f}]")
    u_str = ", ".join(f"{k}:{v:.3f}" for k, v in rec["max_u"].items())
    sat_str = ", ".join(f"{k}:{v}/{rec['n_steps']}" for k, v in rec["sat_steps"].items())
    print(f"max|u| per coord  {{{u_str}}}")
    print(f"saturated steps   {{{sat_str}}}")

    assert rec["nonfinite"] == 0, "non-finite obs/reward during rollout"
    assert monotonic, "simulation time not strictly increasing"
    assert rec["terminated"] and not rec["truncated"], "episode did not terminate cleanly"
    # The controller clamps u to [-1, 1] internally; this just guards the contract.
    assert all(v <= 1.0 + 1e-6 for v in rec["max_u"].values()), "|u| exceeded 1"
    print("Test L: PASS")


# ---------------------------------------------------------------------------
# Test R - determinism / residual state
# ---------------------------------------------------------------------------
def test_residual_state(segment_duration: float, episode_duration: float) -> None:
    print("\n=== Test R: determinism / residual state ===")
    n_steps = int(round(episode_duration / segment_duration)) + 1

    # Reused env: two consecutive episodes with identical scripted actions.
    env = _make_env(
        segment_duration=segment_duration, episode_duration=episode_duration
    )
    try:
        env.reset(seed=0)
        actions = _scripted_actions(env, n_steps, amplitude=0.2, seed=11)
        a1 = _rollout(env, actions)
        env.reset(seed=0)
        a2 = _rollout(env, actions)
    finally:
        env.close()

    # Fresh env (pristine model) with the same scripted actions.
    env_fresh = _make_env(
        segment_duration=segment_duration, episode_duration=episode_duration
    )
    try:
        env_fresh.reset(seed=0)
        b = _rollout(env_fresh, actions)
    finally:
        env_fresh.close()

    d_reuse_reward = float(np.max(np.abs(a1["rewards"] - a2["rewards"])))
    d_reuse_obs = float(np.max(np.abs(a1["obs_norms"] - a2["obs_norms"])))
    d_fresh_reward = float(np.max(np.abs(a2["rewards"] - b["rewards"])))
    d_fresh_obs = float(np.max(np.abs(a2["obs_norms"] - b["obs_norms"])))

    print(f"n_steps                 {a1['n_steps']}")
    print(f"reuse  A1 vs A2  d_reward {d_reuse_reward:.3e}  d_obs_norm {d_reuse_obs:.3e}")
    print(f"fresh  A2 vs B   d_reward {d_fresh_reward:.3e}  d_obs_norm {d_fresh_obs:.3e}")

    tol = 1e-9
    assert d_reuse_reward < tol and d_reuse_obs < tol, (
        "reused reset is non-deterministic (residual state across episodes)"
    )
    assert d_fresh_reward < tol and d_fresh_obs < tol, (
        "reused-reset episode differs from a freshly-built model (residual state)"
    )
    print("Test R: PASS (reuse is bitwise-deterministic and matches a fresh model)")


# ---------------------------------------------------------------------------
# Test C - reset cost: rebuild True vs False
# ---------------------------------------------------------------------------
def test_reset_cost(segment_duration: float, episode_duration: float, reps: int) -> None:
    print("\n=== Test C: reset cost (rebuild True vs False) ===")

    env_reuse = _make_env(
        segment_duration=segment_duration,
        episode_duration=episode_duration,
        rebuild_model_on_reset=False,
    )
    try:
        env_reuse.reset(seed=0)  # warm up
        reuse_times = []
        for i in range(reps):
            t0 = time.perf_counter()
            env_reuse.reset(seed=i)
            reuse_times.append(time.perf_counter() - t0)
    finally:
        env_reuse.close()

    env_rebuild = _make_env(
        segment_duration=segment_duration,
        episode_duration=episode_duration,
        rebuild_model_on_reset=True,
    )
    try:
        env_rebuild.reset(seed=0)  # warm up
        rebuild_times = []
        for i in range(reps):
            t0 = time.perf_counter()
            env_rebuild.reset(seed=i)
            rebuild_times.append(time.perf_counter() - t0)
    finally:
        env_rebuild.close()

    reuse_mean = float(np.mean(reuse_times))
    rebuild_mean = float(np.mean(rebuild_times))
    print(f"reps                {reps}")
    print(f"reuse_reset_s       mean {reuse_mean:.5f}  all {[round(v, 5) for v in reuse_times]}")
    print(f"rebuild_reset_s     mean {rebuild_mean:.4f}  all {[round(v, 4) for v in rebuild_times]}")
    if reuse_mean > 0:
        print(f"rebuild/reuse ratio {rebuild_mean / reuse_mean:.1f}x")
    print("Test C: done")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seg", type=float, default=0.01, help="segment_duration [s]")
    parser.add_argument(
        "--rollout-dur", type=float, default=0.30, help="Test L episode duration [s]"
    )
    parser.add_argument(
        "--residual-dur", type=float, default=0.05, help="Test R episode duration [s]"
    )
    parser.add_argument("--reset-reps", type=int, default=3)
    parser.add_argument("--skip-cost", action="store_true", help="skip Test C")
    args = parser.parse_args()

    print("setup_xml", SETUP_XML)
    test_longer_rollout(args.seg, args.rollout_dur)
    test_residual_state(args.seg, args.residual_dur)
    if not args.skip_cost:
        test_reset_cost(args.seg, args.residual_dur, args.reset_reps)
    print("\nrl_env_rollout_ab06_pi_ok")


if __name__ == "__main__":
    main()
