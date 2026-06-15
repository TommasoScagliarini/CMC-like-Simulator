"""Validation smoke for: (1) sound-side gait-phase clock and (2) absolute
network output (ex-novo trajectory instead of a delta from prescribed IK).

Run:
    <envCMC-rllib python> validation/_gait_clock_absaction_smoke.py

Temporary validation script (``_`` prefix). It checks correctness, not training.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
_TRAJ_GEN_DIR = REPO_ROOT / "Trajectory Generator"
if _TRAJ_GEN_DIR.is_dir() and str(_TRAJ_GEN_DIR) not in sys.path:
    sys.path.append(str(_TRAJ_GEN_DIR))

from osim_trj_cmc_like import (  # noqa: E402
    CMCEnvConfig,
    CMCLikeProsthesisTrajectoryEnv,
    GaitPhaseClock,
)

SETUP_XML = "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"

_FAILS: list[str] = []


def check(cond: bool, msg: str) -> None:
    status = "PASS" if cond else "FAIL"
    print(f"  [{status}] {msg}")
    if not cond:
        _FAILS.append(msg)


def approx(a, b, tol=1e-6) -> bool:
    return bool(np.all(np.abs(np.asarray(a) - np.asarray(b)) <= tol))


# ---------------------------------------------------------------------------
# Part A: GaitPhaseClock synthetic correctness (no OpenSim).
# ---------------------------------------------------------------------------
def part_a() -> None:
    print("\n=== Part A: GaitPhaseClock (synthetic) ===")
    hs = [1.0, 2.0, 3.2, 4.2]  # periods 1.0, 1.2, 1.0
    clk = GaitPhaseClock(hs, phase_offset=0.0)
    check(clk.available, "available with >=2 heel strikes")
    check(clk.n_cycles == 3, f"n_cycles==3 (got {clk.n_cycles})")
    check(approx(clk.mean_period, (1.0 + 1.2 + 1.0) / 3.0), "mean_period correct")

    check(approx(clk.phase(1.0), 0.0), "phase==0 at heel strike hs[0]")
    check(approx(clk.phase(1.5), 0.5), "phase==0.5 mid first cycle (period 1.0)")
    check(approx(clk.phase(2.0), 0.0), "phase==0 at heel strike hs[1] (wrap)")
    check(approx(clk.phase(2.6), 0.5), "phase==0.5 mid 2nd cycle (period 1.2)")
    check(clk.phase(1.999) > 0.99, "phase ->1 just before next heel strike")
    # Extrapolation outside detected range, wrapped to [0,1).
    check(approx(clk.phase(0.5), 0.5), "extrapolate before first HS (period 1.0)")
    check(approx(clk.phase(4.7), 0.5), "extrapolate after last HS (period 1.0)")
    # Range invariant on a dense sweep.
    ts = np.linspace(-1.0, 7.0, 2000)
    phis = np.array([clk.phase(t) for t in ts])
    check(bool(np.all((phis >= 0.0) & (phis < 1.0))), "phase always in [0,1)")

    # Offset shifts the reset point: with offset 0.25, phi=0 at 25% into a cycle.
    clk2 = GaitPhaseClock(hs, phase_offset=0.25)
    check(approx(clk2.phase(1.25), 0.0), "offset 0.25 -> phi==0 at 25% of cycle")
    check(approx(clk2.phase(1.0), 0.75), "offset 0.25 -> phi==0.75 at heel strike")

    # sin/cos encoding.
    s0, c0 = clk.phase_sin_cos(1.0)
    check(approx([s0, c0], [0.0, 1.0]), "sin/cos at phi=0 -> (0,1)")
    s_q, c_q = clk.phase_sin_cos(1.25)  # phi=0.25 -> (1,0)
    check(approx([s_q, c_q], [1.0, 0.0], tol=1e-6), "sin/cos at phi=0.25 -> (1,0)")

    # Degenerate clock (no strikes) -> unavailable, constant 0.
    empty = GaitPhaseClock([])
    check(not empty.available, "empty clock unavailable")
    check(empty.phase(123.4) == 0.0, "empty clock phase==0")
    one = GaitPhaseClock([2.0])
    check(not one.available, "single-strike clock unavailable")


# ---------------------------------------------------------------------------
# Part B: real env, absolute action mode + clock from prescribed GRF.
# ---------------------------------------------------------------------------
def part_b() -> CMCLikeProsthesisTrajectoryEnv:
    print("\n=== Part B: real env (prescribed GRF, action_mode=absolute) ===")
    env = CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            setup_xml_path=SETUP_XML,
            segment_duration=0.05,
            policy_knots=3,
            action_mode="absolute",
            episode_duration=1.5,
            grf_mode="prescribed",
            include_online_grf_observation=False,
            record_outputs=False,
            fail_fast=True,
        )
    )
    obs, info = env.reset(seed=1)
    names = env.observation_feature_names

    # (1) New gait-phase clock features are present in the observation.
    for feat in ("gait_phase", "gait_phase_sin", "gait_phase_cos"):
        check(feat in names, f"observation has '{feat}'")
    check(obs.shape[0] == len(names), "obs vector length matches feature names")

    # (2) Clock built from the prescribed sound-side (right) GRF.
    gc = info["gait_clock"]
    print(f"      gait_clock: {gc}")
    check(gc["available"], "clock available from prescribed right GRF")
    check(gc["n_cycles"] >= 10, f"detected many sound cycles (got {gc['n_cycles']})")
    check(
        0.5 <= gc["mean_period_s"] <= 2.0,
        f"plausible mean gait period (got {gc['mean_period_s']:.3f}s)",
    )

    # (3) Real clock alignment: phase ~0 at a detected heel strike, ~0.5 half a
    #     period later, and continuous/in-range over a sweep.
    clk = env._gait_clock
    hs = clk.heel_strike_times
    mid_idx = len(hs) // 2
    t_hs = float(hs[mid_idx])
    period = float(hs[mid_idx + 1] - hs[mid_idx])
    check(approx(clk.phase(t_hs), 0.0, tol=1e-6), "real phase==0 at a detected HS")
    check(
        abs(clk.phase(t_hs + 0.5 * period) - 0.5) < 1e-6,
        "real phase==0.5 half a period after HS",
    )
    ts = np.linspace(float(hs[0]), float(hs[-1]), 5000)
    phis = np.array([clk.phase(t) for t in ts])
    check(bool(np.all((phis >= 0.0) & (phis < 1.0))), "real phase in [0,1) over span")

    # (4) Absolute action mapping: action emits an ABSOLUTE trajectory over
    #     absolute_bounds_rad, NOT q_base + delta.
    pros = list(env.cfg.pros_coords)
    bounds = env.env_cfg.absolute_bounds_rad
    n = env.env_cfg.policy_knots
    target_t = min(env.t + env.env_cfg.segment_duration, env._episode_end)

    # zeros -> midpoint of each coord band.
    _, vals0, _ = env._action_to_segment(np.zeros((n, 2)), target_t)
    mid = [0.5 * (bounds[c][0] + bounds[c][1]) for c in pros]
    check(
        approx(vals0[1:], np.tile(mid, (n, 1)), tol=1e-9),
        f"absolute action 0 -> band midpoint {mid}",
    )
    # +1 -> high bound, -1 -> low bound (independent of prescribed IK).
    _, valsP, _ = env._action_to_segment(np.ones((n, 2)), target_t)
    _, valsM, _ = env._action_to_segment(-np.ones((n, 2)), target_t)
    high = [bounds[c][1] for c in pros]
    low = [bounds[c][0] for c in pros]
    check(approx(valsP[1:], np.tile(high, (n, 1)), tol=1e-9), f"absolute +1 -> high {high}")
    check(approx(valsM[1:], np.tile(low, (n, 1)), tol=1e-9), f"absolute -1 -> low {low}")

    # Absolute mapping must NOT equal q_base+delta (i.e. not the imitative path):
    # at action 0 the commanded knee equals the band midpoint, which differs from
    # the prescribed IK knee at the knot time.
    qb, _, _ = env.base_kin.get(target_t)
    knee = pros[0]
    check(
        abs(vals0[-1, 0] - mid[0]) < 1e-9 and abs(mid[0] - qb[knee]) > 1e-3,
        "absolute output is independent of prescribed IK (not a delta)",
    )

    # (5) One real end-to-end step still runs and stays consistent. Command a
    #     BENIGN absolute action (~hold the current pose): invert the absolute
    #     mapping for the current prosthetic q so the SEA is not asked to jump.
    #     (A naive action=0 maps knee to the band midpoint -0.75 rad, a ~0.6 rad
    #     jump from the start pose, which legitimately diverges in one segment.)
    def _abs_action_for(q_targets):
        a = np.empty((n, 2), dtype=float)
        for j, c in enumerate(pros):
            lo, hi = bounds[c]
            a[:, j] = np.clip(2.0 * (q_targets[j] - lo) / (hi - lo) - 1.0, -1.0, 1.0)
        return a

    q_cur = [obs[names.index(c)] for c in pros]
    benign = _abs_action_for(q_cur)
    obs2, reward, terminated, truncated, info2 = env.step(benign)
    check(np.all(np.isfinite(obs2)), "step obs finite")
    check(np.isfinite(reward), f"step reward finite (got {reward:.4f})")
    check(
        not terminated,
        f"benign hold-pose action does not diverge (end_reason={info2.get('end_reason')})",
    )
    check("gait_phase" in info2["observation"], "step info obs has gait_phase")
    p0 = obs[names.index("gait_phase")]
    p_now = info2["observation"]["gait_phase"]
    check(0.0 <= p_now < 1.0, f"step gait_phase in [0,1) (got {p_now:.4f})")
    check(abs(p_now - p0) > 1e-6, f"gait_phase advanced with time ({p0:.4f}->{p_now:.4f})")
    print(f"      reset gait_phase={p0:.4f} -> step={p_now:.4f}  end_reason={info2.get('end_reason')}")
    return env


# ---------------------------------------------------------------------------
# Part C: delta-mode regression (the imitative path still maps q_base+delta).
# ---------------------------------------------------------------------------
def part_c() -> None:
    print("\n=== Part C: delta-mode regression ===")
    env = CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            setup_xml_path=SETUP_XML,
            segment_duration=0.05,
            policy_knots=2,
            action_mode="delta",
            max_delta_rad=0.35,
            episode_duration=0.5,
            grf_mode="prescribed",
            include_online_grf_observation=False,
            record_outputs=False,
            fail_fast=True,
        )
    )
    env.reset(seed=1)
    pros = list(env.cfg.pros_coords)
    n = env.env_cfg.policy_knots
    target_t = min(env.t + env.env_cfg.segment_duration, env._episode_end)
    times, vals, _ = env._action_to_segment(np.ones((n, 2)), target_t)
    ok = True
    for k, tk in enumerate(times[1:]):
        qb, _, _ = env.base_kin.get(float(tk))
        for j, c in enumerate(pros):
            if abs(vals[k + 1, j] - (qb[c] + 0.35)) > 1e-9:
                ok = False
    check(ok, "delta action +1 -> q_base(t_k) + 0.35 rad (imitative path intact)")
    # delta clock features still present (clock is mode-independent).
    check("gait_phase" in env.observation_feature_names, "delta env also has gait_phase")


def part_d() -> None:
    """Reward + action wrapper chain in absolute mode, torch-free.

    Exercises the reward single-source-of-truth (reward_function.RewardShapingWrapper,
    the exact reward path training uses) plus an inline replica of
    FlattenClipAction (flat -> reshape -> clip), so the full training-facing
    transform is validated without importing the torch-laden env_factory."""
    print("\n=== Part D: reward + action wrapper chain (absolute, torch-free) ===")
    _bm = _TRAJ_GEN_DIR / "baseline_MLP"
    if str(_bm) not in sys.path:
        sys.path.append(str(_bm))
    import reward_function  # gymnasium + numpy only, no torch

    base = CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            setup_xml_path=SETUP_XML,
            segment_duration=0.05,
            policy_knots=3,
            action_mode="absolute",
            episode_duration=0.5,
            grf_mode="prescribed",
            include_online_grf_observation=False,
            record_outputs=False,
            fail_fast=True,
        )
    )
    env = reward_function.RewardShapingWrapper(base)  # reward seen by the agent
    obs, info = env.reset(seed=1)
    names = base.observation_feature_names
    check("gait_phase" in names, "wrapped env: obs has gait_phase")
    check(info.get("gait_clock", {}).get("available"), "wrapped env: clock available")
    check(base.env_cfg.action_mode == "absolute", "wrapped env action_mode is absolute")

    # Benign hold-pose absolute action, then exercise the FlattenClipAction
    # transform (flatten -> reshape -> clip) the training stack applies.
    pros = list(base.cfg.pros_coords)
    bounds = base.env_cfg.absolute_bounds_rad
    n = base.env_cfg.policy_knots
    q_cur = [obs[names.index(c)] for c in pros]
    a2d = np.empty((n, 2), dtype=np.float32)
    for j, c in enumerate(pros):
        lo, hi = bounds[c]
        a2d[:, j] = np.clip(2.0 * (q_cur[j] - lo) / (hi - lo) - 1.0, -1.0, 1.0)
    flat = a2d.reshape(-1)  # RLlib sees a flat Box (FlattenClipAction)
    reshaped = np.clip(flat.reshape(n, 2), base.action_space.low, base.action_space.high)

    o2, r, term, trunc, i2 = env.step(reshaped)
    check(np.all(np.isfinite(o2)), "wrapped step obs finite")
    check(np.isfinite(r), f"wrapped step reward finite ({r:.4f})")
    check("reward_components" in i2, "RewardShapingWrapper added reward_components")
    check("reward_terms" in i2, "env exposed reward_terms to the wrapper")
    check(
        isinstance(term, bool) and isinstance(trunc, bool),
        "wrapped step returns bool terminated/truncated",
    )
    env.close()


def main() -> int:
    part_a()
    part_b()
    part_c()
    part_d()
    print("\n=== SUMMARY ===")
    if _FAILS:
        print(f"FAILED ({len(_FAILS)}):")
        for m in _FAILS:
            print(f"  - {m}")
        return 1
    print("ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
