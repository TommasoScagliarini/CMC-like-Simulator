"""Single-process timing of the hybrid env (no Ray): isolates per-step and
per-reset cost of the absolute + gait-clock env, to tell whether slow RLlib
iterations come from the env itself or from Ray/reset overhead.

Run: <envCMC-rllib python via conda run> validation/_env_timing.py
Temporary diagnostic (``_`` prefix).
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
_TRAJ_GEN_DIR = REPO_ROOT / "Trajectory Generator"
if _TRAJ_GEN_DIR.is_dir() and str(_TRAJ_GEN_DIR) not in sys.path:
    sys.path.append(str(_TRAJ_GEN_DIR))

from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv

SETUP_XML = "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"
PROFILE = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
)


def main() -> int:
    t0 = time.perf_counter()
    env = CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            setup_xml_path=SETUP_XML,
            segment_duration=0.01,        # like the real run
            policy_knots=3,
            action_mode="absolute",
            episode_duration=2.0,
            grf_mode="online_sensor",
            online_grf_profile_file=PROFILE,
            include_online_grf_observation=True,
            online_grf_applied_sides=["left"],
            record_outputs=False,
            fail_fast=True,
        )
    )
    build_s = time.perf_counter() - t0
    print(f"env build: {build_s:.2f} s")

    t0 = time.perf_counter()
    obs, info = env.reset(seed=0)
    reset_s = time.perf_counter() - t0
    gc = info.get("gait_clock", {})
    names = env.observation_feature_names
    print(f"reset #1: {reset_s:.2f} s | gait_clock available={gc.get('available')} "
          f"n_cycles={gc.get('n_cycles')} | has gait_phase={('gait_phase' in names)}")

    pros = list(env.cfg.pros_coords)
    bounds = env.env_cfg.absolute_bounds_rad
    n = env.env_cfg.policy_knots

    def hold_action():
        q_cur = [obs[names.index(c)] for c in pros]
        a = np.empty((n, 2), dtype=np.float32)
        for j, c in enumerate(pros):
            lo, hi = bounds[c]
            a[:, j] = np.clip(2.0 * (q_cur[j] - lo) / (hi - lo) - 1.0, -1.0, 1.0)
        return a

    # Time individual steps (benign hold-pose so they don't diverge/truncate).
    step_times = []
    n_steps = 20
    for i in range(n_steps):
        a = hold_action()
        t0 = time.perf_counter()
        obs, r, term, trunc, info = env.step(a)
        step_times.append(time.perf_counter() - t0)
        if term or trunc:
            print(f"  step {i}: ended early term={term} trunc={trunc} "
                  f"reason={info.get('end_reason')}")
            obs, info = env.reset(seed=i)
    step_times = np.array(step_times)
    print(f"steps: n={n_steps} mean={step_times.mean()*1000:.1f} ms "
          f"median={np.median(step_times)*1000:.1f} ms "
          f"max={step_times.max()*1000:.1f} ms")

    # Time a mid-stream reset (the cost that short episodes pay repeatedly).
    t0 = time.perf_counter()
    obs, info = env.reset(seed=99)
    reset2_s = time.perf_counter() - t0
    print(f"reset #2 (mid-stream): {reset2_s:.2f} s")

    # Extrapolate: steps to fill a 256-step batch + the resets a 6-step episode
    # config would incur, vs a 200-step episode config.
    per_step = float(step_times.mean())
    print(
        f"\nestimate for 256 steps: pure-step ~{256 * per_step:.0f}s; "
        f"+ resets if episode=6 steps (~43 resets) ~{43 * reset2_s:.0f}s; "
        f"+ resets if episode=200 steps (~2 resets) ~{2 * reset2_s:.0f}s"
    )
    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
