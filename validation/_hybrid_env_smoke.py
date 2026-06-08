"""Minimal smoke for the hybrid GRF RL env (prescribed sound + online prosthetic)."""
from __future__ import annotations
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.append(str(REPO_ROOT / "Trajectory Generator"))

from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv

cfg = CMCEnvConfig(
    setup_xml_path="models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml",
    grf_mode="online_sensor",
    online_grf_profile_file=(
        "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json"
    ),
    online_grf_applied_sides=["left"],
    include_online_grf_observation=True,
    episode_duration=0.15,
    segment_duration=0.02,
    fail_fast=True,
)

env = CMCLikeProsthesisTrajectoryEnv(cfg)
obs, info = env.reset()
print("reset applied sides:", info.get("online_grf_applied_sides"))
print("reset prescribed disabled:", info.get("prescribed_grf_disabled_sides"))
print("ctx online force count:", len(getattr(env.ctx, "online_grf_force_paths", [])))

import numpy as np
for k in range(5):
    a = np.zeros(env.action_space.shape, dtype=np.float32)
    obs, r, term, trunc, info = env.step(a)
    rt = info["reward_terms"]
    grf = info.get("online_grf", {})
    lfy = grf.get("left", {}).get("normal_force") if isinstance(grf, dict) else None
    lpen = grf.get("left", {}).get("penetration") if isinstance(grf, dict) else None
    print(
        f"step {k}: r={r:.3f} term={term} trunc={trunc} end={info.get('end_reason')} "
        f"pen_m={rt.get('grf_penetration_m'):.4f} pen_loss={rt.get('grf_penetration_loss'):.4f} "
        f"left_Fy={lfy} left_pen={lpen}"
    )
env.close()
print("SMOKE OK")
