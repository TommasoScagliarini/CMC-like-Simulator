"""Long, gait-scale rollout analyzer for the Codex RL trajectory generator.

Drives the env with an exported reference checkpoint and records per-step
diagnostics to characterize integration stability over a long horizon:
SEA command saturation, tracking error, pelvis height, action magnitude,
and the Gymnasium episode-end reason.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
for p in (REPO / "Trajectory Generator" / "Prosthesis_SNN",
          REPO / "Trajectory Generator", REPO):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import prosthesis_snn  # noqa: F401  (DLL workaround, before numpy/MKL)
import torch  # noqa: F401
import numpy as np

from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv
from prosthesis_snn.generator import ReferenceGenerator


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--setup-xml", required=True)
    ap.add_argument("--episode-duration", type=float, default=3.5)
    ap.add_argument("--segment-duration", type=float, default=0.01)
    ap.add_argument("--timesteps", type=int, default=400)
    args = ap.parse_args()

    gen = ReferenceGenerator.from_checkpoint(args.checkpoint, device="cpu")
    action_shape = gen.cfg.action_shape

    env = CMCLikeProsthesisTrajectoryEnv(CMCEnvConfig(
        setup_xml_path=args.setup_xml,
        segment_duration=args.segment_duration,
        episode_duration=args.episode_duration,
        policy_knots=action_shape[0],
        action_mode="delta",
        max_delta_rad=0.35,
        rebuild_model_on_reset=False,
        record_outputs=False,
        fail_fast=False,
    ))
    obs, info = env.reset(seed=123)

    rows = []
    end_step = None
    end_reason = None
    for k in range(args.timesteps):
        feats = info.get("observation", {})
        action = gen.predict_action(feats, action_shape=action_shape)
        obs, reward, terminated, truncated, info = env.step(action)
        rt = info.get("reward_terms", {})
        o = info.get("observation", {})
        rows.append({
            "t": info.get("time"),
            "reward": reward,
            "u_abs_max": rt.get("u_abs_max", float("nan")),
            "u_sat_frac": rt.get("u_saturation_fraction", float("nan")),
            "tracking_loss": rt.get("tracking_loss", float("nan")),
            "reference_loss": rt.get("reference_loss", float("nan")),
            "pelvis_ty": o.get("pelvis_ty", float("nan")),
            "knee": o.get("pros_knee_angle", float("nan")),
            "ankle": o.get("pros_ankle_angle", float("nan")),
            "action_abs_max": float(np.max(np.abs(action))),
        })
        if terminated or truncated:
            end_step = k
            end_reason = info.get(
                "end_reason",
                "terminated" if terminated else "truncated",
            )
            break

    arr = {key: np.array([r[key] for r in rows], dtype=float) for key in rows[0]}
    n = len(rows)
    print(f"== long rollout: {n} steps, dur~{n*args.segment_duration:.2f}s ==")
    print(f"  episode end: reason={end_reason} step={end_step}")
    print(f"  reward            mean {np.nanmean(arr['reward']):.4f}  min {np.nanmin(arr['reward']):.4f}")
    print(f"  u_abs_max         max  {np.nanmax(arr['u_abs_max']):.4f}  mean {np.nanmean(arr['u_abs_max']):.4f}")
    print(f"  u_saturation_frac max  {np.nanmax(arr['u_sat_frac']):.4f}  mean {np.nanmean(arr['u_sat_frac']):.4f}")
    sat_steps = int(np.sum(arr['u_sat_frac'] > 0.0))
    print(f"  steps with any SEA saturation: {sat_steps}/{n}")
    print(f"  tracking_loss     max  {np.nanmax(arr['tracking_loss']):.4e}  mean {np.nanmean(arr['tracking_loss']):.4e}")
    print(f"  reference_loss    max  {np.nanmax(arr['reference_loss']):.4e}  mean {np.nanmean(arr['reference_loss']):.4e}")
    print(f"  pelvis_ty         min  {np.nanmin(arr['pelvis_ty']):.4f}  (fall threshold 0.55)")
    print(f"  knee q [rad]      min {np.nanmin(arr['knee']):.3f}  max {np.nanmax(arr['knee']):.3f}")
    print(f"  ankle q [rad]     min {np.nanmin(arr['ankle']):.3f}  max {np.nanmax(arr['ankle']):.3f}")
    print(f"  action_abs_max    max  {np.nanmax(arr['action_abs_max']):.4f}  (action space bound 1.0)")
    print(f"  finite throughout: reward={bool(np.all(np.isfinite(arr['reward'])))} "
          f"knee={bool(np.all(np.isfinite(arr['knee'])))} ankle={bool(np.all(np.isfinite(arr['ankle'])))}")
    env.close()


if __name__ == "__main__":
    main()
