"""Ad-hoc validation diagnostics for the Codex-built RL trajectory generator.

Run from repo root in envCMC-like. Read-only: does not modify project code.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
for p in (REPO, REPO / "Trajectory Generator", REPO / "Trajectory Generator" / "Prosthesis_SNN"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))


def determinism_check() -> None:
    a = json.load(open(REPO / "runs/_val_train_tiny/summary.json"))
    b = json.load(open(REPO / "runs/_val_train_tiny2/summary.json"))
    keys = ["reward_min", "reward_max", "reward_mean", "reward_last", "best_score", "action_abs_max"]
    print("== determinism (tiny vs tiny2, same seed) ==")
    for k in keys:
        same = a[k] == b[k]
        print(f"  {k}: {a[k]!r} vs {b[k]!r} -> {'SAME' if same else 'DIFF'}")
    print("  episode_returns equal:", a["episode_returns"] == b["episode_returns"])


def setup_window() -> None:
    from setup_io import read_setup_xml
    s = read_setup_xml(str(REPO / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"))
    print("== setup window ==")
    print("  t_start", s.t_start)
    print("  t_end", s.t_end)
    print("  window_s", round(s.t_end - s.t_start, 5))


def termination_semantics() -> None:
    """Show the env labels the end-of-window as a Gymnasium truncation."""
    sys.path.insert(0, str(REPO / "Trajectory Generator"))
    from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv

    env = CMCLikeProsthesisTrajectoryEnv(CMCEnvConfig(
        setup_xml_path=str(REPO / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml"),
        segment_duration=0.005, episode_duration=0.01, policy_knots=3,
        action_mode="delta", rebuild_model_on_reset=False, fail_fast=False,
    ))
    obs, info = env.reset(seed=0)
    last = None
    for _ in range(5):
        a = np.zeros((3, 2), dtype=np.float32)  # zero action = stay near IK
        obs, reward, terminated, truncated, info = env.step(a)
        last = (terminated, truncated, info.get("time"))
        if terminated or truncated:
            break
    print("== env termination semantics (zero-action episode to its end) ==")
    print(f"  final step: terminated={last[0]} truncated={last[1]} t={last[2]:.4f}")
    print("  -> reaching the planned horizon sets truncated=True (a TIME LIMIT),")
    print("     while unsafe task states are reported as terminated=True.")
    env.close()


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "all"
    if cmd in ("all", "det"):
        determinism_check()
    if cmd in ("all", "win"):
        setup_window()
    if cmd in ("all", "term"):
        termination_semantics()
