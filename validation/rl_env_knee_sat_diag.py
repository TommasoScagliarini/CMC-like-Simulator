"""
Per-step decomposition of the cascade SEA command for the trajectory RL env
(Trajectory Generator/osim_trj_cmc_like.py) on the AB06 SEA PI setup.

For each env step it decomposes the knee (and ankle, for contrast) cascade
command into its physical drivers:

    qdot_cas = qdot_ref + Kp_outer * e_q
    e_v      = qdot_cas - qdot_cur          (inner velocity error)
             = (qdot_ref - qdot_cur) + Kp_outer*e_q
             =     term_ref           +   term_pos
    u_raw    = (Kp_inner*e_v + Ki_inner*xi_v) / F_opt
             =     p_contrib          +   i_contrib

This shows whether saturation (|u|>=0.999) is driven by the raw reference
velocity error (term_ref), the outer position-error feedforward (term_pos),
the proportional inner term (p_contrib) or integral windup (i_contrib).

Background: reports/user/2026-06-01_knee_saturazione_env_rl_limit_cycle.md

Run:
    conda run --no-capture-output -n envCMC-like \
        python validation/rl_env_knee_sat_diag.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
# The RL adapter lives under "Trajectory Generator/"; keep repo root first so
# config/simulation_runner resolve against the canonical tree.
_TRAJ_GEN_DIR = REPO_ROOT / "Trajectory Generator"
if _TRAJ_GEN_DIR.is_dir() and str(_TRAJ_GEN_DIR) not in sys.path:
    sys.path.append(str(_TRAJ_GEN_DIR))

from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv  # noqa: E402

SETUP_XML = str(
    REPO_ROOT
    / "models"
    / "AB06_SEASEA_Threadmill"
    / "AB06_SEASEA_stiff321_500_pi_setup.xml"
)

SEG = 0.01
EPISODE = 0.30
AMPLITUDE = 0.10
SEED = 7


def scripted_actions(env, n_steps, amplitude, seed):
    rng = np.random.default_rng(seed)
    shape = tuple(env.action_space.shape)
    phases = rng.uniform(0.0, 2.0 * np.pi, size=shape)
    freqs = rng.uniform(0.5, 2.5, size=shape)
    actions = np.empty((n_steps, *shape), dtype=np.float32)
    for k in range(n_steps):
        actions[k] = amplitude * np.sin(freqs * (k + 1) + phases)
    return actions


def main():
    env = CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            setup_xml_path=SETUP_XML,
            segment_duration=SEG,
            episode_duration=EPISODE,
            policy_knots=1,
            action_mode="delta",
            random_init=False,
            rebuild_model_on_reset=False,
            record_outputs=False,
            fail_fast=True,
        )
    )

    coords = list(env.cfg.pros_coords)  # [pros_knee_angle, pros_ankle_angle]
    sea_names = [env.cfg.sea_knee_name, env.cfg.sea_ankle_name]
    gains = {}
    for coord, sea in zip(coords, sea_names):
        gains[coord] = {
            "Kpo": float(env.cfg.sea_cascade_kp_outer[coord]),
            "Kpi": float(env.cfg.sea_cascade_kp_inner[coord]),
            "Kii": float(env.cfg.sea_cascade_ki_inner[coord]),
            "Fopt": float(env.ctx.sea_f_opt.get(sea, 1.0)),
        }
    print("controller_mode =", env.cfg.sea_outer_controller_mode)
    for coord in coords:
        g = gains[coord]
        sens = g["Kpi"] / g["Fopt"]
        print(
            f"{coord}: Kp_outer={g['Kpo']:.3f}  Kp_inner={g['Kpi']:.3f}  "
            f"Ki_inner={g['Kii']:.1f}  F_opt={g['Fopt']:.0f}  "
            f"=> u-sens per rad/s of e_v = {sens:.3f}  "
            f"(e_v to saturate ~= {1.0/sens:.2f} rad/s)"
        )

    n_steps = int(round(EPISODE / SEG)) + 2
    env.reset(seed=0)
    actions = scripted_actions(env, n_steps, AMPLITUDE, SEED)

    hdr = (
        f"{'t':>6} {'cd':>5} "
        f"{'qd_ref':>8} {'qd_cur':>8} {'e_q':>7} "
        f"{'term_ref':>8} {'term_pos':>8} {'e_v':>8} "
        f"{'p_contr':>8} {'i_contr':>8} {'u':>7} {'sat':>3}"
    )
    rows = []
    sat_count = {c: 0 for c in coords}

    for k in range(len(actions)):
        obs, reward, terminated, truncated, info = env.step(actions[k])
        lsi = env.runner.last_step_info
        u_sea = lsi.get("u_sea", {})
        qdot_ref_raw = lsi.get("qdot_ref", {})
        t = lsi.get("time", info["time"])
        for coord in coords:
            g = gains[coord]
            qd_ref = float(qdot_ref_raw.get(coord, 0.0))
            qd_cas = float(u_sea.get(f"{coord}_cascade_qdot_ref", 0.0))
            e_v = float(u_sea.get(f"{coord}_cascade_velocity_error", 0.0))
            qd_cur = qd_cas - e_v
            term_pos = qd_cas - qd_ref          # = Kp_outer * e_q
            term_ref = qd_ref - qd_cur          # raw velocity tracking error
            e_q = term_pos / g["Kpo"] if g["Kpo"] != 0.0 else 0.0
            p_contrib = float(u_sea.get(f"{coord}_cascade_inner_p_cmd", 0.0)) / g["Fopt"]
            i_contrib = float(u_sea.get(f"{coord}_cascade_inner_i_cmd", 0.0)) / g["Fopt"]
            u = float(u_sea.get(coord, 0.0))
            sat = abs(u) >= 0.999
            if sat:
                sat_count[coord] += 1
            rows.append(
                (t, coord[5:8], qd_ref, qd_cur, e_q, term_ref, term_pos,
                 e_v, p_contrib, i_contrib, u, sat)
            )
        if terminated or truncated:
            break

    # Print first ~16 knee rows + summary so the console stays readable.
    print("\n" + hdr)
    knee_rows = [r for r in rows if r[1] == "kne"]
    ankle_rows = [r for r in rows if r[1] == "ank"]
    for r in knee_rows[:16]:
        t, cd, qd_ref, qd_cur, e_q, term_ref, term_pos, e_v, pc, ic, u, sat = r
        print(
            f"{t:6.3f} {cd:>5} {qd_ref:8.3f} {qd_cur:8.3f} {e_q:7.3f} "
            f"{term_ref:8.3f} {term_pos:8.3f} {e_v:8.3f} "
            f"{pc:8.3f} {ic:8.3f} {u:7.3f} {'Y' if sat else '.':>3}"
        )

    def summ(tag, rs):
        if not rs:
            return
        arr = np.array([[r[2], r[3], r[5], r[6], r[7], r[8], r[9], r[10]] for r in rs])
        # cols: qd_ref, qd_cur, term_ref, term_pos, e_v, p_contrib, i_contrib, u
        print(f"\n--- {tag} summary over {len(rs)} steps ---")
        labels = ["qd_ref", "qd_cur", "term_ref", "term_pos", "e_v",
                  "p_contrib", "i_contrib", "u"]
        for j, lab in enumerate(labels):
            col = arr[:, j]
            print(f"  {lab:>10}: mean={col.mean():8.3f}  "
                  f"absmax={np.abs(col).max():8.3f}")
        print(f"  saturated steps: {sat_count[tag]} / {len(rs)}")

    summ(coords[0], knee_rows)
    summ(coords[1], ankle_rows)

    env.close()


if __name__ == "__main__":
    main()
