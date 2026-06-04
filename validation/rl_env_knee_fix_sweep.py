"""
Remedies 2 and 3 for the knee saturating limit-cycle in the trajectory RL env,
with 1 ms (control-rate) metrics, on the AB06 SEA PI setup.

Remedy 2 (reference excitation): sweep segment_duration and the knee
            max_delta_rad. Pure env-config, no controller change.
Remedy 3 (gain-vs-timing / filter): sweep the knee output-command LPF
            (sea_u_lpf_cutoff_hz), an existing reversible controller knob.

Part 1 also characterises the oscillation frequency from the 1 ms knee command
series (sign-flip rate + FFT peak) to verify it is a discrete-time instability.

Background: reports/user/2026-06-01_knee_saturazione_env_rl_limit_cycle.md

Run:
    conda run --no-capture-output -n envCMC-like \
        python validation/rl_env_knee_fix_sweep.py
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

from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv  # noqa: E402

SETUP_XML = str(
    REPO_ROOT
    / "models"
    / "AB06_SEASEA_Threadmill"
    / "AB06_SEASEA_stiff321_500_pi_setup.xml"
)
EPISODE = 0.20
AMPLITUDE = 0.10
SEED = 7
DT = 0.001  # control/integration step (T_control)


def scripted_actions(env, n_steps, amplitude=AMPLITUDE, seed=SEED):
    rng = np.random.default_rng(seed)
    shape = tuple(env.action_space.shape)
    phases = rng.uniform(0.0, 2.0 * np.pi, size=shape)
    freqs = rng.uniform(0.5, 2.5, size=shape)
    actions = np.empty((n_steps, *shape), dtype=np.float32)
    for k in range(n_steps):
        actions[k] = amplitude * np.sin(freqs * (k + 1) + phases)
    return actions


def make_env(seg, max_delta_knee, max_delta_ankle=0.35):
    # The knee command LPF (remedy 3) is not a CMCEnvConfig field; it lives on
    # the SimulatorConfig and is set on env.cfg after construction.
    return CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            setup_xml_path=SETUP_XML,
            segment_duration=seg,
            episode_duration=EPISODE,
            policy_knots=1,
            action_mode="delta",
            max_delta_rad={
                "pros_knee_angle": max_delta_knee,
                "pros_ankle_angle": max_delta_ankle,
            },
            random_init=False,
            rebuild_model_on_reset=False,
            record_outputs=True,
            save_outputs_on_close=False,
            fail_fast=True,
        )
    )


def set_knee_lpf(env, fc):
    cutoff = dict(getattr(env.cfg, "sea_u_lpf_cutoff_hz", {}) or {})
    cutoff["pros_knee_angle"] = float(fc)
    env.cfg.sea_u_lpf_cutoff_hz = cutoff


def metrics_from_recorder(env):
    rec = env.runner._recorder  # noqa: SLF001
    n = rec.step_count
    knee_col = list(env.ctx.coord_names).index("pros_knee_angle")
    u_knee = np.asarray(rec._rec_sea_controls[:n, 0], dtype=float)  # noqa: SLF001
    qd_knee = np.asarray(rec._rec_qdot[:n, knee_col], dtype=float)  # noqa: SLF001
    u_knee = u_knee[np.isfinite(u_knee)]
    qd_knee = qd_knee[np.isfinite(qd_knee)]
    sat_frac = float(np.mean(np.abs(u_knee) >= 0.999)) if u_knee.size else 0.0
    return {
        "samples": int(u_knee.size),
        "sat_frac": sat_frac,
        "maxabs_u": float(np.abs(u_knee).max()) if u_knee.size else 0.0,
        "maxabs_qd": float(np.abs(qd_knee).max()) if qd_knee.size else 0.0,
        "u_series": u_knee,
    }


def rollout(env):
    seg = env.env_cfg.segment_duration
    n_steps = int(round(EPISODE / seg)) + 2
    env.reset(seed=0)
    actions = scripted_actions(env, n_steps)
    rewards = []
    for k in range(len(actions)):
        _, reward, terminated, truncated, _ = env.step(actions[k])
        rewards.append(float(reward))
        if terminated or truncated:
            break
    m = metrics_from_recorder(env)
    m["mean_reward"] = float(np.mean(rewards)) if rewards else 0.0
    m["env_steps"] = len(rewards)
    return m


def freq_characterization(u):
    """Sign-flip rate and FFT peak of the 1 ms command series."""
    u = np.asarray(u, dtype=float)
    if u.size < 4:
        return {"signflip_hz": 0.0, "fft_peak_hz": 0.0}
    s = np.sign(u)
    s[s == 0] = 1
    flips = int(np.sum(np.abs(np.diff(s)) > 0))
    duration = (u.size - 1) * DT
    signflip_hz = (flips / 2.0) / duration if duration > 0 else 0.0
    spec = np.abs(np.fft.rfft(u - u.mean()))
    freqs = np.fft.rfftfreq(u.size, DT)
    fft_peak_hz = float(freqs[1 + int(np.argmax(spec[1:]))]) if spec.size > 1 else 0.0
    return {"signflip_hz": float(signflip_hz), "fft_peak_hz": fft_peak_hz}


def row(label, m):
    print(
        f"{label:<22} steps={m['env_steps']:>3}  1ms_samples={m['samples']:>4}  "
        f"sat_frac={m['sat_frac']*100:5.1f}%  max|u|={m['maxabs_u']:.3f}  "
        f"max|qd_cur|={m['maxabs_qd']:7.2f}  mean_R={m['mean_reward']:.4f}"
    )


def main():
    print("=== Part 1: baseline + oscillation frequency (seg=0.01, delta_knee=0.35) ===")
    env = make_env(seg=0.01, max_delta_knee=0.35)
    base = rollout(env)
    fc = freq_characterization(base["u_series"])
    row("baseline", base)
    print(
        f"  oscillation: sign-flip rate ~= {fc['signflip_hz']:.1f} Hz, "
        f"FFT peak ~= {fc['fft_peak_hz']:.1f} Hz  (control rate = {1/DT:.0f} Hz, "
        f"Nyquist = {0.5/DT:.0f} Hz)"
    )
    env.close()

    print("\n=== Part 2a: remedy 2 - segment_duration sweep (delta_knee=0.35) ===")
    for seg in (0.01, 0.02, 0.05):
        env = make_env(seg=seg, max_delta_knee=0.35)
        row(f"seg={seg:.3f}", rollout(env))
        env.close()

    print("\n=== Part 2b: remedy 2 - knee max_delta_rad sweep (seg=0.01) ===")
    for d in (0.35, 0.15, 0.05, 0.02):
        env = make_env(seg=0.01, max_delta_knee=d)
        row(f"max_delta_knee={d:.2f}", rollout(env))
        env.close()

    print("\n=== Part 3: remedy 3 - knee output-command LPF sweep (seg=0.01, delta=0.35) ===")
    env = make_env(seg=0.01, max_delta_knee=0.35)
    for cut in (0.0, 80.0, 50.0, 30.0, 20.0, 10.0):
        set_knee_lpf(env, cut)
        label = "lpf=off" if cut == 0.0 else f"lpf={cut:.0f}Hz"
        row(label, rollout(env))
    env.close()


if __name__ == "__main__":
    main()
