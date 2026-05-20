"""SEA driver isolation test suite - PI variant (5 test, 10 worker).

Stessa struttura di tools/driver_isolation_suite.py ma testa la formula PI con
clamp anti-windup attivo:

    e          = tau_ref - tau_spring
    xi         = integrale di e, clamped a +-integral_torque_limit/Ki
    tau_input  = (1+Kp)*tau_ref - Kp*tau_spring + Ki*xi - Kd*omega_m
                 (clamp +-500)
    tau_spring = K*(theta_m - theta_j)
    domega_m/dt = (tau_input - tau_spring - Bm*omega_m) / Jm
    dtheta_m/dt = omega_m
    dxi/dt     = e   (con anti-windup: 0 se al clamp e di segno concorde)

Stato ODE: 3 componenti [theta_m, omega_m, xi].

Modello sorgente: AB06_SEASEA_stiff321_500_pi.osim (contiene <Ki> + <integral_torque_limit>).
"""

from __future__ import annotations

import json
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Tuple

import numpy as np
from scipy.integrate import solve_ivp

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parent.parent
OSIM_PATH = REPO_ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"
PD_OSIM_PATH = REPO_ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim"
RESULTS_DIR = REPO_ROOT / "results/_driver_isolation_pi_20260517"
PLOT_DIR = REPO_ROOT / "plot/05_17_2026_driver_isolation_pi"

MAX_WORKERS = 10
TAU_CLAMP = 500.0


@dataclass(frozen=True)
class SEAParamsPI:
    name: str
    K: float
    Kp: float
    Kd: float
    Ki: float
    Bm: float
    Jm: float
    F_opt: float
    integral_torque_limit: float  # |Ki*xi|_max [Nm]

    @property
    def xi_clamp(self) -> float:
        """Limite del valore assoluto di xi (state integrale)."""
        return self.integral_torque_limit / self.Ki if self.Ki > 0 else float("inf")

    @property
    def omega_n_pd_equiv(self) -> float:
        """Frequenza naturale della parte PD (per confronto col baseline)."""
        return np.sqrt((1.0 + self.Kp) * self.K / self.Jm)

    @property
    def zeta_pd_equiv(self) -> float:
        return (self.Kd + self.Bm) / (2.0 * np.sqrt(self.Jm * (1.0 + self.Kp) * self.K))

    @property
    def f_d_pd_equiv(self) -> float:
        z = self.zeta_pd_equiv
        if z >= 1.0:
            return 0.0
        return self.omega_n_pd_equiv * np.sqrt(1.0 - z * z) / (2.0 * np.pi)

    @property
    def zero_freq_hz(self) -> float:
        """Frequenza dello zero PI: omega_z = Ki/(1+Kp), in Hz."""
        return self.Ki / ((1.0 + self.Kp) * 2.0 * np.pi)

    @property
    def routh_max_ki(self) -> float:
        """Limite di stabilita Routh: Ki < (Kd+Bm)*(1+Kp)/Jm."""
        return (self.Kd + self.Bm) * (1.0 + self.Kp) / self.Jm

    @property
    def stability_margin(self) -> float:
        """Margine relativo di Ki vs limite Routh (1.0 = al limite)."""
        return self.Ki / self.routh_max_ki


def read_sea_params_pi(osim_path: Path) -> Dict[str, SEAParamsPI]:
    tree = ET.parse(osim_path)
    root = tree.getroot()
    params: Dict[str, SEAParamsPI] = {}
    for sea in root.iter("SeriesElasticActuator"):
        name = sea.attrib.get("name", "")
        if name not in ("SEA_Knee", "SEA_Ankle"):
            continue

        def child_float(tag: str, default: float = None) -> float:
            node = sea.find(tag)
            if node is None or node.text is None:
                if default is not None:
                    return default
                raise KeyError(f"missing <{tag}> in {name}")
            return float(node.text)

        params[name] = SEAParamsPI(
            name=name,
            K=child_float("stiffness"),
            Kp=child_float("Kp"),
            Kd=child_float("Kd"),
            Ki=child_float("Ki"),
            Bm=child_float("motor_damping"),
            Jm=child_float("motor_inertia"),
            F_opt=child_float("optimal_force"),
            integral_torque_limit=child_float("integral_torque_limit"),
        )
    return params


def driver_rhs_pi(
    t: float,
    state: np.ndarray,
    p: SEAParamsPI,
    tau_ref_fn: Callable[[float], float],
    theta_j_fn: Callable[[float], float],
) -> np.ndarray:
    """ODE PI non-impedance abs-D con anti-windup sull'integratore."""
    theta_m, omega_m, xi = state[0], state[1], state[2]
    theta_j = theta_j_fn(t)
    tau_spring = p.K * (theta_m - theta_j)
    tau_ref = tau_ref_fn(t)
    e = tau_ref - tau_spring

    # Anti-windup: clamp xi e blocca integrazione se al limite di segno concorde
    xi_clamp = p.xi_clamp
    at_pos_clamp = xi >= xi_clamp - 1e-12 and e > 0
    at_neg_clamp = xi <= -xi_clamp + 1e-12 and e < 0
    xi_dot = 0.0 if (at_pos_clamp or at_neg_clamp) else e

    tau_input_raw = (1.0 + p.Kp) * tau_ref - p.Kp * tau_spring + p.Ki * xi - p.Kd * omega_m
    if tau_input_raw > TAU_CLAMP:
        tau_input = TAU_CLAMP
    elif tau_input_raw < -TAU_CLAMP:
        tau_input = -TAU_CLAMP
    else:
        tau_input = tau_input_raw

    omega_m_dot = (tau_input - tau_spring - p.Bm * omega_m) / p.Jm
    return np.array([omega_m, omega_m_dot, xi_dot])


def simulate(
    p: SEAParamsPI,
    tau_ref_fn: Callable[[float], float],
    theta_j_fn: Callable[[float], float],
    T: float,
    max_step: float,
    state0: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    rtol: float = 1e-7,
    atol: float = 1e-9,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Integra l'ODE PI e ricostruisce tau_spring sulla griglia uniforme."""
    sol = solve_ivp(
        driver_rhs_pi,
        (0.0, T),
        np.array(state0, dtype=float),
        method="RK45",
        max_step=max_step,
        rtol=rtol, atol=atol,
        args=(p, tau_ref_fn, theta_j_fn),
        dense_output=True,
    )
    t_grid = np.arange(0.0, T + max_step / 2, max_step)
    if t_grid[-1] > T:
        t_grid = t_grid[:-1]
    states = sol.sol(t_grid)
    theta_m = states[0]
    omega_m = states[1]
    xi = states[2]
    theta_j = np.array([theta_j_fn(t) for t in t_grid])
    tau_spring = p.K * (theta_m - theta_j)
    return t_grid, theta_m, omega_m, xi, tau_spring


# ---------------------------------------------------------------------------
# FdT analitiche PI (terzo ordine)
# ---------------------------------------------------------------------------
def H_tracking_PI(p: SEAParamsPI, f_hz: np.ndarray) -> np.ndarray:
    """tau_spring/tau_ref [-] analitica con PI."""
    s = 1j * 2.0 * np.pi * f_hz
    num = (1.0 + p.Kp) * p.K * s + p.Ki * p.K
    den = p.Jm * s ** 3 + (p.Kd + p.Bm) * s ** 2 + (1.0 + p.Kp) * p.K * s + p.Ki * p.K
    return num / den


def H_sensitivity_PI(p: SEAParamsPI, f_hz: np.ndarray) -> np.ndarray:
    """tau_spring/omega_j [Nm/(rad/s)] analitica con PI."""
    s = 1j * 2.0 * np.pi * f_hz
    num = -p.K * s * (p.Jm * s + (p.Kd + p.Bm))
    den = p.Jm * s ** 3 + (p.Kd + p.Bm) * s ** 2 + (1.0 + p.Kp) * p.K * s + p.Ki * p.K
    return num / den


def H_tracking_PD_baseline(p: SEAParamsPI, f_hz: np.ndarray) -> np.ndarray:
    """FdT PD usando gli stessi Kp,Kd (per confronto sovrapposto sui plot)."""
    s = 1j * 2.0 * np.pi * f_hz
    num = (1.0 + p.Kp) * p.K
    den = p.Jm * s ** 2 + (p.Kd + p.Bm) * s + (1.0 + p.Kp) * p.K
    return num / den


def H_sensitivity_PD_baseline(p: SEAParamsPI, f_hz: np.ndarray) -> np.ndarray:
    s = 1j * 2.0 * np.pi * f_hz
    num = -p.K * (p.Jm * s + (p.Kd + p.Bm))
    den = p.Jm * s ** 2 + (p.Kd + p.Bm) * s + (1.0 + p.Kp) * p.K
    return num / den


# ---------------------------------------------------------------------------
# Closed-loop pole computation (PI third-order)
# ---------------------------------------------------------------------------
def closed_loop_poles_pi(p: SEAParamsPI) -> np.ndarray:
    """Radici di Jm*s^3 + (Kd+Bm)*s^2 + (1+Kp)*K*s + Ki*K = 0."""
    coef = [p.Jm, p.Kd + p.Bm, (1.0 + p.Kp) * p.K, p.Ki * p.K]
    return np.roots(coef)


def dominant_pole_metrics(poles: np.ndarray) -> Dict[str, float]:
    """Estrai metriche del dominante complesso (con parte immaginaria)."""
    cplx = poles[np.abs(poles.imag) > 1e-6]
    if cplx.size == 0:
        return {"omega_n_hz": 0.0, "zeta": 1.0, "real_dom_hz": 0.0}
    # picca il complesso piu' vicino all'asse immaginario
    p_dom = cplx[np.argmax(cplx.real)]
    omega_n = float(np.abs(p_dom))
    zeta = float(-p_dom.real / omega_n)
    return {
        "omega_n_rad": omega_n,
        "omega_n_hz": omega_n / (2.0 * np.pi),
        "zeta": zeta,
        "real_dom_hz": float(p_dom.real / (2.0 * np.pi)),
    }


# ---------------------------------------------------------------------------
# Test 1 - step response PI, omega_j = 0
# ---------------------------------------------------------------------------
def test1_step_pi(p: SEAParamsPI) -> Dict:
    TAU_REF = 50.0
    T = 0.2  # un po piu' lungo del PD per vedere assestamento PI
    dt = 1e-5
    t, _, _, xi, tau_spring = simulate(
        p,
        tau_ref_fn=lambda t: TAU_REF if t >= 0.0 else 0.0,
        theta_j_fn=lambda t: 0.0,
        T=T, max_step=dt,
    )
    tail = (t >= 0.15) & (t <= 0.2)
    tau_ss = float(np.mean(tau_spring[tail]))
    xi_ss = float(np.mean(xi[tail]))
    e_ss = tau_ss - TAU_REF
    overshoot_pct = 100.0 * max(0.0, (tau_spring.max() - TAU_REF) / TAU_REF)
    band = 0.02 * TAU_REF
    settled = np.abs(tau_spring - TAU_REF) < band
    if np.any(settled):
        n = len(settled)
        ts_idx = n
        for i in range(n):
            if np.all(settled[i:]):
                ts_idx = i; break
        ts = float(t[ts_idx]) if ts_idx < n else float("nan")
    else:
        ts = float("nan")
    return {
        "joint": p.name,
        "tau_ref": TAU_REF,
        "tau_spring_ss": tau_ss,
        "xi_ss": xi_ss,
        "xi_clamp": p.xi_clamp,
        "e_ss_Nm": e_ss,
        "overshoot_pct": overshoot_pct,
        "settling_time_2pct_s": ts,
        "t": t.tolist(),
        "tau_spring": tau_spring.tolist(),
        "xi_series": xi.tolist(),
    }


def sinusoidal_fit(t: np.ndarray, y: np.ndarray, f: float) -> Tuple[float, float]:
    w = 2.0 * np.pi * f
    s = np.sin(w * t)
    c = np.cos(w * t)
    A_mat = np.column_stack([s, c])
    coef, *_ = np.linalg.lstsq(A_mat, y, rcond=None)
    a, b = coef
    amp = np.hypot(a, b)
    phase = np.arctan2(b, a)
    return float(amp), float(phase)


def test2_bode_pi_one(joint_name: str, p_dict: Dict, f_hz: float) -> Dict:
    p = SEAParamsPI(**p_dict)
    AMP = 10.0
    n_cycles_total = 16  # piu' cicli per dare tempo al PI di assestarsi
    n_cycles_tail = 6
    T = max(n_cycles_total / f_hz, 0.08)
    dt = max(min(1.0 / (50.0 * f_hz), 1e-4), 5e-6)
    tau_ref_fn = lambda t: AMP * np.sin(2.0 * np.pi * f_hz * t)
    t, _, _, _, tau_spring = simulate(
        p, tau_ref_fn, lambda t: 0.0, T=T, max_step=dt,
    )
    tail_start = T - n_cycles_tail / f_hz
    sel = t >= tail_start
    t_tail = t[sel]; y_tail = tau_spring[sel]
    amp, phase = sinusoidal_fit(t_tail, y_tail, f_hz)
    gain = amp / AMP
    gain_db = 20.0 * np.log10(gain) if gain > 0 else -200.0
    phase_deg = np.degrees(phase)
    phase_deg = ((phase_deg + 180.0) % 360.0) - 180.0
    return {
        "joint": joint_name,
        "f_hz": f_hz,
        "gain": gain,
        "gain_db": gain_db,
        "phase_deg": phase_deg,
    }


def test3_disturbance_pi_one(joint_name: str, p_dict: Dict, f_hz: float) -> Dict:
    p = SEAParamsPI(**p_dict)
    A_omega = 1.0
    n_cycles_total = 16
    n_cycles_tail = 6
    T = max(n_cycles_total / f_hz, 0.08)
    dt = max(min(1.0 / (50.0 * f_hz), 1e-4), 5e-6)
    omega = 2.0 * np.pi * f_hz
    theta_j_fn = lambda t: -A_omega / omega * np.cos(omega * t)
    t, _, _, _, tau_spring = simulate(
        p, lambda t: 0.0, theta_j_fn, T=T, max_step=dt,
    )
    tail_start = T - n_cycles_tail / f_hz
    sel = t >= tail_start
    amp_out, phase_out = sinusoidal_fit(t[sel], tau_spring[sel], f_hz)
    sensitivity = amp_out / A_omega
    return {
        "joint": joint_name,
        "f_hz": f_hz,
        "sensitivity_Nm_per_rad_s": sensitivity,
        "phase_deg": ((np.degrees(phase_out) + 180.0) % 360.0) - 180.0,
    }


def test4_bias_pi_one(joint_name: str, p_dict: Dict, omega_j_const: float) -> Dict:
    """PI dovrebbe portare e_ss → 0 anche con omega_j costante (vs PD bias_slope*omega_j)."""
    p = SEAParamsPI(**p_dict)
    TAU_REF = 100.0
    T = 3.0  # PI ha bisogno di tempo per saturare l'integratore
    dt = 1e-4
    t, _, _, xi, tau_spring = simulate(
        p,
        tau_ref_fn=lambda t: TAU_REF,
        theta_j_fn=lambda t: omega_j_const * t,
        T=T, max_step=dt,
    )
    tail = t >= 2.7
    tau_ss = float(np.mean(tau_spring[tail]))
    xi_ss = float(np.mean(xi[tail]))
    e_ss_pi = tau_ss - TAU_REF
    # bias PD analitico per confronto: -(Kd+Bm)*omega_j/(1+Kp)
    e_ss_pd_theory = -(p.Kd + p.Bm) / (1.0 + p.Kp) * omega_j_const
    at_clamp = abs(xi_ss) > 0.99 * p.xi_clamp
    return {
        "joint": joint_name,
        "omega_j_const": omega_j_const,
        "tau_spring_ss": tau_ss,
        "xi_ss": xi_ss,
        "xi_clamp": p.xi_clamp,
        "at_xi_clamp": bool(at_clamp),
        "e_ss_pi_Nm": e_ss_pi,
        "e_ss_pd_theory_Nm": e_ss_pd_theory,
        "rejection_factor_vs_pd": (
            abs(e_ss_pi) / abs(e_ss_pd_theory) if abs(e_ss_pd_theory) > 1e-9 else 0.0
        ),
    }


def test5_noise_pi(p: SEAParamsPI, seed: int = 42) -> Dict:
    T = 5.0
    dt = 1e-4
    fs = 1.0 / dt
    rng = np.random.default_rng(seed)
    sigma = 2.0
    t_grid = np.arange(0.0, T + dt / 2, dt)
    if t_grid[-1] > T:
        t_grid = t_grid[:-1]
    raw = rng.normal(0.0, sigma, size=t_grid.size)
    from scipy.signal import butter, filtfilt
    b, a = butter(4, 1000.0 / (fs / 2), btype="low")
    tau_ref_samples = filtfilt(b, a, raw)

    def tau_ref_fn(t: float) -> float:
        return float(np.interp(t, t_grid, tau_ref_samples))

    t, _, _, _, tau_spring = simulate(
        p, tau_ref_fn, lambda t: 0.0, T=T, max_step=dt,
    )
    from scipy.signal import welch
    f_w, Pxx = welch(tau_spring, fs=fs, nperseg=8192)
    f_w_ref, Pxx_ref = welch(tau_ref_samples, fs=fs, nperseg=8192)
    sel = (f_w >= 50) & (f_w <= 150)
    if np.any(sel):
        i = int(np.argmax(Pxx[sel]))
        f_peak = float(f_w[sel][i])
        p_peak = float(Pxx[sel][i])
    else:
        f_peak = 0.0; p_peak = 0.0
    flat = (f_w >= 10) & (f_w <= 30)
    p_flat = float(np.median(Pxx[flat])) if np.any(flat) else 0.0
    peaking_db = 10.0 * np.log10(p_peak / p_flat) if p_flat > 0 and p_peak > 0 else float("nan")
    # frequenza dominante del peak della FdT analitica
    f_analytic = np.linspace(1, 200, 4000)
    H = H_tracking_PI(p, f_analytic)
    mag_db = 20.0 * np.log10(np.abs(H))
    i_pk = int(np.argmax(mag_db))
    f_pk_analytic = float(f_analytic[i_pk])
    mag_pk_analytic = float(mag_db[i_pk])
    return {
        "joint": p.name,
        "f_grid": f_w.tolist(),
        "Pxx_spring": Pxx.tolist(),
        "Pxx_ref": Pxx_ref.tolist(),
        "f_peak_50_150_Hz": f_peak,
        "peaking_above_flat_10_30_Hz_dB": peaking_db,
        "f_d_pd_equiv_Hz": p.f_d_pd_equiv,
        "f_peak_analytic_Hz": f_pk_analytic,
        "mag_peak_analytic_dB": mag_pk_analytic,
    }


def dispatch_sweep_job_pi(args):
    kind = args[0]
    if kind == "test2":
        return ("test2", test2_bode_pi_one(args[1], args[2], args[3]))
    if kind == "test3":
        return ("test3", test3_disturbance_pi_one(args[1], args[2], args[3]))
    if kind == "test4":
        return ("test4", test4_bias_pi_one(args[1], args[2], args[3]))
    raise ValueError(kind)


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_test1_pi(metrics: Dict, p: SEAParamsPI, out_path: Path):
    t = np.array(metrics["t"]); y = np.array(metrics["tau_spring"])
    xi = np.array(metrics["xi_series"])
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    ax1.plot(t * 1000, y, label=r"$\tau_{spring}(t)$ PI", color="C0")
    ax1.axhline(metrics["tau_ref"], color="k", linestyle="--", alpha=0.6, label=r"$\tau_{ref}$")
    ax1.axhline(metrics["tau_ref"] * 1.02, color="0.7", linestyle=":", lw=0.8)
    ax1.axhline(metrics["tau_ref"] * 0.98, color="0.7", linestyle=":", lw=0.8, label="banda 2%")
    if np.isfinite(metrics["settling_time_2pct_s"]):
        ax1.axvline(metrics["settling_time_2pct_s"] * 1000, color="r", linestyle=":", alpha=0.6,
                    label=f"settling 2% = {metrics['settling_time_2pct_s']*1000:.1f} ms")
    ax1.set_ylabel(r"$\tau_{spring}$ [Nm]")
    ax1.set_title(f"Test 1 PI - Step response {p.name} | Kp={p.Kp} Kd={p.Kd} Ki={p.Ki}\n"
                  f"e_ss={metrics['e_ss_Nm']:.4f} Nm, overshoot={metrics['overshoot_pct']:.1f}%, "
                  f"xi_ss={metrics['xi_ss']:.3f} (clamp {p.xi_clamp:.3f})")
    ax1.legend(loc="lower right"); ax1.grid(True, alpha=0.3)

    ax2.plot(t * 1000, xi, color="C1", label=r"$\xi(t)$ (integral state)")
    ax2.axhline(p.xi_clamp, color="r", linestyle=":", alpha=0.5, label=f"+/- xi_clamp = {p.xi_clamp:.3f}")
    ax2.axhline(-p.xi_clamp, color="r", linestyle=":", alpha=0.5)
    ax2.set_xlabel("t [ms]"); ax2.set_ylabel(r"$\xi$ [Nm*s]")
    ax2.legend(); ax2.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=110); plt.close(fig)


def plot_test2_pi(records: List[Dict], p: SEAParamsPI, out_path: Path):
    f = np.array([r["f_hz"] for r in records])
    g_db = np.array([r["gain_db"] for r in records])
    phi = np.array([r["phase_deg"] for r in records])
    order = np.argsort(f); f = f[order]; g_db = g_db[order]; phi = phi[order]
    f_th = np.logspace(np.log10(f[0]), np.log10(f[-1]), 400)
    H_pi = H_tracking_PI(p, f_th)
    H_pd = H_tracking_PD_baseline(p, f_th)
    g_pi_db = 20.0 * np.log10(np.abs(H_pi))
    g_pd_db = 20.0 * np.log10(np.abs(H_pd))
    phi_pi = np.degrees(np.angle(H_pi))
    phi_pd = np.degrees(np.angle(H_pd))

    fig, (ax_m, ax_p) = plt.subplots(2, 1, figsize=(9, 7.5), sharex=True)
    ax_m.semilogx(f, g_db, "o", label="simulato (PI)", color="C0", ms=4)
    ax_m.semilogx(f_th, g_pi_db, "-", label="analitico PI", color="C0", alpha=0.7)
    ax_m.semilogx(f_th, g_pd_db, "--", label="analitico PD baseline", color="C3", alpha=0.6)
    ax_m.axhline(-3, color="0.7", linestyle=":", lw=0.8, label="-3 dB")
    ax_m.axvline(p.f_d_pd_equiv, color="r", linestyle=":", alpha=0.4,
                 label=f"f_d (PD-equiv) = {p.f_d_pd_equiv:.0f} Hz")
    ax_m.axvline(p.zero_freq_hz, color="g", linestyle=":", alpha=0.4,
                 label=f"PI zero ~Ki/(2pi(1+Kp)) = {p.zero_freq_hz:.2f} Hz")
    ax_m.set_ylabel("|H| [dB]")
    pi_peak = float(np.max(g_pi_db))
    pd_peak = float(np.max(g_pd_db))
    ax_m.set_title(f"Test 2 PI - Bode {p.name} | PI peak={pi_peak:+.2f} dB vs PD={pd_peak:+.2f} dB | "
                   f"stab margin Ki/Ki_max = {p.stability_margin:.3f}")
    ax_m.legend(loc="lower left", fontsize=8); ax_m.grid(True, which="both", alpha=0.3)

    ax_p.semilogx(f, phi, "o", label="simulato (PI)", color="C0", ms=4)
    ax_p.semilogx(f_th, phi_pi, "-", label="analitico PI", color="C0", alpha=0.7)
    ax_p.semilogx(f_th, phi_pd, "--", label="analitico PD baseline", color="C3", alpha=0.6)
    ax_p.axvline(p.f_d_pd_equiv, color="r", linestyle=":", alpha=0.4)
    ax_p.axvline(p.zero_freq_hz, color="g", linestyle=":", alpha=0.4)
    ax_p.set_xlabel("f [Hz]"); ax_p.set_ylabel(r"$\angle H$ [deg]")
    ax_p.legend(fontsize=8); ax_p.grid(True, which="both", alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=110); plt.close(fig)


def plot_test3_pi(records: List[Dict], p: SEAParamsPI, out_path: Path):
    f = np.array([r["f_hz"] for r in records])
    s_obs = np.array([r["sensitivity_Nm_per_rad_s"] for r in records])
    order = np.argsort(f); f = f[order]; s_obs = s_obs[order]
    f_th = np.logspace(np.log10(f[0]), np.log10(f[-1]), 400)
    H_pi = H_sensitivity_PI(p, f_th)
    H_pd = H_sensitivity_PD_baseline(p, f_th)
    s_pi = np.abs(H_pi)
    s_pd = np.abs(H_pd)
    bias_dc_pd = abs((p.Kd + p.Bm) / (1.0 + p.Kp))
    fig, ax = plt.subplots(figsize=(9, 5.5))
    ax.loglog(f, s_obs, "o", label="simulato (PI)", color="C0", ms=5)
    ax.loglog(f_th, s_pi, "-", label="analitico PI", color="C0", alpha=0.7)
    ax.loglog(f_th, s_pd, "--", label="analitico PD baseline", color="C3", alpha=0.6)
    ax.axhline(bias_dc_pd, color="g", linestyle=":", alpha=0.5,
               label=f"PD bias DC = {bias_dc_pd:.3f}  (PI -> 0 a DC)")
    ax.axvline(p.f_d_pd_equiv, color="r", linestyle=":", alpha=0.4,
               label=f"f_d (PD-equiv) = {p.f_d_pd_equiv:.0f} Hz")
    ax.set_xlabel("f [Hz]"); ax.set_ylabel(r"$|\tau_{spring} / \omega_j|$ [Nm/(rad/s)]")
    ax.set_title(f"Test 3 PI - Sensitivita disturbo {p.name}")
    ax.legend(fontsize=8); ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=110); plt.close(fig)


def plot_test4_pi(records: List[Dict], p: SEAParamsPI, out_path: Path):
    om = np.array([r["omega_j_const"] for r in records])
    e_pi = np.array([r["e_ss_pi_Nm"] for r in records])
    e_pd = np.array([r["e_ss_pd_theory_Nm"] for r in records])
    at_clamp = np.array([r["at_xi_clamp"] for r in records])
    order = np.argsort(om); om = om[order]; e_pi = e_pi[order]; e_pd = e_pd[order]; at_clamp = at_clamp[order]
    fig, ax = plt.subplots(figsize=(9, 5.5))
    ax.plot(om, e_pi, "o", color="C0", ms=9, label="PI osservato")
    ax.plot(om, e_pd, "x", color="C3", ms=9, label="PD teoria (baseline)")
    om_fine = np.linspace(0, om.max() * 1.05, 100)
    bias_slope_pd = -(p.Kd + p.Bm) / (1.0 + p.Kp)
    ax.plot(om_fine, bias_slope_pd * om_fine, "--", color="C3", alpha=0.5,
            label=f"PD teoria continua = {bias_slope_pd:.3f}*omega_j")
    ax.axhline(0, color="k", linestyle="-", lw=0.5)
    for o, e, c in zip(om, e_pi, at_clamp):
        if c:
            ax.annotate("xi saturato", (o, e), textcoords="offset points",
                        xytext=(5, 8), fontsize=8, color="r")
    ax.set_xlabel(r"$\omega_j$ [rad/s]"); ax.set_ylabel(r"$e_{ss}$ [Nm]")
    ax.set_title(f"Test 4 PI - Bias a velocita costante {p.name} | tau_ref=100 Nm | "
                 f"xi clamp = +/-{p.xi_clamp:.3f}")
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=110); plt.close(fig)


def plot_test5_pi(metrics: Dict, p: SEAParamsPI, out_path: Path):
    f = np.array(metrics["f_grid"])
    Pxx = np.array(metrics["Pxx_spring"])
    Pxx_ref = np.array(metrics["Pxx_ref"])
    sel = (f >= 1) & (f <= 1000)
    fig, ax = plt.subplots(figsize=(9, 5.5))
    ax.semilogx(f[sel], 10 * np.log10(Pxx[sel] + 1e-20), label=r"PSD $\tau_{spring}$ (PI)", color="C0")
    ax.semilogx(f[sel], 10 * np.log10(Pxx_ref[sel] + 1e-20), label=r"PSD $\tau_{ref}$ (input)", color="0.5", alpha=0.6)
    ax.axvline(p.f_d_pd_equiv, color="r", linestyle=":", alpha=0.5,
               label=f"f_d (PD-equiv) = {p.f_d_pd_equiv:.1f} Hz")
    if metrics["f_peak_50_150_Hz"] > 0:
        ax.axvline(metrics["f_peak_50_150_Hz"], color="m", linestyle=":", alpha=0.5,
                   label=f"picco osservato = {metrics['f_peak_50_150_Hz']:.1f} Hz")
    ax.axvline(metrics["f_peak_analytic_Hz"], color="g", linestyle=":", alpha=0.5,
               label=f"picco analitico H_PI = {metrics['f_peak_analytic_Hz']:.1f} Hz "
                     f"({metrics['mag_peak_analytic_dB']:+.2f} dB)")
    ax.set_xlabel("f [Hz]"); ax.set_ylabel("PSD [dB / Hz]")
    pk = metrics["peaking_above_flat_10_30_Hz_dB"]
    ax.set_title(f"Test 5 PI - Risposta a noise broadband {p.name} | "
                 f"peaking osservato vs flat = {pk:.2f} dB | "
                 f"peak FdT analitico = {metrics['mag_peak_analytic_dB']:+.2f} dB")
    ax.legend(fontsize=8); ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=110); plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"\nReading PI SEA params from: {OSIM_PATH}")
    seas = read_sea_params_pi(OSIM_PATH)
    for name, p in seas.items():
        poles = closed_loop_poles_pi(p)
        dom = dominant_pole_metrics(poles)
        print(f"\n  {name}")
        print(f"    K={p.K} Kp={p.Kp} Kd={p.Kd} Ki={p.Ki} Bm={p.Bm} Jm={p.Jm} F_opt={p.F_opt}")
        print(f"    integral_torque_limit={p.integral_torque_limit} -> xi_clamp={p.xi_clamp:.4f}")
        print(f"    PD-equivalent omega_n={p.omega_n_pd_equiv:.2f} rad/s, zeta_pd={p.zeta_pd_equiv:.3f}, "
              f"f_d_pd={p.f_d_pd_equiv:.2f} Hz")
        print(f"    PI zero freq = Ki/(2pi(1+Kp)) = {p.zero_freq_hz:.2f} Hz")
        print(f"    Routh: Ki_max = (Kd+Bm)*(1+Kp)/Jm = {p.routh_max_ki:.1f}, "
              f"stability margin Ki/Ki_max = {p.stability_margin:.4f}")
        print(f"    Closed-loop poles: {poles}")
        print(f"    Dominant complex pole: omega_n={dom['omega_n_hz']:.2f} Hz, zeta={dom['zeta']:.3f}")

    summary = {
        "start": time.time(),
        "seas": {n: {**p.__dict__, "poles": [str(x) for x in closed_loop_poles_pi(p)]}
                 for n, p in seas.items()},
    }

    # ---------- Test 1 ----------
    print("\n[Test 1 PI] Step response  (seriale)")
    t0 = time.time()
    test1_results = {n: test1_step_pi(p) for n, p in seas.items()}
    print(f"  elapsed = {time.time() - t0:.2f} s")
    with open(RESULTS_DIR / "test1_step_metrics.json", "w") as fh:
        json.dump({n: {k: v for k, v in m.items() if k not in ("t", "tau_spring", "xi_series")}
                   for n, m in test1_results.items()}, fh, indent=2)
    for n, m in test1_results.items():
        plot_test1_pi(m, seas[n], PLOT_DIR / f"01_step_response_{n.split('_')[-1].lower()}.png")
        print(f"  {n:10s}  e_ss={m['e_ss_Nm']:.4f} Nm  overshoot={m['overshoot_pct']:.2f}%  "
              f"settling={m['settling_time_2pct_s']*1000:.2f} ms  xi_ss={m['xi_ss']:.4f}")

    # ---------- Test 2 + 3 + 4 parallelo ----------
    print(f"\n[Test 2+3+4 PI] Coda parallela con {MAX_WORKERS} worker (longest-job-first)")
    jobs_sweep = []
    bode_freqs = np.logspace(0, np.log10(500), 50)
    for n, p in seas.items():
        for f in bode_freqs:
            cost = 1.0 / max(f, 1.0)
            jobs_sweep.append((("test2", n, p.__dict__, float(f)), cost))
    dist_freqs = [0.5, 1, 2, 5, 10, 20, 50, 88, 150, 300]
    for n, p in seas.items():
        for f in dist_freqs:
            cost = 1.0 / max(f, 1.0)
            jobs_sweep.append((("test3", n, p.__dict__, float(f)), cost))
    omega_j_vals = [1.0, 2.0, 5.0, 10.0]
    for n, p in seas.items():
        for om in omega_j_vals:
            cost = 3.0  # T=3s in test4 PI
            jobs_sweep.append((("test4", n, p.__dict__, float(om)), cost))

    print(f"  totale job sweep: {len(jobs_sweep)}")
    jobs_sweep.sort(key=lambda jc: -jc[1])

    t0 = time.time()
    bode_records: List[Dict] = []
    dist_records: List[Dict] = []
    bias_records: List[Dict] = []
    with ProcessPoolExecutor(max_workers=MAX_WORKERS) as ex:
        futures = [ex.submit(dispatch_sweep_job_pi, args) for args, _ in jobs_sweep]
        done = 0
        for fut in as_completed(futures):
            kind, rec = fut.result()
            if kind == "test2":
                bode_records.append(rec)
            elif kind == "test3":
                dist_records.append(rec)
            elif kind == "test4":
                bias_records.append(rec)
            done += 1
            if done % 20 == 0 or done == len(jobs_sweep):
                print(f"  progresso: {done}/{len(jobs_sweep)}  ({time.time() - t0:.1f} s)")
    print(f"  totale parallelo elapsed = {time.time() - t0:.2f} s")

    np.savez(RESULTS_DIR / "test2_bode_data.npz",
             knee_f=np.array([r["f_hz"] for r in bode_records if r["joint"] == "SEA_Knee"]),
             knee_gain_db=np.array([r["gain_db"] for r in bode_records if r["joint"] == "SEA_Knee"]),
             knee_phase_deg=np.array([r["phase_deg"] for r in bode_records if r["joint"] == "SEA_Knee"]),
             ankle_f=np.array([r["f_hz"] for r in bode_records if r["joint"] == "SEA_Ankle"]),
             ankle_gain_db=np.array([r["gain_db"] for r in bode_records if r["joint"] == "SEA_Ankle"]),
             ankle_phase_deg=np.array([r["phase_deg"] for r in bode_records if r["joint"] == "SEA_Ankle"]))
    np.savez(RESULTS_DIR / "test3_disturbance_data.npz",
             knee_f=np.array([r["f_hz"] for r in dist_records if r["joint"] == "SEA_Knee"]),
             knee_sens=np.array([r["sensitivity_Nm_per_rad_s"] for r in dist_records if r["joint"] == "SEA_Knee"]),
             ankle_f=np.array([r["f_hz"] for r in dist_records if r["joint"] == "SEA_Ankle"]),
             ankle_sens=np.array([r["sensitivity_Nm_per_rad_s"] for r in dist_records if r["joint"] == "SEA_Ankle"]))
    with open(RESULTS_DIR / "test4_bias_data.json", "w") as fh:
        json.dump(bias_records, fh, indent=2)

    for n, p in seas.items():
        recs = [r for r in bode_records if r["joint"] == n]
        plot_test2_pi(recs, p, PLOT_DIR / f"02_bode_{n.split('_')[-1].lower()}.png")
        recs = [r for r in dist_records if r["joint"] == n]
        plot_test3_pi(recs, p, PLOT_DIR / f"03_disturbance_{n.split('_')[-1].lower()}.png")
        recs = [r for r in bias_records if r["joint"] == n]
        plot_test4_pi(recs, p, PLOT_DIR / f"04_bias_{n.split('_')[-1].lower()}.png")

    print("\n  Test 4 PI - confronto con PD:")
    for n, p in seas.items():
        recs = [r for r in bias_records if r["joint"] == n]
        max_e_pi = max(abs(r["e_ss_pi_Nm"]) for r in recs)
        max_e_pd = max(abs(r["e_ss_pd_theory_Nm"]) for r in recs)
        any_clamp = any(r["at_xi_clamp"] for r in recs)
        print(f"    {n:10s}  max|e_pi|={max_e_pi:.4f} Nm  max|e_pd|={max_e_pd:.4f} Nm  "
              f"reduction = {(1 - max_e_pi/max_e_pd)*100 if max_e_pd > 0 else 0:.1f}%  "
              f"xi_clamp_hit={'YES' if any_clamp else 'no'}")

    # Statistiche Bode: cerca peak
    print("\n  Test 2 PI - peak risonante nella tracking transfer:")
    for n, p in seas.items():
        recs = sorted([r for r in bode_records if r["joint"] == n], key=lambda x: x["f_hz"])
        f_arr = np.array([r["f_hz"] for r in recs])
        g_arr = np.array([r["gain_db"] for r in recs])
        sel = (f_arr >= 30) & (f_arr <= 200)
        if np.any(sel):
            i_pk = int(np.argmax(g_arr[sel]))
            print(f"    {n:10s}  picco simulato {f_arr[sel][i_pk]:.1f} Hz @ {g_arr[sel][i_pk]:+.3f} dB")

    # ---------- Test 5 ----------
    print("\n[Test 5 PI] Broadband noise  (seriale)")
    t0 = time.time()
    test5_results = {n: test5_noise_pi(p) for n, p in seas.items()}
    print(f"  elapsed = {time.time() - t0:.2f} s")
    for n, m in test5_results.items():
        np.savez(RESULTS_DIR / f"test5_noise_spectrum_{n.split('_')[-1].lower()}.npz",
                 f=np.array(m["f_grid"]),
                 Pxx_spring=np.array(m["Pxx_spring"]),
                 Pxx_ref=np.array(m["Pxx_ref"]))
        plot_test5_pi(m, seas[n], PLOT_DIR / f"05_noise_spectrum_{n.split('_')[-1].lower()}.png")
        print(f"  {n:10s}  picco osservato = {m['f_peak_50_150_Hz']:.2f} Hz "
              f"(PD-equiv {m['f_d_pd_equiv_Hz']:.2f} Hz)  "
              f"peaking simulato = {m['peaking_above_flat_10_30_Hz_dB']:.2f} dB  "
              f"peak analitico H_PI = {m['mag_peak_analytic_dB']:+.2f} dB @ {m['f_peak_analytic_Hz']:.1f} Hz")

    summary["test1"] = {n: {k: v for k, v in m.items() if k not in ("t", "tau_spring", "xi_series")}
                        for n, m in test1_results.items()}
    summary["test5"] = {n: {k: v for k, v in m.items()
                            if k not in ("f_grid", "Pxx_spring", "Pxx_ref")}
                        for n, m in test5_results.items()}
    summary["wall_time_total_s"] = time.time() - summary["start"]
    with open(RESULTS_DIR / "summary.json", "w") as fh:
        json.dump(summary, fh, indent=2, default=str)

    print(f"\nDone. Wall time totale = {summary['wall_time_total_s']:.2f} s")
    print(f"Output dati: {RESULTS_DIR}")
    print(f"Output plot: {PLOT_DIR}")


if __name__ == "__main__":
    main()
