"""SEA driver isolation test suite (5 test, 10 worker).

Verifica in isolamento dal sistema completo che il motor driver SEA non-impedance
si comporti come previsto dalla teoria. Replica esattamente la formula attiva
in output.py:471-476:

    tau_input = (1+Kp)*tau_ref - Kp*tau_spring - Kd*omega_m   (clamp +/-500)
    tau_spring = K*(theta_m - theta_j)
    domega_m/dt = (tau_input - tau_spring - Bm*omega_m) / Jm
    dtheta_m/dt = omega_m

I 5 test:
  1. Step response a giunto fermo
  2. Bode magnitude/phase (sweep 1-500 Hz)
  3. Disturbance rejection (sweep omega_j sinusoidale)
  4. Bias a omega_j costante (cammino-like)
  5. Risposta a noise broadband

Parallelismo: 10 worker, coda condivisa longest-job-first per Test 2+3+4.
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
OSIM_PATH = REPO_ROOT / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim"
RESULTS_DIR = REPO_ROOT / "results/_driver_isolation_20260516"
PLOT_DIR = REPO_ROOT / "plot/05_16_2026_3_driver_isolation"

MAX_WORKERS = 10
TAU_CLAMP = 500.0


@dataclass(frozen=True)
class SEAParams:
    name: str
    K: float       # Nm/rad
    Kp: float      # adim (gain inner)
    Kd: float      # Nm*s/rad (inner damping gain)
    Bm: float      # Nm*s/rad (motor viscous damping)
    Jm: float      # kg*m^2
    F_opt: float   # Nm

    @property
    def omega_n(self) -> float:
        return np.sqrt((1.0 + self.Kp) * self.K / self.Jm)

    @property
    def zeta(self) -> float:
        return (self.Kd + self.Bm) / (2.0 * np.sqrt(self.Jm * (1.0 + self.Kp) * self.K))

    @property
    def f_d(self) -> float:
        z = self.zeta
        if z >= 1.0:
            return 0.0
        return self.omega_n * np.sqrt(1.0 - z * z) / (2.0 * np.pi)

    @property
    def bias_slope(self) -> float:
        """tau_spring_ss - tau_ref = -(Kd+Bm)*omega_j/(1+Kp); restituisce coefficient di omega_j."""
        return -(self.Kd + self.Bm) / (1.0 + self.Kp)


def read_sea_params_from_osim(osim_path: Path) -> Dict[str, SEAParams]:
    """Parser XML standalone (senza OpenSim) dei due blocchi SeriesElasticActuator."""
    tree = ET.parse(osim_path)
    root = tree.getroot()
    params: Dict[str, SEAParams] = {}
    for sea in root.iter("SeriesElasticActuator"):
        name = sea.attrib.get("name", "")
        if name not in ("SEA_Knee", "SEA_Ankle"):
            continue
        def child_float(tag: str) -> float:
            node = sea.find(tag)
            if node is None or node.text is None:
                raise KeyError(f"missing <{tag}> in {name}")
            return float(node.text)
        params[name] = SEAParams(
            name=name,
            K=child_float("stiffness"),
            Kp=child_float("Kp"),
            Kd=child_float("Kd"),
            Bm=child_float("motor_damping"),
            Jm=child_float("motor_inertia"),
            F_opt=child_float("optimal_force"),
        )
    return params


def driver_rhs(
    t: float,
    state: np.ndarray,
    p: SEAParams,
    tau_ref_fn: Callable[[float], float],
    theta_j_fn: Callable[[float], float],
) -> np.ndarray:
    """ODE non-impedance abs-D, formula identica a output.py:471-476."""
    theta_m, omega_m = state[0], state[1]
    theta_j = theta_j_fn(t)
    tau_spring = p.K * (theta_m - theta_j)
    tau_ref = tau_ref_fn(t)
    tau_input_raw = (1.0 + p.Kp) * tau_ref - p.Kp * tau_spring - p.Kd * omega_m
    if tau_input_raw > TAU_CLAMP:
        tau_input = TAU_CLAMP
    elif tau_input_raw < -TAU_CLAMP:
        tau_input = -TAU_CLAMP
    else:
        tau_input = tau_input_raw
    omega_m_dot = (tau_input - tau_spring - p.Bm * omega_m) / p.Jm
    return np.array([omega_m, omega_m_dot])


def simulate(
    p: SEAParams,
    tau_ref_fn: Callable[[float], float],
    theta_j_fn: Callable[[float], float],
    T: float,
    max_step: float,
    state0: Tuple[float, float] = (0.0, 0.0),
    rtol: float = 1e-7,
    atol: float = 1e-9,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Integra l'ODE e ricostruisce tau_spring sulla griglia uniforme dt = max_step."""
    sol = solve_ivp(
        driver_rhs,
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
    theta_j = np.array([theta_j_fn(t) for t in t_grid])
    tau_spring = p.K * (theta_m - theta_j)
    return t_grid, theta_m, omega_m, tau_spring


# ---------------------------------------------------------------------------
# Test 1 - step response, omega_j = 0
# ---------------------------------------------------------------------------
def test1_step(p: SEAParams) -> Dict:
    TAU_REF = 50.0
    T = 0.1
    dt = 1e-5
    t, _, _, tau_spring = simulate(
        p,
        tau_ref_fn=lambda t: TAU_REF if t >= 0.0 else 0.0,
        theta_j_fn=lambda t: 0.0,
        T=T, max_step=dt,
    )
    tail = (t >= 0.05) & (t <= 0.1)
    tau_ss = float(np.mean(tau_spring[tail]))
    e_ss = tau_ss - TAU_REF
    overshoot_pct = 100.0 * max(0.0, (tau_spring.max() - TAU_REF) / TAU_REF)
    # settling time 2% (relativo a tau_ref)
    band = 0.02 * TAU_REF
    settled = np.abs(tau_spring - TAU_REF) < band
    if np.any(settled):
        # primo istante in cui rimane settled per il resto
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
        "e_ss_Nm": e_ss,
        "overshoot_pct": overshoot_pct,
        "settling_time_2pct_s": ts,
        "t": t.tolist(),
        "tau_spring": tau_spring.tolist(),
    }


# ---------------------------------------------------------------------------
# Helpers Bode: fit sinusoide a + b cos su una coda
# ---------------------------------------------------------------------------
def sinusoidal_fit(t: np.ndarray, y: np.ndarray, f: float) -> Tuple[float, float]:
    """Restituisce (amplitude, phase_rad) di y(t) ~ A*sin(2pi*f*t + phi)."""
    w = 2.0 * np.pi * f
    s = np.sin(w * t)
    c = np.cos(w * t)
    # least-squares: y = a*s + b*c
    A_mat = np.column_stack([s, c])
    coef, *_ = np.linalg.lstsq(A_mat, y, rcond=None)
    a, b = coef
    amp = np.hypot(a, b)
    phase = np.arctan2(b, a)
    return float(amp), float(phase)


# ---------------------------------------------------------------------------
# Test 2 - Bode, sweep di tau_ref sinusoidale
# ---------------------------------------------------------------------------
def test2_bode_one(joint_name: str, p_dict_pickleable: Dict, f_hz: float) -> Dict:
    p = SEAParams(**p_dict_pickleable)
    AMP = 10.0
    n_cycles_total = 12
    n_cycles_tail = 5
    T = max(n_cycles_total / f_hz, 0.05)
    # dt = 1/(50*f)  (50 samples per ciclo) ma cap inferiore e superiore
    dt = max(min(1.0 / (50.0 * f_hz), 1e-4), 5e-6)
    tau_ref_fn = lambda t: AMP * np.sin(2.0 * np.pi * f_hz * t)
    t, _, _, tau_spring = simulate(
        p, tau_ref_fn, lambda t: 0.0, T=T, max_step=dt,
    )
    tail_start = T - n_cycles_tail / f_hz
    sel = t >= tail_start
    t_tail = t[sel]; y_tail = tau_spring[sel]
    amp, phase = sinusoidal_fit(t_tail, y_tail, f_hz)
    # input ref = AMP * sin -> fase 0
    gain = amp / AMP
    gain_db = 20.0 * np.log10(gain) if gain > 0 else -200.0
    phase_deg = np.degrees(phase)
    # wrap fase nel range [-180, 180]
    phase_deg = ((phase_deg + 180.0) % 360.0) - 180.0
    return {
        "joint": joint_name,
        "f_hz": f_hz,
        "gain": gain,
        "gain_db": gain_db,
        "phase_deg": phase_deg,
    }


# ---------------------------------------------------------------------------
# Test 3 - disturbance rejection
# ---------------------------------------------------------------------------
def test3_disturbance_one(joint_name: str, p_dict_pickleable: Dict, f_hz: float) -> Dict:
    p = SEAParams(**p_dict_pickleable)
    A_omega = 1.0  # rad/s
    n_cycles_total = 12
    n_cycles_tail = 5
    T = max(n_cycles_total / f_hz, 0.05)
    dt = max(min(1.0 / (50.0 * f_hz), 1e-4), 5e-6)
    # theta_j(t) = -A/(2*pi*f)*cos(...) di modo che omega_j = A*sin
    omega = 2.0 * np.pi * f_hz
    theta_j_fn = lambda t: -A_omega / omega * np.cos(omega * t)
    t, _, _, tau_spring = simulate(
        p, lambda t: 0.0, theta_j_fn, T=T, max_step=dt,
    )
    tail_start = T - n_cycles_tail / f_hz
    sel = t >= tail_start
    # disturbo riferimento: omega_j(t) = A * sin(omega*t)
    amp_out, phase_out = sinusoidal_fit(t[sel], tau_spring[sel], f_hz)
    # sensitivita' |tau_spring| / |omega_j|
    sensitivity = amp_out / A_omega
    return {
        "joint": joint_name,
        "f_hz": f_hz,
        "sensitivity_Nm_per_rad_s": sensitivity,
        "phase_deg": ((np.degrees(phase_out) + 180.0) % 360.0) - 180.0,
    }


# ---------------------------------------------------------------------------
# Test 4 - bias a omega_j costante
# ---------------------------------------------------------------------------
def test4_bias_one(joint_name: str, p_dict_pickleable: Dict, omega_j_const: float) -> Dict:
    p = SEAParams(**p_dict_pickleable)
    TAU_REF = 100.0
    T = 1.0
    dt = 1e-4
    t, _, _, tau_spring = simulate(
        p,
        tau_ref_fn=lambda t: TAU_REF,
        theta_j_fn=lambda t: omega_j_const * t,
        T=T, max_step=dt,
    )
    tail = t >= 0.9
    tau_ss = float(np.mean(tau_spring[tail]))
    e_ss = tau_ss - TAU_REF
    e_ss_theory = p.bias_slope * omega_j_const
    return {
        "joint": joint_name,
        "omega_j_const": omega_j_const,
        "tau_spring_ss": tau_ss,
        "e_ss_observed_Nm": e_ss,
        "e_ss_theory_Nm": e_ss_theory,
        "residuo_Nm": e_ss - e_ss_theory,
    }


# ---------------------------------------------------------------------------
# Test 5 - broadband noise
# ---------------------------------------------------------------------------
def test5_noise(p: SEAParams, seed: int = 42) -> Dict:
    T = 5.0
    dt = 1e-4
    fs = 1.0 / dt
    n = int(round(T / dt))
    rng = np.random.default_rng(seed)
    sigma = 2.0
    # white noise gaussiano direttamente sulla griglia fine dt=1e-4 (fs=10 kHz)
    # filtrato Butterworth lowpass a 1 kHz per avere banda piatta su 0-500 Hz
    t_grid = np.arange(0.0, T + dt / 2, dt)
    if t_grid[-1] > T:
        t_grid = t_grid[:-1]
    raw = rng.normal(0.0, sigma, size=t_grid.size)
    from scipy.signal import butter, filtfilt
    b, a = butter(4, 1000.0 / (fs / 2), btype="low")
    tau_ref_samples = filtfilt(b, a, raw)
    # callable da una griglia
    def tau_ref_fn(t: float) -> float:
        return float(np.interp(t, t_grid, tau_ref_samples))
    t, _, _, tau_spring = simulate(
        p, tau_ref_fn, lambda t: 0.0, T=T, max_step=dt,
    )
    # PSD via Welch (banda fino a Nyquist 5 kHz, focus 1-500)
    from scipy.signal import welch
    f_w, Pxx = welch(tau_spring, fs=fs, nperseg=8192)
    f_w_ref, Pxx_ref = welch(tau_ref_samples, fs=fs, nperseg=8192)
    # picco in banda 50-150 Hz
    sel = (f_w >= 50) & (f_w <= 150)
    if np.any(sel):
        i = int(np.argmax(Pxx[sel]))
        f_peak = float(f_w[sel][i])
        p_peak = float(Pxx[sel][i])
    else:
        f_peak = 0.0; p_peak = 0.0
    # flat-band reference (10-30 Hz)
    flat = (f_w >= 10) & (f_w <= 30)
    p_flat = float(np.median(Pxx[flat])) if np.any(flat) else 0.0
    peaking_db = 10.0 * np.log10(p_peak / p_flat) if p_flat > 0 and p_peak > 0 else float("nan")
    return {
        "joint": p.name,
        "f_grid": f_w.tolist(),
        "Pxx_spring": Pxx.tolist(),
        "Pxx_ref": Pxx_ref.tolist(),
        "f_peak_50_150_Hz": f_peak,
        "peaking_above_flat_10_30_Hz_dB": peaking_db,
        "f_d_theory_Hz": p.f_d,
    }


# ---------------------------------------------------------------------------
# Worker dispatcher (module-level per essere pickle-friendly)
# ---------------------------------------------------------------------------
def dispatch_sweep_job(args):
    kind = args[0]
    if kind == "test2":
        return ("test2", test2_bode_one(args[1], args[2], args[3]))
    if kind == "test3":
        return ("test3", test3_disturbance_one(args[1], args[2], args[3]))
    if kind == "test4":
        return ("test4", test4_bias_one(args[1], args[2], args[3]))
    raise ValueError(kind)


# ---------------------------------------------------------------------------
# Analytical reference transfer functions
# ---------------------------------------------------------------------------
def H_tracking(p: SEAParams, f_hz: np.ndarray) -> np.ndarray:
    """tau_spring/tau_ref [-] analitica."""
    s = 1j * 2.0 * np.pi * f_hz
    num = (1.0 + p.Kp) * p.K
    den = p.Jm * s * s + (p.Kd + p.Bm) * s + (1.0 + p.Kp) * p.K
    return num / den


def H_sensitivity(p: SEAParams, f_hz: np.ndarray) -> np.ndarray:
    """tau_spring/omega_j [Nm/(rad/s)] analitica."""
    s = 1j * 2.0 * np.pi * f_hz
    num = -p.K * (p.Jm * s + (p.Kd + p.Bm))
    den = p.Jm * s * s + (p.Kd + p.Bm) * s + (1.0 + p.Kp) * p.K
    return num / den


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_test1(metrics: Dict, p: SEAParams, out_path: Path):
    t = np.array(metrics["t"]); y = np.array(metrics["tau_spring"])
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(t * 1000, y, label=r"$\tau_{spring}(t)$", color="C0")
    ax.axhline(metrics["tau_ref"], color="k", linestyle="--", alpha=0.6, label=r"$\tau_{ref}$")
    ax.axhline(metrics["tau_ref"] * 1.02, color="0.7", linestyle=":", lw=0.8)
    ax.axhline(metrics["tau_ref"] * 0.98, color="0.7", linestyle=":", lw=0.8, label="banda 2%")
    if np.isfinite(metrics["settling_time_2pct_s"]):
        ax.axvline(metrics["settling_time_2pct_s"] * 1000, color="r", linestyle=":", alpha=0.6,
                   label=f"settling 2% = {metrics['settling_time_2pct_s']*1000:.1f} ms")
    ax.set_xlabel("t [ms]"); ax.set_ylabel(r"$\tau_{spring}$ [Nm]")
    ax.set_title(f"Test 1 - Step response {p.name} | Kp={p.Kp} Kd={p.Kd} | "
                 f"omega_n={p.omega_n:.0f} rad/s, zeta={p.zeta:.2f}\n"
                 f"e_ss={metrics['e_ss_Nm']:.3f} Nm, overshoot={metrics['overshoot_pct']:.1f}%, "
                 f"settling={metrics['settling_time_2pct_s']*1000:.1f} ms (teorico 9 ms)")
    ax.legend(loc="lower right"); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=110); plt.close(fig)


def plot_test2(records: List[Dict], p: SEAParams, out_path: Path):
    f = np.array([r["f_hz"] for r in records])
    g_db = np.array([r["gain_db"] for r in records])
    phi = np.array([r["phase_deg"] for r in records])
    order = np.argsort(f); f = f[order]; g_db = g_db[order]; phi = phi[order]
    f_th = np.logspace(np.log10(f[0]), np.log10(f[-1]), 400)
    H_th = H_tracking(p, f_th)
    g_th_db = 20.0 * np.log10(np.abs(H_th))
    phi_th = np.degrees(np.angle(H_th))
    fig, (ax_m, ax_p) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    ax_m.semilogx(f, g_db, "o-", label="simulato", color="C0", ms=4)
    ax_m.semilogx(f_th, g_th_db, "--", label="analitico", color="k", alpha=0.6)
    ax_m.axhline(-3, color="0.7", linestyle=":", lw=0.8, label="-3 dB")
    ax_m.axvline(p.f_d, color="r", linestyle=":", alpha=0.4, label=f"f_d={p.f_d:.0f} Hz")
    ax_m.set_ylabel("|H| [dB]")
    ax_m.set_title(f"Test 2 - Bode {p.name} | omega_n={p.omega_n:.0f} rad/s, zeta={p.zeta:.2f}, f_d={p.f_d:.1f} Hz")
    ax_m.legend(); ax_m.grid(True, which="both", alpha=0.3)
    ax_p.semilogx(f, phi, "o-", label="simulato", color="C0", ms=4)
    ax_p.semilogx(f_th, phi_th, "--", label="analitico", color="k", alpha=0.6)
    ax_p.axvline(p.f_d, color="r", linestyle=":", alpha=0.4)
    ax_p.set_xlabel("f [Hz]"); ax_p.set_ylabel(r"$\angle H$ [deg]")
    ax_p.legend(); ax_p.grid(True, which="both", alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=110); plt.close(fig)


def plot_test3(records: List[Dict], p: SEAParams, out_path: Path):
    f = np.array([r["f_hz"] for r in records])
    s_obs = np.array([r["sensitivity_Nm_per_rad_s"] for r in records])
    order = np.argsort(f); f = f[order]; s_obs = s_obs[order]
    f_th = np.logspace(np.log10(f[0]), np.log10(f[-1]), 400)
    H_th = H_sensitivity(p, f_th)
    s_th = np.abs(H_th)
    bias_dc = abs(p.bias_slope)
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.loglog(f, s_obs, "o-", label="simulato", color="C0", ms=5)
    ax.loglog(f_th, s_th, "--", label="analitico", color="k", alpha=0.6)
    ax.axhline(bias_dc, color="g", linestyle=":", alpha=0.5,
               label=f"bias DC teorico = {bias_dc:.3f} Nm/(rad/s)")
    ax.axvline(p.f_d, color="r", linestyle=":", alpha=0.4, label=f"f_d={p.f_d:.0f} Hz")
    ax.set_xlabel("f [Hz]"); ax.set_ylabel(r"$|\tau_{spring} / \omega_j|$ [Nm/(rad/s)]")
    ax.set_title(f"Test 3 - Sensitivita' al disturbo omega_j {p.name}")
    ax.legend(); ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=110); plt.close(fig)


def plot_test4(records: List[Dict], p: SEAParams, out_path: Path):
    om = np.array([r["omega_j_const"] for r in records])
    e_obs = np.array([r["e_ss_observed_Nm"] for r in records])
    e_th = np.array([r["e_ss_theory_Nm"] for r in records])
    order = np.argsort(om); om = om[order]; e_obs = e_obs[order]; e_th = e_th[order]
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(om, e_obs, "o", color="C0", ms=8, label="osservato")
    om_fine = np.linspace(0, om.max() * 1.05, 100)
    ax.plot(om_fine, p.bias_slope * om_fine, "--", color="k", alpha=0.6,
            label=f"teoria: -(Kd+Bm)*omega_j/(1+Kp) = {p.bias_slope:.3f}*omega_j")
    ax.set_xlabel(r"$\omega_j$ [rad/s]"); ax.set_ylabel(r"$e_{ss} = \tau_{spring,ss} - \tau_{ref}$ [Nm]")
    ax.set_title(f"Test 4 - Bias a velocita' costante {p.name} | tau_ref=100 Nm")
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=110); plt.close(fig)


def plot_test5(metrics: Dict, p: SEAParams, out_path: Path):
    f = np.array(metrics["f_grid"])
    Pxx = np.array(metrics["Pxx_spring"])
    Pxx_ref = np.array(metrics["Pxx_ref"])
    sel = (f >= 1) & (f <= 1000)
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.semilogx(f[sel], 10 * np.log10(Pxx[sel] + 1e-20), label=r"PSD $\tau_{spring}$", color="C0")
    ax.semilogx(f[sel], 10 * np.log10(Pxx_ref[sel] + 1e-20), label=r"PSD $\tau_{ref}$ (input)", color="0.5", alpha=0.6)
    ax.axvline(p.f_d, color="r", linestyle=":", alpha=0.5, label=f"f_d teorico = {p.f_d:.1f} Hz")
    if metrics["f_peak_50_150_Hz"] > 0:
        ax.axvline(metrics["f_peak_50_150_Hz"], color="m", linestyle=":", alpha=0.5,
                   label=f"picco osservato = {metrics['f_peak_50_150_Hz']:.1f} Hz")
    ax.set_xlabel("f [Hz]"); ax.set_ylabel("PSD [dB / Hz]")
    pk = metrics["peaking_above_flat_10_30_Hz_dB"]
    ax.set_title(f"Test 5 - Risposta a noise broadband {p.name} | "
                 f"peaking risonanza vs flat = {pk:.1f} dB")
    ax.legend(); ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=110); plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"\nReading SEA params from: {OSIM_PATH}")
    seas = read_sea_params_from_osim(OSIM_PATH)
    for name, p in seas.items():
        print(f"  {name:10s}  K={p.K} Kp={p.Kp} Kd={p.Kd} Bm={p.Bm} Jm={p.Jm} F_opt={p.F_opt}")
        print(f"             omega_n={p.omega_n:.2f} rad/s, zeta={p.zeta:.3f}, f_d={p.f_d:.2f} Hz, bias_slope={p.bias_slope:.4f}")

    summary = {"start": time.time(), "seas": {n: p.__dict__ for n, p in seas.items()}}

    # ---------- Test 1 ----------
    print("\n[Test 1] Step response  (seriale)")
    t0 = time.time()
    test1_results = {n: test1_step(p) for n, p in seas.items()}
    print(f"  elapsed = {time.time() - t0:.2f} s")
    with open(RESULTS_DIR / "test1_step_metrics.json", "w") as fh:
        json.dump({n: {k: v for k, v in m.items() if k not in ("t", "tau_spring")}
                   for n, m in test1_results.items()}, fh, indent=2)
    for n, m in test1_results.items():
        plot_test1(m, seas[n], PLOT_DIR / f"01_step_response_{n.split('_')[-1].lower()}.png")
        print(f"  {n:10s}  e_ss={m['e_ss_Nm']:.4f} Nm  overshoot={m['overshoot_pct']:.2f}%  settling={m['settling_time_2pct_s']*1000:.2f} ms")

    # ---------- Test 2 + 3 + 4 in coda condivisa parallela ----------
    print(f"\n[Test 2+3+4] Coda parallela con {MAX_WORKERS} worker (longest-job-first)")
    # Costruzione job list per i 3 test
    jobs_sweep = []

    # Test 2 - Bode: 50 freq logspaced 1-500 Hz, per ogni SEA
    bode_freqs = np.logspace(0, np.log10(500), 50)
    for n, p in seas.items():
        for f in bode_freqs:
            cost = 1.0 / max(f, 1.0)  # piu' basso -> sim piu' lunga
            jobs_sweep.append((("test2", n, p.__dict__, float(f)), cost))

    # Test 3 - disturbance: 10 freq, per ogni SEA
    dist_freqs = [0.5, 1, 2, 5, 10, 20, 50, 88, 150, 300]
    for n, p in seas.items():
        for f in dist_freqs:
            cost = 1.0 / max(f, 1.0)
            jobs_sweep.append((("test3", n, p.__dict__, float(f)), cost))

    # Test 4 - bias: 4 omega_j, per ogni SEA
    omega_j_vals = [1.0, 2.0, 5.0, 10.0]
    for n, p in seas.items():
        for om in omega_j_vals:
            cost = 1.0  # tempo fisso 1 s
            jobs_sweep.append((("test4", n, p.__dict__, float(om)), cost))

    print(f"  totale job sweep: {len(jobs_sweep)}")
    jobs_sweep.sort(key=lambda jc: -jc[1])  # longest-first

    t0 = time.time()
    bode_records: List[Dict] = []
    dist_records: List[Dict] = []
    bias_records: List[Dict] = []
    with ProcessPoolExecutor(max_workers=MAX_WORKERS) as ex:
        futures = [ex.submit(dispatch_sweep_job, args) for args, _ in jobs_sweep]
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

    # Salva dati test 2-3-4
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

    # Plot
    for n, p in seas.items():
        recs = [r for r in bode_records if r["joint"] == n]
        plot_test2(recs, p, PLOT_DIR / f"02_bode_{n.split('_')[-1].lower()}.png")
        recs = [r for r in dist_records if r["joint"] == n]
        plot_test3(recs, p, PLOT_DIR / f"03_disturbance_{n.split('_')[-1].lower()}.png")
        recs = [r for r in bias_records if r["joint"] == n]
        plot_test4(recs, p, PLOT_DIR / f"04_bias_{n.split('_')[-1].lower()}.png")

    # Statistiche pass/fail Test 4
    print("\n  Test 4 residui osservato vs teoria:")
    for n, p in seas.items():
        recs = [r for r in bias_records if r["joint"] == n]
        res = np.array([r["residuo_Nm"] for r in recs])
        print(f"    {n:10s}  RMS residuo = {np.sqrt(np.mean(res * res)):.4f} Nm, "
              f"max = {np.max(np.abs(res)):.4f} Nm")

    # ---------- Test 5 ----------
    print("\n[Test 5] Broadband noise  (seriale)")
    t0 = time.time()
    test5_results = {n: test5_noise(p) for n, p in seas.items()}
    print(f"  elapsed = {time.time() - t0:.2f} s")
    for n, m in test5_results.items():
        np.savez(RESULTS_DIR / f"test5_noise_spectrum_{n.split('_')[-1].lower()}.npz",
                 f=np.array(m["f_grid"]),
                 Pxx_spring=np.array(m["Pxx_spring"]),
                 Pxx_ref=np.array(m["Pxx_ref"]))
        plot_test5(m, seas[n], PLOT_DIR / f"05_noise_spectrum_{n.split('_')[-1].lower()}.png")
        print(f"  {n:10s}  picco osservato = {m['f_peak_50_150_Hz']:.2f} Hz "
              f"(teorico {m['f_d_theory_Hz']:.2f} Hz)  "
              f"peaking = {m['peaking_above_flat_10_30_Hz_dB']:.2f} dB")

    summary["test1"] = {n: {k: v for k, v in m.items() if k not in ("t", "tau_spring")}
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
