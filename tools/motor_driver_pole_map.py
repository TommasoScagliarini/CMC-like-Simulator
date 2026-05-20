"""
tools/motor_driver_pole_map.py
==============================
Plot SEA motor-driver closed-loop poles on the complex plane.

Linearized state x = [theta_m, omega_m, xi]^T with theta_j held fixed
(driver-isolation analysis). The plugin PI motor law is

    tau_spring = K * (theta_m - theta_j)
    xi_dot     = tau_ref - tau_spring
    tau_input  = tau_ref + Kp*(tau_ref - tau_spring) + Ki*xi - Kd*omega_m
    omega_m_dot = (tau_input - tau_spring - Bm*omega_m) / Jm

A matrix:
    [[0,            1,             0     ],
     [-(1+Kp)K/Jm, -(Kd+Bm)/Jm,   Ki/Jm  ],
     [-K,           0,             0     ]]

The script also plots the PD-only sub-system (Ki=0) for comparison and
overlays the cascade outer/velocity loop bandwidths as vertical reference
lines on the real axis.
"""

from __future__ import annotations

import os
import numpy as np
import matplotlib.pyplot as plt

# Current plugin parameters (model AB06_SEASEA_stiff321_500_pi.osim)
SEAS = {
    "knee":  dict(Jm=0.01, Bm=0.10, K=321.0, Kp=18.0,  Kd=11.0, Ki=190.0, F_opt=100.0),
    "ankle": dict(Jm=0.01, Bm=0.10, K=500.0, Kp=11.3,  Kd=11.0, Ki=123.0, F_opt=250.0),
}

# Current cascade outer loop bandwidths (config.py)
CASCADE = {
    "knee":  dict(Kp_outer=18.85,  Kp_v=29.2,    Ki_v=1377.0),
    "ankle": dict(Kp_outer=47.125, Kp_v=2.8275,  Ki_v=213.0),
}

COLORS  = {"knee": "#1f77b4", "ankle": "#d62728"}
OUTPUT_DIR = "plot/05_18_2026_motor_driver_poles"


def build_A_pi(p):
    return np.array([
        [0.0,                            1.0,                       0.0],
        [-(1.0 + p["Kp"]) * p["K"] / p["Jm"],
         -(p["Kd"] + p["Bm"]) / p["Jm"],
         p["Ki"] / p["Jm"]],
        [-p["K"],                        0.0,                       0.0],
    ])


def build_A_pd(p):
    return np.array([
        [0.0,                                1.0],
        [-(1.0 + p["Kp"]) * p["K"] / p["Jm"], -(p["Kd"] + p["Bm"]) / p["Jm"]],
    ])


def annotate_pole(ax, ev, color, dx=15.0, dy=20.0):
    if abs(ev.imag) < 1e-6:
        tau_ms = -1000.0 / ev.real if ev.real < 0 else None
        if tau_ms is not None:
            ax.annotate(
                f" τ={tau_ms:.0f} ms\n ({-ev.real:.1f} rad/s)",
                xy=(ev.real, ev.imag), xytext=(ev.real + dx, ev.imag + dy),
                fontsize=8, color=color,
            )
    elif ev.imag > 0:
        fn = abs(ev) / (2.0 * np.pi)
        zeta = -ev.real / abs(ev)
        ax.annotate(
            f" f={fn:.1f} Hz\n |s|={abs(ev):.0f} rad/s\n ζ={zeta:.2f}",
            xy=(ev.real, ev.imag), xytext=(ev.real + dx, ev.imag + dy),
            fontsize=8, color=color,
        )


def add_iso_lines(ax, real_min, real_max, imag_min, imag_max):
    ax.axhline(0, color='gray', lw=0.5)
    ax.axvline(0, color='gray', lw=0.5)

    r_max = max(abs(real_min), abs(real_max), abs(imag_min), abs(imag_max))
    for zeta in [0.3, 0.5, 0.707, 1.0]:
        theta = np.pi - np.arccos(zeta)
        r = np.linspace(0, r_max * 1.4, 50)
        ax.plot(r * np.cos(theta), r * np.sin(theta), '--',
                color='lightgray', lw=0.7, zorder=0)
        ax.plot(r * np.cos(theta), -r * np.sin(theta), '--',
                color='lightgray', lw=0.7, zorder=0)
        ax.text(0.85 * r_max * np.cos(theta), 0.85 * r_max * np.sin(theta),
                f"ζ={zeta}", color='gray', fontsize=7,
                ha='center', va='center')

    theta_circ = np.linspace(np.pi / 2, 3 * np.pi / 2, 200)
    for omega_n in [10, 30, 100, 300, 1000]:
        ax.plot(omega_n * np.cos(theta_circ), omega_n * np.sin(theta_circ), ':',
                color='lightgray', lw=0.7, zorder=0)
        if -omega_n > real_min and -omega_n < real_max:
            ax.text(-omega_n, -8, f"{omega_n}", color='gray',
                    fontsize=7, ha='center', va='top')


def add_cascade_marks(ax, real_min, imag_max):
    for sea_name, c in CASCADE.items():
        color = COLORS[sea_name]
        omega_pos = c["Kp_outer"]
        omega_v_zero = c["Ki_v"] / c["Kp_v"]
        for omega, label in [(omega_pos, "cascade pos"),
                             (omega_v_zero, "cascade vel-zero")]:
            ax.axvline(-omega, color=color, lw=0.8, alpha=0.35, ls='-.')
            ax.text(-omega, imag_max * 0.95, f" {sea_name}\n {label}\n {omega:.1f}",
                    color=color, fontsize=7, alpha=0.7, ha='left', va='top')


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    fig, (ax_full, ax_zoom) = plt.subplots(1, 2, figsize=(15, 7))

    for sea_name, params in SEAS.items():
        A_pi = build_A_pi(params)
        ev_pi = np.linalg.eigvals(A_pi)
        color = COLORS[sea_name]

        for ax in (ax_full, ax_zoom):
            ax.scatter(ev_pi.real, ev_pi.imag, s=180, marker='o',
                       facecolors='none', edgecolors=color, linewidth=2.0,
                       label=f"{sea_name} PI (current)")

        # Annotate only on full view to keep zoom uncluttered
        for ev in ev_pi:
            annotate_pole(ax_full, ev, color)

        # Plugin PI zero in (tau_ref - tau_spring) → tau_input path
        zero = -params["Ki"] / params["Kp"]
        for ax in (ax_full, ax_zoom):
            ax.scatter([zero], [0], s=120, marker='s',
                       facecolors='none', edgecolors=color, linewidth=1.3,
                       alpha=0.7,
                       label=f"{sea_name} PI zero (-Ki/Kp = {zero:.1f})")

    # Full view
    ax_full.set_xlim(-1200, 200)
    ax_full.set_ylim(-1000, 1000)
    add_iso_lines(ax_full, -1200, 200, -1000, 1000)
    add_cascade_marks(ax_full, -1200, 1000)
    ax_full.set_xlabel("Re(s) [rad/s]")
    ax_full.set_ylabel("Im(s) [rad/s]")
    ax_full.set_title("Motor driver poles — full view")
    ax_full.legend(loc='upper left', fontsize=8)
    ax_full.grid(True, alpha=0.3)

    # Zoom near origin to see slow real pole + zero
    ax_zoom.set_xlim(-40, 5)
    ax_zoom.set_ylim(-20, 20)
    add_iso_lines(ax_zoom, -40, 5, -20, 20)
    add_cascade_marks(ax_zoom, -40, 20)
    ax_zoom.set_xlabel("Re(s) [rad/s]")
    ax_zoom.set_ylabel("Im(s) [rad/s]")
    ax_zoom.set_title("Zoom near origin (slow pole + PI zero)")
    ax_zoom.legend(loc='upper left', fontsize=8)
    ax_zoom.grid(True, alpha=0.3)

    fig.suptitle(
        "SEA motor driver closed-loop poles\n"
        "Plant: rotor (Jm, Bm) + spring (K) + plugin PI (Kp, Kd, Ki); θ_j held fixed",
        fontsize=12,
    )
    plt.tight_layout()

    out = os.path.join(OUTPUT_DIR, "motor_driver_poles.png")
    plt.savefig(out, dpi=150)
    print(f"Saved: {out}")

    # Numeric summary
    print("\n=== Numeric pole values ===")
    for sea_name, params in SEAS.items():
        A_pi = build_A_pi(params)
        ev_pi = np.linalg.eigvals(A_pi)
        Ki_max = (params["Kd"] + params["Bm"]) * (1.0 + params["Kp"]) / params["Jm"]
        print(f"\n--- {sea_name} ---")
        print(f"  PI poles:")
        for ev in ev_pi:
            if abs(ev.imag) > 1e-6:
                print(f"    {ev:+.3f}   |s|={abs(ev):.2f} rad/s "
                      f"({abs(ev)/(2*np.pi):.2f} Hz)  "
                      f"ζ={-ev.real/abs(ev):+.3f}")
            else:
                print(f"    {ev:+.3f}                       (real pole, "
                      f"τ={-1000/ev.real:.1f} ms)")
        print(f"  PI zero at s = -Ki/Kp = {-params['Ki']/params['Kp']:+.2f} rad/s "
              f"({params['Ki']/params['Kp']/(2*np.pi):.2f} Hz)")
        print(f"  Routh Ki_max = (Kd+Bm)(1+Kp)/Jm = {Ki_max:.1f}, "
              f"Ki/Ki_max = {params['Ki']/Ki_max:.4f}")


if __name__ == "__main__":
    main()
