"""
tools/motor_driver_pole_locus.py
================================
Plot the root-locus of the SEA motor driver closed-loop poles as ω_n varies,
with ζ=0.7 and p=0.2·ω_n held fixed.

For each ω_n the gains are computed from the design formulas

    Ki = Jm·p·ω_n²/Ks
    Kp = Jm·(ω_n² + 2ζω_n·p)/Ks - 1
    Kd = Jm·(p + 2ζω_n) - Bm

and the closed-loop poles are eigenvalues of

    A = [[0,              1,             0    ],
         [-(1+Kp)·Ks/Jm, -(Kd+Bm)/Jm,   Ki/Jm],
         [-Ks,            0,             0    ]]

The plot marks the current operating point (×) and the proposed design (★).
"""

from __future__ import annotations

import os
import numpy as np
import matplotlib.pyplot as plt

SEAS = {
    "knee":  dict(Jm=0.01, Bm=0.10, K=321.0, F_opt=100.0,
                  Kp_curr=18.0, Kd_curr=11.0, Ki_curr=190.0,
                  omega_n_target=360.0),
    "ankle": dict(Jm=0.01, Bm=0.10, K=500.0, F_opt=250.0,
                  Kp_curr=11.3, Kd_curr=11.0, Ki_curr=123.0,
                  omega_n_target=280.0),
}

COLORS = {"knee": "#1f77b4", "ankle": "#d62728"}
OUTPUT_DIR = "plot/05_18_2026_motor_driver_locus"

ZETA = 0.7
P_OVER_OMEGA = 0.2


def gains_from_design(omega_n, p_real, zeta, Jm, Bm, Ks):
    Ki = Jm * p_real * omega_n**2 / Ks
    Kp = Jm * (omega_n**2 + 2.0 * zeta * omega_n * p_real) / Ks - 1.0
    Kd = Jm * (p_real + 2.0 * zeta * omega_n) - Bm
    return Kp, Kd, Ki


def build_A(Jm, Bm, Ks, Kp, Kd, Ki):
    return np.array([
        [0.0,                       1.0,             0.0],
        [-(1.0 + Kp) * Ks / Jm,    -(Kd + Bm) / Jm, Ki / Jm],
        [-Ks,                       0.0,             0.0],
    ])


def eig_split(evs):
    real_evs = [e for e in evs if abs(e.imag) < 1e-6]
    complex_evs = [e for e in evs if abs(e.imag) > 1e-6]
    upper = next((e for e in complex_evs if e.imag > 0), None)
    return (real_evs[0].real if real_evs else np.nan,
            (upper.real, upper.imag) if upper else (np.nan, np.nan))


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    fig, (ax_full, ax_zoom) = plt.subplots(1, 2, figsize=(15, 7))

    omega_n_range = np.linspace(200.0, 900.0, 250)

    for sea_name, props in SEAS.items():
        color = COLORS[sea_name]
        Jm, Bm, Ks = props["Jm"], props["Bm"], props["K"]
        omega_mech = np.sqrt(Ks / Jm)

        # Design locus
        complex_re, complex_im, real_p, pi_zero = [], [], [], []
        for omega_n in omega_n_range:
            p_real = P_OVER_OMEGA * omega_n
            Kp, Kd, Ki = gains_from_design(omega_n, p_real, ZETA, Jm, Bm, Ks)
            evs = np.linalg.eigvals(build_A(Jm, Bm, Ks, Kp, Kd, Ki))
            rp, (cre, cim) = eig_split(evs)
            real_p.append(rp)
            complex_re.append(cre)
            complex_im.append(cim)
            pi_zero.append(-Ki / Kp if Kp > 0 else np.nan)

        complex_re = np.array(complex_re)
        complex_im = np.array(complex_im)
        real_p = np.array(real_p)
        pi_zero = np.array(pi_zero)

        for ax in (ax_full, ax_zoom):
            ax.plot(complex_re, complex_im, '-', color=color, lw=1.6, alpha=0.7,
                    label=f"{sea_name} complex pair (ζ=0.7 line)")
            ax.plot(complex_re, -complex_im, '-', color=color, lw=1.6, alpha=0.7)
            ax.plot(real_p, np.zeros_like(real_p), '--', color=color,
                    lw=1.4, alpha=0.7,
                    label=f"{sea_name} real pole locus (-p = -0.2·ω_n)")
            ax.plot(pi_zero, np.zeros_like(pi_zero), ':', color=color,
                    lw=1.0, alpha=0.45,
                    label=f"{sea_name} PI zero locus (-Ki/Kp)")

        # Current operating point (×)
        Kp_c, Kd_c, Ki_c = props["Kp_curr"], props["Kd_curr"], props["Ki_curr"]
        ev_curr = np.linalg.eigvals(build_A(Jm, Bm, Ks, Kp_c, Kd_c, Ki_c))
        for ax in (ax_full, ax_zoom):
            ax.scatter(ev_curr.real, ev_curr.imag, marker='x', s=200,
                       c=color, linewidth=2.5, zorder=5,
                       label=f"{sea_name} CURRENT (ω_n≈774)")

        # Proposed operating point (★)
        wn_t = props["omega_n_target"]
        p_t = P_OVER_OMEGA * wn_t
        Kp_t, Kd_t, Ki_t = gains_from_design(wn_t, p_t, ZETA, Jm, Bm, Ks)
        ev_target = np.linalg.eigvals(build_A(Jm, Bm, Ks, Kp_t, Kd_t, Ki_t))
        for ax in (ax_full, ax_zoom):
            ax.scatter(ev_target.real, ev_target.imag, marker='*', s=380,
                       c=color, edgecolors='black', linewidth=1.0, zorder=6,
                       label=f"{sea_name} PROPOSED (ω_n={wn_t:.0f})")

        # Proposed PI zero
        zero_t = -Ki_t / Kp_t if Kp_t > 0 else np.nan
        for ax in (ax_full, ax_zoom):
            ax.scatter([zero_t], [0], marker='s', s=180,
                       facecolors='none', edgecolors=color, linewidth=2.0,
                       zorder=6)

        # Mechanical resonance vertical mark on real axis
        for ax in (ax_full, ax_zoom):
            ax.axvline(-omega_mech, color=color, lw=0.6, alpha=0.25, ls='-.')
            ax.text(-omega_mech, 0.95 * (ax.get_ylim()[1] if ax.get_ylim()[1] != 0 else 100),
                    f" ω_mech\n {omega_mech:.0f}",
                    color=color, fontsize=7, alpha=0.6, ha='left', va='top')

    # Reference lines on both
    for ax in (ax_full, ax_zoom):
        ax.axhline(0, color='gray', lw=0.5)
        ax.axvline(0, color='gray', lw=0.5)

    # Iso-zeta=0.7 line drawn lightly (matches the complex pair locus, helpful as ref)
    for ax, r_max in [(ax_full, 1100), (ax_zoom, 400)]:
        theta = np.pi - np.arccos(ZETA)
        r = np.linspace(0, r_max, 50)
        ax.plot(r * np.cos(theta), r * np.sin(theta), ':',
                color='lightgray', lw=1.0, alpha=0.8, zorder=0)
        ax.plot(r * np.cos(theta), -r * np.sin(theta), ':',
                color='lightgray', lw=1.0, alpha=0.8, zorder=0)

    # ω_n iso-circles
    theta_circ = np.linspace(np.pi / 2, 3 * np.pi / 2, 200)
    for ax in (ax_full, ax_zoom):
        for omega_n in [280, 360, 500, 774]:
            ax.plot(omega_n * np.cos(theta_circ), omega_n * np.sin(theta_circ), ':',
                    color='lightgray', lw=0.6, alpha=0.5, zorder=0)
            if -omega_n > ax.get_xlim()[0] and -omega_n < ax.get_xlim()[1]:
                ax.text(-omega_n, -6, f"ω_n={omega_n}",
                        color='gray', fontsize=6, ha='center', va='top', alpha=0.7)

    ax_full.set_xlim(-900, 100)
    ax_full.set_ylim(-700, 700)
    ax_full.set_xlabel("Re(s) [rad/s]")
    ax_full.set_ylabel("Im(s) [rad/s]")
    ax_full.set_title("Motor driver pole locus — full view (ω_n: 200 → 900)")
    ax_full.legend(loc='upper left', fontsize=7)
    ax_full.grid(True, alpha=0.3)

    ax_zoom.set_xlim(-400, 30)
    ax_zoom.set_ylim(-320, 320)
    ax_zoom.set_xlabel("Re(s) [rad/s]")
    ax_zoom.set_ylabel("Im(s) [rad/s]")
    ax_zoom.set_title("Zoom near proposed design points")
    ax_zoom.legend(loc='upper left', fontsize=7)
    ax_zoom.grid(True, alpha=0.3)

    fig.suptitle(
        "SEA motor-driver root locus as ω_n varies (ζ=0.7, p=0.2·ω_n)\n"
        "× = current design ; ★ = proposed (knee ω_n=360, ankle ω_n=280) ; □ = PI zero",
        fontsize=12,
    )
    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, "motor_driver_pole_locus.png")
    plt.savefig(out, dpi=150)
    print(f"Saved: {out}")

    # Numeric summary
    print("\n=== Proposed operating points ===")
    for sea_name, props in SEAS.items():
        wn = props["omega_n_target"]
        pp = P_OVER_OMEGA * wn
        Kp, Kd, Ki = gains_from_design(wn, pp, ZETA,
                                       props["Jm"], props["Bm"], props["K"])
        evs = np.linalg.eigvals(build_A(props["Jm"], props["Bm"], props["K"],
                                         Kp, Kd, Ki))
        print(f"\n--- {sea_name} (Ks={props['K']:.0f}, F_opt={props['F_opt']:.0f}) ---")
        print(f"  ω_n={wn:.0f} rad/s ({wn/(2*np.pi):.1f} Hz),  p={pp:.0f} rad/s,  ζ={ZETA}")
        print(f"  Gains:  Kp={Kp:.3f}, Kd={Kd:.3f}, Ki={Ki:.3f}")
        print(f"  PI zero: s = {-Ki/Kp:+.2f} rad/s")
        for e in evs:
            if abs(e.imag) > 1e-6:
                print(f"  pole {e:+.2f}   |s|={abs(e):.1f} rad/s "
                      f"({abs(e)/(2*np.pi):.1f} Hz),  ζ={-e.real/abs(e):.3f}")
            else:
                print(f"  pole {e:+.2f}   (real, τ={-1000/e.real:.1f} ms, "
                      f"{-e.real/(2*np.pi):.1f} Hz)")


if __name__ == "__main__":
    main()
