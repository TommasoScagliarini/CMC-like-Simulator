"""Analyze 1049 completed sweep results: find Pareto-optimal candidates."""
from __future__ import annotations

import csv
import math
from pathlib import Path

CSV_PATH = Path(
    "results/_sea_driver_sweep_20260418_052206/sweep_results.csv"
)


def to_float(x: str) -> float:
    try:
        return float(x)
    except (ValueError, TypeError):
        return math.nan


def to_bool(x: str) -> bool:
    return str(x).strip().lower() == "true"


def main() -> None:
    rows: list[dict] = []
    with CSV_PATH.open("r", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for r in reader:
            rows.append(r)

    n = len(rows)
    print(f"Total rows: {n}")

    # Filter: need complete + finite + not insane
    viable = []
    for r in rows:
        if not to_bool(r["complete"]):
            continue
        if not to_bool(r["finite_outputs"]):
            continue
        k_rms = to_float(r["knee_tracking_rms_deg"])
        a_rms = to_float(r["ankle_tracking_rms_deg"])
        k_nf = to_float(r["knee_noise_frac"])
        a_nf = to_float(r["ankle_noise_frac"])
        max_tau = to_float(r["max_tau_input_raw_abs"])
        max_u = to_float(r["max_u"])
        if not all(math.isfinite(v) for v in [k_rms, a_rms, k_nf, a_nf, max_tau, max_u]):
            continue
        viable.append(r)

    print(f"Viable (complete+finite+finite metrics): {len(viable)}")

    # Hard thresholds from SWEEP_INSTRUCTIONS.md
    def passes_all(r: dict) -> tuple[bool, list[str]]:
        reasons = []
        k_rms = to_float(r["knee_tracking_rms_deg"])
        a_rms = to_float(r["ankle_tracking_rms_deg"])
        k_nf = to_float(r["knee_noise_frac"])
        a_nf = to_float(r["ankle_noise_frac"])
        max_tau = to_float(r["max_tau_input_raw_abs"])
        max_u = to_float(r["max_u"])
        if max(k_rms, a_rms) >= 5.0:
            reasons.append(f"track={max(k_rms,a_rms):.2f}")
        if max(k_nf, a_nf) >= 0.20:
            reasons.append(f"noise={max(k_nf,a_nf):.3f}")
        if max_tau >= 4500:
            reasons.append(f"tau={max_tau:.0f}")
        if max_u >= 0.95:
            reasons.append(f"u={max_u:.3f}")
        return (len(reasons) == 0, reasons)

    all_ok = [r for r in viable if passes_all(r)[0]]
    print(f"Pass ALL thresholds: {len(all_ok)}")

    # Pareto analysis: minimize (worst_tracking, worst_noise)
    def worst_track(r):
        return max(to_float(r["knee_tracking_rms_deg"]),
                   to_float(r["ankle_tracking_rms_deg"]))

    def worst_noise(r):
        return max(to_float(r["knee_noise_frac"]),
                   to_float(r["ankle_noise_frac"]))

    # Only keep "safe" candidates: tau and u within bounds
    safe = []
    for r in viable:
        if to_float(r["max_tau_input_raw_abs"]) >= 4500:
            continue
        if to_float(r["max_u"]) >= 0.95:
            continue
        safe.append(r)
    print(f"Safe (tau<4500 AND |u|<0.95): {len(safe)}")

    # Find Pareto front on (worst_track, worst_noise)
    pareto = []
    for r in safe:
        t1, n1 = worst_track(r), worst_noise(r)
        dominated = False
        for s in safe:
            if s is r:
                continue
            t2, n2 = worst_track(s), worst_noise(s)
            if t2 <= t1 and n2 <= n1 and (t2 < t1 or n2 < n1):
                dominated = True
                break
        if not dominated:
            pareto.append(r)
    pareto.sort(key=worst_track)
    print(f"Pareto-optimal front: {len(pareto)} candidates")
    print()

    # Print Pareto front
    header = ("run_id", "Kknee", "Kpk", "Kdk", "Kankle",
              "Kpa", "Kda", "wn", "zeta", "tau_d",
              "trk_k", "trk_a", "nf_k", "nf_a",
              "maxTau", "maxU")
    print(("{:<38} {:>5} {:>5} {:>5} {:>5} {:>5} {:>5} "
           "{:>5} {:>5} {:>6} {:>6} {:>6} {:>6} {:>6} "
           "{:>7} {:>5}").format(*header))
    print("-" * 160)
    for r in pareto:
        vals = (
            r["run_id"][:38],
            f"{to_float(r['knee_K']):.0f}",
            f"{to_float(r['knee_Kp']):.2f}",
            f"{to_float(r['knee_Kd']):.2f}",
            f"{to_float(r['ankle_K']):.0f}",
            f"{to_float(r['ankle_Kp']):.2f}",
            f"{to_float(r['ankle_Kd']):.2f}",
            f"{to_float(r['omega_n']):.0f}",
            f"{to_float(r['zeta']):.2f}",
            f"{to_float(r['derivative_filter_tau']):.4f}",
            f"{to_float(r['knee_tracking_rms_deg']):.2f}",
            f"{to_float(r['ankle_tracking_rms_deg']):.2f}",
            f"{to_float(r['knee_noise_frac']):.3f}",
            f"{to_float(r['ankle_noise_frac']):.3f}",
            f"{to_float(r['max_tau_input_raw_abs']):.0f}",
            f"{to_float(r['max_u']):.2f}",
        )
        print(("{:<38} {:>5} {:>5} {:>5} {:>5} {:>5} {:>5} "
               "{:>5} {:>5} {:>6} {:>6} {:>6} {:>6} {:>6} "
               "{:>7} {:>5}").format(*vals))
    print()

    # Best candidates by weighted composite score (on the safe set)
    # scale: tracking in deg, noise dimensionless — scale noise by 10
    def composite(r):
        wt = max(worst_track(r), 0.0)
        wn = max(worst_noise(r), 0.0)
        return wt + 10.0 * wn

    safe_ranked = sorted(safe, key=composite)
    print("Top 15 by composite = worst_track + 10*worst_noise:")
    print(("{:<38} {:>5} {:>5} {:>5} {:>5} {:>5} {:>5} "
           "{:>5} {:>5} {:>6} {:>6} {:>6} {:>6} {:>6} {:>7}").format(
            "run_id", "Kknee", "Kpk", "Kdk", "Kankle",
            "Kpa", "Kda", "wn", "zeta", "tau_d",
            "trk_k", "trk_a", "nf_k", "nf_a", "maxTau"))
    print("-" * 150)
    for r in safe_ranked[:15]:
        vals = (
            r["run_id"][:38],
            f"{to_float(r['knee_K']):.0f}",
            f"{to_float(r['knee_Kp']):.2f}",
            f"{to_float(r['knee_Kd']):.2f}",
            f"{to_float(r['ankle_K']):.0f}",
            f"{to_float(r['ankle_Kp']):.2f}",
            f"{to_float(r['ankle_Kd']):.2f}",
            f"{to_float(r['omega_n']):.0f}",
            f"{to_float(r['zeta']):.2f}",
            f"{to_float(r['derivative_filter_tau']):.4f}",
            f"{to_float(r['knee_tracking_rms_deg']):.2f}",
            f"{to_float(r['ankle_tracking_rms_deg']):.2f}",
            f"{to_float(r['knee_noise_frac']):.3f}",
            f"{to_float(r['ankle_noise_frac']):.3f}",
            f"{to_float(r['max_tau_input_raw_abs']):.0f}",
        )
        print(("{:<38} {:>5} {:>5} {:>5} {:>5} {:>5} {:>5} "
               "{:>5} {:>5} {:>6} {:>6} {:>6} {:>6} {:>6} {:>7}").format(*vals))
    print()

    # Slices by tau_d: show best per filter value
    print("Best candidate per derivative_filter_tau (by composite):")
    by_taud: dict[float, dict] = {}
    for r in safe_ranked:
        td = to_float(r["derivative_filter_tau"])
        if td not in by_taud:
            by_taud[td] = r
    for td in sorted(by_taud.keys()):
        r = by_taud[td]
        print(f"  tau_d={td:.4f}  composite={composite(r):.3f}  "
              f"trk={worst_track(r):.2f}deg  nf={worst_noise(r):.3f}  "
              f"wn={to_float(r['omega_n']):.0f} z={to_float(r['zeta']):.2f} "
              f"Kk={to_float(r['knee_K']):.0f} Ka={to_float(r['ankle_K']):.0f}")
    print()

    # Stats on the relaxation landscape
    print("Relaxed threshold counts (on safe set):")
    for tk in [5.0, 6.0, 7.0, 8.0, 10.0]:
        for nf in [0.20, 0.30, 0.40, 0.50]:
            c = sum(1 for r in safe
                    if worst_track(r) < tk and worst_noise(r) < nf)
            print(f"  tracking<{tk:.1f} AND noise<{nf:.2f}: {c}")


if __name__ == "__main__":
    main()
