"""Pair up candidates by (K_knee,K_ankle,wn,zeta) and compare metrics across tau_d values."""
from __future__ import annotations

import csv
import math
from collections import defaultdict
from pathlib import Path

CSV_PATH = Path(
    "results/_sea_driver_sweep_20260418_052206/sweep_results.csv"
)


def to_float(x):
    try: return float(x)
    except Exception: return math.nan


def to_bool(x): return str(x).strip().lower() == "true"


def main():
    rows = []
    with CSV_PATH.open("r", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            if not to_bool(r["complete"]) or not to_bool(r["finite_outputs"]):
                continue
            rows.append(r)

    # Group by (Kk, Ka, wn, zeta), collect per-tau_d
    groups = defaultdict(dict)
    for r in rows:
        key = (to_float(r["knee_K"]), to_float(r["ankle_K"]),
               to_float(r["omega_n"]), to_float(r["zeta"]))
        td = to_float(r["derivative_filter_tau"])
        groups[key][td] = r

    # Find groups with all 4 tau_d values
    complete_groups = {k: v for k, v in groups.items() if len(v) == 4}
    print(f"Groups with all 4 tau_d values: {len(complete_groups)}")
    print()

    # For each complete group, show noise_frac (worst) vs tau_d
    print("Filter effect on worst_noise_frac (knee noise only — the hard metric):")
    print(f"{'Kk':>5} {'Ka':>5} {'wn':>5} {'zeta':>5} "
          f"{'td=0':>8} {'td=1e-3':>8} {'td=2e-3':>8} {'td=5e-3':>8} "
          f"{'trk_base':>10}")
    # Sort by the td=0 value
    def kf(kv):
        return to_float(kv[1][0.0]["knee_noise_frac"])
    for key, d in sorted(complete_groups.items(), key=kf):
        Kk, Ka, wn, z = key
        try:
            r0 = d[0.0]; r1 = d[0.001]; r2 = d[0.002]; r5 = d[0.005]
        except KeyError:
            continue
        n0 = to_float(r0["knee_noise_frac"])
        n1 = to_float(r1["knee_noise_frac"])
        n2 = to_float(r2["knee_noise_frac"])
        n5 = to_float(r5["knee_noise_frac"])
        trk = max(to_float(r0["knee_tracking_rms_deg"]),
                  to_float(r0["ankle_tracking_rms_deg"]))
        print(f"{Kk:>5.0f} {Ka:>5.0f} {wn:>5.0f} {z:>5.2f} "
              f"{n0:>8.3f} {n1:>8.3f} {n2:>8.3f} {n5:>8.3f} "
              f"{trk:>10.2f}")


if __name__ == "__main__":
    main()
