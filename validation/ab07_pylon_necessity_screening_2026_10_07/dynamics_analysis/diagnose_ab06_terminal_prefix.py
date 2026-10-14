"""Read-only localization of the failed exact AB06 prefix comparison.

No FK, integration, tolerance relaxation, metric overwrite or episode change.
Only saved arrays, callback payloads and the captured reference-filter step are
used; the original all-row exact comparison remains false.
"""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
NEW = HERE.parent / "dynamics/AB06_control/run"
OLD = REPO / "results/AB06_paired_prescribed_HS_9s_2026_10_07"
TRAP = np.trapezoid if hasattr(np, "trapezoid") else np.trapz


def read_json(p):
    return json.loads(p.read_text(encoding="utf-8"))


def fp(p):
    return {"path": p.relative_to(REPO).as_posix(), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}


def table(p):
    lines = p.read_text().splitlines()
    i = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    names = lines[i+1].split()
    data = np.loadtxt(lines[i+2:], ndmin=2)
    if len(names) != data.shape[1] or len(set(names)) != len(names) or not np.all(np.isfinite(data)) or np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError(f"Invalid saved native table {p}")
    return data[:, 0], names[1:], data[:, 1:]


def main():
    output = HERE / "AB06_terminal_prefix_supplement.json"
    if output.exists():
        raise FileExistsError(output)
    metric_path = HERE / "AB06_control/metrics.json"
    metric = read_json(metric_path)
    if metric["prefix_comparison"]["all_available_tables_exact_numeric_prefix"] is not False:
        raise ValueError("Supplement is specific to the retained exact-prefix failure.")
    nr, oldr = read_json(NEW/"receipt.json"), read_json(OLD/"receipt.json")
    paths = {metric_path, NEW/"receipt.json", OLD/"receipt.json", NEW/"trace.jsonl", OLD/"trace.jsonl", Path(__file__), REPO/"Trajectory Generator/osim_trj_cmc_like.py"}
    new_trace = [json.loads(line) for line in (NEW/"trace.jsonl").read_text().splitlines()]
    old_trace = [json.loads(line) for line in (OLD/"trace.jsonl").read_text().splitlines()]
    keys = ("time", "target_time", "policy_segment_times", "policy_segment_values", "policy_segment_derivatives", "reference_governor_diagnostics", "sea_segment_diagnostics", "so_diagnostics", "online_grf")
    diffs = [i for i, row in enumerate(new_trace) if any(row.get(k) != old_trace[i].get(k) for k in keys)]
    first = diffs[0]
    if first != len(new_trace)-1:
        raise AssertionError("Differences are not confined to the final callback.")
    dt = float(nr["effective_simulator_config"]["dt"])
    segments = {}
    for label, row in (("new", new_trace[first]), ("old", old_trace[first])):
        t0, t1 = row["policy_segment_times"]
        segments[label] = {"t0_s": t0, "t1_s": t1, "duration_s": t1-t0,
                           "duration_over_captured_filter_dt": (t1-t0)/dt,
                           "third_order_filter_substeps_from_source_ceil": max(2, math.ceil((t1-t0)/dt)),
                           "policy_values": row["policy_segment_values"], "policy_derivatives": row["policy_segment_derivatives"]}
    stop, results, loaded = segments["new"]["t0_s"], {}, {}
    for p in sorted((NEW/"sim_outputs").glob("*.sto")):
        op = OLD/"sim_outputs"/p.name
        paths |= {p, op}
        t, columns, values = table(p)
        ot, oc, ov = table(op)
        if columns != oc or not np.array_equal(t, ot[:len(t)]):
            raise ValueError("Columns/time identity required for localized comparison.")
        ov = ov[:len(t)]
        error = values-ov
        changed = np.flatnonzero(np.any(error != 0, axis=1))
        before = t <= stop
        fields = np.flatnonzero(np.any(error != 0, axis=0))
        name = p.stem.removeprefix(nr["effective_simulator_config"]["output_prefix"]+"_")
        results[name] = {"rows": len(t), "identical_rows_before_or_at_last_segment_start": bool(np.array_equal(values[before], ov[before])),
                         "exact_initial_prefix_rows": int(changed[0]) if len(changed) else len(t),
                         "changed_rows": len(changed), "first_changed_time_s": float(t[changed[0]]) if len(changed) else None,
                         "last_changed_time_s": float(t[changed[-1]]) if len(changed) else None,
                         "changed_fields_max_abs_difference_in_column_unit": {columns[i]: float(np.max(np.abs(error[:, i]))) for i in fields}}
        loaded[name] = t, columns, values, ov
    primary = metric["windows"]["primary_all"]["requested_range_s"]
    knots = metric["actual_COM"]["requested_exact_knots_s"]
    times = loaded["kinematics"][0]
    grid = np.unique(np.r_[primary[0], times[(times > primary[0]) & (times < primary[1])], knots, primary[1]])
    def values(name, column, which):
        t, columns, nv, ov = loaded[name]
        return np.interp(grid, t, (nv if which == "new" else ov)[:, columns.index(column)])
    primary_results = {}
    for which in ("old", "new"):
        force = values("reserve_torques", "pelvis_ty_reserve_torque", which)
        speed = values("states", "pelvis_ty_qdot", which)
        primary_results[which] = {"left_online_impulse_Ns": float(TRAP(values("online_grf", "left_force_y", which), grid)),
                                  "pelvis_ty_signed_impulse_Ns": float(TRAP(force, grid)),
                                  "pelvis_ty_signed_work_J": float(TRAP(force*speed, grid))}
    delta = {k: primary_results["new"][k]-primary_results["old"][k] for k in primary_results["new"]}
    before = {p: fp(p) for p in paths}
    result = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "localized_terminal_reference_discretization_difference",
              "exact_all_rows_prefix_result_preserved": False, "original_metrics": fp(metric_path),
              "native_run_changes": False, "new_FK_or_integration": False, "tolerance_relaxation": False,
              "first_different_callback_one_based": first+1, "callbacks_before_identical_for_declared_payloads": first,
              "declared_payloads": keys, "last_segments": segments,
              "last_target_time_new_minus_old_s": segments["new"]["t1_s"]-segments["old"]["t1_s"],
              "reference_filter_dt_s_from_effective_config": dt,
              "source_evidence": {"path": "Trajectory Generator/osim_trj_cmc_like.py", "segment_end_min_episode_end_line": 1828,
                                  "filter_n_ceil_line": 736, "control_dt_binding_line": 2133,
                                  "equation": "n=max(2, ceil((t1-t0)/cfg.dt)); dt_filter=(t1-t0)/n"},
              "interpretation": "Last new target is min(t+0.01, episode_end) and is shorter by picoseconds; this crosses ceil(duration/captured_filter_dt) from11 to10. This source-grounded discretization mechanism explains why identical preceding callbacks acquire a small served-reference difference in the last segment; it is not model drift or an exact prefix PASS.",
              "tables": results, "all_tables_identical_through_last_segment_start": all(x["identical_rows_before_or_at_last_segment_start"] for x in results.values()),
              "primary_grid": "Same native timestamps + exact7HS/TOknots as V2 metrics, no shifted cycles", "primary_range_s": primary,
              "primary_metrics": primary_results, "primary_new_minus_old": delta,
              "limitations": "Primary last HS lies inside final segment; tiny differences are retained, not asserted zero or promoted to a tolerance PASS. No counterfactual runtime/solver replay was performed.",
              "sources_before": {x["path"]: x["sha256"] for x in before.values()}}
    if any(fp(p) != old for p, old in before.items()):
        raise AssertionError("Protected source changed during scalar supplement.")
    result["sources_unchanged_after"] = {"count": len(before), "all_match": True}
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    print(json.dumps({"file": fp(output), "primary_new_minus_old": delta, "segments": segments}, indent=2))


if __name__ == "__main__":
    main()
