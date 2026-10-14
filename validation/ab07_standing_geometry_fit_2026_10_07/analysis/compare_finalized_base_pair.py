"""Saved-data diagnostics of the finalized original standing/reference pair.

No OpenSim/FK/model/environment/IK imports or calls. The partial common prefix
is secondary; it cannot replace the prospectively frozen three primary cycles.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
AUDIT = HERE.parent
ROOT = AUDIT.parents[1]
IDS = ("STANDING_AB07", "PYLON400_REFERENCE")
TRAP = getattr(np, "trapezoid", np.trapz)


def read(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def fp(path):
    p = Path(path).resolve()
    return {"path": p.relative_to(ROOT).as_posix(), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}


def storage(path):
    lines = Path(path).read_text().splitlines()
    boundary = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    remaining = [line for line in lines[boundary + 1:] if line.strip()]
    labels = remaining[0].split()
    values = np.array([[float(v) for v in line.split()] for line in remaining[1:]])
    if len(set(labels)) != len(labels) or labels[0] != "time" or values.ndim != 2 or values.shape[1] != len(labels):
        raise ValueError("Unique full native table schema required")
    if not np.isfinite(values).all() or len(values) < 2 or not np.all(np.diff(values[:, 0]) > 0):
        raise ValueError("Finite, strictly increasing native samples required")
    header = {line.split("=", 1)[0].strip().lower(): line.split("=", 1)[1].strip().lower()
              for line in lines[:boundary] if "=" in line}
    return values[:, 0], dict(zip(labels[1:], values[:, 1:].T)), header


def sample(table, grid, column):
    times, values, _ = table
    if grid[0] < times[0] or grid[-1] > times[-1] or column not in values:
        raise ValueError("No extrapolation or missing recorded channels")
    return np.interp(grid, times, values[column])


def stats(values, time, unit):
    duration = time[-1] - time[0]
    return {"unit": unit, "mean": float(TRAP(values, time) / duration),
            "RMS": float(np.sqrt(TRAP(values * values, time) / duration)),
            "max_abs": float(np.abs(values).max())}


def integrals(values, time):
    return {"signed": float(TRAP(values, time)), "absolute": float(TRAP(np.abs(values), time)),
            "positive": float(TRAP(np.maximum(values, 0), time)),
            "negative": float(TRAP(np.minimum(values, 0), time))}


def identity(value):
    # Keep JSON type distinctions, exact decoded float bits (including -0),
    # all semantic strings and list order. No metadata/path exclusions needed.
    if isinstance(value, dict):
        return tuple((key, identity(item)) for key, item in sorted(value.items()))
    if isinstance(value, list):
        return tuple(identity(item) for item in value)
    if isinstance(value, float):
        return ("float", value.hex())
    return (type(value).__name__, value)


def main():
    output = HERE / "finalized_base_pair_saved_data_diagnostics.json"
    if output.exists():
        raise FileExistsError("Preserve the finalized diagnostic")
    dynamic = read(AUDIT / "dynamic_protocol.json")
    if set(dynamic["cases"]) != set(IDS):
        raise ValueError("Require the frozen original two-case protocol")
    metrics, receipts, folders, sources, tables = {}, {}, {}, {}, {}
    for name in IDS:
        metric_path = HERE / name / "metrics.json"
        folder = AUDIT / "dynamics" / name / "run"
        receipt_path = folder / "receipt.json"
        metric, receipt = read(metric_path), read(receipt_path)
        if receipt["status"] not in {"completed", "interrupted", "failed"} or not receipt.get("finished_utc"):
            raise ValueError("Only finalized saved artifacts may be consumed")
        if metric["receipt"]["sha256"] != fp(receipt_path)["sha256"] or not metric["run_artifacts_unchanged"]["passed"]:
            raise AssertionError("Final metric/runtime binding differs")
        metrics[name], receipts[name], folders[name] = metric, receipt, folder
        sources[metric_path.as_posix()] = fp(metric_path)
        for p in folder.rglob("*"):
            if p.is_file(): sources[p.as_posix()] = fp(p)
        cfg = receipt["effective_simulator_config"]
        prefix = cfg["output_prefix"]
        tables[name] = {key: storage(folder / "sim_outputs" / f"{prefix}_{key}.sto")
                        for key in ("states", "kinematics", "kinematics_reference", "reserve_torques", "online_grf")}
        if tables[name]["kinematics"][2].get("indegrees") != "no" or tables[name]["states"][2].get("indegrees") != "no":
            raise AssertionError("Recorded coordinate units must be radians/meters")
    control = metrics["PYLON400_REFERENCE"]["prefix_comparison"]
    historical = ROOT / receipts["PYLON400_REFERENCE"]["historical_control_receipt"]["path"]
    sources[historical.as_posix()] = fp(historical)
    old_folder = historical.parent
    for p in old_folder.rglob("*"):
        if p.is_file(): sources[p.as_posix()] = fp(p)
    old_trace, new_trace = old_folder / "trace.jsonl", folders["PYLON400_REFERENCE"] / "trace.jsonl"
    old_rows = [read_line for line in old_trace.read_text().splitlines() if line.strip()
                for read_line in [json.loads(line, parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))]]
    new_rows = [json.loads(line, parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))
                for line in new_trace.read_text().splitlines() if line.strip()]
    row_equal = [identity(a) == identity(b) for a, b in zip(old_rows, new_rows)]
    trace = {"historical_rows": len(old_rows), "fresh_rows": len(new_rows),
             "all_rows_exact_decoded_numeric_and_semantic_equal": len(old_rows) == len(new_rows) and all(row_equal),
             "raw_file_bytes_equal": old_trace.read_bytes() == new_trace.read_bytes(),
             "differing_row_indices": [i for i, value in enumerate(row_equal) if not value],
             "fields_excluded_from_comparison": [], "float_comparison": "float.hex preserves decoded float bits and negative zero; JSON field order is ignored, array order and strings retained"}
    native = {name: {key: value[key] for key in ("rows", "same_columns", "times_exactly_equal", "decoded_values_exactly_equal", "max_abs_numeric_difference")}
              | {"raw_file_bytes_equal": (ROOT / value["old_file"]["path"]).read_bytes() == (ROOT / value["new_file"]["path"]).read_bytes()}
              for name, value in control["tables"].items() if value["available"]}
    event_csvs = {}
    for p in sorted(old_folder.glob("*.csv")):
        new = folders["PYLON400_REFERENCE"] / p.name
        event_csvs[p.name] = {"fresh_file_present": new.exists(), "raw_file_bytes_equal": new.exists() and p.read_bytes() == new.read_bytes()}
    support = [t for group in tables.values() for t in group.values()]
    lo, hi = max(t[0][0] for t in support), min(t[0][-1] for t in support)
    if not hi > lo:
        raise ValueError("Positive common observed native support required")
    grid = np.unique(np.concatenate(([lo, hi], *[t[0][(t[0] >= lo) & (t[0] <= hi)] for t in support])))
    cases = {}
    for name in IDS:
        t = tables[name]
        translations = set(receipts[name]["effective_simulator_config"]["translation_coords"])
        reserve = sample(t["reserve_torques"], grid, "pelvis_ty_reserve_torque")
        speed = sample(t["states"], grid, "pelvis_ty_qdot")
        tracking = {}
        for coord in t["kinematics"][1]:
            difference = sample(t["kinematics"], grid, coord) - sample(t["kinematics_reference"], grid, coord + "_q_ref")
            tracking[coord] = stats(difference if coord in translations else difference * 180 / np.pi, grid, "m" if coord in translations else "deg")
        cases[name] = {"runtime_status": receipts[name]["status"], "end_reason": metrics[name]["runtime"]["end_reason"],
                       "pelvis_vertical_reserve_force": stats(reserve, grid, "N"),
                       "pelvis_vertical_reserve_impulse_Ns": integrals(reserve, grid),
                       "pelvis_vertical_reserve_work_J": integrals(reserve * speed, grid),
                       "left_actual_force": stats(sample(t["online_grf"], grid, "left_force_y"), grid, "N"),
                       "left_actual_impulse_Ns": float(TRAP(sample(t["online_grf"], grid, "left_force_y"), grid)),
                       "left_max_penetration_mm": float(sample(t["online_grf"], grid, "left_penetration").max() * 1000),
                       "actual_minus_own_served_position": tracking}
    sources[Path(__file__).as_posix()] = fp(Path(__file__))
    sources[(AUDIT / "dynamic_protocol.json").as_posix()] = fp(AUDIT / "dynamic_protocol.json")
    after = {key: fp(ROOT / item["path"]) for key, item in sources.items()}
    if sources != after:
        raise AssertionError("Saved source changed during diagnostic")
    result = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "completed",
              "fresh400_reproducibility": {"native_tables": native, "available_table_count": len(native),
                                           "all_17_native_files_byte_equal": len(native) == 17 and all(v["raw_file_bytes_equal"] for v in native.values()),
                                           "trace": trace, "event_CSVs": event_csvs,
                                           "captured_contract": "Original analyzer validated exact numerical simulator/environment options and episode bounds except permitted local input/output paths; own model/static/fullIK already byte-identical.",
                                           "inherited_prefix_claim_qualification": "The generic analyzer string mentions differing whole-file hashes; in this fresh400 run all17 native files actually have identical bytes. This separate evidence qualifies that string without rewriting the frozen metrics."},
              "secondary_common_observed_prefix": {"range_s": [float(lo), float(hi)], "duration_s": float(hi - lo), "quadrature_points": len(grid),
                                                   "purpose": "Same recorded physical prefix only; never a replacement for missing three-cycle primary coverage",
                                                   "cases": cases, "momentum_required_or_COM_closure_claim": None,
                                                   "work_definition": "Recorded generalized vertical force times separately saved actual pelvis_ty speed, not a derivative of rounded positions"},
              "sources_before": sources, "sources_after": after, "sources_unchanged": True,
              "new_FK_IK_native_calls": 0, "new_Claude_branch_read": False}
    with output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    print(json.dumps({"output": fp(output), "byte_equal_STO": result["fresh400_reproducibility"]["all_17_native_files_byte_equal"], "trace_equal": trace["all_rows_exact_decoded_numeric_and_semantic_equal"], "prefix_s": hi - lo}), flush=True)


if __name__ == "__main__":
    main()
