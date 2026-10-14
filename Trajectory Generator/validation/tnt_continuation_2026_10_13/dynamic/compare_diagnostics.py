"""Read finalized paired dynamic diagnostics, without simulation or training.

Separate raw generator error, slew/governor distortion, and physical tracking.
Force channels are post-hoc plant diagnostics only. They do not select or fit
any controller parameters. An incomplete run is never reported as a full pass.
"""
from __future__ import annotations

import argparse
import io
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from run_tnt_diagnostic import fingerprint, local_path, write_json


def storage(path):
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    end = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    names = lines[end + 1].split()
    body = "\n".join(line for line in lines[end + 2:] if line.strip())
    if not body:
        return np.array([]), {}
    data = np.atleast_2d(np.loadtxt(io.StringIO(body)))
    if names[0] != "time" or len(names) != data.shape[1]:
        raise ValueError(f"Invalid storage table {path}")
    t = data[:, 0]
    if not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError(f"Invalid native timestamps {path}")
    return t, {name: data[:, i] for i, name in enumerate(names[1:], 1)}


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def statistics(x):
    return {"mean": float(np.mean(x)), "rms": rms(x),
            "min": float(np.min(x)), "max": float(np.max(x))}


def load_run(folder):
    receipt = json.loads((folder / "receipt.json").read_text(encoding="utf-8"))
    if receipt["status"] not in {"completed", "aborted", "simulation_completed_finalization_recovered"}:
        raise ValueError(f"Run not finalized: {folder}: {receipt['status']}")
    trace = json.loads((folder / "generator_trace.json").read_text(encoding="utf-8"))
    tables = {p.stem.removeprefix(receipt["effective_simulator_config"]["output_prefix"] + "_"): storage(p)
              for p in sorted((folder / "sim_outputs").glob("*.sto"))}
    if not trace or not len(tables["states"][0]):
        raise ValueError("Run has no recorded coverage")
    before = [fingerprint(p) for p in sorted(folder.rglob("*")) if p.is_file()]
    return {"folder": folder, "receipt": receipt, "trace": trace, "tables": tables, "before": before}


def summarize(run, window=None):
    receipt, trace, tables = run["receipt"], run["trace"], run["tables"]
    t, state = tables["states"]
    cfg = receipt["effective_simulator_config"]
    if window is None:
        window = (float(t[0]), float(t[-1]))
    mask = (t >= window[0] - 1e-8) & (t <= window[1] + 1e-8)
    rows = [r for r in trace if window[0] - 1e-8 <= r["time"] <= window[1] + 1e-8]
    if not np.any(mask) or not rows:
        raise ValueError("Empty requested comparison support")
    required = ("states", "kinematics_reference", "sea_controls", "sea_torques", "sea_diagnostics",
                "online_grf", "recruitment", "reserve_torques", "power")
    for key in required:
        if not np.array_equal(tables[key][0], t):
            raise ValueError(f"Native output grids differ: {key}")
    ref, control = tables["kinematics_reference"][1], tables["sea_controls"][1]
    diag, force = tables["sea_diagnostics"][1], tables["online_grf"][1]
    recruitment, reserve = tables["recruitment"][1], tables["reserve_torques"][1]
    theta = np.array([r["thigh_actual_deg"] for r in rows])
    result = {
        "native_window_s": [float(t[mask][0]), float(t[mask][-1])],
        "native_samples": int(mask.sum()), "decision_samples": len(rows),
        "all_native_tables_finite": {key: all(np.isfinite(v).all() for v in values.values())
                                     for key, (_, values) in tables.items()},
        "joints": {},
        "actual_thigh_deg": statistics(theta),
        "pelvis_height_m": statistics(state["pelvis_ty_q"][mask]),
        "force_diagnostics_use": "Post-hoc plant observation only, not reward/controller selection",
        "applied_left_plant": {key: statistics(force[key][mask]) for key in
                               ("left_force_y", "left_normal_force", "left_penetration", "left_slip_speed")},
        "recruitment": {key: statistics(recruitment[key][mask]) for key in
                        ("muscle_share", "muscle_capable_share", "muscle_capable_reserve_norm",
                         "unactuated_reserve_norm", "residual_norm")},
        "pelvis_reserves": {key: statistics(values[mask]) for key, values in reserve.items()
                            if key.startswith("pelvis_")},
        "reserve_units": "Generalized force N for pelvis translations; torque Nm for rotations; mixed norms retained only as original diagnostics",
    }
    root = ET.parse(local_path(cfg["model_file"])).getroot()
    mass = sum(float(body.findtext("mass")) for body in root.iter("Body") if body.findtext("mass") is not None)
    result["body_mass_kg"] = mass
    result["applied_left_plant"]["peak_normal_force_bw"] = float(np.max(force["left_normal_force"][mask]) / (mass * 9.80665))
    for j, (name, sea) in enumerate(zip(cfg["pros_coords"], (cfg["sea_knee_name"], cfg["sea_ankle_name"]))):
        actual, served = state[name + "_q"][mask], ref[name + "_q_ref"][mask]
        expected = np.array([r["prescribed_same_time_q_rad"][name] for r in rows])
        actual_100 = np.array([r["actual_at_start_rad"][j] for r in rows])
        served_100 = np.array([r["served_at_start_rad"][j] for r in rows])
        raw = np.array([r["raw_target_rad"][j] for r in rows])
        bounded = np.array([r["bounded_target_rad"][j] for r in rows])
        limited = np.array([r["slew_limited_target_rad"][j] for r in rows])
        limits = receipt["inherited_reference_and_safety_config"]["truncation_bounds_rad"][name]
        result["joints"][name] = {
            "actual_range_deg": np.rad2deg([actual.min(), actual.max()]),
            "raw_range_deg": np.rad2deg([raw.min(), raw.max()]),
            "served_range_deg": np.rad2deg([served.min(), served.max()]),
            "native_actual_minus_served_rmse_deg": float(np.rad2deg(rms(actual - served))),
            "native_actual_minus_served_max_abs_deg": float(np.rad2deg(np.max(np.abs(actual - served)))),
            "decision_raw_minus_prescribed_rmse_deg": float(np.rad2deg(rms(raw - expected))),
            "decision_served_minus_prescribed_rmse_deg": float(np.rad2deg(rms(served_100 - expected))),
            "decision_actual_minus_prescribed_rmse_deg": float(np.rad2deg(rms(actual_100 - expected))),
            "decision_served_minus_raw_rmse_deg": float(np.rad2deg(rms(served_100 - raw))),
            "raw_bounds_clipping_fraction": float(np.mean(abs(raw - bounded) > 1e-10)),
            "target_slew_limited_fraction": float(np.mean(abs(bounded - limited) > 1e-10)),
            "native_joint_bounds_violations": int(np.sum((actual < limits[0]) | (actual > limits[1]))),
            "normalized_control": statistics(control[name][mask]),
            "normalized_control_saturation_fraction_ge099": float(np.mean(abs(control[name][mask]) >= .99)),
            "motor_torque_Nm": statistics(diag[sea + "_tau_input_plugin"][mask]),
            "motor_torque_saturation_fraction": float(np.mean(diag[sea + "_tau_input_saturated"][mask] > .5)),
            "spring_torque_Nm": statistics(diag[sea + "_tau_spring_state"][mask]),
            "torque_tracking_error_Nm": statistics(diag[sea + "_tau_error"][mask]),
            "plugin_python_motor_torque_difference_max_abs_Nm": float(np.max(abs(diag[sea + "_tau_input_plugin_minus_python"][mask]))),
            "joint_power_W": statistics(tables["power"][1][sea + "_joint_power"][mask]),
        }
    if receipt["mode"] == "tnt":
        states = np.array([r["generator"]["state"] for r in rows])
        result["TNT"] = {
            "internal_state_counts": {int(s): int(np.sum(states == s)) for s in np.unique(states)},
            "phase": statistics(np.array([r["generator"]["T"] for r in rows])),
            "rate": statistics(np.array([r["generator"]["rate"] for r in rows])),
            "completed_swings_at_last_decision": rows[-1]["generator"]["completed"],
            "events": [{"time": r["time"], "event": r["generator"]["event"]}
                       for r in rows if r["generator"].get("event")],
            "state_semantics": "Internal modes, not measured HS/TO or load states",
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prescribed", type=Path, required=True)
    parser.add_argument("--tnt", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output must be new")
    runs = [load_run(args.prescribed.resolve()), load_run(args.tnt.resolve())]
    common = (max(r["tables"]["states"][0][0] for r in runs),
              min(r["tables"]["states"][0][-1] for r in runs))
    result = {"schema_version": 1, "analyzer": fingerprint(Path(__file__)),
              "comparison_window_s": common, "runs": {},
              "qualification_limit": "Short same-trial dynamic execution diagnostic, within calibration period; no clinical, holdout or speed generalization claim"}
    for run in runs:
        receipt = run["receipt"]
        result["runs"][receipt["mode"]] = {
            "receipt": fingerprint(run["folder"] / "receipt.json"),
            "status": receipt["status"], "error": receipt.get("error"),
            "requested_window_s": receipt["requested_window_s"],
            "reached_time_s": receipt["final_time_s"],
            "elapsed_wall_time_s": receipt["elapsed_wall_time_s"],
            "full_recorded_metrics": summarize(run), "common_window_metrics": summarize(run, common),
        }
        if run["before"] != [fingerprint(p) for p in sorted(run["folder"].rglob("*")) if p.is_file()]:
            raise AssertionError("Finalized run changed during analysis")
    a, b = [r["receipt"] for r in runs]
    excluded = {"output_dir"}
    result["paired_physical_config_differences"] = {
        k: [a["effective_simulator_config"].get(k), b["effective_simulator_config"].get(k)]
        for k in set(a["effective_simulator_config"]) | set(b["effective_simulator_config"])
        if k not in excluded and a["effective_simulator_config"].get(k) != b["effective_simulator_config"].get(k)}
    result["reference_and_safety_config_equal"] = a["inherited_reference_and_safety_config"] == b["inherited_reference_and_safety_config"]
    result["run_artifacts_unchanged"] = True
    write_json(args.output, result)
    print(json.dumps({"comparison": str(args.output), "status": {k: v["status"] for k, v in result["runs"].items()}}))


if __name__ == "__main__":
    main()
