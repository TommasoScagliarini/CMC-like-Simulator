"""Consume finalized prescribed screening runs; never run an environment or IK.

Descriptive metrics use frozen measured-cycle boundaries, saved native states,
and the complete effective config. Seven sparse Velocity-stage FK poses supply
COM momentum; no Dynamics/Acceleration stage, assembly, integration or fit is
performed. Writes exclusive new diagnostics and, optionally, canonical plots.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import fields, replace
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.dont_write_bytecode = True
REPO = Path(__file__).resolve().parents[3]
AUDIT = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
PROTOCOL = HERE / "analysis_protocol.json"
PAIR = REPO / "Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07"
TRAP = np.trapezoid if hasattr(np, "trapezoid") else np.trapz


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def relative(path):
    return Path(path).resolve().relative_to(REPO).as_posix()


def path(raw):
    normalized = str(raw).replace("\\", "/")
    candidate = Path(normalized)
    if candidate.exists():
        return candidate.resolve()
    if not candidate.is_absolute() and (REPO / candidate).exists():
        return (REPO / candidate).resolve()
    for marker in ("/validation/", "/results/", "/models/", "/Trajectory Generator/", "/online_grf_profiles/", "/plugins/"):
        if marker in normalized:
            candidate = REPO / marker.strip("/") / normalized.split(marker, 1)[1]
            if candidate.exists():
                return candidate.resolve()
    raise FileNotFoundError(raw)


def fingerprint(p):
    p = Path(p)
    return {"path": relative(p), "sha256": sha(p), "size_bytes": p.stat().st_size}


def check_manifest(manifest):
    failures = [name for name, expected in manifest.items() if sha(path(name)) != expected]
    if failures:
        raise ValueError(f"Protected inputs changed: {failures}")
    return {"count": len(manifest), "all_match": True}


def inventory(directory):
    return {relative(p): {"sha256": sha(p), "size_bytes": p.stat().st_size}
            for p in sorted(Path(directory).rglob("*")) if p.is_file()}


def exclusive_json(p, data):
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write("\n")


def module(name, p):
    spec = importlib.util.spec_from_file_location(name, p)
    result = importlib.util.module_from_spec(spec)
    sys.modules[name] = result
    spec.loader.exec_module(result)
    return result


def helpers(protocol):
    for p in (REPO, REPO / "Trajectory Generator/baseline_MLP"):
        sys.path.insert(0, str(p))
    import win_runtime  # noqa: F401: Windows DLL search setup, no environment
    from config import SimulatorConfig
    from validation import validate_sim_results as validator
    from kinematics_interpolator import KinematicsInterpolator
    from model_loader import _load_plugin, _configure_geometry_search_paths
    import opensim
    bindings = protocol["source_bindings"]
    numerical = module("screen_numerical", path(bindings["numerical"]["path"]))
    transfer = module("screen_transfer_helpers", path(bindings["transfer"]["path"]))
    balance = module("screen_balance_helpers", path(bindings["balance"]["path"]))
    render = module("screen_prescribed_render", path(bindings["render"]["path"]))
    precision = int(opensim.CoordinateActuator().getOutput("actuation").getNumberOfSignificantDigits())
    if precision != 8:
        raise ValueError("SEA serialization evidence requires 8 significant plugin Output digits.")
    _load_plugin(str(REPO / "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"))
    _load_plugin(str(REPO / "plugins/OnlineGRFContact"))
    return SimpleNamespace(cfg=SimulatorConfig, validator=validator, numerical=numerical,
                           transfer=transfer, balance=balance, render=render,
                           interpolator=KinematicsInterpolator, osim=opensim,
                           geometry_paths=_configure_geometry_search_paths, precision=precision)


def supported(table, grid, column):
    if table is None or column not in table.index or not len(table.time):
        raise ValueError(f"Missing required recorded channel {column}")
    if grid[0] < table.time[0] or grid[-1] > table.time[-1]:
        raise ValueError(f"Interpolation would extrapolate {column}")
    values = np.interp(grid, table.time, table.data[:, table.index[column]])
    if not np.all(np.isfinite(values)):
        raise ValueError(f"Nonfinite recorded channel {column}")
    return values


def weighted(values, grid, unit):
    duration = float(grid[-1] - grid[0])
    return {"unit": unit, "mean": float(TRAP(values, grid) / duration),
            "mean_abs": float(TRAP(np.abs(values), grid) / duration),
            "rms": float(np.sqrt(TRAP(values * values, grid) / duration)),
            "min": float(np.min(values)), "max": float(np.max(values)),
            "max_abs": float(np.max(np.abs(values))), "quadrature_points": len(grid)}


def signed_integrals(values, grid):
    return {"signed": float(TRAP(values, grid)), "absolute": float(TRAP(np.abs(values), grid)),
            "positive": float(TRAP(np.maximum(values, 0), grid)),
            "negative": float(TRAP(np.minimum(values, 0), grid))}


def sparse_com(model_path, times, states, cfg, h):
    """Actual q/qdot separately interpolated at covered, exact frozen knots."""
    h.geometry_paths(str(model_path))
    model = h.osim.Model(str(model_path))
    if model.getConstraintSet().getSize():
        raise ValueError("Sparse COM audit does not assemble constrained models.")
    state = model.initSystem()
    coords = [model.getCoordinateSet().get(i) for i in range(model.getCoordinateSet().getSize())]
    expected = {c.getName() for c in coords}
    if expected != {name[:-2] for name in states.columns if name.endswith("_q")}:
        raise ValueError("Actual state coordinate schema differs from model.")
    if expected != {name[:-5] for name in states.columns if name.endswith("_qdot")}:
        raise ValueError("Actual speed schema differs from model.")
    if states.in_degrees:
        raise ValueError("Saved native states must be rad/m, not degrees.")
    bodies = [model.getBodySet().get(i) for i in range(model.getBodySet().getSize())]
    mass, gravity = float(model.getTotalMass(state)), h.balance.v3(model.getGravity())
    if mass <= 0 or gravity[0] != 0 or gravity[2] != 0 or gravity[1] >= 0:
        raise ValueError("Mass/vertical gravity contract differs.")
    rows, pos_error, vel_error, q_error, speed_error = {}, 0., 0., 0., 0.
    for t in times:
        grid = np.array([t])
        state.setTime(float(t))
        for c in coords:
            q = float(supported(states, grid, c.getName() + "_q")[0])
            speed = float(supported(states, grid, c.getName() + "_qdot")[0])
            c.setValue(state, q, False)
            c.setSpeedValue(state, speed)
            q_error = max(q_error, abs(float(c.getValue(state)) - q))
            speed_error = max(speed_error, abs(float(c.getSpeedValue(state)) - speed))
        model.realizeVelocity(state)
        position = sum((float(b.getMass()) * h.balance.v3(b.findStationLocationInGround(state, b.getMassCenter()))
                        for b in bodies), np.zeros(3)) / mass
        velocity = sum((float(b.getMass()) * h.balance.v3(b.findStationVelocityInGround(state, b.getMassCenter()))
                        for b in bodies), np.zeros(3)) / mass
        pos_error = max(pos_error, float(np.max(np.abs(position - h.balance.v3(model.calcMassCenterPosition(state))))))
        vel_error = max(vel_error, float(np.max(np.abs(velocity - h.balance.v3(model.calcMassCenterVelocity(state))))))
        rows[str(float(t))] = {"time_s": float(t), "position_m": position.tolist(), "velocity_m_s": velocity.tolist()}
    for c in coords:
        c.setSpeedValue(state, 1. if c.getName() == "pelvis_ty" else 0.)
    model.realizeVelocity(state)
    virtual_error = float(np.max(np.abs(np.array([h.balance.v3(b.findStationVelocityInGround(state, b.getMassCenter()))
                                                 for b in bodies]) - [0, 1, 0])))
    contract = h.balance.root_translation_contract(model_path)
    if max(pos_error, vel_error, virtual_error, q_error, speed_error) > 1e-10:
        raise AssertionError("Sparse COM/coordinate/vertical virtual-work parity failed.")
    return {"mass_kg": mass, "gravity_m_s2": gravity.tolist(), "BW_N": mass * -gravity[1],
            "covered_exact_knots": rows, "position_parity_max_error_m": pos_error,
            "velocity_parity_max_error_m_s": vel_error, "coordinate_assignment_max_error_native_unit": q_error,
            "speed_assignment_max_error_native_unit_s": speed_error,
            "unit_pelvis_ty_speed_body_velocity_max_error_m_s": virtual_error,
            "root_translation_contract": contract, "assembly_or_constraint_enforcement_used": False}


def tracking(grid, tables, raw, production, cfg, h):
    out = {}
    for coord in tables["kinematics"].columns:
        translation = coord in cfg.translation_coords
        unit, factor = ("m", 1.) if translation else ("deg", 180. / np.pi)
        actual = supported(tables["kinematics"], grid, coord)
        speed = supported(tables["states"], grid, coord + "_qdot")
        served = supported(tables["kinematics_reference"], grid, coord + "_q_ref")
        served_speed = supported(tables["kinematics_reference"], grid, coord + "_qdot_ref")
        entry = {"actual": weighted(actual * factor, grid, unit),
                 "actual_minus_served": {"position": weighted((actual-served)*factor, grid, unit),
                                          "speed": weighted((speed-served_speed)*factor, grid, unit + "/s")}}
        source = h.validator.ref_series(raw, coord, cfg)
        if source is not None:
            t, values, column = source
            if grid[0] < t[0] or grid[-1] > t[-1]:
                raise ValueError("Raw IK does not cover the declared analysis window.")
            raw_q = np.interp(grid, t, values)
            entry["actual_minus_source_raw_ik"] = {"position": weighted((actual-raw_q)*factor, grid, unit), "column": column}
        if coord not in production._splines:
            raise ValueError("Own production filtered IK coordinate unavailable.")
        if grid[0] < production.time_bounds[0] or grid[-1] > production.time_bounds[1]:
            raise ValueError("Filtered full-trial IK would extrapolate.")
        spline = production._splines[coord]
        entry["actual_minus_source_filtered_ik"] = {"position": weighted((actual-spline(grid))*factor, grid, unit),
                                                    "speed": weighted((speed-spline(grid, 1))*factor, grid, unit + "/s")}
        entry["served_minus_source_filtered_ik"] = {"position": weighted((served-spline(grid))*factor, grid, unit),
                                                   "speed": weighted((served_speed-spline(grid, 1))*factor, grid, unit + "/s")}
        out[coord] = entry
    return {"coordinates": out}


def reserve_metrics(grid, tables, cfg, mass, bw):
    out, so = {}, tables["so_torque_diagnostics"]
    for column in tables["reserve_torques"].columns:
        if not column.endswith("_reserve_torque"):
            continue
        coord = column.removesuffix("_reserve_torque")
        translation = coord in cfg.translation_coords
        unit = "N" if translation else "Nm"
        tau = supported(tables["reserve_torques"], grid, column)
        speed = supported(tables["states"], grid, coord + "_qdot")
        force_stats = weighted(tau, grid, unit)
        if translation:
            force_stats["rms_BW"] = force_stats["rms"] / bw
        work = signed_integrals(tau * speed, grid)
        entry = {"unit": unit, "impulse_unit": "Ns" if translation else "Nm*s",
                 "force_or_torque": force_stats, "impulse": signed_integrals(tau, grid),
                 "speed": weighted(speed, grid, "m/s" if translation else "rad/s"),
                 "work_J": work, "work_J_per_kg": {k: v / mass for k, v in work.items()},
                 "power_W": weighted(tau * speed, grid, "W")}
        block = "tau_muscle_" + coord
        if block in so.index:
            muscle, residual = supported(so, grid, block), supported(so, grid, "tau_residual_" + coord)
            abs_m, abs_r = float(TRAP(np.abs(muscle), grid)), float(TRAP(np.abs(tau), grid))
            entry["muscle_vs_reserve_absolute_generalized_impulse"] = {
                "muscle": abs_m, "reserve": abs_r, "unit": entry["impulse_unit"],
                "muscle_fraction": abs_m / (abs_m+abs_r) if abs_m+abs_r > 0 else None,
                "reserve_fraction": abs_r / (abs_m+abs_r) if abs_m+abs_r > 0 else None,
                "denominator_zero": abs_m+abs_r == 0}
            entry["so_residual"] = weighted(residual, grid, unit)
            entry["recorded_muscle_row_capacity"] = weighted(supported(so, grid, "muscle_row_capacity_" + coord), grid, unit)
        out[coord] = entry
    return {"per_coordinate": out, "work_definition": "saved generalized force/torque times independently saved actual coordinate speed; positive/negative signed pieces; no derivative of q"}


def window_metrics(start, end, tables, measured, raw, production, cfg, com, h):
    supports = [t for t in tables.values() if t is not None and len(t.time)] + [measured]
    lo, hi = max(start, max(t.time[0] for t in supports)), min(end, min(t.time[-1] for t in supports))
    complete = lo == start and hi == end
    result = {"requested_range_s": [start, end], "requested_duration_s": end-start,
              "coverage": "complete" if complete else "partial" if hi > lo else "unavailable",
              "observed_intersection_range_s": [float(lo), float(hi)] if hi > lo else None,
              "boundary_clamping_or_extrapolation_used": False}
    if hi <= lo:
        return result
    anchor = tables["kinematics"].time
    frozen = np.array(com["requested_exact_knots_s"])
    grid = np.unique(np.r_[lo, anchor[(anchor > lo) & (anchor < hi)], frozen[(frozen > lo) & (frozen < hi)], hi])
    result["observed_duration_s"] = float(hi-lo)
    mass, bw = com["mass_kg"], com["BW_N"]
    values = {"left_online": supported(tables["online_grf"], grid, "left_force_y"),
              "right_measured": supported(measured, grid, "ground_force2_vy"),
              "left_measured": supported(measured, grid, "ground_force1_vy")}
    result["forces"] = {}
    for name, force in values.items():
        stats = weighted(force, grid, "N")
        stats.update(mean_N=stats["mean"], mean_BW=stats["mean"]/bw, rms_BW=stats["rms"]/bw,
                     impulse_Ns=float(TRAP(force, grid)))
        result["forces"][name] = stats
    result["left_online_minus_measured_impulse_Ns"] = float(TRAP(values["left_online"]-values["left_measured"], grid))
    result["tracking"] = tracking(grid, tables, raw, production, cfg, h)
    result["reserves"] = reserve_metrics(grid, tables, cfg, mass, bw)
    result["penetration"] = {side: weighted(supported(tables["online_grf"], grid, side + "_penetration"), grid, "m")
                             for side in ("left", "right")}
    for side in result["penetration"]:
        result["penetration"][side]["applied_online_side"] = side in cfg.online_grf_applied_sides
    begin, finish = com["covered_exact_knots"].get(str(float(start))), com["covered_exact_knots"].get(str(float(end)))
    if complete and begin is not None and finish is not None:
        momentum = mass * (finish["velocity_m_s"][1] - begin["velocity_m_s"][1])
        left, right = (result["forces"][n]["impulse_Ns"] for n in ("left_online", "right_measured"))
        reserve = result["reserves"]["per_coordinate"]["pelvis_ty"]["impulse"]["signed"]
        weight = bw * (end-start)
        residual = left+right+reserve-weight-momentum
        result["vertical_balance"] = {"available": True, "left_online_Ns": left, "right_measured_Ns": right,
                                      "pelvis_ty_reserve_Ns": reserve, "weight_Ns": weight,
                                      "actual_COM_momentum_delta_Ns": momentum,
                                      "closure_residual_Ns": residual,
                                      "closure_residual_over_BW_duration": residual / weight,
                                      "implied_left_required_minus_reserve_Ns": weight+momentum-right}
    else:
        result["vertical_balance"] = {"available": False, "reason": "requested exact endpoints not both covered/frozen knots; no replacement endpoints"}
    return result


def prefix_compare(case_id, run, tables, cfg, receipt, h):
    if case_id not in ("P4_A1", "AB06_control"):
        return {"applicable": False}
    old = path(receipt["old_matched_runtime"]["path"])
    original = read_json(old)
    old_prefix = original["effective_simulator_config"]["output_prefix"]
    results = {}
    for name, new in tables.items():
        old_table = h.transfer.load_table_allow_empty(old.parent/"sim_outputs"/f"{old_prefix}_{name}.sto", h.validator)
        if old_table is None or new is None or not len(new.time):
            results[name] = {"available": False}
            continue
        if len(old_table.time) < len(new.time):
            raise ValueError("Historical physical prefix is shorter than current recording.")
        same_columns = new.columns == old_table.columns
        same_time = np.array_equal(new.time, old_table.time[:len(new.time)])
        same_values = same_columns and np.array_equal(new.data, old_table.data[:len(new.time)])
        results[name] = {"available": True, "rows": len(new.time), "same_columns": same_columns,
                         "times_exactly_equal": same_time, "decoded_values_exactly_equal": same_values,
                         "max_abs_numeric_difference": float(np.max(np.abs(new.data-old_table.data[:len(new.time)]))) if same_columns else None,
                         "old_file": fingerprint(old_table.path), "new_file": fingerprint(new.path)}
    required = ("kinematics", "states", "online_grf", "reserve_torques", "sea_states", "sea_controls", "sea_torques", "sea_derivatives", "sea_diagnostics", "kinematics_reference", "recruitment", "so_torque_diagnostics", "power")
    missing = [name for name in required if not results.get(name, {}).get("available")]
    available = sum(v["available"] for v in results.values())
    return {"applicable": True, "old_receipt": fingerprint(old), "old_receipt_status_preserved": original["status"],
            "old_receipt_steps_preserved": original["steps"], "tables": results,
            "available_table_count": available, "missing_required_native_tables": missing,
            "all_available_tables_exact_numeric_prefix": available > 0 and not missing and all(v.get("times_exactly_equal") and v.get("decoded_values_exactly_equal") for v in results.values() if v["available"]),
            "claim": "Decoded recorded physical prefix equality; whole-file hashes/header row counts differ. Historical AB06 terminal callback failure remains failed."}


def trace_audit(run, receipt):
    p = run / "trace.jsonl"
    if not p.is_file():
        return {"available": False}
    rows = [json.loads(line) for line in p.read_text(encoding="utf-8").splitlines() if line.strip()]
    sample_dt = float(receipt["effective_env_config"]["detector_sample_dt_s"])
    segment_dt = float(receipt["effective_env_config"]["segment_duration"])
    expected_count = int(round(segment_dt / sample_dt))
    violations, events, samples = [], [], 0
    for i, row in enumerate(rows, 1):
        binary = row.get("binary_phase_sensor_samples", [])
        times = np.array([float(x["time_s"]) for x in binary])
        samples += len(times)
        if row.get("segment_completed"):
            expected = float(row["time"]) - segment_dt + np.arange(1, expected_count+1)*sample_dt
            if len(times) != expected_count or not np.allclose(times, expected, atol=1e-8, rtol=0):
                violations.append(i)
        events += [{"callback": i, "recorded_transition": x}
                   for x in row.get("phase_fsm", {}).get("accepted_transitions_this_step", [])]
    return {"available": True, "source": fingerprint(p), "callbacks": len(rows),
            "callbacks_equal_receipt_steps": len(rows) == receipt["steps"],
            "last_callback_time_s": rows[-1].get("time") if rows else None,
            "binary_sensor_sample_count": samples, "expected_samples_per_complete_segment": expected_count,
            "complete_segment_sensor_grid_violation_callbacks": violations,
            "last_phase_fsm": rows[-1].get("phase_fsm") if rows else None,
            "accepted_transitions": events,
            "event_semantics": "Existing actor/FSM transport only; not a new detector, measured event oracle, or regenerated event stream."}


def analyze(case_id, dynamic, protocol, h, plots):
    case = dynamic["cases"][case_id]
    folder = AUDIT / "dynamics" / case_id
    run = folder / "run"
    receipt_path = run / "receipt.json"
    receipt = read_json(receipt_path)
    if receipt.get("status") not in ("completed", "interrupted", "failed") or not receipt.get("finished_utc"):
        raise ValueError(f"{case_id}: outputs are not finalized; no partial files are consumed.")
    if receipt.get("case", {}).get("id") != case_id:
        raise ValueError("Receipt case identity differs.")
    output = HERE / case_id / "metrics.json"
    if output.exists():
        raise FileExistsError(output)
    before = inventory(run)
    final_info = receipt.get("final_info") or {}
    # A finalized startup/reset failure is a real result, not missing success.
    # Prepared requested bounds are labelled requested, never invented actual.
    if ("effective_simulator_config" not in receipt or "effective_env_config" not in receipt
            or "actual_episode_start_s" not in receipt or "actual_episode_end_s" not in receipt):
        result = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
                  "case_id": case_id, "subject": case["subject"], "case": case,
                  "receipt": fingerprint(receipt_path), "analysis_status": "unavailable_finalized_startup_or_reset_failure",
                  "runtime": {"status": receipt["status"], "steps": receipt.get("steps"),
                              "error": receipt.get("error"), "failure": receipt.get("final_failure"),
                              "end_reason": final_info.get("end_reason"), "close_error": receipt.get("close_error"),
                              "completed_requested_duration": receipt.get("completed_requested_duration", False),
                              "requested_range_s_from_preparation_only": receipt.get("requested_time_range"),
                              "actual_episode_range_s": None, "final_clocks_s": receipt.get("final_clocks_s")},
                  "windows": {}, "tables": {}, "prefix_comparison": {"applicable": case_id in ("P4_A1", "AB06_control"), "available_table_count": 0, "all_available_tables_exact_numeric_prefix": False},
                  "analysis_protocol": fingerprint(PROTOCOL), "dynamic_protocol": fingerprint(AUDIT/"dynamic_protocol.json"),
                  "diagnostic_script": fingerprint(Path(__file__)), "new_integrations": 0, "new_IK_solves": 0,
                  "new_FK_poses": 0, "policy_loaded": False, "clinical_or_hardware_necessity_gates_applied": False,
                  "run_artifacts_unchanged": {"count": len(before), "passed": inventory(run) == before, "files": before}}
        exclusive_json(output, result)
        return output, result
    expected = {f.name for f in fields(h.cfg) if f.init}
    snapshot = receipt["effective_simulator_config"]
    if set(snapshot) != expected:
        raise ValueError("Effective simulator configuration is incomplete.")
    cfg = h.cfg(**snapshot)
    model, ik = path(snapshot["model_file"]), path(snapshot["kinematics_file"])
    if sha(model) != case["model"]["sha256"] or sha(ik) != case["IK"]["sha256"]:
        raise ValueError("Model/IK differ from prospective case bindings.")
    start, end = float(receipt["actual_episode_start_s"]), float(receipt["actual_episode_end_s"])
    outputs = run / "sim_outputs"
    cfg = replace(cfg, model_file=str(model), kinematics_file=str(ik), model_bundle_dir=str(model.parent), output_dir=str(outputs))
    coverage_cfg = replace(cfg, t_start=start, t_end=end)
    tables = {p.stem.removeprefix(cfg.output_prefix + "_"): h.transfer.load_table_allow_empty(p, h.validator)
              for p in sorted(outputs.glob(cfg.output_prefix + "_*.sto"))}
    required = ("kinematics", "states", "kinematics_reference", "reserve_torques", "reserve_controls", "recruitment", "so_torque_diagnostics", "online_grf", "sea_states", "sea_controls", "sea_torques", "sea_derivatives", "sea_diagnostics", "power")
    missing = [n for n in required if n not in tables or tables[n] is None or not len(tables[n].time)]
    anchor = tables.get("kinematics")
    result = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(), "case_id": case_id,
              "subject": case["subject"], "case": case, "receipt": fingerprint(receipt_path),
              "runtime": {"status": receipt["status"], "steps": receipt.get("steps"), "error": receipt.get("error"),
                          "failure": receipt.get("final_failure"), "end_reason": final_info.get("end_reason"),
                          "close_error": receipt.get("close_error"),
                          "completed_requested_duration": receipt.get("completed_requested_duration", False),
                          "actual_episode_range_s": [start, end], "wall_time_s": receipt.get("wall_time_s")},
              "analysis_protocol": fingerprint(PROTOCOL), "dynamic_protocol": fingerprint(AUDIT/"dynamic_protocol.json"),
              "diagnostic_script": fingerprint(Path(__file__)), "missing_required_tables": missing,
              "windows": {}, "new_integrations": 0, "new_IK_solves": 0, "policy_loaded": False,
              "clinical_or_hardware_necessity_gates_applied": False}
    result["effective_config_source"] = "exact finalized receipt snapshot; only local input/output path rebasing for read-only consumers"
    result["source_files"] = {}
    for name, entry in receipt["source_files"].items():
        actual = fingerprint(path(entry["path"]))
        if actual["sha256"] != entry["sha256"]:
            raise ValueError(f"Receipt source binding changed: {name}")
        result["source_files"][name] = actual
    result["trace_transport"] = trace_audit(run, receipt)
    clocks = receipt.get("final_clocks_s") or {"environment": receipt.get("final_time_s"), "runner": receipt.get("final_runner_time_s"), "state": receipt.get("final_state_time_s")}
    result["runtime"].update(final_clocks_s=clocks, clocks_at_requested_end=all(v is not None and abs(float(v)-end) <= 1e-7 for v in clocks.values()))
    result["tables"] = h.transfer.partial_table_audit(tables, coverage_cfg, anchor.time if anchor is not None else np.array([]), h.numerical)
    finite = all(v.get("all_values_finite") and v.get("time_finite") for v in result["tables"].values())
    result["numerical_data_finite"] = finite
    result["prefix_comparison"] = prefix_compare(case_id, run, tables, cfg, receipt, h)
    if missing or anchor is None or len(anchor.time) < 2 or not finite:
        result["analysis_status"] = "unavailable_or_invalid_recorded_numerics"
    else:
        if any(not np.array_equal(t.time, anchor.time) for t in tables.values()):
            raise ValueError("Native STO grids differ; SEA identities cannot use interpolated states.")
        selection = read_json(PAIR/"selection.json")["subjects"][case["subject"]]
        cycles = selection["primary_cycles"]
        knots = sorted({float(c[k]) for c in cycles for k in ("hs_s", "to_s", "next_hs_s")})
        covered = [t for t in knots if anchor.time[0] <= t <= anchor.time[-1]]
        com = sparse_com(model, covered, tables["states"], cfg, h)
        com["requested_exact_knots_s"] = knots
        com["missing_exact_knots_s"] = [t for t in knots if t not in covered]
        result["actual_COM"] = com
        result["mass_kg"], result["BW_N"] = com["mass_kg"], com["BW_N"]
        result["duration_primary_s"] = cycles[-1]["next_hs_s"] - cycles[0]["hs_s"]
        if list(cfg.online_grf_applied_sides) != ["left"]:
            raise ValueError("Balance contract requires left online + right prescribed measured GRF.")
        measured_path = path(selection["source_fingerprints"]["measured_GRF"]["path"])
        measured = h.validator.load_table(measured_path)
        raw = h.validator.load_table(ik)
        production = h.interpolator(replace(cfg, model_file=str(model), kinematics_file=str(ik)))
        props = h.numerical.xml_sea_properties(model, [cfg.sea_knee_name, cfg.sea_ankle_name], h.validator)
        result["SEA_all_recorded"] = h.numerical.sea_audit(cfg, tables, props, h.precision)
        h.transfer.command_limit_audit(cfg, tables, result["SEA_all_recorded"], h.numerical)
        for name, sea in result["SEA_all_recorded"].items():
            sea["motor_angle_rad"] = h.numerical.metrics(supported(tables["sea_states"], anchor.time, name + "_motor_angle"))
            sea["motor_speed_rad_s"] = h.numerical.metrics(supported(tables["sea_states"], anchor.time, name + "_motor_speed"))
        result["recruitment_all_recorded"] = h.numerical.reserves_and_recruitment(cfg, tables)
        result["contact_all_recorded"] = h.transfer.high_rate_contact_audit(tables, cfg, receipt["effective_env_config"], {}, h.numerical)
        queries = {"primary_all": (cycles[0]["hs_s"], cycles[-1]["next_hs_s"]), "all_recorded": (float(anchor.time[0]), float(anchor.time[-1]))}
        for i, c in enumerate(cycles, 1):
            queries.update({f"cycle_{i}": (c["hs_s"], c["next_hs_s"]), f"cycle_{i}_stance": (c["hs_s"], c["to_s"]), f"cycle_{i}_swing": (c["to_s"], c["next_hs_s"])})
        for key, (a, b) in queries.items():
            result["windows"][key] = window_metrics(float(a), float(b), tables, measured, raw, production, cfg, com, h)
        mask = (anchor.time >= queries["primary_all"][0]) & (anchor.time <= queries["primary_all"][1])
        if np.count_nonzero(mask):
            native_primary = {k: replace(v, time=v.time[mask], data=v.data[mask]) for k, v in tables.items()}
            result["SEA_primary_native_samples"] = h.numerical.sea_audit(cfg, native_primary, props, h.precision)
            h.transfer.command_limit_audit(cfg, native_primary, result["SEA_primary_native_samples"], h.numerical)
            result["recruitment_primary_native_samples"] = h.numerical.reserves_and_recruitment(cfg, native_primary)
        result["joint_ranges"] = {}
        ranges = {c.get("name"): [float(v) for v in c.findtext("range").split()] for c in ET.parse(model).getroot().iter("Coordinate") if c.findtext("range")}
        for coord in anchor.columns:
            if coord in cfg.translation_coords or coord not in ranges:
                continue
            q = anchor.data[:, anchor.index[coord]]
            bounds = ranges[coord]
            result["joint_ranges"][coord] = {"configured_range_deg": [v*180/np.pi for v in bounds],
                                             "actual_deg": h.numerical.metrics(q*180/np.pi),
                                             "samples_below_XML_range": int(np.count_nonzero(q < bounds[0])),
                                             "samples_above_XML_range": int(np.count_nonzero(q > bounds[1])),
                                             "clinical_gate": False}
        result["source_IK"] = fingerprint(ik)
        result["measured_GRF"] = fingerprint(measured_path)
        result["analysis_status"] = "complete_primary_metrics" if result["windows"]["primary_all"]["coverage"] == "complete" else "partial_primary_metrics"
        if plots:
            result["canonical_plots"] = render_case(case_id, case, run, receipt, cfg, result, h, selection)
    if inventory(run) != before:
        raise AssertionError("Postprocessor changed finalized native run artifacts.")
    result["run_artifacts_unchanged"] = {"count": len(before), "passed": True, "files": before}
    exclusive_json(output, result)
    return output, result


def render_case(case_id, case, run, receipt, cfg, metrics, h, selection):
    """Prescribed adapter reuses canonical root functions; never --mlp/inference."""
    out = REPO / "plot/AB07_pylon_necessity_screening_2026_10_07" / case_id
    if out.exists():
        raise FileExistsError(out)
    plotter = h.render.load_plotter()
    missing = plotter.MissingReport()
    tables = {name: h.render.read_storage(run/"sim_outputs"/f"{cfg.output_prefix}_{name}.sto", plotter)
              for name in ("sea_torques", "sea_controls", "states", "kinematics_reference", "reserve_torques", "sea_states", "sea_diagnostics", "power")}
    actual = h.render.read_storage(run/"sim_outputs"/f"{cfg.output_prefix}_kinematics.sto", plotter)
    subject = case["subject"]
    healthy_root = REPO / f"models/{subject}_SEASEA_Threadmill/data/healthy"
    healthy_q = h.render.read_storage(healthy_root/f"{subject}_treadmill_01_01_Kinematics_q.sto", plotter)
    healthy_force = h.render.read_storage(healthy_root/f"{subject}_treadmill_01_01_Actuation_force.sto", plotter)
    raw = h.render.read_storage(path(cfg.kinematics_file), plotter)
    healthy = plotter.HealthyData(healthy_root, healthy_q, healthy_force, ["Healthy power uses derived dataset velocity, not actual native speed"])
    support = [actual, healthy_q, healthy_force, raw, *tables.values()]
    window = max(t.time[0] for t in support), min(t.time[-1] for t in support)
    names, cycles, excluded = h.render.selected_cycles(path(selection["primary_reference_csv"]["path"]), window)
    out.mkdir(parents=True)
    event_file = out / "reference_cycles_in_recorded_window.csv"
    with event_file.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=names); writer.writeheader(); writer.writerows(cycles)
    events = plotter.load_events(event_file, missing)
    recorded_reference = plotter.recorded_reference_angle_series
    def served(table, time, coord, side):
        if table is None or table.series([coord + "_q_ref"]) is None or table.time[0] > time[0] or table.time[-1] < time[-1]:
            missing.add(f"figure6 {side}: unavailable complete served reference"); return None
        return recorded_reference(table, time, coord, side)
    plotter.recorded_reference_angle_series = served
    outer = plotter.outer_loop_subtitle(cfg)
    inner_params = plotter.load_sea_params(cfg)
    inner = plotter.inner_loop_subtitle(inner_params, cfg)
    save = plotter.save_figure
    def labelled(fig, destination, filename):
        title = fig._suptitle.get_text() if fig._suptitle is not None else ""
        fig.suptitle(f"{subject} {case_id} prescribed | receipt {receipt['status']} | {metrics['analysis_status']}\n{title}", fontsize=12, y=.995)
        fig.tight_layout(rect=(0, 0, 1, .865))
        save(fig, destination, filename)
    plotter.save_figure = labelled
    plotter.plot_figure_1(tables, out, missing, outer, inner)
    plotter.plot_figure_2(tables, out, missing, outer, inner)
    plotter.plot_figure_3(tables, events, "left", healthy, out, missing, outer, inner)
    plotter.plot_figure_4(tables, events, "left", healthy, out, missing, outer, inner)
    plotter.plot_figure_5(tables, plotter.load_sea_f_opt(cfg, missing), out, missing, outer, inner)
    plotter.plot_figure_6(tables, None, out, missing, outer, inner)
    h.render.figure7(SimpleNamespace(subject=subject), plotter, {"actual": actual, "IK": raw, "healthy": healthy_q}, out, missing, outer, inner)
    missing.write(out/"missing_channels.txt")
    pngs = sorted(out.glob("*.png"))
    if len(pngs) != 7:
        raise ValueError("Seven canonical PNGs required.")
    manifest = {"status": "rendered_pending_visual_review", "case_id": case_id, "subject": subject,
                "receipt": fingerprint(run/"receipt.json"), "metrics_source_script": fingerprint(Path(__file__)),
                "root_plotter": fingerprint(REPO/"plot/plotter.py"), "prescribed_adapter": fingerprint(path(read_json(PROTOCOL)["source_bindings"]["render"]["path"])),
                "files": {p.name: fingerprint(p) for p in pngs}, "cycles_used": cycles,
                "cycles_excluded_for_incomplete_support": excluded, "timestamp_changes": False,
                "missing_channels": missing.items, "model_inner_loop_parameters": inner_params,
                "neural_inference": False, "visual_review": {p.name: "pending" for p in pngs}}
    exclusive_json(out/"plot_rendering_manifest.json", manifest)
    return fingerprint(out/"plot_rendering_manifest.json")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument("--case")
    choice.add_argument("--all", action="store_true")
    parser.add_argument("--plots", action="store_true")
    parser.add_argument("--aggregate-only", action="store_true", help="With --all, assemble already finalized per-case metrics without FK or plots")
    args = parser.parse_args()
    protocol, dynamic = read_json(PROTOCOL), read_json(AUDIT/"dynamic_protocol.json")
    check_manifest({v["path"]: v["sha256"] for v in protocol["source_bindings"].values()})
    protected_before = check_manifest(dynamic["protected_sources_before"])
    ids = list(dynamic["cases"]) if args.all else [args.case]
    if any(i not in dynamic["cases"] for i in ids):
        parser.error("Case is not part of the frozen prospective protocol.")
    if args.aggregate_only and not args.all:
        parser.error("--aggregate-only requires --all.")
    if not args.aggregate_only:
        # Preflight all selected receipts before importing OpenSim or doing any FK.
        for i in ids:
            receipt = read_json(AUDIT/"dynamics"/i/"run/receipt.json")
            if not receipt.get("finished_utc") or receipt.get("status") not in ("completed", "interrupted", "failed"):
                raise ValueError(f"{i} still active or not finalized.")
        h = helpers(protocol)
        for i in ids:
            p, result = analyze(i, dynamic, protocol, h, args.plots)
            print(json.dumps({"case": i, "status": result["analysis_status"], "path": relative(p)}), flush=True)
    protected_after = check_manifest(dynamic["protected_sources_before"])
    if args.all:
        aggregate = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
                     "status": "all_18_finalized_descriptive_metrics", "analysis_protocol": fingerprint(PROTOCOL),
                     "dynamic_protocol": fingerprint(AUDIT/"dynamic_protocol.json"), "cases": {},
                     "protected_before": protected_before, "protected_after": protected_after,
                     "clinical_gate_or_forcefit_used": False, "new_Claude_branch_read": False}
        for i in ids:
            p = HERE/i/"metrics.json"; data = read_json(p)
            current = fingerprint(AUDIT/"dynamics"/i/"run/receipt.json")
            if data["receipt"] != current or data["dynamic_protocol"]["sha256"] != sha(AUDIT/"dynamic_protocol.json"):
                raise ValueError("Per-case analysis is not bound to the current finalized receipt/protocol.")
            aggregate["cases"][i] = {"metrics": fingerprint(p), "subject": data["subject"], "case": data["case"],
                                     "runtime": data["runtime"], "analysis_status": data["analysis_status"],
                                     "mass_kg": data.get("mass_kg"), "BW_N": data.get("BW_N"),
                                     "primary_all": data["windows"].get("primary_all"),
                                     "prefix_comparison": data["prefix_comparison"]}
        exclusive_json(HERE/"aggregate_metrics.json", aggregate)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
