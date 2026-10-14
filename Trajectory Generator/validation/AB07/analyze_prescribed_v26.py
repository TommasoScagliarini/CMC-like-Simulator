"""Analyze saved AB07 V26 diagnostics; optional native replay sets poses only.

Completion is reported separately from contact/event qualification. No gains,
profiles, thresholds, checkpoints or runtime state are changed by this tool.
"""

from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable

import numpy as np


REPO = Path(__file__).resolve().parents[3]
DEFAULT_SESSION = "results/ab07_v26_left_prescribed_9p01s_2026_10_06"
DEFAULT_ORACLE = "results/AB07_prescribed_2026_10_06/sim_output_gait_events.csv"
MATCH_SOURCE = REPO / "validation/validate_online_grf_events.py"
CROSSING_SOURCE = REPO / "validation/validate_online_grf.py"
V26_SOURCE = REPO / "validation/validate_binary_phase_fsm_v20_development.py"
HOLDOUT_SOURCE = REPO / "validation/online_grf_acceptance_thresholds.json"


def path(raw: str | Path) -> Path:
    value = Path(str(raw).replace("\\", "/"))
    return value.resolve() if value.is_absolute() else (REPO / value).resolve()


def fingerprint(raw: str | Path) -> dict[str, str]:
    value = path(raw)
    digest = hashlib.sha256()
    with value.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"path": str(value), "sha256": digest.hexdigest()}


def existing_helper(source: Path, name: str):
    """Reuse only a pure established helper, without importing its native module."""
    tree = ast.parse(source.read_text(encoding="utf-8"))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == name)
    module = ast.Module(body=[function], type_ignores=[])
    namespace = {"np": np, "Any": Any, "Iterable": Iterable}
    exec(compile(module, str(source), "exec"), namespace)
    return namespace[name]


def _v26_tolerances() -> dict[str, float]:
    tree = ast.parse(V26_SOURCE.read_text(encoding="utf-8"))
    result = {}
    wanted = {"HS_TOLERANCE_S", "TO_TOLERANCE_S", "DELIVERED_HS_TOLERANCE_S", "DELIVERED_TO_TOLERANCE_S"}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in wanted:
                    result[target.id] = float(ast.literal_eval(node.value))
    if set(result) != wanted:
        raise ValueError("Cannot identify the frozen V26 inherited timing tolerances")
    return result


def stats(values: Iterable[float]) -> dict[str, float | int] | None:
    array = np.asarray(list(values), dtype=float)
    if array.size == 0:
        return None
    if not np.all(np.isfinite(array)):
        raise ValueError("Non-finite diagnostic values")
    return {
        "count": int(array.size), "min": float(np.min(array)),
        "p05": float(np.percentile(array, 5)), "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)), "max": float(np.max(array)),
        "mean": float(np.mean(array)), "rms": float(np.sqrt(np.mean(array * array))),
    }


def json_value(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(v) for v in value]
    if hasattr(value, "tolist"):
        return json_value(value.tolist())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def read_trace(source: Path, pending: bool) -> list[dict]:
    rows = []
    with source.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                if pending:
                    break
                raise
            rows.append({key: row[key] for key in (
                "time", "phase_fsm", "binary_phase_fsm", "binary_phase_sensor_samples",
                "binary_phase_active_adapter", "online_grf", "online_grf_detector",
                "online_events", "legacy_online_events", "observation", "reward_terms",
                "so_diagnostics", "end_reason",
            ) if key in row})
    if not rows:
        raise ValueError("No complete trace rows")
    times = np.asarray([row["time"] for row in rows], dtype=float)
    if not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        raise ValueError("Trace times must be finite and strictly increasing")
    return rows


def read_storage(source: Path) -> tuple[np.ndarray, list[str], np.ndarray, bool]:
    rows, names = [], None
    in_degrees = False
    with source.open(encoding="utf-8") as stream:
        for line in stream:
            text = line.strip()
            if text.lower().startswith("indegrees"):
                in_degrees = text.split("=", 1)[-1].strip().lower() == "yes"
            if text and text.split()[0].lower() == "time":
                names = text.split()[1:]
                break
        if names is None:
            raise ValueError(f"Storage header is absent: {source}")
        for line in stream:
            if line.strip():
                values = [float(value) for value in line.split()]
                if len(values) != len(names) + 1:
                    raise ValueError(f"Storage column count differs: {source}")
                rows.append(values)
    array = np.asarray(rows, dtype=float)
    if not rows or not np.all(np.isfinite(array)):
        raise ValueError(f"Storage values are absent or non-finite: {source}")
    return array[:, 0], names, array[:, 1:], in_degrees


def oracle_cycles(source: Path, start: float, end: float) -> tuple[list[dict], dict[str, list[float]]]:
    with source.open(encoding="utf-8") as stream:
        cycles = [row for row in csv.DictReader(stream) if row["side"] == "left"]
    nominal = {"heel_strike": [], "toe_off": []}
    complete = []
    for row in cycles:
        hs, nxt, contact = (float(row[key]) for key in ("cycle_start", "cycle_end", "contact_duration_s"))
        if start - 1e-8 <= hs <= end + 1e-8:
            nominal["heel_strike"].append(hs)
        if start - 1e-8 <= nxt <= end + 1e-8:
            nominal["heel_strike"].append(nxt)
        if start - 1e-8 <= hs + contact <= end + 1e-8:
            nominal["toe_off"].append(hs + contact)
        if hs >= start - 1e-8 and nxt <= end + 1e-8:
            complete.append({"hs_s": hs, "to_s": hs + contact, "next_hs_s": nxt,
                             "period_s": nxt - hs, "stance_s": contact,
                             "stance_fraction": contact / (nxt - hs)})
    for key in nominal:
        nominal[key] = sorted(set(nominal[key]))
    return complete, nominal


def ordered_delay(reference: list[float], predicted: list[float]) -> list[dict]:
    """Diagnostic nearest reference; never treat these as accepted event matches."""
    result = []
    for time in predicted:
        if reference:
            nearest = min(reference, key=lambda ref: abs(ref - time))
            result.append({"predicted_time_s": time, "nearest_reference_time_s": nearest,
                           "signed_delay_s": time - nearest, "accepted_match": False})
    return result


def analyze(session: Path, oracle: Path, *, pending: bool = False) -> dict:
    receipt_file, trace_file = session / "receipt.json", session / "trace.jsonl"
    receipt = json.loads(receipt_file.read_text(encoding="utf-8"))
    if not pending and receipt.get("status") not in {"completed", "interrupted", "failed"}:
        raise ValueError("Run is still pending; use --pending for a provisional report")
    rows = read_trace(trace_file, pending)
    start = float(receipt["requested_time_range"][0])
    end = float(rows[-1]["time"])
    times = np.asarray([row["time"] for row in rows])
    cycles, nominal = oracle_cycles(oracle, start, end)
    binary = [event for row in rows for event in row["binary_phase_fsm"]["events_this_step"]]
    transitions = [event for row in rows for event in row["phase_fsm"]["accepted_transitions_this_step"]]
    actor_cycles = [event for event in transitions if float(event.get("cycle_valid", -1)) == 1.0]
    source_tolerances = _v26_tolerances()
    holdout = json.loads(HOLDOUT_SOURCE.read_text(encoding="utf-8"))
    matcher = existing_helper(MATCH_SOURCE, "match_events")
    crossing = existing_helper(CROSSING_SOURCE, "_crossing_times")
    comparisons = {}
    for name, code, holdout_key in (
        ("heel_strike", "HS", "sensor_holdout_heel_strike_max_error_s"),
        ("toe_off", "TO", "sensor_holdout_toe_off_max_error_s"),
    ):
        selected = [event for event in binary if event["event"] == name]
        onset = [event["event_time_s"] for event in selected]
        confirmed = [event["confirmed_time_s"] for event in selected]
        delivered = [event["delivered_time_s"] for event in selected]
        comparisons[name] = {
            "onset_times_s": onset, "confirmed_times_s": confirmed, "delivered_times_s": delivered,
            "confirmed_v26_diagnostic_matching": matcher(nominal[name], confirmed, source_tolerances[f"{code}_TOLERANCE_S"]),
            "delivered_v26_diagnostic_matching": matcher(nominal[name], delivered, source_tolerances[f"DELIVERED_{code}_TOLERANCE_S"]),
            "onset_holdout_reference_matching": matcher(nominal[name], onset, float(holdout[holdout_key])),
            "nearest_onset_delays_diagnostic_only": ordered_delay(nominal[name], onset),
        }
    left = [row["online_grf"]["left"] for row in rows]
    normal = np.asarray([sample["normal_force"] for sample in left])
    fz = np.asarray([sample["force"][1] for sample in left])
    penetration = np.asarray([sample["penetration"] for sample in left])
    force_file = path(receipt["source_files"]["prescribed_grf"]["path"])
    ft, labels, force_data, _ = read_storage(force_file)
    with oracle.open(encoding="utf-8") as stream:
        row = next(row for row in csv.DictReader(stream) if row["side"] == "left")
    threshold = float(row["threshold_n"])
    ref_fz = np.interp(times, ft, force_data[:, labels.index(row["source_force"])])
    contact_ref, contact_pred = ref_fz > threshold, fz > threshold
    tp = int(np.count_nonzero(contact_ref & contact_pred))
    fp = int(np.count_nonzero(~contact_ref & contact_pred))
    fn = int(np.count_nonzero(contact_ref & ~contact_pred))
    force_events = {
        name: crossing(times, fz, threshold, rising=name == "heel_strike").tolist()
        for name in ("heel_strike", "toe_off")
    }
    samples = [sample for item in rows for sample in item["binary_phase_sensor_samples"]]
    phase = rows[-1]["phase_fsm"]
    physics = receipt.get("effective_simulator_config", {})
    cfg = receipt["env_kwargs"]
    so = [row["so_diagnostics"] for row in rows]
    integrate = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
    primary_impulse = float(integrate(fz, times))
    reference_impulse = float(integrate(ref_fz, times))
    result = {
        "schema_version": 1, "scope": "AB07 prescribed diagnostic with shared frozen V26 candidates; no learned policy",
        "status": "provisional" if pending else "review_required",
        "profiles_status": "contact/events not qualified for AB07 by this run",
        "completed_requested_duration": receipt.get("completed_requested_duration", False),
        "qualification_is_not_duration_completion": True,
        "source_files": {"analysis_script": fingerprint(Path(__file__)),
                         "receipt": fingerprint(receipt_file), "trace": fingerprint(trace_file),
                         "oracle_events": fingerprint(oracle), "prescribed_grf": fingerprint(force_file),
                         "matching_helper": fingerprint(MATCH_SOURCE), "crossing_helper": fingerprint(CROSSING_SOURCE),
                         "v26_tolerances": fingerprint(V26_SOURCE), "holdout_tolerances": fingerprint(HOLDOUT_SOURCE)},
        "analyzed_time_range_s": [start, end], "trace_rows_10ms": len(rows),
        "binary_samples_1ms": len(samples),
        "methodology": {
            "event_reference": "left complete-cycle CSV; unique HS starts/ends and TO=HS+contact_duration; excludes unanchored startup partial stance",
            "reference_contact_threshold_n": threshold,
            "reference_min_contact_duration_s": float(row["min_contact_duration_s"]),
            "reference_min_cycle_duration_s": float(row["min_cycle_duration_s"]),
            "event_matching": "established greedy one-to-one helper within declared tolerance; missing/extras retained",
            "v26_inherited_confirmed_and_delivered_tolerances_s": source_tolerances,
            "sensor_holdout_tolerances_s": {"heel_strike": holdout["sensor_holdout_heel_strike_max_error_s"],
                                             "toe_off": holdout["sensor_holdout_toe_off_max_error_s"]},
            "tolerance_scope": "V26 AB06 development/legacy analog holdout references only; neither is an adopted AB07 G10 acceptance gate",
            "primary_force_event_crossings": "raw interpolated 20 N crossings at saved 10 ms boundaries; separate from binary/actor events",
        },
        "nominal_complete_cycles": cycles, "nominal_events_s": nominal,
        "binary_event_comparison": comparisons, "binary_events": binary,
        "actor_accepted_transitions": transitions, "actor_valid_cycle_closures": actor_cycles,
        "actor_final_counters": {key: phase[key] for key in (
            "valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count",
            "resync_count", "hs_cancelled_count", "timeout_exceeded", "last_stance_fraction", "last_period_s",
        )},
        "actor_cycle_summary": {
            "nominal_complete_cycle_count": len(cycles), "internal_valid_cycle_count": phase["valid_cycle_count"],
            "nominal_stance_duration_s": stats(cycle["stance_s"] for cycle in cycles),
            "nominal_stance_fraction": stats(cycle["stance_fraction"] for cycle in cycles),
            "accepted_stance_duration_s": stats(event["segment_end_time_s"] - event["segment_start_time_s"]
                                                for event in transitions if event.get("closed_segment_type") == "stance"),
            "interpretation": "internal validity confirms the frozen FSM accepted an HS-TO-HS sequence; it does not validate agreement with measured contact",
        },
        "binary_boolean_contact": {
            key: {"sample_count": len(samples), "contact_fraction": float(np.mean([sample[key] for sample in samples]))}
            for key in ("left_heel_contact", "left_toe_contact")
        },
        "duty_cycles_in_saved_window": {
            "binary_heel_fraction_1ms": float(np.mean([sample["left_heel_contact"] for sample in samples])),
            "binary_toe_fraction_1ms": float(np.mean([sample["left_toe_contact"] for sample in samples])),
            "binary_any_point_fraction_1ms": float(np.mean([sample["left_heel_contact"] or sample["left_toe_contact"] for sample in samples])),
            "binary_functional_stance_fraction_10ms": float(np.mean([item["binary_phase_fsm"]["in_contact"] for item in rows])),
            "primary_contact_over_20n_fraction_10ms": float(np.mean(contact_pred)),
            "prescribed_contact_over_20n_fraction_10ms": float(np.mean(contact_ref)),
        },
        "primary_left_contact": {
            "normal_force_n": stats(normal), "vertical_force_n": stats(fz), "prescribed_vertical_force_n": stats(ref_fz),
            "vertical_error_n": stats(fz - ref_fz),
            "vertical_correlation": float(np.corrcoef(fz, ref_fz)[0, 1]),
            "relative_load_diagnostics": {
                "mean_force_ratio_to_oracle": float(np.mean(fz) / np.mean(ref_fz)),
                "peak_force_ratio_to_oracle": float(np.max(fz) / np.max(ref_fz)),
                "primary_vertical_impulse_ns": primary_impulse,
                "prescribed_vertical_impulse_ns": reference_impulse,
                "vertical_impulse_ratio_to_oracle": primary_impulse / reference_impulse,
                "time_basis_s": [float(times[0]), float(times[-1])],
                "integration_method": "trapezoid on saved 10 ms boundaries; diagnostic only",
            },
            "penetration_m": stats(penetration), "slip_speed_m_s": stats(sample["slip_speed"] for sample in left),
            "existing_soft_threshold_m": cfg["grf_penetration_penalty_threshold_m"],
            "existing_termination_threshold_m": cfg["grf_penetration_termination_m"],
            "existing_runner_hard_threshold_m": physics.get("online_grf_max_penetration_m"),
            "soft_exceedance_fraction": float(np.mean(penetration > cfg["grf_penetration_penalty_threshold_m"])),
            "termination_exceedance_fraction": float(np.mean(penetration > cfg["grf_penetration_termination_m"])),
            "contact_20n": {"true_positive_samples": tp, "false_positive_samples": fp, "false_negative_samples": fn,
                              "f1": 2 * tp / max(1, 2 * tp + fp + fn)},
            "threshold_crossings_s": force_events,
            "force_event_holdout_reference_matching": {
                name: matcher(nominal[name], force_events[name], float(holdout[key]))
                for name, key in (("heel_strike", "sensor_holdout_heel_strike_max_error_s"),
                                  ("toe_off", "sensor_holdout_toe_off_max_error_s"))
            },
        },
        "tracking_actual_vs_served_reference_deg": {
            name: stats(math.degrees(row["observation"][f"{name}_tracking_error"]) for row in rows)
            for name in ("pros_knee_angle", "pros_ankle_angle")
        },
        "so_diagnostics_at_saved_boundaries": {
            key: stats(item[key] for item in so if key in item)
            for key in ("tau_reserve_norm", "muscle_capable_reserve_norm", "unactuated_reserve_norm", "muscle_share", "muscle_capable_share")
        },
        "clearance_m": {"status": "not exported in the 1 ms Boolean trace; requires saved-pose replay"},
        "limitations": [
            "A single prescribed run does not qualify policy transfer or multi-start robustness.",
            "Primary force/penetration statistics here use 10 ms boundaries, not all integrator samples.",
            "Frozen AB06 ground plane, surface velocity and residual fits are unchanged hypotheses for AB07.",
            "Positive internal valid_cycle_count does not imply matching nominal contact timing or stance duration.",
        ],
    }
    return result


def clearance_replay(session: Path, *, plain_kinematics: Path) -> dict:
    """Evaluate exact frozen points on saved actual poses; no integration."""
    receipt = json.loads((session / "receipt.json").read_text(encoding="utf-8"))
    if receipt.get("status") not in {"completed", "interrupted", "failed"}:
        raise ValueError("Clearance replay requires a final exported run")
    import opensim

    opensim.Logger.setLevelString("error")
    plugin = path(receipt["effective_simulator_config"]["plugin_name"])
    if opensim.LoadOpenSimLibrary(str(plugin)) is False:
        raise RuntimeError(f"Unable to load SEA plugin: {plugin}")
    model_path = path(receipt["source_files"]["model"]["path"])
    profile_path = path(receipt["profile_association"]["binary_phase_detector_profile_file"]["path"])
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    origin = np.asarray(profile["ground"]["origin"], dtype=float)
    normal = np.asarray(profile["ground"]["normal"], dtype=float)
    start, end = receipt["requested_time_range"]
    result = {
        "method": "OpenSim initSystem + coordinate setValue + realizePosition + findStationLocationInGround; no integration",
        "units": "m", "ground_reference": profile["ground"],
        "contact_rule": profile["contact_rule"], "source_model": fingerprint(model_path),
        "source_binary_profile": fingerprint(profile_path), "streams": {},
    }
    for label, kin in (("plain_prescribed_actual", plain_kinematics),
                       ("active_v26_actual", session / "sim_outputs/sim_output_kinematics.sto")):
        times, names, data, degrees = read_storage(kin)
        keep = (times >= start - 1e-8) & (times <= end + 1e-8)
        times, data = times[keep], data[keep]
        model = opensim.Model(str(model_path))
        state = model.initSystem()
        coordinates = [(model.getCoordinateSet().get(name), index) for index, name in enumerate(names)]
        frames = [opensim.PhysicalFrame.safeDownCast(model.getComponent(point["frame"])) for point in profile["points"]]
        if any(frame is None for frame in frames):
            raise ValueError("A frozen detector point frame is not a PhysicalFrame")
        values = {point["name"]: [] for point in profile["points"]}
        for time, row in zip(times, data):
            state.setTime(float(time))
            for coord, index in coordinates:
                value = float(row[index])
                if degrees and int(coord.getMotionType()) == 1:
                    value = math.radians(value)
                coord.setValue(state, value, False)
            model.realizePosition(state)
            for point, frame in zip(profile["points"], frames):
                vec = frame.findStationLocationInGround(state, opensim.Vec3(*point["location"]))
                ground = np.asarray([vec.get(i) for i in range(3)], dtype=float)
                values[point["name"]].append(float((ground - origin) @ normal))
        result["streams"][label] = {
            "source_kinematics": fingerprint(kin), "in_degrees_source": degrees,
            "time_range_s": [float(times[0]), float(times[-1])], "pose_count": len(times),
            "points": {point["name"]: {"frame": point["frame"], "location_m": point["location"],
                                       "signed_clearance_m": stats(values[point["name"]]),
                                       "contact_fraction_le_zero": float(np.mean(np.asarray(values[point["name"]]) <= 0))}
                       for point in profile["points"]},
        }
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", default=DEFAULT_SESSION)
    parser.add_argument("--oracle-events", default=DEFAULT_ORACLE)
    parser.add_argument("--pending", action="store_true", help="Provisional analysis printed to stdout only.")
    parser.add_argument("--with-clearance", action="store_true", help="Native saved-pose replay after final output export.")
    parser.add_argument("--plain-kinematics", default="results/AB07_prescribed_2026_10_06/sim_output_kinematics.sto")
    parser.add_argument("--out", default="", help="Final new receipt; default session/qualification_analysis.json.")
    args = parser.parse_args(argv)
    session = path(args.session)
    if args.pending and (args.with_clearance or args.out):
        parser.error("--pending cannot write final receipts or replay unfinished outputs")
    result = analyze(session, path(args.oracle_events), pending=args.pending)
    if args.with_clearance:
        result["clearance_m"] = clearance_replay(session, plain_kinematics=path(args.plain_kinematics))
    if args.pending:
        print(json.dumps(json_value(result), indent=2, allow_nan=False))
    else:
        output = path(args.out) if args.out else session / "qualification_analysis.json"
        with output.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(json_value(result), indent=2, allow_nan=False) + "\n")
        print(f"Diagnostic qualification receipt: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
