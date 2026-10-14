"""Pure-JSON summary of three finalized, independently bound standing cases.

No FK, environment, solver, raw force integration or event redetection occurs.
The numerical-only packet has no local paths; provenance is a separate receipt.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
AUDIT = HERE.parent
ROOT = AUDIT.parents[1]
IDS = ("STANDING_FOURROI_AB07", "PYLON400_REFERENCE", "STANDING_AB07")


def read(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def fp(path):
    p = Path(path).resolve()
    return {"path": p.relative_to(ROOT).as_posix(), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}


def phase_channel(channel):
    out = {"coverage": channel["status"]}
    if channel["comparison"] is None:
        out["events"] = None
        return out
    events = {}
    for kind, entry in channel["comparison"].items():
        clocks = {}
        for clock, row in entry["channels"].items():
            match = row["matching"]
            guards = row["fully_observed_reference_matching_guard_s"]
            guarded_pairs = [p for p in match["pairs"] if p["reference_time_s"] in guards]
            clocks[clock] = {
                "reference_count": match["reference_count"], "observed_candidate_count": match["predicted_count"],
                "matched_count": match["matched_count"], "extra_count": match["false_positives"],
                "raw_matcher_unmatched_reference_count": match["false_negatives"],
                "unmatched_fully_observed_count": len(row["unmatched_reference_fully_observed_matching_guard_s"]),
                "unmatched_censored_or_unknown_count": len(row["unmatched_reference_censored_or_unknown_s"]),
                "reference_matching_guard_complete_count": len(guards),
                "reference_matching_guard_incomplete_count": match["reference_count"] - len(guards),
                "matched_guard_complete_count": len(guarded_pairs),
                "matched_guard_incomplete_observations_count": match["matched_count"] - len(guarded_pairs),
                "matched_onset_or_clock_bias_ms": match["timing_bias_s"] * 1000 if match["timing_bias_s"] is not None else None,
                "matched_onset_or_clock_MAE_ms": match["timing_mae_s"] * 1000 if match["timing_mae_s"] is not None else None,
                "matched_max_abs_error_ms": match["timing_max_abs_s"] * 1000 if match["timing_max_abs_s"] is not None else None,
                "tolerance_ms": row["tolerance_s"] * 1000,
                "observed_event_times_s": row["observed_times_s"],
                "unmatched_observed_event_times_s": match["unmatched_predicted"],
                "unmatched_reference_fully_observed_s": row["unmatched_reference_fully_observed_matching_guard_s"],
                "unmatched_reference_censored_or_unknown_s": row["unmatched_reference_censored_or_unknown_s"],
            }
        events[kind] = clocks
    out["events"] = events
    return out


def primary_stats(window):
    if not window or "forces" not in window:
        return None
    forces, reserve = window["forces"], window["reserves"]["per_coordinate"]["pelvis_ty"]
    duration = window["observed_duration_s"]
    balance = window["vertical_balance"]
    need = balance["implied_left_required_minus_reserve_Ns"] / duration if balance["available"] else None
    biological = {key: value for key, value in window["reserves"]["per_coordinate"].items()
                  if key not in {"pros_knee_angle", "pros_ankle_angle"}}
    all_reserve = {key: {"force_or_torque_unit": value["unit"],
                         "mean": value["force_or_torque"]["mean"], "RMS": value["force_or_torque"]["rms"],
                         "impulse_unit": value["impulse_unit"], "impulse": value["impulse"],
                         "work_J": value["work_J"], "work_J_per_kg": value["work_J_per_kg"]}
                   for key, value in biological.items()}
    top_by_unit = {}
    for unit in ("N", "Nm"):
        names = sorted((key for key, value in all_reserve.items() if value["force_or_torque_unit"] == unit),
                       key=lambda key: all_reserve[key]["RMS"], reverse=True)[:5]
        top_by_unit[unit] = {key: all_reserve[key] for key in names}
    tracking = window["tracking"]["coordinates"]
    return {"observed_range_s": window["observed_intersection_range_s"], "observed_duration_s": duration,
            "left_actual_mean_N": forces["left_online"]["mean_N"], "left_actual_mean_BW": forces["left_online"]["mean_BW"],
            "left_actual_impulse_Ns": forces["left_online"]["impulse_Ns"],
            "left_momentum_required_mean_N": need,
            "left_measured_mean_N": forces["left_measured"]["mean_N"],
            "right_measured_mean_N": forces["right_measured"]["mean_N"],
            "pelvis_vertical_reserve": {"mean_N": reserve["force_or_torque"]["mean"],
                                        "RMS_N": reserve["force_or_torque"]["rms"],
                                        "RMS_BW": reserve["force_or_torque"]["rms_BW"],
                                        "impulse_Ns": reserve["impulse"], "work_J": reserve["work_J"],
                                        "work_J_per_kg": reserve["work_J_per_kg"]},
            "left_actual_penetration_mean_mm": window["penetration"]["left"]["mean"] * 1000,
            "left_actual_penetration_max_mm": window["penetration"]["left"]["max"] * 1000,
            "vertical_balance_closure_residual_Ns": balance.get("closure_residual_Ns"),
            "all_biological_reserves": all_reserve, "top_biological_reserves_by_RMS_separate_units": top_by_unit,
            "sum_per_DOF_absolute_biological_reserve_work_J": sum(v["work_J"]["absolute"] for v in biological.values()),
            "sum_per_DOF_signed_biological_reserve_work_J": sum(v["work_J"]["signed"] for v in biological.values()),
            "actual_minus_served_position_RMS": {key: {"value": value["actual_minus_served"]["position"]["rms"],
                                                       "unit": value["actual_minus_served"]["position"]["unit"]}
                                                   for key, value in tracking.items()},
            "prosthetic_knee_RMS_deg": tracking["pros_knee_angle"]["actual_minus_served"]["position"]["rms"],
            "prosthetic_ankle_RMS_deg": tracking["pros_ankle_angle"]["actual_minus_served"]["position"]["rms"]}


def main():
    output, receipt_output = HERE / "lean_numeric_summary.json", HERE / "lean_summary_receipt.json"
    if output.exists() or receipt_output.exists():
        raise FileExistsError("Preserve finalized lean summary")
    groups = {}
    inputs = {"assembler": fp(Path(__file__))}
    for group, prefix in (("base", ""), ("four_roi", "full_support_")):
        dynamic_path = AUDIT / (prefix + "dynamic_protocol.json")
        phase_protocol = HERE / (prefix + "phase_protocol.json")
        analysis_protocol = HERE / (prefix + "analysis_protocol.json")
        dynamic, protocol = read(dynamic_path), read(phase_protocol)
        expected = {"STANDING_FOURROI_AB07"} if prefix else {"STANDING_AB07", "PYLON400_REFERENCE"}
        if set(dynamic["cases"]) != expected or set(protocol["cases"]) != expected:
            raise ValueError("Preserved base plus prospectively frozen single FOURROI case required")
        bindings = {"dynamic": fp(dynamic_path), "phase_protocol": fp(phase_protocol), "analysis_protocol": fp(analysis_protocol)}
        groups[group] = (dynamic, protocol, bindings, prefix)
        inputs.update({group + "_" + key: value for key, value in bindings.items()})
    diagnostic_path = HERE / "finalized_three_case_saved_data_diagnostics.json"
    diagnostic = read(diagnostic_path)
    if not diagnostic["sources_unchanged"] or set(diagnostic["secondary_common_observed_prefix"]["cases"]) != set(IDS):
        raise ValueError("Finalized same-support saved-data comparison required")
    inputs["three_case_saved_diagnostic"] = fp(diagnostic_path)
    cases = {}
    for name in IDS:
        group = "four_roi" if name == "STANDING_FOURROI_AB07" else "base"
        dynamic, protocol, bindings, prefix = groups[group]
        metric_path, phase_path = HERE / name / "metrics.json", HERE / "phase" / name / "phase_metrics.json"
        runtime_path = AUDIT / ("full_support_dynamics" if prefix else "dynamics") / name / "run/receipt.json"
        metric, phase, runtime = read(metric_path), read(phase_path), read(runtime_path)
        if metric["case"] != dynamic["cases"][name] or phase["case"] != dynamic["cases"][name]:
            raise ValueError("Case differs from frozen selection")
        if metric["receipt"]["sha256"] != fp(runtime_path)["sha256"] or phase["receipt"]["sha256"] != fp(runtime_path)["sha256"]:
            raise ValueError("Summary sources are not bound to current finalized receipt")
        if metric["dynamic_protocol"]["sha256"] != bindings["dynamic"]["sha256"] or metric["analysis_protocol"]["sha256"] != bindings["analysis_protocol"]["sha256"] or phase["phase_protocol"]["sha256"] != bindings["phase_protocol"]["sha256"]:
            raise ValueError("Final metric/phase protocol binding differs")
        if not metric["run_artifacts_unchanged"]["passed"] or not phase["run_artifacts_unchanged"]["passed"]:
            raise ValueError("Native integrity guard failed")
        inputs[name + "_metrics"], inputs[name + "_phase"], inputs[name + "_receipt"] = fp(metric_path), fp(phase_path), fp(runtime_path)
        w = metric["windows"].get("primary_all")
        complete = bool(w and w["coverage"] == "complete")
        stats = primary_stats(w)
        counters = phase["observed_full_episode_journal_secondary"]["last_counters"]
        pooled = phase["swing_impulse_fraction"].get("pooled")
        physical_start = runtime["actual_episode_start_s"]
        physical_end = metric["runtime"].get("final_clocks_s", {}).get("state")
        recorded_window = metric["windows"].get("all_recorded", {})
        cases[name] = {"comparison_role": "secondary_original_two_ROI" if name == "STANDING_AB07" else "main",
                       "runtime_status": metric["runtime"]["status"], "runtime": metric["runtime"],
                       "primary_coverage": w["coverage"] if w else "unavailable", "primary_requested_range_s": w.get("requested_range_s") if w else None,
                       "mass_kg": metric.get("mass_kg"), "BW_N": metric.get("BW_N"),
                       "physical_episode_duration_s": physical_end - physical_start if physical_end is not None else None,
                       "recorded_native_range_s": recorded_window.get("observed_intersection_range_s"),
                       "recorded_native_duration_s": recorded_window.get("observed_duration_s"),
                       "distal_mount_length_mm": dynamic["cases"][name]["length_mm"],
                       "static_ankle_reference_offset_deg": dynamic["cases"][name]["static_ankle_offset_deg"],
                       "primary_full_three_cycle_statistics": stats if complete else None,
                       "observed_partial_primary_statistics": stats if not complete else None,
                       "nominal_swing_impulse_fraction": pooled["swing_fraction_of_primary_impulse"] if pooled else None,
                       "primary_phase": {label: phase_channel(channel) for label, channel in phase["primary_channels"].items()},
                       "full_episode_FSM_counters": counters,
                       "full_episode_revoked_HS_count": len(phase["observed_full_episode_journal_secondary"]["heel_strike_cancellations"]),
                       "SEA_all_recorded": metric.get("SEA_all_recorded"),
                       "partial_startup_only_separately_observed_statistics": primary_stats(recorded_window) if not complete and w and w["coverage"] == "unavailable" else None,
                       "legacy_recruitment_mixed_unit_diagnostics": {key: value for key, value in metric.get("recruitment_primary_native_samples", {}).items() if key != "equilibrium_failure_interpretation"},
                       "fresh400_exact_numeric_table_reproduction": metric["prefix_comparison"].get("all_available_tables_exact_numeric_prefix") if name == "PYLON400_REFERENCE" else None}
    # Runtime/error diagnostic strings may contain local paths; keep packet
    # compact by retaining structured clock/outcome fields and error type only.
    for value in cases.values():
        runtime = value["runtime"]
        value["runtime"] = {key: runtime.get(key) for key in ("status", "steps", "end_reason", "completed_requested_duration", "actual_episode_range_s", "wall_time_s", "final_clocks_s", "clocks_at_requested_end")}
        value["runtime"]["configured_episode_range_s"] = value["runtime"].pop("actual_episode_range_s")
        value["runtime"]["failure_type"] = runtime.get("failure", {}).get("error_type") if isinstance(runtime.get("failure"), dict) else None
    geometry = {}
    for name, fit_file in (("STANDING_AB07", AUDIT / "geometry/fit_geometry_receipt_completed.json"),
                           ("STANDING_FOURROI_AB07", AUDIT / "geometry/four_roi_support/fit_receipt.json")):
        fit = read(fit_file)
        inputs[name + "_geometry_receipt"] = fp(fit_file)
        mounts, g = fit["mounts"], fit["geometry"]
        geometry[name] = {"strict_status": fit.get("strict_verdict", fit["status"]),
                          "mounts": mounts, "foot_rotation_ground": g["foot_rotation_ground"],
                          "foot_origin_XYZ_m": g.get("foot_origin_XYZ_m", g.get("fitted_ankle_XYZ_m")),
                          "healthy_reference_deg_m": fit["reference_coordinate_maps"]["healthy_deg_m"],
                          "prepared_reference_deg_m": fit["reference_coordinate_maps"]["prepared_deg_m"],
                          "ROI_min_Y_m": g.get("ROI_min_Y_m", {"heel": g.get("heel_min_Y_m"), "forefoot": g.get("forefoot_min_Y_m")}),
                          "whole_STL_min_Y_m": g["whole_STL_min_Y_m"],
                          "actual_contact_count": g.get("actual_contact_count", g.get("support_vertices_at_declared_tolerance")),
                          "native_parity": fit.get("native_parity")}
    static_contact_path = AUDIT / "geometry/static_force_sphere_comparison.json"
    static_contact = read(static_contact_path)
    inputs["static_contact_geometry"] = fp(static_contact_path)
    if set(static_contact["cases"]) != set(IDS) or static_contact["force_evaluation_or_profile_change"]:
        raise ValueError("Pure static geometry of all three saved poses required")
    for index, item in enumerate(static_contact["sources"]):
        current = fp(ROOT / item["path"])
        if current["sha256"] != item["sha256"]: raise ValueError("Static sphere source binding differs")
        inputs["static_contact_source_" + str(index)] = current
    static_spheres = {"ground_Y_mm": static_contact["ground_Y_m"] * 1000,
                      "runtime_penetration_guard_mm": static_contact["runtime_penetration_guard_mm"],
                      "cases": {name: {"positive_sphere_count": row["positive_sphere_count"],
                                       "max_geometric_penetration_mm": row["maximum_geometric_penetration_mm"],
                                       "mean_geometric_penetration_mm": row["mean_geometric_penetration_mm"],
                                       "eight_clipped_geometric_penetrations_mm": [v["geometric_penetration_m"] * 1000 for v in row["spheres"]],
                                       "eight_signed_geometric_depths_mm": [v["raw_radius_minus_plane_distance_m"] * 1000 for v in row["spheres"]]}
                                for name, row in static_contact["cases"].items()},
                      "force_evaluation_performed": False, "walking_cause_identified": False}
    reproducibility = diagnostic["fresh400_reproducibility"]
    packet = {"cases": cases, "main_comparison_cases": ["STANDING_FOURROI_AB07", "PYLON400_REFERENCE"],
              "secondary_case": "STANDING_AB07", "standing_geometry": geometry, "static_force_sphere_geometry": static_spheres,
              "secondary_common_observed_prefix": diagnostic["secondary_common_observed_prefix"],
              "fresh400_reproducibility": {"all_17_native_files_byte_equal": reproducibility["all_17_native_files_byte_equal"],
                                           "trace": reproducibility["trace"], "event_CSVs": reproducibility["event_CSVs"]},
              "units_and_scope": "Actual forward reserve-assisted diagnostics, own coherent preparation; every force/phase statistic uses the same three frozen measured nominal cycles or an explicitly labelled observed partial prefix.",
              "support_scope": "FOURROI matches kneeY, all four outer heel/forefoot region minima within10micrometers, whole STL nonpenetration and a noncollinear3vertex support polygon in encoded benchmarkY0. Base2ROI gives2medialcontacts; neither is distributed pressure or clinical proof.",
              "work_scope": "Sum of per-DOF absolute reserve work, distinct from absolute work of summed generalized power and from physiological/metabolic effort.",
              "phase_scope": "Binary/retained actor/raw20N force separate; original matcher counts plus complete matching guards and censored closingHS. No new detector or acceptance criterion.",
              "clinical_or_hardware_necessity_identified": False}
    # Prevent accidental path-bearing objects in the numerical-only packet.
    def no_paths(value):
        if isinstance(value, dict):
            if "path" in value or "source_files" in value:
                raise ValueError("Unexpected local source object in numerical packet")
            for item in value.values(): no_paths(item)
        elif isinstance(value, list):
            for item in value: no_paths(item)
    no_paths(packet)
    if any(token in json.dumps(packet) for token in (str(ROOT), "validation/", "Trajectory Generator/", "\\Users\\")):
        raise ValueError("Unexpected local path in numerical-only packet")
    with output.open("x") as stream:
        json.dump(packet, stream, indent=2, allow_nan=False); stream.write("\n")
    if any(fp(ROOT / item["path"]) != item for item in inputs.values()):
        raise AssertionError("Finalized lean summary inputs changed during consumption")
    receipt = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "completed",
               "inputs": inputs, "inputs_unchanged": True, "output": fp(output),
               "new_native_FK_or_IK_calls": 0, "raw_force_or_event_redetection": False,
               "new_Claude_branch_read": False}
    with receipt_output.open("x") as stream:
        json.dump(receipt, stream, indent=2, allow_nan=False); stream.write("\n")
    print(json.dumps({"packet": fp(output), "receipt": fp(receipt_output)}), flush=True)


if __name__ == "__main__":
    main()
