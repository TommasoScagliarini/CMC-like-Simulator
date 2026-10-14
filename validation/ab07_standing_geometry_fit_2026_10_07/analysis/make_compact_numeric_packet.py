"""Project finalized numerical JSON to a compact packet, without local sources."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def fp(path):
    return {"path": path.relative_to(ROOT).as_posix(), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def select(row, keys):
    return {key: row[key] for key in keys if key in row}


def stats(row):
    if row is None:
        return None
    out = select(row, ("observed_range_s", "observed_duration_s", "left_actual_mean_N", "left_actual_mean_BW",
                       "left_actual_impulse_Ns", "left_momentum_required_mean_N", "left_measured_mean_N", "right_measured_mean_N",
                       "pelvis_vertical_reserve", "left_actual_penetration_mean_mm", "left_actual_penetration_max_mm",
                       "vertical_balance_closure_residual_Ns", "sum_per_DOF_absolute_biological_reserve_work_J",
                       "sum_per_DOF_signed_biological_reserve_work_J", "actual_minus_served_position_RMS"))
    out["biological_reserves"] = {key: select(value, ("force_or_torque_unit", "mean", "RMS", "impulse_unit", "impulse", "work_J"))
                                  for key, value in row["all_biological_reserves"].items()}
    return out


def main():
    source = HERE / "lean_numeric_summary.json"
    receipt_source = HERE / "lean_summary_receipt.json"
    packet_file, receipt_file = HERE / "compact_numeric_packet.json", HERE / "compact_numeric_packet_receipt.json"
    if packet_file.exists() or receipt_file.exists():
        raise FileExistsError("Preserve finalized packet")
    before = {"comprehensive_summary": fp(source), "source_receipt": fp(receipt_source), "helper": fp(Path(__file__))}
    receipt = json.loads(receipt_source.read_text())
    if not receipt["inputs_unchanged"] or receipt["output"] != before["comprehensive_summary"]:
        raise AssertionError("Final summary source binding differs")
    data = json.loads(source.read_text())
    cases = {}
    for name, row in data["cases"].items():
        case = select(row, ("comparison_role", "runtime_status", "runtime", "primary_coverage", "primary_requested_range_s",
                            "mass_kg", "BW_N", "distal_mount_length_mm", "static_ankle_reference_offset_deg",
                            "physical_episode_duration_s", "recorded_native_range_s", "recorded_native_duration_s",
                            "nominal_swing_impulse_fraction", "full_episode_FSM_counters", "full_episode_revoked_HS_count"))
        case["primary_three_cycle_statistics"] = stats(row["primary_full_three_cycle_statistics"])
        case["secondary_startup_statistics_own_support"] = stats(row["partial_startup_only_separately_observed_statistics"])
        case["primary_phase"] = {}
        for label, channel in row["primary_phase"].items():
            phase = {"coverage": channel["coverage"], "events": channel["events"]}
            case["primary_phase"][label] = phase
        case["SEA_all_recorded_native_sample_diagnostics"] = {}
        for actuator, value in row["SEA_all_recorded"].items():
            sea = select(value, ("coordinate", "all_identity_checks_pass", "control_bound_violations",
                                 "motor_torque_bound_violations", "normalized_control_saturation", "motor_torque_clamp",
                                 "inner_integral_torque_clamp", "tau_ref_minus_spring_Nm", "tau_spring_Nm", "tau_motor_Nm"))
            evidence = value["command_limit_evidence"]
            sea["raw_command_clipping"] = select(evidence, ("exactly_at_plus_or_minus_one", "unclipped_command_above_unit_bound",
                                                             "first_command_clipping_sample_s", "last_command_clipping_sample_s", "cascade_i_clamped"))
            case["SEA_all_recorded_native_sample_diagnostics"][actuator] = sea
        cases[name] = case
    packet = {key: data[key] for key in ("main_comparison_cases", "secondary_case", "standing_geometry",
                                        "static_force_sphere_geometry", "secondary_common_observed_prefix",
                                        "fresh400_reproducibility", "clinical_or_hardware_necessity_identified")}
    packet["cases"] = cases
    packet["units"] = "Translation reserve=N/impulse Ns; rotation reserve=Nm/impulse Nm*s; work=recorded force/torque times actual saved coordinate speed in m/s or rad/s, integrated J. Position errors=deg or m as labelled. All negative integrals retain sign."
    packet["coverage"] = "Primary=three frozen measured nominal cycles, complete only for400. Both standing cases stopped before primary; their phase is unavailable, with no future-event misses. Common159ms is startup secondary, no COM-required load claim."
    packet["geometry_scope"] = "Saved static representative pose, benchmark groundY0, fixed mesh. Four ROI within10micrometers and3noncollinearcontactvertices are geometric witnesses, not pressure or clinical validation. Sphere depths are static geometry only, not evaluated forces or walking-cause proof."
    packet["SEA_scope"] = "Near-unit normalized commands, clipping of raw commands and motor500Nm clamp are distinct native-sample diagnostics. Samplecount-times-dt is separate from recorder support."
    # No paths, fingerprints, source line citations or code text belong here.
    payload = json.dumps(packet, separators=(",", ":"), allow_nan=False) + "\n"
    if any(token in payload for token in (str(ROOT), '"path"', '"sha256"', "validation/", "Trajectory Generator/", "prosthesis_controller.py")):
        raise ValueError("Unexpected local source content in numerical packet")
    after = {key: fp(ROOT / value["path"]) for key, value in before.items()}
    if before != after:
        raise AssertionError("Summary changed during projection")
    with packet_file.open("x") as stream:
        stream.write(payload)
    result = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "completed",
              "sources_before": before, "sources_after": after, "sources_unchanged": True,
              "packet": fp(packet_file), "packet_bytes": len(payload.encode()), "new_numerical_calculation": False,
              "projection_only": True, "new_Claude_branch_read": False}
    with receipt_file.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    print(json.dumps(result["packet"] | {"packet_bytes": result["packet_bytes"]}), flush=True)


if __name__ == "__main__":
    main()
