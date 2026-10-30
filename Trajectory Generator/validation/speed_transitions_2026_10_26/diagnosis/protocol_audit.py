"""Read-only audit of the synthetic stop/restart protocol and saved rollout.

This is a trace analysis, not a new dynamical simulation or a proof of static
equilibrium. Run with envCMC-rllib from any working directory.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

HERE = Path(__file__).resolve().parent
EXPERIMENT = HERE.parent
ROOT = EXPERIMENT.parents[2]
CASE = EXPERIMENT / "stop_restart_aligned"
sys.path.insert(0, str(ROOT / "Trajectory Generator/baseline_MLP"))
import _bootstrap  # noqa: E402

_bootstrap.ensure_sim_paths()
import win_runtime  # noqa: E402,F401
import numpy as np  # noqa: E402


def storage(path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip().lower() == "endheader":
                break
        columns = next(stream).split()
        data = np.loadtxt(stream, ndmin=2)
    return {name: data[:, i] for i, name in enumerate(columns)}


def stats(values):
    a = np.asarray(values, dtype=float)
    return {"min": float(a.min()), "max": float(a.max()),
            "mean": float(a.mean()), "range": float(np.ptp(a)),
            "rms": float(np.sqrt(np.mean(a*a)))}


def main():
    protocol = json.loads((CASE / "protocol.json").read_text(encoding="utf-8"))
    summary = json.loads((CASE / "rollout_summary.json").read_text(encoding="utf-8"))
    trace = json.loads((CASE / "rollout_policy_trace.json").read_text(encoding="utf-8"))
    origin = protocol["start_s"]
    _, start, end, _ = next(row for row in protocol["stages"] if row[0] == "hold")
    start, end = origin + start, origin + end
    settled_start = start + .5
    held = [r for r in trace if start <= r["time"] <= end]
    settled = [r for r in trace if settled_start <= r["time"] <= end]
    table = {name: storage(CASE / "sim_outputs" / ("rollout_episode_"+name+".sto"))
             for name in ("states", "kinematics_reference", "online_grf", "reserve_torques")}
    prescribed = storage(CASE / "prescribed_grf.mot")
    p_mask = (prescribed["time"] >= start) & (prescribed["time"] <= end)
    frozen = {k: stats(v[p_mask]) for k, v in prescribed.items() if k != "time"}
    assert max(v["range"] for v in frozen.values()) < 1e-9

    setup = ET.parse(CASE / "setup.xml")
    model_path = Path(setup.find(".//model_file").text)
    model = ET.parse(model_path)
    mass = sum(float(e.text) for e in model.findall(".//BodySet/objects/Body/mass"))
    body_weight = mass*9.80665
    states = table["states"]
    mask = (states["time"] >= settled_start) & (states["time"] <= end)
    bio = [name[:-2] for name in states if name.endswith("_q") and not name.startswith("pros_")]
    ref = table["kinematics_reference"]
    ref_mask = (ref["time"] >= settled_start) & (ref["time"] <= end)
    biological = {}
    for name in bio:
        biological[name] = {
            "units": "m, m/s" if name in ("pelvis_tx", "pelvis_ty", "pelvis_tz") else "rad, rad/s",
            "actual_position": stats(states[name+"_q"][mask]),
            "actual_velocity": stats(states[name+"_qdot"][mask]),
            "reference_position": stats(ref[name+"_q_ref"][ref_mask]),
            "reference_velocity": stats(ref[name+"_qdot_ref"][ref_mask]),
        }
    assert max(v["reference_velocity"]["max"] for v in biological.values()) == 0.
    assert min(v["reference_velocity"]["min"] for v in biological.values()) == 0.

    forces = table["online_grf"]
    f_mask = (forces["time"] >= settled_start) & (forces["time"] <= end)
    loads = {k: stats(v[f_mask]) for k, v in forces.items() if k.startswith("left_")}
    right = {axis: frozen["ground_force2_v"+axis]["mean"] for axis in "xyz"}
    # These are measured external forces, not an inferred acceleration/balance.
    total = {axis: stats(forces["left_force_"+axis][f_mask] + right[axis]) for axis in "xyz"}
    total["vertical_over_bw"] = stats((forces["left_force_y"][f_mask]+right["y"])/body_weight)
    reserves = table["reserve_torques"]
    r_mask = (reserves["time"] >= settled_start) & (reserves["time"] <= end)
    reserve_stats = {k: stats(v[r_mask]) for k,v in reserves.items() if k != "time"}

    nearest = min(trace, key=lambda row: abs(row["time"]-start))
    observed_contacts = []
    for r in held:
        left = r["detector_sensors"]
        observed_contacts.append(bool(left["left_heel"]["in_contact"]) or bool(left["left_toe"]["in_contact"]))
    phase_fields = [name for name in held[0]["actor_observation_before"] if "phase" in name or "duration" in name]
    phase_stats = {name: stats([r["actor_observation_before"][name] for r in held]) for name in phase_fields}
    plateau_targets = {joint: {"position": stats([r["imitation_target_q"][joint] for r in held]),
                                "velocity": stats([r["imitation_target_qdot"][joint] for r in held])}
                       for joint in ("pros_knee_angle", "pros_ankle_angle")}
    schedule = np.loadtxt(CASE / "schedule.csv", delimiter=",", skiprows=1)
    ramp = (schedule[:,0] >= origin+protocol["stages"][1][1]) & (schedule[:,0] <= start)
    surface = np.asarray(protocol["native_checks"]["baseline_surface_vectors"][0], dtype=float)
    # Same physical state is not replayed here: surface validity comes from
    # native checks plus current C++ getter access in calcContact.
    result = {
        "scope": "saved-rollout and protocol audit; no causal ablation or equilibrium solve",
        "hold_absolute_s": [start,end], "settled_window_absolute_s": [settled_start,end],
        "hold_source_time_s": protocol["hold_source_s"],
        "model_mass_kg": mass, "model_body_weight_n": body_weight,
        "source_left_and_right_force_frozen": frozen,
        "right_horizontal_force_n": float(np.hypot(right["x"], right["z"])),
        "right_vertical_force_bw": right["y"]/body_weight,
        "biological_motion_settled": biological,
        "left_online_force_settled": loads,
        "physical_online_left_plus_prescribed_right_force_settled": total,
        "reserve_per_coordinate_settled": reserve_stats,
        "near_hold_onset": {
            "elapsed_s": nearest["time"]-origin,
            "actor_state": nearest["phase_fsm"]["state_name"],
            "sensors": nearest["detector_sensors"],
            "prosthetic_state": nearest["prosthetic_state"],
        },
        "left_detector_contact_fraction_hold_policy_boundaries": float(np.mean(observed_contacts)),
        "actor_phase_fields_during_hold": phase_stats,
        "gait_clock_enable": summary["gait_clock_enable"],
        "imitation_target_during_hold": plateau_targets,
        "surface_baseline_vector_m_s": surface.tolist(),
        "surface_baseline_norm_m_s": float(np.linalg.norm(surface)),
        "ramp_duration_s": .5,
        "max_rate_derivative_s_inverse": float(np.max(np.abs(schedule[ramp,3]))),
        "max_surface_acceleration_norm_m_s2": float(np.max(np.abs(schedule[ramp,3]))*np.linalg.norm(surface)),
        "native_checks": protocol["native_checks"],
        "sources_sha256": {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in [Path(__file__), CASE/"protocol.json", CASE/"rollout_policy_trace.json",
                                     CASE/"prescribed_grf.mot", ROOT/"online_GRF/sources/OnlineGRFSphereHalfSpaceForce.cpp"]},
    }
    destination=HERE/"protocol_audit.json"
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps({"output":str(destination.relative_to(ROOT)), "body_weight_n":body_weight,
                      "right_frozen_force_n":right,"right_horizontal_force_n":result["right_horizontal_force_n"],
                      "left_contact_fraction":result["left_detector_contact_fraction_hold_policy_boundaries"],
                      "physical_total_vertical_bw":total["vertical_over_bw"],
                      "gait_clock_enable":result["gait_clock_enable"],
                      "max_surface_acceleration_m_s2":result["max_surface_acceleration_norm_m_s2"]},indent=2))


if __name__ == "__main__":
    main()
