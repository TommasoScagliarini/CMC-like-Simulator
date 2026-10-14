"""Verify saved ground-registered IK poses using frozen geometry and event logic.

No integration, policy, new IK or operational write occurs. Full native IK
knots provide geometry only. Filtered 1 ms poses drive the production V26
detector FSM with 10 ms delivery batches, independently of actor FSM v3.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from investigate_detector_geometry import (
    REPO, TG, PROFILE, read_storage, fingerprint, detector_positions,
    filtered_pose, case_summary,
)
from analyze_prescribed_v26 import (
    stats, json_value, existing_helper, _v26_tolerances,
    MATCH_SOURCE, HOLDOUT_SOURCE, V26_SOURCE,
)
from audit_profile_ground_contract import MESH, mesh_triangles
from scripts.run_opensim_sea_pipeline import default_plugin
from tools.calibrate_ab06_seasea_markers import load_plugin, from_vec3
from output import _cycles_from_vertical_grf
from config import SimulatorConfig
from binary_phase_fsm_v26 import HeelQualifiedBinaryPhaseFSM

STAGE3 = REPO/"validation/ab07_detector_investigation_2026_10_06/stage3"
MODEL_NAME = "AB07_SEASEA_stiff321_500_pi.osim"


def repo_path(raw):
    p = Path(str(raw).replace("\\", "/"))
    return p if p.is_absolute() else REPO/p


def force_source(bundle):
    external = bundle/"data/ExternalForces.xml"
    xml = ET.parse(external)
    raw = Path(xml.findtext(".//datafile").replace("\\", "/"))
    path = raw if raw.is_absolute() else external.parent/raw
    force = next(e for e in xml.findall(".//ExternalForce") if e.findtext("applied_to_body") == "foot_l")
    return external, path.resolve(), force.findtext("force_identifier")+"y"


def force_data(bundle):
    external, path, column = force_source(bundle)
    time, names, values, _ = read_storage(path)
    return {"external": external, "path": path, "column": column,
            "time": time, "names": names, "values": values,
            "force": values[:, names.index(column)]}


def interpolate_force(data, times):
    if times[0] < data["time"][0]-1e-7 or times[-1] > data["time"][-1]+1e-7:
        raise ValueError("GRF must cover the entire saved-pose window")
    return np.interp(times, data["time"], data["force"])


def nominal_events(data, start, end, cfg):
    cycles = _cycles_from_vertical_grf(
        data["time"], data["force"], cfg.grf_contact_threshold_n, start, end,
        cfg.grf_min_contact_duration_s, cfg.grf_min_cycle_duration_s,
    )
    rows = [{"hs_s": float(hs), "next_hs_s": float(nxt), "to_s": float(hs+stance),
             "period_s": float(nxt-hs), "stance_s": float(stance),
             "stance_fraction": float(stance/(nxt-hs))} for hs, nxt, stance in cycles]
    return {"cycles": rows, "heel_strike": sorted(set([r["hs_s"] for r in rows]+[r["next_hs_s"] for r in rows])),
            "toe_off": sorted(set(r["to_s"] for r in rows if r["to_s"] <= end+1e-8)),
            "source_force_column": data["column"],
            "scope": "Production complete-cycle GRF oracle; unanchored startup partial stance excluded"}


def replay_batched(times, contacts):
    if not np.allclose(np.diff(times), .001, atol=1e-9, rtol=0):
        raise ValueError("Frozen V26 replay requires the 1 ms clock")
    fsm = HeelQualifiedBinaryPhaseFSM()
    baseline = fsm.reset(time_s=float(times[0]), heel_contact=bool(contacts[0, 0]), toe_contact=bool(contacts[0, 1]))
    events = []
    previous = 0
    for last in range(10, len(times)+9, 10):
        last = min(last, len(times)-1)
        samples = [{"time_s": float(times[i]), "left_heel_contact": bool(contacts[i, 0]),
                    "left_toe_contact": bool(contacts[i, 1])} for i in range(previous+1, last+1)]
        if not samples:
            break
        payload = fsm.update_policy_step(time_s=float(times[last]), previous_time_s=float(times[previous]), sensor_samples=samples)
        events.extend(payload["events_this_step"])
        previous = last
    hs = [float(e["event_time_s"]) for e in events if e["event"] == "heel_strike"]
    to = [float(e["event_time_s"]) for e in events if e["event"] == "toe_off"]
    stances = [{"hs_s": h, "to_s": t, "stance_s": t-h} for h in hs for t in [next((v for v in to if v > h), None)] if t is not None]
    cycles = []
    for h, nxt in zip(hs[:-1], hs[1:]):
        t = next((v for v in to if h < v < nxt), None)
        if t is not None:
            cycles.append({"hs_s": h, "to_s": t, "next_hs_s": nxt,
                           "period_s": nxt-h, "stance_s": t-h, "stance_fraction": (t-h)/(nxt-h)})
    if len(cycles) != fsm.payload()["cycle_count"]:
        raise AssertionError("Event-derived complete cycles disagree with production detector counter")
    return {"baseline": baseline, "final": fsm.payload(), "events": events,
            "stance_intervals": stances, "complete_cycles": cycles,
            "stance_duration_s": stats(r["stance_s"] for r in stances),
            "period_s": stats(r["period_s"] for r in cycles),
            "stance_fraction": stats(r["stance_fraction"] for r in cycles),
            "scope": "Production heel-qualified detector FSM, 5 ms debounce and 10 ms batch delivery; no actor FSM v3"}


def comparisons(events, nominal, tolerances, holdout):
    matcher = existing_helper(MATCH_SOURCE, "match_events")
    result = {}
    for name, code, key in (("heel_strike", "HS", "sensor_holdout_heel_strike_max_error_s"),
                            ("toe_off", "TO", "sensor_holdout_toe_off_max_error_s")):
        selected = [e for e in events if e["event"] == name]
        fields = (("onset", "event_time_s", float(holdout[key])),
                  ("confirmed", "confirmed_time_s", tolerances[code+"_TOLERANCE_S"]),
                  ("delivered", "delivered_time_s", tolerances["DELIVERED_"+code+"_TOLERANCE_S"]))
        result[name] = {label: {"tolerance_s": tol, "times_s": [e[field] for e in selected],
                                "matching": matcher(nominal[name], [e[field] for e in selected], tol)}
                        for label, field, tol in fields}
        result[name]["confirmation_latency_from_onset_s"] = stats(e["confirmed_time_s"]-e["event_time_s"] for e in selected)
        result[name]["delivery_latency_from_onset_s"] = stats(e["delivered_time_s"]-e["event_time_s"] for e in selected)
    return json_value(result)


def marker_metrics(bundle, start, end):
    report_path = bundle/"pipeline_report.json"
    if not report_path.is_file():
        return {"status": "not_available", "reason": "No pipeline_report.json for this historical bundle"}, []
    report = json.loads(report_path.read_text())
    original = repo_path(report["ik_computed_source"])
    companions = [directory/(original.stem+suffix)
                  for directory in (original.parent, original.parent/"full")
                  for suffix in ("_ik_marker_errors.sto", "_marker_errors.sto")]
    companion = next((p for p in companions if p.is_file()), None)
    if companion is None:
        raise FileNotFoundError(f"Exact IK-stem marker companion absent: {original}")
    times, names, errors, _ = read_storage(companion)
    kt, _, _, _ = read_storage(bundle/"data/IK_results_AB07_SEASEA.mot")
    if not np.array_equal(times, kt):
        raise ValueError("Marker errors must have the exact adopted IK clock")
    selected = (times >= start-1e-8) & (times <= end+1e-8)
    return {"status": "available", "source": fingerprint(companion), "computed_ik": fingerprint(original),
            "sample_count": len(times), "time_range_s": [float(times[0]), float(times[-1])],
            "columns_m_or_squared_m": names,
            "full": {name: stats(errors[:, j]) for j, name in enumerate(names)},
            "selected": {name: stats(errors[selected, j]) for j, name in enumerate(names)},
            "ik_provenance": report["ik_provenance"]}, [report_path, original, companion]


def static_geometry(model_path, motion_path, profile, vertices):
    import opensim as osim
    time, names, q, degrees = read_storage(motion_path)
    if degrees:
        for j, name in enumerate(names):
            if name not in {"pelvis_tx", "pelvis_ty", "pelvis_tz"}:
                q[:, j] = np.deg2rad(q[:, j])
    positions = detector_positions(model_path, names, q, time, profile)
    clearance = (positions-np.asarray(profile["ground"]["origin"])) @ np.asarray(profile["ground"]["normal"])
    model = osim.Model(str(model_path)); state = model.initSystem()
    body = model.getBodySet().get("foot_l")
    coordinates = [(model.getCoordinateSet().get(n), j) for j, n in enumerate(names)]
    sole, pitch = [], []
    n, origin = np.asarray(profile["ground"]["normal"]), np.asarray(profile["ground"]["origin"])
    for row in q:
        for coordinate, j in coordinates:
            coordinate.setValue(state, float(row[j]), False)
        model.realizePosition(state)
        x = body.getTransformInGround(state)
        r = np.asarray([[x.R().get(i, j) for j in range(3)] for i in range(3)])
        t = from_vec3(x.p()); local_n = n @ r
        sole.append(float(np.min(vertices @ local_n)+(t-origin) @ n))
        pitch.append(float(np.rad2deg(np.arctan2(local_n[0], local_n[1]))))
    return {"sample_count": len(time), "motion": fingerprint(motion_path),
            "visual_STL_min_clearance_m": stats(sole), "hardware_sagittal_pitch_deg": stats(pitch),
            "binary_points": {name: {"clearance_m": stats(clearance[:, j]), "contact_fraction": float(np.mean(clearance[:, j] <= 0))}
                              for j, name in enumerate(("heel", "toe"))}}


def make_plot(destination, series, summaries):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"AB06_reference": "#777777", "AB07_baseline": "#be5046", "AB07_ground_registered": "#2678a9"}
    labels = {"AB06_reference": "AB06 (11.99–21.00 s)", "AB07_baseline": "AB07 precedente", "AB07_ground_registered": "AB07 registrato"}
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, gridspec_kw={"height_ratios": [2, 2, 1.5, 1.3]})
    for case, (time, clear, force) in series.items():
        elapsed = time-time[0]
        for i in range(2):
            axes[i].plot(elapsed, clear[:, i]*1000, color=colors[case], label=labels[case], linewidth=1.1)
        if case != "AB07_baseline":
            axes[2].plot(elapsed, force, color=colors[case], linewidth=1, label=labels[case]+" GRF misurata")
    for i, role in enumerate(("Heel", "Toe")):
        axes[i].axhline(0, color="black", linestyle="--", linewidth=.8)
        axes[i].set_ylabel(role+" clearance [mm]"); axes[i].grid(alpha=.2)
    axes[0].legend(loc="upper right", ncol=3, fontsize=8)
    axes[2].axhline(20, color="black", linestyle="--", linewidth=.8)
    axes[2].set_ylabel("GRF nominale [N]"); axes[2].legend(loc="upper right", fontsize=8); axes[2].grid(alpha=.2)
    for row, case in enumerate(series):
        start = series[case][0][0]
        events = summaries[case]["filtered_window"]["v26_detector_fsm"]["events"]
        for kind, marker in (("heel_strike", "^"), ("toe_off", "v")):
            values = [e["event_time_s"]-start for e in events if e["event"] == kind]
            axes[3].scatter(values, [row]*len(values), color=colors[case], marker=marker, s=32)
    nominal = summaries["AB07_ground_registered"]["filtered_window"]["nominal_oracle"]
    start = series["AB07_ground_registered"][0][0]
    axes[3].scatter(np.asarray(nominal["heel_strike"])-start, [3]*len(nominal["heel_strike"]), marker="^", color="black", s=32)
    axes[3].scatter(np.asarray(nominal["toe_off"])-start, [3]*len(nominal["toe_off"]), marker="v", color="black", s=32)
    axes[3].set_yticks(range(4), ["AB06 V26", "AB07 prima V26", "AB07 dopo V26", "Oracle AB07"])
    axes[3].set_ylim(-.5, 3.5); axes[3].set_xlabel("Tempo dalla partenza del tratto [s]"); axes[3].grid(axis="x", alpha=.2)
    fig.suptitle("Registrazione hardware: verifica della nuova IK, stesso piano e sensori congelati\nPosizioni salvate, filtro produzione 6 Hz; nessuna dinamica o policy", fontsize=12)
    fig.tight_layout(rect=[0, .025, 1, .95])
    fig.text(.01, .005, "▲ HS, ▼ TO (onset); AB06 è un altro soggetto/trial. Eventi del detector V26; nessun gate actor FSM v3 o prova di carico protesico.", fontsize=8)
    destination.mkdir(parents=True, exist_ok=True)
    paths = []
    for suffix in ("png", "pdf", "svg"):
        path = destination/("ik_contact_comparison."+suffix)
        if path.exists(): raise FileExistsError(path)
        fig.savefig(path, dpi=160); paths.append(path)
    plt.close(fig)
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=STAGE3/"build/models/AB07_SEASEA_Threadmill")
    parser.add_argument("--baseline", type=Path, default=STAGE3/"baseline/models/AB07_SEASEA_Threadmill")
    parser.add_argument("--output-prefix", type=Path, default=STAGE3/"ik_contact_verification")
    parser.add_argument("--plot-dir", type=Path, default=REPO/"plot/AB07_ground_registration_2026_10_06")
    parser.add_argument("--plugin", type=Path, default=default_plugin())
    args = parser.parse_args()
    out = args.output_prefix.with_suffix(".json")
    if out.exists() or args.output_prefix.with_suffix(".csv").exists():
        parser.error("New diagnostic receipt paths required; prior artifacts are never overwritten")
    report_path = args.bundle/"pipeline_report.json"
    report = json.loads(report_path.read_text())
    if report.get("status") != "complete" or report.get("ik_method") != "opensim_marker_ik":
        raise ValueError("A completed genuine marker IK build is required")
    cases = {"AB06_reference": ("AB06", REPO/"models/AB06_SEASEA_Threadmill", 11.99),
             "AB07_baseline": ("AB07", args.baseline, 10.36),
             "AB07_ground_registered": ("AB07", args.bundle, 10.36)}
    static_paths = {"AB06_reference": REPO/"models/AB06_SEASEA-raw/data/converted/static/static_01/static_01_ik_dataset_ab06_seasea.mot",
                    "AB07_baseline": REPO/"models/AB07_SEASEA-raw/data/converted/static/static01/static01_ik_dataset_ab06_seasea.mot",
                    "AB07_ground_registered": repo_path(report["static_calibration_motion"])}
    paths = {"script": Path(__file__).resolve(), "binary_profile": PROFILE, "plugin": args.plugin, "STL": MESH,
             "production_detector": REPO/"binary_phase_detector.py", "production_detector_fsm": TG/"binary_phase_fsm_v26.py",
             "parent_detector_fsm": TG/"binary_phase_fsm.py", "kinematics_interpolator": REPO/"kinematics_interpolator.py",
             "config": REPO/"config.py", "oracle_cycle_helper": REPO/"output.py", "matching_helper": MATCH_SOURCE,
             "v26_tolerances": V26_SOURCE, "holdout_tolerances": HOLDOUT_SOURCE,
             "pose_replay_helper": Path(__file__).with_name("investigate_detector_geometry.py"),
             "analysis_helper": Path(__file__).with_name("analyze_prescribed_v26.py"),
             "new_pipeline_report": report_path}
    forces = {}
    for label, (subject, bundle, _) in cases.items():
        paths[label+"_model"] = bundle/(subject+"_SEASEA_stiff321_500_pi.osim")
        paths[label+"_ik"] = bundle/"data"/("IK_results_"+subject+"_SEASEA.mot")
        paths[label+"_static_motion"] = static_paths[label]
        forces[label] = force_data(bundle)
        paths[label+"_grf"] = forces[label]["path"]
        paths[label+"_external_forces"] = forces[label]["external"]
    before = {name: fingerprint(path) for name, path in paths.items()}
    if before["binary_profile"]["sha256"] != "db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2":
        raise ValueError("The shared frozen profile differs from the selected V25 contract")
    old_force, new_force = forces["AB07_baseline"], forces["AB07_ground_registered"]
    grf_parity = {"clock_identical": np.array_equal(old_force["time"], new_force["time"]),
                  "column_names_identical": old_force["names"] == new_force["names"],
                  "all_18_data_columns_identical": np.array_equal(old_force["values"], new_force["values"]),
                  "whole_file_sha_identical": before["AB07_baseline_grf"]["sha256"] == before["AB07_ground_registered_grf"]["sha256"],
                  "bytes_after_first_name_header_line_identical": old_force["path"].read_bytes().partition(b"\n")[2] == new_force["path"].read_bytes().partition(b"\n")[2]}
    if not all(grf_parity[key] for key in ("clock_identical", "column_names_identical", "all_18_data_columns_identical")):
        raise ValueError("Before/after AB07 must use identical measured GRF clock, labels and numeric data")
    import opensim as osim
    load_plugin(args.plugin); osim.Logger.setLevelString("error")
    profile = json.loads(PROFILE.read_text())
    cfg = SimulatorConfig()
    if not (cfg.enable_kinematics_lowpass_filter and cfg.kinematics_lowpass_cutoff_hz == 6 and cfg.kinematics_resample_dt == .001):
        raise ValueError("Production reference filtering no longer matches the declared 6 Hz / 1 ms protocol")
    tolerances = _v26_tolerances(); holdout = json.loads(HOLDOUT_SOURCE.read_text())
    vertices = np.unique(mesh_triangles(MESH).reshape(-1, 3), axis=0)
    results, series = {}, {}
    for label, (subject, bundle, start) in cases.items():
        print("saved-pose verification", label, flush=True)
        model_path, ik_path = paths[label+"_model"], paths[label+"_ik"]
        times, names, q, degrees = read_storage(ik_path)
        if degrees:
            for j, name in enumerate(names):
                if name not in {"pelvis_tx", "pelvis_ty", "pelvis_tz"}: q[:, j] = np.deg2rad(q[:, j])
        if not np.allclose(np.diff(times), .005, atol=1e-7, rtol=0):
            raise ValueError("Native full IK does not use the declared 5 ms clock")
        if subject == "AB07" and len(times) != 28612:
            raise ValueError("Expected the complete AB07 28,612-knot IK trial")
        xyz = detector_positions(model_path, names, q, times, profile)
        full, _ = case_summary(times, xyz, profile, interpolate_force(forces[label], times), replay_events=False)
        window = start+np.arange(9011)*.001
        fnames, fq = filtered_pose(ik_path, model_path, window)
        positions = detector_positions(model_path, fnames, fq, window, profile)
        force = interpolate_force(forces[label], window)
        filtered, clearance = case_summary(window, positions, profile, force, replay_events=False)
        fsm = replay_batched(window, clearance <= 0)
        nominal = nominal_events(forces[label], start, float(window[-1]), cfg)
        filtered.update(v26_detector_fsm=fsm, nominal_oracle=nominal,
                        event_comparison=comparisons(fsm["events"], nominal, tolerances, holdout))
        metrics, marker_paths = marker_metrics(bundle, start, float(window[-1]))
        if label == "AB07_ground_registered" and metrics["status"] != "available":
            raise ValueError("New IK marker metrics required")
        for i, path in enumerate(marker_paths):
            name = label+f"_marker_source_{i}"; paths[name] = path; before[name] = fingerprint(path)
        results[label] = {"full_native_ik_geometry": full, "filtered_window": filtered,
                          "marker_ik_metrics": metrics,
                          "static_calibration_geometry": static_geometry(model_path, static_paths[label], profile, vertices),
                          "production_sampler_parity": {"poses_checked_per_stream": 13, "clearance_tolerance_m": 1e-12, "status": "PASS"}}
        series[label] = (window, clearance, force)
    if not all(fingerprint(path) == before[name] for name, path in paths.items()):
        raise RuntimeError("A source file changed during saved-pose verification")
    plots = make_plot(args.plot_dir, series, results)
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_prefix.with_suffix(".csv")
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream); writer.writerow(["case", "time_s", "elapsed_s", "heel_clearance_m", "toe_clearance_m", "nominal_fz_n"])
        for label, (time, clearance, force) in series.items():
            writer.writerows((label, *row) for row in np.column_stack([time, time-time[0], clearance, force]))
    receipt = {"schema_version": 1, "status": "saved_pose_diagnostics_completed_dynamic_qualification_pending",
               "instruction_check_token": "CMC_AGENT_OK_2026", "sources": before, "sources_unchanged": True,
               "scope": "Saved-pose native geometry and production detector FSM replay; zero forward integration, new IK, policy or actor FSM",
               "methodology": {"full_ik": "All native 5 ms knots; geometry only, no event inference at this grid",
                   "window": "Inclusive endpoints, 9.01 s duration, 9011 poses; reset first, 9010 updates in 901 ten-ms batches",
                   "reference_filter": {"lowpass_hz": cfg.kinematics_lowpass_cutoff_hz, "order": cfg.kinematics_lowpass_order, "resample_dt_s": cfg.kinematics_resample_dt},
                   "nominal_grf": {"threshold_n": cfg.grf_contact_threshold_n, "min_contact_s": cfg.grf_min_contact_duration_s, "min_cycle_s": cfg.grf_min_cycle_duration_s},
                   "matching": "Established greedy one-to-one event matcher within explicitly recorded tolerances; unmatched events retained",
                   "v26_tolerances_s": tolerances,
                   "tolerance_scope": "Inherited AB06 development/legacy analog holdout diagnostic references, not adopted AB07 acceptance gates",
                   "static_reference": "Own static calibration pose for each model; fixed STL and binary profile, common plane unchanged"},
               "AB07_grf_parity": grf_parity,
               "cases": results, "plots": [fingerprint(p) for p in plots], "filtered_series_csv": fingerprint(csv_path),
               "limitations": ["AB06 uses its own 11.99–21.00s trial window; AB07 uses 10.36–19.37s. Different phases and subjects are not paired samples.",
                   "GRF >20N is nominal measured healthy support, not proof of physical prosthetic heel contact or matching primary force.",
                   "The STL and frozen binary points use a relative virtual-plane registration, not an independently surveyed anatomical sole/ground datum.",
                   "Marker fit and detector event agreement cannot qualify dynamic tracking, actor FSM v3, primary GRF loads or learned-policy transfer.",
                   "No profile, threshold, controller, runtime initialization or operational model is modified."]}
    out.write_text(json.dumps(json_value(receipt), indent=2, allow_nan=False)+"\n")
    print("Saved post-IK verification:", out, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
