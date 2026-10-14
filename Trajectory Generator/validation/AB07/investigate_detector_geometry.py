"""Saved-pose detector diagnosis; no integration, policy, or production writes.

Compare the frozen heel/toe sensor on AB06 and AB07, replay its event logic,
and isolate reference filtering, governor joints and mounting in counterfactual
poses. Counterfactuals are diagnostic poses, not candidate runtime settings.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
import tempfile
from xml.etree import ElementTree as ET

import numpy as np
from scipy.interpolate import CubicSpline

REPO = Path(__file__).resolve().parents[3]
TG = REPO / "Trajectory Generator"
for directory in (REPO, TG, Path(__file__).resolve().parent):
    sys.path.insert(0, str(directory))

from analyze_prescribed_v26 import read_storage, stats, fingerprint

PROFILE = REPO / (
    "validation/binary_phase_detector_v25_geometry_runs/"
    "2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json"
)
SESSION = REPO / "results/ab07_v26_left_prescribed_9p01s_2026_10_06"


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def storage_pose(source: Path, times: np.ndarray) -> tuple[list[str], np.ndarray]:
    original_time, names, data, degrees = read_storage(source)
    if times[0] < original_time[0] - 1e-7 or times[-1] > original_time[-1] + 1e-7:
        raise ValueError(f"Pose request exceeds source coverage: {source}")
    if any(name.endswith("_q_ref") for name in names):
        columns = [i for i, name in enumerate(names) if name.endswith("_q_ref")]
        names = [names[i][:-6] for i in columns]
        data = data[:, columns]
    data = np.array(data, copy=True)
    if degrees:
        for column, name in enumerate(names):
            if name not in {"pelvis_tx", "pelvis_ty", "pelvis_tz"}:
                data[:, column] = np.deg2rad(data[:, column])
    return names, CubicSpline(original_time, data, axis=0)(times)


def filtered_pose(source: Path, model: Path, times: np.ndarray) -> tuple[list[str], np.ndarray]:
    from config import SimulatorConfig
    from kinematics_interpolator import KinematicsInterpolator

    cfg = SimulatorConfig()
    cfg.model_bundle_dir = str(model.parent)
    cfg.model_file = str(model)
    cfg.kinematics_file = str(source)
    cfg.t_start, cfg.t_end = float(times[0]), float(times[-1])
    interpolation = KinematicsInterpolator(cfg)
    return interpolation.coord_names, np.column_stack([
        interpolation._splines[name](times) for name in interpolation.coord_names
    ])


def measured_force(subject: str, times: np.ndarray) -> np.ndarray:
    bundle = REPO / f"models/{subject}_SEASEA_Threadmill"
    external = bundle / "data/ExternalForces.xml"
    xml = ET.parse(external)
    raw = Path(xml.findtext(".//datafile").replace("\\", "/"))
    source = raw if raw.is_absolute() else external.parent / raw
    time, names, data, _ = read_storage(source)
    if time[0] > times[0] or time[-1] < times[-1] - 1e-7:
        raise ValueError("Measured GRF does not cover the pose window")
    force = next(e for e in xml.findall(".//ExternalForce")
                 if e.findtext("applied_to_body") == "foot_l")
    return np.interp(times, time, data[:, names.index(force.findtext("force_identifier") + "y")])


def detector_positions(model_path: Path, names: list[str], values: np.ndarray,
                       times: np.ndarray, profile: dict) -> np.ndarray:
    import opensim
    from binary_phase_detector import BinaryPhaseDetectorSampler, load_binary_phase_detector_profile
    model = opensim.Model(str(model_path))
    state = model.initSystem()
    coordinates = [(model.getCoordinateSet().get(name), column)
                   for column, name in enumerate(names)]
    if len(coordinates) != model.getCoordinateSet().getSize():
        raise ValueError("Every model coordinate must have one explicit pose value")
    frames = [opensim.PhysicalFrame.safeDownCast(model.getComponent(p["frame"]))
              for p in profile["points"]]
    if any(frame is None for frame in frames):
        raise ValueError("Detector PhysicalFrame missing")
    stations = [opensim.Vec3(*p["location"]) for p in profile["points"]]
    sampler = BinaryPhaseDetectorSampler(model, load_binary_phase_detector_profile(PROFILE))
    parity_indices = set(np.linspace(0, len(times)-1, 13, dtype=int).tolist())
    positions = np.empty((len(times), 2, 3))
    for row, (time, q) in enumerate(zip(times, values)):
        state.setTime(float(time))
        for coordinate, column in coordinates:
            coordinate.setValue(state, float(q[column]), False)
        model.realizePosition(state)
        for point, (frame, station) in enumerate(zip(frames, stations)):
            vector = frame.findStationLocationInGround(state, station)
            positions[row, point] = [vector.get(i) for i in range(3)]
        if row in parity_indices:
            production = sampler.sample(state, float(time))
            manual = (positions[row] - np.asarray(profile["ground"]["origin"])) @ np.asarray(profile["ground"]["normal"])
            for point, role in enumerate(("left_heel", "left_toe")):
                if abs(production.signed_clearance_m[role] - manual[point]) > 1e-12:
                    raise AssertionError("Production sampler clearance differs from pose diagnostic")
                if production.contacts[role] != bool(manual[point] <= 0):
                    raise AssertionError("Production sampler contact bit differs from pose diagnostic")
    if not np.all(np.isfinite(positions)):
        raise ValueError("Non-finite sensor positions")
    return positions


def replay(times: np.ndarray, contacts: np.ndarray, *, heel_qualified: bool) -> dict:
    from binary_phase_fsm import BinaryPhaseFSM
    from binary_phase_fsm_v26 import HeelQualifiedBinaryPhaseFSM

    if not np.allclose(np.diff(times), 0.001, atol=1e-9, rtol=0):
        raise ValueError("FSM replay requires the shared 1 ms sample clock")
    fsm = HeelQualifiedBinaryPhaseFSM() if heel_qualified else BinaryPhaseFSM()
    fsm.reset(time_s=float(times[0]), heel_contact=bool(contacts[0, 0]),
              toe_contact=bool(contacts[0, 1]))
    events = []
    stance = [bool(fsm.payload()["in_contact"])]
    for i in range(1, len(times)):
        result = fsm.update(time_s=float(times[i]), heel_contact=bool(contacts[i, 0]),
                            toe_contact=bool(contacts[i, 1]))
        events.extend(result["events_this_step"])
        stance.append(bool(result["in_contact"]))
    return {"event_count": fsm.payload()["event_count"],
            "functional_contact_fraction": float(np.mean(stance)),
            "events": events,
            "scope": "Detector event interpretation only; not the downstream actor FSM v3"}


def case_summary(times: np.ndarray, positions: np.ndarray, profile: dict,
                 force: np.ndarray, *, replay_events: bool) -> tuple[dict, np.ndarray]:
    origin = np.asarray(profile["ground"]["origin"])
    normal = np.asarray(profile["ground"]["normal"])
    clearance = (positions - origin) @ normal
    contacts = clearance <= 0
    loaded = force > 20.0
    result = {
        "pose_count": len(times), "time_range_s": [float(times[0]), float(times[-1])],
        "measured_grf_contact_fraction_gt20n": float(np.mean(loaded)),
        "clearance_m": {name: {"all": stats(clearance[:, i]),
                                "during_measured_contact": stats(clearance[loaded, i])}
                        for i, name in enumerate(("heel", "toe"))},
        "contact_fractions": {
            "heel": float(np.mean(contacts[:, 0])), "toe": float(np.mean(contacts[:, 1])),
            "any": float(np.mean(np.any(contacts, axis=1))),
            "heel_during_measured_contact": float(np.mean(contacts[loaded, 0])),
            "toe_during_measured_contact": float(np.mean(contacts[loaded, 1])),
            "any_during_measured_contact": float(np.mean(np.any(contacts[loaded], axis=1))),
            "toe_only": float(np.mean(~contacts[:, 0] & contacts[:, 1])),
        },
    }
    if replay_events:
        result["frozen_v26_heel_qualified"] = replay(times, contacts, heel_qualified=True)
        result["counterfactual_v20_any_contact"] = replay(times, contacts, heel_qualified=False)
    return result, clearance


def saved_trace_parity() -> dict:
    from binary_phase_fsm_v26 import HeelQualifiedBinaryPhaseFSM
    receipt = json.loads((SESSION / "receipt.json").read_text())
    baseline = receipt["reset"]["binary_phase_sensor_baseline"]
    fsm = HeelQualifiedBinaryPhaseFSM()
    fsm.reset(time_s=baseline["time_s"], heel_contact=baseline["left_heel_contact"],
              toe_contact=baseline["left_toe_contact"])
    previous, observed, recorded, samples = baseline["time_s"], [], [], 0
    with (SESSION / "trace.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            raw = row["binary_phase_sensor_samples"]
            payload = fsm.update_policy_step(time_s=row["time"], previous_time_s=previous,
                                             sensor_samples=raw)
            observed.extend(payload["events_this_step"])
            recorded.extend(row["binary_phase_fsm"]["events_this_step"])
            samples += len(raw)
            previous = row["time"]
    if observed != recorded:
        raise AssertionError("Saved Boolean samples do not reproduce recorded detector events")
    return {"status": "PASS", "samples": samples, "events": len(observed),
            "exact_event_dictionary_parity": True,
            "scope": "Replays the original 10 ms delivery batches, no OpenSim integration"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path,
                        default=REPO / "validation/ab07_detector_investigation_2026_10_06")
    parser.add_argument("--force", action="store_true", help="Replace this tool's diagnostic JSON/CSV only.")
    args = parser.parse_args()
    if not args.force and any((args.output_dir / name).exists() for name in
                              ("detector_geometry_analysis.json", "sensor_pose_comparison.csv")):
        parser.error("Diagnostic output exists; choose another directory or use --force.")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    import opensim
    from config import SimulatorConfig
    from model_loader import _configure_geometry_search_paths, _load_plugin

    opensim.Logger.setLevelString("error")
    _load_plugin(str(REPO / SimulatorConfig().plugin_name))
    _configure_geometry_search_paths(str(REPO / "models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi.osim"))
    protected = json.loads((REPO / "validation/ab07_pipeline_2026_10_06/control_fingerprint_before.json").read_text())
    extra = [PROFILE, REPO / "Trajectory Generator/osim_trj_cmc_like.py",
             REPO / "Trajectory Generator/binary_phase_fsm_v26.py",
             REPO / "models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi.osim"]
    before = {str(p): fingerprint(p)["sha256"] for p in extra}
    profile = json.loads(PROFILE.read_text())
    models = {s: REPO / f"models/{s}_SEASEA_Threadmill/{s}_SEASEA_stiff321_500_pi.osim"
              for s in ("AB06", "AB07")}
    sources = {s: models[s].parent / f"data/IK_results_{s}_SEASEA.mot" for s in models}
    times = {"AB06": 11.99 + np.arange(9010) * 0.001,
             "AB07": 10.36 + np.arange(9010) * 0.001}
    profile_hash = fingerprint(PROFILE)["sha256"]
    if profile_hash != "db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2":
        raise ValueError("Frozen profile changed")
    named = {}
    for subject in models:
        named[subject] = filtered_pose(sources[subject], models[subject], times[subject])
    ab07_names, ab07_q = named["AB07"]
    served_source = SESSION / "sim_outputs/sim_output_kinematics_reference.sto"
    served_names, served = storage_pose(served_source, times["AB07"])
    if ab07_names != served_names:
        raise ValueError("Served reference coordinate order differs")
    raw_names, raw_q = storage_pose(sources["AB07"], times["AB07"])
    healthy_source = models["AB07"].parent / "data/healthy/AB07_treadmill_01_01_Kinematics_q.sto"
    healthy_names, healthy_q = filtered_pose(healthy_source, models["AB07"], times["AB07"])
    cases = [
        ("AB06_new_ik_filtered", "AB06", sources["AB06"], *named["AB06"], None),
        ("AB07_new_ik_raw", "AB07", sources["AB07"], raw_names, raw_q, None),
        ("AB07_new_ik_filtered", "AB07", sources["AB07"], ab07_names, ab07_q, None),
        ("AB07_healthy_remapped_on_fitted_prosthesis", "AB07", healthy_source, healthy_names, healthy_q, None),
    ]
    for label, filename in (("AB07_actual_prescribed", REPO / "results/AB07_prescribed_2026_10_06/sim_output_kinematics.sto"),
                            ("AB07_actual_v26", SESSION / "sim_outputs/sim_output_kinematics.sto"),
                            ("AB07_served_v26", served_source)):
        names, q = storage_pose(filename, times["AB07"])
        cases.append((label, "AB07", filename, names, q, None))
    for joints in (("knee",), ("ankle",), ("knee", "ankle")):
        q = ab07_q.copy()
        for joint in joints:
            name = f"pros_{joint}_angle"
            q[:, ab07_names.index(name)] = served[:, served_names.index(name)]
        cases.append(("AB07_counterfactual_served_" + "_".join(joints), "AB07",
                      served_source, ab07_names, q, {"replaced_joints": list(joints)}))
    # Restore only mounts in a temporary clone to quantify their geometric effect.
    tree = ET.parse(models["AB07"])
    donor = ET.parse(models["AB06"])
    reverted = []
    for joint, frame in (("osseo", "transfemur_offset"), ("pros_ankle", "tibia_pylon_offset")):
        xpath = f".//JointSet/objects/*[@name='{joint}']/frames/PhysicalOffsetFrame[@name='{frame}']/translation"
        actual, reference = tree.find(xpath), donor.find(xpath)
        reverted.append({"joint": joint, "from": actual.text, "to": reference.text})
        actual.text = reference.text
    with tempfile.TemporaryDirectory(prefix="ab07_detector_pose_") as temporary:
        clone = Path(temporary) / "AB07_donor_mount_counterfactual.osim"
        tree.write(clone, encoding="utf-8", xml_declaration=True)
        cases.append(("AB07_counterfactual_donor_mounts", "AB07", sources["AB07"],
                      ab07_names, ab07_q, {"temporary_model": str(clone), "mount_changes": reverted}))
        result = {"schema_version": 2, "status": "diagnosis_only", "instruction_check_token": "CMC_AGENT_OK_2026",
                  "scope": "Saved/reference poses and offline FSM replay; no integration or learned policy",
                  "profile": fingerprint(PROFILE), "cases": {}, "saved_trace_parity": saved_trace_parity(),
                  "counterfactuals_are_not_runtime_changes": True,
                  "limitations": ["AB06 and AB07 windows belong to different subjects/trials and phases.",
                                  "Measured GRF marks nominal healthy support, not proof that prosthetic heel touches.",
                                  "Mount/joint counterfactuals hold other coordinates fixed; they are not dynamically valid rollouts.",
                                  "Plane/points/thresholds are frozen; no fit or threshold sweep is performed."]}
        rows = {}
        for label, subject, source, names, q, counterfactual in cases:
            print("pose replay", label, flush=True)
            model_path = clone if label.endswith("donor_mounts") else models[subject]
            positions = detector_positions(model_path, names, q, times[subject], profile)
            summary, clearance = case_summary(times[subject], positions, profile,
                                               measured_force(subject, times[subject]), replay_events=True)
            summary.update(source=fingerprint(source), model=fingerprint(models[subject]),
                           counterfactual=counterfactual)
            summary["effective_model_sha256"] = fingerprint(model_path)["sha256"]
            summary["model_field_scope"] = "base/source model; effective_model_sha256 identifies the evaluated XML"
            summary["production_sampler_parity"] = {"poses": 13, "points_per_pose": 2,
                                                     "clearance_tolerance_m": 1e-12, "status": "PASS"}
            if counterfactual:
                summary["counterfactual"].pop("temporary_model", None)
            result["cases"][label] = summary
            rows[label] = (times[subject], clearance, measured_force(subject, times[subject]))
        # Full-span geometry at native IK knots; no 1 ms event inference from 5 ms samples.
        for subject in models:
            all_times, names, q, degrees = read_storage(sources[subject])
            if degrees:
                for c, name in enumerate(names):
                    if name not in {"pelvis_tx", "pelvis_ty", "pelvis_tz"}:
                        q[:, c] = np.deg2rad(q[:, c])
            print("full-span pose replay", subject, flush=True)
            xyz = detector_positions(models[subject], names, q, all_times, profile)
            summary, _ = case_summary(all_times, xyz, profile, measured_force(subject, all_times), replay_events=False)
            summary["source"] = fingerprint(sources[subject])
            summary["production_sampler_parity"] = {"poses": 13, "points_per_pose": 2,
                                                     "clearance_tolerance_m": 1e-12, "status": "PASS"}
            result.setdefault("full_span_native_ik", {})[subject] = summary
        with (args.output_dir / "sensor_pose_comparison.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["case", "time_s", "heel_clearance_m", "toe_clearance_m", "nominal_fz_n"])
            for label, (t, clear, force) in rows.items():
                writer.writerows((label, *row) for row in np.column_stack((t, clear, force)))
        result["source_files"] = {"script": fingerprint(Path(__file__)),
                                  "detector": fingerprint(REPO / "binary_phase_detector.py"),
                                  "event_logic": fingerprint(TG / "binary_phase_fsm_v26.py")}
        result["checks"] = {
            "all_20_original_control_fingerprints_unchanged": all(fingerprint(REPO / p)["sha256"] == sha for p, sha in protected.items()),
            "all_additional_protected_fingerprints_unchanged": all(fingerprint(p)["sha256"] == sha for p, sha in before.items()),
        }
        if not all(result["checks"].values()):
            raise AssertionError("Protected source changed")
        write_json(args.output_dir / "detector_geometry_analysis.json", result)
        print(json.dumps({"cases": len(cases), "checks": result["checks"], "output": str(args.output_dir)}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
