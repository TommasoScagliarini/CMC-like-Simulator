"""Validate an ABx prescribed bundle without running a simulator or converter.

The report checks artifacts and provenance, not biomechanical validity.  Use
--capture-fingerprint before preparation, then --fingerprint during validation.
Only the explicitly requested small JSON report is written.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path, PureWindowsPath
import re
from xml.etree import ElementTree as ET

import numpy as np

REPO = Path(__file__).resolve().parents[2]
CONTROL_FILES = (
    "config.py", "main.py", "model_loader.py", "prosthesis_controller.py",
    "outer_loop.py", "inverse_dynamics.py", "static_optimization.py",
    "simulation_runner.py", "kinematics_interpolator.py", "output.py",
    "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim",
    "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def fingerprint() -> dict[str, str]:
    paths = [REPO / name for name in CONTROL_FILES]
    paths += sorted(p for p in (REPO / "plugins").glob("*SEA*") if p.is_file())
    return {p.relative_to(REPO).as_posix(): sha256(p) for p in paths}


def read_storage(path: Path) -> tuple[tuple[str, ...], np.ndarray, bool]:
    """Reject incomplete headers, malformed rows and non-finite coordinates."""
    with path.open(encoding="utf-8") as stream:
        metadata = {}
        for line in stream:
            if line.strip().lower() == "endheader":
                break
            if "=" in line:
                key, value = line.split("=", 1)
                metadata[key.strip().lower()] = value.strip()
        else:
            raise ValueError(f"Missing endheader: {path}")
        labels = tuple(next(line for line in stream if line.strip()).split())
        values = np.loadtxt(stream, ndmin=2)
    if not labels or labels[0].lower() != "time" or len(set(labels)) != len(labels):
        raise ValueError(f"Invalid Storage labels: {path}")
    if values.shape[1] != len(labels) or values.shape[0] < 2:
        raise ValueError(f"Invalid Storage dimensions: {path}: {values.shape}")
    for key, expected in (("nrows", values.shape[0]), ("ncolumns", len(labels))):
        if key in metadata and int(metadata[key]) != expected:
            raise ValueError(f"Header {key} disagrees with data: {path}")
    if not np.all(np.isfinite(values)) or not np.all(np.diff(values[:, 0]) > 0):
        raise ValueError(f"Non-finite data or non-monotone time: {path}")
    return labels, values, metadata.get("indegrees", "no").lower() == "yes"


def resolve_reference(text: str | None, base: Path) -> Path:
    if not text:
        raise ValueError(f"Empty file reference in {base}")
    normalized = text.strip().replace("\\", "/")
    candidate = Path(normalized)
    choices = [candidate] if candidate.is_absolute() else [base / candidate, REPO / candidate]
    for path in choices:
        if path.is_file():
            return path.resolve()
    raise FileNotFoundError(f"Unresolved reference {text!r} from {base}")


def model_info(path: Path) -> dict:
    tree = ET.parse(path)
    bodies = tree.findall(".//BodySet/objects/Body")
    forces = list(tree.find(".//ForceSet/objects"))
    sea = {}
    for force in forces:
        if force.tag != "SeriesElasticActuator":
            continue
        props = {}
        for child in force:
            text = " ".join((child.text or "").split())
            try:
                value = float(text)
            except ValueError:
                value = text
            props[child.tag] = value
        sea[force.get("name")] = props
    return {
        "coordinates": [c.get("name") for c in tree.findall(".//Coordinate")],
        "bodies": [body.get("name") for body in bodies],
        "muscles": [f.get("name") for f in forces if "Muscle" in f.tag],
        "mass_kg": sum(float(body.findtext("mass")) for body in bodies),
        "sea": sea,
        "meshes": sorted({n.text.strip() for n in tree.findall(".//mesh_file") if n.text}),
    }


def completed_run_metadata(status_path: Path, actual_time: np.ndarray) -> dict:
    """A finite partial recording cannot substitute for a completed main.py run."""
    status = dict(line.split("=", 1) for line in status_path.read_text().splitlines() if "=" in line)
    if status.get("status") != "complete" or status.get("grf_mode") != "prescribed":
        raise ValueError("Simulator run must complete with prescribed GRF")
    start, end, final = (float(status[key]) for key in ("t_start", "t_end", "t"))
    integration_dt = float(status["integration_dt"])
    control_dt = float(status["T_control"])
    if (not all(np.isfinite(v) for v in (start, end, final, integration_dt, control_dt))
            or integration_dt <= 0 or control_dt <= 0 or end <= start
            or status.get("use_control_window") not in {"True", "False"}):
        raise ValueError("Invalid run-status interval/integration step")
    window = max(control_dt, integration_dt) if status.get("use_control_window") == "True" else integration_dt
    substeps = max(1, round(window / integration_dt))
    record_dt = window / substeps
    if (abs(actual_time[0] - start) > 1e-7
            or abs(final - end) > record_dt * 0.51 + 1e-7
            or abs(actual_time[-1] - (final - record_dt)) > 1e-7
            or not np.allclose(np.diff(actual_time), record_dt, atol=1e-7, rtol=0)
            or int(status["step"]) != len(actual_time)):
        raise ValueError("Actual output grid/endpoints disagree with the completed run status")
    return {"status": "complete", "requested_time_range_s": [start, end],
            "final_integrated_time_s": final, "last_recorded_time_s": float(actual_time[-1]),
            "record_dt_s": record_dt, "record_before_integration": True}


def validate(args: argparse.Namespace) -> dict:
    bundle = args.bundle.resolve()
    model = resolve_reference(args.model, bundle)
    bundle_models = sorted(bundle.glob("*.osim"))
    if len(bundle_models) != 1 or bundle_models[0].resolve() != model:
        raise ValueError("Bundle must contain one operational .osim model without aliases")
    reference_model = args.reference_model.resolve()
    subject = args.subject.upper()
    healthy_model = args.healthy_model.resolve()
    current, baseline, healthy = map(model_info, (model, reference_model, healthy_model))
    if current["coordinates"] != baseline["coordinates"] or len(current["coordinates"]) != 21:
        raise ValueError("Bundle must preserve the baseline's 21 coordinate names and order")
    current_muscles, baseline_muscles = current["muscles"], baseline["muscles"]
    if (len(current_muscles) != 76 or len(set(current_muscles)) != 76
            or sorted(current_muscles) != sorted(baseline_muscles)):
        missing = sorted(set(baseline_muscles) - set(current_muscles))
        extra = sorted(set(current_muscles) - set(baseline_muscles))
        raise ValueError(
            "Bundle must preserve the baseline's 76 unique biological muscle names: "
            f"missing={missing}, extra={extra}, count={len(current_muscles)}")
    muscle_order_matches = current_muscles == baseline_muscles
    muscle_order_changed_count = sum(a != b for a, b in zip(current_muscles, baseline_muscles))
    if current["sea"] != baseline["sea"] or len(current["sea"]) != 2:
        raise ValueError("SEA XML properties differ from the reference model")
    if abs(current["mass_kg"] - healthy["mass_kg"]) > 1e-8:
        raise ValueError("Prosthetic mass differs from the subject's healthy model mass")
    before = json.loads(args.fingerprint.read_text())
    observed = {name: sha256(REPO / name) for name in before}
    if observed != before:
        raise ValueError(f"Control/baseline fingerprint changed: {[n for n in before if before[n] != observed[n]]}")

    setup_paths = sorted(bundle.glob("*_setup.xml"))
    if len(setup_paths) != 1:
        raise ValueError(f"Expected one operational setup XML, found {len(setup_paths)}")
    setup = ET.parse(setup_paths[0])
    paths = {tag: resolve_reference(setup.findtext(f".//{tag}"), setup_paths[0].parent)
             for tag in ("model_file", "kinematics_file", "external_loads_xml", "reserve_actuators_xml")}
    if paths["model_file"] != model:
        raise ValueError("Setup does not select the validated model")
    for name in ("CMC_Actuators.xml", "CMC_Tasks - modified Kp_Kv.xml"):
        ET.parse(bundle / "data" / name)
    labels, ik, in_degrees = read_storage(paths["kinematics_file"])
    if labels[1:] != tuple(current["coordinates"]) or not in_degrees:
        raise ValueError("IK coordinate labels or degree convention differ from the model")
    if ik.shape[0] != args.expected_rows or not np.allclose(
        ik[[0, -1], 0], args.expected_time, atol=1e-8, rtol=0
    ) or not np.allclose(np.diff(ik[:, 0]), 0.005, atol=1e-8, rtol=0):
        raise ValueError("IK does not cover the expected full trial at 200 Hz")
    overlay_paths = sorted((bundle / "data" / "healthy").glob("*Kinematics_q.sto"))
    if len(overlay_paths) != 1:
        raise ValueError("Expected one explicit healthy kinematic overlay")
    overlay_labels, overlay, overlay_degrees = read_storage(overlay_paths[0])
    if overlay_labels != labels or overlay_degrees != in_degrees or not np.array_equal(ik[:, 0], overlay[:, 0]):
        raise ValueError("Healthy overlay and new IK differ in labels, units or time grid")
    delta = ik[:, 1:] - overlay[:, 1:]
    if np.allclose(delta, 0, atol=1e-7, rtol=0):
        raise ValueError("Operational IK is only the remapped healthy dataset")
    differences = {name: {"rmse": float(np.sqrt(np.mean(delta[:, i] ** 2))),
                           "max_abs": float(np.max(np.abs(delta[:, i])))}
                   for i, name in enumerate(labels[1:])}

    external = ET.parse(paths["external_loads_xml"])
    grf_path = resolve_reference(external.findtext(".//datafile"), paths["external_loads_xml"].parent)
    grf_labels, grf, _ = read_storage(grf_path)
    if grf[0, 0] > ik[0, 0] or grf[-1, 0] < ik[-1, 0]:
        raise ValueError("GRF does not cover the IK interval")
    loads = external.findall(".//ExternalForce")
    if len(loads) != 2 or {load.findtext("applied_to_body") for load in loads} != {"foot_l", "calcn_r"}:
        raise ValueError("ExternalLoads must apply left/right GRF to foot_l/calcn_r")
    for load in loads:
        for tag in ("force_identifier", "point_identifier", "torque_identifier"):
            identifier = load.findtext(tag)
            if not identifier or any(identifier + axis not in grf_labels for axis in "xyz"):
                raise ValueError(f"Missing GRF columns for {tag}: {identifier}")
        for tag in ("force_expressed_in_body", "point_expressed_in_body"):
            if load.findtext(tag) != "ground":
                raise ValueError(f"Unexpected ExternalLoads frame: {tag}")
    geometry_roots = [model.parent, model.parent / "Geometry", REPO / "Geometry", *args.geometry_dir]
    unresolved = []
    for mesh in current["meshes"]:
        name = PureWindowsPath(mesh).name
        if not any((root / name).is_file() for root in geometry_roots):
            unresolved.append(mesh)
    if unresolved:
        raise ValueError(f"Unresolved visual mesh assets: {unresolved}")

    report_path = bundle / "pipeline_report.json"
    report = json.loads(report_path.read_text())
    ik_source = resolve_reference(report.get("ik_computed_source"), bundle)
    if report.get("ik_method") != "opensim_marker_ik" or "IK_full" not in ik_source.name or sha256(ik_source) != sha256(paths["kinematics_file"]):
        raise ValueError("Bundle lacks an identical full-window OpenSim IK source")
    calibrated_model = resolve_reference(report.get("calibrated_model"), bundle)
    provenance = report.get("ik_provenance", {})
    for file_key, hash_key in (("source_file", "source_sha256"),
                               ("model_file", "model_sha256"),
                               ("marker_file", "marker_sha256")):
        provenance_file = resolve_reference(provenance.get(file_key), bundle)
        if sha256(provenance_file) != provenance.get(hash_key):
            raise ValueError(f"IK provenance hash changed: {file_key}")
    if sha256(calibrated_model) != sha256(model):
        raise ValueError("Bundle model differs from the calibrated model used for IK")
    marker_path = calibrated_model.with_suffix(".marker_calibration.csv")
    with marker_path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or any(int(row["samples"]) <= 0 for row in rows):
        raise ValueError("Marker calibration contains no valid samples")
    marker_metrics = {"markers": len(rows), "min_samples": min(int(row["samples"]) for row in rows),
                      "old_mean_error_m": float(np.mean([float(row["old_mean_error_m"]) for row in rows])),
                      "new_mean_fit_rms_m": float(np.mean([float(row["new_fit_rms_m"]) for row in rows])),
                      "max_fit_error_m": max(float(row["new_fit_max_m"]) for row in rows)}
    if not all(np.isfinite(value) for value in marker_metrics.values()):
        raise ValueError("Non-finite marker calibration metrics")
    ik_marker_metrics = {}
    pipeline_root = Path(report.get("pipeline_root", ""))
    if not pipeline_root.is_absolute():
        pipeline_root = REPO / pipeline_root
    ik_report_path = pipeline_root / "pipeline_report.md"
    if ik_report_path.is_file():
        text = ik_report_path.read_text()
        for label, key in (("Maximum frame marker RMS", "max_frame_marker_rms_m"),
                           ("Maximum marker error", "max_marker_error_m")):
            match = re.search(rf"{re.escape(label)}: ([0-9.eE+-]+) m", text)
            if match:
                ik_marker_metrics[key] = float(match.group(1))
    marker_error_name = ik_source.stem + "_ik_marker_errors.sto"
    marker_error_candidates = [ik_source.parent / marker_error_name,
                               ik_source.parent / "full" / marker_error_name]
    marker_error_path = next((p for p in marker_error_candidates if p.is_file()), marker_error_candidates[0])
    marker_error_labels, marker_errors, _ = read_storage(marker_error_path)
    if not np.array_equal(marker_errors[:, 0], ik[:, 0]):
        raise ValueError("Marker error recording does not match the computed IK time grid")
    ik_marker_metrics["source"] = str(marker_error_path)
    ik_marker_metrics["rows"] = len(marker_errors)
    for name in ("marker_error_RMS", "marker_error_max"):
        error_values = marker_errors[:, marker_error_labels.index(name)]
        maximum_row = int(np.argmax(error_values))
        ik_marker_metrics[name] = {"mean_m": float(np.mean(error_values)),
                                   "median_m": float(np.median(error_values)),
                                   "p95_m": float(np.percentile(error_values, 95)),
                                   "max_m": float(error_values[maximum_row]),
                                   "max_time_s": float(marker_errors[maximum_row, 0])}

    state_check = {"status": "not_checked", "reason": "No simulator outputs requested"}
    prescribed_run = None
    if args.results_dir:
        state_path = args.results_dir / f"{args.prefix}_sea_states.sto"
        state_labels, states, _ = read_storage(state_path)
        names = state_labels[1:]
        expected_states = {f"{sea}_{suffix}" for sea in current["sea"]
                           for suffix in ("motor_angle", "motor_speed", "torque_error_integral")}
        if len(names) != 6 or set(names) != expected_states:
            raise ValueError(f"Expected six SEA states, three per actuator: {names}")
        state_check = {"status": "PASS", "labels": list(names), "rows": len(states)}
        actual_labels, actual, actual_degrees = read_storage(
            args.results_dir / f"{args.prefix}_kinematics.sto")
        if actual_labels != labels or not np.array_equal(actual[:, 0], states[:, 0]):
            raise ValueError("Simulated kinematics and SEA states differ in labels/time grid")
        if actual[0, 0] < ik[0, 0] or actual[-1, 0] > ik[-1, 0]:
            raise ValueError("Simulated motion extends beyond the IK reference")
        run_status_path = args.results_dir / f"{args.prefix}_run_status.txt"
        completion = completed_run_metadata(run_status_path, actual[:, 0])
        tracking = {}
        for i, name in enumerate(labels[1:], start=1):
            translation = name in {"pelvis_tx", "pelvis_ty", "pelvis_tz"}
            actual_scale = 1000.0 if translation else (1.0 if actual_degrees else 180.0 / np.pi)
            reference_scale = 1000.0 if translation else (1.0 if in_degrees else 180.0 / np.pi)
            reference_values = np.interp(actual[:, 0], ik[:, 0], ik[:, i])
            error = actual[:, i] * actual_scale - reference_values * reference_scale
            tracking[name] = {"unit": "mm" if translation else "deg",
                              "rmse": float(np.sqrt(np.mean(error ** 2))),
                              "max_abs": float(np.max(np.abs(error)))}
        control_labels, controls, _ = read_storage(
            args.results_dir / f"{args.prefix}_sea_controls.sto")
        if set(control_labels[1:]) != {props["coordinate"] for props in current["sea"].values()} or not np.array_equal(controls[:, 0], actual[:, 0]) or np.max(np.abs(controls[:, 1:])) > 1.00000001:
            raise ValueError("SEA controls violate the normalized command/time contract")
        prescribed_run = {"rows": len(actual), "time_range_s": actual[[0, -1], 0].tolist(),
                          "completion": completion,
                          "tracking_actual_vs_new_ik": tracking,
                          "max_abs_normalized_sea_control": float(np.max(np.abs(controls[:, 1:]))) }
        for suffix in ("reserve_torques", "recruitment"):
            path = args.results_dir / f"{args.prefix}_{suffix}.sto"
            if not path.is_file():
                prescribed_run[suffix] = {"status": "not_provided"}
                continue
            summary_labels, summary_values, _ = read_storage(path)
            if not np.array_equal(summary_values[:, 0], actual[:, 0]):
                raise ValueError(f"Time grid mismatch in {suffix}")
            prescribed_run[suffix] = {
                name: {"mean": float(np.mean(summary_values[:, i])),
                       "max_abs": float(np.max(np.abs(summary_values[:, i])))}
                for i, name in enumerate(summary_labels[1:], start=1)}
    manifest_paths = [model, *paths.values(), grf_path, overlay_paths[0], report_path, marker_path, marker_error_path]
    manifest = {str(p.relative_to(REPO)) if p.is_relative_to(REPO) else str(p): sha256(p)
                for p in dict.fromkeys(manifest_paths)}
    return {"status": "PASS", "subject": subject, "bundle": str(bundle),
            "scope": "Artifact/provenance checks; no integration or biomechanical qualification",
            "model": {"coordinates": 21, "muscles": 76, "mass_kg": current["mass_kg"], "sea": current["sea"],
                      "muscle_names_preserved": True,
                      "muscle_order_matches_reference": muscle_order_matches,
                      "muscle_order_changed_positions": muscle_order_changed_count,
                      "muscle_order_scope": "Non-blocking metadata; simulator lookup is by name, policy compatibility is not assessed"},
            "control_fingerprint_unchanged": True,
            "ik": {"rows": len(ik), "time_range_s": ik[[0, -1], 0].tolist(), "dt_s": 0.005,
                   "source": str(ik_source), "healthy_differences": differences},
            "grf": {"rows": len(grf), "time_range_s": grf[[0, -1], 0].tolist()},
            "marker_calibration": marker_metrics, "ik_marker_errors": ik_marker_metrics,
            "visual_meshes_resolved": len(current["meshes"]),
            "sea_states": state_check, "prescribed_run": prescribed_run, "sha256": manifest}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-fingerprint", type=Path)
    parser.add_argument("--fingerprint", type=Path)
    parser.add_argument("--bundle", type=Path)
    parser.add_argument("--model", default="AB07_SEASEA_stiff321_500_pi.osim")
    parser.add_argument("--subject", default="AB07")
    parser.add_argument("--reference-model", type=Path, default=REPO / CONTROL_FILES[-2])
    parser.add_argument("--healthy-model", type=Path, default=REPO / "models/AB07-raw/osimxml/ab07.osim")
    parser.add_argument("--expected-rows", type=int, default=28612)
    parser.add_argument("--expected-time", nargs=2, type=float, default=[10.36, 153.415])
    parser.add_argument("--geometry-dir", type=Path, action="append", default=[])
    parser.add_argument("--results-dir", type=Path)
    parser.add_argument("--prefix", default="sim_output")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.capture_fingerprint:
        args.capture_fingerprint.parent.mkdir(parents=True, exist_ok=True)
        with args.capture_fingerprint.open("x") as stream:
            json.dump(fingerprint(), stream, indent=2)
            stream.write("\n")
        print(f"Captured fingerprint: {args.capture_fingerprint}")
        return 0
    if not args.bundle or not args.fingerprint:
        parser.error("Validation requires --bundle and --fingerprint")
    try:
        result = validate(args)
    except (OSError, ValueError, KeyError, ET.ParseError) as exc:
        result = {"status": "FAIL", "error": f"{type(exc).__name__}: {exc}"}
    text = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    print(text if result["status"] == "FAIL" else json.dumps({k: result[k] for k in ("status", "subject", "bundle")}))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
