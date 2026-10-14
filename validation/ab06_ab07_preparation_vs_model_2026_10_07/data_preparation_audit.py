"""Read-only EPIC source/converted/bundle audit; no IK solve or dynamics.

The optional output is exclusive. MATLAB's static tables have a checked
12-slot MCOS wrapper, unlike the checked 11-slot dynamic tables. This audit
supports both layouts locally; it does not change any production decoder.
"""
from __future__ import annotations

import argparse
import ast
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
from xml.etree import ElementTree as ET

import numpy as np
from scipy.io.matlab._mio5 import MatFile5Reader


REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from tools import convert_epic_ab06_tables as converter


def fingerprint(path):
    path = Path(path).resolve()
    return {"path": path.relative_to(REPO).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def pure_parsers():
    path = REPO / "tools/calibrate_ab06_seasea_markers.py"
    names = {"TrcData", "MotionData", "parse_trc", "parse_motion"}
    tree = ast.parse(path.read_text())
    nodes = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))
             and n.name in names]
    assert {n.name for n in nodes} == names
    ns = {"Path": Path, "np": np, "math": math, "dataclass": dataclass}
    module = ast.Module(body=nodes, type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(path), "exec"), ns)
    return ns["parse_trc"], ns["parse_motion"]


parse_trc, parse_motion = pure_parsers()


def parse_storage(path):
    lines = Path(path).read_text().splitlines()
    end = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    body = [line for line in lines[end+1:] if line.strip()]
    labels = body[0].split()
    values = np.loadtxt(body[1:], dtype=float)
    assert values.ndim == 2 and values.shape[1] == len(labels)
    assert labels[0].lower() == "time" and np.isfinite(values).all()
    return SimpleNamespace(labels=labels, values=values, times=values[:, 0])


def generic_table(path, public_name=b"data"):
    """Check public/table descriptors and extract numeric 1-D table columns."""
    path = Path(path).resolve()
    with path.open("rb") as stream:
        reader = MatFile5Reader(stream, struct_as_record=True)
        root = reader.get_variables()
    opaque = root.get("None")
    assert isinstance(opaque, np.ndarray) and opaque.size == 1
    assert opaque.dtype.names == ("s0", "s1", "s2", "arr")
    record = opaque.reshape(-1)[0]
    assert record["s0"] == public_name and record["s1"] == b"MCOS"
    assert record["s2"] == b"table"
    workspace = root["__function_workspace__"]
    assert workspace.dtype == np.uint8 and workspace.ndim == 2
    stream = io.BytesIO(workspace.tobytes(order="C"))
    assert stream.read(2) == b"\x00\x01"
    endian = stream.read(2)
    assert endian in (b"IM", b"MI") and stream.read(4) == b"\x00" * 4
    reader.mat_stream = stream
    reader.byte_order = "<" if endian == b"IM" else ">"
    reader.initialize_read()
    header, _ = reader.read_var_header()
    wrapper = reader.read_var_array(header, process=False)
    assert header.name in (None, b"")
    assert wrapper.size == 1 and wrapper.dtype.names == ("MCOS",)
    file_wrapper = wrapper["MCOS"].item()
    assert file_wrapper.size == 1 and file_wrapper.dtype.names == ("s0", "s1", "s2", "arr")
    assert converter._single_bytes(file_wrapper["s0"], "s0") == b""
    assert converter._single_bytes(file_wrapper["s1"], "s1") == b"MCOS"
    assert converter._single_bytes(file_wrapper["s2"], "s2") == b"FileWrapper__"
    payload = np.asarray(file_wrapper["arr"].item(), dtype=object)
    assert payload.shape in ((11, 1), (12, 1)), payload.shape
    slots = payload.ravel(order="F")
    cells = np.asarray(slots[2], dtype=object).ravel(order="F")
    nrows = converter._single_exact_integer(slots[4], "nrows")
    nvars = converter._single_exact_integer(slots[6], "nvars")
    names = tuple(converter._matlab_text(x, "name")
                  for x in np.asarray(slots[7], dtype=object).ravel(order="F"))
    assert len(names) == len(set(names)) == len(cells) == nvars
    columns = {}
    for name, cell in zip(names, cells):
        array = np.asarray(cell)
        assert array.dtype.kind in {"b", "i", "u", "f"} and array.size == nrows
        array = np.asarray(array, dtype=float).ravel(order="F")
        assert np.isfinite(array).all(), (path, name)
        columns[name] = array
    units = None
    metadata = np.asarray(slots[8])
    if metadata.dtype.names and "VariableUnits" in metadata.dtype.names:
        units = [converter._matlab_text(x, "VariableUnits") if np.asarray(x).size else ""
                 for x in metadata["VariableUnits"].item().ravel(order="F")]
    return converter.DecodedTable(path, names, columns, nrows), {
        "wrapper_slot_count": int(len(slots)), "variable_units_raw": units,
        "public_name": public_name.decode(),
        "trialStarts": float(np.asarray(root["trialStarts"]).item()) if "trialStarts" in root else None,
        "trialEnds": float(np.asarray(root["trialEnds"]).item()) if "trialEnds" in root else None}


def clock(t):
    assert np.isfinite(t).all() and (np.diff(t) > 0).all()
    return {"rows": int(len(t)), "range_s": t[[0, -1]].tolist(),
            "median_dt_s": float(np.median(np.diff(t)))}


def stats(x):
    x = np.asarray(x, float)
    assert x.size and np.isfinite(x).all()
    return {"mean": float(np.mean(x)), "rms": float(np.sqrt(np.mean(x * x))),
            "min": float(np.min(x)), "max": float(np.max(x))}


def exact_integral(t, y, lo, hi):
    assert t[0] <= lo < hi <= t[-1]
    grid = np.r_[lo, t[(t > lo) & (t < hi)], hi]
    values = np.interp(grid, t, y)
    return float(np.trapezoid(values, grid))


def array_difference(actual, expected):
    assert actual.shape == expected.shape
    assert np.isfinite(actual).all() and np.isfinite(expected).all()
    return {"shape": list(actual.shape),
            "max_abs_difference": float(np.max(np.abs(actual - expected)))}


def external_loads(path):
    root = ET.parse(path).getroot()
    fields = ("applied_to_body", "force_expressed_in_body", "point_expressed_in_body",
              "force_identifier", "point_identifier", "torque_identifier")
    return {force.attrib.get("name"): {name: force.findtext(name) for name in fields}
            for force in root.findall(".//ExternalForce")}


def marker_quality(subject, selection, trc, model_path, ik_path):
    audit = REPO / "validation/ab08_calibration_investigation_2026_10_06/ik_error_analysis.json"
    cache = audit.with_name(f"ik_error_analysis_{subject.lower()}_residuals.npz")
    manifest = json.loads(audit.read_text())["subjects"][subject]
    for key, path in (("model", model_path), ("ik", ik_path)):
        assert manifest["sources"][key]["sha256"] == fingerprint(path)["sha256"]
    legacy_trc = REPO / f"models/{subject}_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01.trc"
    assert manifest["sources"]["trc"]["sha256"] == fingerprint(legacy_trc)["sha256"]
    legacy = parse_trc(legacy_trc)
    assert np.array_equal(legacy.times, trc.times)
    assert all(np.array_equal(legacy.markers[n], trc.markers[n]) for n in trc.markers)
    with np.load(cache, allow_pickle=False) as data:
        t = data["times"].copy(); names = data["names"].tolist()
        residual = data["residual_ground_m"].copy()
        error = data["error_m"].copy(); rms = data["global_rms_m"].copy()
    assert np.array_equal(t, trc.times)
    assert np.isfinite(residual).all() and np.isfinite(error).all()
    assert np.max(np.abs(np.linalg.norm(residual, axis=2) - error)) < 1e-15
    assert np.max(np.abs(np.sqrt(np.mean(error**2, axis=1)) - rms)) < 1e-15
    lo, hi = selection["primary_cycles"][0]["hs_s"], selection["primary_cycles"][-1]["next_hs_s"]
    mask = (t >= lo) & (t <= hi)
    groups = {"left_prosthetic": [i for i, n in enumerate(names) if n.startswith("L_") and n not in ("L_ASIS", "L_PSIS")],
              "right_biological": [i for i, n in enumerate(names) if n.startswith("R_") and n not in ("R_ASIS", "R_PSIS")],
              "pelvis": [i for i, n in enumerate(names) if n.endswith("ASIS") or n.endswith("PSIS")]}
    return {"scope": "Subset of previously computed native FK on saved IK; no new solver or FK runtime. Source hashes and cache algebra checked.",
            "sources": [fingerprint(audit), fingerprint(cache), fingerprint(legacy_trc)],
            "primary_native_5ms_frame_count": int(mask.sum()),
            "mean_frame_RMS_mm": float(rms[mask].mean() * 1000),
            "max_marker_error_mm": float(error[mask].max() * 1000),
            "frames_with_any_marker_error_over_30mm": int((error[mask].max(axis=1) > .03).sum()),
            "groups": {g: {"mean_frame_RMS_mm": float(np.sqrt(np.mean(error[mask][:, idx]**2, axis=1)).mean() * 1000),
                            "pooled_RMS_mm": float(np.sqrt(np.mean(error[mask][:, idx]**2)) * 1000)} for g, idx in groups.items()},
            "per_marker_RMS_mm": {n: float(np.sqrt(np.mean(error[mask, i]**2)) * 1000) for i, n in enumerate(names)}}


def before_after_ik(selection, final_motion):
    old_path = REPO / "validation/ab07_detector_investigation_2026_10_06/stage3/baseline/models/AB07_SEASEA_Threadmill/data/IK_results_AB07_SEASEA.mot"
    old = parse_motion(old_path)
    assert old.labels == final_motion.labels and np.array_equal(old.times, final_motion.times)
    assert old.in_degrees and final_motion.in_degrees
    windows = [(c["hs_s"], c["next_hs_s"]) for c in selection["primary_cycles"]]
    windows += [(windows[0][0], windows[-1][1])]
    results = []
    for lo, hi in windows:
        grid = np.r_[lo, old.times[(old.times > lo) & (old.times < hi)], hi]
        coordinates = {}
        for name in ("pros_ankle_angle", "pros_knee_angle", "pelvis_ty"):
            index = old.labels.index(name)
            delta = np.interp(grid, final_motion.times, final_motion.values[:, index]) - np.interp(grid, old.times, old.values[:, index])
            scale = 1000 if name == "pelvis_ty" else 1
            coordinates[name] = {"unit": "mm" if name == "pelvis_ty" else "deg",
                "time_weighted_mean": float(np.trapezoid(delta, grid) / (hi-lo) * scale),
                "time_weighted_RMS": float(np.sqrt(np.trapezoid(delta**2, grid) / (hi-lo)) * scale),
                "min": float(delta.min() * scale), "max": float(delta.max() * scale)}
        results.append({"window_s": [lo, hi], "registered_minus_pre_ground": coordinates})
    return {"old_IK": fingerprint(old_path), "times_identical": True,
        "windows": results, "meaning": "Preparation produces an indirect change in dynamic ankle IK through static calibration plus a new marker solver, despite unchanged joint zero and absence of a manual runtime angle offset. This saved comparison changes model mounting and marker positions together; it is not a force-law causal intervention."}


def subject_audit(subject, session, static_trial, selection):
    root = REPO / f"models/{subject}-raw/{session}"
    bundle = REPO / f"models/{subject}_SEASEA_Threadmill"
    pipeline_path = bundle / "pipeline_report.json"
    pipeline = json.loads(pipeline_path.read_text()) if pipeline_path.exists() else None
    converted = REPO / pipeline["converted_trial_dir"] if pipeline else REPO / f"models/{subject}_SEASEA-raw/data/converted/treadmill/treadmill_01_01"
    converted_static = REPO / pipeline["converted_static_dir"] if pipeline else REPO / f"models/{subject}_SEASEA-raw/data/converted/static/{static_trial}"
    paths = {kind: root / f"treadmill/{kind}/treadmill_01_01.mat" for kind in ("markers", "fp", "ik", "conditions")}
    marker_table = converter.decode_mcos_table(paths["markers"])
    marker_raw = converter.extract_marker_data(marker_table)
    force_table = converter.decode_mcos_table(paths["fp"])
    force_raw = converter.extract_grf_data(force_table)
    coordinate_raw = converter.extract_coordinate_data(converter.decode_mcos_table(paths["ik"]))
    trc_path = converted / "treadmill_01_01.trc"
    trc = parse_trc(trc_path)
    assert set(trc.markers) == set(marker_raw.names)
    source_scale = .001 if marker_raw.units == "mm" else 1
    marker_actual = np.stack([trc.markers[n] for n in marker_raw.names], axis=1)
    marker_diff = array_difference(marker_actual, marker_raw.values * source_scale)
    marker_clock_diff = array_difference(trc.times, marker_raw.time)
    grf_path = bundle / f"data/{subject}_SEASEA_GRF_FullSpan.mot"
    grf = parse_storage(grf_path)
    assert tuple(grf.labels[1:]) == force_raw.labels
    grf_diff = array_difference(grf.values[:, 1:], force_raw.values)
    grf_clock_diff = array_difference(grf.times, force_raw.time)
    converted_ik_path = converted / "treadmill_01_01_ik_dataset_ab06_seasea.mot"
    remapped_ik = parse_motion(converted_ik_path)
    assert tuple(remapped_ik.labels[1:]) == coordinate_raw.labels
    remapped_diff = array_difference(remapped_ik.values[:, 1:], coordinate_raw.values)
    remapped_clock_diff = array_difference(remapped_ik.times, coordinate_raw.time)
    ik_path = bundle / f"data/IK_results_{subject}_SEASEA.mot"
    final = parse_motion(ik_path)
    assert np.array_equal(final.times, trc.times)
    model_path = bundle / f"{subject}_SEASEA_stiff321_500_pi.osim"
    mass = sum(float(x.text) for x in ET.parse(model_path).getroot().findall(".//Body/mass"))
    assert abs(mass - selection["model_mass_kg_from_XML"]) < 1e-12
    lo, hi = selection["primary_cycles"][0]["hs_s"], selection["primary_cycles"][-1]["next_hs_s"]
    fy = {side: grf.values[:, grf.labels.index(f"ground_force{index}_vy")] for side, index in (("left", 1), ("right", 2))}
    impulses = {side: exact_integral(grf.times, values, lo, hi) for side, values in fy.items()}
    condition_table, condition_metadata = generic_table(paths["conditions"], b"speed")
    assert condition_table.names == ("Header", "Speed")
    condition_t = condition_table.columns["Header"]; speed = condition_table.columns["Speed"]
    speed_windows = {}
    for name, start, end in (("requested_9s", selection["reset_hs_s"], selection["requested_end_s"]), ("primary", lo, hi)):
        assert condition_t[0] <= start < end <= condition_t[-1]
        mask = (condition_t >= start) & (condition_t <= end)
        speed_windows[name] = stats(speed[mask])
    static_paths = {kind: root / f"static/{kind}/{static_trial}.mat" for kind in ("markers", "ik")}
    static_marker_table, sm_meta = generic_table(static_paths["markers"])
    static_coordinate_table, sq_meta = generic_table(static_paths["ik"])
    static_markers = converter.extract_marker_data(static_marker_table)
    static_coordinates = converter.extract_coordinate_data(static_coordinate_table)
    static_trc_path = converted_static / f"{static_trial}.trc"
    static_ik_path = converted_static / f"{static_trial}_ik_dataset_ab06_seasea.mot"
    static_trc = parse_trc(static_trc_path); static_ik = parse_motion(static_ik_path)
    assert static_trc.times.shape == static_markers.time.shape
    assert static_ik.times.shape == static_coordinates.time.shape
    static_marker_scale = .001 if static_markers.units == "mm" else 1
    static_marker_diff = array_difference(np.stack([static_trc.markers[n] for n in static_markers.names], axis=1), static_markers.values * static_marker_scale)
    static_ik_diff = array_difference(static_ik.values[:, 1:], static_coordinates.values)
    result = {"sources": [fingerprint(q) for q in (*paths.values(), *static_paths.values(), trc_path, grf_path, converted_ik_path, ik_path, model_path, static_trc_path, static_ik_path)],
        "dynamic_clock": {"MAT_markers": clock(marker_raw.time), "TRC": clock(trc.times), "final_IK": clock(final.times), "MAT_fp": clock(force_raw.time), "bundle_GRF": clock(grf.times), "marker_minus_force_first_timestamp_s": float(marker_raw.time[0]-force_raw.time[0])},
        "source_parity": {"all_28_marker_xyz_in_m": marker_diff, "marker_clock_s": marker_clock_diff, "all_18_GRF_columns": grf_diff, "GRF_clock_s": grf_clock_diff, "remapped_dataset_IK": remapped_diff, "remapped_dataset_IK_clock_s": remapped_clock_diff,
            "per_GRF_column_max_abs_difference": {n: float(np.max(np.abs(grf.values[:, i+1]-force_raw.values[:, i]))) for i, n in enumerate(force_raw.labels)}},
        "axis_mapping": {"markers": "Original XYZ preserved; only mm-to-m scaling for this audit.", "force": "Original Treadmill_L XYZ -> ground_force1 XYZ, Treadmill_R -> ground_force2; no rotation or sign change.", "COP": "Original XYZ with shared automatic units scaling; no coordinate rotation.", "torque": "Default free_vertical: M_origin-cross(COP,F), retaining Y and zeroing X/Z. Direct parity verifies the convention actually present.", "external_loads": external_loads(bundle / "data/ExternalForces.xml")},
        "measured_primary_loading": {"window_s": [lo, hi], "mass_kg_XML": mass, "BW_N": mass*9.80665, "impulse_Ns": impulses, "mean_bilateral_Fy_div_BW": sum(impulses.values())/(hi-lo)/(mass*9.80665), "integration_method": "Direct measured GRF native 1 ms source grid with exact nominal boundaries. The paired comparator instead interpolates measurement to the simulator 1 ms grid, with a subject/reset-specific phase offset. These source checks do not replace its saved simulation/measurement impulse ratios."},
        "data_coverage": {name: {"covers_requested_9s": bool(t[0] <= selection["reset_hs_s"] and selection["requested_end_s"] <= t[-1]), "covers_primary": bool(t[0] <= lo and hi <= t[-1])} for name, t in (("IK", final.times), ("GRF", grf.times), ("markers", trc.times))},
        "conditions": {"source_metadata": condition_metadata, "units_from_original_MAT_VariableUnits": dict(zip(condition_table.names, condition_metadata["variable_units_raw"] or [])), "Speed_values_original_units_not_inferred": speed_windows, "interpretation": "Both subjects have constant Speed=0.5 in these windows. The original MAT VariableUnits explicitly identifies Header as s and Speed as m/s."},
        "static_conversion": {"MAT_marker_clock": clock(static_markers.time), "converted_TRC_clock": clock(static_trc.times), "MAT_IK_clock": clock(static_coordinates.time), "converted_remapped_IK_clock": clock(static_ik.times), "rowwise_marker_xyz_difference_m": static_marker_diff, "rowwise_remapped_IK_difference": static_ik_diff, "converted_marker_and_IK_times_identical": bool(np.array_equal(static_trc.times, static_ik.times)), "raw_marker_and_IK_times_identical": bool(np.array_equal(static_markers.time, static_coordinates.time)), "static_MCOS_metadata": {"markers": sm_meta, "IK": sq_meta}},
        "marker_quality_on_primary": marker_quality(subject, selection, trc, model_path, ik_path)}
    if pipeline:
        provenance = pipeline["ik_provenance"]
        checks = {}
        for name, path_key, hash_key in (("computed_IK", "source_file", "source_sha256"), ("solver_model", "model_file", "model_sha256"), ("solver_TRC", "marker_file", "marker_sha256")):
            current = fingerprint(REPO / provenance[path_key])
            checks[name] = {"source": current, "matches_recorded_sha": current["sha256"] == provenance[hash_key]}
        assert all(v["matches_recorded_sha"] for v in checks.values())
        result["IK_provenance"] = {"pipeline": fingerprint(pipeline_path), "method": pipeline["ik_method"], "checks": checks, "bundle_IK_matches_computed_source": checks["computed_IK"]["source"]["sha256"] == fingerprint(ik_path)["sha256"], "bundle_model_matches_solver_model": checks["solver_model"]["source"]["sha256"] == fingerprint(model_path)["sha256"]}
        result["saved_preparation_before_after_IK"] = before_after_ik(selection, final)
    else:
        result["IK_provenance"] = {"status": "Current AB06 bundle lacks pipeline_report.json with source-model/TRC/solver hashes. Historical reports document a genuine marker IK; currently absent original full-span solver outputs/receipt limit exact source attestation.", "historical_reports": [fingerprint(REPO / "reports/user/2026-05-11_ab06_seasea_epic_pipeline.md"), fingerprint(REPO / "reports/daily/2026-05-12_daily-report.md")]}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path(__file__).with_suffix(".json"))
    args = parser.parse_args()
    selection_path = REPO / "Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/selection.json"
    selection = json.loads(selection_path.read_text())
    result = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(), "instruction_check_token": "CMC_AGENT_OK_2026", "scope": __doc__, "selection": fingerprint(selection_path), "subjects": {}}
    for subject, session, static in (("AB06", "10_09_18", "static_01"), ("AB07", "10_14_18", "static01")):
        result["subjects"][subject] = subject_audit(subject, session, static, selection["subjects"][subject])
    source_refs = [source for value in result["subjects"].values() for source in value["sources"]]
    result["source_files_unchanged_at_completion"] = all(fingerprint(REPO/r["path"])["sha256"] == r["sha256"] for r in source_refs)
    assert result["source_files_unchanged_at_completion"]
    result["helper"] = fingerprint(Path(__file__))
    result["production_helpers"] = [fingerprint(REPO/"tools/convert_epic_ab06_tables.py"), fingerprint(REPO/"tools/calibrate_ab06_seasea_markers.py")]
    preliminary = Path(__file__).with_suffix(".json")
    if preliminary.exists() and preliminary.resolve() != args.out.resolve():
        result["previous_preliminary_output"] = {"source": fingerprint(preliminary), "status": "Preserved preliminary valid JSON, preceding explicit decoding of original conditions VariableUnits and clarification of the direct measured-GRF integration grid."}
    result["interpretation"] = ["Dynamic MAT/TRC/GRF parity tests rule out a conversion-introduced axis, side, sign or timestamp error for the checked columns. They do not prove physical correctness of the source acquisition or contact model.", "AB06 historical static times are dilated twofold, but pose and marker rows remain aligned and numerically preserved. Median static ground registration uses poses without velocity/time weighting; the clock discrepancy alone does not demonstrate an effect on paired dynamics.", "The AB07 ankle IK changes indirectly by about +5.10 degrees after preparation. This is a real preparation effect, not proof of an error or unique explanation for the +40% load difference.", "Both requested and primary windows have original Speed=0.5 m/s, with units attested by original MAT VariableUnits, and subject-specific measured forces/body weights. Source data checks alone cannot separate intrinsic anatomy from prosthesis fitting, marker calibration, reference/governor effects or contact geometry.", "The primary saved-pose marker residuals are lower on the AB07 prosthetic side than AB06. A larger marker-fitting residual on AB07 is not supported as the explanation for the measured load discrepancy; small marker residuals do not uniquely validate foot hardware orientation or contact placement."]
    with args.out.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": fingerprint(args.out), "source_files_unchanged": True}, ensure_ascii=False))


if __name__ == "__main__":
    main()
