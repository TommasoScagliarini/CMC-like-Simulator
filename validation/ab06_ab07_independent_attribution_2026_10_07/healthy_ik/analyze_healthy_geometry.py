"""Supplemental saved-data healthy IK comparison; pure FK, no IK/integration.

Reuses the frozen geometry helper and cached original/prepared FK summaries.
Fresh healthy IK is evaluated on exactly the same immutable primary grid and
full-trial filter. Marker fits are reported independently from joint centers.
"""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
AUDIT = HERE.parent
GEOMETRY = AUDIT / "geometry/functional_ankle_geometry.py"
CACHE = GEOMETRY.with_suffix(".json")
PAIR = ROOT / "Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07"
OUTPUT = HERE / "supplemental_healthy_geometry.json"
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location("frozen_functional_ankle_geometry", GEOMETRY)
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)
previous = geometry.previous
from tools.calibrate_ab06_seasea_markers import parse_trc


def canonical(node):
    tokens = (node.text or "").split()
    try:
        tokens = tuple(map(float, tokens))
    except ValueError:
        tokens = tuple(tokens)
    return node.tag, tuple(sorted(node.attrib.items())), tokens, tuple(canonical(n) for n in node)


def weighted_rms(values, t, windows):
    duration = sum(b-a for a, b in windows)
    square = 0.0
    minimum, maximum = float("inf"), float("-inf")
    for a, b in windows:
        x = np.r_[a, t[(t>a) & (t<b)], b]
        y = np.interp(x, t, values)
        square += float(geometry.INTEGRATE(y*y, x))
        minimum, maximum = min(minimum, float(y.min())), max(maximum, float(y.max()))
    return {"rms": float(np.sqrt(square/duration)), "minimum": minimum, "maximum": maximum}


def q_delta(original, fresh, times, cycles, translations):
    if set(original) != set(fresh) or len(fresh) != 23:
        raise ValueError("Healthy full-coordinate source schemas differ")
    windows = [(c["hs_s"], c["to_s"]) for c in cycles]
    rows = {}
    for name in sorted(fresh):
        delta = fresh[name](times)-original[name](times)
        factor, unit = (1000.0, "mm") if name in translations else (180/np.pi, "deg")
        values = delta*factor
        rows[name] = {"unit": unit, "source": "fresh minus original, production filtered, no angular wrap",
                      "stance_mean": geometry.metric(times, {"delta": values[:, None]}, windows)["delta_time_weighted_mean"][0],
                      **weighted_rms(values, times, windows), "max_abs_primary": float(np.max(np.abs(values)))}
    return rows


def native_ik_error_summary(path, cycles):
    t, data, _ = previous.storage(path)
    if set(data) != {"total_squared_error", "marker_error_RMS", "marker_error_max"}:
        raise ValueError("Unexpected native marker error schema")
    rms, maximum = data["marker_error_RMS"], data["marker_error_max"]
    windows = [(c["hs_s"], c["to_s"]) for c in cycles]
    return {"source": geometry.fp(path), "rows": len(t), "range_s": [float(t[0]), float(t[-1])],
            "scope": "Native tool raw full-marker IK residuals on 28 original markers; no production smoothing applied",
            "unit": "mm", "fulltrial_mean_frame_RMS_mm": float(geometry.INTEGRATE(rms, t)/(t[-1]-t[0])*1000),
            "fulltrial_max_frame_RMS_mm": float(rms.max()*1000),
            "fulltrial_max_individual_marker_error_mm": float(maximum.max()*1000),
            "primary_stance_mean_frame_RMS_mm": geometry.metric(t, {"rms": (rms*1000)[:, None]}, windows)["rms_time_weighted_mean"][0]}


def filtered_marker_fit(model_path, original, fresh, times, cycles, trc_path):
    """Same raw TRC targets for both full-trial-filtered healthy solutions."""
    trc = parse_trc(trc_path)  # Parser converts mm to m exactly once.
    previous._configure_geometry_search_paths(str(model_path))
    model = previous.opensim.Model(str(model_path))
    state = model.initSystem()
    coordinates = [model.getCoordinateSet().get(i) for i in range(model.getCoordinateSet().getSize())]
    markers = [model.getMarkerSet().get(i) for i in range(model.getMarkerSet().getSize())]
    names = [m.getName() for m in markers]
    if len(names) != 28 or set(names) != set(trc.markers):
        raise ValueError("Healthy marker/TRC names differ")
    if times[0] < trc.times[0] or times[-1] > trc.times[-1]:
        raise ValueError("Raw marker support does not cover frozen window")
    measured = np.stack([np.column_stack([np.interp(times, trc.times, trc.markers[n][:, a]) for a in range(3)]) for n in names], axis=1)
    if not np.all(np.isfinite(measured)):
        raise ValueError("Nonfinite raw marker observation")
    results = {}
    for key, splines in (("healthy_original", original), ("healthy_fresh", fresh)):
        errors = np.empty_like(measured)
        assignment = 0.0
        for i, t in enumerate(times):
            state.setTime(float(t))
            for coordinate in coordinates:
                value = float(splines[coordinate.getName()](t))
                coordinate.setValue(state, value, False)
                assignment = max(assignment, abs(coordinate.getValue(state)-value))
            model.realizePosition(state)
            for j, marker in enumerate(markers):
                errors[i, j] = geometry.vector(marker.getLocationInGround(state))-measured[i, j]
        if assignment != 0.0 or not np.all(np.isfinite(errors)):
            raise AssertionError("Filtered marker FK assignment/finite check failed")
        rms = np.sqrt(np.mean(np.sum(errors*errors, axis=2), axis=1))*1000
        maximum = np.sqrt(np.sum(errors*errors, axis=2)).max(axis=1)*1000
        summary = {}
        for phase, windows in (("stance", [(c["hs_s"], c["to_s"]) for c in cycles]),
                               ("swing", [(c["to_s"], c["next_hs_s"]) for c in cycles])):
            summary[phase] = {"mean_frame_RMS_mm": geometry.metric(times, {"rms": rms[:, None]}, windows)["rms_time_weighted_mean"][0],
                              "max_frame_RMS_mm": weighted_rms(rms, times, windows)["maximum"],
                              "max_individual_marker_error_mm": weighted_rms(maximum, times, windows)["maximum"]}
        results[key] = {"all_28_markers": names, "primary_grid_samples": len(times), "assignment_max_abs_error": assignment, **summary}
    results["scope"] = "Comparable healthy original/fresh filtered poses versus same unfiltered measured TRC, linearly interpolated at saved native1ms plus exact frozen HS/TO/nextHS; these differ from native raw-IK fulltrial tool residuals."
    return results


def main():
    if OUTPUT.exists():
        raise FileExistsError("Preserve sealed supplemental output")
    cached = json.loads(CACHE.read_text())
    selection = json.loads((PAIR / "selection.json").read_text())
    protocol = json.loads((HERE / "protocol.json").read_text())
    sources = {Path(__file__), GEOMETRY, CACHE, HERE / "protocol.json", PAIR / "selection.json",
               geometry.REUSED, ROOT / "config.py", ROOT / "kinematics_interpolator.py", ROOT / "path_resolver.py",
               ROOT / "model_loader.py", ROOT / "tools/convert_epic_ab06_tables.py", ROOT / "tools/calibrate_ab06_seasea_markers.py"}
    sources.update(ROOT / p for p in cached["inputs_integrity"]["fingerprints_before"])
    inputs = {}
    for subject in ("AB06", "AB07"):
        receipt_path = HERE / subject / "receipt.json"
        receipt = json.loads(receipt_path.read_text())
        if receipt["status"] != "completed" or not receipt["source_files_unchanged"] or receipt["opensim_cmd"] is not None:
            raise ValueError("Healthy IK receipt completion/backend/protection contract differs")
        fresh = ROOT / receipt["output_motion"]["path"]
        if geometry.fp(fresh)["sha256"] != receipt["output_motion"]["sha256"]:
            raise AssertionError("Fresh healthy output hash differs from completed receipt")
        p = protocol["subjects"][subject]
        healthy, trc = ROOT / p["model"], ROOT / p["trc"]
        bundle = AUDIT / "rebuild/models" / f"{subject}_SEASEA_Threadmill"
        prepared = bundle / f"{subject}_SEASEA_stiff321_500_pi.osim"
        prepared_ik = bundle / "data" / f"IK_results_{subject}_SEASEA.mot"
        old_prepared = ROOT / selection["subjects"][subject]["source_fingerprints"]["model"]["path"]
        if canonical(ET.parse(prepared).find(".//JointSet")) != canonical(ET.parse(old_prepared).find(".//JointSet")):
            raise AssertionError("Cannot reuse prepared FK: joint geometry differs")
        if geometry.fp(prepared_ik) != cached["subjects"][subject]["prepared_IK"]:
            if geometry.fp(prepared_ik)["sha256"] != cached["subjects"][subject]["prepared_IK"]["sha256"]:
                raise AssertionError("Cannot reuse prepared FK: full IK bytes differ")
        fresh_errors = next((HERE / subject / "02_ik/full").glob("*_ik_marker_errors.sto"))
        prepared_errors = next((AUDIT / "rebuild/results" / subject / f"{subject.lower()}_seasea_treadmill_treadmill_01_01_ik/02_ik/full").glob("*_ik_marker_errors.sto"))
        sources.update([receipt_path, fresh, healthy, trc, prepared, prepared_ik, fresh_errors, prepared_errors])
        inputs[subject] = dict(receipt=receipt, fresh=fresh, healthy=healthy, trc=trc, prepared=prepared,
                               fresh_errors=fresh_errors, prepared_errors=prepared_errors)
    before = {geometry.fp(p)["path"]: geometry.fp(p)["sha256"] for p in sorted(sources)}
    # Frozen original geometry summaries are reused only with their entire
    # original source integrity registry still matching current source bytes.
    for p, sha in cached["inputs_integrity"]["fingerprints_before"].items():
        if before[p] != sha:
            raise AssertionError(f"Cached geometry source changed: {p}")
    previous._load_plugin(str(ROOT / "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"))
    subjects = {}
    for subject, paths in inputs.items():
        frozen = selection["subjects"][subject]
        run = ROOT / f"results/{subject}_paired_prescribed_HS_9s_2026_10_07"
        cfgdict = dict(json.loads((run / "receipt.json").read_text())["effective_simulator_config"])
        cfgdict.update(model_bundle_dir=str(paths["healthy"].parent), model_file=str(paths["healthy"]), kinematics_file=str(paths["fresh"]),
                       external_loads_xml=str(ROOT / frozen["source_fingerprints"]["external_loads"]["path"]),
                       reserve_actuators_xml=str(ROOT / frozen["source_fingerprints"]["reserve_actuators"]["path"]))
        cfg = previous.SimulatorConfig(**cfgdict)
        if (cfg.enable_kinematics_lowpass_filter, cfg.kinematics_lowpass_cutoff_hz, cfg.kinematics_lowpass_order, cfg.kinematics_resample_dt) != (True, 6., 4, .001):
            raise AssertionError("Frozen fulltrial preprocessing contract changed")
        ft, fq, header = previous.storage(paths["fresh"])
        if "indegrees=yes" not in header.lower() or len(fq) != 23:
            raise ValueError("Fresh healthy IK requires explicit degrees and all23 coordinates")
        fresh_splines = previous.KinematicsInterpolator(cfg)._splines
        day = "10_09_18" if subject == "AB06" else "10_14_18"
        original_splines = previous.original_splines(ROOT / f"models/{subject}-raw/{day}/treadmill/ik/treadmill_01_01.mat", cfg)
        native_times, _, _ = previous.storage(run / "sim_outputs/sim_output_states.sto")
        cycles = frozen["primary_cycles"]
        a, b = cycles[0]["hs_s"], cycles[-1]["next_hs_s"]
        times = np.unique(np.r_[native_times[(native_times>a)&(native_times<b)], [c[k] for c in cycles for k in ("hs_s", "to_s", "next_hs_s")]])
        if len(times) != cached["subjects"][subject]["grid_samples"]:
            raise AssertionError("Saved primary lattice differs from cached geometry")
        print(subject, "fresh healthy Velocity FK", len(times), "poses", flush=True)
        contract, arrays, validation = geometry.fk(paths["healthy"], fresh_splines, times, "healthy_original")
        fresh_summary = dict(contract=contract, validation=validation, **geometry.summarize(times, arrays, cycles))
        cases = deepcopy(cached["subjects"][subject]["cases"])
        cases["healthy_fresh"] = fresh_summary
        cases["healthy_original"]["cached_fk_source"] = geometry.fp(CACHE)
        cases["prepared_current"]["cached_fk_source"] = geometry.fp(CACHE)
        center_delta = {}
        for phase in ("stance", "swing", "all_primary"):
            center_delta[phase] = {"fresh_minus_original_joint_center_Y_mm": 1000*(fresh_summary["aggregate"][phase]["joint_center_Y_time_weighted_mean_m"]-cases["healthy_original"]["aggregate"][phase]["joint_center_Y_time_weighted_mean_m"])}
        subjects[subject] = {"cases": cases, "healthy_IK_receipt": geometry.fp(HERE / subject / "receipt.json"),
            "original_to_fresh_center_delta": center_delta, "filtered_primary_coordinate_deltas": q_delta(original_splines, fresh_splines, times, cycles, cfg.translation_coords),
            "native_full_marker_ik_errors": {"healthy_fresh": native_ik_error_summary(paths["fresh_errors"], cycles),
                                             "common_prepared": native_ik_error_summary(paths["prepared_errors"], cycles)},
            "filtered_healthy_marker_comparison": filtered_marker_fit(paths["healthy"], original_splines, fresh_splines, times, cycles, paths["trc"]),
            "reused_common_prepared_geometry": {"new_model": geometry.fp(paths["prepared"]), "JointSet_numeric_semantic_equal_to_cached_model": True, "full_IK_byte_equal_to_cached": True},
            "primary_samples": len(times), "fresh_fulltime_range_s": [float(ft[0]), float(ft[-1])], "fresh_output": geometry.fp(paths["fresh"])}
    budgets = {}
    case = lambda s, name, phase: subjects[s]["cases"][name]["aggregate"][phase]
    for phase in ("stance", "swing", "all_primary"):
        budgets[phase] = {"dataset_healthy": geometry.decomposition(case("AB06", "healthy_original", phase), case("AB06", "prepared_current", phase), case("AB07", "healthy_original", phase), case("AB07", "prepared_current", phase)),
                          "fresh_healthy": geometry.decomposition(case("AB06", "healthy_fresh", phase), case("AB06", "prepared_current", phase), case("AB07", "healthy_fresh", phase), case("AB07", "prepared_current", phase))}
    phase_bins = []
    for i in range(20):
        get = lambda s, name: subjects[s]["cases"][name]["pooled_stance_phase_bins"][i]
        phase_bins.append({"phase_start": i/20, "phase_end": (i+1)/20,
            "fresh_healthy_budget": geometry.decomposition(get("AB06", "healthy_fresh"), get("AB06", "prepared_current"), get("AB07", "healthy_fresh"), get("AB07", "prepared_current"))})
    after = {geometry.fp(p)["path"]: geometry.fp(p)["sha256"] for p in sorted(sources)}
    if before != after:
        raise AssertionError("Protected source changed during supplemental FK")
    result = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(), "branch": "Codex", "status": "PASS",
              "helper": geometry.fp(Path(__file__)), "subjects": subjects, "subject_difference_budgets": budgets,
              "fresh_healthy_stance_phase_budgets": phase_bins,
              "methodology": {"fresh_healthy_solver": "Same healthy23model/28 original markers, OpenSim Python API and native tool flags as rebuilt preparation, no ScaleTool/remap/calibration/ID/RRA/dynamics.",
                              "functional_point": "Ankle parent joint socket origin, not sole/marker; same healthy model for original-to-fresh solution comparison.",
                              "geometry_grid_filter": "Recorded full-trial zero-phase6Hz/order4/resample1ms; saved primary native1ms grid plus exact frozen HS/TO/nextHS; same3 cycles/percycle/20 stance bins as frozen geometry helper.",
                              "cache_reuse": "Original healthy and production-prepared FK summaries reused with all cached source hashes verified; common rebuilt JointSet semantically exact and full IK byte-identical.",
                              "interpretation": "Difference of IK solutions under observed markers/tool contract; healthy B-A combines subject anatomy/gait/marker/acquisition and solver conventions. No anatomy causal share, hardware-ground validation or dynamic causal conclusion."},
              "inputs_integrity": {"protected_count": len(before), "unchanged": True, "fingerprints_before": before, "fingerprints_after": after}}
    with OUTPUT.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    print("PASS", geometry.fp(OUTPUT))
    for phase, budget in budgets.items():
        print(phase, {key: value["joint_center_Y_budget_mm"] for key, value in budget.items()})


if __name__ == "__main__":
    main()
