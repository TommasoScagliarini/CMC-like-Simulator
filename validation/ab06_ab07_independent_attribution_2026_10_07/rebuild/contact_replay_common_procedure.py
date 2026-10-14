"""FK contact comparison of coherent, complete common-procedure bundles.

Requires --builds-complete after both builds have been confirmed complete.
Old cases reuse reviewed saved FK arrays, not mixed model/motion cells. New
cases use each new calibrated model and its own complete marker-IK output.
No dynamics, assembly, solver, model serialization, inference or input edits.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import platform
import sys
import xml.etree.ElementTree as ET

import numpy as np

AUDIT = Path(__file__).resolve().parent
ROOT = AUDIT.parents[2]
PAIR = ROOT / "Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07"
PREVIOUS = ROOT / "validation/ab06_ab07_preparation_vs_model_2026_10_07"
SAVED = ROOT / "validation/ab06_ab07_causal_investigation_2026_10_07/model"
PHASE = SAVED.parent / "alternative/phase_contact_budget.py"
INTEGRATE = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
sys.path.insert(0, str(ROOT))


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


cr = load_module("common_procedure_contact_law", PREVIOUS / "contact_replay.py")
xml = load_module("common_procedure_xml", PREVIOUS / "compare_model_xml.py")
phase = load_module("common_procedure_phase", PHASE)
rigid = load_module("common_procedure_rigid", SAVED / "foot_pose_decomposition.py")


def fp(path):
    path = Path(path).resolve()
    return {"path": path.relative_to(ROOT).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def read_json(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))


def repo_path(raw):
    p = Path(raw)
    p = p if p.is_absolute() else ROOT / p
    p = p.resolve()
    p.relative_to(ROOT)
    if not p.is_file():
        raise FileNotFoundError(p)
    return p


def equal(a, b):
    if isinstance(a, (float, int)) and isinstance(b, (float, int)):
        return math.isclose(a, b, rel_tol=1e-14, abs_tol=1e-14)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(equal(x, y) for x, y in zip(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(equal(a[k], b[k]) for k in a)
    return a == b


def xml_model(path):
    return ET.parse(path).find("Model")


def model_contract(healthy_path, donor_path, old_path, new_path):
    healthy, donor, old, new = map(xml_model, (healthy_path, donor_path, old_path, new_path))
    hb, nb, db = (xml.objects(m, "BodySet") for m in (healthy, new, donor))
    retained = sorted(set(hb) & set(nb))
    bodies = {n: equal(xml.flat(hb[n], physical=True), xml.flat(nb[n], physical=True)) for n in retained}
    hj, nj, dj = (xml.objects(m, "JointSet") for m in (healthy, new, donor))
    hip_ok = equal(xml.flat(xml.adjusted_hip(healthy)), xml.flat(nj["hip_l"]))
    common_joints = {n: equal(xml.flat(hj[n]), xml.flat(nj[n])) for n in sorted(hj.keys() & nj.keys()) if n != "hip_l"}
    hc, nc = xml.coordinates(healthy), xml.coordinates(new)
    common_coords = {n: equal(hc[n], nc[n]) for n in sorted(hc.keys() & nc.keys())}
    hardware, mounts = {}, {}
    for name in ("osseo", "pros_knee", "pros_ankle"):
        actual, source = deepcopy(nj[name]), deepcopy(dj[name])
        mounts[name] = xml.joint_frames(nj[name])
        if name in ("osseo", "pros_ankle"):
            for joint in (actual, source):
                frame_name = joint.findtext("socket_parent_frame")
                frame = next(f for f in joint.findall("frames/PhysicalOffsetFrame") if f.get("name") == frame_name)
                values = [float(x) for x in frame.findtext("translation").split()]
                if not all(math.isfinite(x) for x in values):
                    raise ValueError("Nonfinite mount")
                values[1] = 0.
                frame.find("translation").text = " ".join(map(repr, values))
        hardware[name] = equal(xml.flat(actual), xml.flat(source))
    nf, df = xml.objects(new, "ForceSet"), xml.objects(donor, "ForceSet")
    sea_names = sorted(n for n, f in nf.items() if f.tag == "SeriesElasticActuator")
    sea = {n: {"all_properties_semantically_equal_to_frozen_donor": equal(xml.flat(nf[n]), xml.flat(df[n])),
               "raw_float_token_differences": xml.differences(xml.flat(df[n]), xml.flat(nf[n]))} for n in sea_names}
    ankle = nj["pros_ankle"]
    child = next(f for f in ankle.findall("frames/PhysicalOffsetFrame") if f.get("name") == ankle.findtext("socket_child_frame"))
    child_zero = [float(x) for x in child.findtext("translation").split()] == [0., 0., 0.] and child.findtext("socket_parent") == "/bodyset/foot_l"
    healthy_mass = sum(float(b.findtext("mass")) for b in hb.values())
    mass = sum(float(b.findtext("mass")) for b in nb.values())
    checks = {"seven_retained_healthy_bodies": len(retained) == 7 and all(bodies.values()),
              "healthy_hip_geometry_retained_after_socket_rename": hip_ok,
              "common_healthy_joints_retained": all(common_joints.values()),
              "common_healthy_coordinate_definitions_retained": all(common_coords.values()),
              "hardware_equal_except_allowed_two_mount_Y_values": all(hardware.values()),
              "two_SEA_numeric_contracts_equal": len(sea) == 2 and all(v["all_properties_semantically_equal_to_frozen_donor"] for v in sea.values()),
              "pros_ankle_child_origin_equals_foot_l_origin": child_zero,
              "healthy_total_mass_retained": math.isclose(healthy_mass, mass, rel_tol=1e-12, abs_tol=1e-12),
              "complete_prepared_coordinate_count21": len(nc) == 21}
    if not all(checks.values()):
        raise AssertionError({k: v for k, v in checks.items() if not v})
    return {"status": "PASS", "checks": checks, "retained_bodies": bodies,
            "common_healthy_joints": common_joints, "common_healthy_coordinates": common_coords,
            "coordinate_names": list(nc), "healthy_mass_kg": healthy_mass, "new_mass_kg": mass,
            "hardware_checks": hardware, "new_joint_frames": mounts, "SEA": sea,
            "numeric_comparison_tolerance": {"rel": 1e-14, "abs": 1e-14, "purpose": "XML serialization tolerance, not a scientific qualification gate"},
            "old_to_new_physical_XML_differences": xml.differences(xml.flat(old, physical=True, omit_markers=True), xml.flat(new, physical=True, omit_markers=True)),
            "old_to_new_marker_XML_differences": xml.differences(xml.flat(old.find("MarkerSet")), xml.flat(new.find("MarkerSet")))}


def complete_bundle(subject, models_dir):
    bundle = models_dir / f"{subject}_SEASEA_Threadmill"
    path = bundle / "pipeline_report.json"
    report = read_json(path)
    if report["status"] != "complete" or report["subject"] != subject or report["ik_method"] != "opensim_marker_ik":
        raise ValueError("Both finalized full marker-IK bundles are required")
    setup = repo_path(report["setup_xml"])
    config = ET.parse(setup).getroot()
    model = repo_path(config.findtext(".//model_file"))
    ik = repo_path(config.findtext(".//kinematics_file"))
    external = repo_path(config.findtext(".//external_loads_xml"))
    if model.parent != bundle.resolve() or ik.parent != (bundle / "data").resolve():
        raise ValueError("Bundle model/IK resolve outside this coherent bundle")
    provenance = report["ik_provenance"]
    sources = {path, setup, model, ik, external}
    binds = {}
    for role in ("source", "model", "marker"):
        p = repo_path(provenance[role + "_file"])
        if fp(p)["sha256"] != provenance[role + "_sha256"]:
            raise AssertionError("Full IK source/model/marker binding changed")
        sources.add(p)
        binds[role] = fp(p)
    if fp(ik)["sha256"] != binds["source"]["sha256"] or fp(model)["sha256"] != binds["model"]["sha256"]:
        raise AssertionError("Bundle is not the exact model/IK pair bound in the solver provenance")
    for k in ("healthy_model", "donor_model", "graft_model", "anatomical_model", "calibrated_model", "static_calibration_motion", "grf_source"):
        sources.add(repo_path(report[k]))
    reference_binding = {}
    refs = report["preparation_references"]
    sources.add(repo_path(refs["manifest"]))
    for role, record in refs["inputs"].items():
        original, snapshot = repo_path(record["source_file"]), repo_path(record["snapshot_file"])
        if fp(original)["sha256"] != record["sha256"] or fp(snapshot)["sha256"] != record["sha256"]:
            raise AssertionError("Frozen preparation reference snapshot identity differs")
        sources.update([original, snapshot])
        reference_binding[role] = record
    for v in report["ground_registration"]["sources"].values():
        if isinstance(v, dict) and "path" in v:
            p = repo_path(v["path"])
            if fp(p)["sha256"] != v["sha256"]:
                raise AssertionError("Ground registration input/source hash differs")
            sources.add(p)
        elif isinstance(v, str):
            sources.add(repo_path(v))
    grf = bundle / "data" / f"{subject}_SEASEA_GRF_FullSpan.mot"
    sources.add(grf)
    t, cols, header = cr.read_storage(ik)
    if "indegrees=yes" not in header.lower() or len(cols) != 21:
        raise AssertionError("Full 21-coordinate degree-IK contract differs")
    actual_setup = repo_path(provenance["source_file"]).with_name(repo_path(provenance["source_file"]).stem + "_setup.xml")
    actual_xml = ET.parse(actual_setup)
    actual_model = repo_path(actual_xml.findtext(".//model_file"))
    actual_markers = repo_path(actual_xml.findtext(".//marker_file"))
    if fp(actual_model)["sha256"] != binds["model"]["sha256"] or fp(actual_markers)["sha256"] != binds["marker"]["sha256"]:
        raise AssertionError("Actual IK setup model/marker copies do not equal source-bound files")
    actual_log = actual_setup.parent / "ik_full.log"
    command_log = (ROOT / report["pipeline_root"]).resolve().parent / (report["subject"].lower() + "_seasea_treadmill_treadmill_01_01") / "logs/opensim_ik_pipeline.log"
    command = command_log.read_text().splitlines()[0]
    if not command.startswith("$ " + sys.executable + " ") or "--ik-only" not in command or "--skip-smoke-test" not in command:
        raise AssertionError("Replay Python must match the completed full-IK pipeline interpreter")
    if "--opensim-cmd" in command:
        raise ValueError("Native CLI backend needs an explicit version parity review before this helper")
    actual_text = actual_log.read_text()
    if str(actual_model) not in actual_text or "Loaded library " + str(repo_path(report["plugin"])) not in actual_text:
        raise AssertionError("Actual IK log does not attest the bound model/plugin")
    sources.update([actual_setup, actual_model, actual_markers, actual_log, command_log])
    requested = report["kinematics_time_range"]
    if [float(v) for v in actual_xml.findtext(".//time_range").split()] != requested:
        raise AssertionError("Actual IK setup interval differs")
    if max(abs(t[0] - requested[0]), abs(t[-1] - requested[1])) > 1e-7 or provenance["time_range"] != requested:
        raise AssertionError("Full trial IK support differs from solver/builder range")
    return {"bundle": bundle, "report_path": path, "report": report, "model": model, "IK": ik,
            "external": external, "grf": grf, "source_paths": sources,
            "binding": {"status": "PASS", "solver": provenance, "bundle_model": fp(model), "bundle_IK": fp(ik),
                        "marker": binds["marker"], "raw_IK_time_range_s": [float(t[0]), float(t[-1])],
                        "raw_IK_samples": len(t), "raw_coordinate_names": list(cols),
                        "exact_model_and_IK_source_bytes_bound": True,
                        "frozen_preparation_reference_snapshots": reference_binding,
                        "actual_solver_setup": fp(actual_setup), "actual_solver_model_copy": fp(actual_model),
                        "actual_solver_marker_copy": fp(actual_markers), "actual_solver_log": fp(actual_log),
                        "pipeline_command_log": fp(command_log), "actual_pipeline_command": command,
                        "backend": "Python OpenSim tool runner; actual pipeline interpreter equals current sys.executable; no native CLI override"}}


def summarize(times, values, measured, selected, bw, names):
    total = values["force"][..., 1].sum(axis=1)
    i07 = names.index("left_basis_07")
    series = {"foot_ankle_center_Y": values["foot_origin_y"],
              "basis07_positive_penetration": values["penetration"][:, i07],
              "maximum_positive_penetration": values["penetration"].max(axis=1)}
    result = phase.budget(times, total, measured, selected["primary_cycles"], bw, series, values["force"][:, i07, 1])
    result["contact_summary"] = cr.summarize(times, values, measured, selected["primary_cycles"], bw, names)
    result["aggregate"]["mean_force_BW"] = result["aggregate"]["mean_force_N"] / bw
    return result


def saved_case(subject, selected, times, law, hull, binary):
    p = SAVED / f"baseline_{subject}.npz"
    with np.load(p, allow_pickle=False) as z:
        a = {k: z[k].copy() for k in z.files}
    if not np.array_equal(a["times_s"], times) or a["sphere_names"].tolist() != [s.name for s in law.profile.spheres]:
        raise AssertionError("Frozen saved FK grid/sphere order differs")
    for k in ("times_s", "q", "qdot", "contact_centers_m", "contact_velocity_m_s", "aggregate_force_n"):
        if not np.isfinite(a[k]).all():
            raise ValueError("Saved baseline contains nonfinite data")
    values = law.evaluate(a["contact_centers_m"], a["contact_velocity_m_s"])
    force_error = float(np.max(np.abs(values["force"].sum(axis=1) - a["aggregate_force_n"])))
    if force_error > 1e-10:
        raise AssertionError("Baseline frozen law parity differs")
    rotation, origin, rigid_check = rigid.recover(law.locations, a["contact_centers_m"])
    world = np.einsum("tij,nj->tni", rotation, hull) + origin[:, None, :]
    locations = np.array([p["location"] for p in binary["points"]])
    binary_world = np.einsum("tij,nj->tni", rotation, locations) + origin[:, None, :]
    bn = np.array(binary["ground"]["normal"], dtype=float); bn /= np.linalg.norm(bn)
    values.update(centers=a["contact_centers_m"], velocity=a["contact_velocity_m_s"],
                  foot_origin_y=origin[:, 1], pitch=np.arctan2(rotation[:, 1, 0], np.hypot(rotation[:, 0, 0], rotation[:, 2, 0])),
                  mesh_sole_clearance=((world-law.origin) @ law.normal).min(axis=1),
                  binary_clearance=(binary_world - np.array(binary["ground"]["origin"])) @ bn)
    return values, a, {"status": "PASS", "method": "Reuse protected own-model filtered-IK FK export; no baseline FK rerun", "force_max_abs_error_N": force_error, "rigid_recovery": rigid_check}


def change(old, new):
    result = {"aggregate": {}, "cycles": [], "pooled_stance_phase_bins": []}
    for key in ("mean_force_N", "mean_force_BW", "total_impulse_Ns", "stance_impulse_Ns", "swing_impulse_Ns", "swing_fraction_of_total_impulse"):
        result["aggregate"][key + "_new_minus_old"] = new["aggregate"][key]-old["aggregate"][key]
    result["aggregate"]["total_impulse_change_percent"] = 100*(new["aggregate"]["total_impulse_Ns"]/old["aggregate"]["total_impulse_Ns"]-1)
    result["aggregate"]["basis07_impulse_new_minus_old_Ns"] = new["aggregate"]["basis07"]["total_impulse_Ns"]-old["aggregate"]["basis07"]["total_impulse_Ns"]
    for name in old["aggregate"]["depth"]:
        result["aggregate"][name+"_mean_stance_new_minus_old_m"] = new["aggregate"]["depth"][name]["mean_stance_m"]-old["aggregate"]["depth"][name]["mean_stance_m"]
        result["aggregate"][name+"_mean_swing_new_minus_old_m"] = new["aggregate"]["depth"][name]["mean_swing_m"]-old["aggregate"]["depth"][name]["mean_swing_m"]
    for a,b in zip(old["cycles"],new["cycles"]):
        result["cycles"].append({"hs_s":a["hs_s"], "to_s":a["to_s"], "next_hs_s":a["next_hs_s"],
                                 **{k+"_new_minus_old":b[k]-a[k] for k in ("total_impulse_Ns","stance_impulse_Ns","swing_impulse_Ns","stance_mean_force_N")}})
    for a,b in zip(old["pooled_stance_phase_bins"],new["pooled_stance_phase_bins"]):
        keys = ("force_mean_N","force_mean_BW","foot_ankle_center_Y_mean_m","basis07_positive_penetration_mean_m","maximum_positive_penetration_mean_m")
        result["pooled_stance_phase_bins"].append({"phase_start":a["phase_start"], "phase_end":a["phase_end"], **{k+"_new_minus_old":b[k]-a[k] for k in keys}})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--builds-complete", action="store_true", help="Required explicit authorization after both complete builds")
    parser.add_argument("--models-dir", type=Path, default=AUDIT/"models")
    parser.add_argument("--output", type=Path, default=AUDIT/"contact_replay_common_procedure.json")
    args = parser.parse_args()
    if not args.builds_complete:
        parser.error("Wait for parent confirmation that BOTH full marker-IK builds are complete")
    models_dir, output = args.models_dir.resolve(), args.output.resolve()
    models_dir.relative_to(AUDIT)
    if output.parent != AUDIT or output.exists():
        raise FileExistsError("Only a new audit JSON is permitted")
    protocol = read_json(AUDIT/"protocol.json")
    protected = protocol["inputs_before"]
    if len(protected) != protocol["protected_count"]:
        raise AssertionError("Protected manifest count mismatch")
    for raw, sha in protected.items():
        if fp(ROOT/raw)["sha256"] != sha:
            raise AssertionError("An original protected input changed before replay: " + raw)
    bundles = {s: complete_bundle(s, models_dir) for s in ("AB06","AB07")}
    ref_roles = ("donor_model", "healthy_model", "static_motion")
    if any(bundles["AB06"]["binding"]["frozen_preparation_reference_snapshots"][r]["sha256"] != bundles["AB07"]["binding"]["frozen_preparation_reference_snapshots"][r]["sha256"] for r in ref_roles):
        raise AssertionError("The two builds did not use identical frozen donor/healthy/static references")
    selection = read_json(PAIR/"selection.json")
    old_replay = read_json(PREVIOUS/"contact_replay.json")
    factorial = read_json(SAVED/"factorial_geometry_fk.json")
    paths = {ROOT/p for p in protected}
    paths.update([Path(__file__), AUDIT/"protocol.json", PAIR/"selection.json", PREVIOUS/"contact_replay.py", PREVIOUS/"contact_replay.json", PREVIOUS/"compare_model_xml.py", PHASE, SAVED/"foot_pose_decomposition.py", SAVED/"factorial_geometry_fk.json", cr.PRIMARY,cr.BINARY,cr.MESH])
    for s,b in bundles.items():
        paths.update(b["source_paths"])
        paths.add(SAVED/f"baseline_{s}.npz")
        paths.add(ROOT/f"results/{s}_paired_prescribed_HS_9s_2026_10_07/receipt.json")
        paths.add(ROOT/f"results/{s}_paired_prescribed_HS_9s_2026_10_07/sim_outputs/sim_output_states.sto")
        paths.update(ROOT/v["path"] for v in selection["subjects"][s]["source_fingerprints"].values())
    before = {fp(p)["path"]:fp(p)["sha256"] for p in sorted(paths)}
    raw = read_json(cr.PRIMARY)
    profile = cr.OnlineGRFProfile.from_mapping(dict(raw,spheres=[s for s in raw["spheres"] if s["side"]=="left"]),required_sides=("left",))
    law = cr.ContactLaw(profile)
    if not np.array_equal(law.normal,[0.,1.,0.]) or len(profile.spheres)!=8 or any(s.frame!="/bodyset/foot_l" for s in profile.spheres):
        raise AssertionError("Frozen eight-sphere vertical contact contract differs")
    hull,_ = cr.mesh_hull_vertices();binary=read_json(cr.BINARY)
    cr._load_plugin(str(ROOT/"plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"))
    cr._load_plugin(str(ROOT/"plugins/OnlineGRFContact"))
    subjects={}
    for subject,bundle in bundles.items():
        print(f"COMMON PROCEDURE {subject}: complete coherent new bundle, FK only",flush=True)
        selected=selection["subjects"][subject];cycles=selected["primary_cycles"]
        start,end=cycles[0]["hs_s"],cycles[-1]["next_hs_s"]
        run=ROOT/f"results/{subject}_paired_prescribed_HS_9s_2026_10_07"
        receipt=read_json(run/"receipt.json")
        for field,p in (("online_grf_profile_file",cr.PRIMARY),("binary_phase_detector_profile_file",cr.BINARY)):
            if receipt["profile_association"][field]["sha256"]!=fp(p)["sha256"]:
                raise AssertionError("Fixed original receipt profile changed")
        native,_,_=cr.read_storage(run/"sim_outputs/sim_output_states.sto")
        knots=np.unique([c[k] for c in cycles for k in ("hs_s","to_s","next_hs_s")])
        times=np.unique(np.r_[native[(native>start)&(native<end)],knots])
        if len(knots)!=7 or times[0]!=start or times[-1]!=end:
            raise AssertionError("Frozen native grid and seven exact event boundaries differ")
        mt,mc,_=cr.read_storage(ROOT/selected["source_fingerprints"]["measured_GRF"]["path"])
        if start<mt[0] or end>mt[-1]: raise ValueError("Measured GRF support incomplete")
        measured=np.interp(times,mt,mc[selected["left_force_column"]])
        ngt,ngc,_=cr.read_storage(bundle["grf"])
        if not np.array_equal(mt,ngt) or set(mc)!=set(ngc) or any(not np.array_equal(mc[k],ngc[k]) for k in mc):
            raise AssertionError("Rebuild changed original converted measured GRF clock/values")
        old_model=ROOT/selected["source_fingerprints"]["model"]["path"]
        contract=model_contract(repo_path(bundle["report"]["healthy_model"]),repo_path(bundle["report"]["donor_model"]),old_model,bundle["model"])
        cr._configure_geometry_search_paths(str(bundle["model"]))
        model=cr.opensim.Model(str(bundle["model"]))
        if model.getConstraintSet().getSize(): raise ValueError("Unconstrained explicit FK required")
        cr.add_online_grf_forces(model,profile,applies_force=False,name_prefix="contact_replay_")
        state=model.initSystem();coords=[model.getCoordinateSet().get(i) for i in range(model.getCoordinateSet().getSize())]
        names=[c.getName() for c in coords]
        if set(names)!=set(bundle["binding"]["raw_coordinate_names"]) or len(names)!=21:
            raise AssertionError("Full model/IK coordinates differ; no default fill permitted")
        mass=float(model.getTotalMass(state));bw=selected["BW_normalization_n"]
        if abs(mass-selected["model_mass_kg_from_XML"])>1e-10:
            raise AssertionError("Mass/BW convention changed")
        cfg_data=dict(receipt["effective_simulator_config"])
        required={"enable_kinematics_lowpass_filter":True,"kinematics_lowpass_cutoff_hz":6.,"kinematics_lowpass_order":4,"kinematics_resample_dt":.001}
        if any(cfg_data[k]!=v for k,v in required.items()): raise AssertionError("Recorded full-trial filtering contract differs")
        cfg_data.update(model_bundle_dir=str(bundle["bundle"]),model_file=str(bundle["model"]),kinematics_file=str(bundle["IK"]),external_loads_xml=str(bundle["external"]))
        interp=cr.KinematicsInterpolator(cr.SimulatorConfig(**cfg_data))
        if start<interp.time_bounds[0] or end>interp.time_bounds[1]: raise ValueError("Filtered full-trial IK support incomplete")
        q=np.column_stack([interp._splines[n](times) for n in names]);qd=np.column_stack([interp._splines[n](times,1) for n in names])
        if not np.isfinite(q).all() or not np.isfinite(qd).all(): raise ValueError("Nonfinite new full IK interpolation")
        new_values,parity=cr.replay(model,state,coords,model.getBodySet().get("foot_l"),law,times,q,qd,hull,binary,set(np.linspace(0,len(times)-1,11,dtype=int)))
        if parity["cpp_max_abs_force_error_n"]>1e-8 or parity["station_transform_max_abs_error_m_or_m_s"]>1e-10:
            raise AssertionError("C++ contact-law or station velocity parity failed")
        old_values,old_arrays,old_validation=saved_case(subject,selected,times,law,hull,binary)
        advertised=next(x for x in factorial["baseline_exports"] if x["path"]==fp(SAVED/f"baseline_{subject}.npz")["path"])
        if advertised!=fp(SAVED/f"baseline_{subject}.npz"): raise AssertionError("Reviewed old baseline export hash mismatch")
        if old_arrays["coordinate_names"].tolist()!=names: raise AssertionError("Old/new prepared coordinate order differs")
        old=summarize(times,old_values,measured,selected,bw,[s.name for s in profile.spheres])
        new=summarize(times,new_values,measured,selected,bw,[s.name for s in profile.spheres])
        expected=old_replay["subjects"][subject]["cases"]["filtered_IK"]["aggregate"]
        baseline_errors={k:old["contact_summary"]["aggregate"][k]-expected[k] for k in ("impulse_ns","mean_force_n","mean_foot_origin_y_m","mean_sagittal_foot_pitch_rad")}
        if max(abs(v) for v in baseline_errors.values())>1e-8: raise AssertionError("Original filtered endpoint not reproduced")
        old_validation["original_filtered_endpoint_errors"]=baseline_errors
        qdelta=q-old_arrays["q"];qddelta=qd-old_arrays["qdot"]
        subjects[subject]={"source_binding":bundle["binding"],"new_model_contract":contract,"mass_kg":mass,"BW_N":bw,
                           "grid":{"primary_s":[start,end],"native_interior_count":len(times)-7,"exact_event_knots_s":knots.tolist(),"samples":len(times),"old_new_grid_identical":True},
                           "receipt_status_of_original_paired_run":receipt["status"],"filtering":required,
                           "old_case":old,"common_procedure_case":new,"new_minus_old":change(old,new),
                           "validation":{"status":"PASS","new_CPP_and_station_parity":parity,"old_saved_FK":old_validation},
                           "pose_changes":{"coordinate_order":names,"q_new_minus_old_max_abs_by_coordinate":dict(zip(names,np.max(np.abs(qdelta),axis=0).tolist())),
                                           "qdot_new_minus_old_max_abs_by_coordinate":dict(zip(names,np.max(np.abs(qddelta),axis=0).tolist())),
                                           "foot_ankle_Y_new_minus_old_max_abs_m":float(np.max(np.abs(new_values["foot_origin_y"]-old_values["foot_origin_y"]))),
                                           "total_force_new_minus_old_max_abs_N":float(np.max(np.abs(new_values["force"].sum(axis=1)-old_values["force"].sum(axis=1))))},
                           "new_vs_old_full_physical_XML_semantic_equal":equal(xml.flat(xml_model(old_model),physical=True,omit_markers=True),xml.flat(xml_model(bundle["model"]),physical=True,omit_markers=True))}
    pair={}
    for case in ("old_case","common_procedure_case"):
        a,b=(subjects[s][case]["aggregate"] for s in ("AB06","AB07"))
        pair[case]={"AB07_minus_AB06_mean_force_N":b["mean_force_N"]-a["mean_force_N"],"AB07_over_AB06_mean_force_N":b["mean_force_N"]/a["mean_force_N"],
                    "AB06_own_measured_impulse_error_percent":a["total_impulse_error_percent"],"AB07_own_measured_impulse_error_percent":b["total_impulse_error_percent"],
                    "AB07_minus_AB06_stance_foot_ankle_center_Y_m":b["depth"]["foot_ankle_center_Y"]["mean_stance_m"]-a["depth"]["foot_ankle_center_Y"]["mean_stance_m"]}
    after={fp(p)["path"]:fp(p)["sha256"] for p in sorted(paths)}
    if before!=after or any(after[k]!=v for k,v in protected.items()): raise AssertionError("Protected source changed during replay")
    result={"created_utc":datetime.now(timezone.utc).isoformat(),"instruction_check_token":"CMC_AGENT_OK_2026","status":"PASS_saved_reference_FK_only",
            "helper":fp(__file__),"backend":{"python":sys.version,"platform":platform.platform(),"numpy":np.__version__,"opensim_module":str(cr.opensim.__file__),"opensim_version":str(cr.opensim.GetVersionAndDate()) if hasattr(cr.opensim,"GetVersionAndDate") else str(getattr(cr.opensim,"__version__","unknown"))},
            "methodology":{"coherent_cases":"Old own-model/full-trial filtered IK saved FK versus new own-model/new full marker IK; no mixed geometry-motion cases",
                           "grid":"Frozen primary3cycles: native1ms interior + seven exact nominal HS/TO/nextHS; no phase shift/reselection",
                           "filter":"Recorded production full-trial6Hz order4 resample1ms, rotations rad before filtering, spline qdot",
                           "force":"Same frozen primary8sphere profile; C++ parity11probe poses per subject; no profile fitting",
                           "phase":"Measured healthy nominal stance/swing; 20 duration-weighted stance bins descriptive only; original durations used for impulses",
                           "scope":"No dynamics, assembly, IK solve, clamp, policy, model serialization, operational input edits"},
            "subjects":subjects,"pair_comparison":pair,
            "limitations":["This is direct contact response of coherent prepared reference poses, not a new dynamic prescribed rollout.",
                           "A changed AB06 baseline does not retrospectively alter the old paired measured-force result or its failed final callback status.",
                           "The difference after the same procedure still combines healthy anatomy, each subject motion, model graft, marker fitting, registration and solver response.",
                           "Measured healthy GRF and a virtual prosthetic contact basis are different systems; agreement/disagreement does not alone prove a registration or hardware error.",
                           "Original AB06 full IK has weaker historical solver provenance; the new model/IK/marker trio is explicitly hash-bound.",
                           "Numerical parity/XML tolerances check implementation identity; they are not new subject qualification gates."],
            "inputs_integrity":{"original_protected_count":len(protected),"consumed_count":len(before),"before":before,"after":after,"all_unchanged":True}}
    with output.open("x",encoding="utf-8") as f:
        json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
        f.write("\n")
    print(json.dumps({"output":fp(output),"status":result["status"],"pair":pair},allow_nan=False),flush=True)


if __name__=="__main__":
    main()
