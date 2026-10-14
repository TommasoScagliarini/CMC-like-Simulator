"""Prepare or build isolated AB07 mount/static-marker 2x2 variants.

--prepare freezes saved cached sources and writes the protocol only.
--case performs one explicitly selected fresh marker calibration and complete
marker IK using the original Python API runner, opensim_cmd=None, no scaling,
RRA, smoke test, dynamics or operational input edits. Outputs never overwrite.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import platform
import sys
import traceback
import xml.etree.ElementTree as ET

import numpy as np

VARIANTS=Path(__file__).resolve().parent
AUDIT=VARIANTS.parent
ROOT=AUDIT.parents[1]
PREVIOUS=ROOT/"validation/ab06_ab07_independent_attribution_2026_10_07"
REBUILD=PREVIOUS/"rebuild"
PROTOCOL=AUDIT/"variants_protocol.json"
sys.path.insert(0,str(ROOT))


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec)
    sys.modules[name]=m
    spec.loader.exec_module(m)
    return m


calibration=module("mount_variant_calibration",ROOT/"tools/calibrate_ab06_seasea_markers.py")
runner=module("mount_variant_original_runner",ROOT/"scripts/run_opensim_sea_pipeline.py")
xml=module("mount_variant_xml_pure",ROOT/"validation/ab06_ab07_preparation_vs_model_2026_10_07/compare_model_xml.py")


def fp(path):
    path=Path(path).resolve()
    return {"path":path.relative_to(ROOT).as_posix(),"sha256":hashlib.sha256(path.read_bytes()).hexdigest()}


def jread(path):
    return json.loads(Path(path).read_text(),parse_constant=lambda s:(_ for _ in ()).throw(ValueError(s)))


def source(raw):
    p=Path(raw)
    p=(p if p.is_absolute() else ROOT/p).resolve()
    p.relative_to(ROOT)
    if not p.is_file():raise FileNotFoundError(p)
    return p


def write_new(path,obj):
    with Path(path).open("x",encoding="utf-8") as f:
        json.dump(obj,f,indent=2,ensure_ascii=False,allow_nan=False);f.write("\n")


def recheck(manifest):
    actual={raw:fp(ROOT/raw)["sha256"] for raw in manifest}
    if actual!=manifest:
        raise AssertionError("Frozen inputs changed: "+repr([k for k in manifest if actual[k]!=manifest[k]]))
    return actual


def mount_element(tree):
    joint=tree.find(".//JointSet/objects/PinJoint[@name='pros_ankle']")
    if joint is None:raise ValueError("Required unchanged pros_ankle PinJoint missing")
    frame_name=joint.findtext("socket_parent_frame")
    frame=joint.find(f"frames/PhysicalOffsetFrame[@name='{frame_name}']")
    position=[float(v) for v in frame.findtext("translation").split()]
    if position[0]!=0 or position[2]!=0 or position[1]>=0 or not np.isfinite(position).all():
        raise ValueError("Expected longitudinal distal mounting")
    return frame.find("translation")


def prepare():
    if PROTOCOL.exists():raise FileExistsError("Preserve frozen protocol")
    contact=jread(REBUILD/"contact_replay_common_procedure.json")
    geometry=jread(PREVIOUS/"geometry/functional_ankle_geometry.json")
    parent_protocol=jread(AUDIT/"protocol.json")
    protected=dict(parent_protocol["original_and_previous_audit_sources_before"])
    protected.update(contact["inputs_integrity"]["after"])
    protected.update(geometry["inputs_integrity"]["fingerprints_after"])
    recheck(protected)
    report_path=REBUILD/"models/AB07_SEASEA_Threadmill/pipeline_report.json"
    report=jread(report_path)
    if report["status"]!="complete" or report["ik_method"]!="opensim_marker_ik":
        raise ValueError("Require completed common-procedure AB07 reference")
    g=report["ground_registration"]
    raw_static=source(g["sources"]["input_static_motion"]["path"])
    static_dir=source(report["static_calibration_motion"]).parent
    converted_static=ROOT/report["converted_static_dir"]
    static_trcs=list(converted_static.glob("*.trc"))
    if len(static_trcs)!=1:raise ValueError("Require exactly one cached raw-static converted TRC")
    trial_dir=ROOT/report["converted_trial_dir"]
    trial_trc=source(report["ik_provenance"]["marker_file"])
    trial_grf=trial_dir/(report["trial"]+"_grf.mot")
    roles={"parent_protocol":AUDIT/"protocol.json","reference_pipeline_report":report_path,"anatomical_model":source(report["anatomical_model"]),
           "unshifted_static_motion":raw_static,"static_TRC":static_trcs[0],"full_trial_TRC":trial_trc,
           "full_trial_GRF":trial_grf,"current_registered_model":source(report["graft_model"]),
           "current_static_motion":source(report["static_calibration_motion"]),
           "current_calibrated_model":source(report["calibrated_model"]),
           "current_full_IK":source(report["ik_provenance"]["source_file"]),"plugin":source(report["plugin"]),
           "calibration_helper":ROOT/"tools/calibrate_ab06_seasea_markers.py",
           "IK_runner":ROOT/"scripts/run_opensim_sea_pipeline.py",
           "XML_helper":ROOT/"validation/ab06_ab07_preparation_vs_model_2026_10_07/compare_model_xml.py",
           "this_helper":Path(__file__),"common_procedure_contact":REBUILD/"contact_replay_common_procedure.json",
           "functional_geometry":PREVIOUS/"geometry/functional_ankle_geometry.json",
           "frozen_selection":ROOT/"Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/selection.json"}
    for p in roles.values():protected[fp(p)["path"]]=fp(p)["sha256"]
    support=ROOT/"models/SEASEA/data"
    for p in support.glob("*.xml"):protected[fp(p)["path"]]=fp(p)["sha256"]
    short=g["subject_before"]["distal_mount_y_m"]
    current=g["subject_after"]["distal_mount_y_m"]
    angle=g["adjustments"]["static_ankle_pose_offset_deg"]
    if float(mount_element(ET.parse(roles["anatomical_model"])).text.split()[1])!=short:
        raise AssertionError("Anatomical model does not contain short mount endpoint")
    cases={}
    for length_id,mount in ((0,short),(1,current)):
        for angle_id,offset in ((0,0.),(1,angle)):
            name=f"L{length_id}_A{angle_id}"
            cases[name]={"id":name,"mount_parent_Y_m":mount,"distal_mount_length_m":-mount,
                         "static_ankle_offset_deg":offset,"length_factor":"anatomical_short" if length_id==0 else "registered_long",
                         "static_angle_factor":"unshifted" if angle_id==0 else "registered_static_offset",
                         "case_dir":(VARIANTS/name).relative_to(ROOT).as_posix(),
                         "is_current_endpoint":name=="L1_A1"}
    obj={"created_utc":datetime.now(timezone.utc).isoformat(),"instruction_check_token":"CMC_AGENT_OK_2026",
         "status":"frozen_before_any_variant_calibration_or_IK","subject":"AB07","factors":{
             "distal_mount_short_Y_m":short,"distal_mount_registered_Y_m":current,
             "added_length_m":short-current,"static_ankle_offsets_deg":[0.,angle],
             "meaning":"Mount changes one parent-offset translation along pylon localY; static angle affects marker calibration motion only, not dynamic joint zero or commands"},
         "cases":cases,"recommended_execution_order":["L1_A1","L0_A0","L0_A1","L1_A0"],
         "source_roles":{k:fp(p) for k,p in roles.items()},"support_data_dir":support.relative_to(ROOT).as_posix(),
         "full_time_range_s":report["kinematics_time_range"],"calibration_markers":list(calibration.DEFAULT_MARKERS),
         "calibration_time_range":None,"calibration_stride":1,
         "backend":{"opensim_cmd":None,"skip_scaling":True,"ik_only":True,"run_smoke":False,"dry_run":False,
                    "python":sys.executable,"python_version":sys.version,"opensim_version":str(calibration.osim.GetVersionAndDate()),
                    "implementation":"Original calibration.calibrate_markers and runner.run_pipeline API; runner forces Python OpenSim backend"},
         "frozen_analysis":{"primary":"Unchanged measured-GRF primary3cycles and seven exact events from frozen selection",
                            "grid":"Original paired native1ms interior plus seven exact HS/TO/nextHS",
                            "filter":"Original receipt full-trial6Hz order4 resample1ms; no new dynamic rollout",
                            "profiles":"Frozen original primary force/legacy/binary profiles; no fitting or force gate selection"},
         "protected_count":len(protected),"protected_sources_before":protected,
         "current_endpoint_required":{"calibrated_model_bytes_equal":True,"full_IK_bytes_equal":True},
         "limitations":["The 2x2 compares finite coherent model/marker/IK preparations; no mixed geometry-motion cells.",
                        "Variation of static marker calibration can absorb geometric changes in IK; identical fits do not alone validate physical hardware placement.",
                        "FK contact/COM results will be reference pose responses, not actual dynamic tracking or reserve measurements.",
                        "Virtual prosthetic contact and healthy measured GRF are different systems; this verification cannot by itself establish clinical mount length."]}
    recheck(protected)
    write_new(PROTOCOL,obj)
    print(json.dumps({"protocol":fp(PROTOCOL),"protected_count":len(protected),"cases":cases},allow_nan=False),flush=True)


def equivalent(a,b):
    if isinstance(a,(int,float)) and isinstance(b,(int,float)):
        return math.isclose(a,b,rel_tol=1e-14,abs_tol=1e-14)
    if isinstance(a,list) and isinstance(b,list):
        return len(a)==len(b) and all(equivalent(x,y) for x,y in zip(a,b))
    if isinstance(a,dict) and isinstance(b,dict):
        return a.keys()==b.keys() and all(equivalent(a[k],b[k]) for k in a)
    return a==b


def marker_preservation(before_path,after_path):
    a,b=(ET.parse(p).find("Model") for p in (before_path,after_path))
    af,bf=(xml.flat(m,physical=True,omit_markers=True) for m in (a,b))
    differences=xml.differences(af,bf)
    substantive=[d for d in differences if "/groups/" not in d["field"] and not equivalent(d["before"],d["after"])]
    if substantive:raise AssertionError("Calibration changed non-marker physical fields: "+repr(substantive[:4]))
    force_names=set(xml.objects(b,"ForceSet"))
    group_differences=[d for d in differences if "/groups/" in d["field"]]
    if any([n for n in (d["before"] or []) if n in force_names] != (d["after"] or []) for d in group_differences):
        raise AssertionError("Native serialization changed group metadata beyond cleanup of absent forces")
    marker_a,marker_b=(xml.objects(m,"MarkerSet") for m in (a,b))
    if marker_a.keys()!=marker_b.keys():raise AssertionError("Calibration changed marker membership")
    changed={n:xml.differences(xml.flat(marker_a[n]),xml.flat(marker_b[n])) for n in marker_a}
    selected=set(calibration.DEFAULT_MARKERS)
    for n,diff in changed.items():
        if n not in selected and diff:raise AssertionError("Calibration modified an unselected marker")
        if any(not d["field"].endswith("/location/#text") for d in diff):raise AssertionError("Calibration changed marker attachment/contract")
    return {"status":"PASS","selected12_marker_changes":{n:d for n,d in changed.items() if d},
            "nonmarker_raw_XML_serialization_differences":differences,
            "numeric_XML_comparison_tolerance":{"rel":1e-14,"abs":1e-14,"purpose":"Serialization only, no new scientific threshold"},
            "all_other_physical_fields_semantically_unchanged":True,"unselected_marker_fields_unchanged":True}


def run_case(name):
    protocol=jread(PROTOCOL)
    recheck(protocol["protected_sources_before"])
    if sys.executable!=protocol["backend"]["python"] or str(calibration.osim.GetVersionAndDate())!=protocol["backend"]["opensim_version"]:
        raise AssertionError("Frozen Python/OpenSim backend differs")
    c=protocol["cases"][name]
    output=ROOT/c["case_dir"]
    if output.exists():raise FileExistsError("No variant reuse/overwrite permitted")
    output.mkdir()
    def role(n):return source(protocol["source_roles"][n]["path"])
    input_model=output/"AB07_SEASEA.osim"
    static=output/"AB07_SEASEA_static_ground_registered.mot"
    calibrated=output/"AB07_SEASEA_marker_calibrated.osim"
    report=output/"AB07_SEASEA_marker_calibrated.marker_calibration.csv"
    receipt={"created_utc":datetime.now(timezone.utc).isoformat(),"status":"prepared","case":c,
             "protocol":fp(PROTOCOL),"helper":fp(__file__),"backend":protocol["backend"],
             "calibration_inputs":{n:protocol["source_roles"][n] for n in ("anatomical_model","unshifted_static_motion","static_TRC","full_trial_TRC","full_trial_GRF")}}
    write_new(output/"receipt_prepared.json",receipt)
    try:
        tree=ET.parse(role("anatomical_model"))
        text=mount_element(tree);v=text.text.split();v[1]=format(c["mount_parent_Y_m"],".17g");text.text=" ".join(v)
        tree.write(input_model,encoding="utf-8",xml_declaration=True)
        source_tree=ET.parse(role("anatomical_model"));mount_element(source_tree).text="0 0 0"
        check_tree=ET.parse(input_model);mount_element(check_tree).text="0 0 0"
        if xml.flat(source_tree.getroot())!=xml.flat(check_tree.getroot()):
            raise AssertionError("Variant construction changed fields beyond one mountY")
        motion=calibration.parse_motion(role("unshifted_static_motion"))
        if not motion.in_degrees or len(motion.labels)!=22 or len(set(motion.labels))!=22:
            raise AssertionError("Require complete21coordinate degree-valued static motion")
        values=motion.values.copy();index=motion.labels.index("pros_ankle_angle")
        values[:,index]+=c["static_ankle_offset_deg"]
        header=f"{static.stem}\nnRows={len(motion.times)}\nnColumns={len(motion.labels)}\ninDegrees=yes\nendheader\n"
        with static.open("x",encoding="utf-8") as f:
            f.write(header+"\t".join(motion.labels)+"\n");np.savetxt(f,values,delimiter="\t",fmt="%.17g")
        if c["is_current_endpoint"]:
            if input_model.read_bytes()!=role("current_registered_model").read_bytes() or static.read_bytes()!=role("current_static_motion").read_bytes():
                raise AssertionError("Current endpoint construction does not exactly reproduce registered model/static source")
        calibration.calibrate_markers(input_model,role("static_TRC"),static,calibrated,report,role("plugin"),
                                      list(protocol["calibration_markers"]),None,1)
        preservation=marker_preservation(input_model,calibrated)
        if c["is_current_endpoint"] and calibrated.read_bytes()!=role("current_calibrated_model").read_bytes():
            raise AssertionError("Current endpoint fresh marker calibration does not reproduce reference model bytes")
        cfg=runner.ToolConfig(opensim_cmd=None,plugin=role("plugin"),model=calibrated,marker_set=calibrated,
                             trc=role("full_trial_TRC"),grf=role("full_trial_GRF"),support_data=ROOT/protocol["support_data_dir"],
                             dry_run=False,skip_scaling=True,ik_only=True,full_time_range=tuple(protocol["full_time_range_s"]))
        if cfg.opensim_cmd is not None or not cfg.skip_scaling or not cfg.ik_only or cfg.dry_run:
            raise AssertionError("Required pure-Python full-IK backend contract violated")
        pipeline=output/"pipeline"
        result=runner.run_pipeline(cfg,runner.create_paths(pipeline),run_smoke=False)
        if result!=0:raise RuntimeError(f"Full marker IK returncode{result}")
        ik=pipeline/"02_ik"/("IK_full_"+runner.range_label(cfg.full_time_range)+".mot")
        full_motion=calibration.parse_motion(ik)
        if not full_motion.in_degrees or len(full_motion.labels)!=22 or not np.isfinite(full_motion.values).all() or np.any(np.diff(full_motion.times)<=0):
            raise AssertionError("Full IK coordinates/clock/finiteness invalid")
        if [float(full_motion.times[0]),float(full_motion.times[-1])]!=protocol["full_time_range_s"]:
            raise AssertionError("Full IK does not cover original full trial")
        setup=ik.with_name(ik.stem+"_setup.xml");setup_xml=ET.parse(setup)
        actual_model=source(setup_xml.findtext(".//model_file"));actual_marker=source(setup_xml.findtext(".//marker_file"))
        if actual_model.read_bytes()!=calibrated.read_bytes() or actual_marker.read_bytes()!=role("full_trial_TRC").read_bytes():
            raise AssertionError("Actual solver copies differ from calibrated model/TRC")
        matches_current=ik.read_bytes()==role("current_full_IK").read_bytes()
        if c["is_current_endpoint"] and not matches_current:
            raise AssertionError("Current endpoint fresh full marker IK does not reproduce reference bytes")
        recheck(protocol["protected_sources_before"])
        receipt.update(status="completed",completed_utc=datetime.now(timezone.utc).isoformat(),
                       marker_calibration_preservation=preservation,
                       output_model=fp(calibrated),static_motion=fp(static),constructed_model=fp(input_model),marker_calibration_report=fp(report),
                       full_IK=fp(ik),full_IK_time_range_s=[float(full_motion.times[0]),float(full_motion.times[-1])],full_IK_samples=len(full_motion.times),
                       actual_solver_setup=fp(setup),actual_solver_model_copy=fp(actual_model),actual_solver_marker_copy=fp(actual_marker),
                       actual_solver_log=fp(pipeline/"02_ik/ik_full.log"),
                       backend_contract={"opensim_cmd":None,"skip_scaling":True,"ik_only":True,"run_smoke":False,"python":sys.executable,"opensim_version":str(calibration.osim.GetVersionAndDate())},
                       current_endpoint_byte_matches={"calibrated_model":calibrated.read_bytes()==role("current_calibrated_model").read_bytes(),"full_IK":matches_current},
                       protected_inputs_unchanged=True,protected_input_count=protocol["protected_count"])
        write_new(output/"receipt.json",receipt)
        print(json.dumps({"case":name,"status":"completed","model":fp(calibrated),"IK":fp(ik),"current_endpoint_byte_matches":receipt["current_endpoint_byte_matches"]},allow_nan=False),flush=True)
    except Exception as exc:
        receipt.update(status="failed",error_type=type(exc).__name__,error=str(exc),traceback=traceback.format_exc(),
                       protected_inputs_unchanged=(recheck(protocol["protected_sources_before"])==protocol["protected_sources_before"]))
        write_new(output/"receipt.json",receipt)
        raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_mutually_exclusive_group(required=True)
    commands.add_argument("--prepare",action="store_true")
    commands.add_argument("--case",choices=("L0_A0","L0_A1","L1_A0","L1_A1"))
    args=parser.parse_args()
    prepare() if args.prepare else run_case(args.case)


if __name__=="__main__":
    main()
