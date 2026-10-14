"""Freeze and build coherent AB07 pylon sweep; immutable endpoint reuse.

--freeze writes protocol only. --case builds one fresh12marker/fullIK case or
records reuse of one exact completed endpoint. No dynamics, profile fitting,
MATLAB conversion, NN inference/training, source edit or output overwrite.
"""
from datetime import datetime,timezone
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import traceback
import xml.etree.ElementTree as ET
import numpy as np

PREPARATION=Path(__file__).resolve().parent
AUDIT=PREPARATION.parent
ROOT=AUDIT.parents[1]
OLD=ROOT/"validation/ab07_mount_registration_verification_2026_10_07"
PROTOCOL=PREPARATION/"preparation_protocol.json"
sys.path.insert(0,str(ROOT))
spec=importlib.util.spec_from_file_location("sweep_original_preparation",OLD/"variants/prepare_mount_variants.py")
old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
fp,source,recheck,write_new=old.fp,old.source,old.recheck,old.write_new
calibration,runner=old.calibration,old.runner


def read(path):
    return json.loads(Path(path).read_text(),parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))


def freeze():
    if PROTOCOL.exists():raise FileExistsError("Preserve frozen protocol")
    previous=read(OLD/"variants_protocol.json")
    comparison=read(OLD/"variant_contact_comparison.json")
    rotation=read(OLD/"marker_rotation_invariance.json")
    protected=dict(rotation["inputs_integrity"]["after"])
    protected.update(comparison["inputs_integrity"]["after"])
    roles=dict(previous["source_roles"])
    for p in (Path(__file__),OLD/"variants/prepare_mount_variants.py",OLD/"variants_protocol.json",OLD/"variant_contact_comparison.json",OLD/"marker_rotation_invariance.json",OLD/"variant_contact_scientific_review.json"):
        protected[fp(p)["path"]]=fp(p)["sha256"]
    recheck(protected)
    lengths=[385.89929908923726,390.,395.,400.,403.4784289483052,408.,413.]
    angles=[0.,4.730648501276871]
    cases={}
    for i,length in enumerate(lengths):
        for branch,angle in enumerate(angles):
            name=f"P{i}_A{branch}"
            endpoint=("L0_A"+str(branch)) if i==0 else ("L1_A"+str(branch)) if i==4 else None
            length_m=previous["cases"][endpoint]["distal_mount_length_m"] if endpoint else length/1000
            reused=None
            if endpoint:
                p=OLD/"variants"/endpoint/"receipt.json";r=read(p)
                if r["status"]!="completed" or r["case"]["distal_mount_length_m"]!=length_m or r["case"]["static_ankle_offset_deg"]!=angle:
                    raise AssertionError("Reused endpoint does not match exact requested grid")
                for k in ("output_model","full_IK","static_motion","actual_solver_setup","actual_solver_model_copy","actual_solver_marker_copy"):
                    if fp(ROOT/r[k]["path"])!=r[k]:raise AssertionError("Completed endpoint artifact hash changed")
                reused={"receipt":fp(p),"endpoint_id":endpoint,"output_model":r["output_model"],"full_IK":r["full_IK"],"reference_case_in_old_comparison":endpoint}
                protected[fp(p)["path"]]=fp(p)["sha256"]
            cases[name]={"id":name,"length_index":i,"angle_branch":branch,"distal_mount_length_mm":length,"distal_mount_length_m":length_m,
                         "mount_parent_Y_m":-length_m,"static_ankle_offset_deg":angle,
                         "case_dir":(PREPARATION/"cases"/name).relative_to(ROOT).as_posix(),
                         "preparation_kind":"immutable_endpoint_reuse" if endpoint else "fresh_marker_calibration_full_IK","reused_endpoint":reused}
    selection=read(ROOT/roles["frozen_selection"]["path"])
    support={k:fp(ROOT/selection["subjects"]["AB07"]["source_fingerprints"][k]["path"]) for k in ("external_loads","reserve_actuators","measured_GRF")}
    obj={"created_utc":datetime.now(timezone.utc).isoformat(),"instruction_check_token":"CMC_AGENT_OK_2026","status":"frozen_before_sweep_build",
         "confirmed_grid":{"lengths_mm":lengths,"static_ankle_offsets_deg":angles,"total_cells":14,"fresh_build_cells":10,"immutable_reused_cells":4,"endpoint_unit_note":"Endpoint metre values are pinned exactly from old receipts; displayed millimetres are not converted back because binary rounding can differ by1ULP"},
         "cases":cases,"execution_order":[n for n,c in cases.items() if c["preparation_kind"]=="immutable_endpoint_reuse"]+[n for n,c in cases.items() if c["preparation_kind"]!="immutable_endpoint_reuse"],
         "source_roles":roles,"support_data_dir":previous["support_data_dir"],"support_paths":support,
         "full_time_range_s":previous["full_time_range_s"],"calibration_markers":previous["calibration_markers"],"calibration_stride":1,"calibration_time_range":None,
         "backend":previous["backend"],"method":"Original12marker calibration and original Python fullIK runner per fresh case, opensim_cmdNone, scaling off, ik_only, smokeFalse; no model-motion hybrids",
         "prospective_dynamic_plan":{"cells":"All14, no selection by FK forcefit","runtime":"Current V26 prescribed_action, no checkpoint/no NN","reset_HS_s":12.591740519774012,"duration_s":6.10,
                                     "requested_end_s":12.591740519774012+6.10,"cycles":"One startup then exactly the original3primary frozen cycles, no rewindowing", "owner":"root"},
         "FK_plan":{"scope":"Reference contact only before dynamics, full-trial6Hz order4 resample1ms from original effective receipt","grid":"Frozen AB07 primary3cycles, original native1ms interior+seven exact events",
                    "endpoints":"Reuse previous reviewed endpoint FK outputs at exact same geometry/IK/filter/profile/grid; no duplicate FK needed","fresh_cells":"Each own fresh model+fullIK, C++ law/station parity, markerfit and COM reference budget",
                    "qualification":"No forceprofile fitting, clinical mount inference or arbitrary new acceptance gate"},
         "protected_count":len(protected),"protected_sources_before":protected}
    recheck(protected);write_new(PROTOCOL,obj)
    print(json.dumps({"protocol":fp(PROTOCOL),"grid":obj["confirmed_grid"],"protected_count":len(protected),"execution_order":obj["execution_order"]},allow_nan=False),flush=True)


def build(name):
    protocol=read(PROTOCOL);recheck(protocol["protected_sources_before"])
    if sys.executable!=protocol["backend"]["python"] or str(calibration.osim.GetVersionAndDate())!=protocol["backend"]["opensim_version"]:
        raise AssertionError("Frozen backend version/interpreter mismatch")
    case=protocol["cases"][name];out=ROOT/case["case_dir"]
    if out.exists():raise FileExistsError("Do not overwrite or reuse case build directories")
    out.parent.mkdir(parents=True,exist_ok=True);out.mkdir()
    receipt={"created_utc":datetime.now(timezone.utc).isoformat(),"status":"prepared","case":case,"protocol":fp(PROTOCOL),"helper":fp(__file__),
             "backend":protocol["backend"],"support_paths":protocol["support_paths"]}
    write_new(out/"receipt_prepared.json",receipt)
    def role(n):return source(protocol["source_roles"][n]["path"])
    try:
        if case["preparation_kind"]=="immutable_endpoint_reuse":
            reuse=case["reused_endpoint"];original=read(ROOT/reuse["receipt"]["path"])
            if fp(ROOT/reuse["receipt"]["path"])!=reuse["receipt"]:raise AssertionError("Endpoint receipt hash changed")
            for k in ("output_model","full_IK"):
                if fp(ROOT/reuse[k]["path"])!=reuse[k]:raise AssertionError("Endpoint model/IK hash changed")
            receipt.update({k:original[k] for k in ("output_model","full_IK","static_motion","constructed_model","marker_calibration_report","full_IK_time_range_s","full_IK_samples","actual_solver_setup","actual_solver_model_copy","actual_solver_marker_copy","actual_solver_log","marker_calibration_preservation")})
            receipt["reused_endpoint_receipt"]=reuse["receipt"]
        else:
            input_model=out/"AB07_SEASEA.osim";static=out/"AB07_SEASEA_static_ground_registered.mot"
            calibrated=out/"AB07_SEASEA_marker_calibrated.osim";marker_report=out/"AB07_SEASEA_marker_calibrated.marker_calibration.csv"
            tree=ET.parse(role("anatomical_model"));text=old.mount_element(tree);v=text.text.split();v[1]=format(case["mount_parent_Y_m"],".17g");text.text=" ".join(v)
            tree.write(input_model,encoding="utf-8",xml_declaration=True)
            baseline=ET.parse(role("anatomical_model"));old.mount_element(baseline).text="0 0 0"
            made=ET.parse(input_model);old.mount_element(made).text="0 0 0"
            if old.xml.flat(baseline.getroot())!=old.xml.flat(made.getroot()):raise AssertionError("Only one mountY property may change")
            motion=calibration.parse_motion(role("unshifted_static_motion"));values=motion.values.copy()
            if not motion.in_degrees or len(motion.labels)!=22 or len(set(motion.labels))!=22 or not np.isfinite(values).all() or np.any(np.diff(motion.times)<=0):raise AssertionError("Static coordinate/clock contract invalid")
            values[:,motion.labels.index("pros_ankle_angle")]+=case["static_ankle_offset_deg"]
            with static.open("x",encoding="utf-8") as f:
                f.write(f"{static.stem}\nnRows={len(values)}\nnColumns=22\ninDegrees=yes\nendheader\n"+"\t".join(motion.labels)+"\n");np.savetxt(f,values,delimiter="\t",fmt="%.17g")
            calibration.calibrate_markers(input_model,role("static_TRC"),static,calibrated,marker_report,role("plugin"),list(protocol["calibration_markers"]),None,1)
            preservation=old.marker_preservation(input_model,calibrated)
            cfg=runner.ToolConfig(opensim_cmd=None,plugin=role("plugin"),model=calibrated,marker_set=calibrated,trc=role("full_trial_TRC"),grf=role("full_trial_GRF"),support_data=ROOT/protocol["support_data_dir"],dry_run=False,skip_scaling=True,ik_only=True,full_time_range=tuple(protocol["full_time_range_s"]))
            result=runner.run_pipeline(cfg,runner.create_paths(out/"pipeline"),run_smoke=False)
            if result!=0:raise RuntimeError("Full markerIK failed")
            ik=out/"pipeline/02_ik"/("IK_full_"+runner.range_label(cfg.full_time_range)+".mot")
            full=calibration.parse_motion(ik);model_names=set(old.xml.coordinates(ET.parse(calibrated).find("Model")))
            if len(full.labels)!=22 or len(set(full.labels))!=22 or set(full.labels[1:])!=model_names or len(model_names)!=21 or not full.in_degrees or not np.isfinite(full.values).all() or np.any(np.diff(full.times)<=0):raise AssertionError("Complete21unique model/fullIK coordinate schema invalid")
            if [float(full.times[0]),float(full.times[-1])]!=protocol["full_time_range_s"]:raise AssertionError("FullIK time range incomplete")
            setup=ik.with_name(ik.stem+"_setup.xml");sx=ET.parse(setup)
            actual_model=source(sx.findtext(".//model_file"));actual_markers=source(sx.findtext(".//marker_file"))
            if actual_model.read_bytes()!=calibrated.read_bytes() or actual_markers.read_bytes()!=role("full_trial_TRC").read_bytes():raise AssertionError("Actual solver copies differ from source-bound model/TRC")
            receipt.update(output_model=fp(calibrated),full_IK=fp(ik),static_motion=fp(static),constructed_model=fp(input_model),marker_calibration_report=fp(marker_report),
                           marker_calibration_preservation=preservation,full_IK_samples=len(full.times),full_IK_time_range_s=[float(full.times[0]),float(full.times[-1])],
                           actual_solver_setup=fp(setup),actual_solver_model_copy=fp(actual_model),actual_solver_marker_copy=fp(actual_markers),actual_solver_log=fp(out/"pipeline/02_ik/ik_full.log"))
        recheck(protocol["protected_sources_before"])
        receipt.update(status="completed",completed_utc=datetime.now(timezone.utc).isoformat(),protected_inputs_unchanged=True,protected_input_count=protocol["protected_count"],
                       model_bundle_dir=str((ROOT/receipt["output_model"]["path"]).parent.relative_to(ROOT)),
                       backend_contract={"opensim_cmd":None,"skip_scaling":True,"ik_only":True,"run_smoke":False,"python":sys.executable,"opensim_version":str(calibration.osim.GetVersionAndDate())})
        write_new(out/"receipt.json",receipt)
        print(json.dumps({"case":name,"status":"completed","preparation_kind":case["preparation_kind"],"model":receipt["output_model"],"full_IK":receipt["full_IK"],"receipt":fp(out/"receipt.json")},allow_nan=False),flush=True)
    except Exception as exc:
        receipt.update(status="failed",error_type=type(exc).__name__,error=str(exc),traceback=traceback.format_exc(),protected_inputs_unchanged=(recheck(protocol["protected_sources_before"])==protocol["protected_sources_before"]))
        write_new(out/"receipt.json",receipt);raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    g=parser.add_mutually_exclusive_group(required=True);g.add_argument("--freeze",action="store_true");g.add_argument("--case")
    args=parser.parse_args()
    freeze() if args.freeze else build(args.case)


if __name__=="__main__":main()
