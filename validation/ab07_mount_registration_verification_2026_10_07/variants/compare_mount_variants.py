"""Analyze four completed coherent AB07 preparations; saved references and FK only.

Exact frozen primary cycles, recorded production full-trial filtering and fixed
contact profile. No dynamics, solver, assembly, neural inference or model edits.
"""
from datetime import datetime, timezone
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

VARIANTS=Path(__file__).resolve().parent
AUDIT=VARIANTS.parent
ROOT=AUDIT.parents[1]
PROTOCOL=AUDIT/"variants_protocol.json"
OLD=ROOT/"validation/ab06_ab07_preparation_vs_model_2026_10_07"
CAUSAL=ROOT/"validation/ab06_ab07_causal_investigation_2026_10_07"
PAIR=ROOT/"Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07"
INTEGRATE=np.trapezoid if hasattr(np,"trapezoid") else np.trapz
sys.path.insert(0,str(ROOT))


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m


cr=module("mount_variant_contact_replay",OLD/"contact_replay.py")
phase=module("mount_variant_phase",CAUSAL/"alternative/phase_contact_budget.py")
calibration=module("mount_variant_marker_parse",ROOT/"tools/calibrate_ab06_seasea_markers.py")


def fp(p):
    p=Path(p).resolve();return {"path":p.relative_to(ROOT).as_posix(),"sha256":hashlib.sha256(p.read_bytes()).hexdigest()}


def read(path):
    return json.loads(Path(path).read_text(),parse_constant=lambda s:(_ for _ in ()).throw(ValueError(s)))


def interp_supported(t,y,x):
    if x[0]<t[0] or x[-1]>t[-1]:raise ValueError("Source interval incomplete; no endpoint clamping allowed")
    return np.interp(x,t,y)


def pose_diagnostics(model,state,coords,times,q,qd,trc,cycles):
    markers=[model.getMarkerSet().get(i) for i in range(model.getMarkerSet().getSize())]
    names=[m.getName() for m in markers]
    if set(names)!=set(trc.markers) or trc.units.lower() not in ("m","mm"):
        raise AssertionError("Full marker membership/units differ")
    if not np.isfinite(trc.times).all() or np.any(np.diff(trc.times)<=0):raise ValueError("TRC clock invalid")
    measured=np.stack([np.column_stack([interp_supported(trc.times,trc.markers[n][:,j],times) for j in range(3)]) for n in names],axis=1)
    if not np.isfinite(measured).all():raise ValueError("Acquired marker coverage nonfinite")
    error=np.empty_like(measured)
    boundaries=set(c[k] for c in cycles for k in ("hs_s","to_s","next_hs_s"))
    com={};center_error=0.
    joint=model.getJointSet().get("pros_ankle")
    for i,t in enumerate(times):
        cr.set_pose(model,state,coords,q[i],qd[i],t)
        origin=cr.v3(model.getBodySet().get("foot_l").getTransformInGround(state).p())
        center=cr.v3(joint.getParentFrame().getTransformInGround(state).p())
        child=cr.v3(joint.getChildFrame().getTransformInGround(state).p())
        center_error=max(center_error,float(np.max(np.abs(origin-center))),float(np.max(np.abs(center-child))))
        for j,m in enumerate(markers):error[i,j]=cr.v3(m.getLocationInGround(state))-measured[i,j]
        if float(t) in boundaries:
            com[float(t)]={"time_s":float(t),"COM_position_m":cr.v3(model.calcMassCenterPosition(state)).tolist(),
                           "COM_velocity_m_s":cr.v3(model.calcMassCenterVelocity(state)).tolist()}
    if len(com)!=7 or center_error>1e-10:raise AssertionError("Ankle/foot center identity or COM boundary coverage failed")
    def window(a,b):
        mask=(times>=a)&(times<=b);t=times[mask];e=error[mask]
        if t[0]!=a or t[-1]!=b:raise AssertionError("Marker metrics lack exact boundary")
        rms=np.sqrt(INTEGRATE(np.sum(e*e,axis=2),t,axis=0)/(b-a))
        signed=INTEGRATE(e,t,axis=0)/(b-a)
        rows={n:{"RMS_3D_m":float(rms[j]),"signed_mean_XYZ_m":signed[j].tolist(),"max_3D_m":float(np.linalg.norm(e[:,j],axis=1).max())} for j,n in enumerate(names)}
        left=[rows[n]["RMS_3D_m"] for n in calibration.DEFAULT_MARKERS]
        return {"window_s":[a,b],"duration_s":b-a,"per_marker":rows,"selected12_left_markers_mean_of_RMS_m":float(np.mean(left)),
                "all28_markers_mean_of_RMS_m":float(np.mean(rms))}
    marker_cycles=[]
    for c in cycles:marker_cycles.append({"nominal":c,"stance":window(c["hs_s"],c["to_s"]),"swing":window(c["to_s"],c["next_hs_s"])})
    duration=sum(c["to_s"]-c["hs_s"] for c in cycles)
    # Pool squared errors before RMS; preserve actual stance durations.
    rms={n:float(np.sqrt(sum(r["stance"]["per_marker"][n]["RMS_3D_m"]**2*r["stance"]["duration_s"] for r in marker_cycles)/duration)) for n in names}
    pooled={"total_stance_duration_s":duration,"per_marker_RMS_3D_m":rms,
            "selected12_left_markers_mean_of_RMS_m":float(np.mean([rms[n] for n in calibration.DEFAULT_MARKERS])),
            "all28_markers_mean_of_RMS_m":float(np.mean(list(rms.values())))}
    return {"validation":{"status":"PASS","ankle_parent_child_footorigin_max_abs_error_m":center_error},
            "marker_fit":{"cycles":marker_cycles,"pooled_stance":pooled,"meaning":"Reference prepared-model markers versus acquired converted marker positions; not a healthy/prosthetic hardware point identity"},
            "COM_boundaries":list(com.values())}


def com_budget(times,force,right,measured_left,cycles,mass,gravity,rows):
    endpoints={r["time_s"]:r for r in rows}
    def one(start,end):
        duration=end-start
        momentum=mass*(endpoints[end]["COM_velocity_m_s"][1]-endpoints[start]["COM_velocity_m_s"][1])
        weight=-mass*gravity[1]*duration
        jr=phase.integral(times,right,start,end);jl=phase.integral(times,force,start,end)
        jm=phase.integral(times,measured_left,start,end)
        required=weight+momentum-jr
        return {"window_s":[start,end],"duration_s":duration,"gravity_impulse_Ns":weight,"COM_vertical_momentum_change_Ns":momentum,
                "right_measured_impulse_Ns":jr,"left_measured_impulse_Ns":jm,"left_virtual_contact_impulse_Ns":jl,
                "left_required_if_right_measured_and_no_external_pelvis_support_Ns":required,
                "external_vertical_support_required_by_reference_and_contact_Ns":required-jl,
                "external_vertical_support_required_mean_N":(required-jl)/duration,
                "actual_dynamic_reserve_impulse_Ns":None,"actual_tracking_error":None}
    return {"equation":"J_virtual_left + J_measured_right + J_external_support - m*g*T = m*Delta(COM_y_velocity)",
            "interpretation":"Reference kinematic vertical momentum compatibility only; hypothetical support budget, not actual reserve force, dynamics or a qualification result",
            "aggregate":one(cycles[0]["hs_s"],cycles[-1]["next_hs_s"]),
            "cycles":[dict(c,**one(c["hs_s"],c["next_hs_s"])) for c in cycles]}


def scalar_case(case):
    a=case["contact"]["aggregate"]
    return {"contact_mean_force_N":a["mean_force_N"],"contact_total_impulse_Ns":a["total_impulse_Ns"],
            "basis07_total_impulse_Ns":a["basis07"]["total_impulse_Ns"],"basis07_positive_penetration_mean_stance_m":a["depth"]["basis07_positive_penetration"]["mean_stance_m"],
            "ankle_center_Y_mean_stance_m":a["depth"]["ankle_center_Y"]["mean_stance_m"],
            "sole_clearance_mean_stance_m":a["depth"]["mesh_sole_clearance"]["mean_stance_m"],
            "contact_swing_impulse_fraction":a["swing_fraction_of_total_impulse"],
            "left12_marker_mean_stance_RMS_m":case["pose_diagnostics"]["marker_fit"]["pooled_stance"]["selected12_left_markers_mean_of_RMS_m"],
            "reference_external_support_impulse_Ns":case["reference_COM_budget"]["aggregate"]["external_vertical_support_required_by_reference_and_contact_Ns"]}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--all-cases-complete",action="store_true")
    args=parser.parse_args()
    if not args.all_cases_complete:parser.error("Wait for parent confirmation that all four fresh calibration/IK cases have completed")
    output=AUDIT/"variant_contact_comparison.json"
    if output.exists():raise FileExistsError("Preserve existing comparison; version explicitly")
    protocol=read(PROTOCOL)
    if sys.executable!=protocol["backend"]["python"] or str(calibration.osim.GetVersionAndDate())!=protocol["backend"]["opensim_version"]:
        raise AssertionError("Frozen preparation/replay backend differs")
    paths={ROOT/p for p in protocol["protected_sources_before"]}
    paths.update([Path(__file__),PROTOCOL,OLD/"contact_replay.py",CAUSAL/"alternative/phase_contact_budget.py",OLD/"contact_replay.json",
                  ROOT/"results/AB07_paired_prescribed_HS_9s_2026_10_07/receipt.json",ROOT/"results/AB07_paired_prescribed_HS_9s_2026_10_07/sim_outputs/sim_output_states.sto"])
    receipts={}
    for name,c in protocol["cases"].items():
        case_dir=ROOT/c["case_dir"];receipt=read(case_dir/"receipt.json")
        if receipt["status"]!="completed" or receipt["case"]!=c or receipt["protocol"]!=fp(PROTOCOL):
            raise AssertionError("All four receipts must attest completed frozen cases")
        for key in ("output_model","full_IK","actual_solver_model_copy","actual_solver_marker_copy","actual_solver_log","static_motion"):
            if fp(ROOT/receipt[key]["path"])!=receipt[key]:raise AssertionError("Case artifact provenance hash mismatch")
        if receipt["backend_contract"]["opensim_cmd"] is not None or receipt["backend_contract"]["run_smoke"] or not receipt["backend_contract"]["ik_only"]:
            raise AssertionError("Required reference-only fullIK backend differs")
        paths.update(p for p in case_dir.rglob("*") if p.is_file())
        receipts[name]=receipt
    for raw,sha in protocol["protected_sources_before"].items():
        if fp(ROOT/raw)["sha256"]!=sha:raise AssertionError("Original source changed")
    before={fp(p)["path"]:fp(p)["sha256"] for p in sorted(paths)}
    selection=read(PAIR/"selection.json");selected=selection["subjects"]["AB07"];cycles=selected["primary_cycles"]
    native,_,_=cr.read_storage(ROOT/"results/AB07_paired_prescribed_HS_9s_2026_10_07/sim_outputs/sim_output_states.sto")
    knots=np.unique([c[k] for c in cycles for k in ("hs_s","to_s","next_hs_s")]);start,end=knots[0],knots[-1]
    times=np.unique(np.r_[native[(native>start)&(native<end)],knots])
    if len(knots)!=7:raise AssertionError("Require exact seven frozen event knots")
    mt,mc,_=cr.read_storage(ROOT/selected["source_fingerprints"]["measured_GRF"]["path"])
    measured=interp_supported(mt,mc[selected["left_force_column"]],times)
    right=interp_supported(mt,mc["ground_force2_vy"],times)
    source_trc=ROOT/protocol["source_roles"]["full_trial_TRC"]["path"]
    trc=calibration.parse_trc(source_trc)
    frozen_receipt=read(ROOT/"results/AB07_paired_prescribed_HS_9s_2026_10_07/receipt.json")
    raw=read(cr.PRIMARY)
    profile=cr.OnlineGRFProfile.from_mapping(dict(raw,spheres=[s for s in raw["spheres"] if s["side"]=="left"]),required_sides=("left",))
    law=cr.ContactLaw(profile);sphere_names=[s.name for s in profile.spheres];i07=sphere_names.index("left_basis_07")
    if len(sphere_names)!=8 or not np.array_equal(law.normal,[0.,1.,0.]):raise AssertionError("Frozen primary contact geometry differs")
    for field,path in (("online_grf_profile_file",cr.PRIMARY),("binary_phase_detector_profile_file",cr.BINARY)):
        if frozen_receipt["profile_association"][field]["sha256"]!=fp(path)["sha256"]:raise AssertionError("Frozen profile receipt hash mismatch")
    binary=read(cr.BINARY);hull,_=cr.mesh_hull_vertices()
    cr._load_plugin(str(ROOT/"plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"));cr._load_plugin(str(ROOT/"plugins/OnlineGRFContact"))
    cases={};poses={}
    for name,receipt in receipts.items():
        print("MOUNT VARIANT FK",name,flush=True)
        model_path=ROOT/receipt["output_model"]["path"];ik=ROOT/receipt["full_IK"]["path"]
        cr._configure_geometry_search_paths(str(model_path));model=cr.opensim.Model(str(model_path))
        if model.getConstraintSet().getSize():raise ValueError("Explicit unconstrained FK required")
        cr.add_online_grf_forces(model,profile,applies_force=False,name_prefix="contact_replay_")
        state=model.initSystem();coords=[model.getCoordinateSet().get(i) for i in range(model.getCoordinateSet().getSize())];names=[c.getName() for c in coords]
        cfg_data=dict(frozen_receipt["effective_simulator_config"])
        expected={"enable_kinematics_lowpass_filter":True,"kinematics_lowpass_cutoff_hz":6.,"kinematics_lowpass_order":4,"kinematics_resample_dt":.001}
        if any(cfg_data[k]!=v for k,v in expected.items()):raise AssertionError("Original production full-trial filtering differs")
        cfg_data.update(model_bundle_dir=str(model_path.parent),model_file=str(model_path),kinematics_file=str(ik),external_loads_xml=str(ROOT/selected["source_fingerprints"]["external_loads"]["path"]))
        raw_t,raw_columns,raw_header=cr.read_storage(ik)
        if len(raw_columns)!=21 or set(raw_columns)!=set(names) or "indegrees=yes" not in raw_header.lower():
            raise AssertionError("Raw fullIK must contain exactly21unique coordinate columns in degrees")
        interp=cr.KinematicsInterpolator(cr.SimulatorConfig(**cfg_data))
        if set(interp.coord_names)!=set(names) or len(names)!=21 or len(interp.coord_names)!=21 or len(set(interp.coord_names))!=21:
            raise AssertionError("Coherent full21unique coordinate model/IK required")
        if start<interp.time_bounds[0] or end>interp.time_bounds[1]:raise ValueError("Full-trial IK lacks primary support")
        q=np.column_stack([interp._splines[n](times) for n in names]);qd=np.column_stack([interp._splines[n](times,1) for n in names])
        poses[name]={"names":names,"q":q,"qdot":qd}
        values,parity=cr.replay(model,state,coords,model.getBodySet().get("foot_l"),law,times,q,qd,hull,binary,set(np.linspace(0,len(times)-1,11,dtype=int)))
        if parity["cpp_max_abs_force_error_n"]>1e-8 or parity["station_transform_max_abs_error_m_or_m_s"]>1e-10:raise AssertionError("Inherited C++/NumPy parity failed")
        parity["cpp_metric_units_note"]="Inherited max aggregates force/normal errors[N] and penetration[m]; numeric parity diagnostic, not a dimensionally homogeneous error statistic or qualification gate"
        f=values["force"][...,1].sum(axis=1);b07=values["force"][:,i07,1]
        contact=phase.budget(times,f,measured,cycles,selected["BW_normalization_n"],
                             {"ankle_center_Y":values["foot_origin_y"],"mesh_sole_clearance":values["mesh_sole_clearance"],
                              "basis07_positive_penetration":values["penetration"][:,i07],"maximum_positive_penetration":values["penetration"].max(axis=1)},b07)
        contact["contact_summary"]=cr.summarize(times,values,measured,cycles,selected["BW_normalization_n"],sphere_names)
        for c in contact["cycles"]:
            for row in c["stance_phase_bins"]:
                a,b=row["time_window_s"];row["basis07_force_mean_N"]=phase.integral(times,b07,a,b)/(b-a)
        for i,row in enumerate(contact["pooled_stance_phase_bins"]):
            cb=[c["stance_phase_bins"][i] for c in contact["cycles"]]
            row["basis07_force_mean_N"]=sum(r["basis07_force_mean_N"]*r["duration_s"] for r in cb)/sum(r["duration_s"] for r in cb)
        diagnostics=pose_diagnostics(model,state,coords,times,q,qd,trc,cycles)
        mass=float(model.getTotalMass(state));gravity=cr.v3(model.getGravity())
        if abs(mass-selected["model_mass_kg_from_XML"])>1e-10:raise AssertionError("Source mass convention changed")
        budget=com_budget(times,f,right,measured,cycles,mass,gravity,diagnostics["COM_boundaries"])
        cases[name]={"case":receipt["case"],"receipt":fp(ROOT/receipt["case"]["case_dir"]/"receipt.json"),"model":fp(model_path),"full_IK":fp(ik),
                     "mass_kg":mass,"gravity_m_s2":gravity.tolist(),"contact":contact,"pose_diagnostics":diagnostics,"reference_COM_budget":budget,"validation":parity}
    current=cases["L1_A1"];old=read(OLD/"contact_replay.json")["subjects"]["AB07"]["cases"]["filtered_IK"]["aggregate"]
    endpoint={k:current["contact"]["contact_summary"]["aggregate"][k]-old[k] for k in ("impulse_ns","mean_force_n","mean_foot_origin_y_m")}
    if max(abs(x) for x in endpoint.values())>1e-8:raise AssertionError("Current coherent endpoint does not reproduce original filtered FK")
    reference=poses["L1_A1"]
    for name,pose in poses.items():
        if pose["names"]!=reference["names"]:raise AssertionError("Variant coordinate order differs")
        delta=pose["q"]-reference["q"]
        pose_change={"meaning":"Prepared reference q difference versus current case, not actual dynamic tracking error", "coordinate_names":pose["names"],
                     "max_abs_q_difference_by_coordinate":dict(zip(pose["names"],np.max(np.abs(delta),axis=0).tolist())),
                     "RMS_q_difference_by_coordinate":dict(zip(pose["names"],np.sqrt(INTEGRATE(delta*delta,times,axis=0)/(end-start)).tolist())),
                     "mean_q_difference_by_coordinate":dict(zip(pose["names"],(INTEGRATE(delta,times,axis=0)/(end-start)).tolist()))}
        cases[name]["reference_pose_difference_vs_current"]=pose_change
    scalar={k:scalar_case(v) for k,v in cases.items()}
    def difference(a,b):return {k:scalar[b][k]-scalar[a][k] for k in scalar[a]}
    effects={"length_long_minus_short_at_angle0":difference("L0_A0","L1_A0"),"length_long_minus_short_at_angle1":difference("L0_A1","L1_A1"),
             "static_angle1_minus0_at_length0":difference("L0_A0","L0_A1"),"static_angle1_minus0_at_length1":difference("L1_A0","L1_A1"),
             "joint_endpoint_current_minus_unregistered":difference("L0_A0","L1_A1")}
    effects["interaction_difference_of_conditional_length_effects"]={k:effects["length_long_minus_short_at_angle1"][k]-effects["length_long_minus_short_at_angle0"][k] for k in scalar["L0_A0"]}
    after={fp(p)["path"]:fp(p)["sha256"] for p in sorted(paths)}
    if before!=after:raise AssertionError("Consumed/protected source changed during FK analysis")
    result={"created_utc":datetime.now(timezone.utc).isoformat(),"instruction_check_token":"CMC_AGENT_OK_2026","status":"PASS_reference_preparation_FK_only","helper":fp(__file__),"protocol":fp(PROTOCOL),
            "methodology":{"coherent2x2":"Each mount/static-angle case has freshly calibrated12left markers and its own full marker IK; no mixed model-motion cells",
                           "grid":{"primary_s":[start,end],"samples":len(times),"exact7events_s":knots.tolist(),"source":"Frozen paired native1ms interior, unchanged selection"},
                           "phase":"Original measured healthy-GRF nominal stance/swing;20 bins descriptive only, actual duration used in all impulses",
                           "contact_profile":fp(cr.PRIMARY),"current_endpoint_filtered_FK_errors":endpoint,"filtering":expected,
                           "markers":"All28 reference residuals against acquiredTRC; left12 recalibrated subset separately",
                           "COM":"Velocity-stage FK at seven exact events, integral momentum change; no acceleration finite differences or actual reserve claims"},
            "cases":cases,"scalar_comparison":scalar,"conditional_procedural_differences":effects,
            "limitations":["Effects are conditional finite differences of preparation response, not causal percentage shares of anatomy or clinical hardware necessity.",
                           "A marker/IK solver may compensate mount length through joint motion; small marker residuals do not independently prove mechanical placement correctness.",
                           "Contact uses a virtual prosthetic empirical basis while nominal stance is from measured healthy GRF; incompatibility does not alone diagnose registration error.",
                           "No new forward dynamics were run. Reference pose differences and COM support requirements are not actual tracking errors or reserve outputs.",
                           "All old paired rollout outcomes and frozen windows remain preserved; no endpoint replacement or subject qualification is inferred."],
            "inputs_integrity":{"original_protected_count":protocol["protected_count"],"consumed_count":len(before),"before":before,"after":after,"all_unchanged":True}}
    with output.open("x",encoding="utf-8") as stream:
        json.dump(result,stream,indent=2,ensure_ascii=False,allow_nan=False);stream.write("\n")
    print(json.dumps({"output":fp(output),"scalars":scalar,"effects":effects},allow_nan=False),flush=True)


if __name__=="__main__":main()
