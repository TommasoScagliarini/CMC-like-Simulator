"""Fixed-profile FK screening of coherent pylon preparations; no dynamics.

Four endpoints reuse reviewed exact prior FK; ten fresh cases replay their own
new calibrated model/fullIK. All14 dynamic trials are already predeclared;
this screening never selects or fits a profile or changes a model.
"""
from datetime import datetime,timezone
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
AUDIT=HERE.parent
ROOT=AUDIT.parents[1]
OLD=ROOT/"validation/ab07_mount_registration_verification_2026_10_07"
PAIR=ROOT/"Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07"
PROTOCOL=HERE/"preparation_protocol.json"
sys.path.insert(0,str(ROOT))
spec=importlib.util.spec_from_file_location("pylon_screening_original_metrics",OLD/"variants/compare_mount_variants.py")
previous=importlib.util.module_from_spec(spec);sys.modules[spec.name]=previous;spec.loader.exec_module(previous)
cr,phase,calibration=previous.cr,previous.phase,previous.calibration
INTEGRATE=np.trapezoid if hasattr(np,"trapezoid") else np.trapz


def fp(p):
    p=Path(p).resolve();return {"path":p.relative_to(ROOT).as_posix(),"sha256":hashlib.sha256(p.read_bytes()).hexdigest()}


def read(p):return json.loads(Path(p).read_text(),parse_constant=lambda s:(_ for _ in ()).throw(ValueError(s)))


def config(receipt,model,ik,selected):
    data=dict(receipt["effective_simulator_config"])
    expected={"enable_kinematics_lowpass_filter":True,"kinematics_lowpass_cutoff_hz":6.,"kinematics_lowpass_order":4,"kinematics_resample_dt":.001}
    if any(data[k]!=v for k,v in expected.items()):raise AssertionError("Production full-trial filtering differs")
    data.update(model_bundle_dir=str(model.parent),model_file=str(model),kinematics_file=str(ik),external_loads_xml=str(ROOT/selected["source_fingerprints"]["external_loads"]["path"]))
    return cr.SimulatorConfig(**data)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--all-preparations-complete',action='store_true');args=parser.parse_args()
    if not args.all_preparations_complete:parser.error('Wait for all14 completed coherent preparation receipts')
    output=HERE/'contact_replay_screening.json'
    if output.exists():raise FileExistsError('Preserve previous FK screen')
    protocol=read(PROTOCOL);batch=read(HERE/'batch_receipt.json')
    if batch['status']!='completed' or len(batch['cases'])!=14:raise AssertionError('All14 preparations must be completed')
    paths={ROOT/p for p in protocol['protected_sources_before']}
    old_comparison_path=OLD/'variant_contact_comparison.json'
    paths.update([Path(__file__),PROTOCOL,HERE/'batch_receipt.json',HERE/'build_all_cases.py',old_comparison_path,OLD/'variants/compare_mount_variants.py'])
    receipts={}
    for name,case in protocol['cases'].items():
        path=ROOT/case['case_dir']/'receipt.json';r=read(path)
        if r['status']!='completed' or r['case']!=case or r['protocol']!=fp(PROTOCOL):raise AssertionError('Preparation receipt/protocol mismatch')
        for role in ('output_model','full_IK','actual_solver_setup','actual_solver_model_copy','actual_solver_marker_copy','actual_solver_log'):
            if fp(ROOT/r[role]['path'])!=r[role]:raise AssertionError('Consumed preparation model/IK/solver artifact changed')
        paths.update(p for p in path.parent.rglob('*') if p.is_file())
        paths.update(ROOT/r[role]['path'] for role in ('output_model','full_IK','actual_solver_setup','actual_solver_model_copy','actual_solver_marker_copy','actual_solver_log'))
        receipts[name]=r
    for raw,sha in protocol['protected_sources_before'].items():
        if fp(ROOT/raw)['sha256']!=sha:raise AssertionError('Original protected input changed')
    before={fp(p)['path']:fp(p)['sha256'] for p in sorted(paths)}
    selection=read(PAIR/'selection.json');selected=selection['subjects']['AB07'];cycles=selected['primary_cycles']
    native,_,_=cr.read_storage(ROOT/'results/AB07_paired_prescribed_HS_9s_2026_10_07/sim_outputs/sim_output_states.sto')
    knots=np.unique([c[k] for c in cycles for k in ('hs_s','to_s','next_hs_s')]);start,end=knots[0],knots[-1]
    times=np.unique(np.r_[native[(native>start)&(native<end)],knots])
    if len(knots)!=7 or start<native[0] or end>native[-1]:raise AssertionError('Frozen exact-event/native grid support differs')
    raw=read(cr.PRIMARY);profile=cr.OnlineGRFProfile.from_mapping(dict(raw,spheres=[s for s in raw['spheres'] if s['side']=='left']),required_sides=('left',))
    law=cr.ContactLaw(profile);sphere_names=[s.name for s in profile.spheres];i07=sphere_names.index('left_basis_07')
    frozen_receipt=read(ROOT/'results/AB07_paired_prescribed_HS_9s_2026_10_07/receipt.json')
    for field,p in (('online_grf_profile_file',cr.PRIMARY),('binary_phase_detector_profile_file',cr.BINARY)):
        if frozen_receipt['profile_association'][field]['sha256']!=fp(p)['sha256']:raise AssertionError('Frozen profile identity differs')
    old_comparison=read(old_comparison_path)
    old_grid=old_comparison['methodology']['grid']
    expected_filter={'enable_kinematics_lowpass_filter':True,'kinematics_lowpass_cutoff_hz':6.,'kinematics_lowpass_order':4,'kinematics_resample_dt':.001}
    if old_comparison['methodology']['filtering']!=expected_filter:raise AssertionError('Prior endpoint filter differs from exact production screening contract')
    if old_grid['primary_s']!=[start,end] or old_grid['samples']!=len(times) or old_grid['exact7events_s']!=knots.tolist() or old_comparison['methodology']['contact_profile']!=fp(cr.PRIMARY):
        raise AssertionError('Endpoint FK cannot be reused on a different grid/profile')
    mt,mc,_=cr.read_storage(ROOT/selected['source_fingerprints']['measured_GRF']['path'])
    measured=previous.interp_supported(mt,mc[selected['left_force_column']],times);right=previous.interp_supported(mt,mc['ground_force2_vy'],times)
    trc=calibration.parse_trc(ROOT/protocol['source_roles']['full_trial_TRC']['path'])
    if sys.executable!=protocol['backend']['python'] or str(calibration.osim.GetVersionAndDate())!=protocol['backend']['opensim_version']:raise AssertionError('Frozen preparation/replay backend differs')
    cr._load_plugin(str(ROOT/'plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff'));cr._load_plugin(str(ROOT/'plugins/OnlineGRFContact'))
    hull,_=cr.mesh_hull_vertices();binary=read(cr.BINARY)
    reference_model=ROOT/receipts['P4_A1']['output_model']['path'];reference_ik=ROOT/receipts['P4_A1']['full_IK']['path']
    ref=cr.KinematicsInterpolator(config(frozen_receipt,reference_model,reference_ik,selected));refnames=ref.coord_names
    if len(refnames)!=21 or len(set(refnames))!=21:raise AssertionError('Current reference coordinate schema differs')
    qref=np.column_stack([ref._splines[n](times) for n in refnames])
    cases={}
    for name,receipt in receipts.items():
        case=receipt['case']
        if case['preparation_kind']=='immutable_endpoint_reuse':
            endpoint=case['reused_endpoint']['reference_case_in_old_comparison'];prior=old_comparison['cases'][endpoint]
            if prior['model']!=receipt['output_model'] or prior['full_IK']!=receipt['full_IK']:raise AssertionError('Reused endpoint model/IK mismatch')
            result={k:prior[k] for k in ('contact','pose_diagnostics','reference_COM_budget','validation','reference_pose_difference_vs_current')}
            result['evaluation']='Reused immutable exact prior coherent FK, no new FK or calibration/solver for this endpoint'
        else:
            print('SWEEP FK',name,case['distal_mount_length_mm'],case['static_ankle_offset_deg'],flush=True)
            mp=ROOT/receipt['output_model']['path'];ik=ROOT/receipt['full_IK']['path']
            raw_t,raw_cols,raw_header=cr.read_storage(ik)
            cr._configure_geometry_search_paths(str(mp));model=cr.opensim.Model(str(mp))
            if model.getConstraintSet().getSize():raise AssertionError('Explicit unconstrained FK required')
            cr.add_online_grf_forces(model,profile,applies_force=False,name_prefix='contact_replay_');state=model.initSystem()
            coords=[model.getCoordinateSet().get(i) for i in range(model.getCoordinateSet().getSize())];names=[c.getName() for c in coords]
            if names!=refnames or len(raw_cols)!=21 or set(raw_cols)!=set(names) or 'indegrees=yes' not in raw_header.lower():raise AssertionError('Full21unique rawIK/model coordinate contract differs')
            interp=cr.KinematicsInterpolator(config(frozen_receipt,mp,ik,selected))
            if len(interp.coord_names)!=21 or len(set(interp.coord_names))!=21 or set(interp.coord_names)!=set(names) or start<interp.time_bounds[0] or end>interp.time_bounds[1]:raise AssertionError('Complete full-trial filtered support/schema differs')
            q=np.column_stack([interp._splines[n](times) for n in names]);qd=np.column_stack([interp._splines[n](times,1) for n in names])
            values,parity=cr.replay(model,state,coords,model.getBodySet().get('foot_l'),law,times,q,qd,hull,binary,set(np.linspace(0,len(times)-1,11,dtype=int)))
            if parity['cpp_max_abs_force_error_n']>1e-8 or parity['station_transform_max_abs_error_m_or_m_s']>1e-10:raise AssertionError('Inherited C++/station parity failed')
            parity['cpp_metric_units_note']='Inherited numeric maximum aggregates force/normal[N] and penetration[m]; implementation identity diagnostic, not homogeneous physical metric or qualification gate'
            f=values['force'][...,1].sum(axis=1);b07=values['force'][:,i07,1]
            contact=phase.budget(times,f,measured,cycles,selected['BW_normalization_n'],{'ankle_center_Y':values['foot_origin_y'],'mesh_sole_clearance':values['mesh_sole_clearance'],'basis07_positive_penetration':values['penetration'][:,i07],'maximum_positive_penetration':values['penetration'].max(axis=1)},b07)
            contact['contact_summary']=cr.summarize(times,values,measured,cycles,selected['BW_normalization_n'],sphere_names)
            for c in contact['cycles']:
                for row in c['stance_phase_bins']:
                    a,b=row['time_window_s'];row['basis07_force_mean_N']=phase.integral(times,b07,a,b)/(b-a)
            for i,row in enumerate(contact['pooled_stance_phase_bins']):
                bins=[c['stance_phase_bins'][i] for c in contact['cycles']];row['basis07_force_mean_N']=sum(b['basis07_force_mean_N']*b['duration_s'] for b in bins)/sum(b['duration_s'] for b in bins)
            diagnostics=previous.pose_diagnostics(model,state,coords,times,q,qd,trc,cycles)
            mass=float(model.getTotalMass(state));gravity=cr.v3(model.getGravity())
            if abs(mass-selected['model_mass_kg_from_XML'])>1e-10:raise AssertionError('Original mass convention differs')
            budget=previous.com_budget(times,f,right,measured,cycles,mass,gravity,diagnostics['COM_boundaries'])
            delta=q-qref
            result={'contact':contact,'pose_diagnostics':diagnostics,'reference_COM_budget':budget,'validation':parity,
                    'reference_pose_difference_vs_current':{'meaning':'Reference q changes, not actual tracking error','coordinate_names':names,
                        'max_abs_q_difference_by_coordinate':dict(zip(names,np.max(np.abs(delta),axis=0).tolist())),
                        'RMS_q_difference_by_coordinate':dict(zip(names,np.sqrt(INTEGRATE(delta*delta,times,axis=0)/(end-start)).tolist())),
                        'mean_q_difference_by_coordinate':dict(zip(names,(INTEGRATE(delta,times,axis=0)/(end-start)).tolist()))},
                    'evaluation':'Fresh coherent ownmodel/ownfullIK FK at Velocity stage only'}
        result.update(case=case,preparation_receipt=fp(ROOT/case['case_dir']/'receipt.json'),model=receipt['output_model'],full_IK=receipt['full_IK'])
        cases[name]=result
    scalar={name:dict(distal_mount_length_mm=c['case']['distal_mount_length_mm'],static_ankle_offset_deg=c['case']['static_ankle_offset_deg'],**previous.scalar_case(c)) for name,c in cases.items()}
    after={fp(p)['path']:fp(p)['sha256'] for p in sorted(paths)}
    if before!=after:raise AssertionError('Consumed/protected source changed during screening')
    result={'created_utc':datetime.now(timezone.utc).isoformat(),'instruction_check_token':'CMC_AGENT_OK_2026','status':'PASS_reference_FK_screening_only','helper':fp(__file__),'protocol':fp(PROTOCOL),
            'methodology':{'grid':{'primary_s':[start,end],'samples':len(times),'exact7events_s':knots.tolist()},'same_filter':'Original whole-trial6Hz order4 resample1ms; qdot spline derivative',
                           'fixed_contact_profile':fp(cr.PRIMARY),'reuse':'Four exact previous FK endpoints, own model+own fullIK, grid/profile/filter/bytebound identity validated',
                           'fresh_cells':'Ten independent12markerfits and fullIK, own coherent model/fullIK, no mixed cells',
                           'phase':'Measured healthy nominal stance/swing, real-time impulses and20 descriptive stancebins',
                           'prospective_dynamic_trials':'All14 already specified before this screen. No profile fitting or case selection by forcefit. Root amendment uses6.11s for recorder boundary support.'},
            'cases':cases,'scalar_comparison':scalar,'selection_or_profile_fit_performed':False,
            'limitations':['FK is reference contact response, not a forward dynamic result.','Reference COM external support is hypothetical, actual reserve/tracking are null.','Empirical virtual prosthetic spheres and healthy measured GRF are different contact systems.','The sweep cannot alone establish anatomical or clinical mount necessity.'],
            'inputs_integrity':{'original_protected_count':protocol['protected_count'],'consumed_count':len(before),'before':before,'after':after,'all_unchanged':True}}
    with output.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2,ensure_ascii=False,allow_nan=False);stream.write('\n')
    print(json.dumps({'output':fp(output),'scalar_comparison':scalar},allow_nan=False),flush=True)


if __name__=='__main__':main()
