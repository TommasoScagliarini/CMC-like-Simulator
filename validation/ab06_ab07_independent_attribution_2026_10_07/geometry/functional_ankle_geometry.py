"""Independent healthy/prepared functional-ankle FK on frozen nominal cycles.

OpenSim Velocity realization only. No assembly, dynamics, integration, IK solve,
coordinate clamp, policy or operational serialization. Original23-coordinate
healthy IK and prepared21-coordinate IK use recorded full-trial preprocessing.
"""
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

AUDIT = Path(__file__).resolve().parent
ROOT = AUDIT.parents[2]
PAIR = ROOT/'Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07'
REUSED = ROOT/'validation/ab06_ab07_causal_investigation_2026_10_07/alternative/vertical_impulse_audit.py'
sys.path.insert(0,str(ROOT))
spec = importlib.util.spec_from_file_location('independent_source_helpers',REUSED)
previous = importlib.util.module_from_spec(spec)
spec.loader.exec_module(previous)
opensim = previous.opensim
INTEGRATE = np.trapezoid if hasattr(np,'trapezoid') else np.trapz
N_BINS = 20


def fp(p):
    p = Path(p).resolve()
    return {'path':p.relative_to(ROOT).as_posix(),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}


def vector(value):
    return np.array([value.get(i) for i in range(3)])


def rotation(value):
    return np.array([[value.get(i,j) for j in range(3)] for i in range(3)])


def source_contract(path,kind):
    tree = ET.parse(path)
    name = 'ankle_l' if kind == 'healthy_original' else 'pros_ankle'
    joint = tree.find(f"./Model/JointSet/objects/*[@name='{name}']")
    frames = {f.get('name'):{'socket_parent':f.findtext('socket_parent'),
                            'translation_m':[float(x) for x in f.findtext('translation').split()],
                            'orientation_XYZ_rad':[float(x) for x in f.findtext('orientation').split()]}
              for f in joint.findall('./frames/*')}
    if kind == 'healthy_original':
        if joint.tag != 'CustomJoint':
            raise ValueError('Healthy ankle is not the declared CustomJoint.')
        axes = joint.findall('./SpatialTransform/TransformAxis')
        active = [a for a in axes if (a.findtext('coordinates') or '').strip()]
        if len(active) != 1 or active[0].findtext('coordinates') != 'ankle_angle_l':
            raise ValueError('Healthy joint axis/coordinate contract differs.')
        if [float(x) for x in active[0].findtext('LinearFunction/coefficients').split()] != [1.,0.]:
            raise ValueError('Healthy active rotation is not unit-positive linear.')
        for a in axes:
            if a is not active[0]:
                constant = a.find('Constant') or a.find('MultiplierFunction/function/Constant')
                # Element truthiness depends on child count; use explicit None.
                if constant is None:
                    constant = a.find('Constant')
                if constant is None or float(constant.findtext('value')) != 0:
                    raise ValueError('Additional joint transform is not constant0.')
        raw_axis = np.array([float(x) for x in active[0].findtext('axis').split()])
        coordinate,body,foot = 'ankle_angle_l','talus_l','calcn_l'
    else:
        if joint.tag != 'PinJoint':
            raise ValueError('Prepared ankle is not a PinJoint.')
        raw_axis = np.array([0.,0.,1.])
        coordinate,body,foot = 'pros_ankle_angle','foot_l','foot_l'
    parent_name,child_name = joint.findtext('socket_parent_frame'),joint.findtext('socket_child_frame')
    if frames[child_name]['translation_m'] != [0.,0.,0.]:
        raise ValueError('Declared coincident child-body origin differs from source.')
    return {'model':fp(path),'joint_name':name,'joint_type':joint.tag,
            'parent_frame':parent_name,'child_frame':child_name,'socket_frames':frames,
            'coordinate':coordinate,'positive_axis_in_parent_joint_frame_raw':raw_axis.tolist(),
            'positive_axis_in_parent_joint_frame_unit':(raw_axis/np.linalg.norm(raw_axis)).tolist(),
            'coincident_child_body_origin':body,'distal_foot_body_annotation':foot,
            'point_definition':'Functional joint center: origin of the joint parent socket frame, checked against the joint child socket origin at every pose.',
            'point_comparability':'Same functional category ankle center; not identical hardware, anatomical axis, marker or sole geometry.',
            'body_origin_contract':'Preparedfoot_l body origin coincides with pros_ankle center (childoffset0). Healthy talus_l origin coincides with ankle_l center; healthy calcn_l origin is a distinct distal-foot annotation, not the homologous point.'}


def fk(path,splines,times,kind):
    contract = source_contract(path,kind)
    previous._configure_geometry_search_paths(str(path))
    model = opensim.Model(str(path))
    if model.getConstraintSet().getSize():
        raise ValueError('Explicit unconstrained coordinates required.')
    state = model.initSystem()
    coords = [model.getCoordinateSet().get(i) for i in range(model.getCoordinateSet().getSize())]
    names = [c.getName() for c in coords]
    if set(names) != set(splines):
        raise ValueError('Full source/model coordinate names differ; no default fill/remap.')
    joint = model.getJointSet().get(contract['joint_name'])
    parent,child = joint.getParentFrame(),joint.getChildFrame()
    body = model.getBodySet().get(contract['coincident_child_body_origin'])
    foot = model.getBodySet().get(contract['distal_foot_body_annotation'])
    axis = np.array(contract['positive_axis_in_parent_joint_frame_unit'])
    center,axis_world,body_origin,foot_origin = [],[],[],[]
    coincidence = origin_error = axis_child_error = assigned_error = axis_derivative_error = 0.
    probes = set(np.linspace(0,len(times)-1,11,dtype=int).tolist())
    ankle = model.getCoordinateSet().get(contract['coordinate'])
    for i,t in enumerate(times):
        state.setTime(float(t))
        for c in coords:
            s = splines[c.getName()]
            q,d = float(s(t)),float(s(t,1))
            c.setValue(state,q,False)
            c.setSpeedValue(state,d)
            assigned_error = max(assigned_error,abs(c.getValue(state)-q),abs(c.getSpeedValue(state)-d))
        model.realizeVelocity(state)
        pt,ct = parent.getTransformInGround(state),child.getTransformInGround(state)
        p,c = vector(pt.p()),vector(ct.p())
        pr,cr = rotation(pt.R()),rotation(ct.R())
        ax = pr@axis
        center.append(p);axis_world.append(ax)
        bo,fo = vector(body.getTransformInGround(state).p()),vector(foot.getTransformInGround(state).p())
        body_origin.append(bo);foot_origin.append(fo)
        coincidence = max(coincidence,float(np.max(np.abs(p-c))))
        origin_error = max(origin_error,float(np.max(np.abs(bo-p))))
        axis_child_error = max(axis_child_error,float(np.max(np.abs(cr@axis-ax))))
        if i in probes:
            q0,h = ankle.getValue(state),1e-6
            matrices = []
            for delta in (h,-h):
                ankle.setValue(state,q0+delta,False)
                model.realizeVelocity(state)
                matrices.append(rotation(child.getTransformInGround(state).R()))
            omega = (matrices[0]-matrices[1])/(2*h)@cr.T
            numerical_axis = np.array([omega[2,1],omega[0,2],omega[1,0]])
            axis_derivative_error = max(axis_derivative_error,float(np.max(np.abs(numerical_axis-ax))))
            ankle.setValue(state,q0,False)
            model.realizeVelocity(state)
    result = {'joint_center_xyz_m':np.array(center),'positive_joint_axis_world':np.array(axis_world),
              'coincident_body_origin_xyz_m':np.array(body_origin),'distal_foot_body_origin_xyz_m':np.array(foot_origin)}
    if not all(np.all(np.isfinite(a)) for a in result.values()):
        raise ValueError('FK produced nonfinite geometry.')
    unit_error = float(np.max(np.abs(np.linalg.norm(result['positive_joint_axis_world'],axis=1)-1)))
    if max(coincidence,origin_error,axis_child_error,unit_error) > 1e-10 or assigned_error != 0 or axis_derivative_error > 1e-7:
        raise AssertionError('Point/axis/assignment identity failed.')
    result['coincident_body_origin_offset_from_joint_m'] = result['coincident_body_origin_xyz_m']-result['joint_center_xyz_m']
    result['distal_foot_body_origin_offset_from_joint_m'] = result['distal_foot_body_origin_xyz_m']-result['joint_center_xyz_m']
    validation = {'status':'PASS','poses':len(times),'coordinate_count':len(names),
                  'parent_child_center_max_abs_error_m':coincidence,
                  'coincident_child_body_origin_vs_center_max_abs_error_m':origin_error,
                  'parent_child_world_axis_max_abs_error':axis_child_error,'axis_unit_norm_max_abs_error':unit_error,
                  'requested_coordinate_value_speed_max_abs_error':assigned_error,
                  'axis_positive_rotation_finite_difference_max_abs_error':axis_derivative_error,'finite_difference_step_rad':1e-6,'axis_probes':len(probes),
                  'no_assembly_dynamics_clamping_or_default_coordinate_fill':True}
    return contract,result,validation


def interpolated_window(t,values,start,end):
    if start < t[0] or end > t[-1] or end <= start:
        raise ValueError('Requested interval not fully recorded; windows cannot change.')
    x = np.r_[start,t[(t>start)&(t<end)],end]
    return x,np.column_stack([np.interp(x,t,values[:,k]) for k in range(values.shape[1])])


def metric(t,arrays,windows):
    duration = sum(b-a for a,b in windows)
    out = {'duration_s':duration,'windows_s':[list(w) for w in windows]}
    for name,values in arrays.items():
        samples = [interpolated_window(t,values,a,b) for a,b in windows]
        average = sum((INTEGRATE(y,x,axis=0) for x,y in samples),np.zeros(values.shape[1]))/duration
        out[name+'_time_weighted_mean'] = average.tolist()
        if name == 'joint_center_xyz_m':
            out['joint_center_Y_time_weighted_mean_m'] = float(average[1])
            out['joint_center_Y_min_m'] = min(float(v[:,1].min()) for _,v in samples)
            out['joint_center_Y_max_m'] = max(float(v[:,1].max()) for _,v in samples)
        if name == 'positive_joint_axis_world':
            out['axis_world_Y_signed_time_weighted_mean'] = float(average[1])
            out['axis_world_Y_signed_min'] = min(float(v[:,1].min()) for _,v in samples)
            out['axis_world_Y_signed_max'] = max(float(v[:,1].max()) for _,v in samples)
    return out


def summarize(t,arrays,cycles):
    rows = []
    for c in cycles:
        hs,to,end = c['hs_s'],c['to_s'],c['next_hs_s']
        bins = []
        for i in range(N_BINS):
            a,b = hs+(to-hs)*i/N_BINS,hs+(to-hs)*(i+1)/N_BINS
            bins.append(dict(phase_start=i/N_BINS,phase_end=(i+1)/N_BINS,**metric(t,arrays,[(a,b)])))
        rows.append(dict(c,stance=metric(t,arrays,[(hs,to)]),swing=metric(t,arrays,[(to,end)]),stance_phase_bins=bins))
    stance_windows = [(c['hs_s'],c['to_s']) for c in cycles]
    swing_windows = [(c['to_s'],c['next_hs_s']) for c in cycles]
    aggregate = {'stance':metric(t,arrays,stance_windows),'swing':metric(t,arrays,swing_windows),
                 'all_primary':metric(t,arrays,[(cycles[0]['hs_s'],cycles[-1]['next_hs_s'])])}
    pooled = []
    for i in range(N_BINS):
        windows = [(r['stance_phase_bins'][i]['windows_s'][0][0],r['stance_phase_bins'][i]['windows_s'][0][1]) for r in rows]
        pooled.append(dict(phase_start=i/N_BINS,phase_end=(i+1)/N_BINS,**metric(t,arrays,windows)))
    return {'cycles':rows,'aggregate':aggregate,'pooled_stance_phase_bins':pooled}


def decomposition(healthy_a,prepared_a,healthy_b,prepared_b):
    keys = ('joint_center_xyz_m_time_weighted_mean','positive_joint_axis_world_time_weighted_mean',
            'coincident_body_origin_offset_from_joint_m_time_weighted_mean','distal_foot_body_origin_offset_from_joint_m_time_weighted_mean')
    rows = {}
    for key in keys:
        ha,pa,hb,pb = (np.array(x[key]) for x in (healthy_a,prepared_a,healthy_b,prepared_b))
        baseline,change_a,change_b = hb-ha,pa-ha,pb-hb
        observed = pb-pa
        introduced = change_b-change_a
        error = observed-baseline-introduced
        if np.max(np.abs(error)) > 1e-12:
            raise AssertionError('Exact before/after subject-difference identity failed.')
        rows[key] = {'healthy_AB07_minus_AB06':baseline.tolist(),'prepared_AB07_minus_AB06':observed.tolist(),
                     'prepared_minus_healthy_AB06':change_a.tolist(),'prepared_minus_healthy_AB07':change_b.tolist(),
                     'differential_construction_response_AB07_minus_AB06':introduced.tolist(),'sum_identity_error':error.tolist()}
    center = rows['joint_center_xyz_m_time_weighted_mean']
    rows['joint_center_Y_budget_mm'] = {k:1000*v[1] for k,v in center.items()}
    rows['axis_budget_scope'] = 'Vector identity only. HealthyCustomJoint oblique axis and prostheticPinZ have different contracts; not an identity of anatomical axes.'
    rows['distal_foot_budget_scope'] = 'Body-origin annotation only; healthycalcn_l and preparedfoot_l are distinct mechanisms, not homologous measured sole/marker points.'
    return rows


def main():
    output = AUDIT/'functional_ankle_geometry.json'
    if output.exists():
        raise FileExistsError('Preserve an existing sealed output; version separately.')
    selection = json.loads((PAIR/'selection.json').read_text())
    inputs = {}
    sources = {Path(__file__),REUSED,ROOT/'AGENTS.md',PAIR/'selection.json',ROOT/'config.py',ROOT/'path_resolver.py',
               ROOT/'kinematics_interpolator.py',ROOT/'model_loader.py',ROOT/'tools/convert_epic_ab06_tables.py'}
    for subject in ('AB06','AB07'):
        s = selection['subjects'][subject]
        run = ROOT/f'results/{subject}_paired_prescribed_HS_9s_2026_10_07'
        healthy = ROOT/f'models/{subject}-raw/osimxml'/('AB06.osim' if subject == 'AB06' else 'ab07.osim')
        day = '10_09_18' if subject == 'AB06' else '10_14_18'
        mat = ROOT/f'models/{subject}-raw/{day}/treadmill/ik/treadmill_01_01.mat'
        prepared,ik = (ROOT/s['source_fingerprints'][k]['path'] for k in ('model','IK'))
        sources.update([healthy,mat,prepared,ik,run/'receipt.json',run/'sim_outputs/sim_output_states.sto'])
        sources.update(ROOT/f['path'] for f in s['source_fingerprints'].values())
        for field,record in s['source_fingerprints'].items():
            if fp(ROOT/record['path']) != record:
                raise AssertionError('Original frozen selection source differs.')
        inputs[subject] = dict(healthy=healthy,mat=mat,prepared=prepared,ik=ik,run=run)
    for pattern in ('*SEA_Plugin_BlackBox_mCMC_impedence_ff*','*OnlineGRFContact*'):
        sources.update(p for p in (ROOT/'plugins').glob(pattern) if p.suffix in ('.dll','.so','.dylib'))
    before = {fp(p)['path']:fp(p)['sha256'] for p in sorted(sources)}
    previous._load_plugin(str(ROOT/'plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff'))
    previous._load_plugin(str(ROOT/'plugins/OnlineGRFContact'))
    subjects = {}
    for subject,p in inputs.items():
        s = selection['subjects'][subject]
        receipt = json.loads((p['run']/'receipt.json').read_text())
        cfgdict = dict(receipt['effective_simulator_config'])
        cfgdict.update(model_bundle_dir=str(p['prepared'].parent),model_file=str(p['prepared']),kinematics_file=str(p['ik']),
                       external_loads_xml=str(ROOT/s['source_fingerprints']['external_loads']['path']),
                       reserve_actuators_xml=str(ROOT/s['source_fingerprints']['reserve_actuators']['path']))
        cfg = previous.SimulatorConfig(**cfgdict)
        if (cfg.enable_kinematics_lowpass_filter,cfg.kinematics_lowpass_cutoff_hz,cfg.kinematics_lowpass_order,cfg.kinematics_resample_dt) != (True,6.,4,.001):
            raise ValueError('The agreed production preprocessing contract differs.')
        native_times,_,_ = previous.storage(p['run']/'sim_outputs/sim_output_states.sto')
        cycles = s['primary_cycles']
        start,end = cycles[0]['hs_s'],cycles[-1]['next_hs_s']
        if start < native_times[0] or end > native_times[-1]:
            raise ValueError('Primary physical support missing; no rewindowing.')
        boundaries = [c[k] for c in cycles for k in ('hs_s','to_s','next_hs_s')]
        times = np.unique(np.r_[native_times[(native_times>start)&(native_times<end)],boundaries])
        healthy_splines = previous.original_splines(p['mat'],cfg)
        prepared_splines = previous.KinematicsInterpolator(cfg)._splines
        cases = {}
        for kind,path,splines in [('healthy_original',p['healthy'],healthy_splines),('prepared_current',p['prepared'],prepared_splines)]:
            if times[0] < max(v.x[0] for v in splines.values()) or times[-1] > min(v.x[-1] for v in splines.values()):
                raise ValueError('Full source reference support missing.')
            print(subject,kind,'VelocityFK',len(times),'poses',flush=True)
            contract,arrays,validation = fk(path,splines,times,kind)
            cases[kind] = dict(contract=contract,validation=validation,**summarize(times,arrays,cycles))
        subjects[subject] = {'cases':cases,'original_IK_MAT':fp(p['mat']),'prepared_IK':fp(p['ik']),
                             'receipt_status':receipt['status'],'primary_support_s':[start,end],
                             'grid_samples':len(times),'preprocessing':{'full_source_trial':True,'cutoff_hz':6.,'order':4,'resample_dt_s':.001}}
    case = lambda subject,kind: subjects[subject]['cases'][kind]
    paired = {}
    for phase in ('stance','swing','all_primary'):
        paired[phase] = decomposition(case('AB06','healthy_original')['aggregate'][phase],case('AB06','prepared_current')['aggregate'][phase],
                                      case('AB07','healthy_original')['aggregate'][phase],case('AB07','prepared_current')['aggregate'][phase])
    pooled = []
    for i in range(N_BINS):
        pooled.append(dict(phase_start=i/N_BINS,phase_end=(i+1)/N_BINS,
                           **decomposition(case('AB06','healthy_original')['pooled_stance_phase_bins'][i],case('AB06','prepared_current')['pooled_stance_phase_bins'][i],
                                           case('AB07','healthy_original')['pooled_stance_phase_bins'][i],case('AB07','prepared_current')['pooled_stance_phase_bins'][i])))
    cycle_decomposition = []
    for i in range(3):
        cycle_decomposition.append({'nominal_cycle_index':i+1,
                                    'stance':decomposition(case('AB06','healthy_original')['cycles'][i]['stance'],case('AB06','prepared_current')['cycles'][i]['stance'],case('AB07','healthy_original')['cycles'][i]['stance'],case('AB07','prepared_current')['cycles'][i]['stance']),
                                    'swing':decomposition(case('AB06','healthy_original')['cycles'][i]['swing'],case('AB06','prepared_current')['cycles'][i]['swing'],case('AB07','healthy_original')['cycles'][i]['swing'],case('AB07','prepared_current')['cycles'][i]['swing'])})
    after = {fp(p)['path']:fp(p)['sha256'] for p in sorted(sources)}
    if before != after:
        raise AssertionError('An operational or reused source changed.')
    result = {'schema_version':1,'created_utc':datetime.now(timezone.utc).isoformat(),'instruction_check_token':'CMC_AGENT_OK_2026',
              'status':'PASS','helper':fp(Path(__file__)),'subjects':subjects,'exact_subject_difference_budget':paired,
              'per_cycle_subject_difference_budget':cycle_decomposition,'pooled_stance_phase_subject_difference_budget':pooled,
              'methodology':{'scope':'Independent direct model/IKFK, only Velocity stage. No newClaude audit directory or conclusions read.',
                             'homologous_point':'Jointsocketcenter healthyankle_l vs preparedpros_ankle, with parent/child coincidence validated at every pose. This is functional-category homology, not marker/axis/hardware equivalence.',
                             'source_motion':'Original23-coordinate healthyEPIC IK includesleftsubtalar/MTP; prepared21-coordinateown IK. Recorded6Hz/order4/1ms zero-phase full-source preprocessing in both.',
                             'time_windows':'Same3 immutable measured-force nominalcycles; fullprecisionHS/TO/nextHS+every saved native1ms interior. No startup substitution, new oracle or phase gate.',
                             'phase_bins':'20 equal measuredHS→TO stancebins. Within-cycle time averages, then duration-weighted pool; phase normalization descriptive only.',
                             'identity':'Prepared(B−A) = Healthy(B−A) + (PreparedB−HealthyB) − (PreparedA−HealthyA), computed on matching summaries. Vectorcenter/axis and signedY units explicitly separated.',
                             'positive_axis':'CustomJoint active oblique rotation axis or PinJointlocalZ; transformed by parentjointframe, checked against childaxis and numerical positive coordinate rotation.',
                             'body_origin':'Preparedfoot_l and healthytalus_l bodyorigin each coincide with their anklecenter; healthycalcn_l distalbodyorigin is a separate nonhomologous annotation.'},
              'limits':['The introduced term is an end-to-end geometrical construction/reference response. It combines mounting, joint replacement, original-versus-markerIK, registration and ownsubjectmotion; it is not an anatomy-only or disease causal effect.',
                        'Original healthy subject differences likewise combine geometry, own gait and ownIK/acquisition conventions. Samefunctionalanklecenter does not make subjects interchangeable.',
                        'Axisvector budgets are descriptive identities across distinct joint contracts, not an assertion that anatomical and prosthetic axes are identical.',
                        'Jointcenterheight is not soleclearance, spherepenetration or measuredmarkerheight. No equivalence with remapped prosthetic markers is used.',
                        'No independent floor measurement, hardware experiment or multi-subject statistical inference is introduced.',
                        'AB06 original terminal adapterfailure is preserved; only fully supported preselectedprimary physical windows are evaluated.'],
              'inputs_integrity':{'status':'PASS','protected_count':len(before),'all_unchanged':True,'fingerprints_before':before,'fingerprints_after':after}}
    with output.open('x',encoding='utf-8') as f:
        json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print('PASS',output.relative_to(ROOT))
    for phase,data in paired.items():
        print(phase,data['joint_center_Y_budget_mm'])


if __name__ == '__main__':
    main()
