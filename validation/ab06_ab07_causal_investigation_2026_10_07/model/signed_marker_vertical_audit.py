"""Signed marker vertical residuals on frozen primary cycles; FK only.

Models, IK and TRC inputs are read unchanged. This does not solve IK or integrate
dynamics. Static measurements are calibration data, not independent validation.
No experimental static force table is available in the subject static folders.
"""
from __future__ import annotations

from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.interpolate import CubicSpline

AUDIT = Path(__file__).resolve().parent
ROOT = AUDIT.parents[2]
PREVIOUS = ROOT/'validation/ab06_ab07_preparation_vs_model_2026_10_07'
sys.path.insert(0,str(ROOT))
from factorial_geometry_fk import fp, load_replay, reference, MODELS, PAIR

MARKERS = ('L_Ankle_Lat','L_Heel','L_Toe_Lat','L_Toe_Med','L_Toe_Tip')


def load_module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module
    spec.loader.exec_module(module)
    return module


def marker_contract(model_path):
    root=ET.parse(model_path).getroot()
    return {n:{'parent_frame':root.find(f"./Model/MarkerSet/objects/Marker[@name='{n}']/socket_parent_frame").text,
               'local_location_m':[float(x) for x in root.find(f"./Model/MarkerSet/objects/Marker[@name='{n}']/location").text.split()]}
            for n in MARKERS}


def interpolated_markers(trc,times):
    if trc.units.lower() not in ('m','mm') or times[0] < trc.times[0] or times[-1] > trc.times[-1]:
        raise AssertionError('Marker units or support differ')
    source=np.stack([trc.markers[n] for n in MARKERS],axis=1)
    if not np.isfinite(source).all() or np.any(np.diff(trc.times)<=0):
        raise AssertionError('Marker source nonfinite/unordered')
    return np.stack([np.column_stack([np.interp(times,trc.times,trc.markers[n][:,a]) for a in range(3)]) for n in MARKERS],axis=1)


def fk(cr,model_path,times,splines,hull,primary,binary,with_sole):
    cr._configure_geometry_search_paths(str(model_path))
    model=cr.opensim.Model(str(model_path))
    if model.getConstraintSet().getSize():
        raise AssertionError('Signed marker FK requires explicit unconstrained coordinates')
    state=model.initSystem()
    coords=[model.getCoordinateSet().get(i) for i in range(model.getCoordinateSet().getSize())]
    names=[c.getName() for c in coords]
    if set(names)!=set(splines):
        raise AssertionError('Motion/model coordinate schema differs')
    markers=[model.getMarkerSet().get(n) for n in MARKERS]
    predictions=np.empty((len(times),len(MARKERS),3))
    sole_primary=np.empty(len(times)) if with_sole else None
    sole_binary=np.empty(len(times)) if with_sole else None
    q_assignment_error=0.0
    for i,t in enumerate(times):
        q=np.array([splines[n](t) for n in names])
        d=np.array([splines[n](t,1) for n in names])
        cr.set_pose(model,state,coords,q,d,t)
        predictions[i]=np.array([cr.v3(m.getLocationInGround(state)) for m in markers])
        if i in (0,len(times)//2,len(times)-1):
            q_assignment_error=max(q_assignment_error,max(abs(c.getValue(state)-q[k]) for k,c in enumerate(coords)))
        if with_sole:
            transform=model.getBodySet().get('foot_l').getTransformInGround(state)
            vertices=hull@cr.matrix3(transform.R()).T+cr.v3(transform.p())
            sole_primary[i]=np.min((vertices-np.asarray(primary['origin']))@np.asarray(primary['normal']))
            sole_binary[i]=np.min((vertices-np.asarray(binary['origin']))@np.asarray(binary['normal']))
    if q_assignment_error != 0.0 or not np.isfinite(predictions).all():
        raise AssertionError('Silent q adjustment or nonfinite FK')
    return predictions,sole_primary,sole_binary,{'status':'PASS','q_assignment_max_abs_error':q_assignment_error,
                                              'coordinate_count':len(names),'FK_pose_count':len(times)}


def weighted_stats(cr,times,values,intervals):
    integrals=np.zeros(values.shape[1:]);squares=np.zeros_like(integrals)
    positive=np.zeros_like(integrals);sampled=[];duration=0.0
    for lo,hi in intervals:
        mask=(times>=lo)&(times<=hi);t=times[mask];v=values[mask]
        if abs(t[0]-lo)>1e-12 or abs(t[-1]-hi)>1e-12:
            raise AssertionError('Exact phase boundary missing')
        duration+=hi-lo;integrals+=cr.integrate(v,t);squares+=cr.integrate(v*v,t)
        positive+=cr.integrate((v>0).astype(float),t);sampled.append(v)
    mean=integrals/duration;rms=np.sqrt(squares/duration);std=np.sqrt(np.maximum(0,rms*rms-mean*mean))
    return {'duration_s':duration,'mean':mean.tolist(),'RMS':rms.tolist(),'std_about_mean':std.tolist(),
            'minimum':np.min(np.concatenate(sampled),axis=0).tolist(),
            'maximum':np.max(np.concatenate(sampled),axis=0).tolist(),
            'time_fraction_positive':(positive/duration).tolist(),
            'signed_bias_squared_fraction_of_mean_square':np.divide(mean*mean,rms*rms,out=np.zeros_like(mean),where=rms!=0).tolist()}


def summarize(cr,times,residual,sole_primary,sole_binary,cycles):
    groups={'all':[(cycles[0]['hs_s'],cycles[-1]['next_hs_s'])],
            'measured_stance':[(c['hs_s'],c['to_s']) for c in cycles],
            'measured_swing':[(c['to_s'],c['next_hs_s']) for c in cycles]}
    def one(intervals):
        vertical=weighted_stats(cr,times,residual[:,:,1]*1000,intervals)
        euclidean=weighted_stats(cr,times,np.linalg.norm(residual,axis=2)*1000,intervals)
        markers={n:{'signed_Y_mean_mm':vertical['mean'][i],'Y_RMS_mm':vertical['RMS'][i],
                    'Y_std_about_mean_mm':vertical['std_about_mean'][i],
                    'Y_min_mm':vertical['minimum'][i],'Y_max_mm':vertical['maximum'][i],
                    'time_fraction_predicted_above_measured':vertical['time_fraction_positive'][i],
                    'bias_squared_fraction_of_Y_mean_square':vertical['signed_bias_squared_fraction_of_mean_square'][i],
                    'euclidean_RMS_mm':euclidean['RMS'][i]} for i,n in enumerate(MARKERS)}
        result={'duration_s':vertical['duration_s'],'markers':markers,
                'five_marker_mean_signed_Y_mm':float(np.mean(vertical['mean'])),
                'five_marker_pooled_Y_RMS_mm':float(np.sqrt(np.mean(np.square(vertical['RMS'])))),
                'five_marker_pooled_euclidean_RMS_mm':float(np.sqrt(np.mean(np.square(euclidean['RMS']))))}
        if sole_primary is not None:
            result['virtual_hardware_STL_sole_to_primary_plane_m']=weighted_stats(cr,times,sole_primary,intervals)
            result['virtual_hardware_STL_sole_to_binary_plane_m']=weighted_stats(cr,times,sole_binary,intervals)
        return result
    return {'aggregate':{k:one(v) for k,v in groups.items()},
            'cycles':[{'cycle_index':i+1,'nominal_window_s':[c['hs_s'],c['next_hs_s']],
                       'all':one([(c['hs_s'],c['next_hs_s'])]),
                       'measured_stance':one([(c['hs_s'],c['to_s'])]),
                       'measured_swing':one([(c['to_s'],c['next_hs_s'])])} for i,c in enumerate(cycles)]}


def static_splines(table,cfg,converter):
    if table.names!=tuple(converter.IK_SOURCE_COLUMNS):
        raise AssertionError('Static healthy IK source schema differs')
    t=table.column('Header');names=table.names[1:]
    return {n:CubicSpline(t,table.column(n)*(1 if n in cfg.translation_coords else np.pi/180)) for n in names}


def main():
    output=AUDIT/'signed_marker_vertical_audit.json'
    if output.exists():
        raise FileExistsError('Preserve original signed marker audit')
    cr=load_replay()
    data_module=load_module('signed_marker_data_source',PREVIOUS/'data_preparation_audit.py')
    com_module=load_module('signed_marker_healthy_source',AUDIT.parent/'alternative/vertical_impulse_audit.py')
    data=json.loads((PREVIOUS/'data_preparation_audit_final.json').read_text())
    selection=json.loads((PAIR/'selection.json').read_text())
    previous=json.loads((AUDIT/'factorial_geometry_fk.json').read_text())
    paths={ROOT/raw for raw in previous['inputs_integrity']['after']}
    paths.update([Path(__file__),PREVIOUS/'data_preparation_audit.py',PREVIOUS/'data_preparation_audit_final.json',
                  AUDIT.parent/'alternative/vertical_impulse_audit.py',AUDIT/'factorial_geometry_fk.json'])
    inputs={}
    for subject in ('AB06','AB07'):
        day='10_09_18' if subject=='AB06' else '10_14_18'
        raw=ROOT/f'models/{subject}-raw/{day}'
        healthy=ROOT/f'models/{subject}-raw/osimxml'/('AB06.osim' if subject=='AB06' else 'ab07.osim')
        source=data['subjects'][subject]['sources']
        trc=next(ROOT/x['path'] for x in source if x['path'].endswith('treadmill_01_01.trc'))
        static_trc=next(ROOT/x['path'] for x in source if x['path'].endswith('.trc') and '/static/' in x['path'])
        static_motion=next(ROOT/x['path'] for x in source if '/static/' in x['path'] and x['path'].endswith('.mot'))
        if subject=='AB07':
            static_motion=ROOT/'models/AB07_SEASEA_Threadmill/data/fitting/AB07_SEASEA_static_ground_registered.mot'
        original=raw/'treadmill/ik/treadmill_01_01.mat'
        static_ik=raw/'static/ik'/('static_01.mat' if subject=='AB06' else 'static01.mat')
        static_marker_mat=raw/'static/markers'/('static_01.mat' if subject=='AB06' else 'static01.mat')
        inputs[subject]=dict(healthy=healthy,trc=trc,original=original,static_trc=static_trc,
                             static_motion=static_motion,static_ik=static_ik,static_marker_mat=static_marker_mat,raw_static_dir=raw/'static')
        paths.update([healthy,trc,original,static_trc,static_motion,static_ik,static_marker_mat])
    before={fp(p)['path']:fp(p)['sha256'] for p in sorted(paths)}
    for raw,expected in previous['inputs_integrity']['after'].items():
        if before[raw]!=expected:
            raise AssertionError('Previous protected source changed')
    cr._load_plugin(str(ROOT/'plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff'))
    hull,_=cr.mesh_hull_vertices()
    primary=json.loads(cr.PRIMARY.read_text())['ground'];binary=json.loads(cr.BINARY.read_text())['ground']
    subjects={}
    for subject,p in inputs.items():
        print('SIGNED MARKER FK',subject,flush=True)
        cycles=selection['subjects'][subject]['primary_cycles'];start,end=cycles[0]['hs_s'],cycles[-1]['next_hs_s']
        trc=data_module.parse_trc(p['trc'])
        times=np.unique(np.r_[trc.times[(trc.times>start)&(trc.times<end)],
                              [c[k] for c in cycles for k in ('hs_s','to_s','next_hs_s')]])
        measured=interpolated_markers(trc,times)
        receipt=json.loads((ROOT/f'results/{subject}_paired_prescribed_HS_9s_2026_10_07/receipt.json').read_text())
        cfg_dict=dict(receipt['effective_simulator_config'])
        prepared_IK=ROOT/selection['subjects'][subject]['source_fingerprints']['IK']['path']
        cfg_dict.update(model_bundle_dir=str(MODELS[subject].parent),model_file=str(MODELS[subject]),
                        kinematics_file=str(prepared_IK),external_loads_xml=str(ROOT/selection['subjects'][subject]['source_fingerprints']['external_loads']['path']))
        cfg=cr.SimulatorConfig(**cfg_dict)
        healthy_splines=com_module.original_splines(p['original'],cfg)
        prepared_splines=cr.KinematicsInterpolator(cfg)._splines
        cases={};raw_residual={}
        for name,model,splines,sole in [('healthy_original_IK_filtered',p['healthy'],healthy_splines,False),
                                        ('prepared_virtual_IK_filtered',MODELS[subject],prepared_splines,True)]:
            predicted,sp,sb,check=fk(cr,model,times,splines,hull,primary,binary,sole)
            residual=predicted-measured;raw_residual[name]=residual
            cases[name]={'model':fp(model),'marker_contract':marker_contract(model),'validation':check,
                         'residuals':summarize(cr,times,residual,sp,sb,cycles)}
        delta=raw_residual['prepared_virtual_IK_filtered']-raw_residual['healthy_original_IK_filtered']
        delta_summary=summarize(cr,times,delta,None,None,cycles)
        # Static comparison is row matched, preserving the known AB06 static
        # timestamp doubling. It is calibration training, not a held-out trial.
        strc=data_module.parse_trc(p['static_trc']);motion=data_module.parse_motion(p['static_motion'])
        stable,static_meta=data_module.generic_table(p['static_ik'])
        if len(strc.times)!=stable.nrows or not np.array_equal(strc.times,motion.times):
            raise AssertionError('Static row/converted marker-motion correspondence differs')
        healthy_static_splines=static_splines(stable,cfg,data_module.converter)
        healthy_t=stable.column('Header')
        prepared_static_splines={n:CubicSpline(motion.times,motion.values[:,i]*(1 if n in cfg.translation_coords else np.pi/180))
                                 for i,n in enumerate(motion.labels) if n!='time'}
        if not motion.in_degrees:
            raise AssertionError('Static calibration motion must declare degrees')
        static_cases={}
        static_observed=np.stack([strc.markers[n] for n in MARKERS],axis=1)
        for name,model,splines,t,sole in [('healthy_original_static_IK',p['healthy'],healthy_static_splines,healthy_t,False),
                                         ('prepared_static_calibration_pose',MODELS[subject],prepared_static_splines,strc.times,True)]:
            pred,sp,sb,check=fk(cr,model,t,splines,hull,primary,binary,sole)
            residual=pred-static_observed
            pseudo=[{'hs_s':float(t[0]),'to_s':float(t[len(t)//2]),'next_hs_s':float(t[-1])}]
            aggregate=summarize(cr,t,residual,sp,sb,pseudo)['aggregate']['all']
            static_cases[name]={'model':fp(model),'validation':check,'residuals':aggregate,
                                'time_range_s':[float(t[0]),float(t[-1])],'row_matched_not_cross_clock_interpolated':True}
        dynamic_minus_static={}
        for dynamic_name,static_name in [('healthy_original_IK_filtered','healthy_original_static_IK'),
                                         ('prepared_virtual_IK_filtered','prepared_static_calibration_pose')]:
            dynamic_minus_static[dynamic_name]={phase:{marker:
                cases[dynamic_name]['residuals']['aggregate'][phase]['markers'][marker]['signed_Y_mean_mm']-
                static_cases[static_name]['residuals']['markers'][marker]['signed_Y_mean_mm']
                for marker in MARKERS} for phase in ('all','measured_stance','measured_swing')}
        static_files=sorted(p['raw_static_dir'].glob('*/*'))
        fp_files=[x for x in static_files if x.is_file() and x.parent.name in ('fp','grf','force','forces')]
        subjects[subject]={'dynamic':cases,'prepared_minus_healthy_same_measured_marker_residual':delta_summary,
            'marker_data':fp(p['trc']),'dynamic_grid':{'native_marker_dt_s':float(np.median(np.diff(trc.times))),
                                                     'pose_count':len(times),'window_s':[start,end],
                                                     'boundary_interpolation':'Measured XYZ interpolated only at inserted exact HS/TO/nextHS; FK spline evaluated directly.'},
            'static_calibration_baseline':{'cases':static_cases,'source_MAT_IK':fp(p['static_ik']),
                                         'source_TRC':fp(p['static_trc']),'prepared_motion':fp(p['static_motion']),
                                         'source_meta':static_meta,'source_raw_time_range_s':[float(healthy_t[0]),float(healthy_t[-1])],
                                         'converted_marker_time_range_s':[float(strc.times[0]),float(strc.times[-1])],
                                         'fit_data_not_independent_validation':True},
            'dynamic_minus_own_static_signed_Y_mean_mm':dynamic_minus_static,
            'static_force_qualification':{'status':'unavailable','static_force_files':[],
                                          'reason':'Raw static contains marker/IK/ik_offset only; no static fp/GRF force table or validated force units, so physical static-force replay qualification is not inferred.'},
            'filter':{'enabled':cfg.enable_kinematics_lowpass_filter,'cutoff_hz':cfg.kinematics_lowpass_cutoff_hz,
                      'order':cfg.kinematics_lowpass_order,'resample_dt_s':cfg.kinematics_resample_dt}}
        if fp_files:
            raise AssertionError('Static force availability changed; review its units before classification')
        print(subject,json.dumps({name:{phase:{'five_marker_signed_Y_mean_mm':case['residuals']['aggregate'][phase]['five_marker_mean_signed_Y_mm'],
                                                    'five_marker_Y_RMS_mm':case['residuals']['aggregate'][phase]['five_marker_pooled_Y_RMS_mm'],
                                                    'per_marker_signed_Y_mean_mm':{n:case['residuals']['aggregate'][phase]['markers'][n]['signed_Y_mean_mm'] for n in MARKERS}}
                                           for phase in ('measured_stance','measured_swing')}
                                  for name,case in cases.items()},indent=2),flush=True)
    after={raw:fp(ROOT/raw)['sha256'] for raw in before}
    if before!=after:
        raise AssertionError('Protected source changed during marker FK')
    result={'schema_version':1,'created_utc':datetime.now(timezone.utc).isoformat(),
        'instruction_check_token':'CMC_AGENT_OK_2026','status':'PASS_signed_marker_FK_diagnostic',
        'helper':fp(__file__),'subjects':subjects,
        'methodology':{'sign':'Predicted marker ground Y minus measured marker Y; positive=predicted above, negative=predicted below.',
                       'units':'Marker xyz converted once from TRC declared m/mm to metres; summary residuals mm. Sole-plane clearance metres.',
                       'dynamic':'Healthy original23-coordinate IK and current prepared21-coordinate IK, production whole-trial6Hz4th-order zero-phase filter from effective receipt; no remap of healthy subtalar/MTP.',
                       'phases':'Frozen measured nominal HS→TO stance, TO→nextHS swing; all three cycles fixed. Not online FSM phases.',
                       'averaging':'Time-weighted trapezoidal statistics at native5ms marker frames plus exact phase boundaries; phases pooled by total duration.',
                       'static':'Unfiltered saved calibration motion and original static healthy IK, row matched to same converted marker rows. AB06 converted static time doubles raw time; no cross-clock interpolation.',
                       'planes':{'primary':primary,'binary':binary},
                       'sole':'Virtual hardware minimum plane clearance across convex-hull STL vertices; no anatomical healthy sole reconstruction or assumed healthy/prosthetic foot equivalence.',
                       'scope':'FK only; no IK solve, model modification, assembly, integration, contact force fit or policy inference.'},
        'interpretation_limits':['RMS includes squared bias but discards its sign; a signed mean and variance distinguish consistent offset from oscillating residual.',
                                 'Healthy markers on tibia_l/calcn_l and virtual remapped markers on foot_l represent different declared constructions. Their residual difference tests registration/IK consistency, not physical equality of healthy anatomy and prosthetic hardware.',
                                 'Static baseline markers were used in preparation calibration, so small static residuals do not independently validate dynamic alignment.',
                                 'No static force acquisition table is available; static marker/mesh agreement cannot establish force-law validity.',
                                 'Signed residuals and visual sole-plane height alone do not identify anatomical versus preparation versus contact-law causal shares.'],
        'inputs_integrity':{'status':'PASS','protected_count':len(before),'all_unchanged':before==after,'before':before,'after':after}}
    with output.open('x',encoding='utf-8') as handle:
        json.dump(result,handle,indent=2,allow_nan=False);handle.write('\n')
    print('PASS',output.relative_to(ROOT),flush=True)


if __name__=='__main__':
    main()
