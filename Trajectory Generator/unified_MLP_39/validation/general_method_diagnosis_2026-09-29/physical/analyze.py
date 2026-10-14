"""Offline matched physical-load audit for completed diagnostic rollout pairs.

Read STO and trace files only; no OpenSim model or trained actor is instantiated.
Use existing contact/penalty thresholds from each rollout, never new gates.
"""
from __future__ import annotations
import argparse
import hashlib
import io
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[2]
REPO=PACKAGE.parents[1]
RUNS=PACKAGE/'runs/rollout/general_method_diagnosis_2026-09-29'
WINDOW=(13.15,17.95)
HASHES={}


def read(path):
    raw=path.read_bytes()
    HASHES[str(path.relative_to(REPO))]=hashlib.sha256(raw).hexdigest()
    return raw.decode()


def table(path):
    raw=read(path).split('endheader',1)[1].strip()
    keys=raw.splitlines()[0].split()
    values=np.loadtxt(io.StringIO(raw),skiprows=1,ndmin=2)
    assert values.shape[1]==len(keys) and np.all(np.isfinite(values))
    assert np.all(np.diff(values[:,0])>0.)
    return dict(zip(keys,values.T))


def load(name):
    folder=RUNS/name
    # A summary may exist before final run_case validation; require its receipt.
    receipt=json.loads(read(folder/'diagnostic_receipt.json'))
    summary=json.loads(read(folder/'rollout_summary.json'))
    trace=json.loads(read(folder/'rollout_policy_trace.json'))
    assert receipt['checkpoint_unchanged'] is True
    assert receipt['steps']==summary['steps']==len(trace)
    tables={k:table(folder/'sim_outputs'/f'rollout_episode_{k}.sto') for k in
            ('online_grf','reserve_torques','sea_torques','sea_diagnostics')}
    available=[(v['time'][0],v['time'][-1]) for v in tables.values()]
    return dict(name=name,folder=folder,summary=summary,trace=trace,receipt=receipt,tables=tables,
                support=[max(a for a,b in available),min(b for a,b in available)])


def grid(tab,window):
    t=tab['time'];lo,hi=window
    assert t[0]-1e-9<=lo<hi<=t[-1]+1e-9
    return np.r_[lo,t[(t>lo+1e-9)&(t<hi-1e-9)],hi]


def integrate(t,y):
    return float(np.trapezoid(y,t) if hasattr(np,'trapezoid') else np.trapz(y,t))


def interpolate(tab,key,t):
    assert np.min(t)>=tab['time'][0]-1e-9 and np.max(t)<=tab['time'][-1]+1e-9
    return np.interp(t,tab['time'],tab[key])


def moments(t,y):
    duration=float(t[-1]-t[0])
    return {'mean':integrate(t,y)/duration,'rms':np.sqrt(integrate(t,y*y)/duration),
            'min':float(np.min(y)),'max':float(np.max(y)),'abs_max':float(np.max(abs(y)))}


def corr(a,b):
    if len(a)<2 or min(np.std(a),np.std(b))<1e-12:return None
    return float(np.corrcoef(a,b)[0,1])


def window_metrics(case,window):
    tab=case['tables'];grf=tab['online_grf'];rsv=tab['reserve_torques'];tau=tab['sea_torques'];diag=tab['sea_diagnostics']
    t=grid(grf,window);duration=float(t[-1]-t[0]);summary=case['summary']
    fy=interpolate(grf,'left_force_y',t);normal=interpolate(grf,'left_normal_force',t)
    penetration=interpolate(grf,'left_penetration',t)
    # At native recorder times this is the logged boolean, not a new force gate.
    np.testing.assert_allclose(grf['left_in_contact'],np.round(grf['left_in_contact']),atol=1e-8)
    contact=interpolate(grf,'left_in_contact',t)
    soft=float(summary['grf_penetration_penalty_threshold_m']);hard=float(summary['grf_penetration_termination_m'])
    force_threshold=float(summary['reward_config']['grf_ankle_moment_flip_force_threshold_n'])
    loaded=normal>=force_threshold
    load={'duration_s':duration,'left_vertical_force_n':moments(t,fy),
          'left_normal_force_n':moments(t,normal),'vertical_impulse_n_s':integrate(t,fy),
          'normal_impulse_n_s':integrate(t,normal),'positive_vertical_impulse_n_s':integrate(t,np.maximum(fy,0)),
          'logged_contact_duration_s':integrate(t,contact),'logged_contact_time_fraction':integrate(t,contact)/duration,
          'existing_load_threshold_n':force_threshold,'loaded_time_fraction':integrate(t,loaded.astype(float))/duration,
          'penetration_mm':moments(t,1000*penetration),
          'existing_penetration_soft_m':soft,'existing_penetration_hard_m':hard,
          'time_above_existing_soft_fraction':integrate(t,(penetration>soft).astype(float))/duration,
          'time_above_existing_hard_fraction':integrate(t,(penetration>hard).astype(float))/duration}
    if np.sum(contact)>0:
        load['normal_force_mean_while_logged_contact_n']=integrate(t,normal*contact)/integrate(t,contact)
    rt=grid(rsv,window)
    per_coordinate={k:moments(rt,interpolate(rsv,k,rt)) for k in rsv if k!='time'}
    translations=[f'pelvis_{axis}_reserve_torque' for axis in ('tx','ty','tz')]
    rotations=[f'pelvis_{axis}_reserve_torque' for axis in ('tilt','list','rotation')]
    local=['pros_knee_angle_reserve_torque','pros_ankle_angle_reserve_torque']
    biological=[k for k in per_coordinate if k not in translations+rotations+local]
    reserves={'units_note':'Columns named reserve_torque for pelvis_tx/ty/tz contain generalized forces in N; all other listed coordinates are rotational and use Nm. No mixed N/Nm norm is formed.',
              'per_coordinate':per_coordinate}
    for label,cols,unit in [('pelvis_translation',translations,'N'),('pelvis_rotation',rotations,'Nm'),('biological_rotation',biological,'Nm'),('prosthetic_rotation',local,'Nm')]:
        signals=np.array([interpolate(rsv,k,rt) for k in cols])
        magnitude=np.sqrt(np.sum(signals**2,axis=0))
        reserves[label]={'columns':cols,'unit':unit,'vector_magnitude':moments(rt,magnitude)}
    st=grid(tau,window);sea={}
    for joint in ('Knee','Ankle'):
        spring=interpolate(tau,f'SEA_{joint}_tau_spring',st)
        ref=interpolate(diag,f'SEA_{joint}_tau_ref',st)
        sea[joint.lower()]={'spring_torque_nm':moments(st,spring),
                           'spring_positive_area_nm_s':integrate(st,np.maximum(spring,0)),
                           'spring_negative_area_nm_s':integrate(st,np.minimum(spring,0)),
                           'spring_minus_command_nm':moments(st,spring-ref)}
    # The saved generalized ankle moment already projects the entire applied
    # online wrench onto the OpenSim ankle axis at the correct current state.
    trace=[r for r in case['trace'] if window[0]-1e-9<=r['time']<=window[1]+1e-9]
    tt=np.array([r['time'] for r in trace]);terms=[r['reward_terms'] for r in trace]
    valid=np.array([r['grf_ankle_moment_flip_available']>0 for r in terms])
    load_force=np.array([r['grf_ankle_moment_flip_force_n'] for r in terms])
    thresholds=np.array([r['grf_ankle_moment_flip_force_threshold_n'] for r in terms])
    np.testing.assert_allclose(thresholds,force_threshold,atol=1e-12)
    moment=np.array([r['grf_ankle_moment_flip_tau_nm'] for r in terms])
    spring=interpolate(tau,'SEA_Ankle_tau_spring',tt)
    loaded_mask=valid&(load_force>=force_threshold)
    selected=int(np.sum(loaded_mask));ankle={'trace_samples':len(tt),'available_samples':int(np.sum(valid)),
        'loaded_samples':selected,'existing_load_threshold_n':force_threshold,
        'sampling':'100 Hz trace endpoints on common STO support; no extrapolation',
        'sign_convention':'Native OpenSim coordinate; tau_GRF=ankle_axis dot (M_ground - ankle_center cross F). Compare spring to -tau_GRF.',
        'limitation':'spring+online_GRF_moment omits inertia, gravity and other couplings; it is not an inverse-dynamics residual or SEA tracking error.'}
    if selected:
        a=spring[loaded_mask];b=moment[loaded_mask]
        ankle.update({'spring_mean_nm':float(np.mean(a)),'spring_rms_nm':float(np.sqrt(np.mean(a*a))),
                      'online_grf_moment_mean_nm':float(np.mean(b)),'online_grf_moment_rms_nm':float(np.sqrt(np.mean(b*b))),
                      'corr_spring_opposite_online_grf_moment':corr(a,-b),
                      'spring_plus_online_grf_moment_mean_nm':float(np.mean(a+b)),
                      'spring_plus_online_grf_moment_rms_nm':float(np.sqrt(np.mean((a+b)**2))),
                      'cop_z_minus_ankle_z_m_mean':float(np.mean([terms[i]['grf_ankle_moment_flip_cop_z_minus_ankle_z_m'] for i in np.flatnonzero(loaded_mask)]))})
    return {'window_s':list(window),'load_contact_penetration':load,'reserves':reserves,
            'sea':sea,'loaded_ankle_external_moment':ankle}


def comparison(base,sequence):
    b=base['load_contact_penetration'];s=sequence['load_contact_penetration']
    out={}
    for key,bv,sv in [('normal_impulse_n_s',b['normal_impulse_n_s'],s['normal_impulse_n_s']),
                     ('vertical_impulse_n_s',b['vertical_impulse_n_s'],s['vertical_impulse_n_s']),
                     ('logged_contact_time_fraction',b['logged_contact_time_fraction'],s['logged_contact_time_fraction']),
                     ('penetration_max_mm',b['penetration_mm']['max'],s['penetration_mm']['max'])]:
        out[key]={'baseline':bv,'sequence':sv,'sequence_minus_baseline':sv-bv,
                  'sequence_over_baseline':sv/bv if abs(bv)>1e-12 else None}
    for group in ('pelvis_translation','pelvis_rotation','biological_rotation','prosthetic_rotation'):
        bv=base['reserves'][group]['vector_magnitude']['rms'];sv=sequence['reserves'][group]['vector_magnitude']['rms']
        out[group+'_reserve_vector_rms']={'unit':base['reserves'][group]['unit'],'baseline':bv,'sequence':sv,
            'sequence_minus_baseline':sv-bv,'sequence_over_baseline':sv/bv if abs(bv)>1e-12 else None}
    return out


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sides',nargs='+',choices=('LEFT','RIGHT'),default=['LEFT','RIGHT'])
    args=parser.parse_args()
    results={'scope':'Matched baseline versus deterministic offline endpoint sequence; no learned-policy or population-generalization claim.',
             'interpretation':'Descriptive loads and reserves assess unloading/assistance as possible confounds, not new pass/fail criteria.',
             'pairs':{},'limitations':[
                 'One reset and deterministic recorded trajectory per side; sequence has full future target available.',
                 'Only applied online LEFT contact is integrated. The right online contact is sensor-only; prescribed right forces are unchanged configuration, not reconstructed here.',
                 'Stored generalized moment exists only for ankle, at 100 Hz; no knee external-moment balance is inferred.',
                 'Reserve translational force and rotational torque magnitudes remain separate because their units differ.',
                 'Smaller reserves or larger contact impulse alone do not certify realistic biomechanics or robustness.',
                 'Contact duration uses the existing logged contact boolean; loaded diagnostics use the existing reward force threshold. No thresholds were selected from these results.']}
    for side in args.sides:
        base,seq=load(side+'_baseline'),load(side+'_sequence')
        common=[max(base['support'][0],seq['support'][0]),min(base['support'][1],seq['support'][1])]
        names=('grf_mode','online_grf_profile_file','online_grf_applied_sides','prescribed_grf_disabled_sides','grf_penetration_penalty_threshold_m','grf_penetration_termination_m')
        for key in names:assert base['summary'][key]==seq['summary'][key],key
        pair={'baseline':str(base['folder'].relative_to(REPO)),'sequence':str(seq['folder'].relative_to(REPO)),
              'steps_baseline':base['summary']['steps'],'steps_sequence':seq['summary']['steps'],
              'end_reason_baseline':base['summary']['end_reason'],'end_reason_sequence':seq['summary']['end_reason'],
              'same_configuration_fields_verified':list(names),'sequence_governor_receipt':seq['receipt'].get('offline_governor_vs_physical_served_max_error'),
              'summary_penetration_max_mm':{'baseline':1000*base['summary']['grf_penetration_max_m'],'sequence':1000*seq['summary']['grf_penetration_max_m']},'windows':{}}
        windows={'whole_common_recorded_support':common}
        interior=[max(WINDOW[0],common[0]),min(WINDOW[1],common[1])]
        if interior[1]>interior[0]:windows['matched_analysis']=interior
        for name,window in windows.items():
            bm=window_metrics(base,window);sm=window_metrics(seq,window)
            pair['windows'][name]={'baseline':bm,'sequence':sm,'comparison':comparison(bm,sm)}
        results['pairs'][side]=pair
    for path in (Path(__file__),REPO/'Trajectory Generator/osim_trj_cmc_like.py'):
        read(path)
    assert all(hashlib.sha256((REPO/p).read_bytes()).hexdigest()==h for p,h in HASHES.items())
    (HERE/'metrics.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
    (HERE/'input_hashes.json').write_text(json.dumps(HASHES,indent=2)+'\n')
    (HERE/'verification.json').write_text(json.dumps({'no_simulation':True,'no_training':True,'source_hashes_unchanged':True,
        'all_checkpoint_receipts_unchanged':True,'no_extrapolation':True,'no_mixed_units_reserve_norm':True},indent=2)+'\n')
    print(json.dumps({k:v['windows'].get('matched_analysis',{}) .get('comparison',{}) for k,v in results['pairs'].items()},indent=2))


if __name__=='__main__':main()
