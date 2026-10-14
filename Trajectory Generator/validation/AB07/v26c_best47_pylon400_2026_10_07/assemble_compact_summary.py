"""Project finalized source-bound statistics; no model/FK, IK or integration.

This post-result projection adds no new thresholds, windows or event detection.
Scientific metrics and the frozen consumer are preserved byte-for-byte.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
OUT = HERE/'analysis'


def read(p):
    return json.loads(Path(p).read_text(),parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))


def fp(p):
    p=Path(p);p=p if p.is_absolute() else ROOT/p
    return {'path':p.relative_to(ROOT).as_posix(),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'size_bytes':p.stat().st_size}


def reserve(w):
    return {k:{n:x[n] for n in ('unit','impulse_unit','force_or_torque','impulse','work_J','work_J_per_kg','power_W','muscle_vs_reserve_absolute_generalized_impulse') if n in x}
            for k,x in w['reserves']['per_coordinate'].items()}


def phase(p):
    result={}
    for label,channel in p['primary_channels'].items():
        result[label]={'status':channel['status'],'saved_support_s':channel['saved_support_s'],'events':{}}
        if channel['comparison'] is None:
            continue
        for kind,entry in channel['comparison'].items():
            result[label]['events'][kind]={}
            for clock,x in entry['channels'].items():
                m=x['matching'];support=channel['saved_support_s'];tol=x['tolerance_s']
                result[label]['events'][kind][clock]={
                    'tolerance_s':tol,'observed_times_s':x['observed_times_s'],
                    'matching':{k:m[k] for k in ('reference_count','predicted_count','matched_count','false_positives','false_negatives','timing_bias_s','timing_mae_s','timing_max_abs_s','pairs','unmatched_reference','unmatched_predicted')},
                    'unmatched_reference_fully_observed_guard_s':x['unmatched_reference_fully_observed_matching_guard_s'],
                    'unmatched_reference_censored_s':x['unmatched_reference_censored_or_unknown_s'],
                    'every_reference_guard':[{'reference_time_s':t,'complete':support[0]<=t-tol and support[1]>=t+tol} for t in entry['reference_times_s']],
                    'bias_definition':'predicted-minus-reference, statistics conditional on matched pairs; matching guard completeness is independent of match status'}
    return result


def sea(data):
    out={}
    for name,x in data.items():
        limit=x['command_limit_evidence']
        out[name]={k:x[k] for k in ('coordinate','model_properties','all_identity_checks_pass','control','control_bound_violations','normalized_control_saturation','motor_torque_clamp','motor_torque_bound_violations','tau_spring_Nm','tau_motor_Nm','tau_ref_minus_spring_Nm')}
        out[name]['normalized_command_limit']={k:limit[k] for k in ('exactly_at_plus_or_minus_one','u_unclipped','unclipped_command_above_unit_bound','unclipped_abs_excess_above_one','first_command_clipping_sample_s','last_command_clipping_sample_s','cascade_i_clamped','cascade_anti_windup_active')}
    return out


def case(m,p):
    w=m['windows']['primary_all']
    return {'runtime':m['runtime'],'mass_kg':m['mass_kg'],'BW_N':m['BW_N'],
            'primary':{'coverage':w['coverage'],'range_s':w['requested_range_s'],'duration_s':w['observed_duration_s'],'forces':w['forces'],'penetration':w['penetration'],'reserves_all_DOF':reserve(w),'tracking':w['tracking'],'vertical_balance':w['vertical_balance']},
            'all_recorded_SEA':sea(m['SEA_all_recorded']),
            'actor_final_counters':p['observed_full_episode_journal_secondary']['last_counters'],
            'swing_impulse_fraction':p['swing_impulse_fraction']['pooled'],
            'primary_event_timing':phase(p)}


def main():
    protocol=read(HERE/'analysis_protocol.json')
    paths={'metrics':OUT/'metrics.json','phase':OUT/'phase_metrics.json','baseline_metrics':ROOT/protocol['source_bindings']['baseline_metrics']['path'],'baseline_phase':ROOT/protocol['source_bindings']['baseline_phase']['path'],'script':Path(__file__),'protocol':HERE/'analysis_protocol.json','implementation_binding':HERE/'implementation_binding.json'}
    before={k:fp(p) for k,p in paths.items()}
    m,p,b,bp=(read(paths[k]) for k in ('metrics','phase','baseline_metrics','baseline_phase'))
    if not(m['run_artifacts_unchanged']['passed'] and p['run_artifacts_unchanged']['passed'] and m['windows']['primary_all']['coverage']==b['windows']['primary_all']['coverage']=='complete'):
        raise ValueError('Require source-bound complete identical primary comparisons')
    if m['windows']['primary_all']['requested_range_s']!=b['windows']['primary_all']['requested_range_s']:
        raise ValueError('Frozen primary windows differ')
    # Rehash all sources and both finalized native inventories; never modify them.
    guards=dict(protocol['protected_sources_before'])
    for data in (m,b):
        for label,item in data['run_artifacts_unchanged']['files'].items():
            if label in guards and guards[label]!=item['sha256']:
                raise ValueError('Conflicting artifact binding')
            guards[label]=item['sha256']
    for label,digest in guards.items():
        if fp(label)['sha256']!=digest:
            raise ValueError('Source/native artifact changed: '+label)
    learned,baseline=case(m,p),case(b,bp)
    def contrast(name,new,old,unit):
        return {'metric':name,'learned':new,'prescribed400':old,'learned_minus_prescribed400':new-old,'unit':unit}
    nw,bw=m['windows']['primary_all'],b['windows']['primary_all']
    comparisons=[contrast('left_online_mean',nw['forces']['left_online']['mean_N'],bw['forces']['left_online']['mean_N'],'N'),contrast('left_online_impulse',nw['forces']['left_online']['impulse_Ns'],bw['forces']['left_online']['impulse_Ns'],'Ns')]
    for key,unit,section in [('mean','N','force_or_torque'),('rms','N','force_or_torque'),('absolute','Ns','impulse'),('absolute','J','work_J')]:
        comparisons.append(contrast('pelvis_ty_'+section+'_'+key,nw['reserves']['per_coordinate']['pelvis_ty'][section][key],bw['reserves']['per_coordinate']['pelvis_ty'][section][key],unit))
    summary={'schema_version':1,'cases':{'V26_PYLON400_AB07':learned,'PYLON400_PRESCRIBED_REFERENCE':baseline},
             'same_window_numeric_contrasts':comparisons,
             'policy_action_clipped_steps':m['policy_trace']['clipped_policy_steps'],
             'actor35_full84_feature_contract_confirmed':m['feature_order']['matches_historical'],
             'actor_clock_identities_pass':m['actor_clock']['all_clock_identity_checks_pass'],
             'completed_binary_sensor_grids_pass':m['runtime_diagnostics']['completed_binary_sensor_grids_all_match'],
             'source_model_hash_equal':m['prescribed400_comparison']['source_model_hash_equal'],
             'source_IK_hash_equal':m['prescribed400_comparison']['source_IK_hash_equal'],
             'configuration_contract_differences':{'simulator':m['prescribed400_comparison']['simulator_config_differences'],'environment':m['prescribed400_comparison']['environment_config_differences']},
             'quadrature_convention':'Original window kernel: native1ms timestamps plus all7exact frozen HS/TO/end knots,4611primary points; trapezoidal means/RMS/impulses/work. No added zero crossings. Tiny RMS differences from another refinement grid are quadrature differences, not physical mismatches.',
             'units':'Translations: reserveN, impulseNs, actualspeedm/s. Rotations: reserveNm, impulseNm*s, actualspeedrad/s. WorkJ andJ/kg. Trackingdeg/deg/s exactly once; penetrationm.',
             'interpretation_limits':protocol['interpretation_limits']+['Normalized SEA command limits,500Nm rotor-input clamp, cascade integral clamp and neural action clipping are separate.',
                'Three actor-valid cycles do not establish event timing agreement. Confirmed/delivered pairing and raw20Nforce are reported separately. ClosingHS lacks a full late matching guard even when matched.',
                'Learned−ownIK deviation is not the actual−served tracking error.']}
    # Capsule contains scientific numbers and labels, with provenance kept separate.
    text=json.dumps(summary,indent=2,allow_nan=False)
    if '/Users/' in text or 'Trajectory Generator/' in text:
        raise ValueError('Unexpected local path in compact numerical capsule')
    target=OUT/'compact_numeric_summary.json'
    with target.open('x') as s:s.write(text+'\n')
    after={k:fp(p) for k,p in paths.items()}
    if before!=after:
        raise AssertionError('Final scientific sources changed during projection')
    for label,digest in guards.items():
        if fp(label)['sha256']!=digest:
            raise ValueError('Source/native artifact changed during projection')
    receipt={'created_utc':datetime.now(timezone.utc).isoformat(),'status':'PASS','sources_before':before,'sources_after':after,'protected_source_and_native_count':len(guards),'all_hashes_match_before_after':True,'numerical_capsule':fp(target),'projection_only':True,'new_FK_or_IK_or_integration':False,'frozen_artifact_edits':False}
    with (OUT/'compact_numeric_summary_receipt.json').open('x') as s:json.dump(receipt,s,indent=2,allow_nan=False);s.write('\n')
    print(json.dumps({'compact_summary':fp(target),'protected_count':len(guards)}))


if __name__=='__main__':
    main()
