"""Isolated saved-data consumer for the frozen learned AB07 pylon400 rollout.

The scientific contract was frozen before outcomes; implementation hashes are
bound additively before first consumption. No policy, environment, IK or
integration is constructed. Sparse saved-state Velocity FK supplies COM only.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import fields, replace
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PROTOCOL = HERE / 'analysis_protocol.json'
BINDING = HERE / 'implementation_binding.json'
OUT = HERE / 'analysis'
CASE_ID = 'V26_PYLON400_AB07'


def read(p):
    return json.loads(Path(p).read_text(encoding='utf-8'),
                      parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))


def resolve(raw):
    p = Path(str(raw).replace('\\', '/'))
    if p.is_file() or p.is_dir():
        return p.resolve()
    if not p.is_absolute() and (ROOT / p).exists():
        return (ROOT / p).resolve()
    for marker in ('/Trajectory Generator/', '/validation/', '/models/', '/online_grf_profiles/', '/plugins/'):
        if marker in str(p):
            candidate = ROOT / marker.strip('/') / str(p).split(marker, 1)[1]
            if candidate.exists():
                return candidate.resolve()
    raise FileNotFoundError(raw)


def fp(p):
    p = resolve(p)
    label = p.relative_to(ROOT).as_posix() if p.is_relative_to(ROOT) else p.as_posix()
    return {'path': label, 'sha256': hashlib.sha256(p.read_bytes()).hexdigest(),
            'size_bytes': p.stat().st_size}


def write_new(p, value):
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('x', encoding='utf-8') as s:
        json.dump(value, s, indent=2, allow_nan=False)
        s.write('\n')


def check(manifest):
    changed = [p for p, h in manifest.items() if fp(p)['sha256'] != h]
    if changed:
        raise ValueError('Protected source changed: ' + repr(changed))
    return {'count': len(manifest), 'all_match': True}


def inventory(run):
    return {fp(p)['path']: fp(p) for p in sorted(run.rglob('*')) if p.is_file()}


def load(name, p):
    spec = importlib.util.spec_from_file_location(name, resolve(p))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def freeze_implementation():
    protocol = read(PROTOCOL)
    check(protocol['protected_sources_before'])
    sources = {'postprocessor': fp(__file__), 'analysis_protocol': fp(PROTOCOL),
               'timing_qualification': fp(HERE/'analysis_timing_qualification.json')}
    write_new(BINDING, {'created_utc': datetime.now(timezone.utc).isoformat(),
                       'status': 'implementation_bound_before_first_new_output_consumption',
                       'contract_protocol': fp(PROTOCOL), 'source_bindings': sources,
                       'native_or_event_data_consumed_by_this_implementation': False,
                       'implementation_completed_after_contract_freeze': True,
                       'contract_freeze_claim': 'Preoutcome scientific contract. Exact order relative to process launch is reported by the launcher, not inferred from this receipt.',
                       'scientific_kernels': 'Imported unchanged from source-bound standing analyzer and learned numerical verifier; new learned receipt/trace/path adapters only.'})
    print(json.dumps({'implementation_binding': fp(BINDING)}), flush=True)


def context():
    protocol, binding = read(PROTOCOL), read(BINDING)
    if binding['contract_protocol'] != fp(PROTOCOL) or binding['source_bindings']['postprocessor'] != fp(__file__):
        raise ValueError('Consumer differs from the additive implementation binding')
    check(protocol['protected_sources_before'])
    run = resolve(protocol['run_directory'])
    attestation = HERE / 'execution_result.json'
    if not attestation.is_file():
        raise ValueError('Wait for explicit finalized launcher execution_result; no runtime output read')
    close = read(attestation)
    if not any(k in close for k in ('returncode', 'return_code', 'exit_code')):
        raise ValueError('Launcher result does not attest terminal process return')
    receipt_path = run / 'receipt.json'
    receipt = read(receipt_path)
    if receipt.get('status') not in {'completed', 'interrupted', 'failed'}:
        raise ValueError('Runtime receipt is still active')
    if receipt.get('case') != CASE_ID:
        raise ValueError('Wrong learned case')
    if receipt.get('checkpoint_logical_iteration') != 47:
        raise ValueError('Wrong frozen checkpoint iteration')
    sources = protocol['source_bindings']
    standing = load('pylon400_standing_kernels', sources['standing_numeric']['path'])
    phase = load('pylon400_standing_phase_kernels', sources['standing_phase']['path'])
    inherited = read(resolve(sources['standing_numeric_protocol']['path']))
    return SimpleNamespace(protocol=protocol, binding=binding, run=run, receipt_path=receipt_path,
                           receipt=receipt, close=close, attestation=attestation,
                           standing=standing, phase=phase, inherited=inherited)


def native_helpers(c):
    # Called only after the terminal receipt/launcher gate. No environment/policy.
    return c.standing.helpers(c.inherited)


def load_native(c, h, cfg):
    tables = {p.stem.removeprefix(cfg.output_prefix + '_'): h.transfer.load_table_allow_empty(p, h.validator)
              for p in sorted((c.run/'sim_outputs').glob(cfg.output_prefix + '_*.sto'))}
    for name, table in tables.items():
        if table is not None and (len(set(table.columns)) != len(table.columns) or
                not np.isfinite(table.time).all() or not np.isfinite(table.data).all() or
                np.any(np.diff(table.time) <= 0)):
            raise ValueError('Invalid saved native table: ' + name)
    for name in ('kinematics', 'states', 'kinematics_reference'):
        if name in tables and tables[name] is not None and tables[name].in_degrees:
            raise ValueError('Native angles must use radians: ' + name)
    return tables


def numerical():
    c = context()
    target = OUT/'metrics.json'
    if target.exists():
        raise FileExistsError(target)
    before = inventory(c.run)
    r = c.receipt
    result = {'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
              'case_id': CASE_ID, 'subject': 'AB07', 'receipt': fp(c.receipt_path),
              'analysis_protocol': fp(PROTOCOL), 'implementation_binding': fp(BINDING),
              'execution_result': fp(c.attestation), 'windows': {}, 'new_integrations': 0,
              'new_IK_solves': 0, 'policy_loaded_by_postprocessor': False,
              'runtime': {k: r.get(k) for k in ('status','steps','diagnostic_steps','end_reason','terminated','truncated','error','final_failure','terminal_contact','completed_requested_duration','final_time_s','final_runner_time_s','final_state_time_s')}}
    if not all(k in r for k in ('effective_simulator_config','effective_env_config','actual_episode_start_s','actual_episode_end_s')):
        result['analysis_status'] = 'unavailable_finalized_startup_failure'
    else:
        h = native_helpers(c)
        if set(r['effective_simulator_config']) != {f.name for f in fields(h.cfg) if f.init}:
            raise ValueError('Incomplete captured simulator configuration')
        cfg = h.cfg(**r['effective_simulator_config'])
        inputs = {}
        for key, item in r['source_fingerprints'].items():
            inputs[key] = fp(item['path'])
            if inputs[key]['sha256'] != item['sha256']:
                raise ValueError('Runtime source changed: ' + key)
        model, ik = resolve(cfg.model_file), resolve(cfg.kinematics_file)
        if fp(model)['sha256'] != inputs['model']['sha256'] or fp(ik)['sha256'] != inputs['ik']['sha256']:
            raise ValueError('Captured model/IK differs from frozen run input')
        cfg = replace(cfg, model_file=str(model), kinematics_file=str(ik), model_bundle_dir=str(model.parent), output_dir=str(c.run/'sim_outputs'))
        if list(cfg.online_grf_applied_sides) != ['left']:
            raise ValueError('Inherited vertical balance requires left online + right measured GRF')
        start, end = float(r['actual_episode_start_s']), float(r['actual_episode_end_s'])
        if [start, end] != c.protocol['requested_range_s']:
            raise ValueError('Captured episode differs from frozen requested range')
        result['runtime']['actual_episode_range_s'] = [start, end]
        result['effective_config_source'] = 'complete finalized receipt; local path rebasing only'
        result['source_files'] = inputs
        tables = load_native(c, h, cfg)
        anchor = tables.get('kinematics')
        required = ('kinematics','states','kinematics_reference','reserve_torques','reserve_controls','recruitment','so_torque_diagnostics','online_grf','sea_states','sea_controls','sea_torques','sea_derivatives','sea_diagnostics','power')
        missing = [n for n in required if n not in tables or tables[n] is None or not len(tables[n].time)]
        result['missing_required_tables'] = missing
        result['tables'] = h.transfer.partial_table_audit(tables, replace(cfg,t_start=start,t_end=end), anchor.time if anchor is not None else np.array([]), h.numerical)
        trace_path = c.run/'rollout_policy_trace.json'
        trace = read(trace_path) if trace_path.is_file() else []
        summary = read(c.run/'rollout_summary.json') if (c.run/'rollout_summary.json').is_file() else {}
        env = r['effective_env_config']
        result['policy_trace'] = h.transfer.policy_trace_audit(trace, summary, h.numerical, r, {'start_s':start,'end_s':end}, env)
        result['actor_clock'] = h.transfer.actor_clock_audit(trace, env, h.numerical)
        result['runtime_diagnostics'] = h.transfer.runtime_diagnostics_audit(c.run/'runtime_diagnostics.jsonl', env, summary, h.numerical)
        result['feature_order'] = {'actor':r.get('actor_feature_names'),'full':r.get('observation_feature_names'),'matches_historical':r.get('feature_order_matches_historical')}
        if len(r.get('actor_feature_names',[])) != 35 or len(r.get('observation_feature_names',[])) != 84 or not r.get('feature_order_matches_historical'):
            raise ValueError('Frozen actor35/full84 feature contract not confirmed')
        if missing or anchor is None or len(anchor.time) < 2:
            result['analysis_status'] = 'unavailable_or_invalid_recorded_numerics'
        else:
            if any(t is None or not np.array_equal(t.time, anchor.time) for t in tables.values()):
                raise ValueError('Native output grids differ')
            result['runtime']['recorded_native_range_s'] = [float(anchor.time[0]),float(anchor.time[-1])]
            selection = read(resolve(c.protocol['source_bindings']['selection']['path']))['subjects']['AB07']
            cycles = selection['primary_cycles']
            knots = sorted({float(x[k]) for x in cycles for k in ('hs_s','to_s','next_hs_s')})
            covered = [t for t in knots if anchor.time[0] <= t <= anchor.time[-1]]
            com = c.standing.sparse_com(model, covered, tables['states'], cfg, h)
            com.update(requested_exact_knots_s=knots,missing_exact_knots_s=[t for t in knots if t not in covered])
            result['actual_COM'] = com
            result['new_FK_poses'] = len(covered) + 1  # Includes inherited unit-pelvis velocity validation.
            result['mass_kg'], result['BW_N'] = com['mass_kg'], com['BW_N']
            measured = c.standing.source_table(resolve(selection['source_fingerprints']['measured_GRF']['path']), h)
            raw = c.standing.source_table(ik, h)
            production = h.interpolator(cfg)
            model_coords = {e.get('name') for e in ET.parse(model).getroot().iter('Coordinate')}
            if len(raw.columns) != 21 or len(set(raw.columns)) != 21 or set(raw.columns) != model_coords or not raw.in_degrees:
                raise ValueError('Source IK must contain the21unique model coordinates in degrees/m')
            props = h.numerical.xml_sea_properties(model,[cfg.sea_knee_name,cfg.sea_ankle_name],h.validator)
            result['SEA_all_recorded'] = h.numerical.sea_audit(cfg,tables,props,h.precision)
            h.transfer.command_limit_audit(cfg,tables,result['SEA_all_recorded'],h.numerical)
            result['recruitment_all_recorded'] = h.numerical.reserves_and_recruitment(cfg,tables)
            result['contact_all_recorded'] = h.transfer.high_rate_contact_audit(tables,cfg,env,summary,h.numerical)
            queries = {'primary_all':(cycles[0]['hs_s'],cycles[-1]['next_hs_s']), 'all_recorded':(float(anchor.time[0]),float(anchor.time[-1]))}
            for i,x in enumerate(cycles,1):
                queries.update({f'cycle_{i}':(x['hs_s'],x['next_hs_s']),f'cycle_{i}_stance':(x['hs_s'],x['to_s']),f'cycle_{i}_swing':(x['to_s'],x['next_hs_s'])})
            for key,(a,b) in queries.items():
                result['windows'][key] = c.standing.window_metrics(float(a),float(b),tables,measured,raw,production,cfg,com,h)
            result['analysis_status'] = result['windows']['primary_all']['coverage']+'_primary_metrics'
            baseline = read(resolve(c.protocol['source_bindings']['baseline_metrics']['path']))
            if fp(baseline['receipt']['path']) != baseline['receipt']:
                raise ValueError('Prescribed400 finalized receipt changed after its bound metrics')
            old = read(resolve(baseline['receipt']['path']))
            config_exclusions = {'model_file','kinematics_file','model_bundle_dir','output_dir'}
            result['prescribed400_comparison'] = {'baseline_metrics':c.protocol['source_bindings']['baseline_metrics'],
                'baseline_receipt':fp(baseline['receipt']['path']),
                'source_model_hash_equal':fp(model)['sha256']==baseline['source_files']['model']['sha256'],
                'source_IK_hash_equal':fp(ik)['sha256']==baseline['source_IK']['sha256'],
                'simulator_config_differences':{k:{'prescribed':old['effective_simulator_config'].get(k),'learned':v} for k,v in r['effective_simulator_config'].items() if k not in config_exclusions and v!=old['effective_simulator_config'].get(k)},
                'environment_config_differences':{k:{'prescribed':old['effective_env_config'].get(k),'learned':v} for k,v in env.items() if k not in {'setup_xml_path','output_dir'} and v!=old['effective_env_config'].get(k)},
                'baseline_primary':baseline['windows']['primary_all'],
                'numerical_identity_required':False,
                'limit':'Learned served action/reference and gait-clock contracts differ; policy weights alone are not isolated. Different coverage is never compared as an identical primary.'}
    if inventory(c.run) != before:
        raise AssertionError('Finalized learned native artifacts changed')
    check(c.protocol['protected_sources_before'])
    result['run_artifacts_unchanged'] = {'count':len(before),'passed':True,'files':before}
    write_new(target,result)
    print(json.dumps({'metrics':fp(target),'status':result['analysis_status']}),flush=True)


def phase_analysis():
    c = context()
    target = OUT/'phase_metrics.json'
    if target.exists():
        raise FileExistsError(target)
    metric_path = OUT/'metrics.json'
    metric_fp = fp(metric_path)
    metric = read(metric_path)
    if metric['receipt'] != fp(c.receipt_path) or metric['analysis_protocol'] != fp(PROTOCOL) or not metric['run_artifacts_unchanged']['passed']:
        raise ValueError('Numerical metrics source is not bound to the same final run')
    before = inventory(c.run)
    p = read(resolve(c.protocol['source_bindings']['standing_phase_protocol']['path']))
    h = c.phase.helpers(p)
    s = read(resolve(c.protocol['source_bindings']['selection']['path']))['subjects']['AB07']
    cycles=s['primary_cycles']; nominal=h['nominal'](cycles); window=[cycles[0]['hs_s'],cycles[-1]['next_hs_s']]
    raw_rows=c.phase.trace(c.run/'runtime_diagnostics.jsonl') if (c.run/'runtime_diagnostics.jsonl').is_file() else []
    rows=[dict(x,step=int(x['diagnostic_step'])) for x in raw_rows]
    h['trace_validation'](rows,CASE_ID)
    actor=[e for row in rows for e in row.get('phase_fsm',{}).get('accepted_transitions_this_step',[])]
    binary=[e for row in rows for e in row.get('binary_phase_fsm',{}).get('events_this_step',[])]
    c.phase.assert_events(actor+binary)
    canceled=[e for e in actor if e.get('event')=='heel_strike_cancelled']
    retained=[e for e in actor if e.get('event') in {'heel_strike','toe_off'} and not(e['event']=='heel_strike' and any(abs(float(e['event_time_s'])-float(x['event_time_s']))<=1e-8 for x in canceled))]
    samples=[x for row in rows for x in row.get('binary_phase_sensor_samples',[])]
    ts=np.asarray([x['time_s'] for x in samples],dtype=float)
    if not np.isfinite(ts).all() or np.any(np.diff(ts)<0):
        raise ValueError('Invalid recorded binary acquisition clocks')
    trace_support=[float(rows[0]['time']),float(rows[-1]['time'])] if rows else None
    sensor_support=[float(ts[0]),float(ts[-1])] if len(ts) else trace_support
    support_actor=sensor_support if samples else trace_support
    channels={'binary_geometry_FSM':c.phase.compare_channel(binary,nominal,window,sensor_support if rows and all('binary_phase_fsm' in x for x in rows) else None,p,h,'binary_geometry_FSM'),
              'actor_retained_after_HS_repairs':c.phase.compare_channel(retained,nominal,window,support_actor if rows and all('phase_fsm' in x for x in rows) else None,p,h,'actor_retained_after_HS_repairs')}
    force_events=[]; force_support=None
    paths=list((c.run/'sim_outputs').glob('*_online_grf.sto'))
    if len(paths)>1:
        raise ValueError('Ambiguous saved force table')
    if paths:
        # Original reader and crossing are unchanged; genuinely empty tables stay unavailable.
        try:
            t,names,data,_=h['reader'](paths[0])
        except ValueError:
            lines=paths[0].read_text().splitlines(); end=next(i for i,x in enumerate(lines) if x.strip().lower()=='endheader')
            body=[x for x in lines[end+1:] if x.strip()]
            if len(body)>1:
                raise
        else:
            if len(t)>1:
                h['require_grid'](t,str(paths[0]))
            dt=float(c.receipt['effective_simulator_config']['dt'])
            if len(set(names))!=len(names) or 'left_force_y' not in names or not np.allclose(np.diff(t),dt,atol=1e-8,rtol=0):
                raise ValueError('Saved force schema/grid differs')
            force_support=[float(t[0]),float(t[-1])]
            force_events=[{'event':kind,'event_time_s':float(x),'confirmed_time_s':float(x),'delivered_time_s':float(x)} for kind in ('heel_strike','toe_off') for x in h['crossing'](t,data[:,names.index('left_force_y')],p['threshold_N'],rising=kind=='heel_strike')]
    channels['primary_force_20N_crossings']=c.phase.compare_channel(force_events,nominal,window,force_support,p,h,'primary_force_20N_crossings')
    for channel in channels.values():
        if channel['comparison'] is None:
            continue
        support=channel['saved_support_s']
        for kind,entry in channel['comparison'].items():
            for matching in entry['channels'].values():
                tol=matching['tolerance_s']
                matching['every_reference_matching_guard']=[{'reference_time_s':t,'complete':support[0]<=t-tol and support[1]>=t+tol} for t in nominal[kind]]
    reached=any(x['comparison'] is not None for x in channels.values())
    journal=h['journal'](rows,h['nominal']([s['startup_cycle']]+cycles if reached else [s['startup_cycle']]),p['confirmed_and_delivered_tolerances_s'],{'sensor_holdout_heel_strike_max_error_s':p['onset_holdout_tolerances_s']['heel_strike'],'sensor_holdout_toe_off_max_error_s':p['onset_holdout_tolerances_s']['toe_off']},h['matcher'],s['reset_hs_s'])
    journal['reference_scope']='startup_plus_fixed_primary' if reached else 'startup_only_primary_not_reached'
    phase_windows=[]
    for i,x in enumerate(cycles,1):
        row={'cycle':i}
        for label in ('stance','swing'):
            w=metric['windows'].get(f'cycle_{i}_{label}',{})
            row[label]={'coverage':w.get('coverage','unavailable'),'impulse_Ns':w.get('forces',{}).get('left_online',{}).get('impulse_Ns') if w.get('coverage')=='complete' else None}
        phase_windows.append(row)
    primary=metric['windows'].get('primary_all',{})
    pooled=None
    if primary.get('coverage')=='complete' and all(x[k]['coverage']=='complete' for x in phase_windows for k in ('stance','swing')):
        total=primary['forces']['left_online']['impulse_Ns']; stance=sum(x['stance']['impulse_Ns'] for x in phase_windows); swing=sum(x['swing']['impulse_Ns'] for x in phase_windows)
        pooled={'stance_impulse_Ns':stance,'swing_impulse_Ns':swing,'primary_impulse_Ns':total,'phase_sum_minus_primary_Ns':stance+swing-total,'swing_fraction_of_primary_impulse':swing/total if total else None}
    result={'created_utc':datetime.now(timezone.utc).isoformat(),'case_id':CASE_ID,'subject':'AB07','receipt':fp(c.receipt_path),'analysis_protocol':fp(PROTOCOL),'implementation_binding':fp(BINDING),'numerical_source':metric_fp,'runtime_status':c.receipt['status'],'primary_reference':{'cycles':cycles,'window_s':window,'events_s':nominal},'primary_channels':channels,'observed_full_episode_journal_secondary':journal,'all_saved_raw_force_crossings':force_events,'swing_impulse_fraction':{'cycles':phase_windows,'pooled':pooled,'status':'complete_three_cycle_fraction' if pooled else 'pooled_withheld_incomplete_primary'},'support_adapter':'Recorded 1ms sensor support used for binary and actor transports; logical trace timestamps retained separately. No detector execution.','logical_trace_support_s':trace_support,'physical_sensor_support_s':sensor_support,'limitations':p['semantic_limits'],'new_native_or_FK':False}
    if inventory(c.run)!=before or fp(metric_path)!=metric_fp:
        raise AssertionError('Bound finalized artifacts changed during phase analysis')
    check(c.protocol['protected_sources_before'])
    result['run_artifacts_unchanged']={'passed':True,'count':len(before),'files':before}
    write_new(target,h['strict'](result))
    print(json.dumps({'phase_metrics':fp(target)}),flush=True)


def render():
    c=context(); metric_path=OUT/'metrics.json'; metric=read(metric_path)
    if metric['receipt']!=fp(c.receipt_path) or metric['analysis_protocol']!=fp(PROTOCOL) or not metric['run_artifacts_unchanged']['passed']:
        raise ValueError('Plot metric/receipt source mismatch')
    before=inventory(c.run)
    out=ROOT/c.protocol['plots_contract']['root']
    if out.exists():
        raise FileExistsError(out)
    original=load('pylon400_explicit_render',c.protocol['source_bindings']['prescribed_render']['path'])
    plotter=original.load_plotter(); missing=plotter.MissingReport()
    cfg=plotter.SimulatorConfig(**c.receipt['effective_simulator_config'])
    cfg=replace(cfg,model_file=str(resolve(cfg.model_file)),kinematics_file=str(resolve(cfg.kinematics_file)),output_dir=str(c.run/'sim_outputs'))
    names=('sea_torques','sea_controls','states','kinematics_reference','reserve_torques','sea_states','sea_diagnostics','power')
    tables={name:original.read_storage(c.run/'sim_outputs'/f'{cfg.output_prefix}_{name}.sto',plotter) for name in names}
    healthy_root=ROOT/'models/AB07_SEASEA_Threadmill/data/healthy'
    healthy_q=original.read_storage(healthy_root/'AB07_treadmill_01_01_Kinematics_q.sto',plotter)
    healthy_force=original.read_storage(healthy_root/'AB07_treadmill_01_01_Actuation_force.sto',plotter)
    healthy=plotter.HealthyData(healthy_root,healthy_q,healthy_force,['Healthy velocity/power are dataset diagnostics.'])
    support=[healthy_q,healthy_force,*tables.values()]
    window=(max(x.time[0] for x in support),min(x.time[-1] for x in support))
    selection=read(resolve(c.protocol['source_bindings']['selection']['path']))['subjects']['AB07']
    fields_,cycles,excluded=original.selected_cycles(resolve(selection['primary_reference_csv']['path']),window)
    out.mkdir(parents=True)
    csv_path=out/'reference_cycles_in_recorded_window.csv'
    with csv_path.open('x',newline='',encoding='utf-8') as s:
        writer=csv.DictWriter(s,fieldnames=fields_);writer.writeheader();writer.writerows(cycles)
    events=plotter.load_events(csv_path,missing)
    outer=plotter.outer_loop_subtitle(cfg); inner_params=plotter.load_sea_params(cfg);inner=plotter.inner_loop_subtitle(inner_params,cfg)
    saved_reference=plotter.recorded_reference_angle_series
    def served(table,time,coord,side):
        if table is None or table.series([coord+'_q_ref']) is None or time[0]<table.time[0] or time[-1]>table.time[-1]:
            missing.add('figure6 '+side+': missing complete recorded served reference');return None
        return saved_reference(table,time,coord,side)
    plotter.recorded_reference_angle_series=served
    def shared_interpolation(series,time):
        if time[0]<series[0][0] or time[-1]>series[0][-1]:
            missing.add('Target/error use shared recorded target and served time support; outside target coverage is blank')
        return np.interp(time,series[0],series[1],left=np.nan,right=np.nan)
    plotter.interpolate_like=shared_interpolation
    save=plotter.save_figure
    def labelled(fig,destination,filename):
        title=fig._suptitle.get_text() if fig._suptitle else ''
        if filename.startswith('07_'):
            title='Served policy C2 vs recorded contralateral IK phase target (diagnostic ex_novo)'
            for ax in fig.axes:
                ax.set_ylabel(ax.get_ylabel().replace('shifted sound leg','IK phase target').replace('sound leg raw','actual sound leg'))
                for line in ax.lines:
                    line.set_label(line.get_label().replace('shifted sound leg target','contralateral IK phase target').replace('shifted sound leg - policy C2','IK phase target - served policy').replace('sound leg raw','actual sound leg'))
                if ax.get_legend() is not None:
                    ax.legend(loc='best',fontsize=8)
        fig.suptitle('AB07 pylon400 | V26C best47 actor35/full84 | receipt '+c.receipt['status']+'\n'+title,fontsize=12,y=.995)
        text=[x for x in fig.texts if x is not fig._suptitle and x.get_position()[1]>.9]
        for x,y in zip(text,(.943,.921)):
            x.set_position((x.get_position()[0],y))
        for ax in fig.axes:
            if filename.startswith(('03_','04_')) and ax.get_xlabel()=='gait cycle [%]':
                ax.set_xlabel('Measured-GRF nominal reference phase [%]')
        fig.tight_layout(rect=(0,0,1,.865));save(fig,destination,filename)
    plotter.save_figure=labelled
    plotter.plot_figure_1(tables,out,missing,outer,inner)
    plotter.plot_figure_2(tables,out,missing,outer,inner)
    plotter.plot_figure_3(tables,events,'left',healthy,out,missing,outer,inner)
    plotter.plot_figure_4(tables,events,'left',healthy,out,missing,outer,inner)
    plotter.plot_figure_5(tables,plotter.load_sea_f_opt(cfg,missing),out,missing,outer,inner)
    plotter.plot_figure_6(tables,None,out,missing,outer,inner)
    summary=read(c.run/'rollout_summary.json')
    context_=plotter.MlpRolloutContext(c.run,c.run/'sim_outputs',c.run/'rollout_summary.json',c.run/'rollout_policy_trace.json',summary,'ex_novo')
    plotter.plot_figure_7(tables,context_,out,missing,outer,inner)
    missing.write(out/'missing_channels.txt');pngs=sorted(out.glob('*.png'))
    if len(pngs)!=7 or inventory(c.run)!=before:
        raise AssertionError('Require exactly7canonical plots and unchanged finalized run')
    manifest={'status':'rendered_pending_visual_review','case_id':CASE_ID,'receipt':fp(c.receipt_path),'metrics':fp(metric_path),'analysis_protocol':fp(PROTOCOL),'implementation_binding':fp(BINDING),'root_plotter':fp(ROOT/'plot/plotter.py'),'files':{p.name:fp(p) for p in pngs},'cycles_used':cycles,'cycles_excluded_for_incomplete_support':excluded,'common_recorded_range_s':window,'timestamp_changes':False,'latest_context_discovery':False,'prescribed_fallback':False,'figure7_target':'Recorded contralateral IK phase template, ex_novo diagnostic; actual sound-leg separate; no target endpoint extrapolation','missing_channels':missing.items,'model_inner_loop_parameters':inner_params,'run_artifacts_unchanged':True,'new_native_or_FK':False,'visual_review':{p.name:'pending' for p in pngs}}
    write_new(out/'plot_rendering_manifest.json',manifest)
    print(json.dumps({'plot_manifest':fp(out/'plot_rendering_manifest.json')}),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--freeze-implementation',action='store_true')
    group.add_argument('--numeric',action='store_true')
    group.add_argument('--phase',action='store_true')
    group.add_argument('--plots',action='store_true')
    args=parser.parse_args()
    if args.freeze_implementation: freeze_implementation()
    elif args.numeric: numerical()
    elif args.phase: phase_analysis()
    else: render()


if __name__=='__main__':
    main()
