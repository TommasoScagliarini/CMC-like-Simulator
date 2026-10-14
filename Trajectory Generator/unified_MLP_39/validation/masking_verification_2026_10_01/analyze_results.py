"""Offline common-window metrics and actor-time phase audit; rerunnable as runs finish."""
import importlib.util
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[1]
ROOT=PACKAGE.parents[1]
METRICS=HERE/'metrics'
PLOTS=ROOT/'plot/10_01_2026_masking_verification_comparison'
spec=importlib.util.spec_from_file_location('prior_analysis',HERE.parent/'masking_diagnosis_2026-10-01/rollouts/analyze.py')
analysis=importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)
import matplotlib.pyplot as plt
analysis.HERE=METRICS

def sources():
    result=dict(analysis.SOURCES)
    for label in ('cp90_hidden','cp112_visible','cp184_visible'):
        result[label]=PACKAGE/'runs/rollout/masking_diagnosis_2026-10-01'/label
    for folder in sorted((PACKAGE/'runs/rollout/masking_verification_2026-10-01').iterdir()):
        if (folder/'rollout_summary.json').exists():
            summary=json.loads((folder/'rollout_summary.json').read_text())
            if summary.get('ok'):
                result[folder.name]=folder
    return result

def phase_audit(label,trace,baseline):
    n=len(trace)
    time=np.array([r['policy_segment_times'][0] for r in trace])
    np.testing.assert_allclose(time,[r['policy_segment_times'][0] for r in baseline[:n]],atol=1e-8,rtol=0)
    obs=[r['actor_observation_before'] for r in trace]
    baseobs=[r['actor_observation_before'] for r in baseline[:n]]
    prescribed=np.mod(np.arctan2([o['gait_phase_sin'] for o in baseobs],
                                [o['gait_phase_cos'] for o in baseobs])/(2*np.pi),1)
    online=np.mod(np.arctan2([o['online_left_gait_phase_sin'] for o in obs],
                            [o['online_left_gait_phase_cos'] for o in obs])/(2*np.pi),1)
    period=np.array([o['online_left_cycle_duration_s'] for o in obs])
    valid=period>0
    delta=(online-prescribed+.5)%1-.5
    error=np.array([[-np.degrees(r['prosthetic_state']['pros_knee_angle']-r['imitation_target_q']['pros_knee_angle']),
                    np.degrees(r['prosthetic_state']['pros_ankle_angle']-r['imitation_target_q']['pros_ankle_angle'])] for r in trace])
    record={'actor_time_definition':'policy_segment_times[0]; reference clock from same-time CP90 visible observation',
            'startup_steps_without_period':int((~valid).sum()),
            'first_period_available_actor_time_s':float(time[valid][0]) if valid.any() else None,
            'states_with_period':int(valid.sum()),'phase_error_units':'cycle, wrapped to [-0.5,0.5)',
            'online_phase_at_boundary_fraction_valid':float(np.mean(np.minimum(online[valid],1-online[valid])<1e-6)) if valid.any() else None,
            'windows':{},'events':[]}
    for name,keep in [('startup',~valid),('period_available',valid)]:
        d=delta[keep]
        if not len(d):
            record['windows'][name]={'steps':0};continue
        shift=float(np.angle(np.mean(np.exp(2j*np.pi*d)))/(2*np.pi))
        residual=(d-shift+.5)%1-.5
        record['windows'][name]={'steps':int(keep.sum()),'phase_rmse_cycles':float(np.sqrt(np.mean(d*d))),
            'phase_mae_cycles':float(np.mean(np.abs(d))),'fitted_constant_phase_offset_cycles':shift,
            'phase_rmse_after_constant_offset_cycles':float(np.sqrt(np.mean(residual*residual))),
            'kinematic_rmse_knee_ankle_deg':np.sqrt(np.mean(error[keep]**2,axis=0)).tolist()}
    for r in trace:
        f=r['phase_fsm']
        if f['accepted_transitions_this_step'] or f['resync_event_this_step'] or f['invalid_event_this_step']:
            record['events'].append({'time':r['time'],'transitions':f['accepted_transitions_this_step'],
                'resync':f['resync_event_this_step'],'invalid':f['invalid_event_this_step'],
                'period_s':f['last_period_s']})
    fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True,layout='constrained')
    axes[0].plot(time,prescribed,label='Prescribed phase');axes[0].plot(time,online,label='Online phase')
    axes[0].fill_between(time,0,1,where=~valid,color='gray',alpha=.2,label='No measured period')
    axes[0].set_ylabel('Phase [cycle]');axes[0].legend()
    axes[1].plot(time,delta,label='Online - prescribed (wrapped)');axes[1].set_ylabel('Difference [cycle]')
    axes[2].plot(time,period,label='Online period [s]')
    axes[2].plot(time,[o['phase_stance_elapsed_norm'] for o in obs],label='Stance timer (normalized)')
    axes[2].plot(time,[o['phase_swing_elapsed_norm'] for o in obs],label='Swing timer (normalized)')
    for e in record['events']:
        axes[2].axvline(e['time'],color='red' if e['resync'] else 'gray',alpha=.25)
    for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=8)
    axes[-1].set_xlabel('Actor observation time [s]');fig.suptitle(label)
    fig.savefig(PLOTS/f'{label}_phase.png',dpi=140);plt.close(fig)
    return record

def compare_group(label,names,metrics,signals):
    names=[n for n in names if n in signals]
    if len(names)<2:return None
    end=min(metrics[n]['window_s'][1] for n in names)
    window=(analysis.WINDOW[0],end)
    paired={'window_s':window,'rollouts':{}}
    fig,axes=plt.subplots(2,2,figsize=(14,8),sharex=True,layout='constrained')
    base=signals[names[0]]
    for j,joint in enumerate(('knee','ankle')):
        for ax in axes[:,j]:ax.plot(base['time'],base['joints'][joint]['target'],color='black',label='Prescribed target')
        axes[0,j].set_title(joint)
    for name in names:
        s=signals[name];keep=analysis.mask(s['time'],window)
        paired['rollouts'][name]={'steps':int(keep.sum()),'rmse_actual_target_deg':{},'rmse_served_target_deg':{}}
        for j,joint in enumerate(('knee','ankle')):
            d=s['joints'][joint]
            for row,kind in enumerate(('actual','served')):
                axes[row,j].plot(s['time'],d[kind],label=name)
                paired['rollouts'][name][f'rmse_{kind}_target_deg'][joint]=analysis.rms((d[kind]-d['target'])[keep])
        paired['rollouts'][name]['prefix_reward_mean']=float(np.mean([r['reward'] for r,k in zip(s['reward'],keep) if k]))
    for ax in axes.flat:ax.grid(alpha=.25);ax.legend(fontsize=8);ax.set_xlim(*window)
    axes[0,0].set_ylabel('Actual [deg]');axes[1,0].set_ylabel('Served [deg]')
    for ax in axes[-1]:ax.set_xlabel('Simulation time [s]')
    fig.suptitle(f'{label}: identical surviving time window')
    fig.savefig(PLOTS/f'{label}_kinematics.png',dpi=150);plt.close(fig)
    return paired

def main():
    METRICS.mkdir(exist_ok=True);PLOTS.mkdir(exist_ok=True)
    folders=sources();metrics={};signals={};phases={}
    baseline=json.loads((folders['cp90_visible']/'rollout_policy_trace.json').read_text())
    for name,folder in folders.items():
        metrics[name],signals[name]=analysis.analyze(name,folder)
        trace=json.loads((folder/'rollout_policy_trace.json').read_text())
        phases[name]=phase_audit(name,trace,baseline)
        for j in ('knee','ankle'):
            np.testing.assert_allclose(signals[name]['joints'][j]['target'],signals['cp90_visible']['joints'][j]['target'][:len(trace)],atol=1e-8,rtol=0)
    groups={f'cp{cp}_factorial':[f'cp{cp}_{m}' for m in ('visible','clock_hidden','target_hidden','hidden')] for cp in (90,112,184)}
    groups.update({f'cp{i}_{v}_sigma':['cp90_'+v]+[f'{t}_cp{i}_{v}' for t in ('learned','frozen_cp90')] for i in (95,100) for v in ('visible','hidden')})
    paired={name:compare_group(name,names,metrics,signals) for name,names in groups.items()}
    output={'rollouts':metrics,'paired_comparisons':paired,'phase':phases,
        'limitations':['Single deterministic initial state per checkpoint/mask.',
         'Startup and period-available windows contain different target portions; their RMSE is descriptive, not a causal comparison.',
         'Constant phase offset is fitted retrospectively; removing it is not an actor improvement.',
         'Circular encoding cannot distinguish online phase 0 from clipped phase 1.',
         'Partial masks retain source target_availability.visible=true; diagnostic_mask identifies actual actor input masking.']}
    (HERE/'analysis.json').write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    (HERE/'analysis_input_hashes.json').write_text(json.dumps(analysis.HASHES,indent=2)+'\n')
    lines=['| Rollout | Steps | RMSE knee/ankle [deg] | Window end [s] | Return |','|---|---:|---:|---:|---:|']
    for n,m in metrics.items():
        vals='/'.join(f"{m['joints'][j]['rmse_actual_target_deg']:.2f}" for j in ('knee','ankle'))
        lines.append(f"| {n} | {m['steps']} | {vals} | {m['window_s'][1]:.2f} | {m['episode_return']:.2f} |")
    (HERE/'metrics_table.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))

if __name__=='__main__':main()
