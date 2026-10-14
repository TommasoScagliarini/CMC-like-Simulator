"""Summarize paired evaluation episodes, with explicit small-sample limits."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import math
import numpy as np


def wilson(successes,n):
    z=1.959963984540054
    p=successes/n; den=1+z*z/n
    center=(p+z*z/(2*n))/den
    half=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/den
    return [center-half,center+half]


def analyze(rollout_root, output):
    episodes=[]
    traces={}
    for diagnostic in sorted(rollout_root.glob("*/noise_diagnostic.json")):
        directory=diagnostic.parent
        meta=json.loads(diagnostic.read_text())
        summary=json.loads((directory/'rollout_summary.json').read_text())
        trace=json.loads((directory/'rollout_policy_trace.json').read_text())
        draws=json.loads((directory/'paired_noise_draws.json').read_text())
        key=(meta['noise_scale'],meta['noise_seed'])
        assert key not in traces
        traces[key]=(trace,draws)
        row={**meta,**{k:summary[k] for k in ('steps','end_reason','episode_return',
                'grf_penetration_max_m','action_clipped_fraction','phase_valid_cycle_count')},
             'completed':summary['end_reason']=='episode_time_limit' and summary['steps']==500,
             'output':str(directory)}
        episodes.append(row)
    if not episodes:
        raise RuntimeError('No completed diagnostics found')
    common_steps=min(r['steps'] for r in episodes)
    paired=[]
    for seed in sorted({r['noise_seed'] for r in episodes}):
        if (.5,seed) not in traces or (1.,seed) not in traces: continue
        half,half_draws=traces[(.5,seed)]; full,full_draws=traces[(1.,seed)]
        count=min(len(half_draws),len(full_draws))
        assert [d['standard_normal'] for d in half_draws[:count]] == [d['standard_normal'] for d in full_draws[:count]]
        assert half[0]['actor_observation_vector_before']==full[0]['actor_observation_vector_before']
        assert half[0]['policy_action_mean']==full[0]['policy_action_mean']
        paired.append({'seed':seed,'matched_draws':count,'initial_observation_and_mean_identical':True})
    for row in episodes:
        trace,_=traces[(row['noise_scale'],row['noise_seed'])]
        row['rmse_degrees']={}
        row['common_prefix_rmse_degrees']={}
        for joint in ('pros_knee_angle','pros_ankle_angle'):
            error=np.array([r['prosthetic_state'][joint]-r['imitation_target_q'][joint] for r in trace])
            row['rmse_degrees'][joint]=float(np.degrees(np.sqrt(np.mean(error**2))))
            row['common_prefix_rmse_degrees'][joint]=float(np.degrees(np.sqrt(np.mean(error[:common_steps]**2))))
        row['final_0_5s']=[{
            'step':r['step'],'time':r['time'],'raw_action':r['raw_policy_action'],
            'applied_action':r['applied_policy_action'],
            'penetration_m':r['reward_terms'].get('grf_penetration_m'),
            'normal_force_bw':r['reward_terms'].get('prosthetic_normal_force_bw'),
            'state':r['prosthetic_state'],'target_q':r['imitation_target_q'],
            'fsm_state':r['phase_fsm'].get('state_name'),
        } for r in trace[-50:]]
    groups=[]
    for scale in sorted({r['noise_scale'] for r in episodes}):
        rows=[r for r in episodes if r['noise_scale']==scale]
        n=len(rows); successes=sum(r['completed'] for r in rows)
        groups.append({'noise_scale':scale,'episodes':n,'complete':successes,
                       'completion_fraction':successes/n,
                       'wilson95':wilson(successes,n) if scale else None,
                       'mean_steps':float(np.mean([r['steps'] for r in rows])),
                       'mean_return':float(np.mean([r['episode_return'] for r in rows])),
                       'end_reasons':{reason:sum(r['end_reason']==reason for r in rows) for reason in sorted({r['end_reason'] for r in rows})}})
    result={'ok':True,'common_prefix_steps':common_steps,'paired_checks':paired,
            'groups':groups,'episodes':episodes,
            'limits':['Five independent noise sequences per stochastic condition; exploratory, not an equivalence test.',
                      'Checkpoint fixed; training completion mixes changing policies and episodes crossing updates.',
                      'Full-episode RMSE uses different horizons on early termination; common-prefix RMSE also supplied.']}
    output.mkdir(parents=True,exist_ok=True)
    (output/'noise_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    lines=['# Diagnosi del completamento con rumore controllato','',
           '| Scala rumore | Episodi completi | Passi medi | Return medio |',
           '|---|---:|---:|---:|']
    for g in groups:
        lines.append(f"| {g['noise_scale']:g} | {g['complete']}/{g['episodes']} | {g['mean_steps']:.1f} | {g['mean_return']:.2f} |")
    lines+=['','Cinque semi per condizione stocastica: risultato esplorativo, non prova di equivalenza statistica.',
            'Il checkpoint resta fisso; durante il training la policy cambia e gli episodi possono attraversare aggiornamenti.',
            f'Confronto RMSE aggiuntivo sul prefisso comune di {common_steps} passi.',
            'Sono verificati gli stessi campioni gaussiani e le stesse osservazioni/medie iniziali per ogni coppia.','']
    (output/'noise_analysis.md').write_text('\n'.join(lines))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
    colors={0.:'black',.5:'tab:blue',1.:'tab:red'}
    for (scale,seed),(trace,_) in sorted(traces.items()):
        time=np.array([r['time'] for r in trace])-trace[0]['time']+.01
        label=f'sigma×{scale:g}, seed {seed}'
        axes[0].plot(time,[1000*r['reward_terms'].get('grf_penetration_m',0) for r in trace],color=colors[scale],alpha=.7,label=label)
        axes[1].plot(time,[r['prosthetic_state']['pros_ankle_angle']*180/math.pi for r in trace],color=colors[scale],alpha=.7)
    axes[0].axhline(28,color='gray',linestyle='--',label='Soglia terminazione 28 mm')
    axes[0].set_ylabel('Penetrazione [mm]');axes[1].set_ylabel('Caviglia [gradi]')
    axes[1].set_xlabel('Tempo episodio [s]')
    axes[0].legend(ncol=3,fontsize=7)
    for ax in axes: ax.grid(alpha=.2)
    fig.suptitle('Checkpoint fisso: rumore esplorativo e penetrazione')
    fig.tight_layout();fig.savefig(output/'noise_penetration.png',dpi=160);plt.close(fig)
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--rollout-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    result=analyze(args.rollout_root,args.output)
    print(json.dumps({'groups':result['groups'],'paired_checks':result['paired_checks']},indent=2))


if __name__=='__main__': main()
