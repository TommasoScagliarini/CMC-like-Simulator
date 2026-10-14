"""Offline final audit: saved histories/traces only; no model or simulator runs."""
from pathlib import Path
import importlib.util
import json
import os

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
TG = PACKAGE.parent
REPO = TG.parent
os.environ.setdefault('MPLCONFIGDIR', str(HERE / 'mpl_cache'))
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'

import numpy as np

spec = importlib.util.spec_from_file_location('saved_rollout_helpers',
    PACKAGE / 'validation/ablation_report_2026-09-20/analyze_results.py')
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)

PLAN = PACKAGE / 'runs/plans/completion_recovery42_2026-09-22'
RUN = PACKAGE / 'runs/training/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21'
OLD = TG / 'runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter'
read = lambda p: json.loads(p.read_text())
history = lambda p: [json.loads(line) for line in (p / 'train_iterations.jsonl').read_text().splitlines()]


def window(rows, end, width=5):
    selected = rows[end-width:end]
    before = rows[end-width-1] if end > width else {}
    now = rows[end-1]
    counts = {}
    for key in set(now.get('termination_metrics', {})) | set(before.get('termination_metrics', {})):
        if '/episode_end/' in key:
            value = now.get('termination_metrics', {}).get(key, 0) - before.get('termination_metrics', {}).get(key, 0)
            assert value >= 0, (end, key, value)
            counts[key.rsplit('/', 1)[-1]] = int(value)
    total = sum(counts.values())
    return {'iterations': [end-width+1, end], 'episodes': total, 'outcomes': counts,
            'completion': counts.get('episode_time_limit', 0)/total if total else None,
            'samples_end': now['num_env_steps_sampled_lifetime'],
            'samples_window': now['num_env_steps_sampled_lifetime']-before.get('num_env_steps_sampled_lifetime', 0),
            'mean_iteration_return': float(np.mean([r['episode_return_mean'] for r in selected]))}


def main():
    assert read(PLAN / 'status.json')['status'] == 'completed'
    summary = read(RUN / 'summary.json')
    assert summary['ok'] and summary['iterations_completed'] == 100
    rows, old = history(RUN), history(OLD)
    assert [r['iteration'] for r in rows] == list(range(1,101))
    for r in rows:
        mask = r['unified_phase_audit']['target_availability_batch']
        assert mask['visible_steps'] == mask['total_steps'] == 4096
    result = {'audit': {'completed_at': read(PLAN / 'status.json')['updated_at'],
                       'training_iterations': 100, 'visible_samples': 409600,
                       'best_training_iteration': read(RUN / 'checkpoint_best_meta.json')['logical_iteration'],
                       'best_training_return': summary['best_episode_return_mean'],
                       'new_simulations': 0},
              'training_windows': {}, 'matched_experience': {}, 'rollouts': {}}
    for stage in ('42_to_60','60_to_80','80_to_100'):
        assert read(PLAN / f'optimizer_progress_{stage}.json')['passed']
    for end in (40,60,80,100):
        current = window(rows,end)
        match = min(old, key=lambda r:abs(r['num_env_steps_sampled_lifetime']-current['samples_end']))
        result['training_windows'][str(end)] = current
        result['matched_experience'][str(end)] = window(old,match['iteration'])
    result['v26_last5'] = window(old,100)
    folders = {'40': PACKAGE / 'runs/rollout/completion_diagnostics_2026-09-22_iter40/noise_0_seed_123',
               **{str(i): PACKAGE / f'runs/rollout/MLP39_completion_2026-09-22_iter{i:03d}/deterministic' for i in (60,80,100)},
               'V26_best87': TG / 'runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout'}
    traces = {}
    for label,folder in folders.items():
        metric, trace, s = helpers.rollout_metrics(folder, historical=label.startswith('V26'),
                                                  scale=None if label.startswith('V26') else 1)
        metric['fsm_final'] = {k:trace[-1]['phase_fsm'].get(k) for k in
            ('valid_hs_count','valid_to_count','valid_cycle_count','invalid_event_count','resync_count','hs_cancelled_count')}
        metric['fsm_events'] = [{'step':r['step'],'time':r['time'],
            'invalid_event_type':r['phase_fsm'].get('invalid_event_type'),
            'resync':r['phase_fsm'].get('resync_event_this_step'),
            'rejection':r['phase_fsm'].get('cycle_reject_reason')}
            for r in trace if any(r['phase_fsm'].get(k) for k in
                ('invalid_event_this_step','resync_event_this_step','cycle_rejected_this_step'))]
        metric['governor_mean'] = {k:float(np.mean([r['reference_governor_diagnostics'].get(k,0) for r in trace]))
            for k in ('velocity_limit_fraction','acceleration_limit_fraction','jerk_limit_fraction')}
        metric['ankle_rmse_by_elapsed_second'] = [helpers.rmse(
            np.array([r['prosthetic_state']['pros_ankle_angle'] for r in trace[i:i+100]]),
            np.array([r['imitation_target_q']['pros_ankle_angle'] for r in trace[i:i+100]])) for i in range(0,500,100)]
        metric['ankle_contact_moment_flip_penalized_steps'] = sum(
            r['reward_terms'].get('grf_ankle_moment_flip_loss',0)>0 for r in trace)
        result['rollouts'][label] = metric
        traces[label] = trace
    for label in ('60','80','100'):
        for joint in helpers.JOINTS:
            np.testing.assert_allclose([r['imitation_target_q'][joint] for r in traces[label]],
                                       [r['imitation_target_q'][joint] for r in traces['40']],atol=1e-10,rtol=0)
    result['audit']['identical_left_targets_40_60_80_100'] = True
    result['limitations'] = ['One training seed; five paired action-noise seeds at one fixed start.',
        'V26 uses a different target and historical worker failures can censor episode counts.',
        'Adam was reset inadvertently at iteration26; corrected continuation starts after40.',
        'Best72 is selected by training return and has no separate physical evaluation in this protocol.']
    failure_traces = {}
    result['paired_failure_windows_100'] = {}
    for seed in (126,127):
        pair = {scale:read(PACKAGE / f'runs/rollout/MLP39_completion_2026-09-22_iter100/noise_{scale}_seed{seed}/rollout_policy_trace.json')
                for scale in ('1','0.5')}
        stop = len(pair['1'])
        records = []
        for before in (20,10,5,0):
            index = stop-1-before
            np.testing.assert_allclose(list(pair['1'][index]['imitation_target_q'].values()),
                                       list(pair['0.5'][index]['imitation_target_q'].values()),atol=1e-10,rtol=0)
            for scale,t in pair.items():
                r=t[index]; state=r['prosthetic_state']; target=r['imitation_target_q'];terms=r['reward_terms']
                records.append({'scale':scale,'steps_before_failure':before,'step':index+1,
                    'target_phase':terms['pros_knee_angle_imitation_target_phase'],
                    'penetration_mm':1000*terms['grf_penetration_m'],
                    'force_bw':terms['prosthetic_normal_force_bw'],'fsm':r['phase_fsm']['state_name'],
                    'resync_count':r['phase_fsm']['resync_count'],
                    'error_deg':{j:float(np.degrees(state[j]-target[j])) for j in helpers.JOINTS},
                    'tracking_error_deg':{j:float(np.degrees(state[j]-state[j+'_served_ref'])) for j in helpers.JOINTS}})
        result['paired_failure_windows_100'][str(seed)]={'failure_step':stop,'matched_target':True,'records':records,
            'limitation':'Same time and prescribed target; states/history differ due to preceding actions. Not an intervention on one isolated joint.'}
        failure_traces[seed]=pair
    (HERE/'final_metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (HERE/'input_hashes.json').write_text(json.dumps(helpers.INPUT_HASHES,indent=2)+'\n')
    import matplotlib.pyplot as plt
    plotdir = REPO / 'plot/09_22_2026_unified39_final_comparison'
    plotdir.mkdir(parents=True,exist_ok=True)
    fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained')
    for ax,joint,title,sign in zip(axes[0],helpers.JOINTS,['Ginocchio','Caviglia'],[-1,1]):
        t=np.arange(1,501)*.01
        ax.plot(t,sign*np.degrees([r['imitation_target_q'][joint] for r in traces['40']]),'k--',label='Target sinistro')
        for label in ('40','60','80','100'):
            ax.plot(t,sign*np.degrees([r['prosthetic_state'][joint] for r in traces[label]]),label=f'Iter {label}',alpha=.85)
        ax.set(title=title,xlabel='Tempo episodio [s]',ylabel='Angolo [°]');ax.legend(fontsize=8);ax.grid(alpha=.25)
    for data,label in [(rows,'39D'),(old,'V26 storico')]:
        windows=[window(data,end) for end in range(5,len(data)+1)]
        axes[1,0].plot([w['samples_end']/1000 for w in windows],[100*w['completion'] for w in windows],label=label)
    axes[1,0].set(title='Episodi registrati completi: finestre di 5 iterazioni',xlabel='Transizioni raccolte [migliaia]',ylabel='Completamento [%]')
    axes[1,0].legend();axes[1,0].grid(alpha=.25)
    for scale,label in [(1.,'Rumore originale'),(.5,'Rumore dimezzato')]:
        vals=[]
        for it in (60,80,100):
            groups=read(PLAN/f'analysis_iter{it:03d}/noise_analysis.json')['groups']
            vals.append(next(g['complete'] for g in groups if g['noise_scale']==scale))
        axes[1,1].plot([60,80,100],vals,'o-',label=label)
    axes[1,1].set(title='Valutazione: 5 seed per condizione, uno start',xlabel='Checkpoint',ylabel='Episodi completi / 5',ylim=(-.2,5.3),xticks=[60,80,100])
    axes[1,1].legend();axes[1,1].grid(alpha=.25)
    fig.suptitle('Unified 39D: qualità della traiettoria e robustezza\nV26: target diverso e possibili fallimenti non registrati; confronto descrittivo',fontsize=13)
    fig.savefig(plotdir/'comparison.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(14,7),layout='constrained')
    for row,(seed,pair) in enumerate(failure_traces.items()):
        stop=len(pair['1']);start=max(0,stop-60)
        for scale,t in pair.items():
            tail=t[start:stop];time=np.array([r['step']*.01 for r in tail])
            label=f'Rumore ×{scale}'
            axes[row,0].plot(time,[1000*r['reward_terms']['grf_penetration_m'] for r in tail],label=label)
            axes[row,1].plot(time,[r['reward_terms']['prosthetic_normal_force_bw'] for r in tail],label=label)
            axes[row,2].plot(time,np.degrees([r['prosthetic_state']['pros_ankle_angle']-r['imitation_target_q']['pros_ankle_angle'] for r in tail]),label=label)
        axes[row,0].axhline(28,color='r',ls='--',label='Arresto 28 mm')
        for ax,title,ylabel in zip(axes[row],['Penetrazione','Carico piede sinistro','Errore caviglia'],['mm','Pesi corporei','Giunto − target [°]']):
            ax.set(title=f'Seed {seed}: {title}',xlabel='Tempo episodio [s]',ylabel=ylabel);ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.suptitle('Checkpoint 100: ultimi 600 ms dei due fallimenti e confronto appaiato\nStesso tempo e target prescritto; traiettorie e stati possono divergere')
    fig.savefig(plotdir/'paired_failures.png',dpi=170);plt.close(fig)
    print(json.dumps({'audit':result['audit'],'windows':result['training_windows'],
                      'matched100':result['matched_experience']['100'],'v26_last5':result['v26_last5']},indent=2))


if __name__ == '__main__':
    main()
