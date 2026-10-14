"""Read-only paired RIGHT58/35/40/V26 analysis using the existing audited helpers."""
from __future__ import annotations
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[2]
REPO=PACKAGE.parents[1]
for name in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','VECLIB_MAXIMUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[name]='1'
os.environ['MPLCONFIGDIR']=str(HERE/'mpl_cache')
os.environ['MPLBACKEND']='Agg'
script=PACKAGE/'validation/right_reference_screening_2026-09-23/analyze.py'
spec=importlib.util.spec_from_file_location('right60_previous_screening',script)
old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
prior,h,np,plt,physical=old.prior,old.h,old.np,old.plt,old.physical
JOINTS=old.JOINTS
ROOTS={'right35':old.ROOTS['best35'],'right40':old.ROOTS['last40'],
       'right58':PACKAGE/'runs/rollout/MLP39_right_screening60_2026-09-23/best58','v26':old.ROOTS['v26']}
LABELS={'right35':'RIGHT39 best35','right40':'RIGHT39 iter40','right58':'RIGHT39 best58','v26':'V26 best87 (replay)'}
FIGURES=REPO/'plot/09_23_2026_right60_comparison'


def hash_file(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def provenance(label,folder):
    summary=h.read(folder/'rollout_summary.json')
    checkpoint=Path(summary['checkpoint'])
    if label.startswith('right'):
        iteration=int(label[5:])
        assert checkpoint.parent.name==f'milestone_iteration_{iteration:06d}'
        assert checkpoint.name=='rl_module_last'
        manifest=HERE/'input_hashes.json' if iteration==58 else PACKAGE/'runs/plans/right_reference_screening_2026-09-23/input_hashes.json'
    else:
        manifest=PACKAGE/'validation/v26_robustness_2026-09-23/input_hashes.json'
    hashes=h.read(manifest)
    matched={name:value for name,value in hashes.items() if str(REPO/name).startswith(str(checkpoint)+os.sep)}
    assert matched,f'Missing original checkpoint hash provenance:{label}'
    for name,value in matched.items():
        assert hash_file(REPO/name)==value,name
    return {'checkpoint':str(checkpoint),'manifest':str(manifest),'verified_checkpoint_files':len(matched),
            'module_state_sha256':hash_file(checkpoint/'module_state.pkl'),
            'metric_source':'Saved per-step logits and actions; no mutable checkpoint_best is loaded.'}


def main():
    assert h.read(HERE/'status.json')['state']=='completed'
    assert h.read(HERE/'results.json')['technical_pass']
    frozen=h.read(HERE/'input_hashes.json')
    assert all(hash_file(REPO/name)==value for name,value in frozen.items())
    source_hashes={str(script.relative_to(REPO)):hash_file(script)}
    episodes,valid,traces,draws,provenances={},{},{},{},{}
    historical=h.read(physical.OLD/'rollout_policy_trace.json')
    for label,base in ROOTS.items():
        provenances[label]=provenance(label,base/'noise_1_seed123')
        for seed in range(123,128):
            folder=base/f'noise_1_seed{seed}'
            metric,trace,good,noise=prior.noise_episode(folder,label,1.,seed)
            metric['fsm_counters']=old.counters(trace)
            assert all(row['target_availability']['visible'] for row in trace)
            for row,reference in zip(good,historical):
                np.testing.assert_allclose(row['time'],reference['time'],atol=2e-8,rtol=0)
                for joint in JOINTS:
                    for field in ('imitation_target_q','imitation_target_qdot'):
                        np.testing.assert_allclose(row[field][joint],reference[field][joint],atol=1e-10,rtol=0)
                for key in ('gait_phase_sin','gait_phase_cos'):
                    np.testing.assert_allclose(row['actor_observation_before'][key],reference['actor_observation_before'][key],atol=1e-7,rtol=0)
            metric['right_target_clock_visible_audit']=True
            episodes[label,seed],valid[label,seed],traces[label,seed],draws[label,seed]=metric,good,trace,noise
    common=min(map(len,valid.values()));signed_series={}
    for key,metric in episodes.items():
        rows=valid[key][:common]
        metric['common_prefix_rmse_degrees']=prior.rmse_rows(rows)
        metric['common_prefix_sea_metrics']=prior.sea_metrics(rows)
        metric['common_prefix_action_metrics']=prior.prefix_action_diagnostics(rows,draws[key][:common],1.)
        if metric['signed_torque_sto_available']:
            table=physical.read_table(Path(metric['rollout']),'sea_torques')
            elapsed=table.time-table.time[0]
            mask=(elapsed>=.2-1e-8)&(elapsed<common*.01-1e-8)
            if mask.sum()>=20:
                metric['common_prefix_signed_torque_after200ms']={}
                for short in ('knee','ankle'):
                    values=table.data[:,table.index[f'SEA_{short.title()}_tau_spring']]*physical.plotter.joint_sign(short)
                    metric['common_prefix_signed_torque_after200ms'][short]=physical.torque_window(table.time,values,mask)
                    if short=='ankle':
                        signed_series[key]=(elapsed[mask],values[mask])
    paired=[]
    for seed in range(123,128):
        n=min(len(draws[label,seed]) for label in ROOTS)
        baseline=[row['standard_normal'] for row in draws['v26',seed][:n]]
        assert all([row['standard_normal'] for row in draws[label,seed][:n]]==baseline for label in ROOTS)
        paired.append({'seed':seed,'matched_draws':n,'all_four_policies_identical_standard_normals':True})
    groups=[]
    for label in ROOTS:
        group=[episodes[label,seed] for seed in range(123,128)]
        count=sum(row['completed'] for row in group)
        counters=lambda name:[row['fsm_counters'][name] for row in group if row['fsm_counters'][name] is not None]
        item={'policy':label,'episodes':5,'completed':count,'completion_fraction':count/5,
            'wilson95':prior.wilson(count,5),'mean_steps':float(np.mean([row['steps'] for row in group])),
            'mean_return':float(np.mean([row['episode_return'] for row in group])),
            'end_reasons':{reason:sum(row['end_reason']==reason for row in group) for reason in sorted({row['end_reason'] for row in group})},
            'two_valid_cycles':sum(row['fsm_counters']['valid_cycle_count']>=2 for row in group),
            'completed_with_two_cycles_and_zero_invalid':sum(row['completed'] and row['fsm_counters']['valid_cycle_count']>=2 and row['fsm_counters']['invalid_event_count']==0 for row in group),
            'total_invalid_events':sum(counters('invalid_event_count')),'total_resync':sum(counters('resync_count')) if counters('resync_count') else None,
            'hard_guard_cases':sum(row['hard_guard_exceeded'] for row in group),
            'max_recorded_penetration_mm':1000*max(row['grf_penetration_max_m'] for row in group),
            'common_prefix_mean_rmse_degrees':{joint:float(np.mean([row['common_prefix_rmse_degrees'][joint] for row in group])) for joint in JOINTS},
            'common_prefix_mean_learned_std':np.mean([row['common_prefix_action_metrics']['learned_std_mean_normalized'] for row in group],axis=0).tolist(),
            'common_prefix_mean_clip_fraction':float(np.mean([row['common_prefix_action_metrics']['clipped_component_fraction'] for row in group]))}
        signed=[row['common_prefix_signed_torque_after200ms'] for row in group if 'common_prefix_signed_torque_after200ms' in row]
        item['common_prefix_mean_signed_torque_metrics']={short:{name:float(np.mean([r[short][name] for r in signed])) for name in ('rms_nm','positive_above_8nm_fraction','highpass_above_20hz_rms_nm')} for short in ('knee','ankle')} if signed else None
        groups.append(item)
    result={'complete_screening':True,'groups':groups,'episodes':list(episodes.values()),'paired_noise_audit':paired,
        'common_prefix_steps':common,'common_prefix_seconds':common*.01,'checkpoint_provenance':provenances,
        'method':{'noise_scale':1,'noise_seeds':list(range(123,128)),'environment_seed':123,'same_prescribed_target_clock':True,
                  'each_policy_retains_own_learned_std':True,'new_simulations':5,'archived_simulations_reused':15},
        'limitations':['Five noise seeds and one fixed start are a diagnostic screen, not statistical equivalence or independent training replicates.',
            'V26 best87 weights are replayed in the current RIGHT39/FSMv3 evaluation path, not newly simulated in the entire historical August software stack.',
            'Matching standard Gaussian draws does not equalize absolute noise because learned std and visited states differ.',
            'Terminal partial integration rows are excluded from physical metrics; a hard penetration guard can exceed the maximum saved in summary.',
            'Completed episode, two validated cycles and zero invalid events are distinct criteria.',
            'Physical metrics use a common prefix across all20episodes; whole-episode returns retain their actual duration.']}
    FIGURES.mkdir(parents=True,exist_ok=True)
    figure_paths=[]
    fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained',sharex=True,sharey=True)
    for ax,label in zip(axes.flat,ROOTS):
        for seed in range(123,128):
            rows=valid[label,seed];elapsed=np.asarray([r['time'] for r in rows])-rows[0]['policy_segment_times'][0]
            ax.plot(elapsed,[1000*r['reward_terms'].get('grf_penetration_m',0) for r in rows],label=str(seed),lw=1)
        group=next(g for g in groups if g['policy']==label)
        ax.axhline(28,color='gray',ls='--',lw=.8)
        ax.set(title=f"{LABELS[label]}: {group['completed']}/5 completi, {group['two_valid_cycles']}/5 con due cicli",xlabel='Tempo dal reset [s]',ylabel='Penetrazione registrata [mm]')
        ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Rumore originale: seed gaussiani appaiati, std appresa da ogni policy')
    path=FIGURES/'noise_paired_penetration.png';fig.savefig(path,dpi=165);plt.close(fig);figure_paths.append(str(path))
    fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained')
    labels=[LABELS[k] for k in ROOTS];xx=np.arange(4)
    rates=np.array([g['completion_fraction'] for g in groups]);bounds=np.array([g['wilson95'] for g in groups])
    axes[0,0].bar(xx,rates,color=['#879cad','#ba9c6c','#458e73','#606070'])
    axes[0,0].errorbar(xx,rates,yerr=np.vstack((rates-bounds[:,0],bounds[:,1]-rates)),fmt='none',color='black',capsize=4)
    axes[0,0].set(title='Episodi completi (Wilson95%, n=5)',ylim=(0,1.06),ylabel='Frazione')
    axes[0,1].bar(xx,[g['two_valid_cycles'] for g in groups],color='#668eaa');axes[0,1].set(title='Almeno due cicli validati',ylim=(0,5.4),ylabel='Episodi su5')
    for column,joint in enumerate(JOINTS):
        values=[[episodes[label,seed]['common_prefix_rmse_degrees'][joint] for seed in range(123,128)] for label in ROOTS]
        axes[1,column].boxplot(values,positions=xx,widths=.4)
        for index,vals in enumerate(values):
            axes[1,column].scatter(np.linspace(index-.12,index+.12,5),vals,s=20)
        axes[1,column].set(title=f"RMSE {'ginocchio' if column==0 else 'caviglia'}: primi{common*.01:.2f}s",ylabel='Gradi')
    for ax in axes.flat:
        ax.set_xticks(xx,labels,rotation=12);ax.grid(axis='y',alpha=.2)
    path=FIGURES/'noise_completion_and_tracking.png';fig.savefig(path,dpi=165);plt.close(fig);figure_paths.append(str(path))
    fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained',sharex=True,sharey=True)
    for ax,label in zip(axes.flat,ROOTS):
        for seed in range(123,128):
            if (label,seed) in signed_series:
                tt,values=signed_series[label,seed];ax.plot(tt,values,lw=.65,label=str(seed))
        ax.axhline(0,color='black',lw=.8);ax.set(title=LABELS[label],xlabel='Tempo dal reset [s]',ylabel='Coppia SEA caviglia [Nm]')
        ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle(f'Coppia con segno canonico: stessa finestra0.2–{common*.01:.2f}s')
    path=FIGURES/'noise_signed_ankle_torques.png';fig.savefig(path,dpi=165);plt.close(fig);figure_paths.append(str(path))
    result['figures']=figure_paths
    h.dump(HERE/'metrics.json',result)
    h.dump(HERE/'analysis_input_hashes.json',{**h.INPUT_HASHES,**source_hashes})
    # Recheck every read artifact, including immutable old checkpoints: analysis is read-only.
    for name,value in {**h.INPUT_HASHES,**source_hashes,**frozen}.items():
        assert hash_file(REPO/name)==value,name
    lines=['# Robustezza stocastica RIGHT39 best58','',f'Cinque seed appaiati, rumore originale, start unico. Metriche fisiche sui primi{common*.01:.2f}s comuni.',
        '', '| Policy | Completi | ≥2cicli | Completi,2cicli,0invalidi | Return medio | RMSE knee | RMSE ankle |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for group in groups:
        errors=group['common_prefix_mean_rmse_degrees']
        lines.append(f"|{LABELS[group['policy']]}|{group['completed']}/5|{group['two_valid_cycles']}/5|{group['completed_with_two_cycles_and_zero_invalid']}/5|{group['mean_return']:.2f}|{errors[JOINTS[0]]:.2f}°|{errors[JOINTS[1]]:.2f}°|")
    lines+=['','Ogni policy conserva la propria deviazione standard: stessi numeri gaussiani non significano perturbazioni assolute identiche. Cinque episodi non provano equivalenza statistica. V26 è il replay dei pesi best87 nel percorso corrente, non una replica completa del software di agosto.','',*[f'- [Grafico](<{path}>)' for path in figure_paths]]
    (HERE/'summary.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({'groups':groups,'common_prefix_steps':common,'figures':figure_paths},indent=2))


if __name__=='__main__':
    main()
