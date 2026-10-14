"""Recompute the completed experiment from immutable training and rollout files.

Offline only: no optimizer, simulator, or checkpoint writes. Outputs are local
to this analysis folder. RMSEs of truncated episodes are never pooled as if they
were complete episodes; a separate equal-prefix comparison is supplied.
"""
from pathlib import Path
import os
import sys
import json
import hashlib
import importlib.util
from datetime import datetime

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
TG = PACKAGE.parent
REPO = TG.parent
PLAN = PACKAGE / 'runs/plans/phase_clock_ablation_2026-09-18'
os.environ.setdefault('MPLCONFIGDIR', str(HERE / '.matplotlib'))
os.environ.setdefault('OMP_NUM_THREADS', '1')
sys.path.insert(0, str(TG))
from unified_MLP_39._bootstrap import ensure_sim_paths
ensure_sim_paths()
from baseline_MLP import win_runtime  # noqa: F401
from unified_MLP_39 import reward_function
from baseline_MLP import reward_function as historical_reward
import numpy as np
import torch
from scipy.special import ndtr
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from ray.rllib.core.rl_module.rl_module import RLModule

JOINTS = ['pros_knee_angle', 'pros_ankle_angle']
LABELS = {'baseline_phase': 'Clock, qdot / 10, slew',
          'velocity_scale1': 'Clock, qdot / 1, slew',
          'no_slew': 'Clock, qdot / 10, senza slew'}
COLORS = dict(zip(LABELS, ['#2766a8', '#218355', '#bf5928']))
INPUT_HASHES = {}


def read(path):
    path = Path(path)
    data = path.read_bytes()
    INPUT_HASHES[str(path.relative_to(REPO))] = hashlib.sha256(data).hexdigest()
    return json.loads(data)


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def array(trace, field, joint):
    return np.asarray([r[field][joint] for r in trace], dtype=float)


def rmse(a, b):
    return float(np.rad2deg(np.sqrt(np.mean((a-b)**2))))


def joint_metrics(trace, joint):
    q = array(trace, 'prosthetic_state', joint)
    target = array(trace, 'imitation_target_q', joint)
    served = array(trace, 'prosthetic_state', joint+'_served_ref')
    phase = array(trace, 'reward_terms', joint+'_imitation_target_phase')
    result = {'rmse_actual_target_deg': rmse(q, target),
              'rmse_served_target_deg': rmse(served, target),
              'rmse_actual_served_deg': rmse(q, served),
              'correlation': float(np.corrcoef(q, target)[0, 1]),
              'actual_min_max_deg': np.rad2deg([q.min(), q.max()]).tolist(),
              'target_min_max_deg': np.rad2deg([target.min(), target.max()]).tolist(),
              'actual_excursion_deg': float(np.rad2deg(np.ptp(q))),
              'target_excursion_deg': float(np.rad2deg(np.ptp(target))),
              'position_range_rad': trace[0]['reward_terms'][joint+'_imitation_position_range_rad'],
              'velocity_range_rad_s': trace[0]['reward_terms'][joint+'_imitation_velocity_range_rad_s']}
    for label, mask in [('phase_0_60', phase < .6), ('phase_60_100', phase >= .6)]:
        result[label] = {'samples': int(mask.sum()), 'rmse_deg': rmse(q[mask], target[mask]) if mask.any() else None}
    return result


def rollout_metrics(folder, *, historical=False, scale=None):
    summary = read(folder/'rollout_summary.json')
    trace = read(folder/'rollout_policy_trace.json')
    assert summary['ok'] and summary['action_selection'] == 'deterministic'
    assert len(trace) == summary['steps'] and len(trace) > 0
    if scale is not None:
        for i, row in enumerate(trace):
            obs = row['actor_observation_before']
            assert row['target_availability']['visible']
            np.testing.assert_allclose(obs['gait_phase_sin']**2 + obs['gait_phase_cos']**2, 1, atol=2e-6)
            if i:
                prev = trace[i-1]
                phase = prev['reward_terms']['pros_knee_angle_imitation_target_phase']
                np.testing.assert_allclose([obs['gait_phase_sin'], obs['gait_phase_cos']],
                                          [np.sin(2*np.pi*phase), np.cos(2*np.pi*phase)], atol=2e-6)
                for joint, short in zip(JOINTS, ['knee', 'ankle']):
                    np.testing.assert_allclose(obs[f'prescribed_left_{short}_q_scaled'], prev['imitation_target_q'][joint], atol=2e-6)
                    np.testing.assert_allclose(obs[f'prescribed_left_{short}_qdot_scaled'], prev['imitation_target_qdot'][joint]/scale, atol=2e-6)
    mod = historical_reward if historical else reward_function
    cfg = mod.RewardConfig.from_mapping(summary['reward_config'])
    rewards = [mod.compute_reward(r['reward_terms'], cfg, reference=r['policy_segment_values']) for r in trace]
    return_error = abs(sum(r for r, _ in rewards)-summary['episode_return'])
    assert return_error < 1e-6
    comp = {k: float(sum(c[k] for _, c in rewards)) for k in rewards[0][1]
            if k.endswith('_term') or k in ['reward', 'reward_base', 'penalty', 'imitation_score', 'served_imitation_score']}
    weights = {'command_rate_weight': 'command_rate_loss', 'policy_action_clip_weight': 'policy_action_clip_loss',
               'effort_weight': 'effort_loss', 'smoothness_weight': 'smoothness_loss',
               'sea_motor_speed_weight': 'sea_motor_speed_loss', 'sea_motor_accel_weight': 'sea_motor_accel_loss',
               'sea_motor_power_weight': 'sea_motor_power_loss', 'phase_invalid_event_weight': 'invalid_event_loss',
               'phase_contact_validity_weight': 'contact_validity_loss'}
    penalty = {k: float(getattr(cfg, k)*sum(r['reward_terms'].get(v, 0) for r in trace)) for k, v in weights.items()}
    result = {k: summary[k] for k in ['steps', 'episode_return', 'end_reason', 'terminated', 'truncated',
                                    'phase_valid_cycle_count', 'grf_penetration_max_m', 'episode_start_offset_s', 'checkpoint']}
    result.update(rollout=str(folder.relative_to(REPO)), duration_s=len(trace)*.01,
                  reward_reconstruction_error=return_error, reward_components_sums=comp,
                  penalty_components_sums=penalty,
                  imitation_loss_means={k:float(np.mean([r['reward_terms'][k] for r in trace]))
                                        for k in ['sound_imitation_loss','served_imitation_loss']},
                  joints={j:joint_metrics(trace, j) for j in JOINTS},
                  final_knee_deg=float(np.rad2deg(trace[-1]['prosthetic_state'][JOINTS[0]])),
                  final_knee_served_deg=float(np.rad2deg(trace[-1]['prosthetic_state'][JOINTS[0]+'_served_ref'])),
                  final_knee_target_deg=float(np.rad2deg(trace[-1]['imitation_target_q'][JOINTS[0]])),
                  governor_limit_mean={k:float(np.mean([r['reference_governor_diagnostics'].get(k,0) for r in trace]))
                                       for k in ['velocity_limit_fraction','acceleration_limit_fraction','jerk_limit_fraction']})
    if scale is not None:
        result['six_prescribed_inputs_verified'] = True
    return result, trace, summary


def policy_audit(trace, summary, *, slew):
    checkpoint = Path(summary['checkpoint'])
    INPUT_HASHES[str((checkpoint/'module_state.pkl').relative_to(REPO))] = hashlib.sha256((checkpoint/'module_state.pkl').read_bytes()).hexdigest()
    module = RLModule.from_checkpoint(str(checkpoint))
    module.eval()
    names = summary['actor_feature_names'][:module._n_actor]
    obs = np.asarray([r['actor_observation_vector_before'][:module._n_actor] for r in trace], dtype=np.float32)
    teacher = [i for i,n in enumerate(names) if 'imitation_target' in n or n.startswith('prescribed_left_')]
    clock = [names.index('gait_phase_sin'), names.index('gait_phase_cos')]
    velocities = [i for i in teacher if 'vel' in names[i] or 'qdot' in names[i]]
    def forward(x):
        with torch.no_grad():
            return module.forward_inference({'obs':torch.as_tensor(x)})['action_dist_inputs'].numpy()
    logits = forward(obs);mean=logits[:,:2];std=np.exp(logits[:,2:])
    error = float(np.max(np.abs(mean-np.asarray([r['raw_policy_action'] for r in trace]))))
    assert error < 2e-6
    mutations = {}
    for label, indices in [('four_targets_zero',teacher),('clock_zero',clock),('all_six_zero',teacher+clock),('target_velocities_zero',velocities)]:
        changed=obs.copy();changed[:,indices]=0
        mutations[label]=np.sqrt(np.mean((forward(changed)[:,:2]-mean)**2,axis=0)).tolist()
    changed=obs[25:].copy();changed[:,teacher+clock]=obs[:-25,teacher+clock]
    mutations['all_six_delayed_250ms_no_wrap']=np.sqrt(np.mean((forward(changed)[:,:2]-mean[25:])**2,axis=0)).tolist()
    raw=np.asarray([r['raw_policy_action'] for r in trace]);applied=np.clip(raw,-1,1)
    midpoint=np.asarray([-.75,0]);half=np.asarray([.75,.7])
    endpoints=np.asarray([r['policy_segment_values'][-1] for r in trace])
    # The slew limiter anchors the previous policy endpoint, not the current
    # segment's initial knot (the latter follows the served governor state).
    anchors=np.vstack((np.asarray(trace[0]['policy_segment_values'][0]),endpoints[:-1]))
    lo=np.full_like(mean,-1);hi=np.full_like(mean,1)
    if slew:
        dt=np.asarray([r['policy_segment_times'][-1]-r['policy_segment_times'][0] for r in trace])[:,None]
        lo=np.maximum(-1,(anchors-np.asarray([2.5,2])*dt-midpoint)/half)
        hi=np.minimum(1,(anchors+np.asarray([2.5,2])*dt-midpoint)/half)
    reconstructed=midpoint+half*np.clip(raw,lo,hi)
    np.testing.assert_allclose(reconstructed,endpoints,atol=2e-7,rtol=0)
    a=(lo-mean)/std;b=(hi-mean)/std
    phi=lambda z:np.exp(-z*z/2)/np.sqrt(2*np.pi)
    expected=mean*(ndtr(b)-ndtr(a))+std*(phi(a)-phi(b))+lo*ndtr(a)+hi*(1-ndtr(b))
    expected_q=midpoint+half*expected
    return {'actor_width':module._n_actor,'full_width':module._n_full,'reproduced_action_error':error,
            'policy_std_mean':std.mean(axis=0).tolist(),
            'input_zeroing_action_mean_delta_rms':mutations,
            'deterministic_absolute_clip_fraction':np.mean(np.abs(raw)>1,axis=0).tolist(),
            'conditional_gaussian_absolute_clip_probability':np.mean(ndtr((-1-mean)/std)+1-ndtr((1-mean)/std),axis=0).tolist(),
            'deterministic_slew_fraction':np.mean(np.abs((midpoint+half*applied)-endpoints)>1e-7,axis=0).tolist() if slew else [0,0],
            'conditional_gaussian_combined_clip_probability':np.mean(ndtr(a)+1-ndtr(b),axis=0).tolist(),
            'conditional_expected_endpoint_vs_deterministic_rms_deg':np.rad2deg(np.sqrt(np.mean((expected_q-endpoints)**2,axis=0))).tolist(),
            'caveat':'Offline sensitivity and conditional Gaussian expectations at visited deterministic states; no masked or stochastic physical rollout.'}


def main():
    spec=importlib.util.spec_from_file_location('queue_audit',PACKAGE/'validation/run_clock_ablations.py')
    queue=importlib.util.module_from_spec(spec);spec.loader.exec_module(queue)
    plan=read(PLAN/'plan.json');queue.validate_plan(plan);queue.source_guard(PLAN)
    saved=read(PLAN/'results.json');completed=read(PLAN/'status.json');assert completed['status']=='completed'
    results={'audit':{'sources_unchanged':True,'same_one_factor_configs':True,'completed_at':completed['updated_at']},
             'variants':{},'historical':{},'common_prefix':{}}
    histories={};traces={};summaries={}
    for v in plan['variants']:
        name=v['name'];run=REPO/v['run'];folder=REPO/v['rollout_root']
        audit=queue.training_audit(run,100);assert audit==saved[name]['training']
        summary=read(run/'summary.json')
        rows=[json.loads(line) for line in (run/'train_iterations.jsonl').read_text().splitlines() if line.strip()]
        INPUT_HASHES[str((run/'train_iterations.jsonl').relative_to(REPO))]=hashlib.sha256((run/'train_iterations.jsonl').read_bytes()).hexdigest()
        histories[name]=rows
        blocks={}
        for start,end in [(1,20),(21,40),(41,60),(61,80),(81,100)]:
            group=rows[start-1:end]; counts=np.array([sum(s['completed_episodes'] for s in r['per_start_metrics'].values()) for r in group])
            blocks[f'{start}-{end}']={
                'completed_episodes':int(counts.sum()),
                'completed_episode_weighted_return':float(sum(sum(s['episode_return_sum'] for s in r['per_start_metrics'].values() if s['completed_episodes']>0) for r in group)/counts.sum()),
                'completed_episode_weighted_length':float(sum(sum(s['episode_length_mean']*s['completed_episodes'] for s in r['per_start_metrics'].values() if s['completed_episodes']>0) for r in group)/counts.sum()),
                'mean_entropy':float(np.mean([r['entropy'] for r in group])),
                'mean_kl':float(np.mean([r['mean_kl_loss'] for r in group]))}
        bestrow=rows[audit['best_iteration']-1]
        training={**audit,'started_at':summary['started_at'],'finished_at':summary['finished_at'],
                  'wall_hours':summary['elapsed_wall_time_s']/3600,'blocks':blocks,
                  'best_training_episode_length':bestrow['episode_len_mean'],
                  'best_preupdate_actor_digest':bestrow['unified_phase_audit']['actor_before'],
                  'best_exported_postupdate_actor_digest':bestrow['unified_phase_audit']['actor_after'],
                  'last20_iteration_mean_return':float(np.mean([r['episode_return_mean'] for r in rows[-20:]])),
                  'last20_iteration_mean_length':float(np.mean([r['episode_len_mean'] for r in rows[-20:]])),
                  'last_return':rows[-1]['episode_return_mean'],'last_episode_length':rows[-1]['episode_len_mean'],
                  'max_minibatch_kl':max(r['max_minibatch_mean_kl_loss'] for r in rows),
                  'termination_counters_final':{k.split('episode_end/')[-1]:v for k,v in rows[-1]['termination_metrics'].items() if 'episode_end/' in k}}
        evaluations={}
        for start in ['early','center','late']:
            key=(name,start)
            evaluations[start],traces[key],summaries[key]=rollout_metrics(folder/start,scale=1 if name=='velocity_scale1' else 10)
            old=next(e for e in saved[name]['evaluations'] if abs(e['episode_start_offset_s']-evaluations[start]['episode_start_offset_s'])<1e-9)
            for j in JOINTS:
                np.testing.assert_allclose(evaluations[start]['joints'][j]['rmse_actual_target_deg'],old['joints'][j]['rmse_actual_target_deg'],atol=1e-10)
        receipt=read(folder/'center/plots/plot_receipt.json')
        canonical_plots=list((folder/'center/plots').rglob('*.png'));assert len(canonical_plots)==7
        results['variants'][name]={'training':training,'evaluations':evaluations,
                                  'complete_episodes':sum(e['steps']==500 and e['end_reason']=='episode_time_limit' for e in evaluations.values()),
                                  'mean_duration_s':float(np.mean([e['duration_s'] for e in evaluations.values()])),
                                  'canonical_plots':[str(f.relative_to(REPO)) for f in sorted(canonical_plots)],
                                  'policy_audit_center':policy_audit(traces[name,'center'],summaries[name,'center'],slew=name!='no_slew')}
    for start in ['early','center','late']:
        length=min(len(traces[name,start]) for name in LABELS)
        for name in LABELS:
            # The prescribed reference must be numerically identical on matched times.
            for j in JOINTS:
                np.testing.assert_allclose(array(traces[name,start][:length],'imitation_target_q',j),array(traces['baseline_phase',start][:length],'imitation_target_q',j),atol=1e-10)
        results['common_prefix'][start]={'steps':length,'duration_s':length*.01,
            'variants':{name:{j:joint_metrics(traces[name,start][:length],j) for j in JOINTS} for name in LABELS}}
    historical={
        'V26_aug20':TG/'runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout',
        '39D_clock_hidden_best54':PACKAGE/'runs/rollout/MLP39_left_phase_best_iter54_center_2026-09-18'}
    for name,folder in historical.items():
        result,t,s=rollout_metrics(folder,historical=name=='V26_aug20')
        result['policy_audit']=policy_audit(t,s,slew=name!='V26_aug20')
        if name=='V26_aug20':
            history_path=Path(s['checkpoint']).parent/'train_iterations.jsonl'
            content=history_path.read_bytes()
            INPUT_HASHES[str(history_path.relative_to(REPO))]=hashlib.sha256(content).hexdigest()
            rows=[json.loads(line) for line in content.splitlines() if line.strip()]
            assert len(rows)==100
            result['training_last20_iteration_mean_return']=float(np.mean([r['episode_return_mean'] for r in rows[-20:]]))
            result['training_last20_iteration_mean_length']=float(np.mean([r['episode_len_mean'] for r in rows[-20:]]))
        results['historical'][name]=result;traces[name,'center']=t;summaries[name,'center']=s
    start=datetime.fromisoformat(read(PLAN/'launch.json')['started_at']);end=datetime.fromisoformat(completed['updated_at'])
    results['audit'].update(total_elapsed_hours=(end-start).total_seconds()/3600,
        total_iterations=300,total_sampled_steps=1382400,deterministic_rollouts=9,canonical_plots=21,
        same_initial_actor=len({v['training']['actor_initial_digest'] for v in results['variants'].values()})==1,
        same_initial_critic=len({v['training']['critic_initial_digest'] for v in results['variants'].values()})==1,
        statistical_scope='One training seed and three training start offsets; descriptive paired diagnostics, not independent replications or held-out generalization.')
    dump(HERE/'metrics.json',results)
    dump(HERE/'input_hashes.json',INPUT_HASHES)
    plots(histories,traces,results)
    print(json.dumps({'audit':results['audit'],'output':str(HERE/'metrics.json')},indent=2))


def plots(histories,traces,results):
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axs=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
    for name,rows in histories.items():
        x=np.arange(1,101)
        for ax,key,title in zip(axs.flat,['episode_return_mean','episode_len_mean','entropy','mean_kl_loss'],
                               ['Return dei campioni PPO','Durata media episodi [step]','Entropia della policy','KL medio PPO']):
            y=np.asarray([r[key] for r in rows]);ax.plot(x,y,color=COLORS[name],alpha=.2,lw=.8)
            ax.plot(x[9:],np.convolve(y,np.ones(10)/10,mode='valid'),color=COLORS[name],label=LABELS[name],lw=1.7)
            ax.set_title(title);ax.grid(alpha=.2);ax.set_xlabel('Iterazione')
        b=results['variants'][name]['training'];axs[0,0].scatter(b['best_iteration'],b['best_train_return'],color=COLORS[name],s=35)
    axs[0,0].legend(fontsize=8);axs[0,1].axhline(500,color='black',ls='--',lw=.7)
    fig.suptitle('Tre training da 100 iterazioni · linee marcate: media mobile di 10 iterazioni')
    fig.savefig(HERE/'training_curves.png',dpi=160);plt.close(fig)
    for j,joint in enumerate(JOINTS):
        fig,axs=plt.subplots(3,3,figsize=(15,9),sharex=True,sharey=True,constrained_layout=True)
        for row,name in enumerate(LABELS):
            for col,start in enumerate(['early','center','late']):
                t=traces[name,start];x=np.arange(1,len(t)+1)*.01;sign=-1 if j==0 else 1;ax=axs[row,col]
                for field,key,label,color,ls in [('imitation_target_q',joint,'Target sinistro','#222222','--'),('prosthetic_state',joint+'_served_ref','Reference servita','#d28722','-'),('prosthetic_state',joint,'Giunto simulato',COLORS[name],'-')]:
                    ax.plot(x,sign*np.rad2deg(array(t,field,key)),label=label,color=color,ls=ls,lw=1.4)
                if len(t)<500:ax.axvspan(x[-1],5,color='#dddddd',alpha=.6);ax.axvline(x[-1],color='#9b3030',ls=':',lw=1)
                e=results['variants'][name]['evaluations'][start]
                ax.set_title(f"{start} · {e['duration_s']:.2f} s · RMSE {e['joints'][joint]['rmse_actual_target_deg']:.2f}°",fontsize=10)
                ax.set_xlim(0,5);ax.grid(alpha=.2)
                if col==0:ax.set_ylabel(LABELS[name]+'\n'+('Flessione ginocchio [°]' if j==0 else 'Caviglia [°]'))
                if row==2:ax.set_xlabel('Tempo dal reset [s]')
        axs[0,0].legend(fontsize=8)
        fig.suptitle('Rollout deterministici del best · area grigia: episodio già terminato')
        fig.savefig(HERE/('knee_all_starts.png' if j==0 else 'ankle_all_starts.png'),dpi=160);plt.close(fig)
    fig,axs=plt.subplots(2,3,figsize=(14,7),sharex=True,sharey='row',constrained_layout=True)
    for col,(name,label) in enumerate([('V26_aug20','V26 agosto · best87'),('39D_clock_hidden_best54','39D senza clock · best54'),('velocity_scale1','39D clock, scala1 · best78')]):
        t=traces[name,'center'];x=np.arange(1,len(t)+1)*.01
        for row,joint in enumerate(JOINTS):
            sign=-1 if row==0 else 1;ax=axs[row,col]
            for field,key,legend,color,ls in [('imitation_target_q',joint,'Target proprio','#222','--'),('prosthetic_state',joint+'_served_ref','Reference servita','#d28722','-'),('prosthetic_state',joint,'Giunto simulato','#2766a8','-')]:
                ax.plot(x,sign*np.rad2deg(array(t,field,key)),label=legend,color=color,ls=ls,lw=1.4)
            ax.grid(alpha=.2);ax.set_xlim(0,5)
            if row==0:ax.set_title(label)
            if row==1:ax.set_xlabel('Tempo dal reset [s]')
            if col==0:ax.set_ylabel('Flessione ginocchio [°]' if row==0 else 'Caviglia [°]')
    axs[0,0].legend(fontsize=8)
    fig.suptitle('Confronto storico descrittivo · V26 ha target e start diversi · tutti gli episodi durano 5 s')
    fig.savefig(HERE/'historical_comparison.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    main()
