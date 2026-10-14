"""Final PPO audit; explicitly marks optional episode telemetry as unavailable."""
import hashlib
import json
import pickle
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[1]
ROOT=PACKAGE.parents[1]
TRAIN=PACKAGE/'runs/training/MLP39_masking_sigma_verification_2026-10-01'
ORIGINAL=PACKAGE/'runs/training/MLP39_LEFT_best90_masking_100iter_2026-09-30/stage01_mask25'
PLOTS=ROOT/'plot/10_01_2026_masking_verification_comparison'
KEYS=('episode_return_mean','episode_len_mean','entropy','mean_kl_loss','current_kl_coeff','policy_loss','vf_loss')

def read_json(path):return json.loads(path.read_text())
def rows(path):return [json.loads(line) for line in path.read_text().splitlines()]

def main():
    results={}
    for treatment in ('learned','frozen_cp90'):
        folder=TRAIN/treatment
        if not (folder/'train_iterations.jsonl').exists():continue
        data=rows(folder/'train_iterations.jsonl')
        if not data:continue
        records=[]
        for r in data:
            audit=r['unified_phase_audit']
            sigma=audit['sigma_diagnostic']
            if treatment=='frozen_cp90':
                assert all(s['logstd_max_delta_vs_cp90_on_fixed_probes']==0 for s in sigma)
            mask=audit['target_availability_batch']
            assert mask['total_steps']==4096
            records.append({'iteration':r['iteration'],**{k:r[k] for k in KEYS},
                'total_steps':mask['total_steps'],'visible_steps':mask['visible_steps'],
                'visible_fraction':mask['visible_fraction'],'sigma':sigma})
        initialization=read_json(folder/'sigma_initialization_audit.json')
        digest=initialization['learner'][0]['reference_sha256']
        assert all(s['reference_sha256']==digest for r in records for s in r['sigma'])
        telemetry=list((folder/'episode_telemetry').glob('*.jsonl'))
        results[treatment]={'iterations':records,'initialization':initialization,
            'sampled_steps':sum(r['total_steps'] for r in records),
            'visible_steps':sum(r['visible_steps'] for r in records),
            'episode_telemetry_available':bool(telemetry),
            'episode_groups':None,'tail_sigma':None,
            'episode_telemetry_note':'Optional UNIFIED39_EPISODE_TELEMETRY_DIR was not enabled. Missing records do not mean zero completed episodes. PPO aggregates and recorded evaluation trajectories are available.'}
    reproduction=[]
    if 'learned' in results:
        original=rows(ORIGINAL/'train_iterations.jsonl')
        control=rows(TRAIN/'learned/train_iterations.jsonl')
        for old,new in zip(original,control):
            iteration=new['iteration']
            relative=f'milestone_iteration_{iteration:06d}/rl_module_last/module_state.pkl'
            if not (TRAIN/'learned'/relative).exists():continue
            a=pickle.loads((ORIGINAL/relative).read_bytes())
            b=pickle.loads((TRAIN/'learned'/relative).read_bytes())
            full_relative=f'milestone_iteration_{iteration:06d}/checkpoint_last/learner_group/learner/rl_module/default_policy/module_state.pkl'
            full_a=pickle.loads((ORIGINAL/full_relative).read_bytes())
            full_b=pickle.loads((TRAIN/'learned'/full_relative).read_bytes())
            critic_keys=[k for k in full_a if k.startswith('vf')]
            assert critic_keys, 'Full learner checkpoint has no critic tensors'
            reproduction.append({'iteration':iteration,
                'metrics_exact':all(old[k]==new[k] for k in KEYS),
                'original_state_exact':all(np.array_equal(v,b[k]) for k,v in a.items()),
                'state_tensors':len(a),'full_learner_state_exact':all(np.array_equal(v,full_b[k]) for k,v in full_a.items()),
                'critic_tensor_count':len(critic_keys),'critic_exact':all(np.array_equal(full_a[k],full_b[k]) for k in critic_keys),
                'mask_batch_exact':old['unified_phase_audit']['target_availability_batch']==new['unified_phase_audit']['target_availability_batch']})
    comparison={}
    if len(results)==2:
        a=results['learned'];b=results['frozen_cp90']
        comparison={'same_reference_digest':a['initialization']['learner'][0]['reference_sha256']==b['initialization']['learner'][0]['reference_sha256'],
            'initial_completed_episode_return_equal':a['iterations'][0]['episode_return_mean']==b['iterations'][0]['episode_return_mean'],
            'initial_completed_episode_length_equal':a['iterations'][0]['episode_len_mean']==b['iterations'][0]['episode_len_mean'],
            'initial_visible_steps_equal':a['iterations'][0]['visible_steps']==b['iterations'][0]['visible_steps']}
        assert comparison['same_reference_digest']
    output={'arms':results,'control_reproduction':reproduction,'initial_comparison':comparison,
        'limits':['PPO episode return/length can be rolling completed-episode metrics, not fresh per-update means.',
            'No optional per-episode training files were recorded in these launches; do not infer completion rates or visibility-specific training RMSE from aggregate PPO metrics.',
            'Evaluation traces provide visibility-specific rewards, kinematics, sigma on visited states, and termination outcomes.',
            'Ten updates per treatment and one seed do not establish long-horizon or population-level effects.']}
    (HERE/'training_summary.json').write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    (HERE/'control_reproduction.json').write_text(json.dumps(reproduction,indent=2)+'\n')
    fig,axes=plt.subplots(3,2,figsize=(13,10),sharex=True,layout='constrained')
    keys=('episode_return_mean','episode_len_mean','entropy','mean_kl_loss','visible_fraction','sigma_delta')
    for treatment,result in results.items():
        rr=result['iterations'];xx=[r['iteration'] for r in rr]
        for ax,key in zip(axes.flat,keys):
            yy=[max(s['logstd_max_delta_vs_cp90_on_fixed_probes'] for s in r['sigma']) if key=='sigma_delta' else r[key] for r in rr]
            ax.plot(xx,yy,'o-',label=treatment)
    for ax,key in zip(axes.flat,keys):ax.set_title(key);ax.grid(alpha=.25);ax.legend()
    for ax in axes[-1]:ax.set_xlabel('Global update')
    fig.suptitle('PPO comparison — sigma_delta is max absolute log-sigma difference on 40 fixed probes')
    PLOTS.mkdir(exist_ok=True)
    fig.savefig(PLOTS/'training_comparison.png',dpi=150);plt.close(fig)
    files=[Path(__file__)]+[TRAIN/t/'train_iterations.jsonl' for t in results]
    files.append(ORIGINAL/'train_iterations.jsonl')
    for record in reproduction:
        iteration=record['iteration']
        for parent in (ORIGINAL,TRAIN/'learned'):
            files.extend([parent/f'milestone_iteration_{iteration:06d}/rl_module_last/module_state.pkl',
                parent/f'milestone_iteration_{iteration:06d}/checkpoint_last/learner_group/learner/rl_module/default_policy/module_state.pkl'])
    (HERE/'training_analysis_hashes.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2)+'\n')
    print(json.dumps({'updates':{t:len(r['iterations']) for t,r in results.items()},'initial_comparison':comparison,'control_reproduction':reproduction},indent=2))

if __name__=='__main__':main()
