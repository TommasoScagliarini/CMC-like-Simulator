"""Same physical metrics for either objective, with exact reward reconstruction."""
import importlib.util, json
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[1]
ROOT=PACKAGE.parents[1]
spec=importlib.util.spec_from_file_location('physical_analysis',PACKAGE/'validation/masking_diagnosis_2026-10-01/rollouts/analyze.py')
analysis=importlib.util.module_from_spec(spec); spec.loader.exec_module(analysis)
analysis.HERE=HERE/'metrics';analysis.HERE.mkdir(exist_ok=True)


def budget(trace,summary,window):
    cfg=analysis.RewardConfig.from_mapping(summary['reward_config'])
    components=[analysis.compute_reward(r['reward_terms'],cfg,reference=r['policy_segment_values'])[1] for r in trace]
    np.testing.assert_allclose(sum(c['reward'] for c in components),summary['episode_return'],atol=1e-7,rtol=0)
    sums={k:float(sum(c.get(k,0) for c in components)) for k in components[0] if k.endswith('_term') or k in ('reward','reward_base','penalty','imitation_score','served_imitation_score','phase_event_progress_score')}
    return {'mode':cfg.reward_mode,'full_episode':sums,'reconstruction_exact':True},components
analysis.reward_budget=budget


def measure(label,folder):
    folder=Path(folder)
    summary=json.loads((folder/'rollout_summary.json').read_text())
    trace=json.loads((folder/'rollout_policy_trace.json').read_text())
    if len(trace)<26:
        result={'steps':len(trace),'source':str(folder),'short_rollout':True,'joints':{},'fsm':{'end_reason':summary.get('end_reason'),'phase_valid_cycle_count':summary.get('phase_valid_cycle_count',0)}}
    else:
        result,_=analysis.analyze(label,folder)
    terms=[r['reward_terms'] for r in trace]
    result['morphology']={k:float(sum(r.get(k,0) for r in terms)) for k in ['morphology_available','morphology_settled_sample_count','morphology_knee_loss','morphology_ankle_loss','morphology_causal_failed_closed']}
    result['morphology']['last_diagnostics']=trace[-1].get('morphology_causal_diagnostics',{})
    visible=np.array([r['target_availability']['visible'] for r in trace])
    obs=np.asarray([r['actor_observation_vector_before'] for r in trace])
    if np.any(~visible):
        np.testing.assert_array_equal(obs[~visible][:,[0,1,35,36,37,38]],0)
    if np.any(visible):
        np.testing.assert_allclose(np.sum(obs[visible,:2]**2,axis=1),1,atol=2e-6)
    result['mask_verified']=True
    joints=result.get('joints',{})
    checks={'full_episode':result['steps']==500,'cycles':(summary.get('phase_valid_cycle_count',0) or 0)>=2,
            'knee_rom':joints.get('knee',{}).get('actual',{}).get('excursion_deg',0)>=20,
            'ankle_rom':joints.get('ankle',{}).get('actual',{}).get('excursion_deg',0)>=15,
            'tracking':all(joints.get(j,{}).get('rmse_actual_served_deg',float('inf'))<=2 for j in ('knee','ankle')),
            'normal_end':summary.get('end_reason')=='episode_time_limit'}
    result['gate']={'checks':checks,'passed':all(checks.values())}
    (HERE/'metrics'/f'{label}.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result
