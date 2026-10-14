"""Paired, gated reward curriculum. Each subprocess and decision is journaled."""
from concurrent.futures import ThreadPoolExecutor
import copy, hashlib, json, os, pickle, subprocess, sys, time
from pathlib import Path
import numpy as np
import yaml
HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[1]
ROOT=PACKAGE.parents[1]
sys.path.insert(0,str(PACKAGE.parent));sys.path.insert(0,str(HERE))
TRAIN=PACKAGE/'runs/training/MLP39_reward_curriculum_2026-10-01'
ROLLOUT=PACKAGE/'runs/rollout/reward_curriculum_2026-10-01'
ARMS=('ex_novo','imitation')


def write(name,payload):
    path=HERE/(name+'.json'); temp=path.with_suffix('.json.tmp')
    temp.write_text(json.dumps(payload,indent=2,allow_nan=False)+'\n');temp.replace(path)


def environment():
    env=dict(os.environ,MPLBACKEND='Agg')
    env['PYTHONPATH']=str(PACKAGE.parent)+os.pathsep+env.get('PYTHONPATH','')
    for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','VECLIB_MAXIMUM_THREADS'):env[k]='1'
    return env


def execute(label,command):
    record={'label':label,'command':command,'started':time.time()}
    with (HERE/f'{label}.log').open('x') as log:
        process=subprocess.Popen(command,cwd=ROOT,env=environment(),stdout=log,stderr=subprocess.STDOUT)
        record['pid']=process.pid;write(label+'_execution',record)
        record['exit_code']=process.wait()
    record['elapsed_s']=time.time()-record['started'];write(label+'_execution',record)
    print(json.dumps(record),flush=True)
    if record['exit_code']:raise RuntimeError(f'{label}: exit {record["exit_code"]}')
    return record


def evaluate(case):
    label,checkpoint,visible=case
    output=ROLLOUT/label
    execute(label,[sys.executable,'-u',str(PACKAGE/'rollout_eval.py'),'--checkpoint',str(checkpoint),'--output-dir',str(output),
        '--episode-duration','5','--episode-start-offset-s','1','--max-steps','500','--seed','123',
        '--action-selection','deterministic','--target-availability-probability',str(visible),'--record-outputs','--record-policy-trace'])
    summary=json.loads((output/'rollout_summary.json').read_text());assert summary['ok'],summary
    from metrics import measure
    # Metrics module import/setup writes only its dedicated directory; each label is unique.
    return {'label':label,'output':str(output),'metrics':measure(label,output)}


def evaluate_group(cases):
    from metrics import measure  # Import once before thread creation.
    with ThreadPoolExecutor(max_workers=3) as pool:return list(pool.map(evaluate,cases))


def audit_training(folder,expected_total,expected_run):
    summary=json.loads((folder/'summary.json').read_text())
    assert summary['ok'] and summary['iterations_completed']==expected_total,summary
    assert summary['iterations_run']==expected_run and not summary.get('skipped_iterations'),summary
    rows=[json.loads(l) for l in (folder/'train_iterations.jsonl').read_text().splitlines() if l.strip()]
    for r in rows:
        a=r['unified_phase_audit']
        if r['unified_phase']=='critic_warmup':assert a['actor_exact'] and a['critic_changed']
        else:assert a['mean_changed'] and a['critic_changed']
    state=pickle.loads((folder/'checkpoint_last/learner_group/learner/rl_module/default_policy/module_state.pkl').read_bytes())
    np.testing.assert_array_equal(state['pi.1.weight'][2:],0)
    np.testing.assert_allclose(np.exp(state['pi.1.bias'][2:]),.01,rtol=1e-6)
    return {'folder':str(folder),'total_iterations':expected_total,'new_iterations':expected_run,'sigma_constant':True,'phases':[r['unified_phase'] for r in rows]}


def main():
    assert json.loads((HERE/'smoke_verification.json').read_text())['passed']
    manifest=json.loads((HERE/'source_manifest.json').read_text())
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in manifest.items())
    scripts=list(PACKAGE.glob('*.py'))+list(HERE.glob('*.py'))+list(HERE.glob('*.yaml'))
    write('execution_code_manifest',{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in scripts})
    records={'protocol':str(HERE/'PROTOCOL.md'),'started':time.time(),'training':[],'evaluations':[],'decisions':[]}
    write('progress',{'phase':'initial_rollouts'})
    cases=[(f'initial_{a}_{"visible" if v else "hidden"}',TRAIN/'smoke'/a/'milestone_iteration_000001/rl_module_last',v) for a in ARMS for v in (0,1)]
    records['evaluations']+=evaluate_group(cases);write('results',records)
    # The actor means and physical runtime must give the same initial trajectory across rewards.
    for v in ('visible','hidden'):
        traces=[json.loads((ROLLOUT/f'initial_{a}_{v}/rollout_policy_trace.json').read_text()) for a in ARMS]
        assert len(traces[0])==len(traces[1])
        for key in ('raw_policy_action','actor_observation_vector_before'):
            np.testing.assert_allclose([r[key] for r in traces[0]],[r[key] for r in traces[1]],rtol=0,atol=1e-7)
    write('initial_physical_parity',{'passed':True,'visible_and_hidden':True})
    total=0;previous={};stop='full_curriculum_completed'
    for stage,p in enumerate((.75,.5,.25,0.),1):
        configs={};outputs={}
        for a in ARMS:
            cfg=yaml.safe_load((HERE/f'{a}.yaml').read_text())
            output=TRAIN/a/f'stage{stage:02d}'; outputs[a]=output
            cfg['run']['output_dir']=str(output)
            if stage>1:
                cfg['run'].update(init_actor_from=None,resume_from=str(previous[a]/'checkpoint_last'),resume_target_curriculum_fork=True)
                cfg['target_availability'].update(target_availability_probability=p,target_curriculum_probabilities=[p],target_curriculum_iteration_offset=total)
            configs[a]=cfg
        for block in (1,2):
            new=6 if total==0 else 5
            target=total+new
            for a in ARMS:
                cfg=configs[a];cfg['simulation']['iterations']=target
                if block==2:
                    cfg['run'].update(init_actor_from=None,resume_from=str(outputs[a]/'checkpoint_last'))
                path=HERE/f'run_{a}_s{stage}_b{block}.yaml';path.write_text(yaml.safe_dump(cfg,sort_keys=False))
                write('progress',{'phase':'training','arm':a,'stage':stage,'block':block,'visibility':p,'target_iteration':target})
                execute(f'train_{a}_s{stage}_b{block}',[sys.executable,'-u',str(PACKAGE/'train_ppo_mlp.py'),'--config',str(path)])
                records['training'].append(audit_training(outputs[a],target,new));write('results',records)
            total=target
            write('progress',{'phase':'evaluating','stage':stage,'block':block,'iteration':total})
            cases=[(f'{a}_s{stage}_u{total-1}_{"visible" if v else "hidden"}',outputs[a]/f'milestone_iteration_{total:06d}/rl_module_last',v) for a in ARMS for v in (0,1)]
            evaluations=evaluate_group(cases);records['evaluations']+=evaluations
            hidden=next(x for x in evaluations if x['label']==f'ex_novo_s{stage}_u{total-1}_hidden')
            passed=hidden['metrics']['gate']['passed']
            decision={'stage':stage,'block':block,'actor_updates_per_arm':total-1,'visibility':p,'gate':hidden['metrics']['gate'],
                      'decision':'advance' if passed and stage<4 else ('complete' if stage==4 and block==2 else ('extend' if block==1 else 'stop_no_recovery'))}
            records['decisions'].append(decision);write('results',records)
            if passed and stage<4:break
            if block==2:
                if not passed:stop='stopped_at_quality_gate'
                break
        previous=outputs
        if stop=='stopped_at_quality_gate':break
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in manifest.items())
    records.update(stop_reason=stop,finished=time.time(),actor_updates_per_arm=total-1,inputs_unchanged=True)
    write('results',records);write('progress',{'phase':'training_and_rollouts_complete','stop_reason':stop,'actor_updates_per_arm':total-1})

if __name__=='__main__':main()
