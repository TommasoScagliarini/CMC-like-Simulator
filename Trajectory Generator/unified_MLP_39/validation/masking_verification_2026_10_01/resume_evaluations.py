"""Resume only evaluations after the standard validator rejected the subclass."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import sys
import time

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from run_sigma_comparison import PACKAGE,ROOT,TRAIN,ROLLOUT,execute,environment,write

def evaluate(case):
    treatment,iteration,visible=case
    label=f"{treatment}_cp{iteration}_{'visible' if visible else 'hidden'}"
    checkpoint=TRAIN/treatment/f'milestone_iteration_{iteration:06d}/rl_module_last'
    output=ROLLOUT/label
    if output.exists():raise FileExistsError(output)
    command=[sys.executable,'-u',str(HERE/'sigma_rollout.py'),'--checkpoint',str(checkpoint),
        '--output-dir',str(output),'--episode-duration','5','--episode-start-offset-s','1',
        '--max-steps','500','--seed','123','--action-selection','deterministic',
        '--target-availability-probability',str(visible),'--record-outputs','--record-policy-trace']
    execute(label,command,environment())
    summary=json.loads((output/'rollout_summary.json').read_text())
    assert summary['ok']
    return {'label':label,'rollout':str(output),'checkpoint':str(checkpoint),
        **{k:summary[k] for k in ('steps','episode_return','end_reason')}}

def main():
    assert json.loads((HERE/'evaluation_contract_preflight.json').read_text())['passed']
    hashes=json.loads((HERE/'training_manifest.json').read_text())
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    records=[]
    for treatment in ('learned','frozen_cp90'):
        summary=json.loads((TRAIN/treatment/'summary.json').read_text())
        assert summary['ok'] and summary['iterations_run']==10 and summary['iterations_completed']==100
        assert not summary['skipped_iterations']
        records.append({'treatment':treatment,'summary':summary})
    files=[Path(__file__),HERE/'sigma_rollout.py',PACKAGE/'actor_transition.py',PACKAGE/'rollout_eval.py']
    eval_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    write('evaluation_manifest.json',eval_hashes)
    write('progress.json',{'phase':'evaluating','completed_training':['learned','frozen_cp90'],
        'validator':'exact diagnostic subclass, all original structural checks retained'})
    cases=[(t,i,v) for t in ('learned','frozen_cp90') for i in (95,100) for v in (1,0)]
    with ThreadPoolExecutor(max_workers=3) as pool:evaluations=list(pool.map(evaluate,cases))
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in {**hashes,**eval_hashes}.items())
    write('comparison_execution.json',{'training':records,'evaluations':evaluations,'inputs_unchanged':True,
        'evaluation_validator_amendment':'sigma_rollout.py admits only SigmaComparisonModule and checks its CP90 reference and treatment; original feature/architecture/action/tensor checks retained.'})
    write('progress.json',{'phase':'executions_complete','ppo_updates':20,'evaluations':evaluations})

if __name__=='__main__':main()
