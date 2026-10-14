"""Authorized two-arm 10-update comparison, followed by eight fixed-policy evals."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
ROOT = PACKAGE.parents[1]
TRAIN = PACKAGE / "runs/training/MLP39_masking_sigma_verification_2026-10-01"
ROLLOUT = PACKAGE / "runs/rollout/masking_verification_2026-10-01"


def write(name, value):
    temp = HERE / (name+".tmp")
    temp.write_text(json.dumps(value,indent=2)+"\n")
    temp.replace(HERE/name)


def execute(label, command, env):
    record = {"label":label,"command":command,"started":time.time()}
    with (HERE / f"{label}.log").open("x") as log:
        process = subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
        record['pid'] = process.pid
        write(f"{label}_launch.json",record)
        record['exit_code'] = process.wait()
    record['elapsed_s'] = time.time()-record['started']
    write(f"{label}_execution.json",record)
    if record['exit_code'] != 0:
        raise RuntimeError(f"{label} exited {record['exit_code']}; inspect retained log")
    print(json.dumps(record),flush=True)
    return record


def evaluate(case):
    treatment, iteration, visible = case
    label = f"{treatment}_cp{iteration}_{'visible' if visible else 'hidden'}"
    checkpoint = TRAIN / treatment / f"milestone_iteration_{iteration:06d}/rl_module_last"
    output = ROLLOUT / label
    if output.exists():
        raise FileExistsError(output)
    command = [sys.executable,"-u",str(PACKAGE / "rollout_eval.py"),
        "--checkpoint",str(checkpoint),"--output-dir",str(output),"--episode-duration","5",
        "--episode-start-offset-s","1","--max-steps","500","--seed","123",
        "--action-selection","deterministic","--target-availability-probability",str(visible),
        "--record-outputs","--record-policy-trace"]
    execute(label,command,environment())
    summary = json.loads((output/'rollout_summary.json').read_text())
    assert summary['ok']
    return {"label":label,"rollout":str(output),"checkpoint":str(checkpoint),
        **{k:summary[k] for k in ('steps','episode_return','end_reason')}}


def environment():
    env = dict(os.environ,MPLBACKEND="Agg")
    env['PYTHONPATH'] = str(PACKAGE.parent)+os.pathsep+env.get('PYTHONPATH','')
    for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','VECLIB_MAXIMUM_THREADS'):
        env[key]='1'
    return env


def main():
    assert json.loads((HERE/'preflight_sigma.json').read_text())['passed']
    if TRAIN.exists():
        raise FileExistsError(TRAIN)
    # Avoid competing with native rollout jobs while the 13-worker PPO runs.
    while not (HERE/'matrix_execution.json').exists():
        time.sleep(10)
    matrix = json.loads((HERE/'matrix_execution.json').read_text())
    assert len(matrix)==6 and all(r['exit_code']==0 and r['mask_verified'] for r in matrix)
    files = list(HERE.glob('*.py'))+list(HERE.glob('*.yaml'))
    source = PACKAGE/'runs/training/MLP39_LEFT_fresh_best80config_100iter_2026-09-29/milestone_iteration_000090/checkpoint_last'
    files += [p for p in source.rglob('*') if p.is_file()]
    hashes = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    write('training_manifest.json',hashes)
    records=[]
    for treatment in ('learned','frozen_cp90'):
        assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
        write('progress.json',{'phase':'training','treatment':treatment,'completed':records})
        env=environment();env['CMC_SIGMA_TREATMENT']=treatment
        execute('train_'+treatment,[sys.executable,'-u',str(HERE/'train_sigma.py'),
                                  '--config',str(HERE/f'{treatment}.yaml')],env)
        summary=json.loads((TRAIN/treatment/'summary.json').read_text())
        assert summary['ok'] and summary['iterations_completed']==100
        assert summary.get('iterations_run')==10
        assert not summary.get('skipped_iterations')
        for iteration in (95,100):
            assert (TRAIN/treatment/f'milestone_iteration_{iteration:06d}/rl_module_last').exists()
        records.append({'treatment':treatment,'summary':summary})
    write('progress.json',{'phase':'evaluating','completed_training':[r['treatment'] for r in records]})
    cases=[(t,i,v) for t in ('learned','frozen_cp90') for i in (95,100) for v in (1,0)]
    with ThreadPoolExecutor(max_workers=3) as pool:
        evaluations=list(pool.map(evaluate,cases))
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    write('comparison_execution.json',{'training':records,'evaluations':evaluations,'inputs_unchanged':True})
    write('progress.json',{'phase':'executions_complete','ppo_updates':20,'evaluations':evaluations})


if __name__=='__main__':
    main()
