"""Five native RIGHT39 best58 Gaussian rollouts; no training or runtime edits."""
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[2]
REPO=PACKAGE.parents[1]
RUN=PACKAGE/'runs/training/MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23'
CHECKPOINT=RUN/'milestone_iteration_000058/rl_module_last'
ROOT=PACKAGE/'runs/rollout/MLP39_right_screening60_2026-09-23/best58'


def read(path):
    return json.loads(path.read_text())


def write(path,data):
    temp=path.with_name(path.name+'.tmp')
    temp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    temp.replace(path)


def now():
    return datetime.now().astimezone().isoformat()


def guard():
    for path,expected in read(HERE/'input_hashes.json').items():
        if hashlib.sha256((REPO/path).read_bytes()).hexdigest()!=expected:
            raise RuntimeError(f'Frozen input changed:{path}')


def cap():
    try:
        return 4 if read(HERE.parent/'rollout_status.json')['state']=='completed' else 2
    except (OSError,KeyError,json.JSONDecodeError):
        return 2


def main():
    with (HERE/'execution.lock').open('x') as stream:
        stream.write(str(os.getpid()))
    prior=read(HERE.parent/'rollout_input_hashes.json')
    assert all(hashlib.sha256((REPO/path).read_bytes()).hexdigest()==expected for path,expected in prior.items())
    assert CHECKPOINT.is_dir()
    files={REPO/path for path in prior}
    files.update(p for p in CHECKPOINT.rglob('*') if p.is_file())
    files.update((PACKAGE/'validation/completion_noise_rollout.py',Path(__file__)))
    hashes={str(p.relative_to(REPO)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    write(HERE/'input_hashes.json',hashes)
    env=dict(os.environ,PYTHONUNBUFFERED='1')
    for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','VECLIB_MAXIMUM_THREADS','NUMEXPR_NUM_THREADS'):
        env[key]='1'
    pending=list(range(123,128));active={};results=[];sleeper=None
    if sys.platform=='darwin' and Path('/usr/bin/caffeinate').exists():
        sleeper=subprocess.Popen(['/usr/bin/caffeinate','-i','-w',str(os.getpid())],stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    def status(state,**extra):
        progress={}
        for seed,(process,_,folder,started) in active.items():
            heartbeat=folder/'watchdog_state.json'
            try:
                progress[str(seed)]={'supervisor_pid':process.pid,**(read(heartbeat) if heartbeat.exists() else {'phase':'startup'})}
            except json.JSONDecodeError:
                progress[str(seed)]={'supervisor_pid':process.pid,'phase':'heartbeat update'}
        write(HERE/'status.json',{'state':state,'updated_at':now(),'pid':os.getpid(),'completed':len(results),'pending':pending,'active':progress,'allowed_noise_concurrency':cap(),'results':results,**extra})
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    try:
        while pending or active:
            while pending and len(active)<cap():
                guard();seed=pending.pop(0);folder=ROOT/f'noise_1_seed{seed}'
                assert not folder.exists(),folder
                command=[sys.executable,'-u',str(PACKAGE/'validation/completion_noise_rollout.py'),
                    '--checkpoint',str(CHECKPOINT),'--output-dir',str(folder),
                    '--noise-scale','1','--noise-seed',str(seed),'--seed','123',
                    '--max-steps','500','--record-outputs','--record-policy-trace','--no-progress','--run-timeout-s','2400']
                stream=(HERE/f'seed{seed}.log').open('x')
                child=subprocess.Popen(command,cwd=REPO,env=env,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,
                    creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name=='nt' else 0)
                active[seed]=(child,stream,folder,time.monotonic())
                launch={'pid':child.pid,'seed':seed,'started_at':now(),'command':command,'noise_concurrency_limit':cap()}
                write(HERE/f'seed{seed}_launch.json',launch);print(json.dumps(launch),flush=True)
            for seed,(process,stream,folder,started) in list(active.items()):
                code=process.poll()
                if code is None:
                    if time.monotonic()-started>2700:
                        raise TimeoutError(f'Seed{seed} exceeded 2700s outer deadline')
                    continue
                stream.close();del active[seed]
                result={'seed':seed,'returncode':code,'folder':str(folder),'finished_at':now()}
                if code!=0:
                    raise RuntimeError(f'Rollout seed{seed} failed with exit{code}')
                summary=read(folder/'rollout_summary.json');noise=read(folder/'noise_diagnostic.json')
                assert summary['ok'] and noise['noise_scale']==1 and noise['noise_seed']==seed and noise['environment_seed']==123
                assert Path(summary['checkpoint']).resolve()==CHECKPOINT.resolve()
                assert summary['steps']<=500
                result.update({key:summary.get(key) for key in ('steps','end_reason','episode_return','grf_penetration_max_m','phase_valid_cycle_count','invalid_event_count')})
                result['completed_full_episode']=summary['steps']==500 and summary['end_reason']=='episode_time_limit'
                results.append(result);print(json.dumps(result),flush=True)
            status('running')
            if pending or active:
                time.sleep(2)
        guard()
        write(HERE/'results.json',{'technical_pass':True,'checkpoint':str(CHECKPOINT),'cases':results,'inputs_unchanged':True})
        status('completed',inputs_unchanged=True)
    except BaseException as exc:
        for process,_,_,_ in active.values():
            if process.poll() is None:
                process.send_signal(signal.CTRL_BREAK_EVENT if os.name=='nt' else signal.SIGINT)
        for process,stream,_,_ in active.values():
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.kill();process.wait(timeout=10)
            stream.close()
        status('failed',error=repr(exc));raise
    finally:
        if sleeper is not None and sleeper.poll() is None:
            sleeper.terminate()


if __name__=='__main__':
    main()
