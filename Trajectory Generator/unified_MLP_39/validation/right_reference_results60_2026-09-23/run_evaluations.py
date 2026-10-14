"""Two deterministic evaluations of immutable RIGHT39 checkpoints58/60."""
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
REPO = PACKAGE.parents[1]
RUN = PACKAGE / 'runs/training/MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23'
PLAN = PACKAGE / 'runs/plans/right_reference_resume60_2026-09-23'


def read(path):
    return json.loads(path.read_text())


def write(path, data):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, indent=2) + '\n')
    temp.replace(path)


def now():
    return datetime.now().astimezone().isoformat()


def main():
    assert read(PLAN / 'status.json')['state'] == 'completed'
    assert read(RUN / 'checkpoint_best_meta.json')['logical_iteration'] == 58
    assert read(RUN / 'checkpoint_last_meta.json')['logical_iteration'] == 60
    with (HERE / 'execution.lock').open('x') as stream:
        stream.write(str(os.getpid()))
    files = {REPO / name for name in read(PLAN / 'source_hashes.json')}
    files.add(Path(__file__))
    for iteration in (58, 60):
        files.update(p for p in (RUN / f'milestone_iteration_{iteration:06d}/rl_module_last').rglob('*') if p.is_file())
    hashes = {str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    write(HERE / 'rollout_input_hashes.json', hashes)
    environment = dict(os.environ, PYTHONUNBUFFERED='1')
    for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        environment[key] = '1'
    active, launches, results = [], [], []
    sleeper = None
    if sys.platform == 'darwin' and Path('/usr/bin/caffeinate').exists():
        sleeper = subprocess.Popen(['/usr/bin/caffeinate', '-i', '-w', str(os.getpid())], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for label, iteration in (('best58', 58), ('last60', 60)):
            checkpoint = RUN / f'milestone_iteration_{iteration:06d}/rl_module_last'
            folder = PACKAGE / f'runs/rollout/MLP39_right_v26_60iter_2026-09-23_{label}'
            assert not folder.exists(), folder
            command = [sys.executable, str(PACKAGE / 'rollout_eval.py'), '--checkpoint', str(checkpoint), '--output-dir', str(folder), '--no-progress', '--run-timeout-s', '2400']
            log = (HERE / f'{label}_rollout.log').open('x')
            process = subprocess.Popen(command, cwd=REPO, env=environment, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == 'nt' else 0)
            record = {'label': label, 'iteration': iteration, 'pid': process.pid, 'started_at': now(), 'command': command, 'output': str(folder)}
            launches.append(record)
            active.append((process, log, record))
        write(HERE / 'rollout_launch.json', launches)
        while active:
            progress = {}
            for process, log, record in active[:]:
                heartbeat = Path(record['output']) / 'watchdog_state.json'
                if heartbeat.exists():
                    try:
                        progress[record['label']] = read(heartbeat)
                    except json.JSONDecodeError:
                        pass
                code = process.poll()
                if code is not None:
                    log.close()
                    results.append(dict(record, returncode=code, finished_at=now()))
                    active.remove((process, log, record))
            write(HERE / 'rollout_status.json', {'state': 'running' if active else 'finished', 'updated_at': now(), 'completed': len(results), 'active': progress})
            if active:
                time.sleep(5)
        write(HERE / 'rollout_execution.json', results)
        assert all(r['returncode'] == 0 and read(Path(r['output']) / 'rollout_summary.json')['ok'] for r in results)
        assert all(hashlib.sha256((REPO / name).read_bytes()).hexdigest() == digest for name, digest in hashes.items())
        write(HERE / 'rollout_status.json', {'state': 'completed', 'updated_at': now(), 'completed': len(results), 'inputs_unchanged': True, 'active': {}})
    finally:
        for process, log, _ in active:
            if process.poll() is None:
                process.terminate()
            log.close()
        if sleeper is not None and sleeper.poll() is None:
            sleeper.terminate()


if __name__ == '__main__':
    main()
