"""Frozen, sequential continuation 40 -> 60 -> 80 -> 100 with separate evaluations.

Preparation and --freeze never run training or simulation. Root completes the
checkpoint40 diagnostics and writes preflight_ready.json before --freeze.
Execution requires the frozen manifest and never starts evaluation while Ray
workers remain alive. Any audit failure stops the sequence for inspection.
"""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
DEFAULT_PLAN = PACKAGE / 'runs/plans/completion_continuation_2026-09-22'
RUN = PACKAGE / 'runs/training/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21'
BASE_CONFIG = PACKAGE / 'runs/plans/v26_aligned_left_phase_fsmv3_resume25_to40_2026-09-21/config.yaml'
THREAD_ENV = {key: '1' for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
    'MKL_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS', 'NUMEXPR_NUM_THREADS')}
ACTIVE_CHILD = None
ACTIVE_EVALUATIONS = {}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temp.replace(path)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve(value):
    path = Path(value)
    return path if path.is_absolute() else REPO / path


def relative(path):
    return str(Path(path).resolve().relative_to(REPO))


def event(plan_dir, state, **details):
    data = {'status': state, 'updated_at': datetime.now().astimezone().isoformat(),
            'pid': os.getpid(), **details}
    write(plan_dir / 'status.json', data)
    with (plan_dir / 'events.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(data, allow_nan=False) + '\n')
    print(json.dumps(data), flush=True)


def config_material(config):
    result = deepcopy(config)
    result.pop('provenance', None)
    result['simulation'].pop('iterations')
    result['run'].pop('resume_from')
    return result


def history_rows(run):
    return [json.loads(line) for line in (run / 'train_iterations.jsonl').read_text().splitlines() if line.strip()]


def prepare(plan_dir):
    require(not (plan_dir / 'plan.json').exists(), 'Plan already exists; preparation refuses overwrite')
    summary = read(RUN / 'summary.json')
    require(summary['ok'] and summary['iterations_completed'] == 40, 'Expected completed iteration40')
    history = history_rows(RUN)
    require([r['iteration'] for r in history] == list(range(1, 41)), 'Expected contiguous history1..40')
    plan_dir.mkdir(parents=True, exist_ok=True)
    base = yaml.safe_load(BASE_CONFIG.read_text())
    stages = []
    for start, target in ((40, 60), (60, 80), (80, 100)):
        config = deepcopy(base)
        config['simulation']['iterations'] = target
        source = RUN / f'milestone_iteration_{start:06d}/checkpoint_last'
        config['run']['resume_from'] = str(source)
        config['provenance'].update({
            'experiment': f'completion_continuation_{start}_to_{target}',
            'created_on': '2026-09-22',
            'actor_initialization': f'Full checkpoint resume at iteration{start}; restore actor, critic and saved Adam state.',
            'continuation_scope': 'Same physical/training configuration, prescribed visible100%, FSMv3; episode telemetry only.',
            'resume_caveat': 'Historical iteration25 resume reset Adam; preserve the actual optimizer saved at40, do not reconstruct hypothetical moments.',
            'source_config': relative(BASE_CONFIG),
            'source_config_sha256': digest(BASE_CONFIG),
        })
        config['provenance']['execution_only_changes']['iterations']['temporary'] = target
        config['provenance']['execution_only_changes']['iterations']['reason'] = 'User approved sequential continuation through100 with separate evaluations at60/80/100.'
        path = plan_dir / f'config_{start}_to_{target}.yaml'
        path.write_text(yaml.safe_dump(config, sort_keys=False, allow_unicode=True), encoding='utf-8')
        stages.append({'start': start, 'target': target, 'config': relative(path),
                       'resume_from': relative(source),
                       'telemetry_dir': relative(plan_dir / f'telemetry_{start}_to_{target}'),
                       'rollout_root': relative(PACKAGE / f'runs/rollout/MLP39_completion_2026-09-22_iter{target:03d}')})
    archive = plan_dir / 'pre_resume_40'
    archive.mkdir()
    for name in ('summary.json', 'train_iterations.jsonl', 'training_cfg.resolved.yaml',
                 'resume_audit.json', 'unified_contract.json', 'checkpoint_last_meta.json'):
        source = RUN / name
        if source.is_file():
            shutil.copy2(source, archive / name)
    write(plan_dir / 'plan.json', {'schema': 'unified39_completion_continuation_v1',
        'run': relative(RUN), 'base_config': relative(BASE_CONFIG), 'start': 40, 'target': 100,
        'stages': stages, 'noise_scales': [1.0, 0.5], 'noise_seeds': [123,124,125,126,127],
        'episode_seed': 123, 'threads': THREAD_ENV, 'training_and_evaluation_concurrent': False, 'evaluation_max_concurrent': 4,
        'history_prefix_sha256': digest(archive / 'train_iterations.jsonl'),
        'history_prefix_bytes': (archive / 'train_iterations.jsonl').stat().st_size,
        'preflight_gate': relative(plan_dir / 'preflight_ready.json'),
        'canonical_plot_root': relative(REPO / 'plot'),
        'notes': ['Noise multiplier affects evaluation only.', 'Full resume preserves saved optimizer; process RNG/ongoing episodes do not resume bitwise.', 'Five evaluation seeds are diagnostic, not evidence of statistical equivalence.']})
    write(plan_dir / 'preparation.json', {'ok': True, 'training_started': False,
        'source_iteration': 40, 'source_checkpoint': relative(RUN / 'milestone_iteration_000040/checkpoint_last'),
        'created_at': datetime.now().astimezone().isoformat()})


def validate_plan(plan_dir):
    plan = read(plan_dir / 'plan.json')
    require(plan['schema'] == 'unified39_completion_continuation_v1', 'Wrong plan schema')
    initial = plan.get('start', 40)
    require(initial in (40,42), 'Unsupported initial checkpoint')
    require([(s['start'], s['target']) for s in plan['stages']] == [(initial,60),(60,80),(80,100)], 'Wrong stage sequence')
    if initial == 42:
        recovery = plan.get('recovery', {})
        require(recovery.get('reason') == 'numpy_bool_episode_telemetry_write_failure', 'Missing recovery provenance')
        require(digest(plan_dir/'pre_resume_42/summary.json') == recovery.get('summary_sha256'), 'Recovery summary changed')
        source = RUN/'milestone_iteration_000042/checkpoint_last'
        require(digest(source/'learner_group/learner/state.pkl') == recovery.get('learner_sha256'), 'Recovery checkpoint changed')
    require(plan['noise_scales'] == [1.0, .5] and plan['noise_seeds'] == [123,124,125,126,127], 'Noise protocol drift')
    require(plan['threads'] == THREAD_ENV, 'Thread environment drift')
    require(plan['evaluation_max_concurrent'] == 4, 'Evaluation concurrency drift')
    base = yaml.safe_load(resolve(plan['base_config']).read_text())
    require(base['ppo']['train_batch_size'] == 4096 and base['parallelism']['num_env_runners'] == 13, 'Batch/workers drift')
    require(base['grf']['binary_phase_actor_fsm_version'] == 'v3', 'FSM drift')
    require(base['target_availability']['target_availability_probability'] == 1.0, 'Prescribed availability drift')
    require(base['target_availability']['target_velocity_scale'] == 1.0, 'Velocity scale drift')
    require(base['simulation']['episode_start_offset_s'] == 1.0 and not base['simulation']['episode_start_offset_choices_s'], 'Start drift')
    run = resolve(plan['run'])
    require(run.resolve() == RUN.resolve(), 'Unexpected training output path')
    for stage in plan['stages']:
        config = yaml.safe_load(resolve(stage['config']).read_text())
        require(config_material(config) == config_material(base), 'Material configuration changed')
        require(config['simulation']['iterations'] == stage['target'], 'Iteration mismatch')
        source = run / f"milestone_iteration_{stage['start']:06d}/checkpoint_last"
        require(Path(config['run']['resume_from']).resolve() == source.resolve() == resolve(stage['resume_from']).resolve(), 'Resume path mismatch')
        resolve(stage['rollout_root']).resolve().relative_to((PACKAGE / 'runs/rollout').resolve())
        resolve(stage['telemetry_dir']).resolve().relative_to(plan_dir.resolve())
    prefix = plan_dir / 'pre_resume_40/train_iterations.jsonl'
    require(digest(prefix) == plan['history_prefix_sha256'], 'Archived history changed')
    return plan


def history_guard(plan_dir, plan):
    prefix = (plan_dir / 'pre_resume_40/train_iterations.jsonl').read_bytes()
    actual = (resolve(plan['run']) / 'train_iterations.jsonl').read_bytes()
    require(actual.startswith(prefix), 'History1..40 was changed or truncated')


def freeze(plan_dir):
    plan = validate_plan(plan_dir)
    require(not (plan_dir / 'source_hashes.json').exists(), 'Already frozen; refusing to bless changed sources')
    gate = read(resolve(plan['preflight_gate']))
    for key in ('approved_to_start', 'diagnostics_completed', 'telemetry_preflight_passed', 'optimizer_preflight_passed'):
        require(gate.get(key) is True, f'Preflight gate failed: {key}')
    history_guard(plan_dir, plan)
    paths = set(REPO.glob('*.py')) | set(PACKAGE.parent.glob('*.py'))
    for folder in (PACKAGE, PACKAGE.parent / 'baseline_MLP'):
        paths.update(folder.glob('*.py'))
    paths.update((PACKAGE / 'validation' / name) for name in (
        'run_completion_continuation.py', 'completion_noise_rollout.py', 'render_canonical_rollout.py',
        'analyze_completion_diagnosis.py', 'analyze_episode_telemetry.py'))
    paths.update((PACKAGE/'tests').glob('*.py'))
    paths.add(plan_dir/'test_orchestration.py')
    paths.update(resolve(stage['config']) for stage in plan['stages'])
    paths.update((resolve(plan['base_config']), plan_dir / 'plan.json', resolve(plan['preflight_gate'])))
    # Freeze every checkpoint40 input, not merely actor weights. Later stage
    # checkpoint manifests are recorded immediately after their generation.
    paths.update(p for p in (RUN / 'milestone_iteration_000040').rglob('*') if p.is_file())
    paths.update(p for p in (RUN / f"milestone_iteration_{plan['start']:06d}").rglob('*') if p.is_file())
    for folder in ('models/AB06_SEASEA_Threadmill', 'online_grf_profiles'):
        paths.update(p for p in (REPO / folder).rglob('*') if p.is_file() and p.suffix.lower() in ('.xml','.osim','.sto','.mot','.json','.dylib','.so','.dll'))
    profile = yaml.safe_load(BASE_CONFIG.read_text())['grf']['binary_phase_detector_profile']
    paths.add(resolve(profile))
    paths = {p.resolve() for p in paths}
    missing = [str(p) for p in paths if not p.is_file()]
    require(not missing, f'Missing inputs: {missing}')
    write(plan_dir / 'source_hashes.json', {relative(p): digest(p) for p in sorted(paths)})
    write(plan_dir / 'freeze_receipt.json', {'ok': True, 'files': len(paths),
        'manifest_sha256': digest(plan_dir / 'source_hashes.json'),
        'frozen_at': datetime.now().astimezone().isoformat()})


def source_guard(plan_dir):
    receipt = read(plan_dir / 'freeze_receipt.json')
    require(digest(plan_dir / 'source_hashes.json') == receipt['manifest_sha256'], 'Frozen manifest changed')
    changed = [p for p, sha in read(plan_dir / 'source_hashes.json').items()
               if not resolve(p).is_file() or digest(resolve(p)) != sha]
    require(not changed, f'Frozen inputs changed: {changed}')


def active_ray_processes(include_rollouts=False):
    """List Ray workers/raylets, ignoring orphan idle GCS metadata servers.

    Avoid an optional psutil dependency. Inspection failure is fatal: the
    pipeline cannot promise disjoint training/evaluation without this check.
    """
    found = []
    if os.name == 'nt':
        command = ['powershell', '-NoProfile', '-Command',
            'Get-CimInstance Win32_Process | Select-Object ProcessId,Name,CommandLine | ConvertTo-Json -Compress']
        result = subprocess.run(command, check=True, capture_output=True, text=True, timeout=15)
        rows = json.loads(result.stdout)
        if isinstance(rows, dict):
            rows = [rows]
        for row in rows:
            name = (row.get('Name') or '').lower()
            args = row.get('CommandLine') or ''
            rollout = include_rollouts and name.startswith('python') and any(script in args for script in ('rollout_eval.py', 'completion_noise_rollout.py'))
            if name == 'raylet.exe' or name.startswith('ray::') or 'default_worker.py' in args or rollout:
                found.append({'pid': row['ProcessId'], 'name': name})
    else:
        result = subprocess.run(['ps','-axo','pid=,stat=,comm=,args='],
            check=True,capture_output=True,text=True,timeout=15)
        for line in result.stdout.splitlines():
            fields = line.strip().split(None,3)
            if len(fields)<3 or fields[1].startswith('Z'):
                continue
            pid,state,name=fields[:3]
            args=fields[3] if len(fields)>3 else ''
            name=Path(name).name.lower()
            rollout = include_rollouts and name.startswith('python') and any(script in args for script in ('rollout_eval.py', 'completion_noise_rollout.py'))
            if name=='raylet' or name.startswith('ray::') or '/ray/_private/workers/default_worker.py' in args or rollout:
                found.append({'pid':int(pid),'name':name})
    return found


def wait_for_ray_exit(plan_dir, stage, timeout=90, include_rollouts=False):
    deadline = time.monotonic() + timeout
    while True:
        processes = active_ray_processes(include_rollouts=include_rollouts)
        if not processes:
            event(plan_dir, 'resources_released', stage=stage, active_ray_processes=[])
            return
        if time.monotonic() >= deadline:
            raise RuntimeError(f'Ray processes remain alive; refusing concurrent rollout/training: {processes}')
        time.sleep(2)


def stop_child():
    global ACTIVE_CHILD
    children = ([ACTIVE_CHILD] if ACTIVE_CHILD is not None else []) + [item['process'] for item in ACTIVE_EVALUATIONS.values()]
    for child in children:
        if child.poll() is None:
            child.send_signal(signal.CTRL_BREAK_EVENT if os.name == 'nt' else signal.SIGINT)
    deadline=time.monotonic()+45
    for child in children:
        if child.poll() is None:
            try:
                child.wait(timeout=max(.1,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                if os.name == 'nt':
                    subprocess.run(['taskkill','/PID',str(child.pid),'/T','/F'], check=False)
                else:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGKILL)
                        child.wait(timeout=10)
    for item in ACTIVE_EVALUATIONS.values():
        item['stream'].close()
    ACTIVE_EVALUATIONS.clear()
    ACTIVE_CHILD = None


def run_command(command, log_path, plan_dir, label, env, poll_check=None, timeout=None):
    global ACTIVE_CHILD
    require(ACTIVE_CHILD is None, 'Another pipeline command is active')
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with log_path.open('x', encoding='utf-8') as log:
        ACTIVE_CHILD = subprocess.Popen(command, cwd=REPO, env=env, stdin=subprocess.DEVNULL,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=(os.name != 'nt'),
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == 'nt' else 0)
        event(plan_dir, label, child_pid=ACTIVE_CHILD.pid, log=str(log_path), command=command)
        while ACTIVE_CHILD.poll() is None:
            if poll_check is not None:
                poll_check()
            require(timeout is None or time.monotonic() - started < timeout, f'{label} exceeded total wall timeout')
            time.sleep(5)
        code = ACTIVE_CHILD.returncode
    ACTIVE_CHILD = None
    require(code == 0, f'{label} failed with exit{code}; see {log_path}')
    if poll_check is not None:
        poll_check()
    return time.monotonic() - started


def evaluation_command(module, folder, scale, seed, environment_seed):
    script=PACKAGE/'validation/completion_noise_rollout.py'
    command=[sys.executable,'-u',str(script),'--noise-scale',str(scale),'--noise-seed',str(seed)]
    return command+['--checkpoint',str(module),'--output-dir',str(folder),'--seed',str(environment_seed),
        '--record-outputs' if scale==0 else '--no-record-outputs','--record-policy-trace','--no-progress',
        '--max-steps','500','--startup-timeout-s','600','--step-timeout-s','180',
        '--stall-timeout-s','300','--run-timeout-s','3600']


def run_parallel_evaluations(jobs, plan_dir, target, env):
    """Four single-thread simulators only after the training process tree exits."""
    require(ACTIVE_CHILD is None and not ACTIVE_EVALUATIONS, 'Another pipeline phase is active')
    require(not active_ray_processes(), 'Refusing evaluations while Ray workers exist')
    pending=list(jobs); completed=0
    while pending or ACTIVE_EVALUATIONS:
        while pending and len(ACTIVE_EVALUATIONS)<4:
            name,command,log_path=pending.pop(0)
            source_guard(plan_dir)
            stream=log_path.open('x',encoding='utf-8')
            try:
                child=subprocess.Popen(command,cwd=REPO,env=env,stdin=subprocess.DEVNULL,
                    stdout=stream,stderr=subprocess.STDOUT,start_new_session=(os.name!='nt'),
                    creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name=='nt' else 0)
            except BaseException:
                stream.close();raise
            ACTIVE_EVALUATIONS[name]={'process':child,'stream':stream,'started':time.monotonic(),'log':str(log_path)}
            event(plan_dir,'parallel_evaluation_started',iteration=target,case=name,
                child_pid=child.pid,active_evaluations=len(ACTIVE_EVALUATIONS),max_concurrent=4,log=str(log_path))
        for name,item in list(ACTIVE_EVALUATIONS.items()):
            code=item['process'].poll()
            if code is None:
                require(time.monotonic()-item['started']<3900,f'Evaluation{name} exceeded wall timeout')
                continue
            item['stream'].close();del ACTIVE_EVALUATIONS[name]
            require(code==0,f'Evaluation{name} failed with exit{code}; see {item["log"]}')
            completed+=1
            event(plan_dir,'parallel_evaluation_completed',iteration=target,case=name,
                completed_cases=completed,total_cases=len(jobs),active_evaluations=len(ACTIVE_EVALUATIONS))
        if ACTIVE_EVALUATIONS:
            time.sleep(5)


def check_iteration(row, previous):
    iteration = row['iteration']
    require(int(row['num_env_steps_sampled_lifetime']) - int(previous['num_env_steps_sampled_lifetime']) == 4096, f'Wrong sample increment at{iteration}')
    mask = row['unified_phase_audit']['target_availability_batch']
    require(mask['visible_steps'] == mask['total_steps'] == 4096, f'Mask/batch mismatch at{iteration}')
    require(row['training_health']['missing_telemetry_rows'] == 0, f'Missing environment info at{iteration}')
    require(row['unified_phase'] == 'imitation', f'Reward mode changed at{iteration}')
    for key in ('policy_loss','vf_loss','mean_kl_loss','entropy'):
        require(math.isfinite(row[key]), f'Nonfinite{key} at{iteration}')


def verify_resume(run, stage, first, source_last):
    audit = read(run / 'resume_audit.json')
    require(Path(audit['source_checkpoint']).resolve() == resolve(stage['resume_from']).resolve(), 'Restored a different checkpoint')
    require(audit['actor_reinitialized'] is False, 'Actor was reinitialized')
    for kind in ('actor','mean','critic'):
        require(audit[f'restored_{kind}_digest'] == source_last['unified_phase_audit'][f'{kind}_after'], f'{kind} differs from source')
        require(audit[f'restored_{kind}_digest'] == first['unified_phase_audit'][f'{kind}_before'], f'{kind} changed before update')
    optimizer = read(run / 'resume_optimizer_audit.json')
    require(optimizer['passed'] is True, 'Optimizer restore failed')
    require(Path(optimizer['source_checkpoint']).resolve() == resolve(stage['resume_from']).resolve(), 'Optimizer source differs')
    require(bool(optimizer['learners']), 'Optimizer audit missing learners')
    for learner in optimizer['learners']:
        require(learner['passed'] and learner['parameters_unchanged'], 'Optimizer rebind changed parameters')
        require(bool(learner['optimizers']), 'No restored optimizer')
        for item in learner['optimizers']:
            require(item['state_exact'] and item['source_digest'] == item['restored_digest'], 'Adam state was not restored exactly')
            require(item['state_count'] > 0 and item['state_count'] == item['linked_state_count'], 'Adam state is not linked to live parameters')
    ppo=optimizer['ppo_training_state']
    require(ppo['passed'] is True and bool(ppo['learners']), 'PPO adaptive-state restore failed')
    expected_source='verified_historical_iteration40_hash_and_history' if stage['start']==40 else 'checkpoint_sidecar'
    require(ppo['source']==expected_source,'Unexpected adaptive-state provenance')
    if stage['start']>40:
        require(ppo['sidecar'] and Path(ppo['sidecar']).is_file(),'Missing PPO state sidecar')
        require(Path(ppo['sidecar']).resolve().parent==resolve(stage['resume_from']).resolve(),'PPO sidecar belongs to another checkpoint')
    for learner in ppo['learners']:
        require(learner['passed'] and learner['constant_schedules_verified'],'PPO schedule restore failed')
        require(bool(learner['kl_coefficient']),'No restored KL coefficient')
        for coefficient in learner['kl_coefficient'].values():
            require(coefficient['passed'] and coefficient['source']==coefficient['restored'],'KL coefficient changed on resume')
            if stage['start']==40:
                require(coefficient['source']==source_last['current_kl_coeff'],'Historical40 KL differs from recorded value')
    return {'weights': audit, 'optimizer': optimizer}


def optimizer_progress_audit(run, stage, resume_audit):
    """Read saved first/final updates only after Ray has stopped.

    The resume audit verifies live Parameter links. Serialized IDs at the first
    update must still match their groups and advance beyond the restored steps;
    this catches a silent moment reset that final-only counters could conceal.
    """
    import pickle
    source_path=resolve(stage['resume_from'])/'learner_group/learner/state.pkl'
    source=pickle.loads(source_path.read_bytes())['optimizer']
    expected={item['name']:item for learner in resume_audit['optimizer']['learners']
              for item in learner['optimizers']}
    require(set(source)==set(expected),'Optimizer progress source names differ from resume audit')
    reports=[]
    preceding=source
    for iteration in (stage['start']+1,stage['target']):
        path=run/f'milestone_iteration_{iteration:06d}/checkpoint_last/learner_group/learner/state.pkl'
        require(path.is_file(),f'Missing optimizer checkpoint at iteration{iteration}')
        current=pickle.loads(path.read_bytes())['optimizer']
        require(set(current)==set(source),'Optimizer topology changed during training')
        optimizers=[]
        for name,payload in current.items():
            state=payload['state'];old=preceding[name]['state'];original=source[name]['state']
            ids=[parameter_id for group in state['param_groups'] for parameter_id in group['params']]
            require(len(ids)==len(set(ids))==len(state['state'])==expected[name]['state_count']==12,
                    f'Unexpected optimizer state topology at{iteration}')
            require(set(state['state'])==set(ids)==set(original['state']),
                    f'Optimizer moment state became orphaned at{iteration}')
            require(expected[name]['linked_state_count']==12,'Resume did not verify twelve live Parameter links')
            for parameter_id,value in state['state'].items():
                step=float(value['step'])
                require(math.isfinite(step) and step>float(old['state'][parameter_id]['step']),
                        f'Adam step reset or stalled at iteration{iteration}')
            optimizers.append({'name':name,'state_count':len(state['state']),
                'serialized_state_ids_match_param_groups':True,
                'live_parameter_link_verified_on_resume':True,
                'source_steps':sorted({float(value['step']) for value in original['state'].values()}),
                'steps':sorted({float(value['step']) for value in state['state'].values()}),
                'steps_advanced':True})
        reports.append({'iteration':iteration,'checkpoint_state_sha256':digest(path),'optimizers':optimizers})
        preceding=current
    return {'passed':True,'source_iteration':stage['start'],'checkpoints':reports,
            'execution':'offline after training workers exited'}


def completion_window(rows, start, end):
    previous = rows[start-2]['termination_metrics'] if start > 1 else {}
    current = rows[end-1]['termination_metrics']
    counts = {key.rsplit('/',1)[-1]: int(value - previous.get(key,0))
              for key,value in current.items() if '/episode_end/' in key}
    total = sum(counts.values())
    return {'iterations': [start,end], 'episodes': total, 'outcomes': counts,
            'completed': counts.get('episode_time_limit',0),
            'completion_fraction': counts.get('episode_time_limit',0)/total if total else None}


def telemetry_audit(folder):
    counts = Counter(); ids = set(); workers = set(); incomplete = 0; missing = 0
    groups = {}
    files = sorted(folder.glob('episodes_*.jsonl'))
    require(bool(files), f'No episode telemetry files in{folder}')
    for path in files:
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                record = json.loads(line)
                require(record['schema'] == 'unified39_episode_telemetry_v1', 'Wrong telemetry schema')
                require(record['episode_uid'] not in ids, 'Duplicate episode telemetry')
                ids.add(record['episode_uid']); workers.add(record['worker_index'])
                require(len(record['tail']) <= 50, 'Unbounded telemetry tail')
                reason = record['end_reason']; counts[reason] += 1
                incomplete += int(record['partial_episode']); missing += int(record['missing_steps'])
                group=groups.setdefault(reason, {'episodes':0,'duration_sum_s':0.,'duration_count':0,
                    'episodes_with_resync':0,'resync_within_last_500ms':0,'hs_cancel_within_last_500ms':0,
                    'episodes_with_policy_switch':0,'policy_switch_count':0,'policy_version_missing_rows':0,
                    'terminal_fsm_states':Counter(),'peak_penetration_sum_m':0.,'peak_penetration_count':0,
                    'action_clip_rows':0,'observed_steps':0,'rows_with_missing_fields':0,
                    'policy_distribution_rows':0,'joint_error_accumulators':{}})
                group['episodes']+=1
                versions=record.get('policy_versions',{})
                switches=int(versions.get('switch_count',0))
                group['episodes_with_policy_switch']+=int(switches>0)
                group['policy_switch_count']+=switches
                group['policy_version_missing_rows']+=int(versions.get('rows_without_version',record['observed_steps']))
                if record['duration_s'] is not None:
                    group['duration_sum_s']+=record['duration_s'];group['duration_count']+=1
                group['episodes_with_resync']+=int(record['event_counts'].get('resync_rows',0)>0)
                for key,out in (('last_resync','resync_within_last_500ms'),('last_hs_cancel','hs_cancel_within_last_500ms')):
                    recent=record.get(key)
                    if recent and record['episode_steps']-recent['episode_step'] <= 50:
                        group[out]+=1
                if record['tail']:
                    group['terminal_fsm_states'][str(record['tail'][-1]['phase_fsm'].get('state_name'))]+=1
                peak=record['peaks'].get('penetration_m',{}).get('value')
                if peak is not None:
                    group['peak_penetration_sum_m']+=peak;group['peak_penetration_count']+=1
                group['action_clip_rows']+=record['event_counts'].get('action_clip_rows',0)
                for key in ('observed_steps','rows_with_missing_fields','policy_distribution_rows'):
                    group[key]+=record[key]
                for joint,values in record['joint_error_accumulators'].items():
                    acc=group['joint_error_accumulators'].setdefault(joint,{'count':0,'sum_squared_rad2':0.})
                    acc['count']+=values['count'];acc['sum_squared_rad2']+=values['sum_squared_rad2']
    require(not incomplete and not missing, 'Episode telemetry missing steps or episode start')
    require(len(workers) == 13, f'Expected telemetry from13workers, got{len(workers)}')
    for group in groups.values():
        group['terminal_fsm_states']=dict(group['terminal_fsm_states'])
        group['duration_mean_s']=group['duration_sum_s']/group['duration_count'] if group['duration_count'] else None
        group['peak_penetration_mean_m']=group['peak_penetration_sum_m']/group['peak_penetration_count'] if group['peak_penetration_count'] else None
        group['action_clip_row_fraction']=group['action_clip_rows']/group['observed_steps'] if group['observed_steps'] else None
        for joint,acc in group['joint_error_accumulators'].items():
            acc['rmse_deg']=math.degrees(math.sqrt(acc['sum_squared_rad2']/acc['count'])) if acc['count'] else None
    return {'files':len(files),'episodes':len(ids),'workers':sorted(workers),
            'outcomes':dict(counts),'partial_episodes':incomplete,'missing_steps':missing,
            'by_outcome':groups,
            'note':'Ongoing episodes at segment shutdown have no terminal record; counts can differ from metrics windows. Resync correlations are observational and do not establish causation.'}


def rollout_metrics(folder):
    summary = read(folder / 'rollout_summary.json')
    require(summary['ok'] is True, f'Rollout failed:{folder}')
    trace = read(folder / 'rollout_policy_trace.json')
    require(len(trace) == summary['steps'] > 0, 'Missing rollout trace')
    require(summary['end_reason'] != 'max_steps', 'Rollout stopped before environment termination')
    require(all(row['target_availability']['visible'] for row in trace), 'Prescribed was hidden in evaluation')
    result = {key:summary.get(key) for key in ('steps','episode_return','end_reason','terminated','truncated',
        'grf_penetration_max_m','phase_valid_cycle_count','action_clipped_fraction','action_selection')}
    result['completed'] = summary['end_reason'] == 'episode_time_limit' and summary['steps'] == 500
    result['folder'] = str(folder)
    result['joints'] = {}
    for joint in ('pros_knee_angle','pros_ankle_angle'):
        actual=[row['prosthetic_state'][joint] for row in trace]
        target=[row['imitation_target_q'][joint] for row in trace]
        result['joints'][joint] = {'rmse_deg': math.degrees(math.sqrt(sum((a-b)**2 for a,b in zip(actual,target))/len(actual)))}
    write(folder / 'completion_metrics.json', result)
    return result


def write_report(plan_dir, results):
    write(plan_dir / 'results.json', results)
    lines=['# Proseguimento39D: episodi completi e rumore della policy', '',
        'Stesso checkpoint ripreso con stato Adam salvato, configurazione invariata, target/clock sempre visibili.',
        'Training e valutazioni sono sequenziali. Cinque seed di rollout misurano sensibilità al rumore, non equivalenza statistica.', '',
        '| Iter | Completi ultime5 | Deterministico | σ×1 completi | σ×0.5 completi |',
        '|---:|---:|---:|---:|---:|']
    for key,value in results.items():
        window=value['training']['last5']; evaluations=value.get('evaluations',[])
        deterministic=next((r for r in evaluations if r['noise_scale']==0),None)
        groups=[[r for r in evaluations if r['noise_scale']==scale] for scale in (1.0,.5)]
        rates=[f"{sum(r['completed'] for r in group)}/{len(group)}" if group else 'in attesa' for group in groups]
        det=(f"{deterministic['steps']}/500" if deterministic else 'in attesa')
        lines.append(f"|{key}|{window['completed']}/{window['episodes']}|{det}|{rates[0]}|{rates[1]}|")
    lines.append('')
    for key,value in results.items():
        if value.get('episode_analysis'):
            link=Path(value['episode_analysis'])/'summary.md'
            lines.append(f'Analisi degli episodi a iterazione {key}: [episodi completi e falliti, eventi FSM e cambi di policy](<{link}>).')
    lines += ['', 'Le distribuzioni stocastiche usano seed appaiati e scala della deviazione standard applicata soltanto in valutazione.',
        'La ripresa ripristina lo stato salvato; non ricrea stream casuali o episodi OpenSim già in corso.',
        'Dettagli per episodio, errori cinematici, penetrazione e audit sono in results.json e nelle cartelle telemetry.']
    (plan_dir/'comparison.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


def execute(plan_dir):
    plan = validate_plan(plan_dir); source_guard(plan_dir); history_guard(plan_dir, plan)
    require(not (plan_dir/'execution.lock').exists(), 'Execution lock exists; inspect previous status before manual recovery')
    with (plan_dir/'execution.lock').open('x') as stream:
        stream.write(str(os.getpid())+'\n')
    run=resolve(plan['run']); results={}
    base_env={**os.environ, **THREAD_ENV, 'PYTHONUNBUFFERED':'1'}
    base_env.pop('UNIFIED39_EPISODE_TELEMETRY_DIR',None)
    try:
        for stage in plan['stages']:
            start,target=stage['start'],stage['target']
            source_guard(plan_dir); history_guard(plan_dir,plan)
            rows=history_rows(run)
            require([r['iteration'] for r in rows] == list(range(1,start+1)), 'Training is not at expected stage start')
            summary=read(run/'summary.json')
            if start == 42 and plan.get('recovery'):
                require(summary['stop_reason']=='user_interrupt' and summary['iterations_completed']==42,
                        'Unexpected recovery boundary')
                require(digest(run/'summary.json')==plan['recovery']['summary_sha256'], 'Recovery source summary changed')
                require((run/'train_iterations.jsonl').read_bytes()==(plan_dir/'pre_resume_42/train_iterations.jsonl').read_bytes(),
                        'Recovery history changed')
            else:
                require(summary['ok'] and summary['stop_reason']=='completed', 'Previous training did not complete cleanly')
            checkpoint=resolve(stage['resume_from'])
            require(checkpoint.is_dir(), f'Missing full checkpoint{checkpoint}')
            if start>40:
                manifest=read(plan_dir/f'checkpoint_{start}_hashes.json')
                require(all(resolve(p).is_file() and digest(resolve(p))==sha for p,sha in manifest.items()),
                    f'Checkpoint{start} changed after evaluation')
            telemetry=resolve(stage['telemetry_dir'])
            require(not telemetry.exists(), 'Refusing existing telemetry output')
            telemetry.mkdir(parents=True)
            wait_for_ray_exit(plan_dir,f'before_training_{start}_{target}',include_rollouts=True)
            previous_history=(run/'train_iterations.jsonl').read_bytes()
            archived=plan_dir/f'pre_resume_{start}'
            archived.mkdir(exist_ok=True)
            for name in ('summary.json','train_iterations.jsonl','training_cfg.resolved.yaml','resume_audit.json','resume_optimizer_audit.json'):
                if (run/name).exists() and not (archived/name).exists():
                    shutil.copy2(run/name,archived/name)
            checked=[start]; resume_checked=[False]
            def poll():
                raw=(run/'train_iterations.jsonl').read_bytes()
                require(raw.startswith(previous_history), 'Earlier training history changed')
                try:
                    current=[json.loads(line) for line in raw.decode().splitlines() if line.strip()]
                except json.JSONDecodeError:
                    return # A final line may be in flight.
                for index in range(checked[0],len(current)):
                    row=current[index]
                    require(row['iteration']==index+1, 'Non-contiguous history during training')
                    check_iteration(row,current[index-1])
                    if not resume_checked[0]:
                        audit=verify_resume(run,stage,row,rows[-1])
                        write(plan_dir/f'resume_{start}_verified.json',audit)
                        resume_checked[0]=True
                    if row['iteration']>=start+2:
                        writers=set()
                        for telemetry_file in telemetry.glob('episodes_*.jsonl'):
                            with telemetry_file.open(encoding='utf-8') as stream:
                                first_record=json.loads(stream.readline())
                            require(first_record['schema']=='unified39_episode_telemetry_v1','Invalid live telemetry schema')
                            writers.add(first_record['worker_index'])
                        require(len(writers)==13,f'Episode telemetry missing workers after two iterations: {sorted(writers)}')
                    checked[0]=row['iteration']
                    event(plan_dir,'training_iteration_verified',start=start,target=target,
                        iteration=row['iteration'],sampled_steps=row['num_env_steps_sampled_lifetime'],
                        episode_return_mean=row['episode_return_mean'],
                        last5=completion_window(current,max(start+1,row['iteration']-4),row['iteration']),
                        telemetry_files=len(list(telemetry.glob('episodes_*.jsonl'))),
                        child_pid=ACTIVE_CHILD.pid if ACTIVE_CHILD else None)
            command=[sys.executable,'-u',str(PACKAGE/'train_ppo_mlp.py'),'--config',str(resolve(stage['config'])),
                '--output-dir',str(run),'--iterations',str(target),'--resume-from',str(checkpoint)]
            elapsed=run_command(command,plan_dir/f'training_{start}_to_{target}.log',plan_dir,
                f'training_{start}_to_{target}',{**base_env,'UNIFIED39_EPISODE_TELEMETRY_DIR':str(telemetry)},poll,timeout=30*3600)
            wait_for_ray_exit(plan_dir,f'after_training_{target}')
            source_guard(plan_dir); history_guard(plan_dir,plan)
            summary=read(run/'summary.json');current=history_rows(run)
            require(summary['ok'] and summary['stop_reason']=='completed', 'Training ended unsuccessfully')
            require(summary['iterations_completed']==target and len(current)==target and checked[0]==target, 'Training target not reached')
            require(not summary.get('skipped_iterations'), 'Training skipped iterations')
            require(not summary.get('crash_restart_count',0), 'Training restarted unexpectedly; inspect before advancing')
            require(resume_checked[0], 'No first-iteration resume verification')
            telemetry_result=telemetry_audit(telemetry)
            progress=optimizer_progress_audit(run,stage,read(plan_dir/f'resume_{start}_verified.json'))
            write(plan_dir/f'optimizer_progress_{start}_to_{target}.json',progress)
            episode_analysis=plan_dir/f'telemetry_analysis_iter{target:03d}'
            run_command([sys.executable,str(PACKAGE/'validation/analyze_episode_telemetry.py'),
                '--telemetry-dir',str(telemetry),'--output',str(episode_analysis)],
                plan_dir/f'iter{target}_episode_analysis.log',plan_dir,f'episode_analysis_{target}',base_env,timeout=600)
            for name in ('summary.json','summary.md','per_episode.jsonl'):
                require((episode_analysis/name).is_file(),f'Missing episode analysis artifact:{name}')
            full_milestone=run/f'milestone_iteration_{target:06d}'
            require((full_milestone/'checkpoint_last').is_dir(), 'Missing resumable milestone')
            milestone_hashes={relative(p):digest(p) for p in full_milestone.rglob('*') if p.is_file()}
            write(plan_dir/f'checkpoint_{target}_hashes.json',milestone_hashes)
            results[str(target)]={'training':{'elapsed_wall_s':elapsed,'sampled_steps':target*4096,
                'last5':completion_window(current,target-4,target),'segment':completion_window(current,start+1,target),
                'telemetry':telemetry_result,'optimizer_progress':progress},'evaluations':[],
                'episode_analysis':str(episode_analysis)}
            write_report(plan_dir,results)
            module=full_milestone/'rl_module_last'; rollroot=resolve(stage['rollout_root'])
            require(not rollroot.exists(), 'Refusing existing rollout outputs')
            jobs=[(0.0,plan['episode_seed'])]+[(scale,seed) for seed in plan['noise_seeds'] for scale in plan['noise_scales']]
            noise_dispatched=False
            for scale,seed in jobs:
                source_guard(plan_dir)
                wait_for_ray_exit(plan_dir,f'before_eval_{target}_{scale}_{seed}')
                name='deterministic' if scale==0 else f'noise_{scale:g}_seed{seed}'
                folder=rollroot/name
                command=evaluation_command(module,folder,scale,seed,plan['episode_seed'])
                if scale==0:
                    run_command(command,plan_dir/f'iter{target}_{name}.log',plan_dir,
                        f'evaluation_{target}_{name}',base_env,timeout=3900)
                elif not noise_dispatched:
                    tasks=[]
                    for job_scale,job_seed in jobs[1:]:
                        job_name=f'noise_{job_scale:g}_seed{job_seed}'
                        tasks.append((job_name,evaluation_command(module,rollroot/job_name,job_scale,job_seed,plan['episode_seed']),
                            plan_dir/f'iter{target}_{job_name}.log'))
                    run_parallel_evaluations(tasks,plan_dir,target,base_env)
                    noise_dispatched=True
                metrics=rollout_metrics(folder);metrics.update(noise_scale=scale,noise_seed=seed)
                if scale:
                    provenance=read(folder/'noise_diagnostic.json')
                    require(provenance['noise_scale']==scale and provenance['noise_seed']==seed and provenance['environment_seed']==plan['episode_seed'], 'Noise/environment protocol changed')
                    metrics['noise_provenance']=provenance
                    if scale==.5:
                        paired=read(rollroot/f'noise_1_seed{seed}'/'paired_noise_draws.json')
                        draws=read(folder/'paired_noise_draws.json')
                        require(all(a['standard_normal']==b['standard_normal'] for a,b in zip(paired,draws)), 'Noise draws were not paired')
                        metrics['paired_draws_verified_steps']=min(len(paired),len(draws))
                results[str(target)]['evaluations'].append(metrics);write_report(plan_dir,results)
                if scale==0:
                    run_command([sys.executable,str(PACKAGE/'validation/render_canonical_rollout.py'),'--rollout',str(folder)],
                        plan_dir/f'iter{target}_canonical_plots.log',plan_dir,f'plotting_{target}',base_env,timeout=600)
                    receipt=read(folder/'plots/plot_receipt.json')
                    require(len(receipt['figures'])==7,'Missing canonical plots')
                    for figure in receipt['figures']:
                        Path(figure).resolve().relative_to((REPO/'plot').resolve())
                        require(Path(figure).is_file(),'Missing canonical plot file')
                    results[str(target)]['canonical_plots']=receipt['figures'];write_report(plan_dir,results)
            analysis_folder=plan_dir/f'analysis_iter{target:03d}'
            run_command([sys.executable,str(PACKAGE/'validation/analyze_completion_diagnosis.py'),
                '--rollout-root',str(rollroot),'--output',str(analysis_folder)],
                plan_dir/f'iter{target}_noise_analysis.log',plan_dir,f'analysis_{target}',base_env,timeout=600)
            results[str(target)]['noise_analysis']=str(analysis_folder);write_report(plan_dir,results)
            changed=[path for path,sha in milestone_hashes.items() if digest(resolve(path))!=sha]
            require(not changed,f'Checkpoint altered by evaluations:{changed}')
        source_guard(plan_dir);history_guard(plan_dir,plan)
        event(plan_dir,'completed',last_iteration=100,results=str(plan_dir/'results.json'),comparison=str(plan_dir/'comparison.md'))
    except KeyboardInterrupt:
        stop_child();event(plan_dir,'interrupted');raise SystemExit(130)
    except BaseException as exc:
        stop_child();event(plan_dir,'failed',error=repr(exc));raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan-dir',type=Path,default=DEFAULT_PLAN)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--prepare',action='store_true')
    mode.add_argument('--freeze',action='store_true')
    mode.add_argument('--check-only',action='store_true')
    args=parser.parse_args();folder=args.plan_dir.resolve()
    if args.prepare:
        prepare(folder);validate_plan(folder)
        print('Prepared only; no training or rollout was started.');return
    if args.freeze:
        freeze(folder);print('Preflight gate accepted and sources frozen.');return
    validate_plan(folder);source_guard(folder)
    if args.check_only:
        history_guard(folder,read(folder/'plan.json'))
        print('PASS: aligned configs, frozen sources and preserved history. No processes started.');return
    execute(folder)


if __name__=='__main__':
    main()
