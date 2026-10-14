"""Run a frozen three-arm PPO experiment and evaluate each best on three starts.

Use --plan <plan.json> --check-only to validate the plan without starting jobs.
The default execution is sequential. Each training has a fresh initialization;
only a successful full run can advance to its deterministic evaluations.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
ACTIVE_CHILD = None


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve(value):
    path = Path(value)
    return path if path.is_absolute() else REPO / path


def status(plan_dir, state, **details):
    payload = {'status': state, 'updated_at': datetime.now().astimezone().isoformat(),
               'pid': os.getpid(), **details}
    write(plan_dir / 'status.json', payload)
    print(json.dumps(payload), flush=True)


def validate_plan(plan):
    assert len(plan['variants']) == 3
    configs = []
    for variant in plan['variants']:
        config = yaml.safe_load(resolve(variant['config']).read_text())
        assert config['target_availability']['target_availability_probability'] == 1.0
        assert config['target_availability']['target_curriculum_probabilities'] == []
        assert config['simulation']['iterations'] == plan['iterations']
        assert config['ppo']['train_batch_size'] == 4608
        assert config['ppo']['num_epochs'] == 10
        assert config['parallelism']['num_env_runners'] == 12
        assert config['model']['seed'] == 123
        assert config['run']['resume_from'] is None and config['run']['init_actor_from'] is None
        assert config['simulation']['episode_start_offset_choices_s'] == plan['start_offsets_s']
        configs.append(config)
        for path, root in [(resolve(variant['run']), PACKAGE/'runs/training'),
                           (resolve(variant['rollout_root']), PACKAGE/'runs/rollout')]:
            path.resolve().relative_to(root.resolve())
    def material(config):
        config = json.loads(json.dumps(config))
        config.pop('provenance', None)
        return config
    base, velocity, slew = map(material, configs)
    assert base['target_availability']['target_velocity_scale'] == 10.0
    assert velocity['target_availability']['target_velocity_scale'] == 1.0
    velocity['target_availability']['target_velocity_scale'] = 10.0
    assert velocity == base, 'Velocity arm contains additional changes'
    for joint, rate in [('knee', 2.5), ('ankle', 2.0)]:
        key = f'pros_{joint}_target_slew_rate_limit_rad_s'
        assert slew['simulation'][key] == 0.0 and base['simulation'][key] == rate
        slew['simulation'][key] = rate
    assert slew == base, 'Slew arm contains additional changes'
    return configs


def source_guard(plan_dir):
    hashes = read(plan_dir/'source_hashes.json')
    changed = [p for p, expected in hashes.items() if not resolve(p).is_file() or digest(resolve(p)) != expected]
    if changed:
        raise RuntimeError(f'Frozen experiment inputs changed: {changed}')


def command_run(command, log_path, plan_dir, stage, variant, first_iteration_check=None):
    global ACTIVE_CHILD
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open('x', encoding='utf-8') as log:
        ACTIVE_CHILD = subprocess.Popen(command, cwd=REPO, stdin=subprocess.DEVNULL,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=(os.name != 'nt'),
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == 'nt' else 0)
        status(plan_dir, stage, variant=variant, child_pid=ACTIVE_CHILD.pid,
               command=command, log=str(log_path))
        while ACTIVE_CHILD.poll() is None:
            if first_iteration_check is not None and first_iteration_check():
                first_iteration_check = None
            time.sleep(5)
        returncode = ACTIVE_CHILD.returncode
    ACTIVE_CHILD = None
    if returncode != 0:
        raise RuntimeError(f'{stage} failed with return code {returncode}; see {log_path}')


def stop_child():
    global ACTIVE_CHILD
    if ACTIVE_CHILD is not None and ACTIVE_CHILD.poll() is None:
        # Ask the training/rollout supervisor to stop its own worker tree.
        ACTIVE_CHILD.send_signal(signal.CTRL_BREAK_EVENT if os.name == 'nt' else signal.SIGINT)
        try:
            ACTIVE_CHILD.wait(timeout=45)
        except subprocess.TimeoutExpired:
            if os.name == 'nt':
                subprocess.run(['taskkill', '/PID', str(ACTIVE_CHILD.pid), '/T', '/F'], check=False)
            else:
                os.killpg(ACTIVE_CHILD.pid, signal.SIGTERM)
    ACTIVE_CHILD = None


def training_audit(run, iterations):
    import pickle
    # Reuse the exact tensor-digest definitions written into the iteration log.
    if str(PACKAGE.parent) not in sys.path:
        sys.path.insert(0, str(PACKAGE.parent))
    from unified_MLP_39.actor_transition import actor_state_digest

    run = Path(run).resolve()
    summary = read(run/'summary.json')
    assert summary['ok'] and summary['stop_reason'] == 'completed'
    assert summary['initialization_mode'] in ('fresh', 'resume_from')
    assert not summary.get('skipped_iterations')
    rows = [json.loads(line) for line in (run/'train_iterations.jsonl').read_text().splitlines() if line.strip()]
    assert [row['iteration'] for row in rows] == list(range(1, iterations+1))
    for row in rows:
        balance = row['exact_start_balance']
        availability = row['unified_phase_audit']['target_availability_batch']
        assert balance['pass'] and balance['learner_batch_pass']
        assert availability['total_steps'] == availability['visible_steps'] == 4608
        assert row['training_health']['missing_telemetry_rows'] == 0
        assert all(math.isfinite(row[key]) for key in ['policy_loss','vf_loss','mean_kl_loss'])
    first = rows[0]['unified_phase_audit']
    initial_dir = run/'rl_module_initial'
    initial = read(initial_dir/'unified_training_state.json')
    assert initial['actor_source'] is None and initial['last_completed_iteration'] == 0
    assert initial['last_completed_phase'] is None and initial['reward_mode'] == 'imitation'
    with (initial_dir/'module_state.pkl').open('rb') as stream:
        initial_weights = pickle.load(stream)
    assert actor_state_digest(initial_weights) == first['actor_before'], 'Initial actor differs from the first update'
    # Initial RLModule exports are inference-only and omit critic tensors.
    # Its first-update digest is compared between arms by the orchestrator.
    assert isinstance(first['critic_before'], str) and len(first['critic_before']) == 64
    if summary['initialization_mode'] == 'resume_from':
        # Internal recovery always restores this run's checkpoint_last. A
        # different experiment's checkpoint or an intentional best transplant
        # would change the experiment and must not pass as crash recovery.
        for key in ('resume_from', 'supervisor_resume_from'):
            assert summary.get(key), f'Missing internal recovery metadata: {key}'
            source = Path(summary[key]).resolve()
            assert source.parent == run and source.name == 'checkpoint_last', 'Recovery used a foreign checkpoint'
            assert source.is_dir()
        resumed_at = int(summary['restored_logical_iteration'])
        assert 0 < resumed_at < iterations
        recovery = read(run/'resume_audit.json')
        assert Path(recovery['source_checkpoint']).resolve() == run/'checkpoint_last'
        assert recovery['actor_reinitialized'] is False
        first_restored = rows[resumed_at]['unified_phase_audit']
        assert recovery['restored_actor_digest'] == first_restored['actor_before']
        assert recovery['restored_critic_digest'] == first_restored['critic_before']
    else:
        assert not summary.get('resume_from') and not summary.get('supervisor_resume_from')
    best = read(run/'checkpoint_best_meta.json')
    checkpoint = run/f"milestone_iteration_{best['logical_iteration']:06d}"/'rl_module_last'
    assert checkpoint.is_dir()
    return {'best_iteration':best['logical_iteration'], 'checkpoint':str(checkpoint),
            'checkpoint_sha256':digest(checkpoint/'module_state.pkl'),
            'actor_initial_digest':first['actor_before'], 'critic_initial_digest':first['critic_before'],
            'initial_actor_weights_verified':True,
            'critic_initial_digest_source':'first_iteration_before_update_audit',
            'internal_recovery':summary['initialization_mode'] == 'resume_from',
            'iterations':len(rows), 'sampled_steps':iterations*4608,
            'best_train_return':summary['best_episode_return_mean']}


def rollout_audit(folder, scale):
    summary = read(folder/'rollout_summary.json')
    trace = read(folder/'rollout_policy_trace.json')
    assert summary['ok'] and summary['action_selection'] == 'deterministic'
    assert len(trace) == summary['steps'] and len(trace) > 0
    assert all(row['target_availability']['visible'] for row in trace)
    for index, row in enumerate(trace):
        obs = row['actor_observation_before']
        assert abs(obs['gait_phase_sin']**2+obs['gait_phase_cos']**2-1) < 2e-6
        if index:
            previous = trace[index-1]
            phase = previous['reward_terms']['pros_knee_angle_imitation_target_phase']
            assert abs(obs['gait_phase_sin']-math.sin(2*math.pi*phase)) < 2e-6
            assert abs(obs['gait_phase_cos']-math.cos(2*math.pi*phase)) < 2e-6
            for joint in ('knee', 'ankle'):
                key=f'pros_{joint}_angle'
                assert abs(obs[f'prescribed_left_{joint}_q_scaled']-previous['imitation_target_q'][key]) < 2e-6
                assert abs(obs[f'prescribed_left_{joint}_qdot_scaled']-previous['imitation_target_qdot'][key]/scale) < 2e-6
    result = {key:summary[key] for key in ['steps','episode_return','end_reason','terminated','truncated',
              'phase_valid_cycle_count','grf_penetration_max_m','episode_start_offset_s']}
    result['duration_s'] = trace[-1]['time']-trace[0]['policy_segment_times'][0]
    result['joints']={}
    for joint in ('pros_knee_angle','pros_ankle_angle'):
        target=[row['imitation_target_q'][joint] for row in trace]
        actual=[row['prosthetic_state'][joint] for row in trace]
        served=[row['prosthetic_state'][joint+'_served_ref'] for row in trace]
        def rmse(a,b):
            return math.degrees(math.sqrt(sum((x-y)**2 for x,y in zip(a,b))/len(a)))
        assert all(math.isfinite(x) for seq in (target,actual,served) for x in seq)
        swing=[i for i,row in enumerate(trace) if row['reward_terms'][joint+'_imitation_target_phase']>=.6]
        result['joints'][joint]={'rmse_actual_target_deg':rmse(actual,target),
            'rmse_served_target_deg':rmse(served,target),'rmse_actual_served_deg':rmse(actual,served),
            'rmse_phase60_100_deg':rmse([actual[i] for i in swing],[target[i] for i in swing]) if swing else None,
            'actual_excursion_deg':math.degrees(max(actual)-min(actual)),
            'target_excursion_deg':math.degrees(max(target)-min(target))}
    write(folder/'experiment_metrics.json',result)
    return result


def write_comparison(plan_dir, results):
    write(plan_dir/'results.json',results)
    lines=['# Ablazioni 39D con clock prescribed sinistro', '',
        'Best selezionato per return di training; valutazione deterministica separata sui tre start.',
        'Un seed di training: confronto diagnostico, non prova di equivalenza statistica.', '',
        '| Variante | Iter best | Start [s] | Durata [s] | RMSE ginocchio [°] | RMSE caviglia [°] | Fine |',
        '|---|---:|---:|---:|---:|---:|---|']
    for variant, result in results.items():
        for row in result.get('evaluations',[]):
            lines.append(f"| {variant} | {result['training']['best_iteration']} | {row['episode_start_offset_s']:.6f} | {row['duration_s']:.2f} | {row['joints']['pros_knee_angle']['rmse_actual_target_deg']:.2f} | {row['joints']['pros_ankle_angle']['rmse_actual_target_deg']:.2f} | {row['end_reason']} |")
    (plan_dir/'comparison.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args()
    plan_dir=args.plan.resolve().parent
    plan=read(args.plan); configs=validate_plan(plan);source_guard(plan_dir)
    if args.check_only:
        print('PASS: three one-factor configs, frozen sources, full training settings')
        return
    with (plan_dir/'execution.lock').open('x') as lock:
        lock.write(str(os.getpid()))
    results={}; initial=None; initial_verified=[None]
    try:
        for variant, config in zip(plan['variants'],configs):
            name=variant['name'];run=resolve(variant['run']);rollout_root=resolve(variant['rollout_root'])
            if run.exists() or rollout_root.exists():
                raise FileExistsError(f'Refusing existing experiment outputs: {run} / {rollout_root}')
            source_guard(plan_dir)
            def check_first_iteration():
                history = run/'train_iterations.jsonl'
                if not history.exists():
                    return False
                with history.open(encoding='utf-8') as stream:
                    line = stream.readline()
                try:
                    first = json.loads(line)
                except json.JSONDecodeError:
                    return False
                assert first['iteration'] == 1
                pair = (first['unified_phase_audit']['actor_before'], first['unified_phase_audit']['critic_before'])
                if initial_verified[0] is None:
                    initial_verified[0] = pair
                assert pair == initial_verified[0], 'Initial seeded weights differ; stopping this arm'
                assert first['exact_start_balance']['pass'] and first['exact_start_balance']['learner_batch_pass']
                availability = first['unified_phase_audit']['target_availability_batch']
                assert availability['visible_steps'] == availability['total_steps'] == 4608
                contract = read(run/'unified_contract.json')
                assert contract['schema_version'] == 'unified_mlp39_v3'
                assert contract['target_channels']['actor_indices'] == [0,1,35,36,37,38]
                assert contract['target_channels']['target_velocity_scale'] == config['target_availability']['target_velocity_scale']
                write(plan_dir/f'{name}_first_iteration_verified.json', {
                    'iteration':1, 'actor_initial_digest':pair[0], 'critic_initial_digest':pair[1],
                    'visible_steps':4608, 'six_channel_mask_contract':contract['target_channels']})
                return True
            command_run([sys.executable,'-u',str(PACKAGE/'train_ppo_mlp.py'),'--config',str(resolve(variant['config'])),
                '--output-dir',str(run),'--iterations',str(plan['iterations'])],plan_dir/f'{name}_training.log',plan_dir,'training',name,check_first_iteration)
            source_guard(plan_dir)
            audit=training_audit(run,plan['iterations'])
            pair=(audit['actor_initial_digest'],audit['critic_initial_digest'])
            if initial is None:initial=pair
            assert pair==initial,'Fresh seeded initial weights differ between arms'
            results[name]={'training':audit,'evaluations':[]}
            write_comparison(plan_dir,results)
            for start_name, offset in zip(['early','center','late'],plan['start_offsets_s']):
                source_guard(plan_dir)
                folder=rollout_root/start_name
                command_run([sys.executable,'-u',str(PACKAGE/'rollout_eval.py'),'--checkpoint',audit['checkpoint'],
                    '--output-dir',str(folder),'--episode-start-offset-s',str(offset),'--action-selection','deterministic',
                    '--record-outputs','--record-policy-trace','--max-steps','500','--startup-timeout-s','600',
                    '--step-timeout-s','180','--stall-timeout-s','300','--run-timeout-s','3600'],
                    plan_dir/f'{name}_{start_name}_rollout.log',plan_dir,'rollout',name)
                source_guard(plan_dir)
                assert digest(Path(audit['checkpoint'])/'module_state.pkl')==audit['checkpoint_sha256']
                result=rollout_audit(folder,config['target_availability']['target_velocity_scale'])
                results[name]['evaluations'].append(result);write_comparison(plan_dir,results)
                if start_name=='center':
                    command_run([sys.executable,str(PACKAGE/'validation/render_canonical_rollout.py'),'--rollout',str(folder)],
                        plan_dir/f'{name}_plots.log',plan_dir,'plotting',name)
        status(plan_dir,'completed',results=str(plan_dir/'results.json'),comparison=str(plan_dir/'comparison.md'))
    except KeyboardInterrupt:
        stop_child();status(plan_dir,'user_interrupt');raise SystemExit(130)
    except BaseException as exc:
        stop_child();status(plan_dir,'failed',error=repr(exc));raise


if __name__=='__main__':
    main()
