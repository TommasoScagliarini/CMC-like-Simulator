"""Offline ON/OFF callback benchmark with real trace values and MetricsLogger."""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
sys.path.insert(0, str(PACKAGE.parent))
import numpy as np
from ray.rllib.env.single_agent_episode import SingleAgentEpisode
from ray.rllib.utils.metrics.metrics_logger import MetricsLogger
from unified_MLP_39.episode_telemetry import ENV_VAR
from unified_MLP_39.reward_function import RewardConfig, compute_reward
from unified_MLP_39.tb_logging import RewardComponentsCallback

SOURCE = PACKAGE / 'runs/rollout/completion_diagnostics_2026-09-22_iter40/noise_1_seed_123'
trace = json.loads((SOURCE/'rollout_policy_trace.json').read_text())
noise = json.loads((SOURCE/'paired_noise_draws.json').read_text())
summary = json.loads((SOURCE/'rollout_summary.json').read_text())
config = RewardConfig.from_mapping(summary['reward_config'])
prepared = []
for i, row in enumerate(trace):
    terms = row['reward_terms']
    reward, components = compute_reward(terms, config, reference=row['policy_segment_values'])
    phase = row['phase_fsm']
    left = {
        'normal_force_n': terms['grf_ankle_moment_flip_force_n'],
        'normal_force_bw': terms['prosthetic_normal_force_bw'],
        'in_contact': bool(terms['grf_ankle_moment_flip_force_n'] > 0),
        'heel_strike': bool(phase.get('valid_hs_count',0)),
        'toe_off': bool(phase.get('valid_to_count',0)),
        'cycle_duration_s': phase.get('last_period_s',0), 'gait_phase': 0.0,
    }
    info = {
        'time':row['time'], 'target_time':row['policy_segment_times'][-1],
        'episode_start_offset_s':1., 'observation':row['prosthetic_state'],
        'reward_terms':terms, 'reward_components':components, 'phase_fsm':phase,
        'policy_segment_values':np.asarray(row['policy_segment_values']),
        'reference_governor_diagnostics':row['reference_governor_diagnostics'],
        'imitation_target':{'q':row['imitation_target_q'],'qdot':row['imitation_target_qdot'],
                            'phase':{j:terms[j+'_imitation_target_phase'] for j in ('pros_knee_angle','pros_ankle_angle')}},
        'target_availability':row['target_availability'],
        'online_gait':{'sides':{'left':left, 'right':left.copy()}},
        'end_reason':summary['end_reason'] if i == len(trace)-1 else None,
    }
    actor = trace[min(i+1,len(trace)-1)]['actor_observation_vector_before']
    observation = np.concatenate((np.asarray(actor, dtype=np.float32), np.zeros(49,dtype=np.float32)))
    logits = noise[i].get('action_dist_inputs')
    if logits is None:
        logits = [*row['policy_action_mean'], *[math.log(x) for x in noise[i]['learned_std']]]
    outputs = {'action_dist_inputs':logits, 'weights_seq_no':40}
    prepared.append((observation,np.asarray(row['raw_policy_action'],dtype=np.float32),reward,info,outputs))

def trial(enabled, number, directory):
    with patch.dict(os.environ, {ENV_VAR: str(directory) if enabled else ''}):
        callback = RewardComponentsCallback()
        logger = MetricsLogger()
        runner = SimpleNamespace(worker_index=1, _weights_seq_no=40)
        episode = SingleAgentEpisode(id_=f'real-metrics-{enabled}-{number}', len_lookback_buffer=0)
        initial = np.concatenate((np.asarray(trace[0]['actor_observation_vector_before'],dtype=np.float32),np.zeros(49,dtype=np.float32)))
        episode.add_env_reset(initial,infos={'time':12.99})
        callback_cpu = callback_wall = reduce_cpu = reduce_wall = 0.
        for i, (observation, action, reward, info, outputs) in enumerate(prepared):
            terminal = i == len(prepared)-1
            episode.add_env_step(observation, action, reward, infos=info,
                terminated=terminal and summary['terminated'], truncated=terminal and summary['truncated'],
                extra_model_outputs=outputs)
            cpu, wall = time.process_time(), time.perf_counter()
            callback.on_episode_step(episode=episode, metrics_logger=logger, env_runner=runner, env_index=0)
            if terminal:
                callback.on_episode_end(episode=episode, metrics_logger=logger, env_runner=runner,
                    env_index=0, prev_episode_chunks=[])
            callback_cpu += time.process_time()-cpu
            callback_wall += time.perf_counter()-wall
        cpu, wall = time.process_time(), time.perf_counter()
        logger.reduce()
        reduce_cpu += time.process_time()-cpu
        reduce_wall += time.perf_counter()-wall
        observer = callback._episode_observer
        return {'enabled':enabled, 'steps':len(prepared), 'callback_cpu_s':callback_cpu,
            'callback_wall_s':callback_wall, 'metrics_reduce_cpu_s':reduce_cpu,
            'metrics_reduce_wall_s':reduce_wall,
            'observer_metrics':observer.metrics if observer else None}

with tempfile.TemporaryDirectory() as directory:
    # Alternate order to reduce monotone warm-up bias; no OpenSim or policy forward.
    trials = [trial(enabled,i,Path(directory)) for i,enabled in enumerate((False,True,True,False))]
    means = {str(flag):statistics.mean(t['callback_cpu_s']+t['metrics_reduce_cpu_s'] for t in trials if t['enabled']==flag)
             for flag in (False,True)}
    increment = means['True']-means['False']
    result = {'passed':all(t['observer_metrics']['observed_steps']==len(prepared)
               and t['observer_metrics']['episodes_written']==1
               and t['observer_metrics'].get('missing_rows',0)==0
               and t['observer_metrics'].get('write_errors',0)==0 for t in trials if t['enabled']),
        'source_trace':str(SOURCE/'rollout_policy_trace.json'),
        'source_trace_sha256':hashlib.sha256((SOURCE/'rollout_policy_trace.json').read_bytes()).hexdigest(),
        'observer_source_sha256':hashlib.sha256((PACKAGE/'episode_telemetry.py').read_bytes()).hexdigest(),
        'uses_real_rllib_metrics_logger':True, 'reward_term_count':len(prepared[0][3]['reward_terms']),
        'reward_component_count':len(prepared[0][3]['reward_components']),
        'trials':trials, 'mean_total_callback_and_reduce_cpu_s':means,
        'incremental_cpu_seconds_per_500_steps':increment,
        'incremental_cpu_microseconds_per_step':increment*1e6/len(prepared),
        'limits':[
            'Benchmark uses real recorded reward terms/FSM/actions and recomputes scalar components outside timing; no simulation or policy forward.',
            'Critic-only observation padding and online gait fields are synthetic API payload reconstruction; this is a callback cost benchmark, not another physics validation.',
            'Includes real MetricsLogger.log_value and one reduce; TensorBoard disk writes remain iteration-scoped and are outside this benchmark.',
            'Measured CPU delta is local and noisy. It cannot guarantee zero slowdown under different Ray/process/disk loads.',
            'Latest observer includes whole-episode policy-version aggregation added after the physical A/B observer process was imported.',
        ]}
(HERE/'real_metrics_benchmark.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
raise SystemExit(0 if result['passed'] else 1)
