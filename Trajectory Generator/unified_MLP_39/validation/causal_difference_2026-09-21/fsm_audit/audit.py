"""Read-only numerical reward and actor-FSM replay audit for the V26 comparison."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import types

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / '.git').exists())
TG = REPO / 'Trajectory Generator'
sys.path[:0] = [str(TG), str(REPO / 'validation')]
from unified_MLP_39 import reward_function as unified
from baseline_MLP import reward_function as baseline
import replay_actor_fsm_v3_on_traces as replay

historical_source = subprocess.check_output(
    ['git', 'show', '68a62cd8:Trajectory Generator/baseline_MLP/reward_function.py'],
    cwd=REPO, text=True,
)
historical = types.ModuleType('audit_historical_reward')
historical.__file__ = str(TG / 'baseline_MLP/reward_function.py')
sys.modules[historical.__name__] = historical
exec(compile(historical_source, historical.__file__, 'exec'), historical.__dict__)

current_run = TG / 'unified_MLP_39/runs/training/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21'
old_run = TG / 'runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter'
old_summary = json.loads((old_run / 'summary.json').read_text())
cur_summary = json.loads((current_run / 'summary.json').read_text())
old_config = old_summary['reward_config']
cur_config = cur_summary['reward_config']
cfg_historical = historical.RewardConfig.from_mapping(old_config)
cfg_baseline = baseline.RewardConfig.from_mapping(old_config)
cfg_unified = unified.RewardConfig.from_mapping(cur_config)

rollouts = {
    '39D_25': TG / 'unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25',
    '39D_29': TG / 'unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_best29',
    '39D_40': TG / 'unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40',
    'V26_87': TG / 'runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout',
}

result = {
    'historical_reward_source_commit': '68a62cd8',
    'reward_common_config_differences': {
        key: [value, cur_config.get(key)] for key, value in old_config.items()
        if value != cur_config.get(key)
    },
    'rollouts': {},
}
for name, directory in rollouts.items():
    trace = json.loads((directory / 'rollout_policy_trace.json').read_text())
    rewards = []
    for row in trace:
        terms = row['reward_terms']
        reference = row['policy_segment_values']
        rewards.append([
            historical.compute_reward(terms, cfg_historical, reference=reference)[0],
            baseline.compute_reward(terms, cfg_baseline, reference=reference)[0],
            unified.compute_reward(terms, cfg_unified, reference=reference)[0],
        ])
    values = np.asarray(rewards)
    summary = json.loads((directory / 'rollout_summary.json').read_text())
    overrides = replay.fsm_overrides(summary)
    labels, data = replay.load_sto(directory / 'sim_outputs/rollout_episode_online_grf.sto')
    fsms = [replay.make_fsm(**overrides, resync_enabled=flag, hs_cancel_enabled=flag)
            for flag in (False, True)]
    rec0 = trace[0]['phase_fsm']
    primed = bool(float(rec0.get('sensor_partial_stance_active', 0)) > 0) or (
        rec0.get('state_name') == 'STANCE_AFTER_HS' and float(rec0.get('valid_hs_count', 0)) == 0)
    reset_time = round(float(trace[0]['time']) - .01, 6)
    for fsm in fsms:
        fsm.reset_from_binary_baseline(time_s=reset_time, in_contact=primed)
    max_obs_error = 0.0
    state_mismatch = 0
    for boundary, events, bcontact, icontact, fbw, knee, ankle, recorded in replay.step_inputs(
            trace, labels, data, initial_latch=primed):
        for fsm in fsms:
            fsm.update_from_binary_events(
                time_s=boundary, previous_time_s=round(boundary - .01, 6), events=events,
                normal_force_bw=fbw, in_contact=icontact, prosthetic_knee_angle_rad=knee,
                prosthetic_ankle_angle_rad=ankle, binary_contact=bcontact,
            )
        observations = [fsm.observation() for fsm in fsms]
        max_obs_error = max(max_obs_error, max(abs(observations[0][k] - observations[1][k]) for k in observations[0]))
        state_mismatch += int(fsms[0].state_id != fsms[1].state_id)
    result['rollouts'][name] = {
        'steps': len(trace),
        'max_abs_reward_historical_vs_unified': float(np.max(np.abs(values[:, 0] - values[:, 2]))),
        'max_abs_reward_historical_vs_current_baseline': float(np.max(np.abs(values[:, 0] - values[:, 1]))),
        'fsm_v2_v3_actor_observation_max_abs_delta': max_obs_error,
        'fsm_v2_v3_state_mismatch_steps': state_mismatch,
        'recorded_resync_count': trace[-1]['phase_fsm'].get('resync_count', 0),
        'recorded_hs_cancelled_count': trace[-1]['phase_fsm'].get('hs_cancelled_count', 0),
    }

rows = [json.loads(line) for line in (current_run / 'train_iterations.jsonl').read_text().splitlines()]
result['training_health'] = {
    'resync_event_rows': sum(r['training_health']['resync_event_rows'] for r in rows),
    'iterations_with_resync': [r['iteration'] for r in rows if r['training_health']['resync_event_rows'] > 0],
    'termination_counts_final': rows[-1]['termination_metrics'],
    'causality_limit': 'No per-step stochastic training traces: repair-to-penetration order and paired v2 effect cannot be reconstructed from aggregated counters.',
}
(HERE / 'audit.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
print(json.dumps(result, indent=2, allow_nan=False))
