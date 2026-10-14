"""Read-only exact comparison of the authorized physical observer OFF/ON runs."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
OFF = PACKAGE / 'runs/rollout/completion_diagnostics_2026-09-22_iter40/noise_1_seed_123'
ON = PACKAGE / 'runs/rollout/completion_telemetry_ab_2026-09-22'
FILES = ('rollout_summary.json', 'rollout_policy_trace.json', 'noise_diagnostic.json', 'paired_noise_draws.json')


def read(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify():
    summaries = [read(path / 'rollout_summary.json') for path in (OFF, ON)]
    traces = [read(path / 'rollout_policy_trace.json') for path in (OFF, ON)]
    noise = [read(path / 'paired_noise_draws.json') for path in (OFF, ON)]
    provenance = [read(path / 'noise_diagnostic.json') for path in (OFF, ON)]
    common_noise_keys = sorted(set(noise[0][0]) & set(noise[1][0]))
    common_noise_equal = len(noise[0]) == len(noise[1]) and all(
        {k:a[k] for k in common_noise_keys} == {k:b[k] for k in common_noise_keys}
        for a,b in zip(*noise))
    logits_delta = max(abs(a-b) for row,draw in zip(traces[1],noise[1])
        for a,b in zip(draw['action_dist_inputs'],
            [*row['policy_action_mean'], *[math.log(x) for x in draw['learned_std']]]))
    summary_diff = {key: [summaries[0].get(key), summaries[1].get(key)]
                    for key in set(summaries[0]) | set(summaries[1])
                    if summaries[0].get(key) != summaries[1].get(key)}
    trace_mismatch = [i+1 for i, (a,b) in enumerate(zip(*traces)) if a != b]
    trace_equal = len(traces[0]) == len(traces[1]) and not trace_mismatch
    t = provenance[1]['telemetry_preflight']
    telemetry_paths = sorted(Path(t['directory']).glob('episodes_*.jsonl'))
    records = [json.loads(line) for path in telemetry_paths for line in path.read_text().splitlines()]
    record = records[0] if len(records) == 1 else {}
    steps = summaries[1]['steps']
    coverage = {
        'one_terminal_record': len(records) == 1,
        'every_transition_observed': record.get('observed_steps') == record.get('episode_steps') == steps,
        'no_missing_steps': record.get('missing_steps') == 0,
        'no_missing_fields': record.get('rows_with_missing_fields') == 0,
        'initial_episode_present': record.get('partial_episode') is False and record.get('episode_start_time_s') is not None,
        'every_distribution_observed': record.get('policy_distribution_rows') == steps,
        'outcome_matches': record.get('end_reason') == summaries[1]['end_reason'],
        'terminal_flags_match': (record.get('terminated'), record.get('truncated')) == (summaries[1]['terminated'], summaries[1]['truncated']),
        'tail_bounded_and_at_end': [row['episode_step'] for row in record.get('tail', [])] == list(range(max(1,steps-49),steps+1)),
    }
    tail_differences = []
    for row in record.get('tail', []):
        trace = traces[1][row['episode_step'] - 1]
        for name, a, b in (
            ('raw_policy_action', row['raw_policy_action'], trace['raw_policy_action']),
            ('actor_observation_before', row['actor_observation_before'], trace['actor_observation_vector_before']),
            ('time', row['time_s'], trace['time']),
            ('policy_segment_values', row['policy_segment_values'], trace['policy_segment_values']),
            ('policy_logits', row['policy_outputs'].get('action_dist_inputs'), noise[1][row['episode_step']-1]['action_dist_inputs']),
        ):
            if a != b:
                tail_differences.append({'step':row['episode_step'], 'field':name})
        for field, value in row['phase_fsm'].items():
            if value != trace['phase_fsm'].get(field):
                tail_differences.append({'step':row['episode_step'], 'field':'fsm.'+field})
        for field, value in row['reward_terms'].items():
            if value != trace['reward_terms'].get(field):
                tail_differences.append({'step':row['episode_step'], 'field':'reward_terms.'+field})
    coverage['tail_matches_full_rollout_trace'] = not tail_differences
    coverage['penetration_peak_matches_full_episode'] = record.get('peaks',{}).get('penetration_m',{}).get('value') == summaries[1]['grf_penetration_max_m']
    scalar_sum_delta = abs(record.get('observed_reward_sum', math.inf) - summaries[1]['episode_return'])
    coverage['return_agrees_with_numpy_sum'] = scalar_sum_delta < 1e-10
    bad_metrics = {key:value for key,value in t['metrics'].items()
                   if key not in ('observed_steps','policy_distribution_rows','episodes_written') and value != 0}
    coverage['observer_reports_no_errors'] = not bad_metrics
    fraction = t['observer_wall_seconds'] / provenance[1]['wall_seconds']
    result = {
        'passed': bool(trace_equal and not summary_diff and common_noise_equal and logits_delta < 2e-7 and all(coverage.values())),
        'baseline': str(OFF), 'observer_on': str(ON),
        'steps': steps,
        'full_trace_values_exactly_equal': trace_equal,
        'full_trace_files_sha256_equal': sha(OFF/FILES[1]) == sha(ON/FILES[1]),
        'full_trace_first_mismatch_steps': trace_mismatch[:10],
        'full_summary_exactly_equal': not summary_diff, 'summary_differences': summary_diff,
        'common_noise_keys': common_noise_keys,
        'all_common_noise_values_exactly_equal': common_noise_equal,
        'on_only_noise_keys': sorted(set(noise[1][0]) - set(noise[0][0])),
        'on_logits_reconstructed_from_recorded_mean_std_max_abs_delta': logits_delta,
        'logits_reconstruction_tolerance': 2e-7,
        'noise_files_sha256_equal': sha(OFF/FILES[3]) == sha(ON/FILES[3]),
        'sink_coverage_checks': coverage, 'sink_tail_first_differences': tail_differences[:10],
        'sink_observed_reward_sum_vs_numpy_sum_abs_delta': scalar_sum_delta,
        'sink_metrics': t['metrics'],
        'physical_outcome': {key:summaries[1][key] for key in ('steps','end_reason','episode_return','grf_penetration_max_m','action_clipped_fraction')},
        'cost': {
            'observer_wall_seconds':t['observer_wall_seconds'],
            'observer_mean_microseconds':t['observer_mean_microseconds'],
            'observer_max_microseconds':t['observer_max_microseconds'],
            'observer_fraction_of_on_run_wall':fraction,
            'observer_percent_of_on_run_wall':100*fraction,
            'measured_observer_under_1_percent': fraction < .01,
            'off_run_wall_seconds':provenance[0]['wall_seconds'],
            'on_run_wall_seconds':provenance[1]['wall_seconds'],
            'telemetry_bytes':sum(path.stat().st_size for path in telemetry_paths),
            'limits': [
                'Observer timer includes observe/finish and terminal JSON writing; excludes construction/add_env_step of the diagnostic SingleAgentEpisode wrapper.',
                'The diagnostic wrapper retains a complete episode for the API adapter; real training already retains its episodes and observer itself holds only 50 tail rows.',
                'OFF/ON runs may overlap other diagnostics; wall-duration subtraction is not an isolated total overhead estimate.',
                'This tests one physical episode and the sink API; actual Ray callback wiring/worker propagation still requires first-training-batch checks.',
                'Baseline noise records predate action_dist_inputs logging. Compare the three common fields exactly; ON logits are also reconstructed from trace mean and learned std, allowing inversion of float32 exp.',
                'Whole-episode policy_versions aggregation was added after the ON process imported its observer; its no-side-effect check is covered by the subsequent offline tests and real-MetricsLogger benchmark.',
            ],
        },
        'input_sha256': {str(path / name):sha(path/name) for path in (OFF,ON) for name in FILES},
    }
    (HERE / 'physical_ab_verification.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2))
    return result['passed']


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--wait', action='store_true')
    args = parser.parse_args()
    deadline = time.monotonic() + 3600
    while not all((path/name).is_file() for path in (OFF,ON) for name in FILES):
        if not args.wait or time.monotonic() > deadline:
            raise SystemExit('Physical A/B artifacts not ready; no verification emitted')
        time.sleep(5)
    raise SystemExit(0 if verify() else 1)
