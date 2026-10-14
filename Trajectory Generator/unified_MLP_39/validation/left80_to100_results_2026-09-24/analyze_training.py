"""Summarize the authorized 80->100 continuation and its source branch."""
from pathlib import Path
from collections import Counter
import hashlib
import json

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
RUN = PACKAGE / 'runs/training/MLP39_LEFT80_penetration_weight2_to100_2026-09-24'
PLAN = PACKAGE / 'runs/plans/left80_penetration_weight2_to100_2026-09-24'

def read(path):
    return json.loads(path.read_text())

def history(path):
    return [json.loads(line) for line in path.read_text().splitlines()]

def window(rows, baseline, start, end):
    by_iteration = {row['iteration']: row for row in [baseline, *rows]}
    before, after = by_iteration[start - 1], by_iteration[end]
    counts = {key.split('/')[-1]: int(value - before['termination_metrics'].get(key, 0))
              for key, value in after['termination_metrics'].items() if '/episode_end/' in key}
    selected = [r for r in rows if start <= r['iteration'] <= end]
    return {'first': start, 'last': end, 'counts': counts, 'ended': sum(counts.values()),
            'completed_fraction': counts['episode_time_limit'] / sum(counts.values()),
            'transitions': int(after['num_env_steps_sampled_lifetime'] - before['num_env_steps_sampled_lifetime']),
            'mean_reported_return': sum(r['episode_return_mean'] for r in selected) / len(selected),
            'mean_entropy': sum(r['entropy'] for r in selected) / len(selected),
            'mean_kl': sum(r['mean_kl_loss'] for r in selected) / len(selected)}

def main():
    rows = history(RUN / 'train_iterations.jsonl')
    baseline = read(PLAN / 'baseline_iteration80.json')
    assert [r['iteration'] for r in rows] == list(range(81, 101))
    for r in rows:
        batch = r['unified_phase_audit']['target_availability_batch']
        assert batch['total_steps'] == 4096 and batch['visible_fraction'] == 1.0
        assert r['training_health']['observed_rows'] == 4096
        assert r['training_health']['missing_telemetry_rows'] == 0
    current = [window(rows, baseline, start, end) for start, end in
               [(81,100),(81,85),(86,90),(91,95),(96,100),(81,90),(91,100)]]
    previous_run = PACKAGE / 'runs/training/MLP39_LEFT72_penetration_weight2_10iter_2026-09-24'
    previous_rows = history(previous_run / 'train_iterations.jsonl')
    previous_baseline = read(PACKAGE / 'runs/plans/left72_penetration_weight2_10iter_2026-09-24/baseline_iteration72.json')
    previous = [window(previous_rows, previous_baseline, start, end) for start, end in [(73,82),(78,82)]]
    records = [json.loads(line) for f in sorted((RUN / 'episode_telemetry').glob('*.jsonl'))
               for line in f.read_text().splitlines()]
    reasons = Counter(r['end_reason'].replace(':','_') for r in records)
    assert dict(reasons) == {k:v for k,v in current[0]['counts'].items() if v}
    full = [r for r in records if r['end_reason'] == 'episode_time_limit']
    source_hashes = read(PLAN / 'source_checkpoint_hashes.json')
    repo = PACKAGE.parents[1]
    assert all(hashlib.sha256((repo/p).read_bytes()).hexdigest() == h for p,h in source_hashes.items())
    result = {'summary': {k:read(RUN/'summary.json')[k] for k in ['ok','finished_at','elapsed_wall_time_s','iterations_run']},
              'current_windows': current, 'previous_windows': previous,
              'best': read(RUN/'checkpoint_best_meta.json'), 'last': read(RUN/'checkpoint_last_meta.json'),
              'source_best80_return': baseline['episode_return_mean'],
              'best_new_return': max(r['episode_return_mean'] for r in rows),
              'ended_episodes': len(records), 'recorded_steps_in_ended_episodes': sum(r['observed_steps'] for r in records),
              'mixed_policy_episodes': sum(r['policy_versions']['switch_count'] > 0 for r in records),
              'completed_episodes_peak_over20mm': sum(r['peaks']['penetration_m']['value'] > .020 for r in full),
              'source_files_unchanged': len(source_hashes), 'all_batches_visible_and_complete': True,
              'last_entropy': rows[-1]['entropy'], 'source_entropy': baseline['entropy'],
              'limitations': ['Training episodes can span policy updates; these are not fixed-checkpoint robustness rates.',
                             'The new best82 is from the branch resumed at80, not the previous branch last82.',
                             'Best in the new output directory is selected among its own updates.']}
    (HERE / 'training_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__ == '__main__':
    main()
