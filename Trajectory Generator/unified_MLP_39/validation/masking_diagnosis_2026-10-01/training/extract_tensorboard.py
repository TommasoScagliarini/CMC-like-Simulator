"""Extract saved TensorBoard scalars without importing the simulator."""
from pathlib import Path
import csv
import json
from statistics import mean
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

OUT = Path(__file__).resolve().parent
TRAIN = OUT.parents[2] / 'runs' / 'training'
BASE = TRAIN / 'MLP39_LEFT_fresh_best80config_100iter_2026-09-29'
PATHS = [BASE] + sorted((TRAIN / 'MLP39_LEFT_best90_masking_100iter_2026-09-30').glob('stage*'))

rows = []
summary = {}
for path in PATHS:
    events = EventAccumulator(str(path / 'tensorboard'), size_guidance={'scalars': 0}).Reload()
    iteration_by_step = {
        int(r['num_env_steps_sampled_lifetime']): r['iteration']
        for r in (json.loads(line) for line in (path/'train_iterations.jsonl').read_text().splitlines())
    }
    indexed = {}
    for tag in events.Tags()['scalars']:
        if not tag.startswith(('reward/', 'reward_diagnostic/', 'env_runners/console/reward_')):
            continue
        for item in events.Scalars(tag):
            iteration = iteration_by_step[item.step]
            if path == BASE and not 81 <= iteration <= 90:
                continue
            indexed.setdefault(iteration, dict(stage=path.name, iteration=iteration))[tag] = item.value
    selected = list(indexed.values())
    rows.extend(selected)
    summary[path.name] = {
        k: mean(r[k] for r in selected if k in r)
        for k in selected[0] if k not in ('stage', 'iteration')
    }
    summary[path.name]['exact_reward_per_step'] = (
        sum(r['env_runners/console/reward_sum_current'] for r in selected) /
        sum(r['env_runners/console/reward_count_current'] for r in selected)
    )

keys = list(dict.fromkeys(k for r in rows for k in r))
with (OUT/'tensorboard_rolling_metrics.csv').open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=keys)
    writer.writeheader()
    writer.writerows(rows)
(OUT/'tensorboard_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
print(f'Extracted {len(rows)} training iterations')
