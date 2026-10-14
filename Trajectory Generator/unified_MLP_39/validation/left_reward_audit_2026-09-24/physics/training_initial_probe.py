"""Read-only training scalar audit: initial LEFT and RIGHT iterations 1-25."""
import hashlib
import json
import math
import os
from pathlib import Path
from statistics import mean

for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ[key] = '1'
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
import yaml

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
REPO = PACKAGE.parents[1]
RUNS = {'left': 'MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21',
        'right': 'MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23'}
KEYS = ['imitation_score', 'served_imitation_score', 'penalty', 'reward_base', 'command_rate_loss',
        'reward', 'safety_term', 'grf_penetration_term', 'grf_ankle_moment_flip_term', 'oob_term']
HASHES = {}


def hashed(path):
    data = path.read_bytes()
    HASHES[str(path.relative_to(REPO))] = hashlib.sha256(data).hexdigest()
    return data


def main():
    result = {'method': {
        'scope': 'Saved TensorBoard scalars and native training JSONL, iterations1-25, before LEFT optimizer restore defect.',
        'sample_alignment': 'Each update collects4096 transitions in both arms; compare at identical cumulative steps.',
        'scalar_aggregation': 'RewardComponentsCallback logs each finite component from each sampled step with reduce=mean, window=100; RLlib merges runner metrics and TensorBoard records float32 scalars. These are smoothed runner aggregates, not exact means over4096 transitions.',
        'clip_lift_identity': 'mean(base_clipped) - [0.2 mean(actual_score) +0.8 mean(served_score) -mean(base_penalty)]. Linear recombination preserves a shared averaging operator; it does NOT apply clip to the mean raw base.',
        'interpretation': 'With nonnegative scores/penalties, blends summing to1 and clip bounds[0,1], upper clipping cannot activate. Positive clip lift measures weighted aggregate upward correction from negative raw bases. It does not reveal number, duration or locations of clipped steps.',
        'cross_check': 'Mean reward equals mean base minus the sum of active post-clip terms within float32 precision. This supports aligned averaging of relevant component streams; raw early transition traces are unavailable.',
        'window_summary': 'Five-iteration table entries are arithmetic means of the five logged smoothed values, not pooled step means or episode-return estimates.'},
        'runs': {}}
    for arm, name in RUNS.items():
        run = PACKAGE / 'runs/training' / name
        history = [json.loads(line) for line in hashed(run / 'train_iterations.jsonl').splitlines()]
        cfg = yaml.safe_load(hashed(run / ('milestone_iteration_000025/checkpoint_last/training_cfg.resolved.yaml'
                                          if arm == 'left' else 'checkpoint_last/training_cfg.resolved.yaml')))['reward']
        assert cfg['reward_mode'] == 'imitation' and cfg['blend_imitation_tracking'] == 0
        assert cfg['blend_imitation'] == .2 and cfg['blend_served_imitation'] == .8
        assert cfg['clip_low'] == 0 and cfg['clip_high'] == 1
        folder = run / 'tensorboard'
        for path in folder.glob('events.out.tfevents.*'):
            hashed(path)
        events = EventAccumulator(str(folder), size_guidance={'scalars': 0})
        events.Reload()
        scalars = {key: {int(e.step): float(e.value) for e in events.Scalars('reward/' + key)} for key in KEYS}
        rows = []
        for row in history[:25]:
            i, steps = row['iteration'], row['num_env_steps_sampled_lifetime']
            assert steps == i * 4096
            values = {key: scalars[key][steps] for key in KEYS}
            assert all(math.isfinite(v) for v in values.values())
            mixed = .2 * values['imitation_score'] + .8 * values['served_imitation_score']
            lift = values['reward_base'] - mixed + values['penalty']
            post = sum(values[key] for key in ('safety_term', 'grf_penetration_term', 'grf_ankle_moment_flip_term', 'oob_term'))
            residual = values['reward'] - (values['reward_base'] - post)
            assert lift >= -1e-7 and abs(residual) < 1e-6
            rows.append({'iteration': i, 'cumulative_steps': steps, 'scalars': values,
                'mixed_imitation': mixed, 'base_penalty': values['penalty'],
                'command_rate_penalty': cfg['command_rate_weight'] * values['command_rate_loss'],
                'clip_lift': lift, 'base_clipped': values['reward_base'], 'reward': values['reward'],
                'postclip_penalties': post, 'identity_residual': residual})
        windows = []
        for lo in (1, 6, 11, 16, 21):
            hi = lo + 4
            select = rows[lo - 1:hi]
            means = {key: mean(r[key] for r in select) for key in (
                'mixed_imitation', 'base_penalty', 'command_rate_penalty', 'clip_lift',
                'base_clipped', 'reward', 'postclip_penalties')}
            means['command_fraction_of_base_penalty'] = means['command_rate_penalty'] / means['base_penalty']
            after = history[hi - 1]['termination_metrics']
            before = history[lo - 2]['termination_metrics'] if lo > 1 else {}
            counts = {k.rsplit('/', 1)[-1]: int(after.get(k, 0) - before.get(k, 0))
                      for k in set(after) | set(before) if '/episode_end/' in k}
            windows.append({'iterations': [lo, hi], 'endings': counts,
                            'complete': counts.get('episode_time_limit', 0), 'episodes': sum(counts.values()),
                            'smoothed_scalar_arithmetic_means': means})
        result['runs'][arm] = {'rows': rows, 'windows': windows,
            'max_identity_residual': max(abs(r['identity_residual']) for r in rows),
            'clip_lift_peak': max(rows, key=lambda r: r['clip_lift']),
            'limitation': 'Different target/normalization and visited states; score values across arms are not common physical quality metrics.'}
    result['conclusion'] = [
        'Early base clipping is real and materially changes aggregate scalar values in both arms. Mature-rollout clipping findings cannot be generalized to initial learning.',
        'RIGHT has larger initial clip lift than LEFT yet subsequently better completion; current evidence does not identify clipping as a selective LEFT failure mechanism.',
        'Initial command-rate cost is almost identical in both arms and dominates base penalty. Whether it blocks recovery corrections is not determined by these smoothed aggregates.',
        'LEFT has better imitation-score aggregates while worse completion later; target-specific normalized scores do not rank cross-task physical robustness.'
    ]
    for path in (Path(__file__).resolve(), PACKAGE / 'tb_logging.py', PACKAGE / 'reward_function.py'):
        hashed(path)
    result['input_hashes'] = HASHES
    (HERE / 'training_initial_probe.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    lines = ['# Reward nei primi25aggiornamenti LEFT/RIGHT', '',
        'Lettura offline dei TensorBoard esistenti: stessi4096step per iterazione, confronto fino102.400transizioni, prima del problema di ripristino LEFT.', '',
        '| Braccio | Iterazioni | Imitazione ponderata | Penalità base | Command-rate | Clip lift | Base dopo clip | Completi |',
        '|---|---|---:|---:|---:|---:|---:|---:|']
    for arm, data in result['runs'].items():
        for window in data['windows']:
            v = window['smoothed_scalar_arithmetic_means']
            lines.append(f"| {arm} | {window['iterations'][0]}–{window['iterations'][1]} | {v['mixed_imitation']:.4f} | {v['base_penalty']:.4f} | {v['command_rate_penalty']:.4f} | {v['clip_lift']:.4f} | {v['base_clipped']:.4f} | {window['complete']}/{window['episodes']} |")
    lines += ['', '## Metodo e limiti', '', *['- ' + v for v in result['method'].values()], '',
              '## Conclusioni', '', *['- ' + v for v in result['conclusion']]]
    text = '\n'.join(lines) + '\n'
    for old, new in [('primi25aggiornamenti', 'primi 25 aggiornamenti'), ('stessi4096step', 'stessi 4096 step'), ('fino102.400transizioni', 'fino a 102.400 transizioni')]:
        text = text.replace(old, new)
    (HERE / 'training_initial_probe.md').write_text(text)
    assert all(hashlib.sha256((REPO / path).read_bytes()).hexdigest() == digest for path, digest in HASHES.items())
    print(json.dumps({arm: data['windows'] for arm, data in result['runs'].items()}, indent=2))


if __name__ == '__main__':
    main()
