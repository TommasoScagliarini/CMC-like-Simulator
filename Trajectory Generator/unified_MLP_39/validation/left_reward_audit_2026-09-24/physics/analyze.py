"""Offline LEFT best72 reward/contact audit; never instantiates a simulator/policy."""
from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
REPO = PACKAGE.parents[1]
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ[key] = '1'
os.environ['MPLCONFIGDIR'] = str(HERE / 'mpl_cache')
sys.path.insert(0, str(PACKAGE.parent))
import numpy as np
from unified_MLP_39.reward_function import RewardConfig, compute_reward

SOURCE = PACKAGE / 'runs/rollout/MLP39_completion_2026-09-22_best072'
PLOTS = REPO / 'plot/09_24_2026_left_reward_audit/physics'
HASHES = {}
JOINTS = ('pros_knee_angle', 'pros_ankle_angle')


def read(path):
    raw = path.read_bytes()
    HASHES[str(path.relative_to(REPO))] = hashlib.sha256(raw).hexdigest()
    return json.loads(raw)


def mean(rows, key):
    return float(np.mean([r[key] for r in rows])) if rows else None


def summarize(rows):
    return {'n': len(rows), **{key: mean(rows, key) for key in (
        'reward', 'mixed_imitation', 'actual_imitation_score', 'served_imitation_score',
        'base_penalty', 'penetration_penalty', 'moment_penalty', 'penetration_mm',
        'normal_force_n', 'command_rate_penalty', 'base_clip_correction')},
        'reward_positive_n': sum(r['reward'] > 0 for r in rows),
        'reward_over_0_5_n': sum(r['reward'] > .5 for r in rows),
        'reward_over_0_8_n': sum(r['reward'] > .8 for r in rows),
        'max_penetration_mm': max((r['penetration_mm'] for r in rows), default=None),
        'rmse_deg': {j: float(np.sqrt(np.mean([r[j + '_error_deg'] ** 2 for r in rows])))
                     if rows else None for j in JOINTS}}


def main():
    episodes, series, draws = {}, {}, {}
    for folder in sorted(p for p in SOURCE.iterdir() if p.is_dir()):
        trace = read(folder / 'rollout_policy_trace.json')
        summary = read(folder / 'rollout_summary.json')
        cfg = RewardConfig.from_mapping(summary['reward_config'])
        converted = []
        for r in trace:
            t = r['reward_terms']
            reward, c = compute_reward(t, cfg, reference=r['policy_segment_values'])
            mixed = cfg.blend_imitation * c['imitation_score'] + cfg.blend_served_imitation * c['served_imitation_score']
            x = {'step': r['step'], 'time_s': r['time'], 'elapsed_s': r['time'] - 12.99,
                 'phase': t['pros_knee_angle_imitation_target_phase'],
                 'fsm': r['phase_fsm']['state_name'], 'reward': reward, 'mixed_imitation': mixed,
                 'actual_imitation_score': c['imitation_score'], 'served_imitation_score': c['served_imitation_score'],
                 'base_penalty': c['penalty'], 'penetration_penalty': c['grf_penetration_term'],
                 'moment_penalty': c['grf_ankle_moment_flip_term'], 'safety_penalty': c['safety_term'],
                 'base_clip_correction': c['reward_base'] - (mixed - c['penalty']),
                 'command_rate_penalty': cfg.command_rate_weight * t['command_rate_loss'],
                 'penetration_mm': 1000 * t['grf_penetration_m'],
                 'normal_force_n': t['grf_ankle_moment_flip_force_n'],
                 'grf_ankle_moment_nm': t['grf_ankle_moment_flip_tau_nm'],
                 'cop_relative_m': t['grf_ankle_moment_flip_cop_z_minus_ankle_z_m'],
                 'target': r['imitation_target_q'], 'invalid_events': r['phase_fsm']['invalid_event_count'],
                 'resyncs': r['phase_fsm']['resync_count']}
            for j in JOINTS:
                q, served, target = r['prosthetic_state'][j], r['prosthetic_state'][j + '_served_ref'], r['imitation_target_q'][j]
                x[j + '_error_deg'] = float(np.degrees(q - target))
                x[j + '_served_error_deg'] = float(np.degrees(served - target))
                x[j + '_tracking_error_deg'] = float(np.degrees(q - served))
                x[j + '_velocity_deg_s'] = float(np.degrees(r['prosthetic_state'][j + '_vel']))
            converted.append(x)
        reconstructed = sum(r['reward'] for r in converted)
        error = reconstructed - summary['episode_return']
        assert abs(error) < 1e-8, (folder, error)
        end = len(converted)
        duplicate = [i for i in range(1, end) if converted[i]['time_s'] <= converted[i - 1]['time_s'] + 1e-9]
        if duplicate:
            assert duplicate == [end - 1] and summary['end_reason'].endswith('_hard')
            end -= 1
        physical = converted[:end]
        series[folder.name] = physical
        if folder.name != 'deterministic':
            draws[folder.name] = read(folder / 'paired_noise_draws.json')
        episodes[folder.name] = {
            'end_reason': summary['end_reason'], 'recorded_steps': len(trace), 'physical_steps': end,
            'excluded_terminal_physical_rows': converted[end:], 'episode_return': summary['episode_return'],
            'scalar_reconstruction_error': error,
            'all_physical_rows': summarize(physical),
            'post_startup': summarize(physical[20:]),
            'physical_categories': {label: summarize([r for r in physical[20:] if condition(r)]) for label, condition in (
                ('pen_15_to_20mm', lambda r: 15 <= r['penetration_mm'] <= 20),
                ('pen_over_20mm', lambda r: r['penetration_mm'] > 20),
                ('pen_over_24mm', lambda r: r['penetration_mm'] > 24),
                ('moment_flip_penalized', lambda r: r['moment_penalty'] > 0),
                ('force_over_800N', lambda r: r['normal_force_n'] > 800),
            )}}

    common = min(len(v) for k, v in series.items() if k != 'deterministic')
    for name, rows in series.items():
        episodes[name]['common_prefix'] = summarize(rows[:common])
    groups = {}
    for scale in ('1', '0.5'):
        names = [f'noise_{scale}_seed{seed}' for seed in range(123, 128)]
        groups[scale] = {'common_prefix_steps_per_episode': common,
            'completed': sum(episodes[name]['end_reason'] == 'episode_time_limit' for name in names),
            'common_prefix_pooled': summarize([r for name in names for r in series[name][:common]]),
            'by_prescribed_phase_equal_horizon': {f'{lo / 10:.1f}-{(lo + 1) / 10:.1f}': summarize([
                r for name in names for r in series[name][:common] if lo / 10 <= r['phase'] < (lo + 1) / 10])
                for lo in range(10)}}

    failures = {}
    for name in ('noise_1_seed125', 'noise_1_seed127', 'noise_0.5_seed127'):
        seed = name[-3:]
        n = len(series[name])
        neighbors = ['deterministic', f'noise_1_seed{seed}', f'noise_0.5_seed{seed}']
        paired_n = min(len(series[v]) for v in neighbors)
        a, b = [draws[f'noise_{scale}_seed{seed}'] for scale in ('1', '0.5')]
        assert all(x['standard_normal'] == y['standard_normal'] for x, y in zip(a, b))
        for neighbor in neighbors:
            for actual, control in zip(series[name][:paired_n], series[neighbor][:paired_n]):
                assert abs(actual['time_s'] - control['time_s']) < 1e-8
                assert actual['target'] == control['target']
        # Window ends at the failed arm's last physically complete state. If a
        # comparator failed earlier, report missing coverage instead of padding.
        first = max(0, n - 20)
        threshold = next((i for i in range(n - 1, first - 1, -1)
                          if series[name][i]['penetration_mm'] <= 20), None)
        failures[name] = {'last_physical_step': n, 'window_first_step': first + 1,
            'last_subthreshold_step_in_final_window': None if threshold is None else threshold + 1,
            'time_from_last_subthreshold_to_last_physical_s': None if threshold is None else (n - 1 - threshold) * .01,
            'matched_standard_gaussian_draws': True,
            'matched_prescribed_target_steps': paired_n,
            'matched_window_end_step': min(n, paired_n),
            'matched_window': {v: summarize(series[v][first:min(n, paired_n)]) for v in neighbors},
            'window': {v: {**summarize(series[v][first:n]),
                          'full_window_covered': len(series[v]) >= n} for v in neighbors},
            'snapshots': {v: [series[v][i] for i in (n - 21, n - 11, n - 6, n - 2, n - 1)
                              if 0 <= i < len(series[v])] for v in neighbors}}

    result = {'scope': 'Offline LEFT best72 frozen-policy traces: 5 paired noise seeds x 2 scales, one deterministic, fixed start; no new rollout or training.',
        'episodes': episodes, 'groups': groups, 'failures': failures,
        'limitations': [
            'Same Gaussian draws do not imply same absolute action perturbations: each trajectory has its own state-dependent policy standard deviation.',
            'Failure windows compare different action/state histories and are associations, not interventions isolating a joint or reward coefficient.',
            'Contact values come from physical reward diagnostics, not detector_sensors geometrical penetration.',
            'Scalar reconstruction includes the hard terminal row; physical statistics exclude its duplicated timestamp and zeroed contact fields.',
            'Means across complete/failing full trajectories are descriptive only; use shared 303-step prefixes or explicitly paired time windows for comparisons.',
            'Thresholds of 15, 20, 24 mm and 800 N are diagnostic categories, not evidence that all included states are failures.',
            'Immediate positive reward does not establish that a behavior has favorable discounted return; episode termination removes future reward.',
            'Only one learned policy, one start and five noise seeds; no independent training replicates or statistical equivalence test.'
        ]}
    (HERE / 'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    with (HERE / 'steps.csv').open('w', newline='') as fh:
        fields = ['episode'] + [k for k in next(iter(series.values()))[0] if k != 'target']
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for name, rows in series.items():
            for row in rows:
                writer.writerow({'episode': name, **{k: v for k, v in row.items() if k != 'target'}})
    make_plots(series)
    write_findings(result)
    for path in (Path(__file__).resolve(), PACKAGE / 'reward_function.py',
                 PACKAGE.parent / 'baseline_MLP/experimental_morphology_corridor.py'):
        HASHES[str(path.relative_to(REPO))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (HERE / 'input_hashes.json').write_text(json.dumps(HASHES, indent=2) + '\n')
    assert all(hashlib.sha256((REPO / path).read_bytes()).hexdigest() == digest for path, digest in HASHES.items())
    print(json.dumps({'episodes': len(episodes), 'common_prefix': common, 'groups': groups,
                      'max_return_reconstruction_error': max(abs(v['scalar_reconstruction_error']) for v in episodes.values())}, indent=2))


def make_plots(series):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    PLOTS.mkdir(parents=True, exist_ok=True)
    fig, axs = plt.subplots(4, 2, figsize=(13, 12), sharex='col', layout='constrained')
    for column, (seed, limits) in enumerate(((125, (2.7, 3.12)), (127, (4.35, 4.75)))):
        for name, label, color in [('deterministic', 'Deterministico', '#222222'),
                                   (f'noise_1_seed{seed}', 'Rumore originale', '#bb3344'),
                                   (f'noise_0.5_seed{seed}', 'Rumore dimezzato', '#3366bb')]:
            selected = [r for r in series[name] if limits[0] <= r['elapsed_s'] <= limits[1]]
            t = [r['elapsed_s'] for r in selected]
            for ax, key in zip(axs[:, column], ['reward', 'penetration_mm', 'grf_ankle_moment_nm', 'mixed_imitation']):
                ax.plot(t, [r[key] for r in selected], label=label, color=color)
                ax.grid(alpha=.2)
        axs[0, column].set_title(f'LEFT best72 — seed {seed}, stessi target e estrazioni gaussiane')
        axs[0, column].axhline(0, color='grey', ls=':')
        for value in (20, 28):
            axs[1, column].axhline(value, color='grey', ls='--')
        axs[2, column].axhline(-8, color='grey', ls='--')
        axs[3, column].set_xlabel('Tempo episodio [s]')
    for ax, label in zip(axs[:, 0], ['Reward effettiva', 'Penetrazione [mm]', 'Momento GRF ankle [Nm]', 'Score imitativo ponderato']):
        ax.set_ylabel(label)
    axs[0, 0].legend(fontsize=8)
    fig.savefig(PLOTS / 'paired_failure_reward_contact.png', dpi=160)
    plt.close(fig)


def write_findings(result):
    lines = ['# Audit fisico della reward LEFT best72', '',
        'Analisi offline di 11 episodi già salvati; nessuna simulazione o modifica del training.', '',
        'La reward viene ricostruita con `compute_reward` e configurazione del rollout. Le somme includono i terminali e coincidono con gli episode_return entro 1e-8. '
        'La riga hard finale di seed127 sigma1 viene esclusa dalle statistiche fisiche perché ha timestamp duplicato e contatti azzerati.', '',
        '| Caso | Esito | Passi fisici | Reward media, prefisso 303 | RMSE knee/ankle, prefisso 303 [°] |',
        '|---|---|---:|---:|---:|']
    for name, ep in result['episodes'].items():
        p = ep['common_prefix']
        lines.append(f"| {name} | {ep['end_reason']} | {ep['physical_steps']} | {p['reward']:.3f} | {p['rmse_deg'][JOINTS[0]]:.3f} / {p['rmse_deg'][JOINTS[1]]:.3f} |")
    lines.extend(['', '## Finestre finali appaiate: fino a 20 passi fisici prima del fallimento', '',
                  'Tutti i confronti nella stessa finestra usano esattamente gli stessi tempi. Per il fallimento sigma0.5 seed127, il confronto comune si ferma a465 e copre16passi, perché sigma1è già terminato.', '',
                  '| Caso fallito | Caso confrontato | N | Reward media | Imitazione ponderata | Penalità penetrazione | Penalità momento | Pen. max [mm] |',
                  '|---|---|---:|---:|---:|---:|---:|---:|'])
    for failed, item in result['failures'].items():
        for other, window in item['matched_window'].items():
            lines.append(f"| {failed} | {other} | {window['n']} | {window['reward']:.3f} | {window['mixed_imitation']:.3f} | {window['penetration_penalty']:.3f} | {window['moment_penalty']:.3f} | {window['max_penetration_mm']:.2f} |")
    lines.extend(['', 'Gli snapshot numerici completi (angoli, errori actual/served, velocità, fase, forza, momento, reward) sono in `metrics.json`; `steps.csv` conserva tutte le righe fisiche valide.', '',
        '## Penetrazione oltre soglia e reward immediata', '',
        'Esclusi i primi0,2s. Gli episodi completi possono superare20mm e recuperare: la categoria descrive una penalità attiva, non classifica automaticamente un fallimento.', '',
        '| Rumore originale | Passi oltre20mm | Reward positiva | Reward >0,5 | Reward media su questi passi |',
        '|---|---:|---:|---:|---:|'])
    for seed in range(123, 128):
        row = result['episodes'][f'noise_1_seed{seed}']['physical_categories']['pen_over_20mm']
        value = '-' if not row['n'] else f"{row['reward']:.3f}"
        lines.append(f"| {seed} | {row['n']} | {row['reward_positive_n']} | {row['reward_over_0_5_n']} | {value} |")
    lines.extend(['',
        '## Interpretazione', '',
        '- Seed125 sigma1: il momento GRF molto negativo attiva una forte penalità prima che la penetrazione superi 20 mm. Nella finestra finale la reward è già fortemente negativa; il caso non supporta una mancata penalizzazione locale.',
        '- Seed127 fallisce con entrambe le scale, con momento GRF positivo nella salita finale della penetrazione: aumentare solo anti-flip non affronta questo meccanismo.',
        '- Le ultime osservazioni sotto20mm precedono l’ultima osservazione fisica di40ms (seed125 sigma1),30ms (seed127 sigma1, poi hard failure) e70ms (seed127 sigma0.5). La salita finale è rapida; questa osservazione motiva verificare il margine anticipatorio, ma non dimostra che anticipare la penalità risolva il problema.',
        '- Seed127 sigma1 negli ultimi20passi fisici ha reward media+0,485 contro−0,349 del deterministico che sopravvive: il deterministico imita peggio e paga maggiore anti-flip. Nei singoli ultimi passi del collasso la reward diventa però negativa; inoltre il fallimento toglie reward future. Il confronto locale non dimostra che la reward globale preferisca il fallimento.',
        '- La penetrazione non costa direttamente sotto 20 mm; subito oltre soglia il costo quadratico è ancora piccolo. Questo non è un bug, ma una scelta di margine/sensibilità da valutare.',
        '- Confronti appaiati tengono fissi tempo e target, ma non stato né storia delle azioni; non provano causalità di un peso della reward.', '',
        '## Limiti', '', *['- ' + text for text in result['limitations']]])
    text = '\n'.join(lines) + '\n'
    for old, new in [('a465', 'a 465'), ('copre16passi', 'copre 16 passi'), ('sigma1è', 'sigma 1 è'),
                     ('primi0,2s', 'primi 0,2 s'), ('oltre20mm', 'oltre 20 mm'), ('superare20mm', 'superare 20 mm'),
                     ('sotto20mm', 'sotto 20 mm'), ('di40ms', 'di 40 ms'), (',30ms', ', 30 ms'), ('e70ms', 'e 70 ms'),
                     ('ultimi20passi', 'ultimi 20 passi'), ('media+0,485', 'media +0,485'), ('contro−0,349', 'contro −0,349')]:
        text = text.replace(old, new)
    (HERE / 'findings.md').write_text(text)


if __name__ == '__main__':
    main()
