"""Stream finished-episode telemetry and compare completions with early endings.

No simulator, Ray, Torch or checkpoint loading. --output is a directory.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

JOINTS = {'knee': 'pros_knee_angle', 'ankle': 'pros_ankle_angle'}
WINDOWS = (5, 10, 25, 50)
DEGREES = 180 / math.pi


def number(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def mean(values):
    valid = [value for value in values if value is not None]
    return sum(valid) / len(valid) if valid else None


def version_label(value):
    value = number(value)
    return 'unknown' if value is None else f'{value:g}'


def proximity(last_step, terminal_step, window, *, known_absent=False, covered=False):
    if last_step is not None and 0 <= terminal_step - last_step <= window:
        return 'yes'
    if last_step is not None or known_absent or covered:
        return 'no'
    return 'unknown'


def onset(tail, predicate):
    """First active row within the retained tail, with explicit left censoring."""
    for index, row in enumerate(tail):
        if predicate(row):
            censored = index == 0 or (tail[index - 1]['episode_step'] + 1 != row['episode_step'])
            contact = row.get('terminal_contact') or {}
            hard_contact = bool(contact.get('available')) and contact.get('source') == 'hard_penetration_guard'
            return {'first_active_step_in_tail': row['episode_step'],
                    'time_s': number(contact.get('time_s')) if hard_contact else row.get('time_s'),
                    'policy_boundary_time_s': row.get('time_s'),
                    'left_censored': censored,
                    'steps_before_end': tail[-1]['episode_step'] - row['episode_step']}
    return None


def summarize_episode(record):
    tail = sorted(record.get('tail', [])[-50:], key=lambda row: row['episode_step'])
    last = tail[-1] if tail else {}
    steps = int(record.get('episode_steps', 0))
    observed = int(record.get('observed_steps', 0))
    reason = record.get('end_reason')
    outcome = ('full' if reason == 'episode_time_limit' else
               'failed' if record.get('terminated') or reason else 'unknown')
    policy = record.get('policy_versions') or {}
    tail_versions = [(row['episode_step'], number(row.get('policy_outputs', {}).get('weights_seq_no')))
                     for row in tail]
    switches = [step for (previous_step, previous), (step, current) in zip(tail_versions, tail_versions[1:])
                if step == previous_step + 1 and previous is not None and current is not None and current != previous]
    first_version, last_version = number(policy.get('first')), number(policy.get('last'))
    if last_version is None and tail_versions:
        last_version = tail_versions[-1][1]
    switch_count = number(policy.get('switch_count'))
    # Legacy files expose only the tail. Do not label the whole episode stable
    # merely because the last 50 observations used the same policy.
    complete_versions = bool(policy) and policy.get('rows_without_version', 0) == 0 and policy.get('rows_with_version') == observed
    stability = ('stable' if switch_count == 0 else 'switched') if complete_versions else 'unknown'
    peaks, events = record.get('peaks', {}), record.get('event_counts', {})
    resync_step = number((record.get('last_resync') or {}).get('episode_step'))
    cancel_step = number((record.get('last_hs_cancel') or {}).get('episode_step'))
    resync_count = number(events.get('resync_rows'))
    cancel_count = number(peaks.get('hs_cancelled_count', {}).get('value'))
    quality_complete = not record.get('partial_episode', True) and record.get('missing_steps') == 0 and record.get('rows_with_missing_fields') == 0
    recent = {}
    for window in WINDOWS:
        rows = [(step, value) for step, value in tail_versions if steps - window <= step <= steps]
        covered = (len(rows) == window + 1 and rows[0][0] == steps - window
                   and all(value is not None for _, value in rows))
        recent[str(window)] = {
            'resync': proximity(resync_step if resync_step is not None and steps - resync_step <= window else None,
                                steps, window, known_absent=quality_complete and (resync_count == 0 or resync_step is not None)),
            'hs_cancel': proximity(cancel_step if cancel_step is not None and steps - cancel_step <= window else None,
                                   steps, window, known_absent=quality_complete and (cancel_count == 0 or cancel_step is not None)),
            'policy_switch': proximity(switches[-1] if switches and steps - switches[-1] <= window else None, steps, window,
                                       known_absent=complete_versions and switch_count == 0, covered=covered),
        }
    errors = {}
    for label, joint in JOINTS.items():
        accumulator = record.get('joint_error_accumulators', {}).get(joint, {})
        count, squared = number(accumulator.get('count')), number(accumulator.get('sum_squared_rad2'))
        tail_errors = []
        for row in tail:
            if row.get('segment_completed') is False:
                # Retain a placeholder at the terminal index. Removing the row
                # and taking [-1] would relabel a previous error as terminal.
                tail_errors.append(None)
                continue
            q = number(row.get('joints', {}).get(joint))
            target = number(row.get('imitation_target', {}).get('q', {}).get(joint))
            tail_errors.append((q - target) if q is not None and target is not None else None)
        terminal = tail_errors[-1] if tail_errors else None
        tail_mse = mean([value * value if value is not None else None for value in tail_errors])
        errors[label] = {
            'terminal_signed_deg': terminal * DEGREES if terminal is not None else None,
            'terminal_abs_deg': abs(terminal) * DEGREES if terminal is not None else None,
            'rmse_observed_episode_deg': math.sqrt(squared / count) * DEGREES if count and squared is not None else None,
            'rmse_tail_deg': math.sqrt(tail_mse) * DEGREES if tail_mse is not None else None,
            'observed_error_rows': count,
            'tail_error_rows': sum(value is not None for value in tail_errors),
        }
    penetration = number(peaks.get('penetration_m', {}).get('value'))
    rates = {name + '_per_100_observed_steps': 100 * value / observed if observed and value is not None else None
             for name, value in [('action_clip_rows', number(events.get('action_clip_rows'))),
                                 ('resync_rows', resync_count), ('hs_cancel_count', cancel_count),
                                 ('invalid_event_rows', number(events.get('invalid_event_rows')))]}
    clips = [number(row.get('reward_terms', {}).get('policy_action_clip_fraction')) for row in tail]
    penetration_onsets = {
        f'above_{threshold}_mm': onset(tail, lambda row, threshold=threshold:
            (number(row.get('reward_terms', {}).get('grf_penetration_m')) or 0) * 1000 > threshold)
        for threshold in (0, 10, 20, 30)}
    penetration_onsets['penalty_active'] = onset(tail, lambda row:
        (number(row.get('reward_terms', {}).get('grf_penetration_loss')) or 0) > 0)
    duration = number(record.get('duration_s'))
    return {
        'episode_uid': record.get('episode_uid'), 'worker_index': record.get('worker_index'),
        'outcome': outcome, 'end_reason': reason or 'unknown', 'episode_steps': steps,
        'observed_steps': observed, 'duration_s': duration, 'terminal_time_s': number(record.get('last_time_s')),
        'terminal_time_basis': 'policy_boundary',
        'terminal_contact_time_s': number(record.get('terminal_contact_time_s')),
        'terminal_segment_completed': last.get('segment_completed'),
        'terminal_kinematic_sample_status': ('incomplete_segment' if last.get('segment_completed') is False else
                                             'completed_segment' if last.get('segment_completed') is True else 'legacy_unknown'),
        'terminal_phase': last.get('phase_fsm', {}).get('state_name') or 'unknown',
        'terminal_target_phase': number(last.get('imitation_target', {}).get('phase', {}).get('pros_knee_angle')),
        'quality_complete': quality_complete, 'partial_episode': bool(record.get('partial_episode', True)),
        'missing_steps': record.get('missing_steps'), 'rows_with_missing_fields': record.get('rows_with_missing_fields'),
        'peak_penetration_mm': penetration * 1000 if penetration is not None else None,
        'peak_penetration_step': peaks.get('penetration_m', {}).get('episode_step'),
        'peak_penetration_time_s': number(peaks.get('penetration_m', {}).get('time_s')),
        'peak_penetration_source': peaks.get('penetration_m', {}).get('source'),
        'peak_penetration_phase': peaks.get('penetration_m', {}).get('phase_fsm_state'),
        'penetration_onsets_in_tail': penetration_onsets, 'joint_errors': errors,
        'event_counts': events, 'hs_cancel_count': cancel_count, 'event_rates': rates,
        'tail_mean_clipped_action_component_fraction': mean(clips),
        'policy_versions': {'first': first_version, 'last': last_version, 'switch_count': switch_count,
                            'coverage_complete': complete_versions, 'stability': stability,
                            'source': 'episode_accumulator' if policy else 'legacy_tail_only',
                            'switch_steps_observed_in_tail': switches},
        'terminal_proximity_steps': recent,
        'duration_band_s': f'{int(duration + 1e-7)}–{int(duration + 1e-7) + 1}' if duration is not None else 'unknown',
        'policy_last_window': (f'{int(last_version) // 5 * 5}–{int(last_version) // 5 * 5 + 4}'
                               if last_version is not None else 'unknown'),
        'tail_observed_rows': len(tail),
        'tail_partial_segment_rows': sum(row.get('segment_completed') is False for row in tail),
    }


class Stats:
    def __init__(self):
        self.count, self.total, self.minimum, self.maximum = 0, 0.0, None, None

    def add(self, value):
        value = number(value)
        if value is None:
            return
        self.count += 1
        self.total += value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    def result(self):
        return {'n': self.count, 'mean': self.total / self.count if self.count else None,
                'min': self.minimum, 'max': self.maximum}


class Group:
    def __init__(self):
        self.outcomes, self.reasons, self.phases = Counter(), Counter(), Counter()
        self.complete_quality = 0
        self.stats = defaultdict(Stats)
        self.near = defaultdict(Counter)
        self.observed_steps, self.events, self.event_observed_steps = 0, Counter(), Counter()
        self.onsets = defaultdict(Counter)

    def add(self, row):
        self.outcomes[row['outcome']] += 1
        self.reasons[row['end_reason']] += 1
        self.phases[row['terminal_phase']] += 1
        self.complete_quality += row['quality_complete']
        self.observed_steps += row['observed_steps']
        for key, value in {**row['event_counts'], 'hs_cancel_count': row['hs_cancel_count']}.items():
            if number(value) is not None:
                self.events[key] += value
                self.event_observed_steps[key] += row['observed_steps']
        for key in ('duration_s', 'episode_steps', 'terminal_time_s', 'terminal_target_phase', 'peak_penetration_mm', 'tail_mean_clipped_action_component_fraction'):
            self.stats[key].add(row[key])
        for joint, values in row['joint_errors'].items():
            for key, value in values.items():
                self.stats[f'{joint}.{key}'].add(value)
        for key, value in row['event_rates'].items():
            self.stats[key].add(value)
        self.stats['policy_switch_count'].add(row['policy_versions']['switch_count'])
        for window, events in row['terminal_proximity_steps'].items():
            for event, status in events.items():
                self.near[f'{event}_within_{window}_steps'][status] += 1
        for key, value in row['penetration_onsets_in_tail'].items():
            self.onsets[key]['active_in_tail' if value is not None else 'not_observed_active'] += 1
            if value is not None:
                self.onsets[key]['left_censored' if value['left_censored'] else 'observed_transition'] += 1
                self.stats[f'{key}.first_active_steps_before_end'].add(value['steps_before_end'])

    def result(self):
        classified = self.outcomes['full'] + self.outcomes['failed']
        return {'episodes': sum(self.outcomes.values()), 'outcomes': dict(self.outcomes),
                'completion_fraction': self.outcomes['full'] / classified if classified else None,
                'quality_complete_episodes': self.complete_quality, 'end_reasons': dict(self.reasons),
                'terminal_phases': dict(self.phases), 'stats': {key: value.result() for key, value in sorted(self.stats.items())},
                'terminal_proximity': {key: dict(value) for key, value in sorted(self.near.items())},
                'penetration_onsets_in_tail': {key: dict(value) for key, value in self.onsets.items()},
                'pooled_event_rows_per_100_observed_steps': {
                    key: 100 * value / self.event_observed_steps[key] if self.event_observed_steps[key] else None
                    for key, value in self.events.items()}, 'observed_steps': self.observed_steps}


LIMITATIONS = [
    'Analisi descrittiva di episodi conclusi e scritti: episodi ancora aperti, processi morti o scritture perse non sono inclusi. La percentuale non coincide automaticamente con le metriche RLlib per iterazione.',
    'Il 38.8% storico è il contesto della diagnosi, non un denominatore di questa nuova telemetria. Confrontare finestre di policy ed esposizione prima di attribuire un miglioramento.',
    'Episodi lunghi hanno più opportunità di clipping, resync, cancellazioni e cambi di policy. Sono riportati tassi per 100 step, durata e strati per versione; questi confronti restano confusi dalla durata e dalla selezione degli episodi sopravvissuti.',
    'RMSE episodio è calcolato sugli step osservati dell’intero episodio; il RMSE tail riguarda al massimo gli ultimi 50. Gli episodi falliti coprono intervalli più brevi: gli RMSE non hanno supporto temporale identico.',
    'Le righe segment_completed=False sono escluse dal RMSE cinematico; il loro errore terminale resta assente, senza sostituirlo con il campione precedente. Nei record legacy senza questo flag il completamento del segmento resta sconosciuto e si conserva il conteggio storico. La durata e terminal_time_s usano il boundary di policy; terminal_contact_time_s e i picchi hard disponibili riportano separatamente il tempo misurato al substep.',
    'La precedenza temporale di resync, cancellazione o cambio policy non dimostra causalità. I conteggi entro N step confrontano la fine degli episodi falliti con la fine amministrativa dei completi; non stimano rischio per singolo evento.',
    'Onset indica la prima attività osservata nella coda, non necessariamente l’inizio dell’evento nell’intero episodio; attività già presente all’inizio è censurata a sinistra. Le soglie 0/10/20/30 mm sono diagnostiche; penalty_active usa la penalità effettivamente registrata.',
    'I cambi di policy temporizzati sono rilevati solo tra righe consecutive nella coda. 50 righe coprono 49 intervalli: l’assenza di cambi nei 50 step precedenti può restare unknown. Le versioni pesi non sono automaticamente numeri di iterazione.',
    'I record legacy privi del riepilogo policy_versions consentono solo controlli nella coda: first e switch_count dell’intero episodio restano sconosciuti. Nessuna ipotesi sul numero di worker.',
]


def percent(value):
    return 'n/d' if value is None else f'{100 * value:.1f}%'


def render_markdown(result):
    groups = result['groups']
    all_group = groups.get('all', {})
    text = ['# Diagnosi offline degli episodi conclusi', '',
            f"Episodi letti: **{result['records_analyzed']}**; completamento: **{percent(all_group.get('completion_fraction'))}**. "
            f"Record duplicati: {result['duplicate_records']}; righe invalide: {result['invalid_records']}.", '',
            '| Esito | Episodi | Durata media [s] | Picco penetrazione medio [mm] | RMSE ginocchio [°] | RMSE caviglia [°] | Step con clipping |',
            '|---|---:|---:|---:|---:|---:|---:|']
    for name in ('full', 'failed', 'unknown'):
        group = groups.get(f'outcome:{name}')
        if not group:
            continue
        def stat(key):
            value = group['stats'].get(key, {}).get('mean')
            return 'n/d' if value is None else f'{value:.3f}'
        clip = group['pooled_event_rows_per_100_observed_steps'].get('action_clip_rows')
        text.append(f"| {name} | {group['episodes']} | {stat('duration_s')} | {stat('peak_penetration_mm')} | "
                    f"{stat('knee.rmse_observed_episode_deg')} | {stat('ankle.rmse_observed_episode_deg')} | "
                    + ('n/d' if clip is None else f'{clip:.1f}%') + ' |')
    text.extend(['', '## Motivi e fasi terminali', ''])
    for name in ('full', 'failed'):
        group = groups.get(f'outcome:{name}')
        if group:
            text.extend([f"- **{name}** — motivi: `{json.dumps(group['end_reasons'], ensure_ascii=False)}`; "
                         f"fasi: `{json.dumps(group['terminal_phases'], ensure_ascii=False)}`."])
    text.extend(['', '## Errori terminali e ultimi 50 step', '',
                 '| Esito | Errore finale ginocchio [°] | Errore finale caviglia [°] | RMSE tail ginocchio [°] | RMSE tail caviglia [°] |',
                 '|---|---:|---:|---:|---:|'])
    for name in ('full', 'failed'):
        group = groups.get(f'outcome:{name}')
        if group:
            cells = []
            for key in ('knee.terminal_abs_deg', 'ankle.terminal_abs_deg', 'knee.rmse_tail_deg', 'ankle.rmse_tail_deg'):
                value = group['stats'].get(key, {}).get('mean')
                cells.append('n/d' if value is None else f'{value:.3f}')
            text.append(f"| {name} | {' | '.join(cells)} |")
    text.extend(['', 'Errori terminali assoluti medi. Il dettaglio firmato è nel JSON. Prima attività di penetrazione nella coda:', '',
                 '| Esito | Soglia | Episodi con attività | Transizioni osservate | Già attiva al bordo / censura |',
                 '|---|---|---:|---:|---:|'])
    for name in ('full', 'failed'):
        for threshold, counts in groups.get(f'outcome:{name}', {}).get('penetration_onsets_in_tail', {}).items():
            text.append(f"| {name} | {threshold} | {counts.get('active_in_tail', 0)} | "
                        f"{counts.get('observed_transition', 0)} | {counts.get('left_censored', 0)} |")
    text.extend(['', '## Eventi vicini alla fine', '',
                 'Conteggi yes / no / unknown; “full” termina al limite temporale. Sono associazioni descrittive.', '',
                 '| Evento | Finestra [step] | full | failed |', '|---|---:|---|---|'])
    for event in ('resync', 'hs_cancel', 'policy_switch'):
        for window in WINDOWS:
            cells = []
            for outcome in ('full', 'failed'):
                counts = groups.get(f'outcome:{outcome}', {}).get('terminal_proximity', {}).get(f'{event}_within_{window}_steps', {})
                cells.append(' / '.join(str(counts.get(status, 0)) for status in ('yes', 'no', 'unknown')))
            text.append(f'| {event} | {window} | {cells[0]} | {cells[1]} |')
    text.extend(['', '## Finestre della versione policy terminale', '',
                 '| Versioni pesi | Episodi | Completamento | Durata media [s] |', '|---|---:|---:|---:|'])
    for name, group in sorted(groups.items()):
        if name.startswith('last_window:'):
            duration = group['stats'].get('duration_s', {}).get('mean')
            text.append(f"| {name.split(':', 1)[1]} | {group['episodes']} | {percent(group['completion_fraction'])} | "
                        + ('n/d' if duration is None else f'{duration:.3f}') + ' |')
    text.extend(['', '| Versione durante l’episodio | Episodi | Completamento | Durata media [s] |',
                 '|---|---:|---:|---:|'])
    for name in ('stable', 'switched', 'unknown'):
        group = groups.get(f'stability:{name}')
        if group:
            duration = group['stats'].get('duration_s', {}).get('mean')
            text.append(f"| {name} | {group['episodes']} | {percent(group['completion_fraction'])} | "
                        + ('n/d' if duration is None else f'{duration:.3f}') + ' |')
    text.extend(['', 'Strati aggiuntivi in `summary.json`: versione iniziale/finale, coppia iniziale→finale, policy stabile/con cambi, durata×versione e qualità dei dati. Dettaglio episodio in `per_episode.jsonl`.', '', '## Limiti', ''])
    text.extend(f'- {item}' for item in LIMITATIONS)
    return '\n'.join(text) + '\n'


def analyze(directory, output):
    directory, output = Path(directory).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    files = sorted(directory.rglob('episodes_*.jsonl'))
    if not files:
        raise FileNotFoundError(f'No episode telemetry JSONL in {directory}')
    groups, seen, workers = defaultdict(Group), set(), set()
    invalid, duplicates, count = 0, 0, 0
    errors = []
    with (output / 'per_episode.jsonl').open('w', encoding='utf-8') as target:
        for path in files:
            with path.open(encoding='utf-8') as stream:
                for line_number, line in enumerate(stream, 1):
                    try:
                        record = json.loads(line)
                        if record.get('schema') != 'unified39_episode_telemetry_v1':
                            raise ValueError('Unsupported telemetry schema')
                        uid = record.get('episode_uid')
                        if not uid:
                            raise ValueError('Missing episode_uid')
                        if uid in seen:
                            duplicates += 1
                            continue
                        row = summarize_episode(record)
                    except (ValueError, KeyError, TypeError, AttributeError) as error:
                        invalid += 1
                        if len(errors) < 20:
                            errors.append({'file': str(path), 'line': line_number, 'error': str(error)})
                        continue
                    seen.add(uid)
                    count += 1
                    workers.add(str(row['worker_index']))
                    target.write(json.dumps(row, allow_nan=False) + '\n')
                    versions = row['policy_versions']
                    keys = ['all', f"outcome:{row['outcome']}", f"quality_complete:{row['quality_complete']}",
                            f"first_version:{version_label(versions['first'])}", f"last_version:{version_label(versions['last'])}",
                            f"version_pair:{version_label(versions['first'])}→{version_label(versions['last'])}",
                            f"stability:{versions['stability']}", f"duration:{row['duration_band_s']}",
                            f"last_window:{row['policy_last_window']}",
                            f"window_duration:{row['policy_last_window']}|{row['duration_band_s']}"]
                    for key in keys:
                        groups[key].add(row)
    result = {'schema': 'unified39_episode_analysis_v1', 'telemetry_directory': str(directory),
              'input_files': [str(path) for path in files], 'records_analyzed': count,
              'invalid_records': invalid, 'duplicate_records': duplicates, 'workers_observed': sorted(workers),
              'input_errors_first20': errors, 'groups': {key: value.result() for key, value in sorted(groups.items())},
              'limitations': LIMITATIONS, 'no_causal_estimation': True,
              'per_episode_summary': str(output / 'per_episode.jsonl')}
    (output / 'summary.json').write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
    (output / 'summary.md').write_text(render_markdown(result), encoding='utf-8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--telemetry-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = analyze(args.telemetry_dir, args.output)
    print(json.dumps({'records_analyzed': report['records_analyzed'], 'invalid_records': report['invalid_records'],
                      'summary': str(args.output / 'summary.md')}))
    if report['invalid_records'] or not report['records_analyzed']:
        raise SystemExit(2)
