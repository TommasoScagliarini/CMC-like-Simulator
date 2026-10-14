"""Seal descriptive interpretation of finalized18-case phaseJSON only.

No event re-detection, STO reads, native integration, FK, fitting or input edit.
Explicitly show incomplete closing-HS guards even when an event was matched.
"""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def read(p):
    return json.loads(p.read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))


def fp(p):
    return {'path': p.resolve().relative_to(ROOT).as_posix(), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}


def stats(pairs):
    values = [p['error_s'] for p in pairs]
    return {'count': len(values), 'bias_s': sum(values)/len(values),
            'mae_s': sum(abs(v) for v in values)/len(values), 'max_abs_s': max(abs(v) for v in values)} if values else None


def main():
    output = HERE / 'scientific_review_v2.json'
    if output.exists():
        raise FileExistsError('Preserve final independent phase review')
    agg_path = HERE / 'aggregate_phase_metrics_v2.json'
    protocol_path = HERE / 'phase_timing_protocol_v2.json'
    aggregate, protocol = read(agg_path), read(protocol_path)
    if aggregate['status'] != 'all18_finalized_descriptive_phase_diagnostics' or len(aggregate['cases']) != 18 or set(aggregate['cases']) != set(protocol['cases']):
        raise ValueError('Require exactly all18 finalphase cases')
    paths = {agg_path, protocol_path, Path(__file__), HERE / 'analyze_phase_timing_v2.py',
             HERE / 'phase_timing_protocol.json', HERE / 'analyze_phase_timing.py', HERE / 'watch_stdout.log',
             HERE / 'attempt_01_receipt.json', HERE / 'watch_v2_stdout.log'}
    paths.update(ROOT / c['metrics']['path'] for c in aggregate['cases'].values())
    before = {fp(p)['path']: fp(p)['sha256'] for p in paths}
    case_summaries = {}
    for name, entry in aggregate['cases'].items():
        path = ROOT / entry['metrics']['path']
        if fp(path) != entry['metrics']:
            raise ValueError('Final phase case changed after aggregation')
        data = read(path)
        if data['case_id'] != name or data['phase_protocol'] != fp(protocol_path) or data['case'] != protocol['cases'][name]:
            raise ValueError('Phase case/protocol identity differs')
        channels = {}
        for label, c in data['primary_channels'].items():
            summary = {'status': c['status'], 'saved_support_s': c['saved_support_s']}
            if c['comparison']:
                summary['events'] = {}
                for kind, event in c['comparison'].items():
                    summary['events'][kind] = {}
                    for clock, e in event['channels'].items():
                        matching, support, tol = e['matching'], c['saved_support_s'], e['tolerance_s']
                        covered = e['fully_observed_reference_matching_guard_s']
                        pairs = matching['pairs']
                        observable = []
                        for t in event['reference_times_s']:
                            pair = next((p for p in pairs if p['reference_time_s'] == t), None)
                            full = support[0] <= t-tol and support[1] >= t+tol
                            if full != (t in covered):
                                raise ValueError('Final guard classification differs from source support/tolerance')
                            observable.append({'reference_time_s': t, 'saved_matching_guard_complete': full,
                                               'matching_guard_s': [t-tol, t+tol], 'matched_observed_event': pair,
                                               'absence_interpretation_if_unmatched': 'observable_algorithmic_reference_disagreement' if full else 'censored_or_unknown_not_physiological_miss'})
                        summary['events'][kind][clock] = {
                            'raw_inherited_matching': matching,
                            'complete_guard_matching_statistics': stats([p for p in pairs if p['reference_time_s'] in covered]),
                            'nominal_observability': observable,
                            'unmatched_with_complete_guard_s': e['unmatched_reference_fully_observed_matching_guard_s'],
                            'unmatched_censored_or_unknown_s': e['unmatched_reference_censored_or_unknown_s']}
            channels[label] = summary
        fraction = data['swing_impulse_fraction']
        pooled = fraction.get('pooled')
        case_summaries[name] = {'phase_metrics': fp(path), 'runtime_status': data['runtime_status'],
                               'close_error': data.get('close_error'), 'primary_event_channels': channels,
                               'swing_fraction_status': fraction['status'],
                               'swing_impulse_percent': pooled['swing_fraction_of_primary_impulse']*100 if pooled and pooled['swing_fraction_of_primary_impulse'] is not None else None,
                               'phase_sum_minus_primary_Ns': pooled['phase_sum_minus_primary_Ns'] if pooled else None,
                               'last_actor_counters': data['observed_full_episode_journal_secondary']['last_counters'],
                               'historically_accepted_HS_revoked_count': len(data['observed_full_episode_journal_secondary']['previously_accepted_heel_strikes_revoked']),
                               'retained_event_count': len(data['observed_full_episode_journal_secondary']['ordinary_actor_events_retained_after_repairs']),
                               'retained_count_scope': 'Whole actually observed episode including startup; distinct from primary matching and independent physiological cycle count'}
    after = {fp(p)['path']: fp(p)['sha256'] for p in paths}
    if before != after:
        raise ValueError('Final phase evidence changed during review')
    result = {'created_utc': datetime.now(timezone.utc).isoformat(), 'instruction_check_token': 'CMC_AGENT_OK_2026',
              'status': 'SEALED_all18_descriptive_phase_timing_review', 'aggregate': fp(agg_path), 'protocol': fp(protocol_path),
              'case_count': 18, 'cases': case_summaries, 'new_Claude_branch_read': False,
              'event_selection': 'Original4HS/3TO nominal comparison and inherited tolerances preserved. Guard-complete subset statistics merely disclose which matched references have full saved support; no threshold/window retuning.',
              'closing_HS': 'Every closingHS has incomplete right matching guard even in full-duration runs. Matched observed closing errors remain valid observations; absent closing events are censored, not physiological misses.',
              'incomplete_cases': 'Primary unobserved/partial cases retain unavailable phases and raw journals. No completed-window zero fallback, survivor-only aggregate or replacement cycles.',
              'interpretation': ['Binary/retained actor timing reports the algorithmic geometryFSM. Force20N crossings are a distinct raw physical-signal channel with only onset timing and explicit extras.',
                                 'Nominal events derive from healthy measured GRF. Virtual prosthetic hardware, empirical force basis, binary geometry and STL sole are different systems; disagreement does not uniquely diagnose preparation error or prove physiology.',
                                 'Swing impulse percentages come only from all3 complete fixed phasewindows in independently bound numerical metrics. Actual dynamic contact differs from filtered referenceFK.',
                                 'Runtime completion, retained FSM events and agreement within inherited pairing tolerances remain separate diagnostics; no clinical, hardware-necessity or subject-qualification gate.'],
              'provenance': {'initial_reader_schema_error_preserved': fp(HERE / 'attempt_01_receipt.json'),
                             'V2_change': 'Physical dt field corrected to captured effective_simulator_config.dt; original first attempted event consumption disclosed, inputs and scientific quantities unchanged'},
              'inputs_integrity': {'count': len(before), 'before': before, 'after': after, 'all_unchanged': True}}
    with output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'review': fp(output), 'case_count': 18, 'sources_unchanged': True}))


if __name__ == '__main__':
    main()
