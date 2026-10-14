"""Frozen descriptive phase timing from finalized saved18-case evidence only.

AST-reuse original paired nominal/compare_events, force crossings, validator
matcher, storage reader and repair-aware event journal. No OpenSim/Ray/model,
detector execution, integration, IK, inference, force fitting or native writes.
"""
from datetime import datetime, timezone
import argparse
import ast
import csv
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any, Iterable
import numpy as np

HERE = Path(__file__).resolve().parent
AUDIT = HERE.parent
ROOT = AUDIT.parents[1]
PROTOCOL = HERE / 'phase_timing_protocol.json'
PAIR = ROOT / 'Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07'
SOURCE_PATHS = {
    'paired': PAIR / 'compare_paired_saved.py',
    'reader': ROOT / 'Trajectory Generator/validation/AB07/analyze_prescribed_v26.py',
    'matcher': ROOT / 'validation/validate_online_grf_events.py',
    'crossing': ROOT / 'validation/validate_online_grf.py',
    'phase_journal': ROOT / 'Trajectory Generator/validation/AB07/v26c_best47_transfer_2026_10_06/compare_transfer.py',
    'tolerances': ROOT / 'results/ab07_v26_ground_registered_prescribed_9p01s_2026_10_06_r1/qualification_analysis.json',
    'selection': PAIR / 'selection.json',
    'dynamic_protocol': AUDIT / 'dynamic_protocol.json',
    'numeric_protocol': AUDIT / 'dynamics_analysis/analysis_protocol_v2.json',
    'this_helper': Path(__file__).resolve()}


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'),
                      parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))


def fp(path):
    path = Path(path).resolve()
    try:
        label = path.relative_to(ROOT).as_posix()
    except ValueError:
        label = path.as_posix()  # Existing explicitly frozen external OpenSim meshes.
    return {'path': label, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def write_new(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        json.dump(obj, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')


def check(manifest):
    current = {p: fp(ROOT / p)['sha256'] for p in manifest}
    if current != manifest:
        raise AssertionError('Protected source changed: ' + repr([p for p in manifest if current[p] != manifest[p]]))
    return current


def pure(source, name, namespace=None):
    tree = ast.parse(Path(source).read_text(encoding='utf-8'))
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name]
    if len(functions) != 1:
        raise ValueError('Require exactly one original pure function: ' + name)
    scope = {'np': np, 'Path': Path, 'Any': Any, 'Iterable': Iterable, 'csv': csv, 'math': math}
    scope.update(namespace or {})
    exec(compile(ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])), str(source), 'exec'), scope)
    return scope[name]


def helpers(protocol):
    sources = {k: ROOT / v['path'] for k, v in protocol['source_bindings'].items()}
    stats = pure(sources['paired'], 'metrics')
    strict = pure(sources['paired'], 'strict_statistics')
    strict.__globals__['strict_statistics'] = strict
    return {
        'nominal': pure(sources['paired'], 'nominal'),
        'compare': pure(sources['paired'], 'compare_events'),
        'trace_validation': pure(sources['paired'], 'validate_trace'),
        'require_grid': pure(sources['paired'], 'require_grid'),
        'strict': strict,
        'reader': pure(sources['reader'], 'read_storage'),
        'matcher': pure(sources['matcher'], 'match_events'),
        'crossing': pure(sources['crossing'], '_crossing_times'),
        'journal': pure(sources['phase_journal'], 'phase_analysis', {'metrics': stats})}


def freeze():
    if PROTOCOL.exists():
        raise FileExistsError('Preserve frozen phase protocol')
    selection, dynamic = read(SOURCE_PATHS['selection']), read(SOURCE_PATHS['dynamic_protocol'])
    inherited = read(SOURCE_PATHS['tolerances'])['methodology']
    if selection['criterion']['measured_GRF_event_threshold_n'] != 20. or selection['criterion']['primary_cycle_count_each_subject'] != 3:
        raise ValueError('Original selection threshold and cycles changed')
    if len(dynamic['cases']) != 18 or set(dynamic['cases']) != set(dynamic['execution_order']):
        raise ValueError('Require exact frozen18-case selection')
    manifest = dict(dynamic['protected_sources_before'])
    sources = {k: fp(p) for k, p in SOURCE_PATHS.items()}
    manifest.update({v['path']: v['sha256'] for v in sources.values()})
    check(manifest)
    obj = {'created_utc': datetime.now(timezone.utc).isoformat(), 'instruction_check_token': 'CMC_AGENT_OK_2026',
           'status': 'frozen_before_first_current_case_event_consumption',
           'timing_disclosure': 'Current physical outputs already exist or are running. This implementation is frozen before this consumer reads their trace event or force data; earlier separate numerical/FK audits are preserved.',
           'case_ids': dynamic['execution_order'], 'cases': dynamic['cases'], 'source_bindings': sources,
           'threshold_N': selection['criterion']['measured_GRF_event_threshold_n'],
           'confirmed_and_delivered_tolerances_s': inherited['v26_inherited_confirmed_and_delivered_tolerances_s'],
           'onset_holdout_tolerances_s': inherited['sensor_holdout_tolerances_s'],
           'primary_contract': 'Exactly own3frozen nominal cycles,4HSincludingclosing+3TO; first startup cycle separate, resetHS initialization retained as missing-reference distinction in secondary journal only',
           'event_contract': 'Original compare_events onset-based candidate buffer=max inherited onset tolerance, original greedy one-to-one match_events; force crossing has only onset semantics. Preserve event histories, HS repairs, retained actor view and every unmatched event.',
           'partial_contract': 'If no saved channel support extends into frozen primary window, primary channel unavailable, never count future physiological misses. Partials retain full reference and source matching as descriptive plus unmatched split by fully observed matching guard versus censored/unknown. No event/window replacement.',
           'support_contract': 'Channel support from actually saved force timestamps, binary sensor timestamps when present, otherwise recorded trace timestamps. Complete±matching-tolerance support required to classify an unmatched reference as observed diagnostic absence; late confirmation beyond saved horizon is censored.',
           'swing_impulse_contract': 'Read finalized independently bound April metrics for3exact nominal stance/swing windows and primary_all. Sum real-time impulses, no clamping or new force integration; withhold pooled fraction unless all3cycles and phasewindows complete.',
           'semantic_limits': ['Measured healthy-limb20N nominal events are production references, not ground truth of the virtual prosthetic limb.',
                               'Binary geometry plane/points, empirical force basis/plane and renderedSTL sole are different constructions; timing disagreement does not uniquely identify registration error.',
                               'Actor counters and retained cycle events are algorithmic diagnostics, not independent physiological cycles.',
                               'All inherited tolerances are diagnostic pairing references only; no new acceptance, hardware, subject or gait gate.'],
           'new_Claude_branch_read': False, 'new_native_or_FK': False,
           'protected_count': len(manifest), 'protected_sources_before': manifest}
    write_new(PROTOCOL, obj)
    print(json.dumps({'protocol': fp(PROTOCOL), 'protected_count': len(manifest), 'cases': len(dynamic['cases'])}), flush=True)


def trace(path):
    rows = [json.loads(line, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            for line in path.read_text(encoding='utf-8').splitlines() if line.strip()]
    return rows


def assert_events(events):
    for event in events:
        for key in ('event_time_s', 'confirmed_time_s', 'delivered_time_s'):
            if key in event and not math.isfinite(float(event[key])):
                raise ValueError('Nonfinite saved event clock')
        if event.get('event') in {'heel_strike', 'toe_off'} and not all(k in event for k in ('event_time_s', 'confirmed_time_s', 'delivered_time_s')):
            raise ValueError('Saved ordinary event lacks original3clock schema')


def compare_channel(events, reference, window, support, protocol, h, label, scope='primary'):
    result = {'source_channel': label, 'scope': scope, 'reference_events_s': reference, 'nominal_analysis_window_s': window,
              'saved_support_s': support, 'new_gate_used': False}
    prefix = 'primary' if scope == 'primary' else 'startup'
    if support is None or support[1] <= window[0] or support[0] >= window[1]:
        result.update(status=f'unavailable_{prefix}_not_reached_or_not_recorded', comparison=None)
        return result
    result['status'] = f'fully_observed_{prefix}_window' if support[0] <= window[0] and support[1] >= window[1] else f'partially_observed_{prefix}_window'
    compared = h['compare'](events, reference, window, h['matcher'], protocol['confirmed_and_delivered_tolerances_s'], protocol['onset_holdout_tolerances_s'])
    for kind, entry in compared.items():
        if label == 'primary_force_20N_crossings':
            entry['channels'] = {'onset': entry['channels']['onset']}
            entry['note'] = 'Raw force crossings have no binary confirmation/delivery clocks; only interpolated force onset.'
        for channel in entry['channels'].values():
            match, tolerance = channel['matching'], channel['tolerance_s']
            full_guard = [t for t in reference[kind] if support[0] <= t-tolerance and support[1] >= t+tolerance]
            unmatched = match['unmatched_reference']
            channel['unmatched_reference_fully_observed_matching_guard_s'] = [t for t in unmatched if t in full_guard]
            channel['unmatched_reference_censored_or_unknown_s'] = [t for t in unmatched if t not in full_guard]
            channel['fully_observed_reference_matching_guard_s'] = full_guard
            channel['interpretation'] = 'Original full-reference matcher counts are descriptive. References without full saved±tolerance support remain censored/unknown, not physiological misses; matched observed events remain reported.'
    result['comparison'] = compared
    return result


def swing_fraction(metrics_path, receipt_path, name, selected, protocol):
    if not metrics_path.exists():
        return {'status': 'finalized_numerical_metrics_not_available', 'pooled': None}
    data = read(metrics_path)
    if data['case_id'] != name or data['case'] != protocol['cases'][name] or data['receipt']['sha256'] != fp(receipt_path)['sha256']:
        raise ValueError('Numerical metrics/receipt/case binding differs')
    if data['dynamic_protocol']['sha256'] != protocol['source_bindings']['dynamic_protocol']['sha256'] or data['analysis_protocol']['sha256'] != protocol['source_bindings']['numeric_protocol']['sha256'] or not data['run_artifacts_unchanged']['passed']:
        raise ValueError('Numerical metric protocol or native-integrity binding differs')
    windows, rows = data['windows'], []
    for i, c in enumerate(selected['primary_cycles'], 1):
        segments = [('stance', c['hs_s'], c['to_s']), ('swing', c['to_s'], c['next_hs_s'])]
        row = {'cycle': i, 'frozen_cycle': c}
        for label, start, end in segments:
            w = windows.get(f'cycle_{i}_{label}')
            if not w or w['coverage'] != 'complete':
                row[label] = {'status': 'unavailable_or_partial', 'coverage': w.get('coverage') if w else None, 'impulse_Ns': None}
            else:
                if w['requested_range_s'] != [start, end] or w['observed_intersection_range_s'] != [start, end] or w['boundary_clamping_or_extrapolation_used']:
                    raise ValueError('Numerical nominal phasewindow differs or clamps')
                value = float(w['forces']['left_online']['impulse_Ns'])
                if not math.isfinite(value):
                    raise ValueError('Nonfinite finalized phase impulse')
                row[label] = {'status': 'complete', 'impulse_Ns': value, 'duration_s': end-start}
        complete = all(row[k]['status'] == 'complete' for k in ('stance', 'swing'))
        total = sum(row[k]['impulse_Ns'] for k in ('stance', 'swing')) if complete else None
        row['swing_fraction'] = row['swing']['impulse_Ns']/total if complete and total != 0 else None
        rows.append(row)
    pooled = None
    primary = windows.get('primary_all')
    if primary and primary['coverage'] == 'complete' and all(r['stance']['status'] == r['swing']['status'] == 'complete' for r in rows):
        expected = [selected['primary_cycles'][0]['hs_s'], selected['primary_cycles'][-1]['next_hs_s']]
        if primary['requested_range_s'] != expected or primary['observed_intersection_range_s'] != expected or primary['boundary_clamping_or_extrapolation_used']:
            raise ValueError('Numerical pooled primarywindow differs or clamps')
        stance, swing = sum(r['stance']['impulse_Ns'] for r in rows), sum(r['swing']['impulse_Ns'] for r in rows)
        total = float(primary['forces']['left_online']['impulse_Ns'])
        pooled = {'stance_impulse_Ns': stance, 'swing_impulse_Ns': swing, 'primary_impulse_Ns': total,
                  'phase_sum_minus_primary_Ns': stance+swing-total,
                  'swing_fraction_of_primary_impulse': swing/total if total != 0 else None,
                  'definition': 'Sum actual force impulses from three exact nominal swing windows / same primary_all impulse, inherited saved-data quadrature; no clamping or recomputation'}
    return {'status': 'complete_three_cycle_fraction' if pooled else 'pooled_withheld_incomplete_primary',
            'source_metrics': fp(metrics_path), 'cycles': rows, 'pooled': pooled}


def analyze(name, protocol, h):
    output = HERE / name / 'phase_metrics.json'
    if output.exists():
        raise FileExistsError('Preserve previous case phase analysis')
    case = protocol['cases'][name]
    run = AUDIT / 'dynamics' / name / 'run'
    receipt_path = run / 'receipt.json'
    receipt = read(receipt_path)
    if not receipt.get('finished_utc') or receipt.get('status') not in {'completed', 'interrupted', 'failed'}:
        raise ValueError('Case receipt is not final; no events read')
    metrics_path = AUDIT / 'dynamics_analysis' / name / 'metrics.json'
    if not metrics_path.is_file():
        raise ValueError('Wait for finalized independently bound numerical metrics before consuming events')
    numerical_source_before = fp(metrics_path)
    if receipt.get('case') != case:
        # Runtime receipt stores case before prepared_receipt field was appended
        # to the dynamic manifest; verify its actual shared scientific fields.
        actual = receipt.get('case', {})
        if actual != {k: v for k, v in case.items() if k != 'prepared_receipt'}:
            raise ValueError('Final receipt case differs from frozen scientific cell')
    inventory = {fp(p)['path']: fp(p)['sha256'] for p in sorted(run.rglob('*')) if p.is_file()}
    selection = read(ROOT / protocol['source_bindings']['selection']['path'])['subjects'][case['subject']]
    primary = selection['primary_cycles']
    nominal = h['nominal'](primary)
    window = [primary[0]['hs_s'], primary[-1]['next_hs_s']]
    trace_path = run / 'trace.jsonl'
    rows = trace(trace_path) if trace_path.exists() else []
    h['trace_validation'](rows, name)
    transitions = [event for row in rows for event in row.get('phase_fsm', {}).get('accepted_transitions_this_step', [])]
    binary = [event for row in rows for event in row.get('binary_phase_fsm', {}).get('events_this_step', [])]
    assert_events(transitions + binary)
    cancellations = [e for e in transitions if e.get('event') == 'heel_strike_cancelled']
    retained = [e for e in transitions if e.get('event') in {'heel_strike', 'toe_off'} and
                not (e['event'] == 'heel_strike' and any(abs(float(e['event_time_s'])-float(c['event_time_s'])) <= 1e-8 for c in cancellations))]
    trace_support = [float(rows[0]['time']), float(rows[-1]['time'])] if rows else None
    binary_recorded = bool(rows) and all('binary_phase_fsm' in r for r in rows)
    actor_recorded = bool(rows) and all('phase_fsm' in r for r in rows)
    samples = [s for row in rows for s in row.get('binary_phase_sensor_samples', [])]
    sample_times = np.asarray([s['time_s'] for s in samples], dtype=float)
    if len(sample_times) and (not np.isfinite(sample_times).all() or np.any(np.diff(sample_times) < 0)):
        raise ValueError('Binary sample saved clocks invalid')
    binary_support = [float(sample_times[0]), float(sample_times[-1])] if len(sample_times) else trace_support
    channels = {'binary_geometry_FSM': compare_channel(binary, nominal, window, binary_support if binary_recorded else None, protocol, h, 'binary_geometry_FSM'),
                'actor_retained_after_HS_repairs': compare_channel(retained, nominal, window, trace_support if actor_recorded else None, protocol, h, 'actor_retained_after_HS_repairs')}
    force_paths = list((run / 'sim_outputs').glob('*_online_grf.sto'))
    force_events, force_support, force_source = [], None, None
    if len(force_paths) > 1:
        raise ValueError('Ambiguous actual online force output')
    if force_paths:
        force_path = force_paths[0]
        # The original reader rejects empty exported tables. An empty table is
        # unavailable rather than reconstructed as a zero-force trace.
        try:
            times, names, values, _ = h['reader'](force_path)
        except ValueError as exc:
            if 'absent or non-finite' not in str(exc):
                raise
            lines = force_path.read_text().splitlines()
            header = next((i for i, line in enumerate(lines) if line.strip() and line.split()[0].lower() == 'time'), None)
            if header is None or any(line.strip() for line in lines[header+1:]):
                raise
        else:
            if len(times) > 1:
                h['require_grid'](times, str(force_path))
            if len(names) != len(set(names)) or 'left_force_y' not in names:
                raise ValueError('Actual force column membership invalid')
            dt = float(receipt['effective_env_config']['integration_dt'])
            if not np.allclose(np.diff(times), dt, rtol=0, atol=1e-8):
                raise ValueError('Saved force acquisition differs from original native1ms')
            force_support = [float(times[0]), float(times[-1])]
            force_source = fp(force_path)
            fy = values[:, names.index('left_force_y')]
            force_events = [{'event': kind, 'event_time_s': float(t), 'confirmed_time_s': float(t), 'delivered_time_s': float(t),
                             'source': 'Original raw20Nforce crossing; no confirmation or delivery semantics'}
                            for kind in ('heel_strike', 'toe_off')
                            for t in h['crossing'](times, fy, protocol['threshold_N'], rising=kind == 'heel_strike')]
    channels['primary_force_20N_crossings'] = compare_channel(force_events, nominal, window, force_support, protocol, h, 'primary_force_20N_crossings')
    primary_reached = any(channel['comparison'] is not None for channel in channels.values())
    secondary_nominal = h['nominal']([selection['startup_cycle']]+primary) if primary_reached else h['nominal']([selection['startup_cycle']])
    secondary = h['journal'](rows, secondary_nominal, protocol['confirmed_and_delivered_tolerances_s'],
                              {'sensor_holdout_heel_strike_max_error_s': protocol['onset_holdout_tolerances_s']['heel_strike'],
                               'sensor_holdout_toe_off_max_error_s': protocol['onset_holdout_tolerances_s']['toe_off']},
                              h['matcher'], selection['reset_hs_s'])
    secondary['interpretation'] += ' Secondary observed journal only: before any primary support its reference is startup only. Startup resetHS is initialization; unmatched references beyond saved support are unknown and never physiological failure counts.'
    secondary['reference_scope'] = 'startup_plus_fixed_primary' if primary_reached else 'startup_only_primary_not_reached'
    startup = selection['startup_cycle']
    startup_nominal, startup_window = h['nominal']([startup]), [startup['hs_s'], startup['next_hs_s']]
    startup_channels = {label: compare_channel(events, startup_nominal, startup_window, support, protocol, h, label, scope='startup_secondary')
                        for label, events, support in [('binary_geometry_FSM', binary, binary_support if binary_recorded else None),
                                                      ('actor_retained_after_HS_repairs', retained, trace_support if actor_recorded else None),
                                                      ('primary_force_20N_crossings', force_events, force_support)]}
    impulse = swing_fraction(metrics_path, receipt_path, name, selection, protocol)
    after = {fp(p)['path']: fp(p)['sha256'] for p in sorted(run.rglob('*')) if p.is_file()}
    if inventory != after:
        raise AssertionError('Native outputs changed during phase consumption')
    if fp(metrics_path) != numerical_source_before:
        raise AssertionError('Finalized numerical metric source changed during phase consumption')
    check(protocol['protected_sources_before'])
    result = {'created_utc': datetime.now(timezone.utc).isoformat(), 'instruction_check_token': 'CMC_AGENT_OK_2026',
              'status': 'finalized_descriptive_phase_diagnostics', 'case_id': name, 'case': case, 'subject': case['subject'],
              'phase_protocol': fp(PROTOCOL), 'dynamic_protocol': protocol['source_bindings']['dynamic_protocol'],
              'receipt': fp(receipt_path), 'runtime_status': receipt['status'], 'close_error': receipt.get('close_error'),
              'primary_reference': {'cycles': primary, 'events_s': nominal, 'window_s': window, 'threshold_N': protocol['threshold_N']},
              'primary_channels': channels, 'startup_channels_secondary': startup_channels,
              'observed_full_episode_journal_secondary': secondary, 'all_saved_raw_force_crossings': force_events,
              'raw_force_source': force_source, 'observed_trace_support_s': trace_support,
              'swing_impulse_fraction': impulse, 'new_Claude_branch_read': False, 'new_native_or_FK': False,
              'limitations': protocol['semantic_limits'], 'run_artifacts_unchanged': {'count': len(inventory), 'before': inventory, 'after': after, 'passed': True}}
    write_new(output, h['strict'](result))
    print(json.dumps({'case': name, 'phase_metrics': fp(output), 'runtime_status': receipt['status'],
                      'primary_channel_status': {k: v['status'] for k, v in channels.items()}}), flush=True)
    return output


def aggregate(protocol):
    output = HERE / 'aggregate_phase_metrics.json'
    if output.exists():
        raise FileExistsError('Preserve previous18-case phase aggregate')
    cases = {}
    for name in protocol['case_ids']:
        path = HERE / name / 'phase_metrics.json'
        data = read(path)
        if data['phase_protocol'] != fp(PROTOCOL) or data['receipt'] != fp(AUDIT / 'dynamics' / name / 'run/receipt.json') or data['case'] != protocol['cases'][name]:
            raise ValueError('Phase case source binding changed')
        current = {fp(p)['path']: fp(p)['sha256'] for p in sorted((AUDIT / 'dynamics' / name / 'run').rglob('*')) if p.is_file()}
        if current != data['run_artifacts_unchanged']['after']:
            raise ValueError('Native artifacts changed after their phase diagnostics')
        numerical = data['swing_impulse_fraction'].get('source_metrics')
        if numerical and numerical != fp(ROOT / numerical['path']):
            raise ValueError('Numerical impulse source changed after phase consumption')
        cases[name] = {'metrics': fp(path), 'runtime_status': data['runtime_status'], 'primary_channels': data['primary_channels'],
                       'swing_impulse_fraction': data['swing_impulse_fraction'], 'case': data['case']}
    check(protocol['protected_sources_before'])
    write_new(output, {'created_utc': datetime.now(timezone.utc).isoformat(), 'status': 'all18_finalized_descriptive_phase_diagnostics',
                       'phase_protocol': fp(PROTOCOL), 'dynamic_protocol': protocol['source_bindings']['dynamic_protocol'],
                       'cases': cases, 'clinical_gate_or_forcefit_used': False, 'new_Claude_branch_read': False,
                       'limitations': protocol['semantic_limits'], 'protected_sources_unchanged': True})
    print(json.dumps({'aggregate': fp(output), 'cases': len(cases)}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--freeze', action='store_true')
    group.add_argument('--case')
    group.add_argument('--watch', action='store_true')
    group.add_argument('--aggregate-only', action='store_true')
    parser.add_argument('--peer-reviewed', action='store_true')
    args = parser.parse_args()
    if args.freeze:
        freeze()
        return
    protocol = read(PROTOCOL)
    check(protocol['protected_sources_before'])
    if args.aggregate_only:
        aggregate(protocol)
        return
    if not args.peer_reviewed:
        parser.error('Peer review PASS required before first event consumption')
    h = helpers(protocol)
    if args.case:
        if args.case not in protocol['cases']:
            parser.error('Case is not in frozen18case selection')
        analyze(args.case, protocol, h)
        return
    while True:
        unfinished = []
        for name in protocol['case_ids']:
            output = HERE / name / 'phase_metrics.json'
            if output.exists():
                continue
            receipt_path = AUDIT / 'dynamics' / name / 'run/receipt.json'
            numerical = AUDIT / 'dynamics_analysis' / name / 'metrics.json'
            if not receipt_path.exists() or not numerical.exists():
                unfinished.append(name)
                continue
            try:
                receipt = read(receipt_path)
                data = read(numerical)
            except json.JSONDecodeError:
                unfinished.append(name)
                continue
            if not receipt.get('finished_utc') or receipt.get('status') not in {'completed', 'interrupted', 'failed'}:
                unfinished.append(name)
                continue
            if data.get('receipt', {}).get('sha256') != fp(receipt_path)['sha256']:
                raise ValueError('Final numerical metrics not bound to final receipt')
            analyze(name, protocol, h)
        if not unfinished:
            aggregate(protocol)
            return
        print(json.dumps({'waiting_final_cases_or_bound_metrics': unfinished}), flush=True)
        time.sleep(15)


if __name__ == '__main__':
    main()
