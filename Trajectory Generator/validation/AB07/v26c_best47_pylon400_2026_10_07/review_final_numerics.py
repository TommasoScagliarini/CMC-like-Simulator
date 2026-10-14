"""Independent saved-data peer check; no OpenSim, policy, IK or integration."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import math
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def read(p):
    return json.loads(p.read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))


def path(raw):
    p = Path(raw)
    return p if p.is_absolute() else ROOT / p


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def fp(p):
    return {'path': p.relative_to(ROOT).as_posix(), 'sha256': sha(p), 'size_bytes': p.stat().st_size}


def check_fp(item):
    p = path(item['path'])
    assert sha(p) == item['sha256'], item['path']
    if 'size_bytes' in item:
        assert p.stat().st_size == item['size_bytes'], item['path']


def storage(p):
    lines = p.read_text().splitlines()
    end = next(i for i, x in enumerate(lines) if x.strip().lower() == 'endheader')
    names = lines[end+1].split()
    data = np.loadtxt(lines[end+2:])
    assert len(names) == len(set(names)) and data.ndim == 2
    assert data.shape[1] == len(names) and np.isfinite(data).all()
    assert np.all(np.diff(data[:, 0]) > 0)
    return data[:, 0], {n: data[:, i] for i, n in enumerate(names[1:], 1)}


def weighted(x, t):
    duration = t[-1] - t[0]
    return dict(mean=float(np.trapz(x, t)/duration), mean_abs=float(np.trapz(abs(x), t)/duration),
                rms=float(np.sqrt(np.trapz(x*x, t)/duration)), min=float(x.min()),
                max=float(x.max()), max_abs=float(abs(x).max()), quadrature_points=len(t))


def integrals(x, t):
    return dict(signed=float(np.trapz(x, t)), absolute=float(np.trapz(abs(x), t)),
                positive=float(np.trapz(np.maximum(x, 0), t)), negative=float(np.trapz(np.minimum(x, 0), t)))


ERRORS = []


def compare(actual, expected, label):
    for key, value in actual.items():
        assert math.isfinite(value)
        target = expected[key]
        error = abs(value-target)
        ERRORS.append(error)
        assert math.isclose(value, target, rel_tol=1e-12, abs_tol=1e-10), (label, key, value, target)


def main():
    protocol = read(HERE/'analysis_protocol.json')
    execution = read(HERE/'execution_result.json')
    metric = read(HERE/'analysis/metrics.json')
    phase = read(HERE/'analysis/phase_metrics.json')
    run = ROOT/protocol['run_directory']
    plotdir = ROOT/protocol['plots_contract']['root']
    plot = read(plotdir/'plot_rendering_manifest.json')
    visual = read(plotdir/'visual_review.json') if (plotdir/'visual_review.json').exists() else None
    sources = [HERE/'analysis_protocol.json', HERE/'execution_result.json', HERE/'implementation_binding.json',
               HERE/'analysis/metrics.json', HERE/'analysis/phase_metrics.json', HERE/'root_primary_reserve_check.json',
               plotdir/'plot_rendering_manifest.json', Path(__file__)]
    if visual is not None:
        sources.append(plotdir/'visual_review.json')
    source_before = [fp(p) for p in sources]
    for relative, expected in protocol['protected_sources_before'].items():
        assert sha(ROOT/relative) == expected, relative
    for item in protocol['source_bindings'].values():
        check_fp(item)
    for key in ('receipt', 'analysis_protocol', 'implementation_binding', 'execution_result'):
        check_fp(metric[key])
    for key in ('receipt', 'analysis_protocol', 'implementation_binding', 'numerical_source'):
        check_fp(phase[key])
    assert phase['numerical_source'] == fp(HERE/'analysis/metrics.json')
    for item in metric['source_files'].values():
        check_fp(item)
    assert execution['returncode'] == 0 and execution['native_attempts'] == 1
    assert metric['runtime']['completed_requested_duration'] and metric['runtime']['steps'] == 611
    assert metric['analysis_status'] == 'complete_primary_metrics'
    assert len(metric['feature_order']['actor']) == 35 and len(metric['feature_order']['full']) == 84
    assert metric['feature_order']['matches_historical']
    native_before = {k: v['sha256'] for k, v in metric['run_artifacts_unchanged']['files'].items()}
    assert len(native_before) == 26
    for relative, expected in native_before.items():
        assert sha(ROOT/relative) == expected, relative
    for relative, item in execution['native_file_inventory'].items():
        assert sha(ROOT/relative) == item['sha256'], relative
    tables = {}
    for name, info in metric['tables'].items():
        p = ROOT/info['file']
        assert sha(p) == info['sha256']
        t, columns = storage(p)
        assert len(t) == 6110 and np.allclose(np.diff(t), .001, atol=1e-11, rtol=0)
        tables[name] = (t, columns)
    assert len(tables) == 17
    native_t = tables['kinematics'][0]
    assert all(np.array_equal(t[0], native_t) for t in tables.values())
    selection = read(path(protocol['source_bindings']['selection']['path']))['subjects']['AB07']
    knots = sorted({x[k] for x in selection['primary_cycles'] for k in ('hs_s', 'to_s', 'next_hs_s')})
    assert phase['primary_reference']['cycles'] == selection['primary_cycles']
    assert metric['actual_COM']['requested_exact_knots_s'] == knots
    assert metric['actual_COM']['missing_exact_knots_s'] == []
    assert metric['new_FK_poses'] == 8 and metric['new_integrations'] == metric['new_IK_solves'] == 0
    cfg = read(path(metric['receipt']['path']))['effective_simulator_config']
    translation = set(cfg['translation_coords'])
    checks = 0
    grids = {}
    for name, window in metric['windows'].items():
        assert window['coverage'] == 'complete' and not window['boundary_clamping_or_extrapolation_used']
        start, end = window['requested_range_s']
        assert window['observed_intersection_range_s'] == [start, end]
        assert native_t[0] <= start < end <= native_t[-1]
        grid = np.unique(np.r_[start, native_t[(native_t > start)&(native_t < end)],
                               [t for t in knots if start < t < end], end])
        grids[name] = len(grid)
        interp = lambda table, col: np.interp(grid, tables[table][0], tables[table][1][col])
        for coord, entry in window['reserves']['per_coordinate'].items():
            force = interp('reserve_torques', coord+'_reserve_torque')
            speed = interp('states', coord+'_qdot')
            assert entry['unit'] == ('N' if coord in translation else 'Nm')
            assert entry['impulse_unit'] == ('Ns' if coord in translation else 'Nm*s')
            assert entry['speed']['unit'] == ('m/s' if coord in translation else 'rad/s')
            assert entry['power_W']['unit'] == 'W'
            compare(weighted(force, grid), entry['force_or_torque'], name+coord+'force')
            compare(integrals(force, grid), entry['impulse'], name+coord+'impulse')
            compare(weighted(speed, grid), entry['speed'], name+coord+'speed')
            work = integrals(force*speed, grid)
            compare(work, entry['work_J'], name+coord+'work')
            compare({k:v/metric['mass_kg'] for k,v in work.items()}, entry['work_J_per_kg'], name+coord+'Jkg')
            compare(weighted(force*speed, grid), entry['power_W'], name+coord+'power')
            for obj in (entry['impulse'], entry['work_J']):
                assert obj['positive'] >= 0 and obj['negative'] <= 0
                assert math.isclose(obj['signed'], obj['positive']+obj['negative'], abs_tol=1e-10)
                assert math.isclose(obj['absolute'], obj['positive']-obj['negative'], abs_tol=1e-10)
            checks += 1
        force = interp('online_grf', 'left_force_y')
        compare(weighted(force, grid), window['forces']['left_online'], name+'left_force')
        compare({'impulse_Ns':float(np.trapz(force, grid))}, window['forces']['left_online'], name+'left_impulse')
        for coord, entry in window['tracking']['coordinates'].items():
            q = interp('kinematics', coord)
            qref = interp('kinematics_reference', coord+'_q_ref')
            speed = interp('states', coord+'_qdot')
            vref = interp('kinematics_reference', coord+'_qdot_ref')
            factor = 1. if coord in translation else 180/np.pi
            compare(weighted((q-qref)*factor, grid), entry['actual_minus_served']['position'], name+coord+'trackingq')
            compare(weighted((speed-vref)*factor, grid), entry['actual_minus_served']['speed'], name+coord+'trackingv')
    rows = [json.loads(line) for line in (run/'runtime_diagnostics.jsonl').read_text().splitlines() if line.strip()]
    assert len(rows) == 611
    actor = [e for row in rows for e in row.get('phase_fsm', {}).get('accepted_transitions_this_step', [])]
    binary = [e for row in rows for e in row.get('binary_phase_fsm', {}).get('events_this_step', [])]
    canceled = [e for e in actor if e.get('event') == 'heel_strike_cancelled']
    assert not canceled and phase['observed_full_episode_journal_secondary']['heel_strike_cancellations'] == []
    counts = {label:{kind:sum(e.get('event')==kind for e in journal) for kind in ('heel_strike','toe_off')}
              for label,journal in [('actor',actor),('binary',binary)]}
    tolerances = protocol['phase_contract']
    assert tolerances['threshold_N'] == 20.
    assert tolerances['confirmed_and_delivered_tolerances_s'] == {'HS_TOLERANCE_S':.05,'TO_TOLERANCE_S':.08,'DELIVERED_HS_TOLERANCE_S':.06,'DELIVERED_TO_TOLERANCE_S':.09}
    timing = {}
    for label, channel in phase['primary_channels'].items():
        assert channel['status'] == 'fully_observed_primary_window'
        lo, hi = channel['saved_support_s']
        timing[label] = {}
        for kind, entry in channel['comparison'].items():
            timing[label][kind] = {}
            for clock, result in entry['channels'].items():
                tol = result['tolerance_s']
                expected = tolerances['onset_holdout_tolerances_s'][kind] if clock=='onset' else tolerances['confirmed_and_delivered_tolerances_s'][('DELIVERED_' if clock=='delivered' else '')+('HS' if kind=='heel_strike' else 'TO')+'_TOLERANCE_S']
                assert tol == expected
                matching = result['matching']
                assert matching['matched_count'] == len(matching['pairs'])
                for pair in matching['pairs']:
                    assert math.isclose(pair['error_s'], pair['predicted_time_s']-pair['reference_time_s'], abs_tol=1e-12)
                    assert abs(pair['error_s']) <= tol
                for guard in result['every_reference_matching_guard']:
                    t = guard['reference_time_s']
                    assert guard['complete'] == (lo <= t-tol and hi >= t+tol)
                timing[label][kind][clock] = {'matched_count':matching['matched_count'], 'reference_count':matching['reference_count'], 'tolerance_s':tol,
                    'unmatched_fully_observed':result['unmatched_reference_fully_observed_matching_guard_s'],
                    'unmatched_censored':result['unmatched_reference_censored_or_unknown_s'], 'all_reference_guards':result['every_reference_matching_guard']}
    for kind, binary_entry in phase['primary_channels']['binary_geometry_FSM']['comparison'].items():
        actor_entry = phase['primary_channels']['actor_retained_after_HS_repairs']['comparison'][kind]
        for clock, binary_result in binary_entry['channels'].items():
            assert binary_result['matching'] == actor_entry['channels'][clock]['matching']
    pooled = phase['swing_impulse_fraction']['pooled']
    assert pooled and math.isclose(pooled['stance_impulse_Ns']+pooled['swing_impulse_Ns'], pooled['primary_impulse_Ns'], abs_tol=1e-10)
    assert pooled['primary_impulse_Ns'] == metric['windows']['primary_all']['forces']['left_online']['impulse_Ns']
    sea = {}
    for name, entry in metric['SEA_all_recorded'].items():
        assert entry['all_identity_checks_pass']
        coord = entry['coordinate']
        raw = tables['sea_diagnostics'][1][name+'_tau_cmd_raw']/entry['model_properties']['F_opt']
        u = tables['sea_controls'][1][coord]
        assert not np.any(abs(raw)>1+1e-8) and not np.any(abs(u)>1)
        assert entry['motor_torque_clamp']['samples'] == 0
        assert entry['normalized_control_saturation']['samples'] == 0
        sea[name] = {'SEA_identities_pass':True,'u_unclipped_exceeds1_samples':0,'applied_u_max_abs':float(abs(u).max()),
                     'motor_clamp500_samples':0,'spring_max_abs_Nm':entry['tau_spring_Nm']['max_abs'],
                     'cascade_integral_flag_samples':entry['command_limit_evidence']['cascade_i_clamped']['samples']}
    assert metric['policy_trace']['clipped_policy_steps'] == 0
    assert metric['runtime_diagnostics']['completed_binary_sensor_grids_all_match']
    assert metric['policy_trace']['covers_requested_segment_end_times']
    for item in plot['files'].values():
        check_fp(item)
    assert len(plot['files']) == 7 and len(list(plotdir.glob('*.png'))) == 7
    for key in ('receipt', 'metrics', 'analysis_protocol', 'implementation_binding', 'root_plotter'):
        check_fp(plot[key])
    assert [fp(p) for p in sources] == source_before
    for relative, expected in native_before.items():
        assert sha(ROOT/relative) == expected
    # The visual receipt schema is validated explicitly in the final review,
    # rather than inferring PASS from seven files existing.
    out = dict(created_utc=datetime.now(timezone.utc).isoformat(), status='PASS_INDEPENDENT_SAVED_DATA_ARITHMETIC',
               review_sources=source_before, protected_sources_count=len(protocol['protected_sources_before']),
               protected_sources_current_match=True, finalized_run_files_unchanged_count=len(native_before),
               native_tables=17, native_rows_each=6110, reserve_coordinate_windows_recomputed=checks,
               scalar_comparisons=len(ERRORS), max_abs_reconstruction_error=max(ERRORS), quadrature_points=grids,
               actor_and_binary_accepted_counts=counts, canceled_HS=0, timing_review=timing, SEA=sea,
               learned_raw_action_clip_samples=0, swing_fraction_review=pooled,
               visual_receipt_for_explicit_review=visual, plots_count=7,
               new_native_FK_IK_inference_or_detector_evaluation=False,
               limitations=['Arithmetic reconstruction checks saved outputs only; no continuous/RK-stage extrema claim.',
                            'Inherited diagnostic pairing tolerances do not certify physiological correctness.',
                            'Full episode startup TO, historical accepted events and retained primary remain distinct.',
                            'Reference IK is not the learned served target; actual-minus-served verified separately.',
                            'Prescribed comparison changes action, slew, clock and FSM/reward contracts; policy weights alone are not isolated.'])
    output = HERE/'independent_saved_data_arithmetic_review.json'
    with output.open('x') as f:
        json.dump(out,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'review':fp(output),'max_error':max(ERRORS),'scalar_comparisons':len(ERRORS),'coordinate_windows':checks}))


if __name__ == '__main__':
    main()
