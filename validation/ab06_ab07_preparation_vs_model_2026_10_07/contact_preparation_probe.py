"""One ankle-only FK probe of archived pre-ground IK on current AB07 geometry.

This is a local diagnostic pose substitution, not a dynamically consistent
model, a recalibration, or a valid alternate subject bundle. No integration,
IK solving, input/model serialization or change of operational settings.
"""
from __future__ import annotations
from datetime import datetime, timezone
import json
from pathlib import Path
import numpy as np
import contact_replay as cr


ORIGINAL = cr.AUDIT/'contact_replay.json'
OLD_IK = cr.REPO/'validation/ab07_detector_investigation_2026_10_06/stage3/baseline/models/AB07_SEASEA_Threadmill/data/IK_results_AB07_SEASEA.mot'
EXPECTED_OLD_SHA = '6803f2fd224890e5ef166c26d8f70203522599be40e6e0659bed1174dcea2583'


def weighted_summary(time, values):
    duration = time[-1]-time[0]
    return {'time_weighted_mean': float(cr.integrate(values, time)/duration),
            'time_weighted_RMS': float(np.sqrt(cr.integrate(values**2, time)/duration)),
            'minimum': float(np.min(values)), 'maximum': float(np.max(values))}


def main():
    original = json.loads(ORIGINAL.read_text())
    if original['status'] != 'PASS' or cr.fp(OLD_IK)['sha256'] != EXPECTED_OLD_SHA:
        raise AssertionError('Original replay or archived pre-ground identity differs.')
    sources = {cr.REPO/path for path in original['inputs_integrity']['fingerprints_after']}
    sources.update([ORIGINAL, OLD_IK, Path(__file__)])
    sources = sorted(p.resolve() for p in sources)
    before = {cr.fp(p)['path']: cr.fp(p)['sha256'] for p in sources}
    for path, old_sha in original['inputs_integrity']['fingerprints_after'].items():
        if before[path] != old_sha:
            raise AssertionError(f'Original replay protected source changed: {path}')
    selection = json.loads((cr.PAIR/'selection.json').read_text())['subjects']['AB07']
    cycles = selection['primary_cycles']
    receipt_path = cr.REPO/'results/AB07_paired_prescribed_HS_9s_2026_10_07/receipt.json'
    receipt = json.loads(receipt_path.read_text())
    model_path = cr.REPO/selection['source_fingerprints']['model']['path']
    current_ik = cr.REPO/selection['source_fingerprints']['IK']['path']
    state_path = receipt_path.parent/'sim_outputs/sim_output_states.sto'
    native_t, _, _ = cr.read_storage(state_path)
    start, end = cycles[0]['hs_s'], cycles[-1]['next_hs_s']
    interior = native_t[(native_t > start) & (native_t < end)]
    boundaries = [c[k] for c in cycles for k in ('hs_s', 'to_s', 'next_hs_s')]
    times = np.unique(np.r_[interior, boundaries])
    cfg_dict = dict(receipt['effective_simulator_config'])
    cfg_dict.update(model_bundle_dir=str(model_path.parent), model_file=str(model_path),
                    kinematics_file=str(current_ik), external_loads_xml=str(cr.REPO/selection['source_fingerprints']['external_loads']['path']))
    current_cfg = cr.SimulatorConfig(**cfg_dict)
    old_cfg_dict = dict(cfg_dict, kinematics_file=str(OLD_IK))
    old_cfg = cr.SimulatorConfig(**old_cfg_dict)
    # Two full-source production preprocessors; no cropping before filtering.
    current_interp = cr.KinematicsInterpolator(current_cfg)
    old_interp = cr.KinematicsInterpolator(old_cfg)
    current_time, _, current_header = cr.read_storage(current_ik)
    old_time, _, old_header = cr.read_storage(OLD_IK)
    if not np.array_equal(current_time, old_time) or 'indegrees=yes' not in old_header.lower():
        raise AssertionError('Archived source clocks/units differ from expected lineage.')
    if times[0] < max(current_interp.time_bounds[0], old_interp.time_bounds[0]) or times[-1] > min(current_interp.time_bounds[1], old_interp.time_bounds[1]):
        raise AssertionError('Probe exceeds source support.')
    cr._load_plugin(str(cr.REPO/'plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff'))
    cr._load_plugin(str(cr.REPO/'plugins/OnlineGRFContact'))
    cr._configure_geometry_search_paths(str(model_path))
    model = cr.opensim.Model(str(model_path))
    if model.getConstraintSet().getSize():
        raise ValueError('Explicit FK probe requires no model constraints.')
    raw = json.loads(cr.PRIMARY.read_text())
    left = dict(raw, spheres=[s for s in raw['spheres'] if s['side'] == 'left'])
    profile = cr.OnlineGRFProfile.from_mapping(left, required_sides=('left',))
    cr.add_online_grf_forces(model, profile, applies_force=False, name_prefix='contact_replay_')
    law = cr.ContactLaw(profile)
    state = model.initSystem()
    coords = [model.getCoordinateSet().get(i) for i in range(model.getCoordinateSet().getSize())]
    names = [c.getName() for c in coords]
    q = np.column_stack([current_interp._splines[n](times) for n in names])
    qdot = np.column_stack([current_interp._splines[n](times, 1) for n in names])
    original_q, original_qdot = q.copy(), qdot.copy()
    ankle = names.index('pros_ankle_angle')
    q[:, ankle] = old_interp._splines['pros_ankle_angle'](times)
    qdot[:, ankle] = old_interp._splines['pros_ankle_angle'](times, 1)
    unchanged = [i for i in range(len(names)) if i != ankle]
    if not np.array_equal(q[:, unchanged], original_q[:, unchanged]) or not np.array_equal(qdot[:, unchanged], original_qdot[:, unchanged]):
        raise AssertionError('Probe changed a non-ankle coordinate.')
    hull, _ = cr.mesh_hull_vertices()
    binary = json.loads(cr.BINARY.read_text())
    checks = set(np.linspace(0, len(times)-1, 11, dtype=int).tolist())
    foot = model.getBodySet().get('foot_l')
    print(f'AB07 ankle-only probe: {len(times)} FK poses; current model fixed', flush=True)
    values, shared_parity = cr.replay(model, state, coords, foot, law, times, q, qdot, hull, binary, checks)
    # Explicitly separate force and penetration units for this supplementary
    # parity report. The original helper metric combines max record errors.
    force_error, penetration_error = 0.0, 0.0
    for i in sorted(checks):
        cr.set_pose(model, state, coords, q[i], qdot[i], times[i])
        for j, sphere in enumerate(profile.spheres):
            record = model.getForceSet().get('contact_replay_'+sphere.name).getRecordValues(state)
            f = np.array([record.get(k) for k in range(3)])
            force_error = max(force_error, float(np.max(np.abs(f-values['force'][i, j]))))
            penetration_error = max(penetration_error, abs(float(record.get(10)-values['penetration'][i, j])))
    if force_error > 1e-8 or penetration_error > 1e-10:
        raise AssertionError('Supplementary contact parity failed.')
    measured_path = cr.REPO/selection['source_fingerprints']['measured_GRF']['path']
    measured_t, measured_cols, _ = cr.read_storage(measured_path)
    if times[0] < measured_t[0] or times[-1] > measured_t[-1]:
        raise AssertionError('Measured force does not cover exact probe window.')
    measured = np.interp(times, measured_t, measured_cols[selection['left_force_column']])
    probe = cr.summarize(times, values, measured, cycles, selection['BW_normalization_n'], [s.name for s in profile.spheres])
    baseline = original['subjects']['AB07']['cases']['filtered_IK']
    if baseline['aggregate']['duration_s'] != probe['aggregate']['duration_s']:
        raise AssertionError('Original and probe duration differ.')
    delta = (original_q[:, ankle]-q[:, ankle])*180/np.pi
    delta_speed = (original_qdot[:, ankle]-qdot[:, ankle])*180/np.pi
    baseline_a, probe_a = baseline['aggregate'], probe['aggregate']
    comparison = {
        'registered_current_minus_ankle_pre_ground_impulse_ns': baseline_a['impulse_ns']-probe_a['impulse_ns'],
        'registered_current_minus_ankle_pre_ground_mean_force_n': baseline_a['mean_force_n']-probe_a['mean_force_n'],
        'registered_current_over_ankle_pre_ground_impulse_ratio': baseline_a['impulse_ns']/probe_a['impulse_ns'],
        'ankle_pre_ground_minus_registered_current_impulse_percent': 100*(probe_a['impulse_ns']/baseline_a['impulse_ns']-1),
        'per_sphere_registered_current_minus_ankle_pre_ground_impulse_ns': {
            name: baseline_a['per_sphere_impulse_ns'][name]-probe_a['per_sphere_impulse_ns'][name]
            for name in baseline_a['per_sphere_impulse_ns']},
        'current_registered_AB07_over_AB06_mean_force_ratio': baseline_a['mean_force_n']/original['subjects']['AB06']['cases']['filtered_IK']['aggregate']['mean_force_n'],
        'ankle_pre_ground_AB07_over_current_AB06_mean_force_ratio': probe_a['mean_force_n']/original['subjects']['AB06']['cases']['filtered_IK']['aggregate']['mean_force_n'],
        'same_grid_registered_current_minus_pre_ground_ankle_deg': weighted_summary(times, delta),
        'same_grid_registered_current_minus_pre_ground_ankle_speed_deg_s': weighted_summary(times, delta_speed)}
    after = {cr.fp(p)['path']: cr.fp(p)['sha256'] for p in sources}
    if before != after:
        raise AssertionError('Protected artifact changed during supplementary probe.')
    result = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'instruction_check_token': 'CMC_AGENT_OK_2026', 'status': 'PASS',
        'helper': cr.fp(__file__), 'original_replay': cr.fp(ORIGINAL),
        'sources': {'current_model': cr.fp(model_path), 'current_IK': cr.fp(current_ik),
                    'archived_pre_ground_IK': cr.fp(OLD_IK), 'profile': cr.fp(cr.PRIMARY),
                    'receipt': cr.fp(receipt_path), 'selection': cr.fp(cr.PAIR/'selection.json')},
        'methodology': {
            'intervention': 'Current registered AB07 model and all current production-filtered coordinates/speeds; substitute ONLY pros_ankle_angle and its spline derivative from archived pre-ground IK.',
            'time_window_s': [start, end], 'pose_count': len(times),
            'production_filter': {'enabled': current_cfg.enable_kinematics_lowpass_filter,
                                  'cutoff_hz': current_cfg.kinematics_lowpass_cutoff_hz,
                                  'order': current_cfg.kinematics_lowpass_order,
                                  'resample_dt_s': current_cfg.kinematics_resample_dt,
                                  'support': 'Each full source trial, identical recorded timestamps before preprocessing'},
            'grid': 'Identical to original contact replay: full-precision frozen HS/TO/next-HS and every native1ms interior timestamp.',
            'baseline': 'Original filtered_IK aggregate/cycles reused from immutable contact_replay.json; six original cases were not rerun for this probe.',
            'scope': 'FK at Velocity stage plus frozen contact arithmetic. No model/input modification, integration, dynamics, IK solver, controller or policy evaluation.',
            'physical_validity': 'Ankle-substituted pose and speed trajectory is a diagnostic hybrid, not an alternate valid bundle or dynamic causal counterfactual.'},
        'validation': {'status': 'PASS', 'only_ankle_q_and_qdot_changed': True,
                       'cpp_max_abs_force_error_n': force_error,
                       'cpp_max_abs_penetration_error_m': penetration_error,
                       'station_transform_max_abs_error_m_or_m_s': shared_parity['station_transform_max_abs_error_m_or_m_s'],
                       'cpp_records_checked': shared_parity['cpp_records_checked']},
        'current_filtered_baseline': baseline, 'ankle_pre_ground_probe': probe,
        'comparison': comparison,
        'interpretation': [
            'This quantifies the direct response of the frozen force law to the preparation-associated ankle trajectory at fixed current geometry and all other coordinates.',
            'It does not identify total preparation effect: registered distal mounting, other marker-induced IK changes and muscle mechanics remain fixed, and no hybrid dynamics is simulated.',
            'The sparse force basis is disjoint from the display STL and AB06-derived; static ground matching alone does not validate its force generalization.',
            'Original actual paired metrics remain authoritative for observed rollout comparison; this supplemental grid includes exact TO points.'],
        'original_replay_metadata_clarifications': {
            'cpp_max_abs_force_error_n': 'Original helper reports maximum across force[N] and penetration[m] record differences; units are mixed. Supplementary parity here separates them.',
            'mean_sagittal_foot_pitch_rad': 'Original metric is elevation of foot local X: atan2(R10,hypot(R00,R20)); it differs from the registration sagittal-plane projection if roll is nonzero.',
            'measured_impulse_grid': 'Original replay includes exact TO points; AB07 measured impulse1373.009579N s differs slightly from official paired1373.005710N s. Keep observed paired percentages separate.'},
        'inputs_integrity': {'status': 'PASS', 'protected_count': len(before),
                             'all_unchanged': before == after,
                             'fingerprints_before': before, 'fingerprints_after': after}}
    output = cr.AUDIT/'contact_preparation_probe.json'
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(comparison, indent=2), flush=True)
    print('PASS '+output.relative_to(cr.REPO).as_posix(), flush=True)


if __name__ == '__main__':
    main()
