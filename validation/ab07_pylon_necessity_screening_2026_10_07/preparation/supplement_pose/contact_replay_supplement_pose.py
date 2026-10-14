"""Reference-only FK of three fresh coherent fixed angular counterexamples.

The primary windows, contact basis, full-trial6Hz filtering and native1ms plus
seven exact knots equal the immutable main14. No forward dynamics or fitting.
"""
from datetime import datetime, timezone
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
PREPARATION = HERE.parent
ROOT = PREPARATION.parents[2]
PROTOCOL = HERE / 'supplement_protocol.json'
spec = importlib.util.spec_from_file_location('supplement_reviewed_screen_metrics', PREPARATION / 'contact_replay_pylon_sweep.py')
screen = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = screen
spec.loader.exec_module(screen)
cr, phase, calibration, previous = screen.cr, screen.phase, screen.calibration, screen.previous
fp, read = screen.fp, screen.read
INTEGRATE = screen.INTEGRATE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--all-preparations-complete', action='store_true')
    args = parser.parse_args()
    if not args.all_preparations_complete:
        parser.error('Require all three completed fresh preparation receipts')
    output = HERE / 'contact_replay_supplement.json'
    if output.exists():
        raise FileExistsError('Preserve previous supplementary analysis')
    protocol, batch = read(PROTOCOL), read(HERE / 'batch_receipt.json')
    if batch['status'] != 'completed' or len(batch['cases']) != 3 or set(protocol['cases']) != {'P0_M2', 'P0_M5', 'P0_M10'}:
        raise AssertionError('All three predeclared preparations must be complete')
    paths = {ROOT / p for p in protocol['protected_sources_before']}
    paths.update([Path(__file__), PROTOCOL, HERE / 'batch_receipt.json', HERE / 'prepare_supplement_pose.py',
                  PREPARATION / 'contact_replay_pylon_sweep.py', PREPARATION / 'contact_replay_screening.json'])
    receipts = {}
    for name, case in protocol['cases'].items():
        path = ROOT / case['case_dir'] / 'receipt.json'
        r = read(path)
        if r['status'] != 'completed' or r['case'] != case or r['protocol'] != fp(PROTOCOL) or case['preparation_kind'] != 'fresh_marker_calibration_full_IK':
            raise AssertionError('Fresh coherent supplementary receipt/protocol mismatch')
        for role in ('output_model', 'full_IK', 'actual_solver_setup', 'actual_solver_model_copy', 'actual_solver_marker_copy', 'actual_solver_log'):
            if fp(ROOT / r[role]['path']) != r[role]:
                raise AssertionError('Consumed fresh preparation artifact changed')
        paths.update(p for p in path.parent.rglob('*') if p.is_file())
        receipts[name] = r
    for raw, sha in protocol['protected_sources_before'].items():
        if fp(ROOT / raw)['sha256'] != sha:
            raise AssertionError('Protected input changed')
    before = {fp(p)['path']: fp(p)['sha256'] for p in sorted(paths)}
    selection = read(screen.PAIR / 'selection.json')
    selected, main_screen = selection['subjects']['AB07'], read(PREPARATION / 'contact_replay_screening.json')
    cycles = selected['primary_cycles']
    native, _, _ = cr.read_storage(ROOT / 'results/AB07_paired_prescribed_HS_9s_2026_10_07/sim_outputs/sim_output_states.sto')
    knots = np.unique([c[k] for c in cycles for k in ('hs_s', 'to_s', 'next_hs_s')])
    start, end = knots[0], knots[-1]
    times = np.unique(np.r_[native[(native > start) & (native < end)], knots])
    grid = main_screen['methodology']['grid']
    if grid != {'primary_s': [start, end], 'samples': len(times), 'exact7events_s': knots.tolist()} or len(knots) != 7 or start < native[0] or end > native[-1]:
        raise AssertionError('Original main14 exact event/native grid differs')
    raw = read(cr.PRIMARY)
    profile = cr.OnlineGRFProfile.from_mapping(dict(raw, spheres=[s for s in raw['spheres'] if s['side'] == 'left']), required_sides=('left',))
    law, sphere_names = cr.ContactLaw(profile), [s.name for s in profile.spheres]
    i07 = sphere_names.index('left_basis_07')
    frozen_receipt = read(ROOT / 'results/AB07_paired_prescribed_HS_9s_2026_10_07/receipt.json')
    for field, path in [('online_grf_profile_file', cr.PRIMARY), ('binary_phase_detector_profile_file', cr.BINARY)]:
        if frozen_receipt['profile_association'][field]['sha256'] != fp(path)['sha256']:
            raise AssertionError('Frozen runtime profile identity changed')
    if main_screen['methodology']['fixed_contact_profile'] != fp(cr.PRIMARY):
        raise AssertionError('Main14 contact profile differs')
    mt, mc, _ = cr.read_storage(ROOT / selected['source_fingerprints']['measured_GRF']['path'])
    measured = previous.interp_supported(mt, mc[selected['left_force_column']], times)
    right = previous.interp_supported(mt, mc['ground_force2_vy'], times)
    trc = calibration.parse_trc(ROOT / protocol['source_roles']['full_trial_TRC']['path'])
    if sys.executable != protocol['backend']['python'] or str(calibration.osim.GetVersionAndDate()) != protocol['backend']['opensim_version']:
        raise AssertionError('Frozen preparation/replay backend differs')
    cr._load_plugin(str(ROOT / 'plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff'))
    cr._load_plugin(str(ROOT / 'plugins/OnlineGRFContact'))
    hull, _ = cr.mesh_hull_vertices()
    binary = read(cr.BINARY)
    reference = main_screen['cases']['P4_A1']
    ref_interp = cr.KinematicsInterpolator(screen.config(frozen_receipt, ROOT / reference['model']['path'], ROOT / reference['full_IK']['path'], selected))
    ref_names = ref_interp.coord_names
    if len(ref_names) != 21 or len(set(ref_names)) != 21:
        raise AssertionError('Current reference complete unique schema required')
    qref = np.column_stack([ref_interp._splines[n](times) for n in ref_names])
    cases = {}
    for name, receipt in receipts.items():
        print('SUPPLEMENT FK', name, flush=True)
        mp, ik = ROOT / receipt['output_model']['path'], ROOT / receipt['full_IK']['path']
        _, raw_cols, raw_header = cr.read_storage(ik)
        cr._configure_geometry_search_paths(str(mp))
        model = cr.opensim.Model(str(mp))
        if model.getConstraintSet().getSize():
            raise AssertionError('Explicit unconstrained FK required')
        cr.add_online_grf_forces(model, profile, applies_force=False, name_prefix='contact_replay_')
        state = model.initSystem()
        coords = [model.getCoordinateSet().get(i) for i in range(model.getCoordinateSet().getSize())]
        names = [c.getName() for c in coords]
        if names != ref_names or len(raw_cols) != 21 or set(raw_cols) != set(names) or 'indegrees=yes' not in raw_header.lower():
            raise AssertionError('Complete raw21unique named degree IK schema differs')
        interp = cr.KinematicsInterpolator(screen.config(frozen_receipt, mp, ik, selected))
        if len(interp.coord_names) != 21 or len(set(interp.coord_names)) != 21 or set(interp.coord_names) != set(names) or start < interp.time_bounds[0] or end > interp.time_bounds[1]:
            raise AssertionError('Complete full-trial filtering/schema/support required')
        q = np.column_stack([interp._splines[n](times) for n in names])
        qd = np.column_stack([interp._splines[n](times, 1) for n in names])
        values, parity = cr.replay(model, state, coords, model.getBodySet().get('foot_l'), law, times, q, qd, hull, binary, set(np.linspace(0, len(times) - 1, 11, dtype=int)))
        if parity['cpp_max_abs_force_error_n'] > 1e-8 or parity['station_transform_max_abs_error_m_or_m_s'] > 1e-10:
            raise AssertionError('Inherited implementation/station parity failed')
        parity['cpp_metric_units_note'] = 'Inherited numerical max combines force[N] and penetration[m]; implementation diagnostic, not homogeneous physical error or qualification gate'
        f, b07 = values['force'][..., 1].sum(axis=1), values['force'][:, i07, 1]
        contact = phase.budget(times, f, measured, cycles, selected['BW_normalization_n'],
                               {'ankle_center_Y': values['foot_origin_y'], 'mesh_sole_clearance': values['mesh_sole_clearance'],
                                'basis07_positive_penetration': values['penetration'][:, i07], 'maximum_positive_penetration': values['penetration'].max(axis=1)}, b07)
        contact['contact_summary'] = cr.summarize(times, values, measured, cycles, selected['BW_normalization_n'], sphere_names)
        for cycle in contact['cycles']:
            for row in cycle['stance_phase_bins']:
                a, b = row['time_window_s']
                row['basis07_force_mean_N'] = phase.integral(times, b07, a, b) / (b - a)
        for i, row in enumerate(contact['pooled_stance_phase_bins']):
            bins = [c['stance_phase_bins'][i] for c in contact['cycles']]
            row['basis07_force_mean_N'] = sum(b['basis07_force_mean_N'] * b['duration_s'] for b in bins) / sum(b['duration_s'] for b in bins)
        diagnostics = previous.pose_diagnostics(model, state, coords, times, q, qd, trc, cycles)
        mass, gravity = float(model.getTotalMass(state)), cr.v3(model.getGravity())
        if abs(mass - selected['model_mass_kg_from_XML']) > 1e-10:
            raise AssertionError('Frozen mass convention changed')
        budget = previous.com_budget(times, f, right, measured, cycles, mass, gravity, diagnostics['COM_boundaries'])
        delta = q - qref
        cases[name] = {'case': receipt['case'], 'preparation_receipt': fp(ROOT / receipt['case']['case_dir'] / 'receipt.json'),
                       'model': receipt['output_model'], 'full_IK': receipt['full_IK'], 'contact': contact, 'pose_diagnostics': diagnostics,
                       'reference_COM_budget': budget, 'validation': parity,
                       'reference_pose_difference_vs_current': {'meaning': 'Reference q differences, not actual tracking errors', 'coordinate_names': names,
                           'max_abs_q_difference_by_coordinate': dict(zip(names, np.max(np.abs(delta), axis=0).tolist())),
                           'RMS_q_difference_by_coordinate': dict(zip(names, np.sqrt(INTEGRATE(delta * delta, times, axis=0) / (end - start)).tolist())),
                           'mean_q_difference_by_coordinate': dict(zip(names, (INTEGRATE(delta, times, axis=0) / (end - start)).tolist()))},
                       'evaluation': 'Fresh coherent own model/fullIK FK at Velocity stage only'}
    scalar = {n: dict(distal_mount_length_mm=c['case']['distal_mount_length_mm'], static_ankle_offset_deg=c['case']['static_ankle_offset_deg'], **previous.scalar_case(c)) for n, c in cases.items()}
    references = {n: main_screen['scalar_comparison'][n] for n in ['P0_A0', 'P4_A1']}
    after = {fp(p)['path']: fp(p)['sha256'] for p in sorted(paths)}
    if before != after:
        raise AssertionError('Protected or consumed input changed during supplementary FK')
    result = {'created_utc': datetime.now(timezone.utc).isoformat(), 'instruction_check_token': 'CMC_AGENT_OK_2026',
              'status': 'PASS_supplement_reference_FK_only', 'new_Claude_audit_content_read': False, 'helper': fp(Path(__file__)), 'protocol': fp(PROTOCOL),
              'methodology': {'grid': grid, 'filtering': 'Whole-trial6Hz order4 resample1ms, exact spline derivative qdot', 'fixed_profile': fp(cr.PRIMARY),
                              'cells': 'Three fresh calibrated12marker/fullIK cases, fixed shortest length and predeclared -2/-5/-10deg static offsets',
                              'phase': 'Frozen own measured healthy stance/swing;20 descriptive bins, real-time impulse durations',
                              'dynamics': 'Root owns all18 predeclared native runs; no native integration or force fitting here'},
              'cases': cases, 'scalar_comparison': scalar, 'immutable_main_reference_scalars': references,
              'limitations': ['Reference contact only; no forward dynamics or actual reserve/tracking evidence here.',
                              'Virtual prosthetic force basis versus measured healthy limb; force compatibility does not validate clinical placement.',
                              'A successful angular counterexample does not establish an optimal mount; all three failures would not establish universal impossibility.'],
              'inputs_integrity': {'original_protected_count': protocol['protected_count'], 'consumed_count': len(before), 'before': before, 'after': after, 'all_unchanged': True}}
    with output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'output': fp(output), 'scalar_comparison': scalar}, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
