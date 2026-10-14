"""Audit vertical momentum of saved paired trials using FK at fixed boundaries.

No integration, dynamics realization, IK optimization or model serialization.
Run with an OpenSim-enabled Python from any working directory. Output is new
and exclusive beside this helper; all source files are fingerprinted unchanged.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.interpolate import CubicSpline

REPO = Path(__file__).resolve().parents[3]
AUDIT = Path(__file__).resolve().parent
PAIR = REPO / 'Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07'
sys.path.insert(0, str(REPO))
import opensim
from config import SimulatorConfig
from kinematics_interpolator import KinematicsInterpolator
from model_loader import _load_plugin, _configure_geometry_search_paths
from tools.convert_epic_ab06_tables import decode_mcos_table, IK_SOURCE_COLUMNS

TRAPEZOID = np.trapezoid if hasattr(np, 'trapezoid') else np.trapz


def fingerprint(path):
    path = Path(path).resolve()
    return {'path': path.relative_to(REPO).as_posix(),
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def storage(path):
    lines = Path(path).read_text(encoding='utf-8-sig').splitlines()
    end = next(i for i, line in enumerate(lines) if line.strip().lower() == 'endheader')
    body = [line for line in lines[end + 1:] if line.strip()]
    names = body[0].split()
    values = np.loadtxt(body[1:], ndmin=2)
    if names[0] != 'time' or len(set(names)) != len(names) or len(names) != values.shape[1]:
        raise ValueError(f'Invalid storage schema: {path}')
    if not np.all(np.isfinite(values)) or np.any(np.diff(values[:, 0]) <= 0):
        raise ValueError(f'Nonfinite/unordered storage: {path}')
    return values[:, 0], dict(zip(names[1:], values[:, 1:].T)), '\n'.join(lines[:end])


def sampled_impulse(t, v, reference_grid, start, end):
    """Paired-native1ms lattice with only exact HS boundaries inserted."""
    if start < t[0] or end > t[-1]:
        raise ValueError('Impulse requires complete recorded support.')
    grid = np.r_[start, reference_grid[(reference_grid > start) & (reference_grid < end)], end]
    return float(TRAPEZOID(np.interp(grid, t, v), grid))


def v3(value):
    return np.array([value.get(i) for i in range(3)])


def original_splines(path, cfg):
    table = decode_mcos_table(path)
    if table.names != tuple(IK_SOURCE_COLUMNS):
        raise ValueError('Original healthy IK schema differs.')
    names = table.names[1:]
    time = table.column('Header')
    data = np.column_stack([table.column(n) for n in names])
    if not np.all(np.isfinite(data)) or np.any(np.diff(time) <= 0):
        raise ValueError('Original IK nonfinite/unordered.')
    for i, name in enumerate(names):
        if name not in cfg.translation_coords:
            data[:, i] *= np.pi / 180
    # Reuse the production filtering function with recorded numerical settings.
    # This path keeps all 23 original healthy coordinates (including left MTP
    # and subtalar); no remap or synthetic output motion is written.
    preprocessor = KinematicsInterpolator.__new__(KinematicsInterpolator)
    preprocessor._cfg = cfg
    if cfg.enable_kinematics_lowpass_filter:
        time, data = preprocessor._lowpass_and_resample(time, data)
    return {name: CubicSpline(time, data[:, i]) for i, name in enumerate(names)}


def root_translation_contract(path):
    root = ET.parse(path).getroot()
    joint = next(j for j in root.iter('CustomJoint') if j.get('name') == 'ground_pelvis')
    axis = next(a for a in joint.iter('TransformAxis') if a.findtext('coordinates') == 'pelvis_ty')
    contract = {'axis': [float(v) for v in axis.findtext('axis').split()],
                'linear_coefficients': [float(v) for v in axis.findtext('LinearFunction/coefficients').split()],
                'parent_orientation': [float(v) for v in joint.find("frames/PhysicalOffsetFrame[@name='ground_offset']/orientation").text.split()]}
    if contract != {'axis': [0., 1., 0.], 'linear_coefficients': [1., 0.], 'parent_orientation': [0., 0., 0.]}:
        raise ValueError('Cannot identify pelvis_ty generalized force with ground vertical force.')
    return contract


def com_boundaries(model_path, times, q, qdot):
    _configure_geometry_search_paths(str(model_path))
    model = opensim.Model(str(model_path))
    if model.getConstraintSet().getSize():
        raise ValueError('COM audit requires explicit unconstrained coordinates.')
    state = model.initSystem()
    coords = [model.getCoordinateSet().get(i) for i in range(model.getCoordinateSet().getSize())]
    names = [c.getName() for c in coords]
    if set(names) != set(q) or set(names) != set(qdot):
        raise ValueError('Motion does not provide exactly the model coordinate schema.')
    bodies = [model.getBodySet().get(i) for i in range(model.getBodySet().getSize())]
    mass = float(model.getTotalMass(state))
    gravity = v3(model.getGravity())
    if gravity[0] != 0 or gravity[2] != 0 or gravity[1] >= 0:
        raise ValueError('Vertical gravity contract differs.')
    rows = []
    parity = 0.
    for t in times:
        state.setTime(float(t))
        for c in coords:
            c.setValue(state, float(q[c.getName()](t)), False)
            c.setSpeedValue(state, float(qdot[c.getName()](t)))
        model.realizeVelocity(state)
        velocity = sum((float(b.getMass()) * v3(b.findStationVelocityInGround(state, b.getMassCenter())) for b in bodies), np.zeros(3)) / mass
        position = sum((float(b.getMass()) * v3(b.findStationLocationInGround(state, b.getMassCenter())) for b in bodies), np.zeros(3)) / mass
        native_v = v3(model.calcMassCenterVelocity(state))
        native_p = v3(model.calcMassCenterPosition(state))
        parity = max(parity, float(np.max(np.abs(velocity-native_v))), float(np.max(np.abs(position-native_p))))
        rows.append({'time_s': float(t), 'COM_position_m': position.tolist(), 'COM_velocity_m_s': velocity.tolist()})
    # A unit pelvis_ty speed moves every body's COM upward at 1 m/s. This
    # independent virtual-work check identifies its generalized force in N.
    for c in coords:
        c.setSpeedValue(state, 1. if c.getName() == 'pelvis_ty' else 0.)
    model.realizeVelocity(state)
    translation_velocity = [v3(b.findStationVelocityInGround(state, b.getMassCenter())) for b in bodies]
    translation_error = float(np.max(np.abs(np.array(translation_velocity) - [0, 1, 0])))
    if parity > 1e-10 or translation_error > 1e-10:
        raise AssertionError('Native COM/vertical-translation identity failed.')
    return {'model': fingerprint(model_path), 'mass_kg': mass, 'gravity_m_s2': gravity.tolist(),
            'rows': rows, 'native_COM_position_velocity_max_error': parity,
            'unit_pelvis_ty_speed_body_velocity_max_error_m_s': translation_error,
            'root_translation_contract': root_translation_contract(model_path)}


def main():
    output = AUDIT / 'vertical_impulse_audit.json'
    if output.exists():
        raise FileExistsError('Preserve an existing diagnostic; use a separate version.')
    selection = json.loads((PAIR / 'selection.json').read_text())
    paths = {Path(__file__), REPO/'AGENTS.md', PAIR/'selection.json', REPO/'config.py',
             REPO/'kinematics_interpolator.py', REPO/'model_loader.py', REPO/'path_resolver.py',
             REPO/'tools/convert_epic_ab06_tables.py', REPO/'simulation_runner.py', REPO/'output.py'}
    inputs = {}
    for subject in ('AB06', 'AB07'):
        s = selection['subjects'][subject]
        run = REPO/f'results/{subject}_paired_prescribed_HS_9s_2026_10_07'
        healthy = REPO/f'models/{subject}-raw/osimxml'/('AB06.osim' if subject == 'AB06' else 'ab07.osim')
        day = '10_09_18' if subject == 'AB06' else '10_14_18'
        original = REPO/f'models/{subject}-raw/{day}/treadmill/ik/treadmill_01_01.mat'
        model = REPO/s['source_fingerprints']['model']['path']
        ik = REPO/s['source_fingerprints']['IK']['path']
        measured = REPO/s['source_fingerprints']['measured_GRF']['path']
        inputs[subject] = dict(healthy=healthy, original=original, model=model, ik=ik, measured=measured, run=run)
        paths.update([healthy, original, model, ik, measured, run/'receipt.json'])
        paths.update(REPO/f['path'] for f in s['source_fingerprints'].values())
        paths.update(run/'sim_outputs'/name for name in ('sim_output_states.sto', 'sim_output_online_grf.sto', 'sim_output_reserve_torques.sto'))
        for path, field in ((model, 'model'), (ik, 'IK'), (measured, 'measured_GRF')):
            if fingerprint(path)['sha256'] != s['source_fingerprints'][field]['sha256']:
                raise AssertionError('Frozen source differs from paired selection.')
    for pattern in ('*SEA_Plugin_BlackBox_mCMC_impedence_ff*', '*OnlineGRFContact*'):
        paths.update(p for p in (REPO/'plugins').glob(pattern) if p.suffix in ('.dll', '.so', '.dylib'))
    before = {fingerprint(p)['path']: fingerprint(p)['sha256'] for p in sorted(paths)}
    _load_plugin(str(REPO/'plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff'))
    _load_plugin(str(REPO/'plugins/OnlineGRFContact'))
    subjects = {}
    for subject in ('AB06', 'AB07'):
        s, p = selection['subjects'][subject], inputs[subject]
        cycles = s['primary_cycles']
        boundary_times = np.unique([c[k] for c in cycles for k in ('hs_s', 'to_s', 'next_hs_s')])
        receipt = json.loads((p['run']/'receipt.json').read_text())
        cfg_dict = dict(receipt['effective_simulator_config'])
        cfg_dict.update(model_bundle_dir=str(p['model'].parent), model_file=str(p['model']), kinematics_file=str(p['ik']),
                        external_loads_xml=str(REPO/s['source_fingerprints']['external_loads']['path']),
                        reserve_actuators_xml=str(REPO/s['source_fingerprints']['reserve_actuators']['path']))
        cfg = SimulatorConfig(**cfg_dict)
        virtual = KinematicsInterpolator(cfg)
        healthy = original_splines(p['original'], cfg)
        st, native, state_header = storage(p['run']/'sim_outputs/sim_output_states.sto')
        gt, grf, _ = storage(p['run']/'sim_outputs/sim_output_online_grf.sto')
        rt, reserve, _ = storage(p['run']/'sim_outputs/sim_output_reserve_torques.sto')
        mt, measured, _ = storage(p['measured'])
        if 'indegrees=no' not in state_header.lower() or not np.array_equal(st, gt) or not np.array_equal(st, rt):
            raise AssertionError('Saved state/force clock or units differ.')
        if boundary_times[0] < st[0] or boundary_times[-1] > st[-1]:
            raise ValueError('Primary boundary is unrecorded; do not change windows.')
        def derivative(splines):
            return {name: spline.derivative() for name, spline in splines.items()}
        actual_q = {name: lambda t, name=name: np.interp(t, st, native[name+'_q']) for name in virtual.coord_names}
        actual_d = {name: lambda t, name=name: np.interp(t, st, native[name+'_qdot']) for name in virtual.coord_names}
        cases = {
            'healthy_original_IK_filtered': com_boundaries(p['healthy'], boundary_times, healthy, derivative(healthy)),
            'virtual_prepared_IK_filtered': com_boundaries(p['model'], boundary_times, virtual._splines, derivative(virtual._splines)),
            'virtual_actual_saved': com_boundaries(p['model'], boundary_times, actual_q, actual_d)}
        for name, case in cases.items():
            if abs(case['mass_kg'] - s['model_mass_kg_from_XML']) > 1e-10:
                raise AssertionError('Source total mass differs from frozen mass.')
            endpoints = {r['time_s']: r for r in case['rows']}
            def summarize(start, end):
                duration = end-start
                left = sampled_impulse(gt, grf['left_force_y'], st, start, end)
                right = sampled_impulse(mt, measured['ground_force2_vy'], st, start, end)
                measured_left = sampled_impulse(mt, measured[s['left_force_column']], st, start, end)
                rv = sampled_impulse(rt, reserve['pelvis_ty_reserve_torque'], st, start, end)
                momentum = case['mass_kg']*(endpoints[end]['COM_velocity_m_s'][1]-endpoints[start]['COM_velocity_m_s'][1])
                gravity = -case['mass_kg']*case['gravity_m_s2'][1]*duration
                required = gravity+momentum-right
                actual_only = name == 'virtual_actual_saved'
                return {'window_s': [start, end], 'duration_s': duration,
                        'vertical_momentum_change_Ns': momentum, 'gravity_impulse_Ns': gravity,
                        'right_measured_impulse_Ns': right, 'left_measured_impulse_Ns': measured_left,
                        'left_required_with_measured_right_and_no_pelvis_support_Ns': required,
                        'healthy_measured_total_minus_required_total_Ns': measured_left+right-gravity-momentum,
                        'healthy_measured_total_momentum_defect_div_BW_duration': (measured_left+right-gravity-momentum)/gravity,
                        'actual_left_impulse_Ns': left if actual_only else None,
                        'actual_pelvis_vertical_reserve_impulse_Ns': rv if actual_only else None,
                        'actual_pelvis_vertical_reserve_mean_N': rv/duration if actual_only else None,
                        'actual_pelvis_vertical_reserve_mean_div_BW': rv/gravity if actual_only else None,
                        'actual_contact_total_without_reserve_momentum_defect_Ns': left+right-gravity-momentum if actual_only else None,
                        'actual_contact_plus_reserve_momentum_closure_error_Ns': left+right+rv-gravity-momentum if actual_only else None,
                        'actual_contact_plus_reserve_momentum_closure_error_div_BW_duration': (left+right+rv-gravity-momentum)/gravity if actual_only else None}
            case['cycles'] = [dict(c, **summarize(c['hs_s'], c['next_hs_s'])) for c in cycles]
            case['aggregate'] = summarize(cycles[0]['hs_s'], cycles[-1]['next_hs_s'])
        subjects[subject] = {'receipt_status': receipt['status'], 'cases': cases,
                             'source_IK_original': fingerprint(p['original']),
                             'production_filter': {'enabled': cfg.enable_kinematics_lowpass_filter, 'cutoff_hz': cfg.kinematics_lowpass_cutoff_hz, 'order': cfg.kinematics_lowpass_order, 'dt_s': cfg.kinematics_resample_dt},
                             'force_acquisition': 'Recorded native1ms lattice, exact HS boundaries; measured right interpolated on the same grid; preserves official primary observed-force metrics.'}
    after = {fingerprint(p)['path']: fingerprint(p)['sha256'] for p in sorted(paths)}
    if before != after:
        raise AssertionError('Protected source changed.')
    result = {'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
              'instruction_check_token': 'CMC_AGENT_OK_2026', 'status': 'PASS',
              'helper': fingerprint(__file__), 'subjects': subjects,
              'methodology': {'scope': 'FK at Velocity stage at fixed4HS+3TO only; no integration/dynamics/optimization or input changes.',
                              'equation': 'J_left + J_right + J_pelvis_reserve - m*g*T = m*(COM_y_dot(end)-COM_y_dot(start))',
                              'internal_forces': 'Muscles and joint SEA actuation exchange internal momentum; the global pelvis_ty reserve supplies external vertical momentum.',
                              'healthy_reference': 'Original23-coordinate EPIC IK with production full-trial zero-phase filtering; original subject healthy model. This preserves original left subtalar/MTP, without virtual remap.',
                              'virtual_reference': 'Frozen prepared21-coordinate IK and current virtual model, recorded production filter.',
                              'actual': 'Saved q/qdot independently interpolated only at full-precision boundary timestamps; exact nonlinear FK of those samples.',
                              'force_sampling': 'Recorded force and reserve tables are sampled before each native advance. Trapezoidal integration and8decimal storage leave a numerical closure residual; no qualification tolerance is inferred.',
                              'support_force': 'pelvis_ty ground transform unitY/linear1 verified by XML and independent unit-speed translation of every body COM.'},
              'interpretation': ['A large online-vs-measured left impulse can coexist with stable prescribed tracking when pelvis reserves compensate the vertical impulse. It is not by itself a unique signature of anatomy or muscle dynamics.',
                                 'Healthy-vs-virtual reference COM comparisons assess the impulse compatibility of measured healthy loading; they do not identify a virtual hardware contact constitutive law.',
                                 'Body mass is unchanged within each subject healthy→virtual; BW normalization cannot create the measured within-subject simulated/measured impulse ratio.',
                                 'This complement is independent of the model×motion FK factorial. Distinct anatomy, prepared reference motion and contact calibration remain distinguishable interventions.',
                                 'AB06 original endpoint adapter failure is preserved; all frozen primary boundaries have saved physical support.'],
              'inputs_integrity': {'all_unchanged': before == after, 'protected_count': len(before), 'fingerprints_before': before, 'fingerprints_after': after}}
    with output.open('x', encoding='utf-8') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    for subject, data in subjects.items():
        a = data['cases']['virtual_actual_saved']['aggregate']
        print(subject, 'reserve_mean_N', a['actual_pelvis_vertical_reserve_mean_N'], 'closure_Ns', a['actual_contact_plus_reserve_momentum_closure_error_Ns'])
    print('PASS', output.relative_to(REPO))


if __name__ == '__main__':
    main()
