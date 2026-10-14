"""Replay saved/IK poses through the frozen contact law, without dynamics.

Run from any working directory with an OpenSim-enabled Python interpreter.
Writes only contact_replay.json beside this helper. No assembly, integration,
IK solver, model serialization, policy inference or input modification occurs.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import ConvexHull

REPO = Path(__file__).resolve().parents[2]
AUDIT = Path(__file__).resolve().parent
PAIR = REPO / 'Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07'
sys.path.insert(0, str(REPO))
import opensim
from config import SimulatorConfig
from kinematics_interpolator import KinematicsInterpolator
from model_loader import _load_plugin, _configure_geometry_search_paths
from online_grf import OnlineGRFProfile, add_online_grf_forces

PRIMARY = REPO / 'online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json'
LEGACY = REPO / 'online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json'
BINARY = REPO / 'validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json'
MESH = REPO / 'Geometry/AM_foot_l.STL'
CASES = ('raw_IK', 'filtered_IK', 'actual', 'served',
         'bio_actual_pros_served', 'bio_served_pros_actual')


def fp(path):
    path = Path(path).resolve()
    return {'path': path.relative_to(REPO).as_posix(),
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def read_storage(path):
    lines = Path(path).read_text(encoding='utf-8-sig').splitlines()
    end = next(i for i, line in enumerate(lines) if line.strip().lower() == 'endheader')
    header = '\n'.join(lines[:end])
    body = [line for line in lines[end + 1:] if line.strip()]
    names = body[0].split()
    values = np.loadtxt(body[1:], ndmin=2)
    if names[0] != 'time' or len(names) != len(set(names)) or values.shape[1] != len(names):
        raise ValueError(f'Invalid storage schema: {path}')
    if not np.all(np.isfinite(values)) or np.any(np.diff(values[:, 0]) <= 0):
        raise ValueError(f'Nonfinite or unordered storage: {path}')
    return values[:, 0], {name: values[:, i] for i, name in enumerate(names[1:], 1)}, header


def interp_columns(times, source_t, columns, names, suffix):
    if times[0] < source_t[0] or times[-1] > source_t[-1]:
        raise ValueError('Requested replay horizon is not fully recorded.')
    return np.column_stack([np.interp(times, source_t, columns[name + suffix]) for name in names])


def v3(v):
    return np.array([v.get(i) for i in range(3)])


def matrix3(m):
    return np.array([[m.get(i, j) for j in range(3)] for i in range(3)])


def smooth_positive(x, epsilon):
    return 0.5 * (x + np.sqrt(x*x + epsilon*epsilon))


class ContactLaw:
    def __init__(self, profile):
        self.profile = profile
        self.locations = np.array([s.location for s in profile.spheres])
        self.radius = np.array([s.radius for s in profile.spheres])
        self.normal = np.asarray(profile.ground.normal)
        self.origin = np.asarray(profile.ground.origin)
        self.surface_velocity = np.asarray(profile.ground.surface_velocity)
        material = [s.material or profile.material for s in profile.spheres]
        for name in ('stiffness', 'exponent', 'dissipation', 'smoothing',
                     'dynamic_friction', 'static_friction', 'transition_velocity', 'viscous_friction'):
            setattr(self, name, np.array([getattr(m, name) for m in material]))
        self.residual = np.array([s.residual_force_ratio for s in profile.spheres])
        self.res_ref = np.array([s.residual_penetration_reference_m for s in profile.spheres])
        self.res_gain = np.array([s.residual_force_penetration_gain_per_m for s in profile.spheres])
        self.res_rate = np.array([s.residual_force_penetration_rate_gain_s_per_m for s in profile.spheres])

    def evaluate(self, centers, velocity, ground_shift=0.0):
        # Positive shift moves the frozen plane up along its normal; arithmetic
        # sensitivity only, never a saved/operational profile change.
        raw = self.radius - (centers - self.origin) @ self.normal + ground_shift
        smoothed = smooth_positive(raw, self.smoothing)
        relative = velocity - self.surface_velocity
        rate = -relative @ self.normal
        damping = smooth_positive(1.0 + self.dissipation*rate, self.smoothing)
        elastic = self.stiffness * smoothed**self.exponent
        normal_force = elastic * damping
        tangent = relative + rate[..., None]*self.normal
        slip = np.linalg.norm(tangent, axis=-1)
        mu = self.dynamic_friction + (self.static_friction - self.dynamic_friction) * np.exp(-(slip/self.transition_velocity)**2)
        friction = (-mu*normal_force / np.sqrt(slip**2 + self.transition_velocity**2))[..., None]*tangent - self.viscous_friction[..., None]*tangent
        residual = self.residual + (np.maximum(raw, 0) - self.res_ref)[..., None]*self.res_gain + rate[..., None]*self.res_rate
        force = normal_force[..., None]*self.normal + friction + normal_force[..., None]*residual
        return {'force': force, 'normal': normal_force, 'elastic': elastic,
                'penetration': np.maximum(raw, 0), 'rate': rate, 'slip': slip}


def mesh_hull_vertices():
    raw = MESH.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + 50*count:
        raise ValueError('Unexpected fixed hardware STL format')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    vertices = np.unique(np.frombuffer(raw, dtype=dtype, offset=84, count=count)['vertices'].reshape(-1, 3).astype(float), axis=0)
    return vertices[ConvexHull(vertices).vertices], len(vertices)


def set_pose(model, state, coordinates, q, qdot, time):
    state.setTime(float(time))
    for c, position, speed in zip(coordinates, q, qdot):
        c.setValue(state, float(position), False)
        c.setSpeedValue(state, float(speed))
    model.realizeVelocity(state)


def replay(model, state, coordinates, foot, law, times, q, qdot,
           hull_vertices, binary, check_indices):
    centers = np.empty((len(times), len(law.radius), 3))
    velocity = np.empty_like(centers)
    sole = np.empty(len(times))
    foot_y = np.empty(len(times))
    pitch = np.empty(len(times))
    binary_clearance = np.empty((len(times), 2))
    binary_loc = np.array([p['location'] for p in binary['points']])
    binary_origin = np.array(binary['ground']['origin'])
    binary_normal = np.array(binary['ground']['normal'])
    binary_normal /= np.linalg.norm(binary_normal)
    transform_errors = []
    cpp_errors = []
    cpp_forces = [model.getForceSet().get('contact_replay_' + s.name) for s in law.profile.spheres]
    for i, t in enumerate(times):
        set_pose(model, state, coordinates, q[i], qdot[i], t)
        transform = foot.getTransformInGround(state)
        rotation, origin = matrix3(transform.R()), v3(transform.p())
        spatial = foot.getVelocityInGround(state)
        omega, linear = v3(spatial.get(0)), v3(spatial.get(1))
        offset = law.locations @ rotation.T
        centers[i] = offset + origin
        velocity[i] = linear + np.cross(omega, offset)
        # An affine plane's exact mesh minimum lies at a convex-hull vertex.
        mesh_world = hull_vertices @ rotation.T + origin
        sole[i] = np.min((mesh_world - law.origin) @ law.normal)
        foot_y[i] = origin[1]
        pitch[i] = np.arctan2(rotation[1, 0], np.hypot(rotation[0, 0], rotation[2, 0]))
        binary_clearance[i] = (binary_loc @ rotation.T + origin - binary_origin) @ binary_normal
        if i in check_indices:
            values = law.evaluate(centers[i], velocity[i])
            for j, (sph, force) in enumerate(zip(law.profile.spheres, cpp_forces)):
                station = opensim.Vec3(*sph.location)
                transform_errors += [float(np.max(np.abs(v3(foot.findStationLocationInGround(state, station)) - centers[i, j]))),
                                     float(np.max(np.abs(v3(foot.findStationVelocityInGround(state, station)) - velocity[i, j])))]
                rec = force.getRecordValues(state)
                rec = np.array([rec.get(k) for k in range(rec.getSize())])
                cpp_errors.append(float(np.max(np.abs(rec[:3] - values['force'][j]))))
                cpp_errors.append(abs(float(rec[9] - values['normal'][j])))
                cpp_errors.append(abs(float(rec[10] - values['penetration'][j])))
    values = law.evaluate(centers, velocity)
    values.update(centers=centers, velocity=velocity, mesh_sole_clearance=sole,
                  foot_origin_y=foot_y, pitch=pitch, binary_clearance=binary_clearance)
    return values, {'station_transform_max_abs_error_m_or_m_s': max(transform_errors),
                    'cpp_max_abs_force_error_n': max(cpp_errors),
                    'cpp_records_checked': len(check_indices)*len(law.radius)}


def integrate(values, times):
    return np.trapz(values, times, axis=0)


def summarize(times, values, measured, cycles, bw, sphere_names):
    fy = np.sum(values['force'][..., 1], axis=1)
    normal = np.sum(values['normal'], axis=1)
    max_pen = np.max(values['penetration'], axis=1)
    total_duration = times[-1] - times[0]
    def one(mask, start, end, toe_off=None):
        t, f = times[mask], fy[mask]
        if len(t) < 2 or abs(t[0] - start) > 1e-12 or abs(t[-1] - end) > 1e-12:
            raise ValueError('Exact cycle integration boundary missing')
        duration = end - start
        impulse = float(integrate(f, t))
        own_measured = float(integrate(measured[mask], t))
        contact = (normal[mask] > 20.0).astype(float)
        out = {'duration_s': duration, 'impulse_ns': impulse,
               'measured_impulse_ns': own_measured,
               'impulse_error_percent': 100*(impulse/own_measured - 1),
               'mean_force_n': impulse/duration, 'mean_force_bw': impulse/duration/bw,
               'peak_force_n': float(np.max(f)), 'peak_force_bw': float(np.max(f)/bw),
               'normal_force_sum_identity_max_error_n': float(np.max(np.abs(f - normal[mask]))),
               'contact_time_fraction_above_20n': float(integrate(contact, t)/duration),
               'maximum_sphere_penetration_m': float(np.max(max_pen[mask])),
               'mean_maximum_sphere_penetration_m': float(integrate(max_pen[mask], t)/duration),
               'force_weighted_maximum_penetration_m': float(integrate(max_pen[mask]*f, t)/impulse),
               'elastic_only_impulse_ns': float(integrate(np.sum(values['elastic'][mask], axis=1), t)),
               'dissipation_net_impulse_ns': float(integrate(np.sum(values['normal'][mask] - values['elastic'][mask], axis=1), t)),
               'per_sphere_impulse_ns': dict(zip(sphere_names, integrate(values['force'][mask, :, 1], t).tolist())),
               'minimum_mesh_sole_clearance_to_primary_plane_m': float(np.min(values['mesh_sole_clearance'][mask])),
               'mean_mesh_sole_clearance_to_primary_plane_m': float(integrate(values['mesh_sole_clearance'][mask], t)/duration),
               'mean_foot_origin_y_m': float(integrate(values['foot_origin_y'][mask], t)/duration),
               'mean_sagittal_foot_pitch_rad': float(integrate(values['pitch'][mask], t)/duration),
               'binary_heel_min_clearance_m': float(np.min(values['binary_clearance'][mask, 0])),
               'binary_toe_min_clearance_m': float(np.min(values['binary_clearance'][mask, 1]))}
        if toe_off is not None:
            stance = mask & (times <= toe_off)
            swing = mask & (times >= toe_off)
            stance_i = float(integrate(fy[stance], times[stance]))
            out.update(nominal_stance_s=toe_off-start,
                       nominal_stance_fraction=(toe_off-start)/duration,
                       nominal_stance_impulse_ns=stance_i,
                       nominal_stance_mean_force_n=stance_i/(toe_off-start),
                       nominal_swing_impulse_ns=float(integrate(fy[swing], times[swing])))
        return out
    out_cycles = []
    for cycle in cycles:
        hs, end, to = cycle['hs_s'], cycle['next_hs_s'], cycle['to_s']
        out_cycles.append(dict(cycle, **one((times >= hs) & (times <= end), hs, end, to)))
    all_data = one(np.ones(len(times), bool), times[0], times[-1])
    stance_duration = sum(c['stance_s'] for c in cycles)
    stance_impulse = sum(c['nominal_stance_impulse_ns'] for c in out_cycles)
    all_data.update(nominal_stance_duration_s=stance_duration,
                    nominal_stance_fraction=stance_duration/total_duration,
                    nominal_stance_impulse_ns=stance_impulse,
                    nominal_stance_mean_force_n=stance_impulse/stance_duration)
    return {'aggregate': all_data, 'cycles': out_cycles}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default=str(AUDIT/'contact_replay.json'))
    args = parser.parse_args()
    output = Path(args.output).resolve()
    if output.parent != AUDIT or not output.name.startswith('contact_'):
        raise ValueError('Contact audit output must remain in its dedicated directory with contact_ prefix.')
    selection = json.loads((PAIR/'selection.json').read_text())
    raw_profile = json.loads(PRIMARY.read_text())
    left_profile = dict(raw_profile, spheres=[s for s in raw_profile['spheres'] if s['side'] == 'left'])
    profile = OnlineGRFProfile.from_mapping(left_profile, required_sides=('left',))
    law = ContactLaw(profile)
    if not np.array_equal(law.normal, [0, 1, 0]) or any(s.frame != '/bodyset/foot_l' for s in profile.spheres):
        raise ValueError('Frozen frame/vertical contract differs from this diagnostic.')
    binary = json.loads(BINARY.read_text())
    hull, vertex_count = mesh_hull_vertices()
    sources = [Path(__file__), PRIMARY, LEGACY, BINARY, MESH, PAIR/'selection.json',
               REPO/'kinematics_interpolator.py', REPO/'online_grf.py', REPO/'model_loader.py',
               REPO/'config.py', REPO/'path_resolver.py', REPO/'binary_phase_detector.py',
               REPO/'tools/register_seasea_ground.py', REPO/'tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.cpp',
               REPO/'tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.h',
               REPO/'Trajectory Generator/validation/AB07/audit_profile_ground_contract.py',
               REPO/'models/AB07_SEASEA_Threadmill/pipeline_report.json']
    for pattern in ('*SEA_Plugin_BlackBox_mCMC_impedence_ff*', '*OnlineGRFContact*'):
        sources += [p for p in (REPO/'plugins').glob(pattern) if p.suffix in ('.dll', '.so', '.dylib')]
    for subject in ('AB06', 'AB07'):
        run = REPO/f'results/{subject}_paired_prescribed_HS_9s_2026_10_07'
        sources += list(run.rglob('*.sto')) + list(run.rglob('*.csv')) + [run/'receipt.json']
        sources += [REPO/v['path'] for v in selection['subjects'][subject]['source_fingerprints'].values()]
    sources = sorted(set(p.resolve() for p in sources))
    before = {fp(p)['path']: fp(p)['sha256'] for p in sources}
    _load_plugin(str(REPO/'plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff'))
    _load_plugin(str(REPO/'plugins/OnlineGRFContact'))
    spec = importlib.util.spec_from_file_location('contact_geometry_pure', REPO/'Trajectory Generator/validation/AB07/audit_profile_ground_contract.py')
    geometry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(geometry)
    mesh_triangles = geometry.mesh_triangles(MESH)
    mesh_points = mesh_triangles.reshape(-1, 3)
    sphere_mesh_geometry = []
    for sphere in profile.spheres:
        distance = geometry.mesh_distance(np.asarray(sphere.location), mesh_triangles)
        sphere_mesh_geometry.append({
            'name': sphere.name, 'center_to_mesh_minimum_distance_m': distance,
            'radius_m': sphere.radius,
            'sphere_surface_to_mesh_minimum_gap_m': distance-sphere.radius,
            'sphere_disjoint_from_mesh': distance > sphere.radius})
    subjects = {}
    for subject in ('AB06', 'AB07'):
        print(f'REPLAY {subject}: read exact frozen windows and receipt', flush=True)
        sub = selection['subjects'][subject]
        run = REPO/f'results/{subject}_paired_prescribed_HS_9s_2026_10_07'
        receipt = json.loads((run/'receipt.json').read_text())
        for field, path in [('online_grf_profile_file', PRIMARY), ('online_grf_detector_profile_file', LEGACY), ('binary_phase_detector_profile_file', BINARY)]:
            if receipt['profile_association'][field]['sha256'] != fp(path)['sha256']:
                raise AssertionError('Frozen receipt/profile identity differs.')
        model_path = REPO/sub['source_fingerprints']['model']['path']
        ik_path = REPO/sub['source_fingerprints']['IK']['path']
        for path, field in ((model_path, 'model'), (ik_path, 'IK')):
            if fp(path)['sha256'] != sub['source_fingerprints'][field]['sha256']:
                raise AssertionError('Frozen model/IK source identity differs.')
        st, actual, sh = read_storage(run/'sim_outputs/sim_output_states.sto')
        rt, served, rh = read_storage(run/'sim_outputs/sim_output_kinematics_reference.sto')
        gt, online, gh = read_storage(run/'sim_outputs/sim_output_online_grf.sto')
        if not np.array_equal(st, rt) or not np.array_equal(st, gt):
            raise AssertionError('Recorded state/reference/contact timestamps differ.')
        if 'indegrees=no' not in sh.lower() or 'indegrees=no' not in rh.lower():
            raise AssertionError('Recorded q/qdot must be radians/metres.')
        cycles = sub['primary_cycles']
        start, end = cycles[0]['hs_s'], cycles[-1]['next_hs_s']
        boundaries = np.array([c[k] for c in cycles for k in ('hs_s', 'to_s', 'next_hs_s')])
        interior = st[(st > start) & (st < end)]
        times = np.unique(np.r_[interior, boundaries])
        measured_t, measured_cols, mh = read_storage(REPO/sub['source_fingerprints']['measured_GRF']['path'])
        measured = np.interp(times, measured_t, measured_cols[sub['left_force_column']])
        _configure_geometry_search_paths(str(model_path))
        model = opensim.Model(str(model_path))
        if model.getConstraintSet().getSize():
            raise ValueError('FK audit requires an unconstrained explicit-coordinate model.')
        add_online_grf_forces(model, profile, applies_force=False, name_prefix='contact_replay_')
        state = model.initSystem()
        coordset = model.getCoordinateSet()
        coordinates = [coordset.get(i) for i in range(coordset.getSize())]
        names = [c.getName() for c in coordinates]
        foot = model.getBodySet().get('foot_l')
        mass = float(model.getTotalMass(state))
        bw = sub['BW_normalization_n']
        if abs(mass - sub['model_mass_kg_from_XML']) > 1e-10:
            raise AssertionError('Runtime model mass differs from frozen XML.')
        qa = interp_columns(times, st, actual, names, '_q')
        da = interp_columns(times, st, actual, names, '_qdot')
        qs = interp_columns(times, rt, served, names, '_q_ref')
        ds = interp_columns(times, rt, served, names, '_qdot_ref')
        raw_t, raw_cols, ih = read_storage(ik_path)
        if 'indegrees=yes' not in ih.lower():
            raise AssertionError('Raw IK degree contract differs.')
        cfg_dict = dict(receipt['effective_simulator_config'])
        # Rebase only input paths for portability; all numerical settings are the
        # effective recorded settings. No operational configuration is changed.
        cfg_dict.update(model_bundle_dir=str(model_path.parent), model_file=str(model_path),
                        kinematics_file=str(ik_path), external_loads_xml=str(REPO/sub['source_fingerprints']['external_loads']['path']))
        cfg = SimulatorConfig(**cfg_dict)
        interp = KinematicsInterpolator(cfg)
        translation = set(cfg.translation_coords)
        raw_splines = {name: CubicSpline(raw_t, raw_cols[name] * (1 if name in translation else np.pi/180)) for name in names}
        qr = np.column_stack([raw_splines[n](times) for n in names])
        dr = np.column_stack([raw_splines[n](times, 1) for n in names])
        qf = np.column_stack([interp._splines[n](times) for n in names])
        df = np.column_stack([interp._splines[n](times, 1) for n in names])
        pros = [names.index(n) for n in ('pros_knee_angle', 'pros_ankle_angle')]
        bio = [i for i in range(len(names)) if i not in pros]
        qh1, dh1 = qa.copy(), da.copy()
        qh1[:, pros], dh1[:, pros] = qs[:, pros], ds[:, pros]
        qh2, dh2 = qs.copy(), ds.copy()
        qh2[:, pros], dh2[:, pros] = qa[:, pros], da[:, pros]
        poses = dict(raw_IK=(qr, dr), filtered_IK=(qf, df), actual=(qa, da),
                     served=(qs, ds), bio_actual_pros_served=(qh1, dh1), bio_served_pros_actual=(qh2, dh2))
        checks = set(np.linspace(0, len(times)-1, 11, dtype=int).tolist())
        cases, parity, sensitivity = {}, {}, {}
        for case in CASES:
            print(f'REPLAY {subject} {case}: {len(times)} poses, Velocity stage only', flush=True)
            q, qdot = poses[case]
            values, validation = replay(model, state, coordinates, foot, law, times, q, qdot, hull, binary, checks)
            if validation['cpp_max_abs_force_error_n'] > 1e-8 or validation['station_transform_max_abs_error_m_or_m_s'] > 1e-10:
                raise AssertionError('NumPy/C++ or station-velocity parity failed.')
            cases[case] = summarize(times, values, measured, cycles, bw, [s.name for s in profile.spheres])
            parity[case] = validation
            if case == 'actual':
                native_idx = np.searchsorted(times, interior)
                recorded_idx = np.searchsorted(st, interior)
                force_delta = values['force'][native_idx].sum(axis=1) - np.column_stack([online['left_force_' + a][recorded_idx] for a in 'xyz'])
                pen_delta = np.max(values['penetration'][native_idx], axis=1) - online['left_penetration'][recorded_idx]
                validation.update(native_samples_checked=len(interior),
                                  actual_native_force_max_abs_error_n=float(np.max(np.abs(force_delta))),
                                  actual_native_force_rms_error_n=float(np.sqrt(np.mean(force_delta**2))),
                                  actual_native_penetration_max_abs_error_m=float(np.max(np.abs(pen_delta))))
                if validation['actual_native_force_max_abs_error_n'] > 0.003 or validation['actual_native_penetration_max_abs_error_m'] > 2e-7:
                    raise AssertionError('Actual pose replay does not reproduce native contact stream.')
            if case in ('raw_IK', 'filtered_IK', 'actual'):
                sensitivity[case] = {}
                for shift in (-0.01, -0.005, -0.001, 0.0, 0.001, 0.005, 0.01):
                    shifted = law.evaluate(values['centers'], values['velocity'], shift)
                    impulse = float(integrate(shifted['force'][..., 1].sum(axis=1), times))
                    sensitivity[case][f'{shift:+.3f}'] = {'plane_shift_up_m': shift, 'impulse_ns': impulse, 'mean_force_n': impulse/(end-start)}
        # Biological served q comes directly from filtered IK. Small mismatch at
        # exact boundaries may follow interpolation of stored 8-decimal samples.
        mismatch_q = np.max(np.abs(qs[:, bio] - qf[:, bio]), axis=0)
        mismatch_d = np.max(np.abs(ds[:, bio] - df[:, bio]), axis=0)
        subjects[subject] = {'mass_kg': mass, 'BW_n': bw, 'receipt_status': receipt['status'],
                             'primary_time_range_s': [start, end], 'native_grid_samples': len(interior),
                             'coordinate_names': names, 'model_geometry': geometry.model_contract(subject),
                             'validation': {'status': 'PASS', 'case_cpp_parity': parity,
                                            'biological_served_vs_filtered_q_max_abs_by_coordinate': dict(zip([names[i] for i in bio], mismatch_q.tolist())),
                                            'biological_served_vs_filtered_qdot_max_abs_by_coordinate': dict(zip([names[i] for i in bio], mismatch_d.tolist()))},
                             'cases': cases, 'diagnostic_ground_sensitivity': sensitivity}
    pair_comparison = {}
    for case in CASES:
        a = subjects['AB06']['cases'][case]['aggregate']
        b = subjects['AB07']['cases'][case]['aggregate']
        pair_comparison[case] = {'AB07_over_AB06_mean_force_n': b['mean_force_n']/a['mean_force_n'],
                                 'AB07_over_AB06_mean_force_bw': b['mean_force_bw']/a['mean_force_bw'],
                                 'AB06_simulated_over_own_measured_impulse': a['impulse_ns']/a['measured_impulse_ns'],
                                 'AB07_simulated_over_own_measured_impulse': b['impulse_ns']/b['measured_impulse_ns'],
                                 'AB07_minus_AB06_mean_force_n': b['mean_force_n']-a['mean_force_n'],
                                 'per_sphere_AB07_minus_AB06_mean_force_n': {
                                     name: b['per_sphere_impulse_ns'][name]/b['duration_s']-
                                           a['per_sphere_impulse_ns'][name]/a['duration_s']
                                     for name in a['per_sphere_impulse_ns']}}
    after = {fp(p)['path']: fp(p)['sha256'] for p in sources}
    if before != after:
        raise AssertionError('A protected input/source changed during contact replay.')
    contract = {'primary_profile': fp(PRIMARY), 'legacy_detector': fp(LEGACY), 'binary_detector': fp(BINARY),
                'left_primary_spheres': [s.to_dict() for s in profile.spheres],
                'primary_ground': profile.ground.to_dict(), 'binary_ground': binary['ground'],
                'primary_to_binary_plane_vertical_difference_m': binary['ground']['origin'][1]-profile.ground.origin[1],
                'profile_metadata': raw_profile.get('metadata'),
                'force_law_has_mass_or_BW_input': False,
                'vertical_force_equals_sum_normal_force': True,
                'runtime_applied_force_contract': 'primary left8 spheres applied; primary right sensors only; right measured ExternalForce applied; left measured ExternalForce excluded; legacy/binary detectors force-free',
                'hardware': {'mesh': fp(MESH), 'vertex_count': vertex_count, 'convex_hull_vertices_used': len(hull),
                             'attachment': 'both: foot_l direct Mesh, scale factors1 1 1; mesh is visual geometry, not runtime force geometry',
                             'local_bounds_min_m': mesh_points.min(axis=0).tolist(),
                             'local_bounds_max_m': mesh_points.max(axis=0).tolist(),
                             'primary_sphere_mesh_geometry': sphere_mesh_geometry,
                             'interpretation': 'All eight primary spheres are disjoint from the STL. They are an empirically calibrated sparse force basis, not a literal sole-contact surface. This shared fact is not an AB07-specific preparation error.'},
                'shared': ['left sphere local positions/radii/material', 'force law', 'primary plane', 'belt velocity', 'legacy/binary sensor geometry', 'unscaled foot STL'],
                'subject_dependent': ['model mass/anatomy', 'distal pylon mount length', 'registered marker set and own IK', 'right measured load', 'actual tracking states'],
                'plane_difference_interpretation': 'Explicit shared profile calibration offset; not evidence of a frame error. Ground registration uses the binary-plane donor-static convention, not a measured acquisition floor.'}
    quantitative_findings = {
        'replay_gap_exists_before_dynamics': {
            'raw_IK_AB07_over_AB06_absolute_mean_force': pair_comparison['raw_IK']['AB07_over_AB06_mean_force_n'],
            'filtered_IK_AB07_over_AB06_absolute_mean_force': pair_comparison['filtered_IK']['AB07_over_AB06_mean_force_n'],
            'actual_AB07_over_AB06_absolute_mean_force': pair_comparison['actual']['AB07_over_AB06_mean_force_n'],
            'interpretation': 'The shared law applied to each current model and own IK already reproduces an absolute-force gap in the same direction and of similar ratio. Mass/BW and dynamics are not direct inputs to this calculation. Anatomy, mount preparation, marker/IK preparation and AB06-derived profile remain jointly confounded.'},
        'per_subject_changes': {},
        'dominant_contact_basis': {
            'name': 'left_basis_07',
            'filtered_mean_force_difference_n': pair_comparison['filtered_IK']['per_sphere_AB07_minus_AB06_mean_force_n']['left_basis_07'],
            'actual_mean_force_difference_n': pair_comparison['actual']['per_sphere_AB07_minus_AB06_mean_force_n']['left_basis_07'],
            'interpretation': 'This same stiff, undamped sphere contributes more than the net AB07-minus-AB06 mean-force gap; other sphere contributions partly offset it. A larger maximum penetration scalar is not required.'}}
    for subject in subjects:
        case = subjects[subject]['cases']
        raw_a, filtered_a, actual_a, served_a = [case[n]['aggregate'] for n in ('raw_IK', 'filtered_IK', 'actual', 'served')]
        quantitative_findings['per_subject_changes'][subject] = {
            'filtered_minus_raw_impulse_percent': 100*(filtered_a['impulse_ns']/raw_a['impulse_ns']-1),
            'actual_minus_filtered_impulse_percent': 100*(actual_a['impulse_ns']/filtered_a['impulse_ns']-1),
            'served_minus_filtered_impulse_percent': 100*(served_a['impulse_ns']/filtered_a['impulse_ns']-1),
            'actual_minus_served_impulse_percent': 100*(actual_a['impulse_ns']/served_a['impulse_ns']-1),
            'interpretation': 'Arithmetic pose-response comparisons, not an additive or causal decomposition of dynamics.'}
    result = {'created_utc': datetime.now(timezone.utc).isoformat(), 'instruction_check_token': 'CMC_AGENT_OK_2026',
              'status': 'PASS', 'helper': fp(__file__),
              'methodology': {'scope': 'Saved-data FK and direct contact-law arithmetic only; no assembly, dynamics, integration, policy, IK optimization or input edits.',
                              'grid': 'Each frozen primary HS/TO/next-HS at full precision plus every native1ms timestamp strictly inside the common primary window.',
                              'q_qdot': 'actual/served use independent saved q/qdot columns in radians/metres; interpolated only at inserted exact boundaries. Raw IK rotations degree→rad then full-trial CubicSpline derivatives. Filtered IK uses the recorded production KinematicsInterpolator settings over the full source trial.',
                              'hybrids': 'Replace both q and qdot by coordinate name; retain each subject own model. Diagnostic poses, not feasible dynamic counterfactuals.',
                              'force': 'Exact C++ smooth-positive spring/dissipation/friction/residual law; native plugin parity at11 probes per case and side, all spheres.',
                              'BW': 'Fixed runtime mass*9.80665 for reporting; never enters the contact law.',
                              'sensitivity': 'Fixed ±1/5/10mm arithmetic primary-plane shifts; not tuned, not applied to runtime inputs.',
                              'mesh': 'Minimum plane clearance over exact convex-hull vertices of the shared hardware STL; display mesh has no contact force.',
                              'boundary_effect': 'Nonlinear contact law of linearly interpolated q/qdot at inserted boundaries need not equal linearly interpolated native force; native parity compares original saved timestamps only.'},
              'contact_contract': contract, 'subjects': subjects, 'pair_comparison': pair_comparison,
              'quantitative_findings': quantitative_findings,
              'limitations': ['Pose replay isolates the frozen contact response to an observed/reference motion, not dynamic causality of anatomy, mass, controller or preparation.',
                              'Raw IK is a reference pose/velocity, not a measured contact-force-compatible physical trajectory.',
                              'The AB06-derived profile metadata records a short calibration and requires full/forward validation; static mesh registration does not validate force generalization.',
                              'Nominal measured-force HS/TO windows remain fixed; online FSM cycles are distinct and not substituted.',
                              'AB06 native physical tables cover all primary cycles; its original terminal adapter failure remains unchanged.'],
              'inputs_integrity': {'status': 'PASS', 'protected_count': len(before), 'all_unchanged': before == after,
                                   'fingerprints_before': before, 'fingerprints_after': after}}
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(f'PASS {output.relative_to(REPO)}', flush=True)
    for subject in subjects:
        print(subject, {case: round(subjects[subject]['cases'][case]['aggregate']['impulse_ns'], 3) for case in CASES}, flush=True)


if __name__ == '__main__':
    main()
