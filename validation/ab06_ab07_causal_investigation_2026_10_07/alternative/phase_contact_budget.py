"""Split saved contact into frozen measured stance/swing; no new FK or dynamics.

Run from any directory with NumPy. Uses exported filtered-IK FK arrays and
original native contact tables. Every source is hash protected. Writes a new
exclusive JSON beside this helper; no operational input or profile is changed.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
AUDIT = Path(__file__).resolve().parent
MODEL = AUDIT.parent/'model'
PREVIOUS = REPO/'validation/ab06_ab07_preparation_vs_model_2026_10_07'
PAIR = REPO/'Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07'
PROFILE = REPO/'online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json'
INTEGRATE = np.trapezoid if hasattr(np, 'trapezoid') else np.trapz
N_PHASE_BINS = 20
THRESHOLD_N = 20.


def fp(p):
    p = Path(p).resolve()
    return {'path': p.relative_to(REPO).as_posix(), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}


def storage(p):
    lines = Path(p).read_text(encoding='utf-8-sig').splitlines()
    end = next(i for i, line in enumerate(lines) if line.strip().lower() == 'endheader')
    body = [line for line in lines[end+1:] if line.strip()]
    names, a = body[0].split(), np.loadtxt(body[1:], ndmin=2)
    if names[0] != 'time' or len(names) != a.shape[1] or len(set(names)) != len(names):
        raise ValueError('Storage schema differs.')
    if not np.all(np.isfinite(a)) or np.any(np.diff(a[:, 0]) <= 0):
        raise ValueError('Storage data are nonfinite/unordered.')
    return a[:, 0], dict(zip(names[1:], a[:, 1:].T))


def window(t, y, start, end):
    if end <= start or start < t[0] or end > t[-1]:
        raise ValueError('Requested interval lacks complete source support.')
    x = np.r_[start, t[(t > start) & (t < end)], end]
    return x, np.interp(x, t, y)


def integral(t, y, start, end):
    x, v = window(t, y, start, end)
    return float(INTEGRATE(v, x))


def active_duration(t, y, start, end):
    """Exact >20N duration for the piecewise-linear recorded force curve."""
    x, f = window(t, y, start, end)
    a, b, dt = f[:-1]-THRESHOLD_N, f[1:]-THRESHOLD_N, np.diff(x)
    fraction = ((a > 0) & (b > 0)).astype(float)
    up, down = (a <= 0) & (b > 0), (a > 0) & (b <= 0)
    fraction[up] = b[up]/(b[up]-a[up])
    fraction[down] = a[down]/(a[down]-b[down])
    return float(np.sum(fraction*dt))


def force_intervals(t, f):
    """All threshold intervals on source support, with censored ends explicit."""
    transitions = []
    for i in np.flatnonzero((f[:-1] > THRESHOLD_N) != (f[1:] > THRESHOLD_N)):
        crossing = float(t[i] + (THRESHOLD_N-f[i])/(f[i+1]-f[i])*(t[i+1]-t[i]))
        transitions.append((crossing, bool(f[i+1] > THRESHOLD_N)))
    intervals, start = [], float(t[0]) if f[0] > THRESHOLD_N else None
    left_censored = start is not None
    for time, enters in transitions:
        if enters:
            start, left_censored = time, False
        else:
            intervals.append({'onset_s': start, 'offset_s': time, 'left_censored': left_censored, 'right_censored': False})
            start = None
    if start is not None:
        intervals.append({'onset_s': start, 'offset_s': float(t[-1]), 'left_censored': left_censored, 'right_censored': True})
    return intervals


def budget(t, f, measured, cycles, bw, series, basis_force=None):
    """series depth values are m; absent sphere-specific actual values stay null."""
    results = []
    for c in cycles:
        hs, to, end = c['hs_s'], c['to_s'], c['next_hs_s']
        duration, stance_duration = end-hs, to-hs
        total, stance = integral(t, f, hs, end), integral(t, f, hs, to)
        swing = integral(t, f, to, end)
        measured_total = integral(t, measured, hs, end)
        measured_stance = integral(t, measured, hs, to)
        measured_swing = integral(t, measured, to, end)
        if abs(total-stance-swing) > 1e-8:
            raise AssertionError('Exact stance/swing impulse sum failed.')
        depth = {}
        for name, v in series.items():
            depth[name] = {'mean_stance_m': integral(t, v, hs, to)/stance_duration,
                           'mean_swing_m': integral(t, v, to, end)/(end-to),
                           'minimum_swing_m': float(window(t, v, to, end)[1].min()),
                           'maximum_swing_m': float(window(t, v, to, end)[1].max())} if v is not None else None
        bins = []
        for j in range(N_PHASE_BINS):
            a, b = hs+stance_duration*j/N_PHASE_BINS, hs+stance_duration*(j+1)/N_PHASE_BINS
            entry = {'phase_start': j/N_PHASE_BINS, 'phase_end': (j+1)/N_PHASE_BINS,
                     'time_window_s': [a, b], 'duration_s': b-a,
                     'force_mean_N': integral(t, f, a, b)/(b-a),
                     'measured_force_mean_N': integral(t, measured, a, b)/(b-a)}
            entry['force_mean_BW'] = entry['force_mean_N']/bw
            for name, v in series.items():
                entry[name+'_mean_m'] = integral(t, v, a, b)/(b-a) if v is not None else None
            bins.append(entry)
        basis = None
        if basis_force is not None:
            btotal, bstance = integral(t, basis_force, hs, end), integral(t, basis_force, hs, to)
            basis = {'total_impulse_Ns': btotal, 'stance_impulse_Ns': bstance,
                     'swing_impulse_Ns': btotal-bstance, 'fraction_of_total_contact_impulse': btotal/total}
        results.append(dict(c, duration_s=duration, nominal_stance_duration_s=stance_duration,
                            nominal_swing_duration_s=end-to, nominal_stance_fraction=stance_duration/duration,
                            total_impulse_Ns=total, stance_impulse_Ns=stance, swing_impulse_Ns=swing,
                            swing_fraction_of_total_impulse=swing/total,
                            stance_mean_force_N=stance/stance_duration, swing_mean_force_N=swing/(end-to),
                            measured_total_impulse_Ns=measured_total, measured_stance_impulse_Ns=measured_stance,
                            measured_swing_impulse_Ns=measured_swing,
                            stance_impulse_error_percent=100*(stance/measured_stance-1),
                            above20N_duration_stance_s=active_duration(t, f, hs, to),
                            above20N_duration_swing_s=active_duration(t, f, to, end),
                            depth=depth, basis07=basis, stance_phase_bins=bins))
    total_duration = sum(c['duration_s'] for c in results)
    stance_duration = sum(c['nominal_stance_duration_s'] for c in results)
    total = sum(c['total_impulse_Ns'] for c in results)
    stance = sum(c['stance_impulse_Ns'] for c in results)
    swing = sum(c['swing_impulse_Ns'] for c in results)
    aggregate = {'duration_s': total_duration, 'nominal_stance_duration_s': stance_duration,
                 'nominal_swing_duration_s': total_duration-stance_duration,
                 'nominal_stance_fraction': stance_duration/total_duration,
                 'total_impulse_Ns': total, 'stance_impulse_Ns': stance, 'swing_impulse_Ns': swing,
                 'swing_fraction_of_total_impulse': swing/total,
                 'mean_force_N': total/total_duration, 'stance_mean_force_N': stance/stance_duration,
                 'swing_mean_force_N': swing/(total_duration-stance_duration),
                 'measured_total_impulse_Ns': sum(c['measured_total_impulse_Ns'] for c in results),
                 'measured_stance_impulse_Ns': sum(c['measured_stance_impulse_Ns'] for c in results),
                 'measured_swing_impulse_Ns': sum(c['measured_swing_impulse_Ns'] for c in results),
                 'above20N_duration_stance_s': sum(c['above20N_duration_stance_s'] for c in results),
                 'above20N_duration_swing_s': sum(c['above20N_duration_swing_s'] for c in results)}
    aggregate['stance_impulse_error_percent'] = 100*(stance/aggregate['measured_stance_impulse_Ns']-1)
    aggregate['total_impulse_error_percent'] = 100*(total/aggregate['measured_total_impulse_Ns']-1)
    aggregate['depth'] = {name: {'mean_stance_m': sum(c['depth'][name]['mean_stance_m']*c['nominal_stance_duration_s'] for c in results)/stance_duration,
                                'mean_swing_m': sum(c['depth'][name]['mean_swing_m']*c['nominal_swing_duration_s'] for c in results)/(total_duration-stance_duration)} if v is not None else None for name, v in series.items()}
    if basis_force is not None:
        bi = {k:sum(c['basis07'][k] for c in results) for k in ('total_impulse_Ns','stance_impulse_Ns','swing_impulse_Ns')}
        bi['fraction_of_total_contact_impulse'] = bi['total_impulse_Ns']/total
        aggregate['basis07'] = bi
    else:
        aggregate['basis07'] = None
    shape = []
    for j in range(N_PHASE_BINS):
        rows = [c['stance_phase_bins'][j] for c in results]
        duration = sum(r['duration_s'] for r in rows)
        row = {'phase_start': j/N_PHASE_BINS, 'phase_end': (j+1)/N_PHASE_BINS,
               'duration_weighted_cycle_pool': True}
        for key in ('force_mean_N','force_mean_BW','measured_force_mean_N',*[name+'_mean_m' for name in series]):
            row[key] = sum(r[key]*r['duration_s'] for r in rows)/duration if rows[0][key] is not None else None
        shape.append(row)
    intervals = force_intervals(t, f)
    intervals = [r for r in intervals if r['offset_s'] >= cycles[0]['hs_s'] and r['onset_s'] <= cycles[-1]['next_hs_s']]
    return {'aggregate': aggregate, 'cycles': results, 'pooled_stance_phase_bins': shape,
            'force_above20N_intervals_intersecting_primary': intervals,
            'force_interval_source_support_s': [float(t[0]), float(t[-1])]}


def main():
    output = AUDIT/'phase_contact_budget.json'
    if output.exists():
        raise FileExistsError('Preserve existing phase diagnostics; version separately.')
    selection = json.loads((PAIR/'selection.json').read_text())
    replay = json.loads((PREVIOUS/'contact_replay.json').read_text())
    factorial = json.loads((MODEL/'factorial_geometry_fk.json').read_text())
    profile = json.loads(PROFILE.read_text())
    spheres = [s for s in profile['spheres'] if s['side'] == 'left']
    sphere = next(s for s in spheres if s['name'] == 'left_basis_07')
    material = sphere.get('material', profile['material'])
    if material['exponent'] != 1 or material['dissipation'] != 0 or profile['ground']['normal'] != [0,1,0]:
        raise ValueError('Analytical basis07 spring contract differs.')
    for name in ('residual_force_ratio','residual_force_penetration_gain_per_m','residual_force_penetration_rate_gain_s_per_m'):
        if sphere.get(name, [0,0,0])[1] != 0:
            raise ValueError('Basis07 vertical residual is not locked.')
    sources = {Path(__file__), PAIR/'selection.json', PREVIOUS/'contact_replay.json', MODEL/'factorial_geometry_fk.json', PROFILE}
    for subject in ('AB06','AB07'):
        sources.add(MODEL/f'baseline_{subject}.npz')
        sources.update(REPO/f['path'] for f in selection['subjects'][subject]['source_fingerprints'].values())
        run = REPO/f'results/{subject}_paired_prescribed_HS_9s_2026_10_07'
        sources.update([run/'receipt.json',run/'sim_outputs/sim_output_online_grf.sto'])
    before = {fp(p)['path']:fp(p)['sha256'] for p in sorted(sources)}
    subjects = {}
    for subject in ('AB06','AB07'):
        selected = selection['subjects'][subject]
        cycles = selected['primary_cycles']
        p = MODEL/f'baseline_{subject}.npz'
        advertised = next(x for x in factorial['baseline_exports'] if x['path'] == fp(p)['path'])
        if advertised != fp(p):
            raise AssertionError('Baseline NPZ differs from reviewed factorial export.')
        with np.load(p, allow_pickle=False) as z:
            if set(z.files) != {'times_s','coordinate_names','q','qdot','sphere_names','contact_centers_m','contact_velocity_m_s','aggregate_force_n'}:
                raise ValueError('NPZ schema differs.')
            arrays = {k:z[k].copy() for k in z.files}
        t, f = arrays['times_s'], arrays['aggregate_force_n'][:,1]
        if not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
            raise ValueError('Baseline timestamps are not finite/strictly increasing.')
        for k in ('q','qdot','contact_centers_m','contact_velocity_m_s','aggregate_force_n'):
            if not np.all(np.isfinite(arrays[k])) or len(arrays[k]) != len(t):
                raise ValueError('Baseline numerical arrays are not finite/aligned.')
        if arrays['sphere_names'].tolist() != [s['name'] for s in spheres] or arrays['contact_centers_m'].shape != (len(t),8,3):
            raise ValueError('Frozen sphere geometry/order differs.')
        boundaries = np.unique([c[k] for c in cycles for k in ('hs_s','to_s','next_hs_s')])
        if t[0] != boundaries[0] or t[-1] != boundaries[-1] or any(not np.any(t == b) for b in boundaries):
            raise ValueError('Exact frozen primary boundaries absent from exported grid.')
        idx = arrays['sphere_names'].tolist().index('left_basis_07')
        raw = sphere['radius']-(arrays['contact_centers_m'][:,idx,:]-np.array(profile['ground']['origin'])) @ np.array(profile['ground']['normal'])
        penetration = np.maximum(raw,0)
        smooth = .5*(raw+np.sqrt(raw*raw+material['smoothing']**2))
        # Production also smooths the dissipation multiplier. With d=0 it is
        # the constant smoothPositive(1), differing from 1 by2.5e-11 here.
        damping_constant = .5*(1+np.sqrt(1+material['smoothing']**2))
        basis_force = material['stiffness']*smooth*damping_constant
        mt, measured = storage(REPO/selected['source_fingerprints']['measured_GRF']['path'])
        if t[0] < mt[0] or t[-1] > mt[-1]:
            raise ValueError('Measured GRF lacks full primary support.')
        own = np.interp(t, mt, measured[selected['left_force_column']])
        filtered = budget(t,f,own,cycles,selected['BW_normalization_n'],
                          {'basis07_raw_signed_depth':raw,'basis07_positive_penetration':penetration}, basis_force)
        old = replay['subjects'][subject]['cases']['filtered_IK']['aggregate']
        errors = {'total_impulse_Ns':abs(filtered['aggregate']['total_impulse_Ns']-old['impulse_ns']),
                  'stance_impulse_Ns':abs(filtered['aggregate']['stance_impulse_Ns']-old['nominal_stance_impulse_ns']),
                  'basis07_impulse_Ns':abs(filtered['aggregate']['basis07']['total_impulse_Ns']-old['per_sphere_impulse_ns']['left_basis_07'])}
        if max(errors.values()) > 1e-8:
            raise AssertionError('Budget/analytical spring differs from original contact replay.')
        run = REPO/f'results/{subject}_paired_prescribed_HS_9s_2026_10_07'
        nt, native = storage(run/'sim_outputs/sim_output_online_grf.sto')
        if boundaries[0] < nt[0] or boundaries[-1] > nt[-1] or nt[0] < mt[0] or nt[-1] > mt[-1]:
            raise ValueError('Native/measured case lacks complete support.')
        native_measured = np.interp(nt,mt,measured[selected['left_force_column']])
        actual = budget(nt,native['left_force_y'],native_measured,cycles,selected['BW_normalization_n'],
                        {'basis07_raw_signed_depth':None,'basis07_positive_penetration':None,
                         'maximum_any_sphere_penetration':native['left_penetration']})
        subjects[subject] = {'receipt_status':json.loads((run/'receipt.json').read_text())['status'],
                             'BW_N':selected['BW_normalization_n'], 'filtered_IK':filtered,
                             'actual_saved':actual, 'original_filtered_replay_parity':errors}
    comparisons = {}
    for name in ('filtered_IK','actual_saved'):
        a,b = (subjects[s][name]['aggregate'] for s in ('AB06','AB07'))
        fbar = .5*(a['nominal_stance_fraction']+b['nominal_stance_fraction'])
        sbar = .5*(a['stance_mean_force_N']+b['stance_mean_force_N'])
        wbar = .5*(a['swing_mean_force_N']+b['swing_mean_force_N'])
        intensity = fbar*(b['stance_mean_force_N']-a['stance_mean_force_N'])
        duration = (sbar-wbar)*(b['nominal_stance_fraction']-a['nominal_stance_fraction'])
        swing = (1-fbar)*(b['swing_mean_force_N']-a['swing_mean_force_N'])
        delta = b['mean_force_N']-a['mean_force_N']
        if abs(delta-intensity-duration-swing) > 1e-9:
            raise AssertionError('Algebraic mean-force phase decomposition failed.')
        comparisons[name] = {'AB07_minus_AB06_mean_force_N':delta,
                             'AB07_over_AB06_mean_force_ratio':b['mean_force_N']/a['mean_force_N'],
                             'AB07_over_AB06_impulse_ratio':b['total_impulse_Ns']/a['total_impulse_Ns'],
                             'AB07_minus_AB06_stance_fraction':b['nominal_stance_fraction']-a['nominal_stance_fraction'],
                             'symmetric_product_identity_terms_N':{'stance_intensity':intensity,'stance_duration':duration,'swing_intensity':swing},
                             'identity_scope':'Exact algebraic decomposition of meanF = stance_fraction*stance_meanF + swing_fraction*swing_meanF; terms are not causal shares.'}
    aa,bb = (subjects[s]['filtered_IK']['pooled_stance_phase_bins'] for s in ('AB06','AB07'))
    depth_differences = np.array([b['basis07_raw_signed_depth_mean_m']-a['basis07_raw_signed_depth_mean_m'] for a,b in zip(aa,bb)])
    constant = float(depth_differences.mean())
    residual = depth_differences-constant
    raw_profile = {'phase_bin_differences_AB07_minus_AB06_m':depth_differences.tolist(),
                   'constant_shift_descriptive_mean_m':constant,
                   'residual_after_subtracting_constant_RMS_m':float(np.sqrt(np.mean(residual**2))),
                   'difference_minimum_m':float(depth_differences.min()),'difference_maximum_m':float(depth_differences.max()),
                   'interpretation':'Raw signed depth avoids zero clipping. A small residual relative to a constant offset is compatible with an offset-like shape; it does not uniquely establish registration causality. No profile fitting, force correction, time shift or significance test.'}
    after = {fp(p)['path']:fp(p)['sha256'] for p in sorted(sources)}
    if before != after:
        raise AssertionError('Protected source changed.')
    result = {'created_utc':datetime.now(timezone.utc).isoformat(),'instruction_check_token':'CMC_AGENT_OK_2026',
              'status':'PASS','helper':fp(Path(__file__)),'subjects':subjects,'pair_comparison':comparisons,
              'basis07_raw_stance_shape_constant_shift_diagnostic':raw_profile,
              'methodology':{'primary_cycles':'Same immutable3cycles per subject from paired selection; startup/rest of9s excluded.',
                             'stance_swing':'Measured-force20N oracle HS→TO is stance, TO→nextHS is swing, not online FSM or binary geometry events.',
                             'integration':'Piecewise-linear recorded force/depth curves, native1ms interior plus exactHS/TO/nextHS boundaries; trapezoidal integration with exact endpoint insertion.',
                             'basis07':'Depth=radius−signed plane distance; positive penetration=max(depth,0). Exact analytical normal forceK*smoothPositive(depth)*smoothPositive(1), exponent1,dissipation0, vertical residual locked. The final constant preserves the production smoothing of the zero-dissipation multiplier. No newFK.',
                             'shape':f'{N_PHASE_BINS} equal measured-stance phase bins. Per-cycle means and duration-weighted pooled means; normalization of phase only for descriptive shape, never main impulse/timing scoring.',
                             'actual':'Native saved force and maximum sphere penetration. Basis07 geometry was not recorded for actual, so sphere-specific fields stay null; no replacement by maximum penetration.',
                             'force_intervals':'All20N force intervals intersecting primary, with source-support/censored ends retained. These are force crossings, not HS/TO detector/FSM semantics.',
                             'dominant_sphere':'left_basis_07 selected from the previous frozen audit, not newly optimized on phase data.'},
              'limitations':['Experimental stance/swing belongs to the healthy measured limb; the simulated left contact belongs to virtual prosthetic hardware. Force in measured swing establishes temporal incompatibility, not a unique registration error or an experimental hardware fault.',
                             'No independent physical sole/floor data are added. Small marker residuals and offset-like depth shapes alone cannot identify a true floor or contact material law.',
                             'There is one trial per subject. Three cycles describe within-trial variability; no significance or population anatomical attribution is claimed.',
                             'The native secondary uses interpolation of measured force on the saved native grid before exactevent insertion; tiny integration differences from the earlier official paired comparator do not replace official percentages.',
                             'The force>20N interval view does not pair missed/extra online events or reconstruct canceled FSM HS.',
                             'AB06 original terminal callback failure is preserved; the primary physical windows are fully supported.'],
              'inputs_integrity':{'protected_count':len(before),'all_unchanged':True,'fingerprints_before':before,'fingerprints_after':after}}
    with output.open('x',encoding='utf-8') as out:
        json.dump(result,out,indent=2,allow_nan=False);out.write('\n')
    for subject,data in subjects.items():
        for name in ('filtered_IK','actual_saved'):
            a=data[name]['aggregate']
            print(subject,name,'stance/swingNs',a['stance_impulse_Ns'],a['swing_impulse_Ns'],'swingfraction',a['swing_fraction_of_total_impulse'],'stance_meanN',a['stance_mean_force_N'])
    print('rawdepth',raw_profile)
    print('PASS',output.relative_to(REPO))


if __name__ == '__main__':
    main()
