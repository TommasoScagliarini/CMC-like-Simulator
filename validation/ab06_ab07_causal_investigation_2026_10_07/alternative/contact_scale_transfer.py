"""Post-hoc saved-data test of a common vertical contact amplitude correction.

Fit only the first frozen AB06 cycle impulse; score the remaining AB06 cycles
and all AB07 cycles without a new FK, simulation or operational parameter edit.
This rejects or supports this specific scalar explanation, not a physical fit.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
AUDIT = Path(__file__).resolve().parent
PREVIOUS = REPO/'validation/ab06_ab07_preparation_vs_model_2026_10_07'


def fp(p):
    return {'path': p.relative_to(REPO).as_posix(), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}


def sums(cycles):
    simulated = sum(c['impulse_ns'] for c in cycles)
    measured = sum(c['measured_impulse_ns'] for c in cycles)
    return {'simulated_impulse_Ns': simulated, 'measured_impulse_Ns': measured,
            'simulated_over_measured': simulated/measured}


def main():
    replay_path = PREVIOUS/'contact_replay.json'
    profile_path = REPO/'online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json'
    paths = [replay_path, profile_path, Path(__file__)]
    before = {fp(p)['path']: fp(p)['sha256'] for p in paths}
    replay = json.loads(replay_path.read_text())
    profile = json.loads(profile_path.read_text())
    if replay['status'] != 'PASS' or not replay['contact_contract']['vertical_force_equals_sum_normal_force']:
        raise AssertionError('Requires validated frozen vertical force identity.')
    if profile['ground']['normal'] != [0.0, 1.0, 0.0]:
        raise AssertionError('The scalar proof requires a vertical ground normal.')
    for s in profile['spheres']:
        if s['side'] == 'left':
            for name in ('residual_force_ratio', 'residual_force_penetration_gain_per_m', 'residual_force_penetration_rate_gain_s_per_m'):
                if s.get(name, [0., 0., 0.])[1] != 0:
                    raise AssertionError('Vertical residual is not locked.')
    tests = {}
    for case in ('filtered_IK', 'actual'):
        a = replay['subjects']['AB06']['cases'][case]['cycles']
        b = replay['subjects']['AB07']['cases'][case]['cycles']
        alpha = a[0]['measured_impulse_ns']/a[0]['impulse_ns']
        scored = {}
        for subject, cycles in [('AB06_cycles_2_3', a[1:]), ('AB07_cycles_1_2_3', b)]:
            score = sums(cycles)
            score.update(scaled_over_measured=alpha*score['simulated_over_measured'],
                         scaled_impulse_error_percent=100*(alpha*score['simulated_over_measured']-1),
                         cycle_errors_percent=[100*(alpha*c['impulse_ns']/c['measured_impulse_ns']-1) for c in cycles],
                         frozen_nominal_windows=[{k:c[k] for k in ('hs_s','to_s','next_hs_s')} for c in cycles])
            scored[subject] = score
        tests[case] = {'fit_rule': 'First frozen AB06 primary cycle only, unweighted impulse equality.',
                       'fitted_common_amplitude_multiplier': alpha,
                       'fit_cycle': {k:a[0][k] for k in ('hs_s','to_s','next_hs_s')},
                       'out_of_fitting_cycle_checks': scored,
                       'descriptive_full_window_multiplier_required_AB06': 1/sums(a)['simulated_over_measured'],
                       'descriptive_full_window_multiplier_required_AB07': 1/sums(b)['simulated_over_measured']}
    after = {fp(p)['path']: fp(p)['sha256'] for p in paths}
    if before != after:
        raise AssertionError('Input changed during arithmetic diagnostic.')
    result = {'created_utc': datetime.now(timezone.utc).isoformat(), 'instruction_check_token': 'CMC_AGENT_OK_2026',
              'helper': fp(Path(__file__)), 'status': 'PASS', 'tests': tests,
              'scope': 'Scalar diagnostic on saved FK/native-pose contact integrals only; no runtime, refit, file edit or new FK.',
              'scalar_proof': 'At fixed pose/velocity with vertical plane and locked vertical residual, multiplying every left stiffness by alpha multiplies vertical normal force and its impulse by alpha. No mass or BW enters this force law.',
              'interpretation': ['A common amplitude error would transfer an AB06-fitted scalar to AB07. This specific explanation can be tested without changing inputs.',
                                 'An AB06 fit that preserves later AB06 impulses but worsens AB07 cannot supply a common scalar fix. It does not distinguish geometry, reference motion and contact basis identification.',
                                 'The tested profile was calibrated on short AB06 saved-forward states and metadata still requires forward validation. Identical hardware does not make that fitted force basis an independently identified material law.',
                                 'The experimental left force is from a healthy subject, not from this virtual hardware. Matching it is a benchmark objective; even a successful numerical fit would not establish hardware realism.'],
              'limitations': ['Post-hoc diagnostic selected after observing the saved experiments; out-of-fitting-cycle checks are not prospective external validation.',
                              'The actual case freezes observed poses. A new dynamic rollout after stiffness adjustment could follow different poses; no such effect is estimated.',
                              'No waveform fit, moment calibration or per-sphere tuning is performed; force/BW scaling is not proposed as an operational change.',
                              'Exact TO knots in the original replay cause tiny denominator differences from the official paired comparison; official observed percentages remain unchanged.'],
              'sources': {'replay': fp(replay_path), 'profile': fp(profile_path), 'metadata': profile.get('metadata')},
              'inputs_integrity': {'all_unchanged': True, 'fingerprints_before': before, 'fingerprints_after': after}}
    with (AUDIT/'contact_scale_transfer.json').open('x', encoding='utf-8') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    for name, test in tests.items():
        print(name, 'alpha', test['fitted_common_amplitude_multiplier'],
              {k: v['scaled_impulse_error_percent'] for k, v in test['out_of_fitting_cycle_checks'].items()})


if __name__ == '__main__':
    main()
