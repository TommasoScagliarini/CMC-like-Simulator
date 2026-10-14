"""Seal independent reference-only supplement conclusions without new FK."""
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('supplement_seal_metrics', HERE / 'contact_replay_supplement_pose.py')
metrics = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = metrics
spec.loader.exec_module(metrics)
fp, read, ROOT = metrics.fp, metrics.read, metrics.ROOT


def main():
    output = HERE / 'scientific_supplement_seal.json'
    if output.exists():
        raise FileExistsError('Preserve prior independent seal')
    data = read(HERE / 'contact_replay_supplement.json')
    if data['status'] != 'PASS_supplement_reference_FK_only' or not data['inputs_integrity']['all_unchanged']:
        raise AssertionError('Require completed immutable supplementary analysis')
    before = dict(data['inputs_integrity']['after'])
    for path in (Path(__file__), HERE / 'contact_replay_supplement.json', HERE / 'contact_replay_stdout.log', HERE / 'build_all_stdout.log'):
        item = fp(path)
        before[item['path']] = item['sha256']
    for path, sha in before.items():
        if fp(ROOT / path)['sha256'] != sha:
            raise AssertionError('Consumed supplementary or main source changed')
    scalar, baseline = data['scalar_comparison'], data['immutable_main_reference_scalars']['P0_A0']
    contrasts = {name: {key: row[key] - baseline[key] for key in (
        'contact_mean_force_N', 'contact_total_impulse_Ns', 'basis07_positive_penetration_mean_stance_m',
        'ankle_center_Y_mean_stance_m', 'left12_marker_mean_stance_RMS_m', 'reference_external_support_impulse_Ns')}
        for name, row in scalar.items()}
    result = {'sealed_utc': datetime.now(timezone.utc).isoformat(), 'instruction_check_token': 'CMC_AGENT_OK_2026',
              'status': 'SEALED_independent_three_fixed_angular_supplements', 'new_Claude_audit_content_read': False,
              'scope': 'Three fresh coherent marker-calibration/fullIK cases and reference FK only; no native dynamics by this agent',
              'protocol': fp(HERE / 'supplement_protocol.json'), 'contact_replay': fp(HERE / 'contact_replay_supplement.json'),
              'preparation_batch': fp(HERE / 'batch_receipt.json'), 'scalar_comparison': scalar,
              'differences_from_immutable_short_length_zero_angle': contrasts,
              'conclusions': [
                  {'finding': 'At exact shortest length, predeclared negative static-angle preparation produces larger reference virtual contact than the zero-angle case.',
                   'means_N': {n: c['contact_mean_force_N'] for n, c in scalar.items()},
                   'interpretation': 'This is a conditional geometric preparation response, not a dynamic support or clinical validation.'},
                  {'finding': 'Marker fit and functional ankle-center height barely change while positive penetration and contact increase.',
                   'marker_stance_RMS_mm': {n: c['left12_marker_mean_stance_RMS_m'] * 1000 for n, c in scalar.items()},
                   'ankle_center_stance_Y_mm': {n: c['ankle_center_Y_mean_stance_m'] * 1000 for n, c in scalar.items()},
                   'interpretation': 'Consistent with the previously proven static-angle marker-objective rotation symmetry; it does not independently determine foot mechanical pitch.'},
                  {'finding': 'The reference vertical momentum budget still requires additional external support for all three cases.',
                   'support_impulse_Ns': {n: c['reference_external_support_impulse_Ns'] for n, c in scalar.items()},
                   'interpretation': 'Hypothetical reference requirement, not an actual reserve output or forward dynamic failure.'},
                  {'finding': 'All three supplements and all14 main cells were fixed before their respective force outcomes; no force fitting or outcome-based selection.',
                   'interpretation': 'Root-owned prospective18-run protocol provides the dynamic test; a failure of these finite candidates would not prove universal impossibility.'}],
              'limitations': data['limitations'], 'native_dynamics_executed_by_this_agent': False,
              'source_integrity': {'consumed_count': len(before), 'before': before,
                                   'after': {p: fp(ROOT / p)['sha256'] for p in before}, 'all_unchanged': True}}
    if result['source_integrity']['before'] != result['source_integrity']['after']:
        raise AssertionError('Source changed during seal')
    with output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'seal': fp(output), 'consumed_count': len(before), 'all_unchanged': True}, allow_nan=False))


if __name__ == '__main__':
    main()
