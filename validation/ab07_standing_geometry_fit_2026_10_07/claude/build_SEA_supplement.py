"""Send only selected, already recorded actuator aggregates to the existing audit."""
import datetime as dt
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

AUDIT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent
j = json.loads((AUDIT / 'analysis/lean_numeric_summary.json').read_text())
models = {
    'STANDING_FOURROI_AB07': AUDIT / 'full_support_preparation/cases/STANDING_FOURROI_AB07/AB07_SEASEA_marker_calibrated.osim',
    'STANDING_AB07': AUDIT / 'preparation/cases/STANDING_AB07/AB07_SEASEA_marker_calibrated.osim',
    'PYLON400_REFERENCE': AUDIT / 'preparation/cases/PYLON400_REFERENCE/AB07_SEASEA_marker_calibrated.osim',
}
packet = {'native_sample_dt_s': .001, 'motor_clamp_Nm': 500, 'cases': {}}
for name, c in j['cases'].items():
    xml = ET.parse(models[name]).getroot()
    case = {'physical_episode_duration_s': c['physical_episode_duration_s'],
            'recorded_native_duration_s': c['recorded_native_duration_s'], 'actuators': {}}
    for sea, s in c['SEA_all_recorded'].items():
        node = xml.find('.//ForceSet/objects/SeriesElasticActuator[@name="' + sea + '"]')
        assert node is not None
        item = {'optimal_force_Nm': float(node.findtext('optimal_force')),
                'impedance_mode': node.findtext('Impedence').strip().lower() == 'true',
                'recorded_sample_count': s['tau_motor_Nm']['samples']}
        for key in ('normalized_control_saturation', 'motor_torque_clamp',
                    'inner_integral_torque_clamp', 'raw_motor_torque_excess_above_500_Nm',
                    'tau_spring_Nm', 'tau_motor_Nm', 'control'):
            item[key] = s[key]
        item['command_limit_evidence'] = {k: v for k, v in s['command_limit_evidence'].items()
                                          if k not in ('source',)}
        case['actuators'][sea] = item
    packet['cases'][name] = case
seal = {'created_utc': dt.datetime.now(dt.timezone.utc).isoformat(),
        'own_previous_seal_sha256': hashlib.sha256((AUDIT/'root_final_findings_before_claude.json').read_bytes()).hexdigest(),
        'recorded_measurements': packet,
        'own_interpretation_before_external_supplement': [
            'The knee normalized reference is limited by F_opt=100Nm; the rotor torque has a separate 500Nm clamp.',
            'The full-support knee command reaches abs(u)>=0.999 in118/160 samples; reconstructed unclipped command exceeds1 in117/160, reaching -2.22647.',
            'Motor500Nm clamp lasts2 sampled milliseconds, not the entire118ms command-limited prefix.',
            'This demonstrates a command-limited startup for the new preparation, but does not isolate its cause among mounting, own IK, contact and tracking.',
            'No actuator or detector setting has been changed, and no further integration has been performed.'
        ]}
with (AUDIT/'root_SEA_numerical_addendum_before_claude.json').open('x') as f:
    json.dump(seal, f, indent=2, allow_nan=False); f.write('\n')
prompt = ('Additional already-recorded numerical actuator measurements for the same cases. '
          'Complete your audit using these aggregates, distinguish the normalized command limit from the motor torque clamp, '
          'and qualify any causal inference. No tools or file access. Reply in Italian, at most300words.\n\n')
payload = prompt + json.dumps(packet, separators=(',', ':'), allow_nan=False) + '\n'
target = OUT/'SEA_numerical_supplement.txt'
with target.open('x') as f: f.write(payload)
scope = json.loads((OUT/'scope_receipt.json').read_text())
with (Path(scope['isolated_cwd'])/target.name).open('x') as f: f.write(payload)
receipt = {'created_utc': dt.datetime.now(dt.timezone.utc).isoformat(),
           'session_id': scope['session_id'], 'isolated_cwd': scope['isolated_cwd'],
           'payload_bytes': len(payload.encode()), 'payload_sha256': hashlib.sha256(payload.encode()).hexdigest(),
           'scope': 'Existing limited consent: numerical inputs and aggregate controls only',
           'code_raw_series_file_access_local_conclusions_sent': False,
           'local_addendum_sha256': hashlib.sha256((AUDIT/'root_SEA_numerical_addendum_before_claude.json').read_bytes()).hexdigest()}
with (OUT/'SEA_supplement_scope_receipt.json').open('x') as f:
    json.dump(receipt, f, indent=2); f.write('\n')
print(json.dumps(receipt))
