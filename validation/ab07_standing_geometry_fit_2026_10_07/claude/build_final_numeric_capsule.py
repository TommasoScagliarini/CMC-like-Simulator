"""Export selected measured numbers/control aggregates, without source or local verdicts."""
import datetime as dt
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
AUDIT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent
summary_path = AUDIT / 'analysis/lean_numeric_summary.json'
j = json.loads(summary_path.read_text())
seal = AUDIT / 'root_final_findings_before_claude.json'
assert seal.exists()
packet = {
    'fixed_controls': {
        'seed':123, 'episode_s':6.11, 'setup_horizon_s':9.0,
        'reset_s':12.591740519774012, 'native_dt_s':.001,
        'detector_recalibrations':0, 'force_profile_changes':0,
        'markers':12, 'full_IK_frames':28612, 'full_IK_range_s':[10.36,153.415],
        'joint_guard_knee_rad':[-2.0,0.0], 'joint_guard_ankle_rad':[-.9,.9],
        'per_segment_wall_guard_s':60, 'force20N_event_threshold_N':20,
        'HS_onset_matching_ms':120, 'TO_onset_matching_ms':180,
        'confirmed_delivered_matching_ms':50,
        'SEA_impedance_mode':False, 'SEA_motor_clamp_Nm':500,
        'neural_policy_loaded':False, 'own_full_IK_reference_each_case':True,
        'force_plane_Y_mm':-9.50395476, 'binary_plane_Y_mm':14.8208231,
        'static_mesh_ground_Y_mm':0.0,
    },
    'cases':{},
    'common_recorded_startup_window_s':j['secondary_common_observed_prefix']['range_s'],
    'common_recorded_startup_duration_s':j['secondary_common_observed_prefix']['duration_s'],
    'common_recorded_startup':{},
    'standing_geometry':{},
    'static_force_spheres':j['static_force_sphere_geometry'],
    'fresh400':{'17_STO_byte_identical':True,'611_trace_rows_byte_identical':True,'event_CSV_byte_identical':True},
}
for name,c in j['cases'].items():
    r = c['runtime']
    item = {k:c[k] for k in ('runtime_status','primary_coverage','primary_requested_range_s','mass_kg','BW_N',
                            'physical_episode_duration_s','recorded_native_duration_s','distal_mount_length_mm',
                            'static_ankle_reference_offset_deg','full_episode_FSM_counters')}
    item['steps'] = r['steps']; item['end_reason'] = r['end_reason']
    item['primary_phase'] = {}
    for channel, ph in c['primary_phase'].items():
        reduced = {'coverage':ph['coverage'],'events':None}
        if ph.get('events') is not None:
            reduced['events'] = {kind:{clock:{k:v for k,v in values.items() if isinstance(v,(int,float))}
                                      for clock,values in data.items()} for kind,data in ph['events'].items()}
        item['primary_phase'][channel] = reduced
    s = c['primary_full_three_cycle_statistics']
    item['primary_three_cycle_stats'] = None if s is None else {k:s[k] for k in (
        'observed_range_s','observed_duration_s','left_actual_mean_N','left_momentum_required_mean_N',
        'left_measured_mean_N','right_measured_mean_N','pelvis_vertical_reserve',
        'prosthetic_knee_RMS_deg','prosthetic_ankle_RMS_deg','left_actual_penetration_max_mm')}
    packet['cases'][name] = item
    common = j['secondary_common_observed_prefix']['cases'][name]
    packet['common_recorded_startup'][name] = {k:v for k,v in common.items() if k!='actual_minus_own_served_position'}
    packet['common_recorded_startup'][name]['knee_ankle_tracking'] = {k:common['actual_minus_own_served_position'][k]
                                                                  for k in ('pros_knee_angle','pros_ankle_angle')}
for name,g in j['standing_geometry'].items():
    packet['standing_geometry'][name] = {k:g[k] for k in ('mounts','ROI_min_Y_m','whole_STL_min_Y_m','actual_contact_count')}
packet['final_knee_angle_deg']={'STANDING_AB07':.39735241899048346,'STANDING_FOURROI_AB07':.14440149807934796}
prompt = ('Audit independently these measured numerical results and aggregate controls for the standing-fit AB07 versus the400mm comparator. '
          'Compare reserve effort, detector evidence, static geometry and runtime coverage. Distinguish established findings from causal hypotheses. '
          'No tools or file access; reply in Italian, at most600words.\n\n')
payload = prompt + json.dumps(packet,separators=(',',':'),allow_nan=False) + '\n'
target = OUT / 'final_numerical_capsule.txt'
with target.open('x') as f:f.write(payload)
scope = json.loads((OUT/'scope_receipt.json').read_text())
isolated = Path(scope['isolated_cwd'])
with (isolated/target.name).open('x') as f:f.write(payload)
receipt = {'created_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
           'payload_bytes':len(payload.encode()),'payload_sha256':hashlib.sha256(payload.encode()).hexdigest(),
           'summary_sha256':hashlib.sha256(summary_path.read_bytes()).hexdigest(),
           'local_seal_sha256':hashlib.sha256(seal.read_bytes()).hexdigest(),
           'scope':'Persistent limited consent: selected numerical inputs and aggregate control outcomes only',
           'code_raw_time_series_file_access_local_conclusions_sent':False,
           'session_id':scope['session_id'],'isolated_cwd':str(isolated)}
with (OUT/'final_capsule_scope_receipt.json').open('x')as f:json.dump(receipt,f,indent=2);f.write('\n')
print(json.dumps(receipt,indent=2))
