"""Publish numerical aggregates only; do not transmit source paths or local verdicts."""
import datetime as dt
import hashlib
import json
from pathlib import Path
import uuid

ROOT = Path(__file__).resolve().parents[3]
AUDIT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent
OUT.mkdir(exist_ok=True)
source = AUDIT / 'geometry/fit_geometry_receipt_v2.json'
# The first serializer stopped only after these complete numerical blocks.
# Preserve that artifact and identify the restricted read; pipeline uses a reviewed recovery.
raw = source.read_text()
geo = json.loads(raw.split(',\n  "checks": {', 1)[0] + '\n}')
baseline_source = ROOT / 'validation/ab07_pylon_necessity_screening_2026_10_07/dynamics_analysis/P3_A1/metrics.json'
baseline = json.loads(baseline_source.read_text())
packet = {
    'units': {'length':'m','angle':'deg','force':'N','reserve_work':'J','time':'s'},
    'healthy_static_representative_coordinates': geo['reference_coordinate_maps']['healthy_deg_m'],
    'prepared_static_representative_coordinates': geo['reference_coordinate_maps']['prepared_deg_m'],
    'static_acquisition_rows': geo['source_static_rows'],
    'static_acquisition_time_s': geo['source_static_time_s'],
    'ground_plane_Y_m': 0.0,
    'mounts_m': geo['mounts'],
    'static_angular_offsets_deg': geo['static_angular_offsets_deg'],
    'geometry': geo['geometry'],
    'native_transform_error': geo['native_parity'],
    'support_regions': geo['support_regions'],
    'comparator': {
        'distal_pylon_length_m':0.4,
        'static_ankle_offset_deg':4.730648501,
        'historical_three_cycle_left_contact_mean_N':304.85818417671163,
        'historical_left_required_mean_N':285.803277829613,
        'historical_pelvis_vertical_reserve_mean_N':-19.062544501518595,
        'historical_pelvis_vertical_reserve_RMS_N':152.66237238700802,
        'historical_pelvis_vertical_reserve_absolute_work_J':19.494591315026696,
        'historical_actor_valid_cycles':3,
        'historical_actor_invalid_cycles':0,
        'historical_force20N_HS_count':6,
        'historical_force20N_TO_count':5,
        'historical_force20N_matched_HS_count':3,
        'historical_force20N_matched_TO_count':3,
    },
    'fixed_controls': {
        'marker_count':12, 'full_marker_IK_time_s':[10.36,153.415],
        'body_mass_kg':58.8075, 'gravity_m_s2':9.80665,
        'episode_time_s':6.11, 'runtime_setup_horizon_s':9.0,
        'reset_time_s':12.591740519774012,
        'comparison_window_s':[14.087704688172044,18.69078019047619],
        'integration_step_s':0.001, 'seed':123,
        'force_event_threshold_N':20,
        'HS_match_window_s':0.12, 'TO_match_window_s':0.18,
        'primary_force_virtual_plane_Y_m':-0.00950395476,
        'binary_phase_virtual_plane_Y_m':0.0148208231,
        'detector_recalibration_count':0,
        'SEA_motor_torque_clamp_Nm':500,
        'new_forward_run_count_so_far':0,
    },
}
prompt = ('Independent numerical audit. Assess the standing prosthetic mounting dimensions and the proposed paired comparison with the 400 mm reference from the numbers below. '
          'Separate geometric deductions from hypotheses that require new forward results. Explain what knee height and heel/forefoot support establish, including lateral support. '
          'Assess which reserve and phase measurements are needed. Detector calibration is fixed. No tools or file access. Reply in Italian, at most 700 words.\n\n')
payload = prompt + json.dumps(packet, indent=2) + '\n'
target = OUT / 'geometry_numerical_capsule.txt'
target.write_text(payload)
session = str(uuid.uuid4())
isolated = Path('/private/tmp') / ('cmc_ab07_standing_fit_' + session)
isolated.mkdir()
(isolated / target.name).write_text(payload)
receipt = {
    'created_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
    'session_id':session,'isolated_cwd':str(isolated),
    'requested_model':'claude-fable-5-1','effort':'max','tools':[],
    'authorization':'Persistent explicit limited user consent: numerical inputs and aggregate controls only',
    'payload_sha256':hashlib.sha256(payload.encode()).hexdigest(),'payload_bytes':len(payload.encode()),
    'numeric_geometry_source':str(source.relative_to(ROOT)),
    'numeric_geometry_source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
    'partial_serializer_read':'Only complete blocks before checks; original partial preserved; no claim of final receipt completion',
    'content_read_policy':'Do not read visible external conclusions until own local findings have been sealed; metadata only first',
    'code_raw_series_file_access_local_conclusions_sent':False,
}
(OUT / 'scope_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt,indent=2))
