"""Offline episode classification, temporal windows and duration denominators."""
import copy
import json
import math

import pytest

from unified_MLP_39.validation.analyze_episode_telemetry import analyze, summarize_episode


def record(uid='failure', full=False):
    tail = []
    for step in range(1, 5):
        tail.append({'episode_step': step, 'time_s': step * .01,
                     'policy_outputs': {'weights_seq_no': 40 if step < 3 else 41},
                     'phase_fsm': {'state_name': 'SWING'},
                     'reward_terms': {'grf_penetration_m': 0 if step < 3 else .031,
                                      'grf_penetration_loss': 0 if step < 3 else 1,
                                      'policy_action_clip_fraction': .5},
                     'joints': {'pros_knee_angle': .1, 'pros_ankle_angle': -.2},
                     'imitation_target': {'q': {'pros_knee_angle': 0, 'pros_ankle_angle': 0}}})
    return {'schema': 'unified39_episode_telemetry_v1', 'episode_uid': uid, 'worker_index': 7,
            'episode_steps': 4, 'observed_steps': 4, 'duration_s': .04, 'last_time_s': .04,
            'end_reason': 'episode_time_limit' if full else 'penetration_guard',
            'terminated': not full, 'truncated': full, 'partial_episode': False,
            'missing_steps': 0, 'rows_with_missing_fields': 0,
            'event_counts': {'action_clip_rows': 2, 'resync_rows': 1, 'invalid_event_rows': 0},
            'last_resync': {'episode_step': 2}, 'last_hs_cancel': None,
            'peaks': {'penetration_m': {'value': .031, 'episode_step': 3, 'phase_fsm_state': 'SWING'},
                      'hs_cancelled_count': {'value': 0}},
            'joint_error_accumulators': {'pros_knee_angle': {'count': 4, 'sum_squared_rad2': .04},
                                         'pros_ankle_angle': {'count': 4, 'sum_squared_rad2': .16}},
            'policy_versions': {'first': 40, 'last': 41, 'switch_count': 1,
                                'rows_with_version': 4, 'rows_without_version': 0}, 'tail': tail}


def test_known_episode_errors_onset_and_recent_events():
    row = summarize_episode(record())
    assert row['outcome'] == 'failed'
    assert row['joint_errors']['knee']['rmse_observed_episode_deg'] == pytest.approx(.1 * 180 / math.pi)
    assert row['joint_errors']['ankle']['terminal_signed_deg'] == pytest.approx(-.2 * 180 / math.pi)
    assert row['event_rates']['action_clip_rows_per_100_observed_steps'] == 50
    assert row['policy_versions']['switch_steps_observed_in_tail'] == [3]
    assert row['terminal_proximity_steps']['5'] == {'resync': 'yes', 'hs_cancel': 'no', 'policy_switch': 'yes'}
    assert row['penetration_onsets_in_tail']['above_30_mm']['left_censored'] is False
    assert row['penetration_onsets_in_tail']['above_30_mm']['steps_before_end'] == 1


def test_tail_does_not_invent_whole_episode_stability_or_50step_coverage():
    source = record()
    source['episode_steps'] = source['observed_steps'] = 100
    source.pop('policy_versions')
    source['tail'] = [dict(copy.deepcopy(source['tail'][-1]), episode_step=step) for step in range(51, 101)]
    row = summarize_episode(source)
    assert row['policy_versions']['stability'] == 'unknown'
    assert row['policy_versions']['first'] is None
    assert row['terminal_proximity_steps']['25']['policy_switch'] == 'no'
    assert row['terminal_proximity_steps']['50']['policy_switch'] == 'unknown'
    assert row['penetration_onsets_in_tail']['above_30_mm']['left_censored'] is True


def test_streaming_aggregation_full_failed_and_dedup(tmp_path):
    directory, output = tmp_path / 'input', tmp_path / 'result'
    directory.mkdir()
    failed, full = record(), record('complete', True)
    # Same event count, longer exposure: pooled clipping is 4/12, not 50%.
    full['observed_steps'] = full['episode_steps'] = 8
    full['duration_s'] = .08
    (directory / 'episodes_w7_part0.jsonl').write_text('\n'.join(json.dumps(item) for item in [failed, full, failed]))
    result = analyze(directory, output)
    assert result['records_analyzed'] == 2 and result['duplicate_records'] == 1
    assert result['workers_observed'] == ['7']
    assert result['groups']['all']['completion_fraction'] == .5
    assert result['groups']['all']['pooled_event_rows_per_100_observed_steps']['action_clip_rows'] == pytest.approx(100 / 3)
    assert len((output / 'per_episode.jsonl').read_text().splitlines()) == 2
    assert (output / 'summary.md').is_file()
