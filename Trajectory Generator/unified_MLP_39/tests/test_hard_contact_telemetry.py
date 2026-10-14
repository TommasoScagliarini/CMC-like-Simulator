"""Telemetry keeps a hard terminal contact distinct from a complete policy step."""
import copy
import json
from types import SimpleNamespace

import pytest

from unified_MLP_39.episode_telemetry import EpisodeTelemetry
from unified_MLP_39.validation.analyze_episode_telemetry import summarize_episode


class Episode:
    """The small episode interface consumed by the read-only telemetry observer."""
    def __init__(self):
        self.id_ = 'hard-contact'
        self.t = 0
        self.infos = [info(12.99)]
        self.rewards = []
        self.extra_model_outputs = {}
        self.is_terminated = False
        self.is_truncated = False

    def add(self, payload, reward, terminated=False, truncated=False):
        self.infos.append(payload)
        self.rewards.append(reward)
        self.t += 1
        self.is_terminated = terminated
        self.is_truncated = truncated

    def get_infos(self, index):
        return self.infos[index]

    def get_actions(self, index):
        return [.2, -.3]

    def get_observations(self, index):
        return [0.] * 88

    def get_rewards(self, index):
        return self.rewards[index]

    def get_return(self):
        return sum(self.rewards)


def info(time_s, *, completed=None):
    result = {
        'time': time_s, 'target_time': time_s, 'episode_start_offset_s': 1.,
        'observation': {'pros_knee_angle': .1, 'pros_ankle_angle': -.2},
        'imitation_target': {'q': {'pros_knee_angle': 0., 'pros_ankle_angle': 0.}},
        'phase_fsm': {'state_id': 1., 'state_name': 'STANCE_AFTER_HS', 'resync_count': 0.},
        'reward_terms': {'grf_penetration_m': .012, 'grf_penetration_loss': 0.},
        'online_gait': {'sides': {'left': {'normal_force_n': 500.}}},
    }
    if completed is not None:
        result['segment_completed'] = completed
        result['reward_terms']['segment_completed'] = float(completed)
        result['reward_terms']['grf_hard_contact_available'] = 0.
    return result


def hard_info():
    payload = info(13., completed=False)
    payload['end_reason'] = 'grf_penetration_hard'
    payload['target_time'] = 13.01
    # These coordinates do not constitute a completed segment and deliberately
    # differ greatly, so including them corrupts the episode error denominator.
    payload['observation'] = {'pros_knee_angle': 9., 'pros_ankle_angle': -9.}
    payload['reward_terms'].update(grf_penetration_m=.0307, grf_penetration_loss=1.78,
                                   grf_hard_contact_available=1., grf_ankle_moment_flip_available=1.,
                                   grf_ankle_moment_flip_tau_nm=-60., grf_ankle_moment_flip_loss=25.)
    payload['terminal_contact'] = {
        'available': True, 'source': 'hard_penetration_guard', 'time_s': 13.004,
        'side': 'left', 'penetration_m': .0307, 'limit_m': .03,
        'online_grf': {'left': {'normal_force': 1032., 'penetration': .0307}},
    }
    return payload


def test_hard_tail_and_episode_peak_preserve_distinct_physical_time(tmp_path):
    observer = EpisodeTelemetry(str(tmp_path))
    episode = Episode()
    runner = SimpleNamespace(worker_index=2, _weights_seq_no=60)
    episode.add(info(13., completed=True), .8)
    observer.observe(episode, env_runner=runner)
    payload = hard_info()
    original = copy.deepcopy(payload)
    episode.add(payload, -3.2, terminated=True)
    observer.observe(episode, env_runner=runner)
    # Snapshot must be detached from mutable environment-owned dictionaries.
    payload['terminal_contact']['online_grf']['left']['normal_force'] = 0.
    observer.finish(episode, env_runner=runner)
    record = json.loads(next(tmp_path.glob('episodes_*.jsonl')).read_text())
    row = record['tail'][-1]
    assert row['segment_completed'] is False
    assert row['time_s'] == 13.
    assert row['terminal_contact'] == original['terminal_contact']
    assert row['reward_terms']['grf_hard_contact_available'] == 1.
    assert row['reward_terms']['segment_completed'] == 0.
    peak = record['peaks']['penetration_m']
    assert peak['value'] == .0307
    assert peak['time_s'] == 13.004
    assert peak['policy_boundary_time_s'] == peak['phase_fsm_time_s'] == 13.
    assert peak['source'] == 'hard_penetration_guard'
    assert peak['segment_completed'] is False
    for name, value in [('normal_force_n', 1032.), ('moment_flip_loss', 25.), ('ankle_moment_min_nm', -60.)]:
        contact_peak = record['peaks'][name]
        assert contact_peak['value'] == value
        assert contact_peak['time_s'] == 13.004
        assert contact_peak['policy_boundary_time_s'] == contact_peak['phase_fsm_time_s'] == 13.
        assert contact_peak['source'] == 'hard_penetration_guard'
    assert record['last_time_s'] == 13.
    assert record['duration_s'] == pytest.approx(.01)
    assert record['terminal_contact_time_s'] == 13.004
    assert record['episode_steps'] == record['observed_steps'] == 2
    assert record['completed_segment_rows'] == 1
    assert record['partial_segment_rows'] == 1
    assert record['unknown_segment_rows'] == 0
    assert record['end_reason'] == 'grf_penetration_hard'
    assert record['observed_reward_sum'] == record['episode_return'] == pytest.approx(-2.4)
    assert record['event_counts']['penetration_penalty_rows'] == 1
    for joint, expected in [('pros_knee_angle', .01), ('pros_ankle_angle', .04)]:
        error = record['joint_error_accumulators'][joint]
        assert error['count'] == error['completed_segment_count'] == 1
        assert error['unknown_segment_count'] == 0
        assert error['sum_squared_rad2'] == pytest.approx(expected)
    assert observer.metrics.get('step_errors', 0) == observer.metrics.get('write_errors', 0) == 0
    analyzed = summarize_episode(record)
    assert analyzed['terminal_segment_completed'] is False
    assert analyzed['terminal_kinematic_sample_status'] == 'incomplete_segment'
    assert analyzed['terminal_time_s'] == 13.
    assert analyzed['terminal_contact_time_s'] == analyzed['peak_penetration_time_s'] == 13.004
    assert analyzed['tail_partial_segment_rows'] == 1
    assert analyzed['peak_penetration_source'] == 'hard_penetration_guard'
    onset = analyzed['penetration_onsets_in_tail']['above_30_mm']
    assert onset['time_s'] == 13.004
    assert onset['policy_boundary_time_s'] == 13.
    for joint, expected in [('knee', .1), ('ankle', .2)]:
        error = analyzed['joint_errors'][joint]
        assert error['terminal_signed_deg'] is None
        assert error['terminal_abs_deg'] is None
        assert error['rmse_tail_deg'] == pytest.approx(expected * 180 / 3.141592653589793)
        assert error['rmse_observed_episode_deg'] == error['rmse_tail_deg']
        assert error['tail_error_rows'] == error['observed_error_rows'] == 1


@pytest.mark.parametrize('completed', [True, None])
def test_regular_and_legacy_rows_keep_boundary_peak_and_error_denominator(tmp_path, completed):
    observer = EpisodeTelemetry(str(tmp_path))
    episode = Episode()
    episode.add(info(13., completed=completed), .8, truncated=True)
    original = copy.deepcopy(episode.infos)
    observer.observe(episode)
    observer.finish(episode)
    record = json.loads(next(tmp_path.glob('episodes_*.jsonl')).read_text())
    assert episode.infos == original
    assert record['tail'][0]['segment_completed'] is completed
    assert record['tail'][0]['terminal_contact'] == {}
    assert record['peaks']['penetration_m']['time_s'] == 13.
    assert record['peaks']['penetration_m']['value'] == .012
    assert record['terminal_contact_time_s'] is None
    assert record['partial_segment_rows'] == 0
    assert record['completed_segment_rows'] == int(completed is True)
    assert record['unknown_segment_rows'] == int(completed is None)
    error = record['joint_error_accumulators']['pros_knee_angle']
    assert error['count'] == 1
    assert error['completed_segment_count'] == int(completed is True)
    assert error['unknown_segment_count'] == int(completed is None)
    analyzed = summarize_episode(record)
    assert analyzed['terminal_kinematic_sample_status'] == ('completed_segment' if completed else 'legacy_unknown')
    assert analyzed['joint_errors']['knee']['terminal_signed_deg'] == pytest.approx(.1 * 180 / 3.141592653589793)
    assert analyzed['joint_errors']['knee']['tail_error_rows'] == 1


def test_unavailable_contact_does_not_invent_substep_peak(tmp_path):
    observer = EpisodeTelemetry(str(tmp_path))
    episode = Episode()
    payload = hard_info()
    payload['terminal_contact'] = {'available': False, 'source': 'hard_penetration_guard'}
    payload['reward_terms'].update(grf_penetration_m=0., grf_hard_contact_available=0.,
                                   grf_ankle_moment_flip_available=0.,
                                   grf_ankle_moment_flip_tau_nm=0., grf_ankle_moment_flip_loss=0.)
    episode.add(payload, -2., terminated=True)
    observer.observe(episode)
    observer.finish(episode)
    record = json.loads(next(tmp_path.glob('episodes_*.jsonl')).read_text())
    assert record['terminal_contact_time_s'] is None
    for name in ('penetration_m', 'normal_force_n', 'moment_flip_loss', 'ankle_moment_min_nm'):
        assert name not in record['peaks']
    assert record['joint_error_accumulators'] == {}
    assert record['partial_segment_rows'] == 1
    analyzed = summarize_episode(record)
    assert analyzed['joint_errors']['knee']['rmse_tail_deg'] is None
    assert analyzed['joint_errors']['knee']['rmse_observed_episode_deg'] is None
    assert analyzed['joint_errors']['knee']['tail_error_rows'] == 0
    assert analyzed['joint_errors']['knee']['terminal_signed_deg'] is None


def test_hard_contact_without_matching_ankle_geometry_has_no_moment_measurement(tmp_path):
    observer = EpisodeTelemetry(str(tmp_path))
    episode = Episode()
    payload = hard_info()
    payload['reward_terms'].update(grf_ankle_moment_flip_available=0.,
                                   grf_ankle_moment_flip_tau_nm=0., grf_ankle_moment_flip_loss=0.)
    episode.add(payload, -3., terminated=True)
    observer.observe(episode)
    observer.finish(episode)
    record = json.loads(next(tmp_path.glob('episodes_*.jsonl')).read_text())
    assert record['peaks']['penetration_m']['value'] == .0307
    assert record['peaks']['normal_force_n']['value'] == 1032.
    assert record['peaks']['normal_force_n']['time_s'] == 13.004
    assert 'moment_flip_loss' not in record['peaks']
    assert 'ankle_moment_min_nm' not in record['peaks']


def test_missing_contact_timestamp_is_not_replaced_with_stale_boundary(tmp_path):
    observer = EpisodeTelemetry(str(tmp_path))
    episode = Episode()
    payload = hard_info()
    payload['terminal_contact']['time_s'] = float('nan')
    episode.add(payload, -2., terminated=True)
    observer.observe(episode)
    observer.finish(episode)
    record = json.loads(next(tmp_path.glob('episodes_*.jsonl')).read_text())
    assert record['peaks']['penetration_m']['value'] == .0307
    assert record['peaks']['penetration_m']['time_s'] is None
    assert record['peaks']['penetration_m']['policy_boundary_time_s'] == 13.
    assert record['tail'][0]['terminal_contact']['time_s'] is None
    assert record['terminal_contact_time_s'] is None
    assert record['rows_with_missing_fields'] == 1
    assert 'terminal_contact_time_s' in record['tail'][0]['missing_fields']
