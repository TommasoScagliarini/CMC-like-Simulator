"""Offline preflight using real RLlib 2.55 SingleAgentEpisode/cut; no Ray runtime."""
from __future__ import annotations

import contextlib
import copy
import io
import json
import os
from pathlib import Path
import random
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / '.git').exists())
sys.path.insert(0, str(REPO / 'Trajectory Generator'))

import numpy as np
import torch
from ray.rllib.env.single_agent_episode import SingleAgentEpisode
from unified_MLP_39.episode_telemetry import EpisodeTelemetry, ENV_VAR, MAX_ENV_SLOTS, TAIL_STEPS
from unified_MLP_39.tb_logging import RewardComponentsCallback


class Logger:
    def __init__(self):
        self.values = {}

    def log_value(self, key, value, **kwargs):
        self.values[key] = self.values.get(key, 0) + value


def make_info(t, end=None):
    # Only deterministic arrays. Explicit episode IDs also avoid uuid RNG calls.
    return {
        'time': 12.99 + t * .01, 'target_time': 12.99 + t * .01,
        'episode_start_offset_s': 1.0,
        'observation': {'pros_knee_angle': -.3, 'pros_knee_angle_vel': .2,
                        'pros_knee_angle_served_ref': -.31, 'pros_ankle_angle': .1},
        'imitation_target': {'q': {'pros_knee_angle': -.4, 'pros_ankle_angle': .2},
                            'qdot': {'pros_knee_angle': .25, 'pros_ankle_angle': .15},
                            'phase': {'pros_knee_angle': (t % 100) / 100}},
        'phase_fsm': {'state_id': 2.0, 'state_name': 'SWING_AFTER_TO',
                      'resync_event_this_step': float(t == 10), 'resync_count': float(t >= 10),
                      'hs_cancelled_count': float(t >= 15), 'timeout_exceeded': 0., 'timeout_side': 0.},
        'reward_terms': {'grf_penetration_m': .035 if t == 12 else .012,
                         'grf_penetration_loss': float(t == 12),
                         'grf_ankle_moment_flip_tau_nm': -25.0,
                         'grf_ankle_moment_flip_loss': .2,
                         'policy_action_clip_fraction': float(t % 7 == 0),
                         'phase_timeout_exceeded': 0., 'phase_timeout_side': 0.,
                         'morphology_causal_failed_closed': 0.},
        'reward_components': {'reward': .75, 'grf_ankle_moment_flip_term': .02},
        'online_gait': {'sides': {'left': {'normal_force_n': 500.0, 'in_contact': True}}},
        'policy_segment_values': np.asarray([[-.3, .1], [-.31, .12]]),
        'target_availability': {'visible': True}, 'end_reason': end,
    }


def make_episode(name):
    e = SingleAgentEpisode(id_=name, len_lookback_buffer=0)
    e.add_env_reset(observation=np.arange(88, dtype=np.float32), infos=make_info(0))
    return e


def add(e, t, end=None, terminated=False, truncated=False, info=None, version=40):
    e.add_env_step(observation=np.arange(88, dtype=np.float32) + t,
                   action=np.array([1.2, -.25], dtype=np.float32), reward=.75,
                   infos=make_info(t, end) if info is None else info,
                   terminated=terminated, truncated=truncated,
                   extra_model_outputs={'action_dist_inputs': np.array([.2, -.1, -.2, -.3]),
                                        'action_logp': -.5, 'weights_seq_no': version})


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name) / 'episodes'
        self.observer = EpisodeTelemetry(str(self.directory))
        self.logger = Logger()
        self.runner = SimpleNamespace(worker_index=3, _weights_seq_no=40)

    def tearDown(self):
        self.temp.cleanup()

    def observe(self, e, **kw):
        self.observer.observe(e, env_runner=self.runner, metrics_logger=self.logger, **kw)

    def finish(self, e, **kw):
        self.observer.finish(e, env_runner=self.runner, metrics_logger=self.logger, **kw)

    def records(self):
        return [json.loads(line) for p in self.directory.glob('episodes_*.jsonl')
                for line in p.read_text().splitlines()]

    def test_vectorized_numpy_visibility_is_json_serializable(self):
        e = make_episode('numpy-vector-info')
        info = make_info(1, 'episode_time_limit')
        info['target_availability']['visible'] = np.bool_(True)
        add(e, 1, truncated=True, info=info)
        self.observe(e)
        self.finish(e)
        self.assertEqual(self.observer.metrics.get('write_errors', 0), 0)
        self.assertEqual(len(self.records()), 1)
        self.assertIs(self.records()[0]['tail'][0]['target_visible'], True)
        self.assertIsInstance(info['target_availability']['visible'], np.bool_)

    def test_real_rllib_fragments_lookback_and_duplicate_callbacks(self):
        e = make_episode('fragmented')
        previous = []
        for t in range(1, 121):
            add(e, t, 'episode_time_limit' if t == 120 else None, truncated=t == 120)
            self.observe(e)
            if t in (30, 60, 95):
                self.observe(e)  # Real duplicated callback must not duplicate data.
                successor = e.cut(len_lookback_buffer=7)
                previous.append(e)
                e = successor
                self.assertEqual(e.id_, 'fragmented')
                self.assertEqual(e.t, t)
        self.finish(e, prev_episode_chunks=previous)
        self.finish(e, prev_episode_chunks=previous)
        records = self.records()
        self.assertEqual(len(records), 1)
        r = records[0]
        self.assertEqual(r['episode_steps'], 120)
        self.assertEqual(r['observed_steps'], 120)
        self.assertEqual(r['missing_steps'], 0)
        self.assertEqual(r['episode_return'], 90.)
        self.assertEqual(r['observed_reward_sum'], 90.)
        self.assertEqual(r['policy_distribution_rows'], 120)
        self.assertEqual(r['rows_with_missing_fields'], 0)
        self.assertEqual([x['episode_step'] for x in r['tail']], list(range(71, 121)))
        self.assertEqual(r['tail'][0]['actor_observation_before'][0], 70.)
        self.assertEqual(r['peaks']['penetration_m']['episode_step'], 12)
        self.assertEqual(r['last_resync']['episode_step'], 10)
        self.assertEqual(r['last_hs_cancel']['episode_step'], 15)
        self.assertEqual(r['event_counts']['resync_rows'], 1)
        self.assertAlmostEqual(r['duration_s'], 1.2)
        self.assertEqual(len(self.observer._slots), 0)

    def test_every_terminal_outcome_is_saved(self):
        endings = [('episode_time_limit', False, True), ('grf_penetration', True, False),
                   ('grf_penetration_hard', True, False), ('invalid_binary_event', True, False),
                   ('phase_timeout:swing', True, False), ('step_wall_timeout', False, True)]
        for index, (reason, terminated, truncated) in enumerate(endings):
            e = make_episode(f'outcome-{index}')
            add(e, 1, reason, terminated=terminated, truncated=truncated)
            self.observe(e)
            self.finish(e)
        self.assertEqual([r['end_reason'] for r in self.records()], [x[0] for x in endings])
        self.assertEqual(len({r['episode_uid'] for r in self.records()}), len(endings))

    def test_bounded_memory_with_repeated_episodes_and_long_episode(self):
        for n in range(200):
            e = make_episode(f'repeated-{n}')
            add(e, 1, 'episode_time_limit', truncated=True)
            self.observe(e)
            self.finish(e)
            self.assertEqual(len(self.observer._slots), 0)
            self.assertEqual(len(self.observer._last_closed), 1)
        e = make_episode('long')
        for t in range(1, 301):
            add(e, t)
            self.observe(e)
        self.assertEqual(len(next(iter(self.observer._slots.values())).tail), TAIL_STEPS)

    def test_abandoned_or_excess_slots_are_bounded(self):
        with contextlib.redirect_stderr(io.StringIO()):
            for t in range(MAX_ENV_SLOTS + 2):
                e = make_episode(f'env-{t}')
                add(e, 1)
                self.observe(e, env_index=t)
            e = make_episode('replacement')
            add(e, 1)
            self.observe(e, env_index=MAX_ENV_SLOTS)
        self.assertEqual(len(self.observer._slots), MAX_ENV_SLOTS)
        self.assertEqual(self.observer.metrics['evicted_episodes'], 2)
        self.assertEqual(self.observer.metrics['abandoned_episodes'], 1)

    def test_old_end_cannot_delete_active_replacement(self):
        e = make_episode('old')
        add(e, 1)
        self.observe(e)
        e2 = make_episode('new')
        add(e2, 1)
        with contextlib.redirect_stderr(io.StringIO()):
            self.observe(e2)
            self.finish(e)
        self.assertEqual(self.observer._slots[(3, 0)].episode_id, 'new')

    def test_io_failure_isolated_and_buffer_released(self):
        self.directory.write_text('occupied by regular file')
        e = make_episode('io-failure')
        add(e, 1, 'grf_penetration', terminated=True)
        self.observe(e)
        with contextlib.redirect_stderr(io.StringIO()) as err:
            self.finish(e)
        self.assertIn('training continues', err.getvalue())
        self.assertEqual(self.observer.metrics['write_errors'], 1)
        self.assertEqual(self.logger.values['episode_telemetry/episodes_dropped'], 1)
        self.assertEqual(len(self.observer._slots), 0)
        self.directory.unlink()
        e2 = make_episode('after-io-failure')
        add(e2, 1, 'episode_time_limit', truncated=True)
        self.observe(e2)
        self.finish(e2)
        self.assertEqual(len(self.records()), 1)
        self.assertTrue(next(self.directory.glob('*.jsonl')).name.endswith('part1.jsonl'))

    def test_relative_path_does_not_write(self):
        observer = EpisodeTelemetry('relative-is-invalid')
        e = make_episode('relative')
        add(e, 1, 'episode_time_limit', truncated=True)
        observer.observe(e)
        with contextlib.redirect_stderr(io.StringIO()):
            observer.finish(e)
        self.assertEqual(observer.metrics['write_errors'], 1)
        self.assertFalse(Path('relative-is-invalid').exists())

    def test_missing_step_and_nonfinite_diagnostics_are_explicit(self):
        e = make_episode('missing-step')
        add(e, 1)
        self.observe(e)
        add(e, 2)  # Deliberately missing callback.
        info = make_info(3, 'episode_time_limit')
        info['reward_terms']['grf_penetration_m'] = float('nan')
        add(e, 3, truncated=True, info=info)
        self.observe(e)
        self.finish(e)
        r = self.records()[0]
        self.assertEqual(r['missing_steps'], 1)
        self.assertEqual(r['rows_with_missing_fields'], 1)
        self.assertIsNone(r['tail'][-1]['reward_terms']['grf_penetration_m'])
        self.assertEqual(self.logger.values['episode_telemetry/missing_rows'], 1)

    def test_partial_episode_after_worker_restart_is_marked(self):
        e = make_episode('partial')
        for t in range(1, 5):
            add(e, t)
        e = e.cut(len_lookback_buffer=1)
        add(e, 5, 'episode_time_limit', truncated=True)
        self.observe(e)
        self.finish(e)
        r = self.records()[0]
        self.assertTrue(r['partial_episode'])
        self.assertEqual(r['missing_steps'], 4)
        self.assertIsNone(r['episode_start_time_s'])
        self.assertIsNone(r['duration_s'])

    def test_first_callback_failure_counts_missing_step_and_recovers(self):
        e = make_episode('first-callback-failure')
        add(e, 1)
        with patch.object(self.observer, '_row', side_effect=ValueError('malformed info')):
            with contextlib.redirect_stderr(io.StringIO()):
                self.observe(e)
        add(e, 2, 'grf_penetration', terminated=True)
        self.observe(e)
        self.finish(e)
        r = self.records()[0]
        self.assertEqual(r['missing_steps'], 1)
        self.assertEqual(r['observed_steps'], 1)
        self.assertEqual(self.observer.metrics['step_errors'], 1)
        self.assertIsNone(r['duration_s'])

    def test_vector_env_and_worker_identity_files(self):
        for env_index in (0, 1):
            e = make_episode(f'vector-{env_index}')
            add(e, 1, 'episode_time_limit', truncated=True)
            self.observe(e, env_index=env_index)
            self.finish(e, env_index=env_index)
        other = EpisodeTelemetry(str(self.directory))
        e = make_episode('other-worker')
        add(e, 1, 'episode_time_limit', truncated=True)
        other.observe(e, env_runner=SimpleNamespace(worker_index=4))
        other.finish(e, env_runner=SimpleNamespace(worker_index=4))
        self.assertEqual(len(self.records()), 3)
        self.assertEqual(len(list(self.directory.glob('episodes_*.jsonl'))), 2)
        self.assertEqual(len({r['episode_uid'] for r in self.records()}), 3)

    def test_policy_switches_before_tail_survive_fragment_boundaries(self):
        e = make_episode('weight-versions')
        previous = []
        for t in range(1, 121):
            version = 40 if t <= 30 else (41 if t <= 80 else 42)
            add(e, t, 'episode_time_limit' if t == 120 else None,
                truncated=t == 120, version=version)
            self.observe(e)
            if t in (30, 80):
                previous.append(e)
                e = e.cut(len_lookback_buffer=3)
        self.finish(e, prev_episode_chunks=previous)
        r = self.records()[0]
        self.assertEqual(r['policy_versions'], {
            'first': 40., 'last': 42., 'min': 40., 'max': 42.,
            'switch_count': 2, 'rows_with_version': 120, 'rows_without_version': 0,
        })
        self.assertNotIn(40., [row['policy_outputs']['weights_seq_no'] for row in r['tail']])

    def test_callback_default_off_and_opt_in_rng_and_data_unchanged(self):
        e = make_episode('callback')
        add(e, 1, 'episode_time_limit', truncated=True)
        with patch.dict(os.environ, {}, clear=False):
            os.environ.pop(ENV_VAR, None)
            cb_off = RewardComponentsCallback()
            cb_off.on_episode_step(episode=e, metrics_logger=Logger(), env_runner=self.runner, env_index=0)
            self.assertIsNone(cb_off._episode_observer)
        before = copy.deepcopy(e.get_state())
        python_rng, np_rng, torch_rng = random.getstate(), np.random.get_state(), torch.get_rng_state().clone()
        with patch.dict(os.environ, {ENV_VAR: str(self.directory)}):
            cb = RewardComponentsCallback()
            cb.on_episode_step(episode=e, metrics_logger=self.logger, env_runner=self.runner, env_index=0)
            cb.on_episode_end(episode=e, prev_episode_chunks=[], metrics_logger=self.logger,
                              env_runner=self.runner, env_index=0)
        self.assertEqual(python_rng, random.getstate())
        np_after = np.random.get_state()
        self.assertEqual(np_rng[0], np_after[0])
        np.testing.assert_array_equal(np_rng[1], np_after[1])
        self.assertEqual(np_rng[2:], np_after[2:])
        self.assertTrue(torch.equal(torch_rng, torch.get_rng_state()))
        # The callback must not store any buffer in custom_data or mutate infos,
        # model outputs or transitions. Compare serialized structure recursively.
        def equal(a, b):
            if isinstance(a, np.ndarray):
                np.testing.assert_array_equal(a, b)
            elif isinstance(a, dict):
                self.assertEqual(set(a), set(b))
                for key in a:
                    equal(a[key], b[key])
            elif isinstance(a, (list, tuple)):
                self.assertEqual(len(a), len(b))
                for x, y in zip(a, b):
                    equal(x, y)
            else:
                self.assertEqual(a, b)
        equal(before, e.get_state())
        self.assertEqual(len(self.records()), 1)

    def test_hostile_diagnostic_metrics_do_not_propagate(self):
        class Hostile:
            def log_value(self, *a, **kw):
                raise RuntimeError('logger unavailable')
        e = make_episode('hostile')
        add(e, 1, 'grf_penetration', terminated=True)
        with contextlib.redirect_stderr(io.StringIO()):
            self.observer.observe(e, metrics_logger=Hostile())
            self.observer.finish(e, metrics_logger=Hostile())
        self.assertEqual(len(self.records()), 1)


def benchmark():
    with tempfile.TemporaryDirectory() as directory:
        elapsed = {}
        for enabled in (False, True):
            with patch.dict(os.environ, {ENV_VAR: directory if enabled else ''}):
                cb, logger = RewardComponentsCallback(), Logger()
                runner = SimpleNamespace(worker_index=1, _weights_seq_no=40)
                cpu_start, wall_start = time.process_time(), time.perf_counter()
                for n in range(5):
                    e = make_episode(f'benchmark-{enabled}-{n}')
                    for t in range(1, 501):
                        add(e, t, 'episode_time_limit' if t == 500 else None, truncated=t == 500)
                        cb.on_episode_step(episode=e, metrics_logger=logger, env_runner=runner, env_index=0)
                    cb.on_episode_end(episode=e, metrics_logger=logger, env_runner=runner, env_index=0)
                elapsed['on' if enabled else 'off'] = {
                    'cpu_s': time.process_time() - cpu_start,
                    'wall_s': time.perf_counter() - wall_start,
                    'steps': 2500,
                }
        elapsed['incremental_cpu_us_per_step'] = (elapsed['on']['cpu_s'] - elapsed['off']['cpu_s']) * 1e6 / 2500
        elapsed['output_bytes_per_episode'] = sum(p.stat().st_size for p in Path(directory).glob('*.jsonl')) / 5
        return elapsed


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(TelemetryTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    receipt = {'tests': result.testsRun, 'failures': len(result.failures), 'errors': len(result.errors),
               'ok': result.wasSuccessful(), 'benchmark': benchmark() if result.wasSuccessful() else None}
    (HERE / 'test_receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))
    sys.exit(0 if result.wasSuccessful() else 1)
