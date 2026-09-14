"""Pinned experiment settings, phase parity and artifact isolation (no simulator)."""
from __future__ import annotations

import copy
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

TRAJECTORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TRAJECTORY_ROOT))

from unified_MLP import rollout_eval, training_config

UNIFIED_ROOT = TRAJECTORY_ROOT / 'unified_MLP'
CONFIG_ROOT = UNIFIED_ROOT / 'configs'
REPO_ROOT = TRAJECTORY_ROOT.parent


def preset(name):
    return training_config.load(CONFIG_ROOT / f'{name}.yaml')


class UnifiedPresetTests(unittest.TestCase):
    def test_exnovo_is_pinned_source_except_fresh_run_identity(self):
        cfg = preset('ex_novo')
        metadata = json.loads((CONFIG_ROOT / 'provenance.json').read_text())
        source = REPO_ROOT / metadata['ex_novo_source']
        if not source.exists():
            self.skipTest('Historical run artifact unavailable; provenance remains in preset')
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), metadata['ex_novo_source_sha256'])
        expected = training_config.load(source)
        expected['simulation']['iterations'] = 50
        expected['run'] = {'output_dir': None, 'resume_from': None, 'init_actor_from': None}
        actual = copy.deepcopy(cfg)
        actual.pop('provenance')
        self.assertEqual(actual, expected)

    def test_phase_runtime_is_identical_with_explicit_penetration_weight_override(self):
        imitation, exnovo = preset('pretraining'), preset('ex_novo')
        simulation = copy.deepcopy(imitation['simulation'])
        simulation['iterations'] = exnovo['simulation']['iterations']
        self.assertEqual(simulation, exnovo['simulation'])
        self.assertEqual(imitation['grf'], exnovo['grf'])
        self.assertEqual(imitation['parallelism'], exnovo['parallelism'])
        for key, value in exnovo['reward'].items():
            if 'imitation' not in key and key not in ('reward_mode', 'grf_penetration_weight'):
                self.assertEqual(imitation['reward'][key], value, key)
        self.assertEqual(imitation['reward']['grf_penetration_weight'], 1.)
        self.assertEqual(exnovo['reward']['grf_penetration_weight'], .05)
        metadata = json.loads((CONFIG_ROOT / 'provenance.json').read_text())
        self.assertEqual(imitation['provenance']['reward_overrides'],
                         metadata['overrides']['imitation']['reward']['overrides'])
        self.assertEqual(imitation['reward']['blend_served_imitation'], .8)
        self.assertEqual(imitation['reward']['imitation_ankle_position_weight'], 2.)
        self.assertEqual(imitation['ppo']['lr'], 1e-4)
        self.assertEqual(imitation['ppo']['num_epochs'], 10)
        self.assertEqual(imitation['ppo']['clip_param'], .2)
        self.assertEqual(imitation['ppo']['kl_coeff'], .2)
        self.assertFalse(imitation['model']['freeze_logstd'])
        self.assertIsNone(imitation['model']['fixed_exploration_sigma'])
        self.assertIsNone(imitation['supervision']['max_minibatch_mean_kl_loss'])
        self.assertEqual(exnovo['model']['fixed_exploration_sigma'], [.01])
        self.assertTrue(exnovo['model']['freeze_logstd'])

    def test_smoke_only_changes_collection_and_optimization_size(self):
        for name, iterations in [('pretraining', 1), ('ex_novo', 2)]:
            with self.subTest(name=name):
                expected = preset(name)
                expected['ppo'].update(train_batch_size=192, minibatch_size=64, num_epochs=1)
                expected['parallelism'].update(num_env_runners=3, ray_num_cpus=4)
                expected['simulation']['iterations'] = iterations
                expected.pop('provenance')
                actual = preset(f'smoke_{name}')
                actual.pop('provenance')
                self.assertEqual(actual, expected)

    def test_portable_config_roundtrip_keeps_provenance_and_reward(self):
        cfg = preset('pretraining')
        flat, reward = training_config.to_argparse_defaults(cfg)
        args = SimpleNamespace(**flat, _cfg_provenance=cfg['provenance'])
        with tempfile.TemporaryDirectory() as tmp:
            snapshot = Path(tmp) / training_config.RESOLVED_CONFIG_NAME
            with patch.object(training_config, '_resolve_reward', lambda value: value):
                training_config.dump_resolved(args, reward, snapshot)
            export = Path(tmp) / 'rl_module_last'
            export.mkdir()
            resolved = training_config.load_resolved_for_checkpoint(export)
            self.assertEqual(resolved['reward'], reward)
            self.assertEqual(resolved['grf'], cfg['grf'])
            self.assertEqual(resolved['provenance']['baseline_commit'], cfg['provenance']['baseline_commit'])
            self.assertEqual(resolved['provenance']['resolved_reward_mode'], 'imitation')
            self.assertNotIn('run', resolved)
        self.assertEqual(training_config.resolve_config_path(r'configs\pretraining.yaml'), CONFIG_ROOT / 'pretraining.yaml')

    def test_rollout_keeps_snapshot_model_fields_and_provenance(self):
        with tempfile.TemporaryDirectory() as tmp:
            export = Path(tmp) / 'rl_module_last'
            export.mkdir()
            (Path(tmp) / training_config.RESOLVED_CONFIG_NAME).write_text(
                (CONFIG_ROOT / 'ex_novo.yaml').read_text(), encoding='utf-8'
            )
            argv = ['rollout_eval.py', '--checkpoint', str(export), '--output-dir', 'parser_test',
                    '--episode-start-offset-s', '1.756870983805102']
            with patch.object(sys, 'argv', argv):
                args = rollout_eval.parse_args()
            self.assertEqual(args.num_hidden_layers, 2)
            self.assertEqual(args.dim_hidden_layers, 256)
            self.assertEqual(args.pros_ref_cutoff_hz, 4.)
            self.assertEqual(args.episode_start_offset_s, 1.756870983805102)
            self.assertEqual(args._cfg_reward['reward_mode'], 'ex_novo')
            self.assertEqual(args._cfg_provenance['schema_version'], 'unified_mlp_v1')

    def test_rollout_runtime_allows_diagnostic_windows_but_rejects_changed_physics(self):
        with tempfile.TemporaryDirectory() as tmp:
            export = Path(tmp) / 'rl_module_last'
            export.mkdir()
            (Path(tmp) / training_config.RESOLVED_CONFIG_NAME).write_text(
                (CONFIG_ROOT / 'ex_novo.yaml').read_text(), encoding='utf-8'
            )
            argv = ['rollout_eval.py', '--checkpoint', str(export), '--output-dir', 'parser_test',
                    '--episode-start-offset-s', '1.756870983805102', '--episode-duration', '0.2',
                    '--max-steps', '3']
            with patch.object(sys, 'argv', argv):
                args = rollout_eval.parse_args()
            with patch.object(training_config, '_resolve_reward', lambda value: value):
                rollout_eval._validate_checkpoint_runtime(export, args, args._cfg_reward)
                args.pros_ref_cutoff_hz = 5.
                with self.assertRaisesRegex(ValueError, 'pros_ref_cutoff_hz'):
                    rollout_eval._validate_checkpoint_runtime(export, args, args._cfg_reward)

    def test_explicit_config_does_not_bypass_checkpoint_runtime(self):
        with tempfile.TemporaryDirectory() as tmp:
            export = Path(tmp) / 'rl_module_last'
            export.mkdir()
            (Path(tmp) / training_config.RESOLVED_CONFIG_NAME).write_text(
                (CONFIG_ROOT / 'ex_novo.yaml').read_text(), encoding='utf-8'
            )
            argv = ['rollout_eval.py', '--checkpoint', str(export), '--output-dir', 'parser_test',
                    '--no-auto-config', '--config', str(CONFIG_ROOT / 'ex_novo.yaml'),
                    '--pros-ref-cutoff-hz', '5']
            with patch.object(sys, 'argv', argv):
                args = rollout_eval.parse_args()
            with patch.object(training_config, '_resolve_reward', lambda value: value):
                with self.assertRaisesRegex(ValueError, 'pros_ref_cutoff_hz'):
                    rollout_eval._validate_checkpoint_runtime(export, args, args._cfg_reward)

    def test_rollout_without_snapshot_or_explicit_config_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            argv = ['rollout_eval.py', '--checkpoint', tmp, '--output-dir', 'parser_test']
            with patch.object(sys, 'argv', argv), patch.object(sys, 'stderr', io.StringIO()) as err:
                with self.assertRaises(SystemExit) as raised:
                    rollout_eval.parse_args()
            self.assertEqual(raised.exception.code, 2)
            self.assertIn('resolved config', err.getvalue())

    def test_missing_default_is_explicit_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            absent = Path(tmp) / 'absent.yaml'
            with patch.object(training_config, 'DEFAULT_CONFIG_PATH', absent):
                with self.assertRaises(FileNotFoundError):
                    training_config.load(absent)


class ArtifactIsolationTests(unittest.TestCase):
    def test_default_and_windows_style_relative_rollout_paths_stay_unified(self):
        destination = rollout_eval._resolve_output_dir(r'Trajectory Generator\unified_MLP\runs\rollout\diagnostic', 'unused')
        self.assertEqual(destination, UNIFIED_ROOT / 'runs/rollout/diagnostic')
        self.assertTrue(rollout_eval._resolve_output_dir(None, 'diagnostic').is_relative_to(UNIFIED_ROOT / 'runs/rollout'))
        self.assertEqual(rollout_eval._TRAINING_RUNS_ROOT, UNIFIED_ROOT / 'runs/training')

    def test_baseline_and_parent_traversal_output_paths_are_rejected(self):
        candidates = [
            str(TRAJECTORY_ROOT / 'runs/rollout/existing_baseline'),
            str(TRAJECTORY_ROOT / 'baseline_MLP'),
            '../../../baseline_MLP',
            '.',
            str(UNIFIED_ROOT / 'runs/rollout'),
        ]
        for value in candidates:
            with self.subTest(path=value), self.assertRaises(ValueError):
                rollout_eval._resolve_output_dir(value, 'diagnostic')


if __name__ == '__main__':
    unittest.main()
