"""Pilot preparation, bounded fake subprocesses and paired frozen-sigma tests."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from unified_MLP_39.reward_tuning import evaluation, pilot


class PilotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="tuning pilot ")
        self.root = Path(self.temp.name)
        self.package = self.root / "Trajectory Generator/unified_MLP_39"
        self.package.mkdir(parents=True)
        self.output = self.package / "validation/pilot_test"
        self.source = self.package / "runs/training/source/checkpoint_best"
        self.snapshot = {"reward": {"reward_mode": "imitation", "grf_penetration_weight": 2.0},
                         "simulation": {"episode_duration": 5.0, "segment_duration": .01},
                         "target_availability": {"target_reference": "left", "target_availability_probability": 1.0},
                         "ppo": {"train_batch_size": 4096}, "model": {"seed": 123}}
        self.checkpoint(self.source, 80)
        self.config = {"_repo_root": str(self.root), "source_checkpoint": str(self.source),
                       "source_config": str(self.source / "training_cfg.resolved.yaml")}
        self.analysis = {"input_hashes": {}, "candidates": {"ankle_10": {"overrides": {"ankle_positive_torque_weight": .15}}}}
        self.pkg_patch = patch.object(pilot, "PACKAGE", self.package)
        self.pkg_patch.start()

    def tearDown(self):
        self.pkg_patch.stop()
        self.temp.cleanup()

    def checkpoint(self, path, iteration):
        path.mkdir(parents=True, exist_ok=True)
        for name in ("algorithm_state.pkl", "rllib_checkpoint.json", "unified_contract.json", "unified_learner_state.json"):
            (path / name).write_text("{}")
        (path / "training_cfg.resolved.yaml").write_text(yaml.safe_dump(self.snapshot))
        pilot.write_json(path / "unified_training_state.json", {"last_completed_iteration": iteration,
            "imitation_critic_rewarm_remaining": 0, "imitation_critic_rewarm_completed_iterations": 1,
            "last_completed_phase": "imitation_critic_rewarm" if iteration == 81 else "imitation"})
        pilot.write_json(path.parent / f"{path.name}_meta.json", {"logical_iteration": iteration})
        nested = path / "learner_group/learner/rl_module/default_policy"
        nested.mkdir(parents=True)
        (nested / "module_state.pkl").write_bytes(b"test fixture")
        (path / "learner_group/learner/state.pkl").write_bytes(b"optimizer fixture")

    def test_prepare_no_execution_and_protocol(self):
        with patch.object(pilot.subprocess, "Popen", side_effect=AssertionError("prepare launched subprocess")):
            manifest = pilot.prepare(self.config, self.analysis, self.output)
        self.assertFalse(manifest["training_started"])
        self.assertEqual(manifest["budget_seconds"], 7200)
        self.assertEqual(manifest["evaluation_parallelism"], 4)
        self.assertEqual(len(manifest["evaluation_cases"]), 12)
        for role in ("control", "ankle_10"):
            branch = yaml.safe_load(Path(manifest["branches"][role]["config"]).read_text())
            self.assertEqual(branch["simulation"]["iterations"], 84)
            self.assertEqual(branch["run"]["imitation_critic_rewarm_iterations"], 1)
            self.assertTrue(branch["run"]["resume_tuning_reward_fork"])
            self.assertEqual(branch["target_availability"], self.snapshot["target_availability"])
        with self.assertRaises(FileExistsError):
            pilot.prepare(self.config, self.analysis, self.output)

    def test_distinct_pilot_parent_names(self):
        first = pilot.prepare(self.config, self.analysis, self.package / "one/pilot")
        second = pilot.prepare(self.config, self.analysis, self.package / "two/pilot")
        self.assertNotEqual(first["branches"]["control"]["output_dir"], second["branches"]["control"]["output_dir"])

    def test_calibrated_ankle_five_role_and_weight(self):
        self.config["pilot"] = {"candidate": "ankle_5"}
        self.analysis["candidates"]["ankle_5"] = {"overrides": {"ankle_positive_torque_weight": .075}}
        manifest = pilot.prepare(self.config, self.analysis, self.output)
        self.assertEqual(manifest["candidate_role"], "ankle_5")
        self.assertEqual(set(manifest["branches"]), {"control", "ankle_5"})
        cfg = yaml.safe_load(Path(manifest["branches"]["ankle_5"]["config"]).read_text())
        self.assertEqual(cfg["reward"]["ankle_positive_torque_weight"], .075)
        self.assertEqual({c["role"] for c in manifest["evaluation_cases"]}, {"best80", "control", "ankle_5"})

    def test_unknown_candidate_rejected(self):
        self.config["pilot"] = {"candidate": "knee_10"}
        with self.assertRaisesRegex(ValueError, "calibrated ankle"):
            pilot.prepare(self.config, self.analysis, self.output)

    def test_source_cannot_be_output(self):
        with self.assertRaisesRegex(ValueError, "disjoint"):
            pilot.prepare(self.config, self.analysis, self.source / "pilot")

    def test_changed_input_rejected_before_execution(self):
        pilot.prepare(self.config, self.analysis, self.output)
        (self.source / "algorithm_state.pkl").write_text("changed")
        with patch.object(pilot, "execute_command", side_effect=AssertionError("launched")):
            with self.assertRaisesRegex(ValueError, "input changed"):
                pilot.run(self.output)

    def test_resume_refuses_extra_budget(self):
        pilot.prepare(self.config, self.analysis, self.output)
        state_path = self.output / "pilot_state.json"
        state = pilot.read_json(state_path)
        state["elapsed_seconds"] = 7200
        pilot.write_json(state_path, state)
        with patch.object(pilot, "execute_command", side_effect=AssertionError("launched")):
            self.assertEqual(pilot.run(self.output)["status"], "budget_exhausted")
            self.assertEqual(pilot.run(self.output)["status"], "budget_exhausted")

    def test_incomplete_checkpoint_never_selected(self):
        run_dir = self.package / "runs/training/branch"
        good = run_dir / "milestone_iteration_000081/checkpoint_last"
        self.checkpoint(good, 81)
        bad = run_dir / "checkpoint_last"
        self.checkpoint(bad, 82)
        (bad / "algorithm_state.pkl").unlink()
        self.assertEqual(pilot.latest_complete_checkpoint(run_dir), (81, good))

    def test_interruption_preserves_milestone_and_resume_source(self):
        manifest = pilot.prepare(self.config, self.analysis, self.output)
        branch = manifest["branches"]["control"]
        checkpoint = Path(branch["output_dir"]) / "milestone_iteration_000081/checkpoint_last"
        def interrupt(*args, **kwargs):
            self.checkpoint(checkpoint, 81)
            raise KeyboardInterrupt
        with patch.object(pilot, "execute_command", side_effect=interrupt):
            result = pilot.run(self.output)
        self.assertEqual(result["status"], "interrupted")
        self.assertEqual(result["latest_checkpoints"]["control"]["iteration"], 81)
        def fail(command, *_args, **_kwargs):
            self.assertEqual(command[-2:], ["--resume-from", str(checkpoint)])
            return {"returncode": 1, "elapsed_seconds": 0, "reason": "failed"}
        with patch.object(pilot, "execute_command", side_effect=fail):
            self.assertEqual(pilot.run(self.output)["status"], "failed")

    def test_fake_complete_and_idempotent_resume(self):
        manifest = pilot.prepare(self.config, self.analysis, self.output)
        def fake(command, *_args, **_kwargs):
            if Path(command[1]).name == "train_ppo_mlp.py":
                cfg = yaml.safe_load(Path(command[3]).read_text())
                for iteration in range(81, 85):
                    self.checkpoint(Path(cfg["run"]["output_dir"]) / f"milestone_iteration_{iteration:06d}/checkpoint_last", iteration)
            elif Path(command[1]).name == "evaluation.py":
                case = pilot.read_json(Path(command[3]))
                folder = Path(case["output_dir"])
                pilot.write_json(folder / "rollout_summary.json", {"completed_full_episode": False})
                if case["freeze_sigma"]:
                    pilot.write_json(Path(case["sigma_path"]), {"sigma": [[.1, .2]] * 500})
                pilot.write_json(folder / "rollout_policy_trace.json", [])
                pilot.write_json(folder / "paired_noise_receipt.json", {"sigma_sha256": pilot.sha256(Path(case["sigma_path"]))})
                names = ("rollout_summary.json", "rollout_policy_trace.json", "paired_noise_receipt.json")
                pilot.write_json(folder / "evaluation_complete.json", {"steps": 500,
                    "artifact_hashes": {name: pilot.sha256(folder / name) for name in names}})
            return {"returncode": 0, "elapsed_seconds": 0, "reason": "completed"}
        with patch.object(pilot, "execute_command", side_effect=fake) as commands:
            result = pilot.run(self.output)
        self.assertEqual(result["status"], "completed")
        self.assertEqual(commands.call_count, 14)
        self.assertEqual(len(result["evaluation"]), 3)
        with patch.object(pilot, "execute_command", side_effect=AssertionError("repeated completed work")):
            self.assertEqual(pilot.run(self.output)["status"], "completed")

    def test_subprocess_timeout_without_training(self):
        result = pilot.execute_command([sys.executable, "-c", "import time; time.sleep(5)"], self.root / "timeout.log",
                                       cwd=self.root, remaining_seconds=.1)
        self.assertEqual(result["reason"], "budget_exhausted")
        self.assertLess(result["elapsed_seconds"], 4)

    def test_windows_paths_resolve_on_posix(self):
        if sys.platform != "win32":
            self.assertEqual(pilot.resolve_path(self.root, r"a\b\config.yaml"), (self.root / "a/b/config.yaml").resolve())


class NoiseTests(unittest.TestCase):
    def test_paired_draws_do_not_use_global_rng(self):
        np.random.seed(400)
        previous = np.random.get_state()
        one, two = evaluation.paired_noise(123, 500, 2), evaluation.paired_noise(123, 500, 2)
        np.testing.assert_array_equal(one, two)
        self.assertEqual(evaluation.noise_digest(one), evaluation.noise_digest(two))
        np.testing.assert_array_equal(previous[1], np.random.get_state()[1])
        self.assertFalse(np.array_equal(one, evaluation.paired_noise(125, 500, 2)))

    def test_fixed_sigma_despite_different_policy_sigmas(self):
        import torch
        class Module:
            def __init__(self, logstd):
                self.logstd = logstd
            def forward_inference(self, _batch):
                return {"action_dist_inputs": torch.tensor([[.3, -.2, self.logstd, self.logstd]])}
        schedule = np.asarray([[.1, .2], [.2, .3]], np.float32)
        draws = evaluation.paired_noise(123, 2, 2)
        records = [[], []]
        selectors = [evaluation.fixed_sigma_selector(torch, np, schedule, draws, rows) for rows in records]
        for _ in range(2):
            a = selectors[0](Module(-1), np.zeros(39), (2,))
            b = selectors[1](Module(2), np.zeros(39), (2,))
            np.testing.assert_array_equal(a[0], b[0])
            np.testing.assert_array_equal(a[2], b[2])
        self.assertNotEqual(records[0][0]["learned_sigma"], records[1][0]["learned_sigma"])
        with self.assertRaisesRegex(ValueError, "exceeded"):
            selectors[0](Module(-1), np.zeros(39), (2,))


if __name__ == "__main__":
    unittest.main()
