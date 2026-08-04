from __future__ import annotations

import copy
import hashlib
import json
import pickle
import subprocess
import tempfile
import unittest
from pathlib import Path

import yaml

from validation import audit_training_restarts as restart_audit
from validation import ppo_pilot_screen as screen
from validation import robust_ppo_gate as gate
from validation.compare_policy_checkpoints import _actor_digest, _load_state


class PilotScreenTests(unittest.TestCase):
    def test_canonical_protocol_content_digest_is_pinned(self) -> None:
        protocol = (
            screen.ROOT_DIR
            / "validation"
            / "pilot_protocols"
            / "2026-07-15_h0_exact_interleaved_lr5e-7_pilot50.json"
        )
        self.assertEqual(
            screen._canonical_json_sha256(
                json.loads(protocol.read_text(encoding="utf-8"))
            ),
            screen.CANONICAL_PROTOCOL_CONTENT_SHA256,
        )

    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        self.source_checkpoint = self.root / "h0_source" / "checkpoint_last"
        self.source_checkpoint.mkdir(parents=True)
        (self.source_checkpoint / "state.bin").write_bytes(b"source")
        self._create_module(
            self.source_checkpoint
            / "learner_group"
            / "learner"
            / "rl_module"
            / "default_policy",
            payload=b"reference",
        )
        self.source_meta = self.source_checkpoint.parent / "checkpoint_last_meta.json"
        self._write_json(
            self.source_meta,
            {
                "logical_iteration": 1,
                "rllib_training_iteration": 1,
                "checkpoint": str(self.source_checkpoint),
            },
        )
        self.config_source = (
            self.root / "config_source_run" / "training_cfg.resolved.yaml"
        )
        self.config_source.parent.mkdir()
        self.source_config_payload = {
            "model": {"freeze_logstd": True},
            "simulation": {"iterations": 2, "episode_duration": 5.0},
            "supervision": {
                "max_consecutive_skips": 5,
                "max_consecutive_crash_restarts": 5,
                "checkpoint_every": 1,
            },
            "reward": {"reserve_residual_weight": 0.0},
        }
        self._write_yaml(self.config_source, self.source_config_payload)

        self.run_dir = self.root / "pilot_run"
        self.run_dir.mkdir()
        run_config = copy.deepcopy(self.source_config_payload)
        run_config["simulation"]["iterations"] = 51
        run_config["supervision"]["max_consecutive_skips"] = 1
        run_config["supervision"]["max_consecutive_crash_restarts"] = 1
        run_config["supervision"]["retain_iteration_checkpoints"] = True
        run_config["supervision"]["max_minibatch_mean_kl_loss"] = 0.01
        self._write_yaml(
            self.run_dir / "training_cfg.resolved.yaml",
            run_config,
        )
        self.records = [
            self._training_record(iteration)
            for iteration in screen.EXPECTED_RETAINED_LOGICAL_ITERATIONS
        ]
        self._write_history(self.records)
        self._create_milestones()
        self._create_final_checkpoint()
        self._write_run_summary()
        self._write_json(
            self.run_dir / "supervisor_state.json",
            {
                "status": "completed",
                "restart_count": 0,
                "skipped_iterations": [],
                "consecutive_skips": 0,
                "crash_restart_count": 0,
                "consecutive_crash_restarts": 0,
                "resume_from": str(self.source_checkpoint),
            },
        )
        self.driver_pid = 4242
        self._write_json(
            self.run_dir / "watchdog_state.json",
            {"phase": "complete", "pid": self.driver_pid},
        )

        self.reference_checkpoint = self.root / "h0_reference" / "rl_module_last"
        self._create_module(self.reference_checkpoint, payload=b"reference")
        self.reference_actor_digest = _actor_digest(
            _load_state(self.reference_checkpoint)
        )
        self.reference_caps = (510.0, 430.0, 620.0, 596.0)
        self.reference_root = self.root / "reference_summaries"
        for spec, cap in zip(
            gate._rollout_specs(gate.DEFAULT_START_OFFSETS_S),
            self.reference_caps,
        ):
            self._write_json(
                self.reference_root / spec.name / "rollout_summary.json",
                self._rollout_summary(
                    spec=spec,
                    checkpoint=self.reference_checkpoint,
                    reserve_norm_max_nm=cap,
                ),
            )

        self.protocol = self.root / "protocols" / "pilot_protocol.json"
        self._write_json(self.protocol, self._protocol_payload())
        self.ray_log_dir = self.root / "ray" / "logs"
        self.ray_log_dir.mkdir(parents=True)
        driver_log = self.ray_log_dir / (
            "python-core-driver-01000000ffffffffffffffffffffffffffffffff_"
            f"{self.driver_pid}.log"
        )
        driver_log.write_text(
            "actor_manager: SingleAgentEnvRunner state: ALIVE, "
            "num_restarts: 0 actor_id=abc\n",
            encoding="utf-8",
        )
        self.restart_audit_path = self.root / "env_runner_restart_audit.json"
        self._write_json(
            self.restart_audit_path,
            restart_audit.audit_training_restarts(
                self.run_dir,
                self.ray_log_dir,
            ),
        )

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _resolved_config_contract(self) -> dict:
        return screen._resolved_config_contract(
            source_path=self.config_source,
            run_path=self.run_dir / "training_cfg.resolved.yaml",
            expected_source_sha256=self._digest(self.config_source),
        )

    def test_resolved_config_exact_preregistered_diff_passes(self) -> None:
        report = self._resolved_config_contract()

        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["failed_checks"], [])
        self.assertEqual(report["unexpected_differences"], [])
        self.assertEqual(
            {item["path"] for item in report["allowed_changes"]},
            {
                "/simulation/iterations",
                "/supervision/max_consecutive_skips",
                "/supervision/max_consecutive_crash_restarts",
                "/supervision/retain_iteration_checkpoints",
                "/supervision/max_minibatch_mean_kl_loss",
            },
        )

    def test_resolved_config_extra_difference_fails(self) -> None:
        run_path = self.run_dir / "training_cfg.resolved.yaml"
        run = yaml.safe_load(run_path.read_text(encoding="utf-8"))
        run["reward"]["tracking_weight"] = 99.0
        self._write_yaml(run_path, run)

        report = self._resolved_config_contract()

        self.assertEqual(report["status"], "FAIL")
        self.assertIn(
            "run_resolved_config_exact_preregistered_overrides",
            report["failed_checks"],
        )
        self.assertIn(
            "/reward/tracking_weight",
            {item.get("path") for item in report["unexpected_differences"]},
        )

    def test_resolved_config_wrong_override_value_fails(self) -> None:
        run_path = self.run_dir / "training_cfg.resolved.yaml"
        run = yaml.safe_load(run_path.read_text(encoding="utf-8"))
        run["supervision"]["max_minibatch_mean_kl_loss"] = 0.02
        self._write_yaml(run_path, run)

        report = self._resolved_config_contract()

        self.assertEqual(report["status"], "FAIL")
        self.assertIn(
            "run_resolved_config.supervision.max_minibatch_mean_kl_loss",
            report["failed_checks"],
        )
        self.assertIn(
            "/supervision/max_minibatch_mean_kl_loss",
            {item.get("path") for item in report["unexpected_differences"]},
        )

    @staticmethod
    def _digest(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    @staticmethod
    def _write_json(path: Path, payload: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    @staticmethod
    def _write_yaml(path: Path, payload: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            yaml.safe_dump(payload, sort_keys=False),
            encoding="utf-8",
        )

    @staticmethod
    def _create_module(path: Path, *, payload: bytes) -> None:
        path.mkdir(parents=True)
        for filename in gate.REQUIRED_RL_MODULE_FILES:
            artifact = path / filename
            if filename == "module_state.pkl":
                scalar = float((sum(payload) % 97) + 1) / 100.0
                state = {
                    "pi.0.0.weight": [[scalar]],
                    "pi.0.0.bias": [scalar],
                    "pi.0.2.weight": [[scalar]],
                    "pi.0.2.bias": [scalar],
                    "pi.1.weight": [[scalar], [scalar], [scalar], [scalar]],
                    "pi.1.bias": [scalar, scalar, scalar, scalar],
                }
                with artifact.open("wb") as stream:
                    pickle.dump(state, stream)
            else:
                artifact.write_bytes(payload + filename.encode("utf-8"))

    def _training_record(self, iteration: int) -> dict:
        expected_steps = {
            gate._offset_metric_label(offset): gate.EXPECTED_START_STEPS
            for offset in gate.DEFAULT_START_OFFSETS_S
        }
        coverage = {
            f"env_runners/episode_start_steps_current/{label}": count
            for label, count in expected_steps.items()
        }
        coverage.update(
            {
                f"env_runners/episode_start_steps/{label}": iteration * count
                for label, count in expected_steps.items()
            }
        )
        return {
            "iteration": iteration,
            "mean_kl_loss": 0.001,
            "max_minibatch_mean_kl_loss": 0.005,
            "min_minibatch_mean_kl_loss": 0.0,
            "kl_minibatch_count": 9,
            "kl_nonfinite_count": 0,
            "start_coverage_metrics": coverage,
            "exact_start_balance": {
                "pass": True,
                "expected_steps": expected_steps,
                "actual_steps": expected_steps,
                "learner_batch_pass": True,
                "expected_real_steps": 4608,
                "learner_connector_steps_in": 4608,
                "learner_connector_steps_out": 4622,
                "pre_compaction_rows": 4622,
                "removed_compaction_rows": 14,
                "compacted_rows": 4608,
                "interleaved_rows": 4608,
                "interleaved_start_conditions": 3,
                "interleaved_rows_per_start": 1536,
                "max_start_run_length": 1,
                "learner_checks": {
                    "start_interleaving": True,
                    "single_epoch_contract": True,
                },
                "expected_module_steps_trained": 4608,
                "module_steps_trained": 4608,
                "advantage_counts": expected_steps,
                "missing": [],
                "unexpected": [],
                "mismatched": {},
            },
            "iteration_milestone": str(
                self.run_dir / f"{screen.MILESTONE_PREFIX}{iteration:06d}"
            ),
        }

    def _write_history(self, records: list[dict]) -> None:
        (self.run_dir / "train_iterations.jsonl").write_text(
            "".join(json.dumps(record) + "\n" for record in records),
            encoding="utf-8",
        )

    def _create_milestones(self) -> None:
        for iteration in screen.EXPECTED_RETAINED_LOGICAL_ITERATIONS:
            root = self.run_dir / f"{screen.MILESTONE_PREFIX}{iteration:06d}"
            checkpoint = root / "checkpoint_last"
            checkpoint.mkdir(parents=True)
            (checkpoint / "state.bin").write_bytes(f"checkpoint-{iteration}".encode())
            module = root / "rl_module_last"
            self._create_module(module, payload=f"module-{iteration}".encode())
            self._write_json(
                root / "checkpoint_last_meta.json",
                {
                    "logical_iteration": iteration,
                    "rllib_training_iteration": iteration,
                    "checkpoint": str(checkpoint),
                },
            )
            self._write_json(
                root / "rl_module_last_meta.json",
                {
                    "logical_iteration": iteration,
                    "rllib_training_iteration": iteration,
                    "rl_module": str(module),
                },
            )

    def _create_final_checkpoint(self) -> None:
        checkpoint = self.run_dir / "checkpoint_last"
        checkpoint.mkdir()
        (checkpoint / "state.bin").write_bytes(b"final")
        self._create_module(self.run_dir / "rl_module_last", payload=b"final-module")
        self._write_json(
            self.run_dir / "checkpoint_last_meta.json",
            {
                "logical_iteration": 51,
                "rllib_training_iteration": 51,
                "checkpoint": str(checkpoint),
            },
        )

    def _write_run_summary(self) -> None:
        milestones = [
            str(self.run_dir / f"{screen.MILESTONE_PREFIX}{iteration:06d}")
            for iteration in screen.EXPECTED_RETAINED_LOGICAL_ITERATIONS
        ]
        self._write_json(
            self.run_dir / "summary.json",
            {
                "ok": True,
                "stop_reason": "completed",
                "interrupted": False,
                "timed_out": False,
                "error": None,
                "output_dir": str(self.run_dir),
                "iterations_run": 50,
                "iterations_completed": 51,
                "iterations_completed_this_process": 50,
                "iteration_start": 2,
                "next_iteration": 52,
                "restored_logical_iteration": 1,
                "restored_training_iteration": 1,
                "resume_from": str(self.source_checkpoint),
                "supervisor_resume_from": str(self.source_checkpoint),
                "warm_start_requested": False,
                "warm_start_applied": False,
                "skipped_iterations": [],
                "restart_count": 0,
                "crash_restart_count": 0,
                "crash_restarts": [],
                "max_consecutive_skips": 1,
                "freeze_logstd": True,
                "freeze_actor": False,
                "lr": 5.0e-7,
                "num_epochs": 1,
                "num_env_runners": 12,
                "ray_num_cpus": 13,
                "exact_start_sampling": True,
                "episode_start_offset_choices_s": list(
                    gate.DEFAULT_START_OFFSETS_S
                ),
                "exact_start_sampling_contract": {
                    "offsets_s": list(gate.DEFAULT_START_OFFSETS_S),
                    "rollout_fragment_length": 384,
                    "expected_steps_per_start": 1536,
                    "runners_per_start": 4,
                },
                "clip_param": 0.05,
                "kl_coeff": 1.0,
                "kl_target": 0.01,
                "kl_update_guard": {
                    "enabled": True,
                    "max_minibatch_mean_kl_loss_limit": 0.01,
                    "min_minibatch_mean_kl_loss_floor": -1.0e-7,
                    "required_kl_nonfinite_count": 0.0,
                },
                "reward_config": {"reserve_residual_weight": 0.0},
                "history": self.records,
                "iteration_checkpoint_retention": {
                    "enabled": True,
                    "milestones": milestones,
                },
            },
        )

    def _protocol_payload(self) -> dict:
        return {
            "schema_version": 1,
            "status": "preregistered",
            "purpose": "diagnostic_50_update_ppo_pilot_without_automatic_promotion",
            "source": {
                "checkpoint": str(self.source_checkpoint),
                "checkpoint_meta": str(self.source_meta),
                "checkpoint_meta_sha256": self._digest(self.source_meta),
                "logical_iteration": 1,
                "actor_digest": self.reference_actor_digest,
                "canonical_name": "H0",
                "immutable": True,
            },
            "run": {
                "output_dir": str(self.run_dir),
                "resolved_config_source": str(self.config_source),
                "resolved_config_source_sha256": self._digest(self.config_source),
                "new_actor_updates": 50,
                "first_logical_iteration": 2,
                "last_logical_iteration": 51,
                "real_transitions_per_update": 4608,
                "total_planned_real_transitions": 230400,
                "checkpoint_every": 1,
                "retain_iteration_checkpoints": True,
                "update_history": False,
                "automatic_promotion": False,
            },
            "optimization": {
                "learning_rate": 5.0e-7,
                "train_batch_size": 4608,
                "minibatch_size": 512,
                "num_epochs": 1,
                "freeze_logstd": True,
                "freeze_actor": False,
                "num_env_runners": 12,
                "ray_num_cpus": 13,
                "clip_param": 0.05,
                "kl_coeff": 1.0,
                "kl_target": 0.01,
                "hard_max_minibatch_mean_kl_loss": 0.01,
                "hard_min_minibatch_mean_kl_loss": -1.0e-7,
                "required_kl_minibatch_count": 9,
                "required_kl_nonfinite_count": 0,
            },
            "sampling": {
                "mode": "exact_post_gae_interleaved",
                "start_offsets_s": list(gate.DEFAULT_START_OFFSETS_S),
                "runners_per_start": 4,
                "rollout_fragment_length": 384,
                "real_transitions_per_start_per_update": 1536,
                "maximum_start_run_length": 1,
            },
            "online_stop_contract": {
                "stop_on_exact_start_balance_failure": True,
                "stop_on_post_gae_compaction_failure": True,
                "stop_on_interleaving_failure": True,
                "stop_on_missing_or_nonfinite_kl_audit": True,
                "stop_on_kl_limit_failure": True,
                "max_consecutive_skips": 1,
                "any_timeout_skip_or_restart_invalidates_promotion_eligibility": True,
                "physical_failure_of_an_intermediate_checkpoint_does_not_stop_training": True,
            },
            "milestones": {
                "retained_logical_iterations": "all integers from 2 through 51 inclusive",
                "screened_pilot_update_indices": list(screen.EXPECTED_PILOT_UPDATE_INDICES),
                "screened_logical_iterations": list(screen.EXPECTED_SCREENED_LOGICAL_ITERATIONS),
                "screening_timing": "after_training_to_avoid_cpu_contention",
                "screening_order_per_milestone": [
                    "training_integrity_and_kl_audit",
                    "stochastic_plus020_seed123_fail_fast",
                    "three_deterministic_condition_matched_rollouts_only_if_critical_passes",
                ],
            },
            "development_gate": {
                "stochastic_seed": 123,
                "seed_status": "development_and_contaminated_not_held_out",
                "expected_sigma": 0.005,
                "reference_checkpoint": str(self.reference_checkpoint),
                "reference_summaries_root": str(self.reference_root),
                "reserve_mode": gate.RESERVE_MODE_CONDITION_MATCHED,
                "penetration_limit_m_strict": 0.025,
                "minimum_valid_cycles": 2,
                "required_steps": 500,
                "maximum_action_clipped_steps": 0,
            },
            "candidate_selection": {
                "eligible_set": "only preregistered screened milestones that pass every development gate",
                "primary_order": "minimum worst condition-matched reserve ratio versus H0",
                "tie_breakers": [
                    "minimum worst penetration ratio versus H0",
                    "minimum cumulative empirical KL versus H0",
                    "earlier pilot update",
                ],
                "maximum_candidates_opened_on_held_out": 1,
                "fallback_after_held_out_failure": False,
                "checkpoint_best_is_not_a_selector": True,
            },
            "held_out_gate": {
                "status": "sealed_until_one_candidate_digest_is_fixed",
                "seeds": [126, 127, 128],
                "seeds_124_125_status": "contaminated_development_only",
                "start_offsets_s": list(gate.DEFAULT_START_OFFSETS_S),
                "sigma": 0.005,
                "paired_reference": "H0 in every seed/start cell",
                "reserve_mode": gate.RESERVE_MODE_CONDITION_MATCHED,
                "open_once": True,
            },
            "interpretation": {
                "reward_reserve_residual_weight": 0.0,
                "training_return_alone_is_not_evidence_of_reserve_robustness": True,
                "successful_completion_does_not_promote_a_checkpoint": True,
                "long_training_tests_optimization_not_long_horizon_closed_loop_stability": True,
            },
        }

    @staticmethod
    def _rollout_summary(
        *,
        spec: gate.RolloutSpec,
        checkpoint: Path,
        reserve_norm_max_nm: float,
    ) -> dict:
        stochastic = spec.action_selection == "stochastic"
        return {
            "ok": True,
            "checkpoint": str(checkpoint),
            "action_mode": "absolute",
            "action_selection": spec.action_selection,
            "action_seed": spec.seed,
            "exploration_std_mean": [0.005, 0.005] if stochastic else None,
            "exploration_noise_realized_rms": [0.0049, 0.0051] if stochastic else None,
            "episode_start_offset_s": spec.offset_s,
            "steps": 500,
            "end_reason": "episode_time_limit",
            "terminated": False,
            "truncated": True,
            "phase_valid_cycle_count": 2,
            "grf_penetration_max_m": 0.024,
            "grf_penetration_samples": 500,
            "action_clipped_steps": 0,
            "action_shape": [2],
            "n_actor": 35,
            "n_observation": 84,
            "reserve_norm_max_nm": reserve_norm_max_nm,
            "reserve_norm_samples": 500,
        }

    def _runner(self, *, fail_all_critical: bool = False):
        commands: list[list[str]] = []

        def run(command, *, check):
            self.assertFalse(check)
            command = list(command)
            commands.append(command)
            checkpoint = Path(command[command.index("--checkpoint") + 1])
            output = Path(command[command.index("--output-dir") + 1])
            action_selection = command[command.index("--action-selection") + 1]
            offset = float(command[command.index("--episode-start-offset-s") + 1])
            seed = int(command[command.index("--seed") + 1])
            spec = gate.RolloutSpec(output.name, offset, action_selection, seed)
            if action_selection == "stochastic":
                cap = self.reference_caps[3]
            else:
                cap = self.reference_caps[
                    list(gate.DEFAULT_START_OFFSETS_S).index(offset)
                ]
            reserve = cap + 1.0 if fail_all_critical and action_selection == "stochastic" else cap - 1.0
            self._write_json(
                output / "rollout_summary.json",
                self._rollout_summary(
                    spec=spec,
                    checkpoint=checkpoint,
                    reserve_norm_max_nm=reserve,
                ),
            )
            return subprocess.CompletedProcess(command, 0)

        return commands, run

    def _config(
        self,
        output_name: str,
        *,
        restart_audit_path: Path | None = None,
        ray_log_dir: Path | None = None,
    ) -> screen.PilotScreenConfig:
        return screen.PilotScreenConfig(
            protocol=self.protocol,
            output_dir=self.root / output_name,
            restart_audit=(
                restart_audit_path
                if restart_audit_path is not None
                else self.restart_audit_path
            ),
            ray_log_dir=(
                ray_log_dir if ray_log_dir is not None else self.ray_log_dir
            ),
            expected_protocol_content_sha256=screen._canonical_json_sha256(
                json.loads(self.protocol.read_text(encoding="utf-8"))
            ),
        )

    def test_full_screen_runs_critical_first_and_never_touches_held_out(self) -> None:
        commands, runner = self._runner()

        report = screen.screen_pilot(
            self._config("screen_pass"),
            command_runner=runner,
        )

        self.assertTrue(report["ok"])
        self.assertEqual(report["eligible_logical_iterations"], list(screen.EXPECTED_SCREENED_LOGICAL_ITERATIONS))
        self.assertEqual(len(commands), 32)
        for index in range(0, len(commands), 4):
            selections = [
                command[command.index("--action-selection") + 1]
                for command in commands[index:index + 4]
            ]
            self.assertEqual(
                selections,
                ["stochastic", "deterministic", "deterministic", "deterministic"],
            )
        seeds = [int(command[command.index("--seed") + 1]) for command in commands]
        self.assertEqual(seeds, [123] * 32)
        self.assertTrue(set(seeds).isdisjoint(screen.SEALED_HELD_OUT_SEEDS))
        self.assertFalse(report["held_out"]["opened"])
        self.assertEqual(report["held_out"]["seeds_used"], [])
        self.assertFalse(report["fallback_attempted"])
        self.assertFalse(report["checkpoint_promoted"])
        self.assertFalse(report["checkpoint_copied"])
        self.assertTrue((self.root / "screen_pass" / screen.REPORT_FILENAME).is_file())

    def test_critical_failure_skips_all_deterministic_cases(self) -> None:
        commands, runner = self._runner(fail_all_critical=True)

        report = screen.screen_pilot(
            self._config("screen_critical_fail"),
            command_runner=runner,
        )

        self.assertFalse(report["ok"])
        self.assertEqual(report["eligible_logical_iterations"], [])
        self.assertEqual(len(commands), 8)
        self.assertTrue(
            all(
                command[command.index("--action-selection") + 1] == "stochastic"
                for command in commands
            )
        )
        for milestone in report["milestones"]:
            self.assertEqual(milestone["critical"]["status"], "FAIL")
            self.assertEqual(milestone["deterministic"]["status"], "SKIPPED")

    def test_failed_training_audit_invalidates_run_before_rollout(self) -> None:
        records = copy.deepcopy(self.records)
        records[0]["max_minibatch_mean_kl_loss"] = 0.02
        self.records = records
        self._write_history(records)
        self._write_run_summary()
        commands, runner = self._runner()

        report = screen.screen_pilot(
            self._config("screen_training_fail"),
            command_runner=runner,
        )

        self.assertFalse(report["ok"])
        self.assertEqual(commands, [])
        self.assertIn(
            "all_50_training_iterations_pass_update_contract",
            report["run_validation"]["failed_checks"],
        )
        self.assertEqual(report["milestones"], [])

    def test_unscreened_iteration_four_failure_stops_all_rollouts(self) -> None:
        records = copy.deepcopy(self.records)
        records[2]["max_minibatch_mean_kl_loss"] = 0.02
        self.records = records
        self._write_history(records)
        self._write_run_summary()
        commands, runner = self._runner()

        report = screen.screen_pilot(
            self._config("screen_unscreened_training_fail"),
            command_runner=runner,
        )

        self.assertFalse(report["ok"])
        self.assertEqual(commands, [])
        self.assertEqual(report["milestones"], [])
        failed = [
            item["expected_iteration"]
            for item in report["run_validation"]["training_iteration_audits"]
            if item["status"] != "PASS"
        ]
        self.assertEqual(failed, [4])

    def test_wrong_resume_source_stops_all_rollouts(self) -> None:
        summary_path = self.run_dir / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["resume_from"] = str(self.root / "not_h0" / "checkpoint_last")
        self._write_json(summary_path, summary)
        commands, runner = self._runner()

        report = screen.screen_pilot(
            self._config("screen_wrong_source"),
            command_runner=runner,
        )

        self.assertFalse(report["ok"])
        self.assertEqual(commands, [])
        self.assertIn(
            "summary.resume_from",
            report["run_validation"]["failed_checks"],
        )

    def test_h0_actor_digest_mismatch_stops_all_rollouts(self) -> None:
        protocol = json.loads(self.protocol.read_text(encoding="utf-8"))
        protocol["source"]["actor_digest"] = "0" * 64
        self._write_json(self.protocol, protocol)
        commands, runner = self._runner()

        report = screen.screen_pilot(
            self._config("screen_actor_digest_mismatch"),
            command_runner=runner,
        )

        self.assertFalse(report["ok"])
        self.assertEqual(commands, [])
        self.assertEqual(
            report["reserve_reference"]["source_actor_identity"]["status"],
            "FAIL",
        )

    def test_missing_restart_audit_stops_all_rollouts(self) -> None:
        commands, runner = self._runner()

        report = screen.screen_pilot(
            self._config(
                "screen_missing_restart_audit",
                restart_audit_path=self.root / "missing_restart_audit.json",
            ),
            command_runner=runner,
        )

        self.assertFalse(report["ok"])
        self.assertEqual(commands, [])
        self.assertIn("restart_audit.read", report["restart_audit"]["failed_checks"])

    def test_failed_restart_audit_stops_all_rollouts(self) -> None:
        failed_path = self.root / "failed_restart_audit.json"
        failed = json.loads(self.restart_audit_path.read_text(encoding="utf-8"))
        failed["ok"] = False
        failed["status"] = "FAIL"
        failed["failed_checks"] = ["no_ray_restart_or_failure_evidence"]
        failed["checks"]["no_ray_restart_or_failure_evidence"] = False
        self._write_json(failed_path, failed)
        commands, runner = self._runner()

        report = screen.screen_pilot(
            self._config(
                "screen_failed_restart_audit",
                restart_audit_path=failed_path,
            ),
            command_runner=runner,
        )

        self.assertFalse(report["ok"])
        self.assertEqual(commands, [])
        self.assertIn("restart_audit.ok", report["restart_audit"]["failed_checks"])

    def test_mismatched_restart_audit_run_and_log_stop_all_rollouts(self) -> None:
        mismatched_path = self.root / "mismatched_restart_audit.json"
        mismatched = json.loads(
            self.restart_audit_path.read_text(encoding="utf-8")
        )
        mismatched["run_dir"] = str(self.root / "different_run")
        mismatched["ray_log_dir"] = str(self.root / "different_ray" / "logs")
        self._write_json(mismatched_path, mismatched)
        commands, runner = self._runner()

        report = screen.screen_pilot(
            self._config(
                "screen_mismatched_restart_audit",
                restart_audit_path=mismatched_path,
            ),
            command_runner=runner,
        )

        self.assertFalse(report["ok"])
        self.assertEqual(commands, [])
        self.assertIn(
            "restart_audit.run_dir", report["restart_audit"]["failed_checks"]
        )
        self.assertIn(
            "restart_audit.ray_log_dir",
            report["restart_audit"]["failed_checks"],
        )

    def test_incomplete_50_milestone_run_fails_before_any_rollout(self) -> None:
        missing = self.run_dir / "milestone_iteration_000051"
        for path in sorted(missing.rglob("*"), reverse=True):
            if path.is_dir():
                path.rmdir()
            else:
                path.unlink()
        missing.rmdir()
        commands, runner = self._runner()

        report = screen.screen_pilot(
            self._config("screen_incomplete"),
            command_runner=runner,
        )

        self.assertFalse(report["ok"])
        self.assertEqual(commands, [])
        self.assertEqual(report["run_validation"]["status"], "FAIL")
        self.assertIn("milestone_directory_set", report["run_validation"]["failed_checks"])
        self.assertEqual(report["milestones"], [])

    def test_existing_output_is_rejected_without_commands(self) -> None:
        output = self.root / "existing_output"
        output.mkdir()
        sentinel = output / "sentinel"
        sentinel.write_text("unchanged", encoding="utf-8")
        commands, runner = self._runner()

        with self.assertRaises(FileExistsError):
            screen.screen_pilot(
                self._config("existing_output"),
                command_runner=runner,
            )

        self.assertEqual(commands, [])
        self.assertEqual(sentinel.read_text(encoding="utf-8"), "unchanged")

    def test_output_inside_training_run_is_rejected_without_mutation(self) -> None:
        forbidden_output = self.run_dir / "screen_output"
        commands, runner = self._runner()
        config = screen.PilotScreenConfig(
            protocol=self.protocol,
            output_dir=forbidden_output,
            restart_audit=self.restart_audit_path,
            ray_log_dir=self.ray_log_dir,
            expected_protocol_content_sha256=screen._canonical_json_sha256(
                json.loads(self.protocol.read_text(encoding="utf-8"))
            ),
        )

        with self.assertRaises(ValueError):
            screen.screen_pilot(config, command_runner=runner)

        self.assertEqual(commands, [])
        self.assertFalse(forbidden_output.exists())


if __name__ == "__main__":
    unittest.main()
