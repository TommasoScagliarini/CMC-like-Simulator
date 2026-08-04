from __future__ import annotations

import copy
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from validation import robust_ppo_gate as gate


BASELINE_DIR = (
    Path(__file__).resolve().parents[1] / "Trajectory Generator" / "baseline_MLP"
)
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

import rollout_eval


class RobustPpoGateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.checkpoint = self.root / "candidate" / "rl_module_last"
        self.checkpoint.mkdir(parents=True)
        for filename in gate.REQUIRED_RL_MODULE_FILES:
            (self.checkpoint / filename).write_bytes(b"checkpoint-data")

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _summary(
        self,
        spec: gate.RolloutSpec,
        *,
        checkpoint: Path | None = None,
        sigma: float = 0.005,
        reserve_norm_max_nm: float = 600.0,
    ) -> dict:
        stochastic = spec.action_selection == "stochastic"
        return {
            "ok": True,
            "checkpoint": str(checkpoint or self.checkpoint),
            "action_mode": "absolute",
            "action_selection": spec.action_selection,
            "action_seed": spec.seed,
            "exploration_std_mean": [sigma, sigma] if stochastic else None,
            "exploration_noise_realized_rms": (
                [0.98 * sigma, 1.02 * sigma] if stochastic else None
            ),
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

    def _write_summary(
        self,
        name: str,
        spec: gate.RolloutSpec,
        *,
        checkpoint: Path | None = None,
        reserve_norm_max_nm: float = 600.0,
        mutations: dict | None = None,
    ) -> Path:
        path = self.root / name / "rollout_summary.json"
        path.parent.mkdir(parents=True)
        summary = self._summary(
            spec,
            checkpoint=checkpoint,
            reserve_norm_max_nm=reserve_norm_max_nm,
        )
        if mutations:
            summary.update(mutations)
        path.write_text(json.dumps(summary), encoding="utf-8")
        return path

    def _make_checkpoint(self, parent_name: str) -> Path:
        checkpoint = self.root / parent_name / "rl_module_last"
        checkpoint.mkdir(parents=True)
        for filename in gate.REQUIRED_RL_MODULE_FILES:
            (checkpoint / filename).write_bytes(b"checkpoint-data")
        return checkpoint

    def _write_train_iterations(
        self,
        *,
        iteration: int = gate.EXPECTED_TRAINING_ITERATION,
        kl: float = 0.01,
        max_minibatch_kl: float = 0.01,
        min_minibatch_kl: float = 0.0,
        kl_minibatch_count: int = 9,
        kl_nonfinite_count: int = 0,
        counts: tuple[int, int, int] = (1536, 1536, 1536),
    ) -> Path:
        actual_steps = {
            gate._offset_metric_label(offset): count
            for offset, count in zip(gate.DEFAULT_START_OFFSETS_S, counts)
        }
        expected_steps = {
            gate._offset_metric_label(offset): 1536
            for offset in gate.DEFAULT_START_OFFSETS_S
        }
        coverage = {}
        for label, count in actual_steps.items():
            # The trainer persists both lifetime and per-update metrics.  The
            # gate must classify only the latter, not reject the extra lifetime
            # keys or accidentally use their cumulative values.
            coverage[f"env_runners/episode_start_steps/{label}"] = 99_999
            coverage[f"env_runners/episode_start_steps_current/{label}"] = count
        path = self.checkpoint.parent / "train_iterations.jsonl"
        path.write_text(
            json.dumps(
                {
                    "iteration": iteration,
                    "mean_kl_loss": kl,
                    "max_minibatch_mean_kl_loss": max_minibatch_kl,
                    "min_minibatch_mean_kl_loss": min_minibatch_kl,
                    "kl_minibatch_count": kl_minibatch_count,
                    "kl_nonfinite_count": kl_nonfinite_count,
                    "start_coverage_metrics": coverage,
                    "exact_start_balance": {
                        "pass": actual_steps == expected_steps,
                        "expected_steps": expected_steps,
                        "actual_steps": actual_steps,
                        "learner_batch_pass": actual_steps == expected_steps,
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
                        "advantage_counts": actual_steps,
                        "missing": [],
                        "unexpected": [],
                        "mismatched": {},
                    },
                }
            )
            + "\n",
            encoding="utf-8",
        )
        return path

    def test_checkpoint_must_be_nonempty_rl_module_last_layout(self) -> None:
        self.assertEqual(gate._validate_rl_module_checkpoint(self.checkpoint), [])

        for basename in ("rl_module_best", "checkpoint_last"):
            with self.subTest(basename=basename):
                wrong = self.root / basename
                wrong.mkdir()
                for filename in gate.REQUIRED_RL_MODULE_FILES:
                    (wrong / filename).write_bytes(b"checkpoint-data")
                failures = gate._validate_rl_module_checkpoint(wrong)
                self.assertTrue(
                    any("basename" in failure for failure in failures), failures
                )

        missing = self.root / "missing" / "rl_module_last"
        failures = gate._validate_rl_module_checkpoint(missing)
        self.assertTrue(any("not a directory" in failure for failure in failures))

        for filename in gate.REQUIRED_RL_MODULE_FILES:
            path = self.checkpoint / filename
            original = path.read_bytes()
            with self.subTest(missing_file=filename):
                path.unlink()
                failures = gate._validate_rl_module_checkpoint(self.checkpoint)
                self.assertTrue(
                    any(filename in failure and "missing" in failure for failure in failures),
                    failures,
                )
                path.write_bytes(original)
            with self.subTest(empty_file=filename):
                path.write_bytes(b"")
                failures = gate._validate_rl_module_checkpoint(self.checkpoint)
                self.assertTrue(
                    any(filename in failure and "empty" in failure for failure in failures),
                    failures,
                )
                path.write_bytes(original)
            with self.subTest(directory_instead_of_file=filename):
                path.unlink()
                path.mkdir()
                failures = gate._validate_rl_module_checkpoint(self.checkpoint)
                self.assertTrue(
                    any(filename in failure and "regular file" in failure for failure in failures),
                    failures,
                )
                path.rmdir()
                path.write_bytes(original)

    def test_classification_passes_complete_contract_and_writes_report(self) -> None:
        specs = gate._rollout_specs(gate.DEFAULT_START_OFFSETS_S)
        summaries = [
            self._write_summary(f"summary_{index}", spec)
            for index, spec in enumerate(specs)
        ]
        output = self.root / "gate_output"
        config = gate.GateConfig(
            checkpoint=self.checkpoint,
            output_dir=output,
            max_reserve_norm_nm=600.0,
            deterministic_summaries=tuple(summaries[:3]),
            stochastic_summary=summaries[3],
            train_iterations=self._write_train_iterations(),
        )

        report = gate.evaluate_gate(config)

        self.assertTrue(report["ok"])
        self.assertEqual(report["status"], "PASS")
        self.assertEqual([item["status"] for item in report["rollouts"]], ["PASS"] * 4)
        self.assertEqual(report["training"]["status"], "PASS")
        self.assertFalse(report["checkpoint_promoted"])
        self.assertFalse(report["checkpoint_copied"])
        persisted = json.loads((output / "robust_gate.json").read_text(encoding="utf-8"))
        self.assertTrue(persisted["ok"])
        self.assertEqual(persisted["schema_version"], 2)
        training_contract = persisted["contract"]["training"]
        self.assertEqual(
            training_contract["mean_kl_loss_scope"], "legacy_last_minibatch"
        )
        self.assertEqual(
            training_contract["max_minibatch_mean_kl_loss"][
                "maximum_inclusive"
            ],
            0.01,
        )
        self.assertEqual(training_contract["kl_minibatch_count"], 9)
        self.assertEqual(training_contract["kl_nonfinite_count"], 0)
        self.assertEqual(
            training_contract["iteration"], gate.EXPECTED_TRAINING_ITERATION
        )
        self.assertEqual(
            persisted["reserve_gate_mode"], gate.RESERVE_MODE_LEGACY_SCALAR
        )

    def test_condition_matched_reference_passes_at_casewise_equality(self) -> None:
        specs = gate._rollout_specs(gate.DEFAULT_START_OFFSETS_S)
        reference_checkpoint = self._make_checkpoint("h0")
        reference_caps = (510.0, 430.0, 620.0, 596.1965626236697)
        reference_summaries = tuple(
            self._write_summary(
                f"h0_{index}",
                spec,
                checkpoint=reference_checkpoint,
                reserve_norm_max_nm=reference_caps[index],
            )
            for index, spec in enumerate(specs)
        )
        candidate_summaries = [
            self._write_summary(
                f"candidate_equal_{index}",
                spec,
                reserve_norm_max_nm=reference_caps[index],
            )
            for index, spec in enumerate(specs)
        ]

        report = gate.evaluate_gate(
            gate.GateConfig(
                checkpoint=self.checkpoint,
                output_dir=self.root / "condition_matched_equal",
                reserve_reference_checkpoint=reference_checkpoint,
                reserve_reference_summaries=reference_summaries,
                deterministic_summaries=tuple(candidate_summaries[:3]),
                stochastic_summary=candidate_summaries[3],
                train_iterations=self._write_train_iterations(),
            )
        )

        self.assertTrue(report["ok"])
        self.assertEqual(
            report["reserve_gate_mode"], gate.RESERVE_MODE_CONDITION_MATCHED
        )
        self.assertEqual(report["reserve_reference"]["status"], "PASS")
        self.assertFalse(report["reserve_reference_failed"])
        for index, result in enumerate(report["rollouts"]):
            self.assertEqual(result["status"], "PASS")
            contract = result["reserve_contract"]
            self.assertEqual(
                contract["mode"], gate.RESERVE_MODE_CONDITION_MATCHED
            )
            self.assertEqual(
                contract["reference_cap_nm"], reference_caps[index]
            )
            expected_tolerance = max(
                1.0e-6, 1.0e-9 * reference_caps[index]
            )
            self.assertEqual(
                contract["numerical_tolerance_nm"], expected_tolerance
            )
            self.assertEqual(len(contract["reference_summary_sha256"]), 64)
        reference_case = report["reserve_reference"]["case_caps"][2]
        self.assertEqual(reference_case["reference_cap_nm"], 620.0)
        self.assertEqual(len(reference_case["summary_sha256"]), 64)
        self.assertIsNone(
            report["contract"]["reserve_gate"]["percent_tolerance"]
        )

    def test_condition_matched_never_uses_another_cases_cap(self) -> None:
        specs = gate._rollout_specs(gate.DEFAULT_START_OFFSETS_S)
        reference_checkpoint = self._make_checkpoint("h0_cross_condition")
        # The nominal candidate is below the first condition's 700 Nm cap but
        # above its own 400 Nm cap.  A scalar/cross-condition implementation
        # would incorrectly pass it.
        reference_caps = (700.0, 400.0, 650.0, 590.0)
        candidate_values = (700.0, 450.0, 650.0, 590.0)
        reference_summaries = tuple(
            self._write_summary(
                f"h0_cross_{index}",
                spec,
                checkpoint=reference_checkpoint,
                reserve_norm_max_nm=reference_caps[index],
            )
            for index, spec in enumerate(specs)
        )
        candidate_summaries = [
            self._write_summary(
                f"candidate_cross_{index}",
                spec,
                reserve_norm_max_nm=candidate_values[index],
            )
            for index, spec in enumerate(specs)
        ]

        report = gate.evaluate_gate(
            gate.GateConfig(
                checkpoint=self.checkpoint,
                output_dir=self.root / "condition_matched_cross",
                reserve_reference_checkpoint=reference_checkpoint,
                reserve_reference_summaries=reference_summaries,
                deterministic_summaries=tuple(candidate_summaries[:3]),
                stochastic_summary=candidate_summaries[3],
                train_iterations=self._write_train_iterations(),
            )
        )

        self.assertFalse(report["ok"])
        self.assertEqual(report["reserve_reference"]["status"], "PASS")
        self.assertEqual(report["rollouts"][0]["status"], "PASS")
        nominal = report["rollouts"][1]
        self.assertEqual(nominal["status"], "FAIL")
        self.assertIn("reserve_norm_max_nm", nominal["failed_checks"])
        self.assertEqual(nominal["reserve_contract"]["reference_cap_nm"], 400.0)
        self.assertEqual(
            nominal["reserve_contract"]["reference_summary_path"],
            str(reference_summaries[1].resolve()),
        )

    def test_condition_matched_invalid_baseline_and_incomplete_config_fail(self) -> None:
        specs = gate._rollout_specs(gate.DEFAULT_START_OFFSETS_S)
        reference_checkpoint = self._make_checkpoint("h0_invalid")
        reference_summaries = []
        candidate_summaries = []
        for index, spec in enumerate(specs):
            reference_summaries.append(
                self._write_summary(
                    f"h0_invalid_{index}",
                    spec,
                    checkpoint=reference_checkpoint,
                    mutations={"steps": 499} if index == 0 else None,
                )
            )
            candidate_summaries.append(
                self._write_summary(f"candidate_for_invalid_{index}", spec)
            )

        invalid_report = gate.evaluate_gate(
            gate.GateConfig(
                checkpoint=self.checkpoint,
                output_dir=self.root / "invalid_baseline",
                reserve_reference_checkpoint=reference_checkpoint,
                reserve_reference_summaries=tuple(reference_summaries),
                deterministic_summaries=tuple(candidate_summaries[:3]),
                stochastic_summary=candidate_summaries[3],
                train_iterations=self._write_train_iterations(),
            )
        )
        self.assertFalse(invalid_report["ok"])
        self.assertEqual(invalid_report["reserve_reference"]["status"], "FAIL")
        self.assertIn(
            "steps",
            invalid_report["reserve_reference"]["rollouts"][0][
                "failed_checks"
            ],
        )
        self.assertIn(
            "condition_matched_reserve_reference",
            invalid_report["rollouts"][0]["failed_checks"],
        )

        incomplete_report = gate.evaluate_gate(
            gate.GateConfig(
                checkpoint=self.checkpoint,
                output_dir=self.root / "incomplete_reference_config",
                reserve_reference_checkpoint=reference_checkpoint,
                deterministic_summaries=tuple(candidate_summaries[:3]),
                stochastic_summary=candidate_summaries[3],
                train_iterations=self._write_train_iterations(),
            )
        )
        self.assertFalse(incomplete_report["ok"])
        self.assertTrue(
            any(
                "requires both" in failure
                for failure in incomplete_report["configuration_failures"]
            )
        )

    def test_condition_matched_requires_valid_reference_rlmodule_layout(self) -> None:
        specs = gate._rollout_specs(gate.DEFAULT_START_OFFSETS_S)
        invalid_checkpoint = self.root / "h0_wrong" / "rl_module_best"
        invalid_checkpoint.mkdir(parents=True)
        for filename in gate.REQUIRED_RL_MODULE_FILES:
            (invalid_checkpoint / filename).write_bytes(b"checkpoint-data")
        reference_summaries = tuple(
            self._write_summary(
                f"h0_wrong_layout_{index}",
                spec,
                checkpoint=invalid_checkpoint,
            )
            for index, spec in enumerate(specs)
        )
        candidate_summaries = [
            self._write_summary(f"candidate_wrong_layout_{index}", spec)
            for index, spec in enumerate(specs)
        ]

        report = gate.evaluate_gate(
            gate.GateConfig(
                checkpoint=self.checkpoint,
                output_dir=self.root / "invalid_reference_layout",
                reserve_reference_checkpoint=invalid_checkpoint,
                reserve_reference_summaries=reference_summaries,
                deterministic_summaries=tuple(candidate_summaries[:3]),
                stochastic_summary=candidate_summaries[3],
                train_iterations=self._write_train_iterations(),
            )
        )

        self.assertFalse(report["ok"])
        self.assertEqual(report["reserve_reference"]["checkpoint"]["status"], "FAIL")
        self.assertTrue(
            any("basename" in failure for failure in report["configuration_failures"])
        )

    def test_condition_matched_candidate_above_cap_and_tolerance_fails(self) -> None:
        specs = gate._rollout_specs(gate.DEFAULT_START_OFFSETS_S)
        reference_checkpoint = self._make_checkpoint("h0_tolerance")
        cap = 600.0
        tolerance = gate._reserve_numerical_tolerance_nm(cap)
        reference_summaries = tuple(
            self._write_summary(
                f"h0_tolerance_{index}",
                spec,
                checkpoint=reference_checkpoint,
                reserve_norm_max_nm=cap,
            )
            for index, spec in enumerate(specs)
        )
        candidate_summaries = [
            self._write_summary(
                f"candidate_tolerance_{index}",
                spec,
                reserve_norm_max_nm=(
                    cap + tolerance + 1.0e-7 if index == 0 else cap
                ),
            )
            for index, spec in enumerate(specs)
        ]

        report = gate.evaluate_gate(
            gate.GateConfig(
                checkpoint=self.checkpoint,
                output_dir=self.root / "candidate_above_tolerance",
                reserve_reference_checkpoint=reference_checkpoint,
                reserve_reference_summaries=reference_summaries,
                deterministic_summaries=tuple(candidate_summaries[:3]),
                stochastic_summary=candidate_summaries[3],
                train_iterations=self._write_train_iterations(),
            )
        )

        self.assertFalse(report["ok"])
        first = report["rollouts"][0]
        self.assertEqual(first["status"], "FAIL")
        self.assertIn("reserve_norm_max_nm", first["failed_checks"])
        self.assertEqual(
            first["reserve_contract"]["numerical_tolerance_nm"], tolerance
        )
        self.assertEqual(
            first["reserve_contract"]["maximum_candidate_value_inclusive_nm"],
            cap + tolerance,
        )

    def test_rollout_contract_is_fail_closed_for_every_required_field(self) -> None:
        spec = gate._rollout_specs(gate.DEFAULT_START_OFFSETS_S)[3]
        valid = self._summary(spec)
        wrong_checkpoint = self.root / "other_checkpoint"
        wrong_checkpoint.mkdir()
        mutations = {
            "summary_ok": ("ok", False),
            "checkpoint": ("checkpoint", str(wrong_checkpoint)),
            "offset": ("episode_start_offset_s", spec.offset_s + 0.01),
            "action_mode": ("action_mode", "delta"),
            "action": ("action_selection", "deterministic"),
            "seed": ("action_seed", 124),
            "sigma": ("exploration_std_mean", [0.0075, 0.0075]),
            "sigma_shape": ("exploration_std_mean", [0.005]),
            "noise_rms_zero": ("exploration_noise_realized_rms", [0.0, 0.005]),
            "noise_rms_low": ("exploration_noise_realized_rms", [0.0024, 0.005]),
            "noise_rms_high": ("exploration_noise_realized_rms", [0.0076, 0.005]),
            "noise_rms_shape": ("exploration_noise_realized_rms", [0.005]),
            "steps": ("steps", 499),
            "end_reason": ("end_reason", "grf_penetration"),
            "terminated": ("terminated", True),
            "truncated": ("truncated", False),
            "cycles": ("phase_valid_cycle_count", 1),
            "penetration_strict": ("grf_penetration_max_m", 0.025),
            "penetration_negative": ("grf_penetration_max_m", -0.001),
            "penetration_samples": ("grf_penetration_samples", 499),
            "clipping": ("action_clipped_steps", 1),
            "action_shape": ("action_shape", [4]),
            "action_shape_bool": ("action_shape", [True]),
            "action_shape_float": ("action_shape", [2.0]),
            "actor_features": ("n_actor", 34),
            "observation_features": ("n_observation", 83),
            "reserve_limit": ("reserve_norm_max_nm", 600.000001),
            "reserve_negative": ("reserve_norm_max_nm", -0.001),
            "reserve_samples": ("reserve_norm_samples", 499),
        }
        for label, (key, value) in mutations.items():
            with self.subTest(label=label):
                summary = copy.deepcopy(valid)
                summary[key] = value
                result = gate.classify_rollout_summary(
                    summary,
                    expected_checkpoint=self.checkpoint,
                    spec=spec,
                    expected_sigma=0.005,
                    max_reserve_norm_nm=600.0,
                )
                self.assertEqual(result["status"], "FAIL")

    def test_missing_and_nonfinite_values_fail(self) -> None:
        spec = gate._rollout_specs(gate.DEFAULT_START_OFFSETS_S)[3]
        for key in (
            "action_mode",
            "action_shape",
            "exploration_std_mean",
            "exploration_noise_realized_rms",
            "grf_penetration_max_m",
            "grf_penetration_samples",
            "reserve_norm_max_nm",
            "reserve_norm_samples",
        ):
            with self.subTest(missing=key):
                summary = self._summary(spec)
                del summary[key]
                result = gate.classify_rollout_summary(
                    summary,
                    expected_checkpoint=self.checkpoint,
                    spec=spec,
                    expected_sigma=0.005,
                    max_reserve_norm_nm=600.0,
                )
                self.assertEqual(result["status"], "FAIL")

        for field in ("reserve_norm_max_nm", "grf_penetration_max_m"):
            for value in (math.nan, math.inf, -math.inf):
                with self.subTest(field=field, nonfinite=value):
                    summary = self._summary(spec)
                    summary[field] = value
                    result = gate.classify_rollout_summary(
                        summary,
                        expected_checkpoint=self.checkpoint,
                        spec=spec,
                        expected_sigma=0.005,
                        max_reserve_norm_nm=600.0,
                    )
                    self.assertEqual(result["status"], "FAIL")

        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(noise_rms_nonfinite=value):
                summary = self._summary(spec)
                summary["exploration_noise_realized_rms"] = [0.005, value]
                result = gate.classify_rollout_summary(
                    summary,
                    expected_checkpoint=self.checkpoint,
                    spec=spec,
                    expected_sigma=0.005,
                    max_reserve_norm_nm=600.0,
                )
                self.assertEqual(result["status"], "FAIL")

    def test_deterministic_rollout_requires_explicit_null_noise_rms(self) -> None:
        spec = gate._rollout_specs(gate.DEFAULT_START_OFFSETS_S)[0]
        for value in ([0.005, 0.005], "<missing>"):
            with self.subTest(value=value):
                summary = self._summary(spec)
                if value == "<missing>":
                    del summary["exploration_noise_realized_rms"]
                else:
                    summary["exploration_noise_realized_rms"] = value
                result = gate.classify_rollout_summary(
                    summary,
                    expected_checkpoint=self.checkpoint,
                    spec=spec,
                    expected_sigma=0.005,
                    max_reserve_norm_nm=600.0,
                )
                self.assertEqual(result["status"], "FAIL")

    def test_rollout_collector_rejects_invalid_nonnegative_metrics(self) -> None:
        self.assertEqual(
            rollout_eval._validated_nonnegative_rollout_metric(
                0.0, field="metric"
            ),
            0.0,
        )
        self.assertEqual(
            rollout_eval._validated_nonnegative_rollout_metric(
                1.25, field="metric"
            ),
            1.25,
        )
        for value in (-0.001, math.nan, math.inf, -math.inf, None, True):
            with self.subTest(value=value):
                with self.assertRaisesRegex(RuntimeError, "metric"):
                    rollout_eval._validated_nonnegative_rollout_metric(
                        value, field="metric"
                    )

    def test_rollout_summary_emits_action_mode(self) -> None:
        source = Path(rollout_eval.__file__).read_text(encoding="utf-8")
        self.assertIn('"action_mode": args.action_mode', source)

    def test_training_gate_requires_kl_and_exact_1536_balance(self) -> None:
        passing = gate.classify_training_iterations(
            self._write_train_iterations(kl=0.01),
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )
        self.assertEqual(passing["status"], "PASS")

        high_kl = gate.classify_training_iterations(
            self._write_train_iterations(kl=0.0100001),
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )
        self.assertEqual(high_kl["status"], "FAIL")
        self.assertIn("mean_kl_loss", high_kl["failed_checks"])

        negative_kl = gate.classify_training_iterations(
            self._write_train_iterations(kl=-1.0e-9),
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )
        self.assertEqual(negative_kl["status"], "FAIL")
        self.assertIn("mean_kl_loss", negative_kl["failed_checks"])

        missing_kl_path = self._write_train_iterations()
        missing_kl_record = json.loads(
            missing_kl_path.read_text(encoding="utf-8")
        )
        del missing_kl_record["mean_kl_loss"]
        missing_kl_path.write_text(
            json.dumps(missing_kl_record) + "\n", encoding="utf-8"
        )
        missing_kl = gate.classify_training_iterations(
            missing_kl_path,
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )
        self.assertEqual(missing_kl["status"], "FAIL")
        self.assertIn("mean_kl_loss", missing_kl["failed_checks"])

        for invalid_kl in (math.nan, math.inf, -math.inf):
            with self.subTest(invalid_kl=invalid_kl):
                invalid_path = self._write_train_iterations(kl=invalid_kl)
                invalid_result = gate.classify_training_iterations(
                    invalid_path,
                    start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
                )
                self.assertEqual(invalid_result["status"], "FAIL")

        unbalanced = gate.classify_training_iterations(
            self._write_train_iterations(counts=(1536, 1535, 1536)),
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )
        self.assertEqual(unbalanced["status"], "FAIL")
        self.assertIn("exact_start_balance.pass", unbalanced["failed_checks"])
        self.assertIn(
            "exact_start_balance.actual_steps", unbalanced["failed_checks"]
        )
        self.assertIn(
            "start_coverage_metrics.episode_start_steps_current",
            unbalanced["failed_checks"],
        )
        self.assertIn(
            "exact_start_balance.advantage_counts", unbalanced["failed_checks"]
        )

    def test_training_gate_selects_requested_milestone_not_last_record(self) -> None:
        path = self._write_train_iterations(iteration=2)
        milestone = json.loads(path.read_text(encoding="utf-8"))
        milestone["iteration"] = 40
        later_failure = copy.deepcopy(milestone)
        later_failure["iteration"] = 41
        later_failure["mean_kl_loss"] = 0.02
        path.write_text(
            "\n".join(
                json.dumps(record)
                for record in (milestone, later_failure)
            )
            + "\n",
            encoding="utf-8",
        )

        result = gate.classify_training_iterations(
            path,
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
            expected_training_iteration=40,
        )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["record_count"], 2)
        self.assertEqual(result["available_iterations"], [40, 41])
        self.assertEqual(result["matching_record_count"], 1)
        self.assertEqual(result["classified_iteration"], 40)

    def test_training_gate_rejects_missing_requested_milestone(self) -> None:
        path = self._write_train_iterations(iteration=2)
        iteration_two = json.loads(path.read_text(encoding="utf-8"))
        iteration_three = copy.deepcopy(iteration_two)
        iteration_three["iteration"] = 3
        path.write_text(
            f"{json.dumps(iteration_two)}\n{json.dumps(iteration_three)}\n",
            encoding="utf-8",
        )

        result = gate.classify_training_iterations(
            path,
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
            expected_training_iteration=5,
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["available_iterations"], [2, 3])
        self.assertEqual(result["matching_record_count"], 0)
        self.assertIsNone(result["classified_iteration"])
        self.assertEqual(
            result["failed_checks"], ["training_iteration_selection"]
        )

    def test_training_gate_rejects_duplicate_requested_milestone(self) -> None:
        path = self._write_train_iterations(iteration=5)
        iteration_five = json.loads(path.read_text(encoding="utf-8"))
        duplicate = copy.deepcopy(iteration_five)
        path.write_text(
            f"{json.dumps(iteration_five)}\n{json.dumps(duplicate)}\n",
            encoding="utf-8",
        )

        result = gate.classify_training_iterations(
            path,
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
            expected_training_iteration=5,
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["available_iterations"], [5, 5])
        self.assertEqual(result["matching_record_count"], 2)
        self.assertIsNone(result["classified_iteration"])
        self.assertEqual(
            result["failed_checks"], ["training_iteration_selection"]
        )

    def test_expected_training_iteration_propagates_to_gate_report(self) -> None:
        specs = gate._rollout_specs(gate.DEFAULT_START_OFFSETS_S)
        summaries = [
            self._write_summary(f"milestone_summary_{index}", spec)
            for index, spec in enumerate(specs)
        ]
        output = self.root / "milestone_gate_output"

        report = gate.evaluate_gate(
            gate.GateConfig(
                checkpoint=self.checkpoint,
                output_dir=output,
                max_reserve_norm_nm=600.0,
                deterministic_summaries=tuple(summaries[:3]),
                stochastic_summary=summaries[3],
                train_iterations=self._write_train_iterations(iteration=40),
                expected_training_iteration=40,
            )
        )

        self.assertTrue(report["ok"])
        self.assertEqual(report["training"]["classified_iteration"], 40)
        self.assertEqual(report["contract"]["training"]["iteration"], 40)
        persisted = json.loads(
            (output / "robust_gate.json").read_text(encoding="utf-8")
        )
        self.assertEqual(persisted["contract"]["training"]["iteration"], 40)

    def test_cli_expected_training_iteration_defaults_and_overrides(self) -> None:
        arguments = [
            "--checkpoint",
            str(self.checkpoint),
            "--output-dir",
            str(self.root / "cli_gate_output"),
            "--max-reserve-norm-nm",
            "600.0",
            "--train-iterations",
            str(self._write_train_iterations()),
        ]

        default_config = gate._config_from_args(
            gate.build_parser().parse_args(arguments)
        )
        overridden_config = gate._config_from_args(
            gate.build_parser().parse_args(
                arguments + ["--expected-training-iteration", "40"]
            )
        )

        self.assertEqual(
            default_config.expected_training_iteration,
            gate.EXPECTED_TRAINING_ITERATION,
        )
        self.assertEqual(overridden_config.expected_training_iteration, 40)

    def test_training_gate_rejects_update_scoped_kl_peak(self) -> None:
        result = gate.classify_training_iterations(
            self._write_train_iterations(
                kl=0.001,
                max_minibatch_kl=0.010001,
                min_minibatch_kl=0.0001,
            ),
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )

        self.assertEqual(result["status"], "FAIL")
        checks = {check["name"]: check["status"] for check in result["checks"]}
        self.assertEqual(checks["mean_kl_loss"], "PASS")
        self.assertEqual(checks["max_minibatch_mean_kl_loss"], "FAIL")
        self.assertIn("max_minibatch_mean_kl_loss", result["failed_checks"])

    def test_training_gate_requires_complete_finite_kl_update_audit(self) -> None:
        fields = (
            "max_minibatch_mean_kl_loss",
            "min_minibatch_mean_kl_loss",
            "kl_minibatch_count",
            "kl_nonfinite_count",
        )
        for field in fields:
            with self.subTest(missing=field):
                path = self._write_train_iterations()
                record = json.loads(path.read_text(encoding="utf-8"))
                del record[field]
                path.write_text(json.dumps(record) + "\n", encoding="utf-8")
                result = gate.classify_training_iterations(
                    path,
                    start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
                )
                self.assertEqual(result["status"], "FAIL")
                self.assertIn(field, result["failed_checks"])

        for field in fields:
            with self.subTest(nonfinite=field):
                path = self._write_train_iterations()
                record = json.loads(path.read_text(encoding="utf-8"))
                record[field] = math.nan
                path.write_text(json.dumps(record) + "\n", encoding="utf-8")
                result = gate.classify_training_iterations(
                    path,
                    start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
                )
                self.assertEqual(result["status"], "FAIL")
                self.assertIn("train_iterations_read", result["failed_checks"])

        invalid_cases = (
            ({"max_minibatch_kl": -1.0e-9}, "max_minibatch_mean_kl_loss"),
            (
                {"min_minibatch_kl": -1.000001e-7},
                "min_minibatch_mean_kl_loss",
            ),
            ({"kl_minibatch_count": -1}, "kl_minibatch_count"),
            ({"kl_nonfinite_count": -1}, "kl_nonfinite_count"),
            ({"kl_nonfinite_count": 1}, "kl_nonfinite_count"),
        )
        for kwargs, failed_check in invalid_cases:
            with self.subTest(kwargs=kwargs):
                result = gate.classify_training_iterations(
                    self._write_train_iterations(**kwargs),
                    start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
                )
                self.assertEqual(result["status"], "FAIL")
                self.assertIn(failed_check, result["failed_checks"])

        tolerated_minimum = gate.classify_training_iterations(
            self._write_train_iterations(min_minibatch_kl=-1.0e-7),
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )
        self.assertEqual(tolerated_minimum["status"], "PASS")

    def test_training_gate_requires_exactly_nine_kl_minibatches(self) -> None:
        result = gate.classify_training_iterations(
            self._write_train_iterations(kl_minibatch_count=8),
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("kl_minibatch_count", result["failed_checks"])

    def test_training_gate_rejects_cyclic_minibatch_reuse(self) -> None:
        path = self._write_train_iterations()
        record = json.loads(path.read_text(encoding="utf-8"))
        balance = record["exact_start_balance"]
        balance["pass"] = False
        balance["learner_batch_pass"] = False
        balance["learner_connector_steps_out"] = 4622
        balance["removed_compaction_rows"] = 0
        balance["compacted_rows"] = 4622
        balance["module_steps_trained"] = 5120
        path.write_text(json.dumps(record) + "\n", encoding="utf-8")

        result = gate.classify_training_iterations(
            path,
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn(
            "exact_start_balance.post_gae_compaction",
            result["failed_checks"],
        )
        self.assertIn(
            "exact_start_balance.module_steps_trained", result["failed_checks"]
        )

    def test_training_gate_requires_explicit_interleaving_attestations(self) -> None:
        path = self._write_train_iterations()
        record = json.loads(path.read_text(encoding="utf-8"))
        balance = record["exact_start_balance"]
        for field in (
            "interleaved_rows",
            "interleaved_start_conditions",
            "interleaved_rows_per_start",
            "max_start_run_length",
        ):
            del balance[field]
        del balance["learner_checks"]
        path.write_text(json.dumps(record) + "\n", encoding="utf-8")

        result = gate.classify_training_iterations(
            path,
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )

        self.assertEqual(result["status"], "FAIL")
        for failed_check in (
            "exact_start_balance.interleaved_rows",
            "exact_start_balance.interleaved_start_conditions",
            "exact_start_balance.interleaved_rows_per_start",
            "exact_start_balance.max_start_run_length",
            "exact_start_balance.learner_checks.start_interleaving",
            "exact_start_balance.learner_checks.single_epoch_contract",
        ):
            self.assertIn(failed_check, result["failed_checks"])

    def test_training_gate_rejects_clustered_start_rows_independently(self) -> None:
        path = self._write_train_iterations()
        record = json.loads(path.read_text(encoding="utf-8"))
        balance = record["exact_start_balance"]
        # Keep the historical aggregate pass flags true deliberately: the gate
        # must verify the direct attestations instead of trusting those flags.
        balance["pass"] = True
        balance["learner_batch_pass"] = True
        balance["max_start_run_length"] = 384
        balance["learner_checks"]["start_interleaving"] = False
        path.write_text(json.dumps(record) + "\n", encoding="utf-8")

        result = gate.classify_training_iterations(
            path,
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn(
            "exact_start_balance.max_start_run_length",
            result["failed_checks"],
        )
        self.assertIn(
            "exact_start_balance.learner_checks.start_interleaving",
            result["failed_checks"],
        )

    def test_training_gate_requires_single_epoch_attestation(self) -> None:
        path = self._write_train_iterations()
        record = json.loads(path.read_text(encoding="utf-8"))
        record["exact_start_balance"]["learner_checks"][
            "single_epoch_contract"
        ] = False
        path.write_text(json.dumps(record) + "\n", encoding="utf-8")

        result = gate.classify_training_iterations(
            path,
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn(
            "exact_start_balance.learner_checks.single_epoch_contract",
            result["failed_checks"],
        )

    def test_training_gate_rejects_extra_start_key(self) -> None:
        path = self._write_train_iterations()
        record = json.loads(path.read_text(encoding="utf-8"))
        record["start_coverage_metrics"][
            "env_runners/episode_start_steps_current/offset_9p000000s"
        ] = 0
        path.write_text(json.dumps(record) + "\n", encoding="utf-8")

        result = gate.classify_training_iterations(
            path,
            start_offsets_s=gate.DEFAULT_START_OFFSETS_S,
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn(
            "start_coverage_metrics.episode_start_steps_current",
            result["failed_checks"],
        )

    def test_launcher_runs_four_processes_serially_with_absolute_action_mode(self) -> None:
        rollout_script = self.root / "rollout_eval.py"
        rollout_script.write_text("# fake rollout entrypoint\n", encoding="utf-8")
        python_executable = self.root / "python"
        python_executable.write_text("fake\n", encoding="utf-8")
        calls: list[list[str]] = []
        specs = gate._rollout_specs(gate.DEFAULT_START_OFFSETS_S)

        def fake_runner(command, *, check):
            self.assertFalse(check)
            index = len(calls)
            calls.append(list(command))
            spec = specs[index]
            output = Path(command[command.index("--output-dir") + 1])
            output.mkdir(parents=True)
            (output / "rollout_summary.json").write_text(
                json.dumps(self._summary(spec)), encoding="utf-8"
            )
            return SimpleNamespace(returncode=0)

        report = gate.evaluate_gate(
            gate.GateConfig(
                checkpoint=self.checkpoint,
                output_dir=self.root / "launched_gate",
                max_reserve_norm_nm=600.0,
                run_rollouts=True,
                python_executable=str(python_executable),
                rollout_script=rollout_script,
                rollout_timeout_s=20.0,
                train_iterations=self._write_train_iterations(),
            ),
            command_runner=fake_runner,
        )

        self.assertTrue(report["ok"])
        self.assertEqual(len(calls), 4)
        self.assertEqual(
            [command[command.index("--action-selection") + 1] for command in calls],
            ["deterministic", "deterministic", "deterministic", "stochastic"],
        )
        for command in calls:
            self.assertEqual(command[command.index("--action-mode") + 1], "absolute")
            self.assertEqual(command[command.index("--seed") + 1], "123")
            self.assertEqual(command.count("--run-timeout-s"), 1)
            self.assertEqual(
                command[command.index("--run-timeout-s") + 1], "20.0"
            )
            self.assertIn("--no-record-outputs", command)
        self.assertNotEqual(calls[0][-1], "&")

    def test_internal_watchdog_timeout_fails_gate_and_writes_report(self) -> None:
        rollout_script = self.root / "rollout_eval.py"
        rollout_script.write_text("# fake rollout entrypoint\n", encoding="utf-8")
        python_executable = self.root / "python"
        python_executable.write_text("fake\n", encoding="utf-8")

        def failed_runner(command, *, check):
            return subprocess.CompletedProcess(command, 124)

        output = self.root / "failed_gate"
        report = gate.evaluate_gate(
            gate.GateConfig(
                checkpoint=self.checkpoint,
                output_dir=output,
                max_reserve_norm_nm=600.0,
                run_rollouts=True,
                python_executable=str(python_executable),
                rollout_script=rollout_script,
                train_iterations=self._write_train_iterations(),
            ),
            command_runner=failed_runner,
        )

        self.assertFalse(report["ok"])
        self.assertEqual(len(report["failed_invocations"]), 4)
        self.assertEqual(
            [item.get("returncode") for item in report["invocations"]],
            [124, 124, 124, 124],
        )
        self.assertTrue((output / "robust_gate.json").is_file())


if __name__ == "__main__":
    unittest.main()
