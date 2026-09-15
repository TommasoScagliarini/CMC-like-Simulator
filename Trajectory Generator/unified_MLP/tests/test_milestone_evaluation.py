"""Milestone selection and unequal-window semantics; unittest, no simulator.

    python -m unittest discover -s "Trajectory Generator/unified_MLP/tests" \
        -p test_milestone_evaluation.py
"""
from __future__ import annotations

import copy
import ast
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


VALIDATION_DIR = Path(__file__).resolve().parents[1] / "validation"
sys.path.insert(0, str(VALIDATION_DIR))
spec = importlib.util.spec_from_file_location("evaluate_training_milestone", VALIDATION_DIR / "evaluate_training_milestone.py")
milestone = importlib.util.module_from_spec(spec)
spec.loader.exec_module(milestone)


def fixture_rollout():
    rows = []
    for i in range(3):
        row = {
            "step": i + 1, "time": (i + 1) * 0.01, "_pre_time": i * 0.01, "_dt": 0.01,
            "policy_segment_times": [i * 0.01, (i + 1) * 0.01],
            "prosthetic_state": {}, "imitation_target_q": {}, "imitation_target_qdot": {},
            "actor_observation_before": {"online_left_cycle_duration_s": 1.5 if i else 0,
                                         "online_left_gait_phase_sin": 0, "online_left_gait_phase_cos": 1,
                                         "gait_phase_sin": 0, "gait_phase_cos": 0},
            "phase_fsm": {"valid_hs_count": i + 1, "valid_to_count": i, "valid_cycle_count": i,
                          "invalid_event_count": 0},
            "reward_terms": {"grf_penetration_m": (0.021, 0.026, 0.029)[i], "u_abs_max": 0.2,
                             "policy_action_clip_fraction": 0, "reserve_norm_nm": 10},
        }
        for joint in milestone.analysis.JOINTS.values():
            error = (i + 1) * (1 if joint == "pros_knee_angle" else 2)
            row["imitation_target_q"][joint] = 0.0
            row["imitation_target_qdot"][joint] = 0.0
            row["prosthetic_state"].update({joint: error, joint + "_vel": error,
                                           joint + "_served_ref": error / 2,
                                           joint + "_served_ref_vel": error / 2})
            prefix = joint + "_sea_"
            count = i + 1
            row["reward_terms"][prefix + "sample_count"] = count
            for metric, suffix, unit in (
                ("tau_spring", "sum_squares_nm2", "nm"), ("torque_error", "sum_squares_nm2", "nm"),
                ("motor_speed", "sum_squares_rad2_s2", "rad_s"),
                ("motor_accel", "sum_squares_rad2_s4", "rad_s2"),
                ("motor_power", "sum_squares_w2", "w"),
            ):
                row["reward_terms"][prefix + metric + "_" + suffix] = count * error**2
                row["reward_terms"][prefix + metric + "_abs_max_" + unit] = error
            for field, value in {"tau_input_abs_max_nm": 100, "tau_input_saturation_count": 0,
                                 "tau_input_plugin_fallback_count": 0, "motor_accel_plugin_fallback_count": 0}.items():
                row["reward_terms"][prefix + field] = value
        rows.append(row)
    return {"rows": rows, "directory": "fixture", "source_hashes": {}, "summary": {
        "ok": True, "steps": 3, "action_selection": "deterministic", "grf_penetration_penalty_threshold_m": 0.02,
        "episode_return": 2, "reward_mean": 2 / 3, "end_reason": "episode_time_limit",
        "terminated": False, "truncated": True,
    }}


def make_checkpoint(run: Path, iteration: int):
    directory = run / f"milestone_iteration_{iteration:06d}"
    module = directory / "rl_module_last"
    module.mkdir(parents=True)
    (module / "module_state.pkl").write_bytes(str(iteration).encode())
    milestone.write_json(directory / "rl_module_last_meta.json", {"logical_iteration": iteration})
    milestone.write_json(module / "unified_training_state.json", {"last_completed_iteration": iteration, "reward_mode": "imitation"})
    milestone.write_json(module / "unified_contract.json", {"actor_feature_names": [f"feature_{i}" for i in range(35)]})


class MilestoneTests(unittest.TestCase):
    def test_best_ignores_future_and_breaks_ties_earliest(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            history = [{"iteration": i, "episode_return_mean": reward} for i, reward in ((1, 3), (2, 5), (4, 5), (50, 1), (51, 100))]
            (root / "train_iterations.jsonl").write_text("\n".join(json.dumps(row) for row in history))
            for i in (2, 50):
                make_checkpoint(root, i)
            result = milestone.select_candidates(root, 50)
            self.assertEqual(result["roles"], {"last": 50, "best_training_return": 2})
            self.assertEqual([c["iteration"] for c in result["candidates"]], [2, 50])
            self.assertNotIn(51, [r["iteration"] for r in result["history_through_milestone"]])
            self.assertFalse(result["best_is_automatically_qualified"])

    def test_best_last_deduplicates_and_mutable_artifact_is_never_used(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            milestone.write_json(root / "summary.json", {"history": [{"iteration": 50, "episode_return_mean": 1}]})
            make_checkpoint(root, 50)
            result = milestone.select_candidates(root, 50)
            self.assertEqual(len(result["candidates"]), 1)
            self.assertEqual(len(result["candidates"][0]["roles"]), 2)
            self.assertIn("milestone_iteration_000050", result["candidates"][0]["checkpoint"])
            (root / "milestone_iteration_000050/rl_module_last_meta.json").unlink()
            with self.assertRaises(FileNotFoundError):
                milestone.select_candidates(root, 50)

    def test_missing_milestone_duplicate_history_or_wrong_actor_fail(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            milestone.write_json(root / "summary.json", {"history": [{"iteration": 5, "episode_return_mean": 1}]})
            with self.assertRaisesRegex(ValueError, "has not completed"):
                milestone.select_candidates(root, 50)
            milestone.write_json(root / "summary.json", {"history": [{"iteration": 50, "episode_return_mean": 1}] * 2})
            with self.assertRaisesRegex(ValueError, "Duplicate"):
                milestone.select_candidates(root, 50)
            milestone.write_json(root / "summary.json", {"history": [{"iteration": 50, "episode_return_mean": 1}]})
            make_checkpoint(root, 50)
            milestone.write_json(root / "milestone_iteration_000050/rl_module_last/unified_contract.json", {"actor_feature_names": list(range(39))})
            with self.assertRaisesRegex(ValueError, "35-feature"):
                milestone.select_candidates(root, 50)

    def test_startup_regime_event_increments_and_thresholds(self):
        rollout = fixture_rollout()
        case = milestone.describe_case(rollout, 50, "early", ["last"])
        self.assertEqual(case["startup"]["samples"], 1)
        regime = case["online_period_regime"]
        self.assertEqual(regime["samples"], 2)
        self.assertEqual(regime["phase"]["valid_hs_count"], 2)
        self.assertEqual(regime["phase"]["valid_hs_count_cumulative_at_end"], 3)
        self.assertAlmostEqual(case["first_online_period_available_elapsed_s"], 0.01)
        thresholds = case["full_episode"]["safety"]["penetration_thresholds_mm"]
        self.assertEqual([thresholds[k]["policy_samples_above"] for k in ("20", "25", "28")], [3, 2, 1])
        self.assertAlmostEqual(regime["safety"]["penetration_thresholds_mm"]["28"]["policy_fraction_above"], 0.5)

    def test_no_period_means_no_regime_and_no_fabricated_reward(self):
        rollout = fixture_rollout()
        for row in rollout["rows"]:
            row["actor_observation_before"]["online_left_cycle_duration_s"] = 0
        case = milestone.describe_case(rollout, 50, "early", ["last"])
        self.assertIsNone(case["online_period_regime"])
        self.assertIsNone(case["first_online_period_available_elapsed_s"])
        self.assertEqual(case["startup"]["samples"], 3)
        self.assertIsNone(case["startup"]["reward"]["episode_return"])

    def test_subwindow_can_cancel_an_hs_credited_before_the_window(self):
        rollout = fixture_rollout()
        for row, hs, cancelled in zip(rollout["rows"], (0, 1, 0), (0, 0, 1)):
            row["phase_fsm"].update(valid_hs_count=hs, hs_cancelled_count=cancelled)
        full = milestone.window_metrics(rollout, rollout["rows"], full=True)["phase"]
        tail = milestone.window_metrics(rollout, rollout["rows"][2:])["phase"]
        self.assertEqual(full["valid_hs_count"], 0)
        self.assertEqual(full["hs_cancelled_count"], 1)
        self.assertEqual(tail["valid_hs_count"], -1)
        self.assertEqual(tail["valid_hs_count_cumulative_at_end"], 0)
        self.assertEqual(tail["hs_cancelled_count"], 1)
        self.assertIn("net of", tail["event_count_semantics"])

    def test_common_prefix_keeps_both_joints_without_survivor_pooling(self):
        long = fixture_rollout()
        short = copy.deepcopy(long)
        short["rows"] = short["rows"][:1]
        a, b = milestone.analysis.common_rows(long, short)
        metrics = milestone.window_metrics(long, a)
        self.assertEqual(len(a), len(b))
        self.assertAlmostEqual(metrics["planned_coverage_fraction"], 0.01 / 5)
        self.assertAlmostEqual(metrics["tracking"]["knee"]["actual_vs_prescribed"]["position"]["rmse_rad"], 1)
        self.assertAlmostEqual(metrics["tracking"]["ankle"]["actual_vs_prescribed"]["position"]["rmse_rad"], 2)
        self.assertIsNone(metrics["reward"]["episode_return"])
        short["rows"][0]["imitation_target_q"]["pros_knee_angle"] = 1
        with self.assertRaisesRegex(ValueError, "targets differ"):
            milestone.analysis.common_rows(long, short)

    def test_quality_requires_all_three_starts_duration_cycles_and_safety(self):
        case = milestone.describe_case(fixture_rollout(), 50, "early", ["last"])
        cases = []
        for start in milestone.START_OFFSETS:
            c = copy.deepcopy(case)
            c["start"] = start
            c["full_episode"]["duration_s"] = 5.0
            c["full_episode"]["safety"]["penetration_max_mm"] = 27.0
            cases.append(c)
        self.assertTrue(milestone.quality_gate(cases)["pass"])
        self.assertFalse(milestone.quality_gate(cases[:2])["pass"])
        for field, value, error in (("duration_s", 4.99, "duration_5s"),):
            bad = copy.deepcopy(cases)
            bad[1]["full_episode"][field] = value
            self.assertIn("center:" + error, milestone.quality_gate(bad)["failures"])
        bad = copy.deepcopy(cases)
        bad[2]["full_episode"]["phase"]["valid_cycle_count"] = 1
        bad[2]["full_episode"]["safety"]["penetration_max_mm"] = 28
        failures = milestone.quality_gate(bad)["failures"]
        self.assertIn("late:two_valid_cycles", failures)
        self.assertIn("late:penetration_below_28mm", failures)

    def test_command_has_fixed_reproducible_budget_and_no_training(self):
        command = milestone.rollout_command("python-custom", {"checkpoint": "immutable"}, "late", Path("outputs"), Path("snapshot.yaml"))
        self.assertEqual(command[0], "python-custom")
        for flag, expected in (("--max-steps", "500"), ("--seed", "123"), ("--episode-duration", "5.0"), ("--action-selection", "deterministic")):
            self.assertEqual(command[command.index(flag) + 1], expected)
        self.assertIn("--record-outputs", command)
        self.assertIn("--record-policy-trace", command)
        self.assertNotIn("train_ppo", " ".join(command))

    def test_default_analysis_only_writes_pending_plan_never_executes(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            milestone.write_json(root / "summary.json", {"history": [{"iteration": 50, "episode_return_mean": 1}]})
            make_checkpoint(root, 50)
            (root / "training_cfg.resolved.yaml").write_text("simulation: {}\n")
            args = milestone.argparse.Namespace(run_dir=root, output_dir=root / "evaluation", milestone=50, python="python", execute_rollouts=False)
            with patch.object(milestone.subprocess, "run", side_effect=AssertionError("execution forbidden")):
                result = milestone.evaluate(args)
            self.assertEqual(result["status"], "pending_rollouts")
            self.assertEqual(len(result["pending_rollouts"]), 3)
            plan = milestone.read_json(root / "evaluation/evaluation_plan.json")
            self.assertEqual(len(plan["commands"]), 3)
            self.assertFalse((root / "evaluation/milestone_evaluation.json").exists())
            # Execute the real output resolver's AST without importing RLlib or
            # OpenSim. This catches the strict category boundary of rollout_eval.
            tree = ast.parse((VALIDATION_DIR.parent / "rollout_eval.py").read_text())
            function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_resolve_category_output_dir")
            namespace = {"_cli_path": Path, "_under": lambda p, r: p.is_relative_to(r.resolve())}
            exec(compile(ast.Module(body=[function], type_ignores=[]), "rollout_resolver", "exec"), namespace)
            for entry in plan["commands"]:
                command = entry["argv"]
                path = Path(command[command.index("--output-dir") + 1])
                resolved = namespace["_resolve_category_output_dir"](str(path), "unused", milestone.UNIFIED_DIR / "runs/rollout", "rollout")
                self.assertEqual(resolved, path)

    def test_runtime_config_comparison_ignores_training_budget_but_not_physics(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "config.yaml"
            first = "simulation:\n  iterations: 50\n  episode_start_offset_choices_s:\n  - 1\n  - 2\n  pros_ref_cutoff_hz: 4.0\ngrf:\n  mode: online\nreward:\n  imitation_weight: 4.0\nlogging:\n  output_dir: old\n"
            path.write_text(first)
            expected = milestone.runtime_config_lines(path)
            path.write_text(first.replace("iterations: 50", "iterations: 100").replace("output_dir: old", "output_dir: new"))
            self.assertEqual(milestone.runtime_config_lines(path), expected)
            path.write_text(first.replace("pros_ref_cutoff_hz: 4.0", "pros_ref_cutoff_hz: 6.0"))
            self.assertNotEqual(milestone.runtime_config_lines(path), expected)

    def test_milestone_100_reuses_verified_50_references_and_rejects_tampering(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            run = root / "training"
            run.mkdir()
            output50 = root / "validation50"
            output50.mkdir()
            report = {"status": "complete", "selection": {"milestone": 50, "roles": {"last": 50, "best_training_return": 50},
                                                           "candidates": [{"iteration": 50}]},
                      "plan": {"run_dir": str(run)}, "cases": []}
            evaluated = {}
            for start in milestone.START_OFFSETS:
                directory = output50 / start
                directory.mkdir()
                rollout = fixture_rollout()
                milestone.write_json(directory / "rollout_summary.json", rollout["summary"])
                milestone.write_json(directory / "rollout_policy_trace.json", rollout["rows"])
                report["cases"].append({"checkpoint_iteration": 50, "start": start,
                                        "source_directory": str(directory), "source_hashes": {
                                            "rollout_summary.json": milestone.sha256(directory / "rollout_summary.json")}})
                evaluated[(100, start)] = rollout
            milestone.write_json(output50 / "milestone_evaluation.json", report)
            with patch.object(milestone, "UNIFIED_DIR", root / "unified"), patch.object(milestone, "validate_rollout") as verify, patch.object(milestone, "validate_pair"):
                references, exclusions = milestone.previous_milestone_reference(run, root / "validation100", evaluated)
                self.assertEqual(len(references), 3)
                self.assertEqual(verify.call_count, 3)
                self.assertEqual(exclusions, [])
                self.assertEqual(references[(50, "early")]["_reference_roles"], ["reference_milestone50"])
                (output50 / "early/rollout_summary.json").write_text("{}")
                with self.assertRaisesRegex(ValueError, "reference changed"):
                    milestone.previous_milestone_reference(run, root / "validation100", evaluated)


if __name__ == "__main__":
    unittest.main()
