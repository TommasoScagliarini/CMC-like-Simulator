"""Offline campaign orchestration checks: no simulator, subprocess or variant build.

Run with ``python -m unittest discover -s 'Trajectory Generator/unified_MLP/tests'
-p test_feasibility_campaign.py``. All writable fixtures live in temporary folders.
"""
from __future__ import annotations

import ast
import argparse
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

TRAJECTORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TRAJECTORY))
from unified_MLP.validation import run_feasibility39 as campaign_module
from unified_MLP.validation import run_authorized_plan as authorized


def declared_flags(path):
    """Read argparse declarations without importing simulator-facing modules."""
    flags = set()
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute) or node.func.attr != "add_argument":
            continue
        current = [arg.value for arg in node.args if isinstance(arg, ast.Constant) and isinstance(arg.value, str) and arg.value.startswith("--")]
        flags.update(current)
        if any(keyword.arg == "action" and isinstance(keyword.value, ast.Attribute) and keyword.value.attr == "BooleanOptionalAction" for keyword in node.keywords):
            flags.update("--no-" + flag[2:] for flag in current)
    return flags


class CampaignTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.plan = self.root / "plan"
        self.plan.mkdir()
        self.campaign = campaign_module.Campaign(self.plan)
        self.campaign.variant = self.root / "unified_MLP_39"

    def write(self, path, data):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data), encoding="utf-8")

    def test_rollout_command_only_uses_supported_cli_flags(self):
        run = self.campaign.variant / "runs/training/fixture"
        checkpoint = run / "milestone_iteration_000050/rl_module_last"
        checkpoint.mkdir(parents=True)
        (checkpoint / "module_state.pkl").write_bytes(b"fixture")
        names = [f"feature_{i}" for i in range(35)]
        self.write(self.plan / "pilot5_snapshot/unified_contract.json", {"actor_feature_names": names})
        target_names = [f"prescribed_left_{i}" for i in range(4)]
        result = {"summary": {"actor_feature_names": names + target_names,
                              "action_seed": 20260914, "action_selection": "deterministic",
                              "episode_start_offset_s": campaign_module.STARTS["early"],
                              "record_policy_trace": True, "record_outputs": True,
                              "checkpoint": str(checkpoint)},
                  "rows": [{"target_availability": {"episode_probability": 0.0, "visible": False},
                            "actor_observation_before": {name: 0.0 for name in target_names}}]}
        commands = []
        with patch.object(self.campaign, "execute", side_effect=lambda label, command, **kw: commands.append(command)), patch.object(campaign_module, "loaded_rollout", return_value=result):
            self.campaign.rollout(run, 50, "early", 0.0, noise_seed=20260914)
        self.assertEqual(len(commands), 1)
        allowed = declared_flags(TRAJECTORY / "unified_MLP/rollout_eval.py")
        allowed.update(declared_flags(TRAJECTORY / "unified_MLP/validation/feasibility39_bundle/target_availability.py"))
        used = {token for token in commands[0] if token.startswith("--")}
        self.assertFalse(used - allowed, f"Unsupported rollout flags: {used - allowed}")
        self.assertIn("--record-policy-trace", used)
        self.assertIn("--record-outputs", used)

    def test_reused_training_cannot_claim_completion_without_history(self):
        run = self.campaign.variant / "runs/training/feasibility_B_seed123_imitation"
        self.write(run / "summary.json", {"ok": True, "stop_reason": "completed", "iterations_completed": 10, "skipped_iterations": []})
        with patch.object(self.campaign, "configuration", return_value=self.root / "config.yaml"), patch.object(self.campaign, "execute", side_effect=AssertionError("no child should launch")):
            with self.assertRaises((OSError, ValueError, RuntimeError)):
                self.campaign.training("B", 123, 10)

    def test_reused_training_cannot_claim_completion_with_skips(self):
        run = self.campaign.variant / "runs/training/feasibility_B_seed123_imitation"
        self.write(run / "summary.json", {"ok": True, "stop_reason": "completed", "iterations_completed": 10, "skipped_iterations": [2]})
        with patch.object(self.campaign, "configuration", return_value=self.root / "config.yaml"), patch.object(self.campaign, "execute", side_effect=AssertionError("no child should launch")):
            with self.assertRaises((OSError, ValueError, RuntimeError)):
                self.campaign.training("B", 123, 10)

    def test_preconditions_run_before_any_materialization_or_experiment(self):
        with patch.object(campaign_module, "preconditions", side_effect=ValueError("missing V1 evaluation")), patch.object(self.campaign, "execute") as execute, patch.object(self.campaign, "run_seed") as run_seed:
            with self.assertRaisesRegex(ValueError, "missing V1"):
                self.campaign.run()
        execute.assert_not_called()
        run_seed.assert_not_called()
        self.assertFalse((self.plan / "feasibility_result.json").exists())

    def run_mock_campaign(self, *, fail_action=None):
        protocol = self.root / "reports/plans/2026-09-14_protocollo_fattibilita_unificata39.md"
        protocol.parent.mkdir(parents=True, exist_ok=True)
        protocol.write_text("Frozen mock protocol")
        unified = self.root / "unified_MLP"
        for target in (50, 100):
            self.write(unified / "runs/validation" / f"pretraining_v1_pen1_milestone_{target}_2026-09-14/milestone_evaluation.json",
                       {"cases": [{"source_directory": f"fixture_cp{target}"}]})
        events = []

        def execute(label, command, **kwargs):
            events.append(label)
            if label == fail_action:
                raise RuntimeError(f"failure at {label}")

        def run_seed(seed):
            events.append(f"seed_{seed}")
            return {"seed": seed, "replication_trigger": {"trigger_replications": False}}

        with patch.object(campaign_module, "REPO", self.root), patch.object(campaign_module, "UNIFIED", unified), patch.object(campaign_module, "preconditions", side_effect=lambda plan: events.append("preconditions")), patch.object(self.campaign, "execute", side_effect=execute), patch.object(self.campaign, "run_seed", side_effect=run_seed), patch.object(self.campaign, "smoke", side_effect=lambda: events.append("smoke")):
            self.campaign.run()
        return events

    def test_prerequisites_precede_abc_and_completion_requires_last_stage(self):
        events = self.run_mock_campaign()
        self.assertLess(events.index("preconditions"), events.index("materialize_variant"))
        self.assertLess(events.index("software_isolation_tests"), events.index("seed_123"))
        self.assertLess(events.index("materialize_variant"), events.index("seed_123"))
        self.assertLess(events.index("smoke"), events.index("seed_123"))
        result = campaign_module.read_json(self.plan / "feasibility_result.json")
        self.assertTrue(result["protocol_completed"])
        self.assertEqual(result["training_seeds"], [123])

    def test_failed_observability_does_not_publish_false_protocol_completion(self):
        with self.assertRaisesRegex(RuntimeError, "observability"):
            self.run_mock_campaign(fail_action="observability")
        self.assertFalse((self.plan / "feasibility_result.json").exists())
        self.assertFalse(self.campaign.state["protocol_completed"])

    def test_replication_trigger_rejects_loss_of_valid_cycles_even_with_lower_errors(self):
        from test_milestone_evaluation import fixture_rollout
        masked = {}
        for start in campaign_module.STARTS:
            for arm, cycles in (("B", 3), ("C", 1)):
                rollout = fixture_rollout()
                case = campaign_module.milestone_metrics.describe_case(rollout, 50, start, [arm])
                case["full_episode"]["duration_s"] = 5.0
                case["full_episode"]["phase"]["valid_cycle_count"] = cycles
                case["full_episode"]["safety"]["penetration_max_mm"] = 20.0
                rollout["case"] = case
                masked[(arm, start)] = rollout
        pair = {side: {"tracking": {joint: {source + "_vs_prescribed": {"position": {"rmse_rad": error}}
                                           for source in ("actual", "served")}
                                  for joint in campaign_module.metrics.JOINTS}}
                for side, error in (("first", 2.0), ("second", 1.0))}
        with patch.object(campaign_module.milestone_metrics, "describe_case", side_effect=lambda rollout, *args: rollout["case"]), patch.object(campaign_module, "paired_comparison", return_value=pair):
            result = campaign_module.favorable_pilot(masked)
        self.assertFalse(result["trigger_replications"], "Better tracking must not hide losing complete gait cycles")

    def run_mock_authorized_plan(self, fail_stage=None):
        """Drive the actual controller main/execute path with fake child handles."""
        unified = self.root / "authorized/unified_MLP"
        run = unified / "runs/training/pilot"
        plan = unified / "runs/plan"
        run.mkdir(parents=True, exist_ok=True)
        plan.mkdir(parents=True, exist_ok=True)
        self.write(plan / "feasibility_result.json", {"protocol_completed": True})
        args = argparse.Namespace(run_dir=run, plan_dir=plan, attach_training_to50=True, run_feasibility=True)
        events = []

        class Child:
            pid = 12345

            def __init__(self, code):
                self.code = code

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

            def wait(self, timeout=None):
                return self.code

        def popen(command, **kwargs):
            if "--milestone" in command:
                stage = "evaluation_" + command[command.index("--milestone") + 1]
            elif "--iterations" in command:
                stage = "training_to_" + command[command.index("--iterations") + 1]
            else:
                stage = "feasibility"
            events.append(stage)
            return Child(7 if stage == fail_stage else 0)

        def attach(*args):
            events.append("attach_training_to_50")
            return {"ok": True, "iterations_completed": 50}

        with patch.object(authorized, "UNIFIED", unified), patch.object(authorized, "REPO", self.root), patch.object(authorized.argparse.ArgumentParser, "parse_args", return_value=args), patch.object(authorized, "source_is_preserved"), patch.object(authorized, "wait_for_existing_training", side_effect=attach), patch.object(authorized, "validate_training", side_effect=lambda run, target: {"ok": True, "iterations_completed": target}), patch.object(authorized, "validate_evaluation", return_value={}), patch.object(authorized.subprocess, "Popen", side_effect=popen):
            if fail_stage:
                with self.assertRaisesRegex(RuntimeError, fail_stage):
                    authorized.main()
            else:
                authorized.main()
        return events, authorized.read_json(plan / "execution_state.json"), plan

    def test_authorized_plan_end_to_end_stage_order_with_mock_children(self):
        events, state, plan = self.run_mock_authorized_plan()
        self.assertEqual(events, ["attach_training_to_50", "evaluation_50", "training_to_100", "evaluation_100", "feasibility"])
        self.assertEqual(state["completed_stages"], ["training_to_50", "evaluation_50", "training_to_100", "evaluation_100", "feasibility"])
        self.assertEqual(state["stage"], "complete")
        self.assertTrue(state["all_five_stages_complete"])
        self.assertFalse((plan / "controller.lock").exists())

    def test_authorized_plan_launches_no_dependent_child_after_failure(self):
        events, state, plan = self.run_mock_authorized_plan(fail_stage="training_to_100")
        self.assertEqual(events, ["attach_training_to_50", "evaluation_50", "training_to_100"])
        self.assertEqual(state["completed_stages"], ["training_to_50", "evaluation_50"])
        self.assertEqual(state["stage"], "failed")
        self.assertFalse(state["all_five_stages_complete"])
        self.assertFalse((plan / "controller.lock").exists())

    def test_evaluation_validator_rejects_stale_or_incomplete_report(self):
        path = self.root / "evaluation.json"
        run = self.root / "run"
        report = {"status": "complete", "selection": {"milestone": 50, "candidates": [{"iteration": 50}]},
                  "plan": {"run_dir": str(run)}, "cases": [{"checkpoint_iteration": 50, "start": start} for start in campaign_module.STARTS]}
        self.write(path, report)
        self.assertEqual(authorized.validate_evaluation(path, run, 50), report)
        for change in ("status", "milestone", "run", "missing_start"):
            with self.subTest(change=change):
                wrong = copy.deepcopy(report)
                if change == "status":
                    wrong["status"] = "pending_rollouts"
                elif change == "milestone":
                    wrong["selection"]["milestone"] = 100
                elif change == "run":
                    wrong["plan"]["run_dir"] = str(self.root / "other")
                else:
                    wrong["cases"].pop()
                self.write(path, wrong)
                with self.assertRaises(ValueError):
                    authorized.validate_evaluation(path, run, 50)


if __name__ == "__main__":
    unittest.main()
