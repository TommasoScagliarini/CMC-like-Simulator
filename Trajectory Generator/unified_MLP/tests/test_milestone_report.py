"""Completed-evidence publishing and TODO preservation; no simulator or training."""
from __future__ import annotations

from datetime import datetime, timedelta, timezone
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


MODULE = Path(__file__).resolve().parents[1] / "validation" / "create_milestone_report.py"
spec = importlib.util.spec_from_file_location("milestone_report", MODULE)
reporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reporter)


def write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data), encoding="utf-8")


def metrics(iteration: int, start: str) -> dict:
    base = iteration + reporter.STARTS.index(start) / 10
    tracking = {}
    for joint, offset in (("knee", 0), ("ankle", 0.01)):
        tracking[joint] = {}
        for source, shift in (("actual", 0), ("served", 0.001)):
            tracking[joint][source + "_vs_prescribed"] = {
                "position": {"rmse_deg": base + offset + shift},
                "velocity": {"rmse_deg_s": base + 100 + offset + shift},
            }
    return {"duration_s": 2.8, "tracking": tracking,
            "phase": {"valid_hs_count": 1, "valid_to_count": 2, "valid_cycle_count": 0},
            "safety": {"penetration_max_mm": 25.125},
            "termination": {"end_reason": "phase_timeout_swing"}}


def fixture(root: Path) -> tuple[Path, Path, dict, dict]:
    evaluation = root / "Trajectory Generator" / "evaluation"
    plan = root / "Trajectory Generator" / "plan"
    run = root / "Trajectory Generator" / "training"
    evaluation.mkdir(parents=True)
    plan.mkdir(parents=True)
    run.mkdir(parents=True)
    candidates = [{"iteration": 20, "roles": ["best_training_return"]}, {"iteration": 50, "roles": ["last"]}]
    data = {"status": "complete", "selection": {"milestone": 50, "roles": {"last": 50, "best_training_return": 20}, "candidates": candidates},
            "plan": {"run_dir": str(run), "pending_rollouts": []}, "cases": [], "common_prefix_comparisons": [],
            "quality_gates": {str(i): {"pass": False, "failures": ["early:two_valid_cycles"]} for i in (5, 20, 50)},
            "figures": ["tracking_early.png", "tracking_center.pdf", "tracking_late.png"]}
    for start in reporter.STARTS:
        for iteration in (5, 20, 50):
            data["cases"].append({"start": start, "checkpoint_iteration": iteration,
                                  "roles": ["historical_CP5" if iteration == 5 else "last" if iteration == 50 else "best_training_return"],
                                  "full_episode": metrics(iteration, start), "first_online_period_available_elapsed_s": None})
        data["common_prefix_comparisons"].append({"start": start, "iterations": [5, 50],
                                                   "metrics": {str(i): metrics(i, start) for i in (5, 50)}})
    summary = {"ok": True, "stop_reason": "completed", "iterations_completed": 50, "output_dir": str(run),
               "skipped_iterations": [], "restart_count": 2, "crash_restart_count": 1,
               "elapsed_wall_time_s": 25000,
               "history": [{"iteration": i, "unified_phase": "imitation", "exact_start_balance": {"pass": True}} for i in range(1, 51)]}
    write_json(evaluation / "milestone_evaluation.json", data)
    write_json(plan / "training_summary_50.json", summary)
    write_json(plan / "execution_state.json", {"run_dir": str(run), "stage": "evaluation_50"})
    (plan / "STATUS.md").write_text("Status\n")
    for filename in data["figures"] + ["metrics.csv", "milestone_evaluation.md"]:
        (evaluation / filename).write_text("fixture artifact\n")
    return evaluation, plan, data, summary


class MilestoneReportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.evaluation, self.plan, self.data, self.summary = fixture(self.root)
        self.now = datetime(2026, 9, 15, 3, 45, tzinfo=timezone(timedelta(hours=2)))

    def publish(self, **kwargs):
        return reporter.create_report(self.evaluation, self.plan, repo_root=self.root, now=kwargs.pop("now", self.now), **kwargs)

    def assert_unpublished(self):
        self.assertFalse((self.plan / "create_report_50_receipt.json").exists())
        self.assertFalse((self.root / "reports/user").exists())
        self.assertFalse((self.root / "reports/daily").exists())

    def test_complete_report_uses_real_metrics_local_date_and_negative_gate(self):
        receipt = self.publish()
        content = (self.root / receipt["report_path"]).read_text()
        self.assertEqual(receipt["report_date"], "2026-09-15")
        self.assertIn("50.000 / 50.001", content)
        self.assertIn("150.000 / 150.001", content)
        self.assertIn("5.200 / 5.201", content)
        self.assertIn("CP20", content)
        self.assertIn("**non superato**", content)
        self.assertNotIn("screening **superato**", content)
        self.assertIn("Non dimostra parità", content)
        self.assertIn("non disponibile", content)
        self.assertIn("crash restart: 1", content)
        self.assertIn("tentativo precedente", content)
        self.assertIn("[metrics.csv](<../../Trajectory Generator/evaluation/metrics.csv>)", content)
        for section in ("Problema", "Soluzione", "Strategia", "File modificati", "Test e verifiche"):
            self.assertIn("## " + section, content)
        self.assertEqual(len(receipt["source_sha256"]), 7)

    def test_pending_wrong_milestone_or_missing_last_never_publishes(self):
        for mutate in (
            lambda d: d.update(status="pending_rollouts"),
            lambda d: d["selection"].update(milestone=100),
            lambda d: d["plan"].update(pending_rollouts=[{}]),
            lambda d: d["selection"]["roles"].update(last=49),
            lambda d: d["cases"].pop(),
            lambda d: d["common_prefix_comparisons"].pop(),
        ):
            data = json.loads(json.dumps(self.data))
            mutate(data)
            write_json(self.evaluation / "milestone_evaluation.json", data)
            with self.assertRaises(ValueError):
                self.publish()
            self.assert_unpublished()

    def test_training_failures_or_different_run_cannot_be_reported_as_success(self):
        for field, value in (("ok", False), ("iterations_completed", 49), ("stop_reason", "error"),
                             ("skipped_iterations", [27]), ("history", self.summary["history"][:-1]),
                             ("output_dir", str(self.root / "wrong"))):
            summary = dict(self.summary, **{field: value})
            write_json(self.plan / "training_summary_50.json", summary)
            with self.assertRaises(ValueError):
                self.publish()
            self.assert_unpublished()

    def test_missing_artifact_and_nonfinite_metrics_fail_before_any_writes(self):
        (self.evaluation / "metrics.csv").unlink()
        with self.assertRaisesRegex(ValueError, "Missing evaluation artifact"):
            self.publish()
        self.assert_unpublished()
        (self.evaluation / "metrics.csv").write_text("restored\n")
        self.data["cases"][0]["full_episode"]["tracking"]["knee"]["actual_vs_prescribed"]["position"]["rmse_deg"] = None
        write_json(self.evaluation / "milestone_evaluation.json", self.data)
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            self.publish()
        self.assert_unpublished()

    def test_rollover_replay_preserves_date_and_tolerates_later_daily_additions(self):
        receipt = self.publish()
        report = self.root / receipt["report_path"]
        daily = self.root / receipt["daily_path"]
        report_bytes = report.read_bytes()
        daily.write_text(daily.read_text() + "\n## Lavoro successivo\nDa preservare.\n")
        state = reporter.read_json(self.plan / "execution_state.json")
        write_json(self.plan / "execution_state.json", dict(state, stage="training_to_100"))
        second = self.publish(now=self.now + timedelta(days=1))
        self.assertEqual(receipt, second)
        self.assertEqual(report_bytes, report.read_bytes())
        self.assertIn("Lavoro successivo", daily.read_text())
        self.assertEqual(daily.read_text().count(receipt["daily_marker"]), 1)
        self.assertFalse((self.root / "reports/daily/2026-09-16_daily-report.md").exists())

    def test_changed_evidence_or_user_report_fails_replay_without_overwriting(self):
        receipt = self.publish()
        report = self.root / receipt["report_path"]
        original = report.read_bytes()
        (self.evaluation / "metrics.csv").write_text("changed metrics\n")
        with self.assertRaisesRegex(ValueError, "conflicts"):
            self.publish()
        self.assertEqual(original, report.read_bytes())

    def test_daily_preserves_prior_multiline_todos_and_existing_content(self):
        daily_dir = self.root / "reports/daily"
        daily_dir.mkdir(parents=True)
        previous = daily_dir / "2026-09-14_daily-report.md"
        previous_text = "# Storico\n\nProsa non da duplicare.\n\n- [ ] Teacher storico\n  con continuazione importante.\n- [ ] Chiuso\n  su due righe.\n- [ ] " + reporter.DONE_50[0] + "\n- [ ] " + reporter.DONE_50[2] + "\n"
        previous.write_text(previous_text)
        daily = daily_dir / "2026-09-15_daily-report.md"
        existing = "# Oggi\n\n## Altro lavoro\nConservare integralmente.\n\n- [x] Chiuso\n  su due righe.\n- [ ] Nuovo lavoro oggi.\n"
        daily.write_text(existing)
        receipt = self.publish()
        content = (self.root / receipt["report_path"]).read_text()
        self.assertIn("- [ ] Teacher storico\n  con continuazione importante.", content)
        self.assertIn("- [ ] Nuovo lavoro oggi.", content)
        self.assertNotIn("- [ ] Chiuso", content)
        self.assertNotIn("- [ ] " + reporter.DONE_50[0], content)
        self.assertNotIn("- [ ] " + reporter.DONE_50[2], content)
        self.assertNotIn("Prosa non da duplicare", content)
        self.assertTrue(daily.read_text().startswith(existing))
        self.assertEqual(previous.read_text(), previous_text)
        self.assertIn("- [x] Completare V1 fino a50", daily.read_text())
        for item in reporter.TODO:
            self.assertIn("- [ ] " + item, daily.read_text())

    def test_no_overwrite_of_existing_user_report_and_no_false_receipt(self):
        target = self.root / "reports/user/2026-09-15_pretraining_unificata_valutazione_intermedia_50.md"
        target.parent.mkdir(parents=True)
        target.write_text("Report scritto dall'utente\n")
        with self.assertRaisesRegex(ValueError, "Refusing to overwrite"):
            self.publish()
        self.assertEqual(target.read_text(), "Report scritto dall'utente\n")
        self.assertFalse((self.plan / "create_report_50_receipt.json").exists())
        self.assertFalse((self.root / "reports/daily").exists())

    def test_best_last_coincidence_deduplicates_but_keeps_cp5(self):
        self.data["selection"]["roles"]["best_training_return"] = 50
        self.data["selection"]["candidates"] = [{"iteration": 50, "roles": ["last", "best_training_return"]}]
        self.data["cases"] = [case for case in self.data["cases"] if case["checkpoint_iteration"] != 20]
        self.data["quality_gates"].pop("20")
        write_json(self.evaluation / "milestone_evaluation.json", self.data)
        receipt = self.publish()
        self.assertIn("6 casi", (self.root / receipt["report_path"]).read_text())

    def test_interrupted_publication_recovers_original_date_without_false_success(self):
        original_write = reporter.atomic_write

        def interrupt_daily(path, content):
            if path.name.endswith("_daily-report.md"):
                raise OSError("simulated interruption before daily write")
            return original_write(path, content)

        with patch.object(reporter, "atomic_write", side_effect=interrupt_daily):
            with self.assertRaisesRegex(OSError, "simulated interruption"):
                self.publish()
        self.assertFalse((self.plan / "create_report_50_receipt.json").exists())
        self.assertTrue((self.plan / "create_report_50_intent.json").exists())
        receipt = self.publish(now=self.now + timedelta(days=1))
        self.assertEqual(receipt["report_date"], "2026-09-15")
        self.assertTrue((self.root / receipt["daily_path"]).exists())
        self.assertFalse((self.plan / "create_report_50_intent.json").exists())


if __name__ == "__main__":
    unittest.main()
