"""The requested report is a prerequisite of returning a completed evaluation."""
import argparse
import json
from pathlib import Path

import pytest

from unified_MLP.validation import evaluate_training_milestone as evaluation


def test_pending_evaluation_does_not_start_a_report(monkeypatch, tmp_path):
    monkeypatch.setattr(argparse.ArgumentParser, "parse_args", lambda _: argparse.Namespace(run_dir=tmp_path, milestone=50))
    monkeypatch.setattr(evaluation, "evaluate", lambda _: {"ok": False, "status": "pending_rollouts"})
    def unexpected(*args):
        raise AssertionError("A report was requested before the evaluation completed")
    monkeypatch.setattr(evaluation, "create_requested_report", unexpected)
    evaluation.main()


def test_report_finishes_before_evaluator_returns(monkeypatch, tmp_path, capsys):
    events = []
    args = argparse.Namespace(run_dir=tmp_path / "run", milestone=50)
    monkeypatch.setattr(argparse.ArgumentParser, "parse_args", lambda _: args)
    def evaluate(_):
        events.append("evaluation_complete")
        return {"ok": True, "status": "complete", "output_dir": str(tmp_path / "evaluation")}
    def report(run, directory, milestone):
        assert run == args.run_dir and directory == tmp_path / "evaluation" and milestone == 50
        events.append("report_written")
        return {"report_path": "reports/user/2026-09-15_intermediate.md"}
    monkeypatch.setattr(evaluation, "evaluate", evaluate)
    monkeypatch.setattr(evaluation, "create_requested_report", report)
    evaluation.main()
    events.append("evaluator_returned")
    assert events == ["evaluation_complete", "report_written", "evaluator_returned"]
    assert json.loads(capsys.readouterr().out)["create_report"]["report_path"].endswith("intermediate.md")


def test_failed_report_fails_the_evaluation_child(monkeypatch, tmp_path):
    monkeypatch.setattr(argparse.ArgumentParser, "parse_args", lambda _: argparse.Namespace(run_dir=tmp_path, milestone=50))
    monkeypatch.setattr(evaluation, "evaluate", lambda _: {"ok": True, "status": "complete", "output_dir": str(tmp_path)})
    def fail(*args):
        raise RuntimeError("Report write failed")
    monkeypatch.setattr(evaluation, "create_requested_report", fail)
    with pytest.raises(RuntimeError, match="Report write failed"):
        evaluation.main()


def test_request_for_50_does_not_create_report_at_100(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / evaluation.REPORT_REQUEST_FILENAME).write_text(json.dumps({"schema_version": 1, "milestones": [50]}))
    assert evaluation.create_requested_report(run, tmp_path, 100) is None


def test_report_request_is_bound_to_its_run(tmp_path, monkeypatch):
    unified = tmp_path / "unified_MLP"
    run = unified / "runs/training/selected"
    run.mkdir(parents=True)
    (run / evaluation.REPORT_REQUEST_FILENAME).write_text(json.dumps({
        "schema_version": 1, "milestones": [50], "run_dir": str(run.parent / "other"),
    }))
    monkeypatch.setattr(evaluation, "UNIFIED_DIR", unified)
    with pytest.raises(ValueError, match="different training run"):
        evaluation.create_requested_report(run, tmp_path, 50)


def test_report_request_routes_to_generator_and_records_receipt(tmp_path, monkeypatch):
    from unified_MLP.validation import create_milestone_report

    unified = tmp_path / "unified_MLP"
    run = unified / "runs/training/selected"
    plan = unified / "runs/plans/authorized"
    destination = unified / "runs/validation/intermediate"
    run.mkdir(parents=True)
    plan.mkdir(parents=True)
    request_path = run / evaluation.REPORT_REQUEST_FILENAME
    request_path.write_text(json.dumps({
        "schema_version": 1, "milestones": [50], "run_dir": str(run),
        "plan_dir": str(plan), "completed_reports": {},
    }))
    receipt = {"status": "complete", "report_path": "reports/user/2026-09-15_intermediate.md"}
    def generate(actual_destination, actual_plan):
        assert actual_destination == destination and actual_plan == plan
        return receipt
    monkeypatch.setattr(evaluation, "UNIFIED_DIR", unified)
    monkeypatch.setattr(create_milestone_report, "create_report", generate)
    assert evaluation.create_requested_report(run, destination, 50) == receipt
    assert json.loads(request_path.read_text())["completed_reports"] == {"50": receipt}
