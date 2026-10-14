"""Evaluate the completed best98 -> 200 branch against archived CP98/CP100.

Reuses the established milestone metrics. Only --execute-rollouts launches the
six missing deterministic simulations; this script cannot start training.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import itertools
import json
from pathlib import Path
import shutil
import subprocess
import sys
from datetime import datetime

import evaluate_training_milestone as metrics

U = Path(__file__).resolve().parents[1]
ROOT = U.parents[1]
RUN = U / "runs/training/pretraining_v1_pen1_best98_to200_2026-09-15"
OUTPUT = U / "runs/validation/pretraining_v1_continuation_200_2026-09-16"
ROLLOUT = U / "runs/rollout/pretraining_v1_continuation_200_2026-09-16"
OLD_REPORT = U / "runs/validation/pretraining_v1_pen1_milestone_100_2026-09-14/milestone_evaluation.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-rollouts", action="store_true")
    args = parser.parse_args()
    summary = metrics.read_json(RUN / "summary.json")
    assert summary["ok"] and summary["stop_reason"] == "completed"
    assert summary["iterations_completed"] == 200 and summary["iterations_completed_this_process"] == 102
    assert not summary["skipped_iterations"]
    selection = metrics.select_candidates(RUN, 200)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    config = OUTPUT / "training_cfg.resolved.yaml"
    if config.exists():
        assert metrics.sha256(config) == metrics.sha256(RUN / config.name)
    else:
        shutil.copy2(RUN / config.name, config)
    plan = {"run_dir": str(RUN), "selection": selection,
            "evaluation_config": str(config), "evaluation_config_sha256": metrics.sha256(config),
            "seed": 123, "max_steps": 500, "planned_duration_s": 5.0,
            "starts": metrics.START_OFFSETS, "reference_report": str(OLD_REPORT),
            "reference_report_sha256": metrics.sha256(OLD_REPORT), "commands": []}
    for candidate in selection["candidates"]:
        for start in metrics.START_OFFSETS:
            directory = ROLLOUT / f"iteration_{candidate['iteration']:06d}_{start}"
            identity = {"iteration": candidate["iteration"], "start": start,
                        "module_state_sha256": candidate["module_state_sha256"],
                        "config_sha256": plan["evaluation_config_sha256"],
                        "seed": 123, "max_steps": 500, "planned_duration_s": 5.0}
            command = metrics.rollout_command(sys.executable, candidate, start, directory, config)
            plan["commands"].append({"candidate": candidate, "start": start,
                                     "directory": str(directory), "identity": identity, "argv": command})
    metrics.write_json(OUTPUT / "evaluation_plan.json", plan)

    def collect(job):
        directory = Path(job["directory"])
        identity = directory / "evaluation_identity.json"
        if identity.exists():
            assert metrics.read_json(identity) == job["identity"], str(directory)
        elif (directory / "rollout_summary.json").exists():
            raise ValueError(f"Existing rollout lacks identity: {directory}")
        if not (directory / "rollout_summary.json").exists():
            if not args.execute_rollouts:
                return {"status": "pending", "iteration": job["candidate"]["iteration"], "start": job["start"]}
            directory.mkdir(parents=True, exist_ok=True)
            metrics.write_json(identity, job["identity"])
            with (directory / "evaluation_console.log").open("a", encoding="utf-8") as log:
                result = subprocess.run(job["argv"], cwd=ROOT, stdout=log,
                                        stderr=subprocess.STDOUT, timeout=3600, check=False)
            if result.returncode:
                raise RuntimeError(f"Rollout failed ({result.returncode}): {directory}")
        rollout = metrics.analysis.load_rollout(directory)
        metrics.validate_rollout(rollout, job["candidate"], job["start"])
        return {"status": "complete", "iteration": job["candidate"]["iteration"], "start": job["start"]}

    progress = {"started_at": datetime.now().astimezone().isoformat(), "jobs": [], "errors": []}
    metrics.write_json(OUTPUT / "rollout_progress.json", progress)
    with ThreadPoolExecutor(max_workers=3) as pool:
        futures = {pool.submit(collect, job): job for job in plan["commands"]}
        for future in as_completed(futures):
            try:
                result = future.result()
                progress["jobs"].append(result)
                print(json.dumps(result), flush=True)
            except Exception as exc:
                job = futures[future]
                progress["errors"].append({"directory": job["directory"], "error": str(exc)})
                print(str(exc), flush=True)
            progress["updated_at"] = datetime.now().astimezone().isoformat()
            metrics.write_json(OUTPUT / "rollout_progress.json", progress)
    if progress["errors"]:
        raise RuntimeError("Evaluation incomplete; inspect rollout_progress.json")
    if any(job["status"] != "complete" for job in progress["jobs"]):
        print("Plan prepared; deterministic rollouts pending.")
        return

    evaluated = {}
    roles = {c["iteration"]: c["roles"] for c in selection["candidates"]}
    for job in plan["commands"]:
        evaluated[(job["candidate"]["iteration"], job["start"])] = metrics.analysis.load_rollout(Path(job["directory"]))
    old = metrics.read_json(OLD_REPORT)
    for case in old["cases"]:
        iteration, start = case["checkpoint_iteration"], case["start"]
        if iteration not in (98, 100):
            continue
        directory = Path(case["source_directory"])
        for name, expected in case["source_hashes"].items():
            assert metrics.sha256(directory / name) == expected, str(directory / name)
        candidate = next(c for c in old["selection"]["candidates"] if c["iteration"] == iteration)
        rollout = metrics.analysis.load_rollout(directory)
        metrics.validate_rollout(rollout, candidate, start)
        evaluated[(iteration, start)] = rollout
        roles[iteration] = ["resume_parent" if iteration == 98 else "original_branch_last100"]
    assert len(evaluated) == 12
    report = {"schema_version": 1, "status": "complete", "selection": selection, "plan": plan,
              "cases": [], "common_prefix_comparisons": [], "quality_gates": {},
              "historical_reference_exclusions": []}
    flat_rows = []
    for (iteration, start), rollout in sorted(evaluated.items()):
        case = metrics.describe_case(rollout, iteration, start, roles[iteration])
        report["cases"].append(case)
        for window in ("full_episode", "startup", "online_period_regime"):
            if case[window] is not None:
                flat_rows.append({"iteration": iteration, "start": start, "window": window,
                                  **metrics.analysis.flatten(case[window])})
    for start in metrics.START_OFFSETS:
        indices = sorted(i for i, label in evaluated if label == start)
        for first, second in itertools.combinations(indices, 2):
            a, b = evaluated[(first, start)], evaluated[(second, start)]
            metrics.validate_pair(a, b)
            ra, rb = metrics.analysis.common_rows(a, b)
            pair = {"start": start, "iterations": [first, second], "metrics": {
                str(first): metrics.window_metrics(a, ra), str(second): metrics.window_metrics(b, rb)}}
            report["common_prefix_comparisons"].append(pair)
            for iteration, values in pair["metrics"].items():
                flat_rows.append({"iteration": iteration, "start": start,
                                  "window": f"common_CP{first}_CP{second}", **metrics.analysis.flatten(values)})
    for iteration in roles:
        report["quality_gates"][str(iteration)] = metrics.quality_gate(
            [c for c in report["cases"] if c["checkpoint_iteration"] == iteration])
    report["figures"] = metrics.plot_tracking(OUTPUT, evaluated, 200)
    metrics.write_json(OUTPUT / "milestone_evaluation.json", report)
    metrics.analysis.write_csv(OUTPUT / "metrics.csv", flat_rows)
    (OUTPUT / "milestone_evaluation.md").write_text(metrics.markdown_report(report), encoding="utf-8")
    print(json.dumps({"status": "complete", "cases": len(report["cases"]),
                      "quality_gates": report["quality_gates"]}, indent=2), flush=True)


if __name__ == "__main__":
    main()
