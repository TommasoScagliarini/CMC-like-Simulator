"""Run the explicitly authorized 5 -> 50 -> evaluation -> 100 -> feasibility plan.

This controller never changes training settings. A preflight snapshot is the
configuration source, immutable iteration directories are the resume sources,
and a failed stage prevents every dependent stage. All children use argv lists.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

UNIFIED = Path(__file__).resolve().parents[1]
REPO = UNIFIED.parents[1]


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, data):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    tmp.replace(path)


def utcnow():
    return datetime.now(timezone.utc).isoformat()


def validate_training(run, target):
    run = Path(run)
    summary = read_json(run / "summary.json")
    history = [json.loads(line) for line in (run / "train_iterations.jsonl").read_text().splitlines() if line]
    if not summary.get("ok") or summary.get("stop_reason") != "completed":
        raise ValueError("Training did not complete successfully")
    if summary.get("iterations_completed") != target:
        raise ValueError("Training summary has a different iteration target")
    if [row["iteration"] for row in history] != list(range(1, target + 1)):
        raise ValueError("Missing, duplicate or skipped training iterations")
    if summary.get("skipped_iterations"):
        raise ValueError("Training skipped iterations")
    if any(row.get("unified_phase") != "imitation" for row in history):
        raise ValueError("Training left imitation mode")
    if any(not row.get("exact_start_balance", {}).get("pass") for row in history):
        raise ValueError("Training did not preserve balanced sampling")
    milestone = run / f"milestone_iteration_{target:06d}"
    meta = read_json(milestone / "checkpoint_last_meta.json")
    state = read_json(milestone / "checkpoint_last" / "unified_training_state.json")
    if meta.get("logical_iteration") != target or state.get("last_completed_iteration") != target:
        raise ValueError("Checkpoint and training history disagree")
    if state.get("reward_mode") != "imitation":
        raise ValueError("Checkpoint is not imitative")
    return summary


def source_is_preserved(plan):
    preflight = read_json(plan / "preflight.json")
    expected_files = {**preflight["checkpoint5_hashes"], **preflight.get("runtime_source_sha256", {})}
    for relative, expected in expected_files.items():
        if hashlib.sha256((REPO / relative).read_bytes()).hexdigest() != expected:
            raise ValueError(f"Frozen runtime source or initial checkpoint changed: {relative}")


def validate_evaluation(path, run, target):
    report = read_json(path)
    if report.get("status") != "complete" or report.get("selection", {}).get("milestone") != target:
        raise ValueError("Evaluation is incomplete or belongs to a different milestone")
    if Path(report.get("plan", {}).get("run_dir", "")).resolve() != Path(run).resolve():
        raise ValueError("Evaluation belongs to a different training run")
    if not report.get("selection", {}).get("candidates"):
        raise ValueError("Evaluation has no selected candidates")
    for candidate in report["selection"]["candidates"]:
        cases = [c for c in report.get("cases", []) if c["checkpoint_iteration"] == candidate["iteration"]]
        if len(cases) != 3 or {c["start"] for c in cases} != {"early", "center", "late"}:
            raise ValueError("Evaluation is missing a candidate/start combination")
    return report


def wait_for_existing_training(run, target, timeout_s, publish):
    """Attach without trusting stale supervisor/summary files from an earlier launch."""
    started = time.monotonic()
    dead_since = None
    while time.monotonic() - started < timeout_s:
        try:
            summary = read_json(run / "summary.json")
            if summary.get("ok") and summary.get("iterations_completed") == target:
                return validate_training(run, target)
        except (OSError, ValueError):
            pass
        watchdog = read_json(run / "watchdog_state.json")
        live = False
        try:
            os.kill(int(watchdog["pid"]), 0)
            live = True
        except (ProcessLookupError, KeyError, ValueError):
            pass
        except PermissionError:
            live = True
        # The supervisor can replace a failed child; allow time for that handoff.
        if live:
            dead_since = None
        elif dead_since is None:
            dead_since = time.monotonic()
        elif time.monotonic() - dead_since > 180:
            raise RuntimeError("Training process ended without completing the target")
        history_path = run / "train_iterations.jsonl"
        history = [json.loads(line) for line in history_path.read_text().splitlines() if line]
        publish("training_to_50", last_completed_iteration=history[-1]["iteration"],
                watchdog_phase=watchdog.get("phase"), training_child_pid=watchdog.get("pid"))
        time.sleep(30)
    raise TimeoutError("Timed out while awaiting the existing training process")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--plan-dir", type=Path, required=True)
    parser.add_argument("--attach-training-to50", action="store_true")
    parser.add_argument("--run-feasibility", action="store_true")
    args = parser.parse_args()
    run, plan = args.run_dir.resolve(), args.plan_dir.resolve()
    run.relative_to((UNIFIED / "runs" / "training").resolve())
    plan.relative_to((UNIFIED / "runs").resolve())
    plan.mkdir(parents=True, exist_ok=True)
    lock = plan / "controller.lock"
    with lock.open("x", encoding="utf-8") as stream:
        stream.write(str(os.getpid()))
    state = {"started_at": utcnow(), "controller_pid": os.getpid(),
             "python": sys.executable, "run_dir": str(run), "completed_stages": [],
             "all_five_stages_complete": False}

    def publish(stage, **extra):
        state.update(stage=stage, updated_at=utcnow(), **extra)
        write_json(plan / "execution_state.json", state)
        lines = ["# Esecuzione del piano 5 → 50 → 100 → fattibilità", "",
                 f"Aggiornamento: {state['updated_at']}", f"Fase corrente: **{stage}**", "",
                 f"Ultima iterazione osservata: {state.get('last_completed_iteration', 'vedere train_iterations.jsonl')}",
                 f"Fasi completate: {', '.join(state['completed_stages']) or 'nessuna nuova fase conclusa'}", "",
                 f"Intero piano completato: **{state['all_five_stages_complete']}**", "",
                 "[Stato dettagliato](execution_state.json) · [Log del resume a 50](training_to50_runtime.log)",
                 "", "La fase successiva parte soltanto dopo i controlli della precedente. I risultati di cammino possono essere negativi senza costituire un errore di esecuzione."]
        if state.get("error"):
            lines += ["", f"Errore: {state['error']}"]
        (plan / "STATUS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    def execute(stage, command, timeout=48 * 3600):
        publish(stage, command=command)
        log_path = plan / f"{stage}.log"
        with log_path.open("a", encoding="utf-8") as output:
            output.write(f"\n[{utcnow()}] {json.dumps(command)}\n")
            output.flush()
            with subprocess.Popen(command, cwd=REPO, stdout=output, stderr=subprocess.STDOUT) as child:
                publish(stage, child_pid=child.pid, log=str(log_path))
                deadline = time.monotonic() + timeout
                try:
                    while True:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise subprocess.TimeoutExpired(command, timeout)
                        try:
                            code = child.wait(timeout=min(30, remaining))
                            break
                        except subprocess.TimeoutExpired:
                            progress = {}
                            if stage.startswith("training_to_"):
                                rows = [json.loads(line) for line in (run / "train_iterations.jsonl").read_text().splitlines() if line]
                                progress["last_completed_iteration"] = rows[-1]["iteration"]
                            elif stage == "feasibility" and (plan / "feasibility/state.json").exists():
                                progress["feasibility_action"] = read_json(plan / "feasibility/state.json").get("current_action")
                            publish(stage, **progress)
                except BaseException:
                    child.terminate()
                    try:
                        child.wait(timeout=90)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
                    raise
            if code:
                raise RuntimeError(f"{stage} exited with {code}; see {log_path}")

    try:
        source_is_preserved(plan)
        config = plan / "pilot5_snapshot" / "training_cfg.resolved.yaml"
        for target in (50, 100):
            if target == 50 and args.attach_training_to50:
                wait_for_existing_training(run, target, 36 * 3600, publish)
            else:
                previous = 5 if target == 50 else 50
                execute(f"training_to_{target}", [sys.executable, "-u", str(UNIFIED / "train_ppo_mlp.py"),
                    "--config", str(config), "--resume-from", str(run / f"milestone_iteration_{previous:06d}" / "checkpoint_last"),
                    "--output-dir", str(run), "--iterations", str(target)])
            summary = validate_training(run, target)
            state["last_completed_iteration"] = target
            write_json(plan / f"training_summary_{target}.json", summary)
            source_is_preserved(plan)
            state["completed_stages"].append(f"training_to_{target}")
            evaluation = UNIFIED / "runs" / "validation" / f"pretraining_v1_pen1_milestone_{target}_2026-09-14"
            execute(f"evaluation_{target}", [sys.executable, str(UNIFIED / "validation" / "evaluate_training_milestone.py"),
                    "--run-dir", str(run), "--milestone", str(target), "--output-dir", str(evaluation),
                    "--execute-rollouts"])
            validate_evaluation(evaluation / "milestone_evaluation.json", run, target)
            state["completed_stages"].append(f"evaluation_{target}")
            publish(f"evaluation_{target}_completed", evaluation=str(evaluation))
        if args.run_feasibility:
            # Materialization and every experiment are deferred until both V1 evaluations exist.
            source_is_preserved(plan)
            execute("feasibility", [sys.executable, str(UNIFIED / "validation" / "run_feasibility39.py"),
                    "--plan-dir", str(plan)], timeout=7 * 24 * 3600)
            feasibility = read_json(plan / "feasibility_result.json")
            if not feasibility.get("protocol_completed"):
                raise RuntimeError("Feasibility protocol is incomplete")
            state["completed_stages"].append("feasibility")
            publish("complete", all_five_stages_complete=True)
        else:
            publish("awaiting_feasibility", all_five_stages_complete=False)
    except BaseException as exc:
        publish("failed", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        lock.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
