"""Launch the authorized fresh RIGHT run only after the smoke gates pass.

The trainer supervises and checkpoints its workers; this small persistent
monitor records progress and completion without starting additional training.
"""
from __future__ import annotations

from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
REPO = PACKAGE.parents[1]
PLAN = PACKAGE / "runs/plans/right_reference_training_2026-09-23"
CONFIG = PACKAGE / "configs/v26_aligned_right_phase_fsmv3_40iter_2026-09-23.yaml"
RUN = PACKAGE / "runs/training/MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23"
CHILD = None


def read(path):
    return json.loads(path.read_text())


def write(path, payload):
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


def guard():
    for path, digest in read(PLAN / "source_hashes.json").items():
        if hashlib.sha256((REPO / path).read_bytes()).hexdigest() != digest:
            raise RuntimeError(f"Registered source/input changed: {path}")
    for name in ("preflight.json", "unit_tests_receipt.json", "smoke2_audit.json",
                 "smoke3_resume_audit.json", "smoke_rollout_audit.json"):
        if not read(HERE / name)["pass"]:
            raise RuntimeError(f"Smoke gate failed: {name}")


def status(state, **details):
    write(PLAN / "status.json", {"state": state, "updated_at": datetime.now().astimezone().isoformat(),
          "monitor_pid": os.getpid(), "run": str(RUN), "target_iterations": 40, **details})


def stop(signum, frame):
    if CHILD is not None and CHILD.poll() is None:
        CHILD.send_signal(signal.CTRL_BREAK_EVENT if os.name == "nt" else signal.SIGINT)


def rows():
    path = RUN / "train_iterations.jsonl"
    if not path.exists():
        return []
    result = []
    for line in path.read_text().splitlines():
        try:
            result.append(json.loads(line))
        except json.JSONDecodeError:
            break  # An in-progress append is read again on the next poll.
    return result


def main():
    global CHILD
    PLAN.mkdir(parents=True, exist_ok=True)
    with (PLAN / "execution.lock").open("x") as stream:
        stream.write(str(os.getpid()) + "\n")
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        guard()
        if RUN.exists():
            raise FileExistsError(f"Fresh training output already exists: {RUN}")
        command = [sys.executable, str(PACKAGE / "train_ppo_mlp.py"), "--config", str(CONFIG),
                   "--output-dir", str(RUN), "--iterations", "40"]
        environment = os.environ.copy()
        for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
            environment[key] = "1"
        with (PLAN / "training.log").open("x") as log:
            CHILD = subprocess.Popen(command, cwd=REPO, env=environment, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=os.name != "nt",
                creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0)
            if sys.platform == "darwin" and Path("/usr/bin/caffeinate").is_file():
                subprocess.Popen(["/usr/bin/caffeinate", "-i", "-w", str(CHILD.pid)],
                                 stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            write(PLAN / "launch.json", {"started_at": datetime.now().astimezone().isoformat(),
                "monitor_pid": os.getpid(), "supervisor_pid": CHILD.pid, "command": command,
                "fresh": True, "reference": "right_v26", "fsm": "v3"})
            while CHILD.poll() is None:
                recorded = rows()
                status("running", supervisor_pid=CHILD.pid, completed_iterations=len(recorded),
                       last_iteration=recorded[-1]["iteration"] if recorded else 0)
                time.sleep(5)
            returncode = CHILD.returncode
        guard()
        summary = read(RUN / "summary.json") if (RUN / "summary.json").exists() else {}
        recorded = rows()
        success = (returncode == 0 and summary.get("ok") and summary.get("stop_reason") == "completed"
                   and not summary.get("skipped_iterations")
                   and [r["iteration"] for r in recorded] == list(range(1, 41)))
        status("completed" if success else "failed", supervisor_pid=CHILD.pid,
               returncode=returncode, completed_iterations=len(recorded),
               stop_reason=summary.get("stop_reason"), skipped_iterations=summary.get("skipped_iterations"))
        if not success:
            raise RuntimeError("Training did not complete all forty iterations; see training.log")
    except BaseException as exc:
        status("failed", error=str(exc), completed_iterations=len(rows()))
        raise


if __name__ == "__main__":
    main()
