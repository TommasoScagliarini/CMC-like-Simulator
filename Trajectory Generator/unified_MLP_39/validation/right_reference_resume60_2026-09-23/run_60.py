"""Prepare/check or explicitly run the authorized RIGHT39 continuation40->60.

Preparation never starts training. Execution appends the existing run, restores
the immutable full checkpoint40 and stops at60; no evaluation or later stage.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import pickle
import shutil
import signal
import subprocess
import sys
import time

import yaml

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
REPO = PACKAGE.parents[1]
RUN = PACKAGE / "runs/training/MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23"
PLAN = PACKAGE / "runs/plans/right_reference_resume60_2026-09-23"
OLD_PLAN = PACKAGE / "runs/plans/right_reference_training_2026-09-23"
SOURCE = RUN / "milestone_iteration_000040/checkpoint_last"
CONFIG = PLAN / "config_40_to_60.yaml"
THREADS = {k: "1" for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS")}
CHILD = None


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write(path, data):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rel(path):
    return str(path.resolve().relative_to(REPO))


def tree_hashes(folder):
    return {rel(p): digest(p) for p in sorted(folder.rglob("*")) if p.is_file()}


def history(partial=False):
    result = []
    for line in (RUN / "train_iterations.jsonl").read_text().splitlines():
        if not line.strip():
            continue
        try:
            result.append(json.loads(line))
        except json.JSONDecodeError:
            if partial:
                break
            raise
    return result


def event(state, **details):
    data = {"state": state, "updated_at": datetime.now().astimezone().isoformat(),
            "monitor_pid": os.getpid(), "run": str(RUN), "start_iteration": 40, "target_iterations": 60, **details}
    write(PLAN / "status.json", data)


def command():
    return [sys.executable, "-u", str(PACKAGE / "train_ppo_mlp.py"), "--config", str(CONFIG),
            "--output-dir", str(RUN), "--iterations", "60", "--resume-from", str(SOURCE)]


def validate_config():
    original = yaml.safe_load((SOURCE / "training_cfg.resolved.yaml").read_text())
    proposed = yaml.safe_load(CONFIG.read_text())
    expected = deepcopy(original)
    expected["simulation"]["iterations"] = 60
    require(proposed == expected, "Configuration differs beyond iterations40->60")
    require(proposed["parallelism"]["num_env_runners"] == 13 and proposed["parallelism"]["ray_num_cpus"] == 14, "Worker/CPU drift")
    require(proposed["target_availability"]["target_reference"] == "right_v26", "Reference changed")
    require(proposed["target_availability"]["target_availability_probability"] == 1 and not proposed["target_availability"]["target_curriculum_probabilities"], "Masking requested unexpectedly")
    require(proposed["grf"]["binary_phase_actor_fsm_version"] == "v3", "FSM changed")


def history_guard():
    prefix = (PLAN / "pre_resume_40/train_iterations.jsonl").read_bytes()
    require((RUN / "train_iterations.jsonl").read_bytes().startswith(prefix), "Original history1..40 changed")


def source_guard():
    receipt = read(PLAN / "freeze_receipt.json")
    require(digest(PLAN / "source_hashes.json") == receipt["manifest_sha256"], "Input manifest changed")
    require(digest(PLAN / "checkpoint40_hashes.json") == receipt["checkpoint_manifest_sha256"], "Checkpoint manifest changed")
    for manifest in ("source_hashes.json", "checkpoint40_hashes.json", "archive_hashes.json"):
        for filename, expected in read(PLAN / manifest).items():
            path = REPO / filename
            require(path.is_file() and digest(path) == expected, f"Frozen input changed: {filename}")
    validate_config()
    history_guard()


def prepare():
    require(not PLAN.exists(), "Plan already exists; preparation refuses overwrite")
    summary = read(RUN / "summary.json")
    rows = history()
    require(summary["ok"] and summary["iterations_completed"] == 40 and summary["stop_reason"] == "completed", "Expected completed RIGHT40")
    require([r["iteration"] for r in rows] == list(range(1, 41)), "Expected exact history1..40")
    require(read(SOURCE.parent / "checkpoint_last_meta.json")["logical_iteration"] == 40, "Wrong checkpoint iteration")
    PLAN.mkdir(parents=True)
    archive = PLAN / "pre_resume_40"
    archive.mkdir()
    for path in RUN.iterdir():
        if path.is_file():
            shutil.copy2(path, archive / path.name)
    for folder in ("rl_module_initial", "milestone_iteration_000040", "tensorboard"):
        shutil.copytree(RUN / folder, archive / folder)
    old_logs = archive / "original_launch_plan"
    old_logs.mkdir()
    for path in OLD_PLAN.iterdir():
        if path.is_file():
            shutil.copy2(path, old_logs / path.name)
    config = yaml.safe_load((SOURCE / "training_cfg.resolved.yaml").read_text())
    config["simulation"]["iterations"] = 60
    CONFIG.write_text(yaml.safe_dump(config, sort_keys=False, allow_unicode=True), encoding="utf-8")
    validate_config()
    protocol = {"schema": "right39_resume40_to60_v1", "created_at": datetime.now().astimezone().isoformat(),
        "run": rel(RUN), "config": rel(CONFIG), "resume_from": rel(SOURCE), "start": 40, "target": 60,
        "command": command(), "environment": THREADS, "training_started": False,
        "next_logical_iteration": 41, "new_iterations": 20, "new_transitions_nominal": 81920,
        "total_transitions_nominal": 245760, "same_output_directory": True,
        "directory_name_retains_40iter": True, "fresh_policy": False,
        "configuration_change": "Only simulation.iterations40->60; resume-from is explicit CLI routing.",
        "automatic_followup": None, "scope": "One training continuation only, stops at60; no automatic rollout or further training.",
        "resume_limit": "Actor/critic/Adam/PPO state restored; environment episodes and process RNG do not continue bitwise.",
        "screening_dependency": "Root operator starts explicitly after authorized screening; no screening file is frozen or polled."}
    write(PLAN / "protocol.json", protocol)
    inputs = {REPO / name for name in read(OLD_PLAN / "source_hashes.json")}
    for folder in (REPO, PACKAGE.parent, PACKAGE, PACKAGE.parent / "baseline_MLP"):
        inputs.update(folder.glob("*.py"))
    for folder in (REPO / "models/AB06_SEASEA_Threadmill", REPO / "online_grf_profiles"):
        inputs.update(p for p in folder.rglob("*") if p.is_file() and p.suffix.lower() in (".xml", ".osim", ".mot", ".sto", ".json"))
    inputs.add(REPO / config["grf"]["binary_phase_detector_profile"])
    inputs.update({HERE / "run_60.py", HERE / "preflight.py", CONFIG, PLAN / "protocol.json"})
    inputs.update(p for p in (RUN / "rl_module_initial").rglob("*") if p.is_file())
    # Do not include the concurrent diagnostic/screening scripts, outputs, or status.
    snapshot = PLAN / "source_snapshot"
    for path in sorted(inputs):
        require(path.is_file(), f"Missing input: {path}")
        destination = snapshot / path.relative_to(REPO)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, destination)
    write(PLAN / "source_hashes.json", {rel(p): digest(p) for p in sorted(inputs)})
    write(PLAN / "checkpoint40_hashes.json", tree_hashes(SOURCE.parent))
    write(PLAN / "archive_hashes.json", tree_hashes(archive))
    write(PLAN / "freeze_receipt.json", {"passed": True,
        "manifest_sha256": digest(PLAN / "source_hashes.json"),
        "checkpoint_manifest_sha256": digest(PLAN / "checkpoint40_hashes.json"),
        "source_count": len(inputs), "checkpoint_file_count": len(tree_hashes(SOURCE.parent)),
        "frozen_at": datetime.now().astimezone().isoformat()})
    event("prepared", training_started=False, completed_iterations=40, command=command())
    source_guard()
    print("Prepared and frozen; no training or simulation started.")


def verify_resume(first, source_last):
    weights = read(RUN / "resume_audit.json")
    require(Path(weights["source_checkpoint"]).resolve() == SOURCE.resolve(), "Wrong resumed checkpoint")
    require(weights["actor_reinitialized"] is False, "Actor reinitialized")
    for component in ("actor", "mean", "critic"):
        expected = source_last["unified_phase_audit"][component + "_after"]
        require(weights["restored_" + component + "_digest"] == expected == first["unified_phase_audit"][component + "_before"], f"{component} changed before update41")
    optimizer = read(RUN / "resume_optimizer_audit.json")
    require(optimizer["passed"] and Path(optimizer["source_checkpoint"]).resolve() == SOURCE.resolve(), "Optimizer resume failed")
    require(optimizer["source_state_sha256"] == digest(SOURCE / "learner_group/learner/state.pkl"), "Optimizer input changed")
    require(bool(optimizer["learners"]), "Missing learner audit")
    for learner in optimizer["learners"]:
        require(learner["passed"] and learner["parameters_unchanged"], "Optimizer changed weights")
        for item in learner["optimizers"]:
            require(item["state_exact"] and item["source_digest"] == item["restored_digest"], "Adam state changed")
            require(item["state_count"] == item["linked_state_count"] == 12, "Adam moments not linked to12 live parameters")
    ppo = optimizer["ppo_training_state"]
    require(ppo["passed"] and ppo["source"] == "checkpoint_sidecar", "PPO adaptive state not restored from source sidecar")
    for learner in ppo["learners"]:
        require(learner["passed"] and learner["constant_schedules_verified"], "PPO schedules changed")
        for coefficient in learner["kl_coefficient"].values():
            require(coefficient["passed"] and coefficient["source"] == coefficient["restored"] == source_last["current_kl_coeff"], "KL coefficient changed")
    write(PLAN / "resume_verified.json", {"passed": True, "weights": weights, "optimizer": optimizer})


def check_row(row, previous):
    i = row["iteration"]
    require(41 <= i <= 60 and i == previous["iteration"] + 1, "Iteration outside authorized41..60")
    require(row["num_env_steps_sampled_lifetime"] == i * 4096, "Wrong cumulative transitions")
    audit = row["unified_phase_audit"]
    require(audit["target_availability_batch"]["total_steps"] == audit["target_availability_batch"]["visible_steps"] == 4096, "Mask or batch changed")
    require(row["unified_phase"] == "imitation" and audit["mean_changed"] and audit["critic_changed"], "No actor/critic update")
    require(row["training_health"]["missing_telemetry_rows"] == 0, "Missing telemetry")
    for key in ("policy_loss", "vf_loss", "mean_kl_loss", "entropy"):
        require(math.isfinite(row[key]), f"Nonfinite {key}")


def optimizer_progress():
    previous = None
    records = []
    for iteration in (40, 41, 60):
        path = RUN / f"milestone_iteration_{iteration:06d}/checkpoint_last/learner_group/learner/state.pkl"
        state = pickle.loads(path.read_bytes())["optimizer"]
        steps = {}
        for name, item in state.items():
            payload = item["state"]
            ids = [i for group in payload["param_groups"] for i in group["params"]]
            require(len(ids) == len(set(ids)) == len(payload["state"]) == 12 and set(ids) == set(payload["state"]), "Optimizer topology changed")
            steps[name] = sorted({float(value["step"]) for value in payload["state"].values()})
            if previous is not None:
                require(min(steps[name]) > max(previous[name]), "Adam steps reset/stalled")
        records.append({"iteration": iteration, "steps": steps, "state_sha256": digest(path)})
        previous = steps
    write(PLAN / "optimizer_progress.json", {"passed": True, "checkpoints": records})


def stop_child():
    if CHILD is None or CHILD.poll() is not None:
        return
    CHILD.send_signal(signal.CTRL_BREAK_EVENT if os.name == "nt" else signal.SIGINT)
    try:
        CHILD.wait(timeout=30)
    except subprocess.TimeoutExpired:
        if os.name == "nt":
            subprocess.run(["taskkill", "/PID", str(CHILD.pid), "/T", "/F"], check=False)
        else:
            os.killpg(CHILD.pid, signal.SIGTERM)
        CHILD.wait(timeout=15)


def execute():
    global CHILD
    source_guard()
    require(len(history()) == 40, "Continuation already started or run is not at40")
    require(read(RUN / "summary.json")["iterations_completed"] == 40, "Unexpected run summary")
    with (PLAN / "execution.lock").open("x") as stream:
        stream.write(str(os.getpid()) + "\n")
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    checked = 40
    source_last = history()[-1]
    try:
        with (PLAN / "training_40_to_60.log").open("x") as log:
            CHILD = subprocess.Popen(command(), cwd=REPO, env={**os.environ, **THREADS, "PYTHONUNBUFFERED": "1"},
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=os.name != "nt", creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0)
            if sys.platform == "darwin" and Path("/usr/bin/caffeinate").is_file():
                subprocess.Popen(["/usr/bin/caffeinate", "-i", "-w", str(CHILD.pid)],
                    stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            write(PLAN / "launch.json", {"started_at": datetime.now().astimezone().isoformat(),
                "monitor_pid": os.getpid(), "supervisor_pid": CHILD.pid, "command": command(), "resume_from": str(SOURCE)})
            while True:
                history_guard()
                rows = history(partial=True)
                require(len(rows) <= 60, "Training exceeded authorized iteration60")
                for row in rows[checked:]:
                    check_row(row, rows[row["iteration"] - 2])
                    if row["iteration"] == 41:
                        verify_resume(row, source_last)
                    checked = row["iteration"]
                event("running", supervisor_pid=CHILD.pid, completed_iterations=checked,
                      new_iterations_completed=checked - 40, next_iteration=min(checked + 1, 60),
                      last_return=rows[-1]["episode_return_mean"])
                if CHILD.poll() is not None:
                    break
                time.sleep(5)
        # The final append can land between the last history read and poll().
        # Audit that tail once the child has exited instead of reporting a
        # false incomplete result for a correctly finished iteration60.
        completed_rows = history()
        for row in completed_rows[checked:]:
            check_row(row, completed_rows[row["iteration"] - 2])
            if row["iteration"] == 41:
                verify_resume(row, source_last)
            checked = row["iteration"]
        source_guard()
        summary = read(RUN / "summary.json")
        require(CHILD.returncode == 0 and summary["ok"] and summary["stop_reason"] == "completed", "Training did not finish cleanly")
        require(checked == summary["iterations_completed"] == 60 and [r["iteration"] for r in history()] == list(range(1, 61)), "Not all60 iterations completed")
        require(not summary.get("skipped_iterations") and not summary.get("crash_restart_count", 0), "Training skipped/restarted; inspect state")
        require(read(PLAN / "resume_verified.json")["passed"], "Resume not verified")
        optimizer_progress()
        write(PLAN / "checkpoint60_hashes.json", tree_hashes(RUN / "milestone_iteration_000060"))
        event("completed", supervisor_pid=CHILD.pid, completed_iterations=60, new_iterations_completed=20,
              returncode=CHILD.returncode, stop_reason="completed", automatic_followup=None)
    except BaseException as exc:
        stop_child()
        event("interrupted" if isinstance(exc, KeyboardInterrupt) else "failed", completed_iterations=checked, error=repr(exc))
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--prepare", action="store_true")
    mode.add_argument("--check-only", action="store_true")
    mode.add_argument("--run", action="store_true")
    args = parser.parse_args()
    if args.prepare:
        prepare()
    elif args.check_only:
        source_guard()
        require(len(history()) == 40, "Run is no longer at40")
        print("PASS: frozen inputs/checkpoint40, configuration changes only iteration budget, preserved history. No process started.")
    else:
        execute()


if __name__ == "__main__":
    main()
