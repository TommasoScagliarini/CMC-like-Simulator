"""Ten paired-noise evaluations before the authorized RIGHT40 -> RIGHT60 resume.

No training is launched here. Four single-thread simulations at most, native
RIGHT evaluator, immutable milestone exports, five shared Gaussian seeds.
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
PLAN = PACKAGE / "runs/plans/right_reference_screening_2026-09-23"
ROOT = PACKAGE / "runs/rollout/MLP39_right_screening_2026-09-23"
RUN = PACKAGE / "runs/training/MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23"
MAX_CONCURRENT = 4


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


def now():
    return datetime.now().astimezone().isoformat()


def guard():
    hashes = read(PLAN / "input_hashes.json")
    for path, digest in hashes.items():
        if hashlib.sha256((REPO / path).read_bytes()).hexdigest() != digest:
            raise RuntimeError(f"Screening input changed: {path}")


def main():
    PLAN.mkdir(parents=True, exist_ok=True)
    with (PLAN / "execution.lock").open("x") as lock:
        lock.write(str(os.getpid()) + "\n")
    guard()
    protocol = read(PLAN / "protocol.json")
    assert protocol["noise_seeds"] == list(range(123, 128))
    assert protocol["noise_scale"] == 1 and protocol["maximum_steps"] == 500
    cases = [{"name": f"{label}_seed{seed}", "label": label, "iteration": iteration,
              "seed": seed, "folder": str(ROOT / label / f"noise_1_seed{seed}")}
             for seed in range(123, 128) for label, iteration in (("best35", 35), ("last40", 40))]
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[name] = "1"
    active, results, pending = {}, [], cases.copy()
    sleeper = None
    if sys.platform == "darwin" and Path("/usr/bin/caffeinate").exists():
        sleeper = subprocess.Popen(["/usr/bin/caffeinate", "-i", "-w", str(os.getpid())],
                                   stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def status(state, **extra):
        progress = {}
        for name, (_, _, case) in active.items():
            path = Path(case["folder"]) / "watchdog_state.json"
            try:
                progress[name] = read(path) if path.exists() else {"phase": "startup"}
            except json.JSONDecodeError:
                progress[name] = {"phase": "heartbeat update"}
        write(PLAN / "status.json", {"state": state, "updated_at": now(), "pid": os.getpid(),
              "completed": len(results), "total": len(cases), "active": progress,
              "pending": [c["name"] for c in pending], "results": results, **extra})

    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    try:
        while pending or active:
            while pending and len(active) < MAX_CONCURRENT:
                case = pending.pop(0)
                name, folder = case["name"], Path(case["folder"])
                if folder.exists():
                    raise FileExistsError(folder)
                checkpoint = RUN / f"milestone_iteration_{case['iteration']:06d}" / "rl_module_last"
                command = [sys.executable, str(PACKAGE / "validation/completion_noise_rollout.py"),
                    "--checkpoint", str(checkpoint), "--output-dir", str(folder),
                    "--noise-scale", "1", "--noise-seed", str(case["seed"]),
                    "--no-progress", "--run-timeout-s", "2400"]
                stream = (PLAN / f"{name}.log").open("x")
                process = subprocess.Popen(command, cwd=REPO, env=env, stdin=subprocess.DEVNULL,
                    stdout=stream, stderr=subprocess.STDOUT,
                    creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0)
                active[name] = (process, stream, case)
                write(PLAN / f"{name}_launch.json", {"started_at": now(), "pid": process.pid, "command": command})
                print("STARTED", name, process.pid, flush=True)
            for name, (process, stream, case) in list(active.items()):
                code = process.poll()
                if code is None:
                    continue
                stream.close()
                del active[name]
                result = dict(case, returncode=code, finished_at=now())
                folder = Path(case["folder"])
                if code == 0 and (folder / "rollout_summary.json").exists() and (folder / "noise_diagnostic.json").exists():
                    data = read(folder / "rollout_summary.json")
                    result.update({k: data.get(k) for k in ("ok", "steps", "end_reason", "episode_return", "grf_penetration_max_m", "phase_valid_cycle_count", "invalid_event_count")})
                    result["completed_full_episode"] = data["steps"] == 500 and data["end_reason"] == "episode_time_limit"
                    result["technical_failure"] = not data["ok"]
                else:
                    result["technical_failure"] = True
                results.append(result)
                print(json.dumps(result), flush=True)
            status("running")
            if pending or active:
                time.sleep(2)
        guard()
        failed = any(r["technical_failure"] for r in results)
        write(PLAN / "results.json", {"protocol": protocol, "cases": results, "technical_pass": not failed})
        status("failed" if failed else "completed")
        if failed:
            raise RuntimeError("Technical failure in the screening; inspect per-case logs")
    except BaseException as exc:
        for process, _, _ in active.values():
            if process.poll() is None:
                process.send_signal(signal.CTRL_BREAK_EVENT if os.name == "nt" else signal.SIGINT)
        for process, stream, _ in active.values():
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.kill()
            stream.close()
        status("interrupted_or_failed", error=str(exc))
        raise
    finally:
        if sleeper is not None and sleeper.poll() is None:
            sleeper.terminate()


if __name__ == "__main__":
    main()
