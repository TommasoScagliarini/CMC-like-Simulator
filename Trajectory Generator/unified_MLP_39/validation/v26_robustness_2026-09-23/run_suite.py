"""Run the ten authorized paired-noise cases after the arithmetic control.

Four independent single-thread simulations at most, as in the best72 protocol.
Each case has its own heartbeat watchdog and log. No training is invoked.
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
ROOT = PACKAGE / "runs/rollout/V26_robustness_2026-09-23"
PLAN = PACKAGE / "runs/plans/V26_robustness_2026-09-23"
MAX_CONCURRENT = 4


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temp.replace(path)


def now():
    return datetime.now().astimezone().isoformat()


def main():
    control = ROOT / "deterministic_original_order"
    summary = read(control / "rollout_summary.json")
    assert summary["ok"] and summary["steps"] > 0, "Arithmetic control has no valid result"
    assert read(control / "watchdog_summary.json")["returncode"] == 0
    assert read(control / "noise_diagnostic.json")["arithmetic_order"] == "original"
    cases = [{"name": f"noise_{scale:g}_seed{seed}", "scale": scale, "seed": seed}
             for seed in range(123, 128) for scale in (1., .5)]
    PLAN.mkdir(parents=True, exist_ok=True)
    with (PLAN / "execution.lock").open("x") as stream:
        stream.write(str(os.getpid()) + "\n")
    manifest = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in (HERE / "case_run.py", Path(__file__))}
    protocol = {"started_at": now(), "training": False, "environment_seed": 123,
                "noise_seeds": list(range(123, 128)), "noise_scales": [1., .5],
                "arithmetic_order": "converted", "max_concurrent": MAX_CONCURRENT,
                "threads_per_simulation": 1, "steps": 500,
                "deterministic_control": str(control), "rollout_root": str(ROOT),
                "record_outputs": True, "best72_stochastic_record_outputs": False,
                "recording_note": "I/O recording enabled here for physical torque analysis; no controller/config change.",
                "script_hashes": manifest}
    write(PLAN / "protocol.json", protocol)
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[key] = "1"
    active, results, pending = {}, [], cases.copy()

    def status(state):
        progress = {}
        for name in active:
            path = ROOT / name / "progress.json"
            try:
                progress[name] = read(path) if path.exists() else {"phase": "startup"}
            except json.JSONDecodeError:
                progress[name] = {"phase": "updating progress"}
        write(PLAN / "status.json", {"state": state, "updated_at": now(), "pid": os.getpid(),
               "completed": len(results), "total": len(cases), "active": progress,
               "pending": [c["name"] for c in pending], "results": results})

    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        while pending or active:
            while pending and len(active) < MAX_CONCURRENT:
                case = pending.pop(0)
                name = case["name"]
                if (ROOT / name).exists():
                    raise FileExistsError(f"Refusing an existing stochastic case: {name}")
                stream = (PLAN / f"{name}.log").open("w")
                command = [sys.executable, str(HERE / "case_run.py"), "--case", name,
                           "--order", "converted", "--scale", str(case["scale"]), "--seed", str(case["seed"])]
                process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, env=env)
                active[name] = (process, stream, case)
                print(f"Started {name}, watchdog pid {process.pid}", flush=True)
            for name, (process, stream, case) in list(active.items()):
                code = process.poll()
                if code is None:
                    continue
                stream.close()
                del active[name]
                result = dict(case, returncode=code, finished_at=now())
                path = ROOT / name / "rollout_summary.json"
                if code == 0 and path.exists():
                    data = read(path)
                    result.update({k: data[k] for k in ("steps", "end_reason", "completed_full_episode",
                                                       "episode_return", "grf_penetration_max_m", "terminal_step_partial")})
                else:
                    result["technical_failure"] = True
                results.append(result)
                print(json.dumps(result), flush=True)
            status("running")
            if pending or active:
                time.sleep(2)
        for path, digest in manifest.items():
            assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest, f"Script changed: {path}"
        status("failed" if any(r.get("technical_failure") for r in results) else "completed")
        write(PLAN / "results.json", {"protocol": protocol, "cases": results})
        if any(r.get("technical_failure") for r in results):
            raise SystemExit(1)
    except BaseException:
        for process, _, _ in active.values():
            if process.poll() is None:
                process.send_signal(signal.SIGINT if os.name != "nt" else signal.SIGTERM)
        for process, stream, _ in active.values():
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.kill()
            stream.close()
        status("interrupted_or_failed")
        raise


if __name__ == "__main__":
    main()
