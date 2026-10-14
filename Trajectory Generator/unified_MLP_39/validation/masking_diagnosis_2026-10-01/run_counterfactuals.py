"""Three frozen-policy visibility controls; no training or source mutation."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
ROOT = PACKAGE.parents[1]
TRAIN = PACKAGE / "runs/training"
MASK = TRAIN / "MLP39_LEFT_best90_masking_100iter_2026-09-30"
CASES = [
    ("cp90_hidden", TRAIN / "MLP39_LEFT_fresh_best80config_100iter_2026-09-29/rl_module_best", 0),
    ("cp112_visible", MASK / "stage01_mask25/rl_module_best", 1),
    ("cp184_visible", MASK / "stage04_mask100/rl_module_best", 1),
]


def hashes(folder):
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.rglob("*")) if p.is_file()}


def run_case(case):
    label, checkpoint, visibility = case
    output = PACKAGE / "runs/rollout/masking_diagnosis_2026-10-01" / label
    if output.exists():
        raise FileExistsError(output)
    before = hashes(checkpoint)
    command = [sys.executable, "-u", str(PACKAGE / "rollout_eval.py"),
               "--checkpoint", str(checkpoint), "--output-dir", str(output),
               "--episode-duration", "5", "--episode-start-offset-s", "1",
               "--max-steps", "500", "--seed", "123", "--action-selection", "deterministic",
               "--target-availability-probability", str(visibility),
               "--record-outputs", "--record-policy-trace"]
    record = {"label": label, "command": command, "source_hashes": before,
              "rollout": str(output), "visibility": visibility, "started_epoch": time.time()}
    (HERE / f"{label}_launch.json").write_text(json.dumps(record, indent=2) + "\n")
    env = dict(os.environ, MPLBACKEND="Agg")
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
        env[key] = "1"
    with (HERE / f"{label}.log").open("x") as log:
        try:
            result = subprocess.run(command, cwd=ROOT, env=env, stdout=log,
                                    stderr=subprocess.STDOUT, timeout=1800)
            record["exit_code"] = result.returncode
        except subprocess.TimeoutExpired:
            record["operational_timeout"] = True
    record["elapsed_seconds"] = time.time() - record["started_epoch"]
    record["checkpoint_unchanged"] = hashes(checkpoint) == before
    summary = output / "rollout_summary.json"
    if summary.exists():
        record["summary"] = json.loads(summary.read_text())
    (HERE / f"{label}_execution.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({k: record[k] for k in ("label", "elapsed_seconds", "checkpoint_unchanged")}), flush=True)
    return record


if __name__ == "__main__":
    with ThreadPoolExecutor(max_workers=3) as pool:
        results = list(pool.map(run_case, CASES))
    (HERE / "counterfactual_results.json").write_text(json.dumps(results, indent=2) + "\n")
    if not all(r.get("exit_code") == 0 and r["checkpoint_unchanged"] for r in results):
        raise SystemExit(1)
