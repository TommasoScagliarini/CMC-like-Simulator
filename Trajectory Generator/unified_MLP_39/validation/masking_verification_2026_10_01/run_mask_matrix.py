"""Six deterministic 5s partial-mask tests, at most three concurrent processes."""
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
SOURCES = {90: TRAIN / "MLP39_LEFT_fresh_best80config_100iter_2026-09-29/rl_module_best",
           112: MASK / "stage01_mask25/rl_module_best",
           184: MASK / "stage04_mask100/rl_module_best"}


def run(case):
    cp, mask = case
    label = f"cp{cp}_{mask}"
    source = SOURCES[cp]
    output = PACKAGE / "runs/rollout/masking_verification_2026-10-01" / label
    if output.exists():
        raise FileExistsError(output)
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in source.rglob("*") if p.is_file()}
    command = [sys.executable, "-u", str(HERE / "mask_rollout.py"),
               "--checkpoint", str(source), "--output-dir", str(output),
               "--episode-duration", "5", "--episode-start-offset-s", "1",
               "--max-steps", "500", "--seed", "123", "--action-selection", "deterministic",
               "--target-availability-probability", "1", "--record-outputs", "--record-policy-trace"]
    env = dict(os.environ, CMC_DIAGNOSTIC_MASK=mask, MPLBACKEND="Agg")
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
        env[key] = "1"
    record = dict(label=label, checkpoint=str(source), rollout=str(output), command=command,
                  input_sha256=hashes, mask=mask, started=time.time())
    (HERE / f"{label}_launch.json").write_text(json.dumps(record, indent=2)+"\n")
    with (HERE / f"{label}.log").open("x") as log:
        result = subprocess.run(command, cwd=ROOT, env=env, stdout=log,
                                stderr=subprocess.STDOUT, timeout=1800)
    record.update(exit_code=result.returncode, elapsed_s=time.time()-record["started"])
    record["inputs_unchanged"] = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    if result.returncode == 0:
        summary = json.loads((output / "rollout_summary.json").read_text())
        trace = json.loads((output / "rollout_policy_trace.json").read_text())
        assert summary["ok"] and summary["steps"] == len(trace)
        for row in trace:
            assert row["target_availability"]["diagnostic_mask"] == mask
            x = row["actor_observation_vector_before"]
            indices = (0,1) if mask == "clock_hidden" else (35,36,37,38)
            assert all(x[i]==0 for i in indices)
            if mask == "target_hidden":
                assert abs(x[0]**2+x[1]**2-1)<1e-5
        record.update(steps=len(trace), episode_return=summary["episode_return"],
                      end_reason=summary["end_reason"], mask_verified=True)
    (HERE / f"{label}_execution.json").write_text(json.dumps(record,indent=2)+"\n")
    print(json.dumps({k:record.get(k) for k in ("label","exit_code","steps","end_reason","elapsed_s")}),flush=True)
    return record


if __name__ == "__main__":
    with ThreadPoolExecutor(max_workers=3) as pool:
        results = list(pool.map(run, [(cp,mask) for cp in SOURCES for mask in ("clock_hidden","target_hidden")]))
    (HERE / "matrix_execution.json").write_text(json.dumps(results,indent=2)+"\n")
    if not all(r['exit_code']==0 and r['inputs_unchanged'] for r in results):
        raise SystemExit(1)
