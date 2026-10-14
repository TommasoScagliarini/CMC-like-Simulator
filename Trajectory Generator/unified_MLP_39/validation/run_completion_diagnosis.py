"""Bounded pre-training evaluations of checkpoint 40; at most four CPU workers."""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
UNIFIED = ROOT / "Trajectory Generator/unified_MLP_39"
BASE = UNIFIED / "validation/completion_diagnostics_2026-09-22"
RUN = UNIFIED / "runs/training/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21"
CHECKPOINT = RUN / "milestone_iteration_000040/rl_module_last"
ROLLOUT_ROOT = UNIFIED / "runs/rollout/completion_diagnostics_2026-09-22_iter40"


def main():
    BASE.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[key] = "1"
    jobs = [(0.,123)] + [(scale,seed) for seed in range(123,128) for scale in (1.,.5)]
    protocol = {"checkpoint": str(CHECKPOINT), "environment_seed":123,
                "noise_seeds":list(range(123,128)), "noise_scales":[0.,.5,1.],
                "max_parallel_rollouts":4, "training_concurrent":False,
                "interpretation":"Exploratory paired check; five episodes per stochastic level cannot establish statistical equivalence."}
    (BASE / "protocol.json").write_text(json.dumps(protocol,indent=2)+"\n")
    def run_one(job):
        scale, seed = job
        name = f"noise_{scale:g}_seed_{seed}"
        output = ROLLOUT_ROOT / name
        if (output / "noise_diagnostic.json").is_file():
            summary = json.loads((output / "rollout_summary.json").read_text())
            assert summary["ok"]
        else:
            command = [sys.executable,"-u",str(UNIFIED / "validation/completion_noise_rollout.py"),
                       "--noise-scale",str(scale),"--noise-seed",str(seed),
                       "--checkpoint",str(CHECKPOINT),"--output-dir",str(output),
                       "--seed","123","--max-steps","500","--no-progress",
                       "--no-record-outputs","--record-policy-trace"]
            with (BASE / f"{name}.log").open("a") as log:
                result = subprocess.run(command,cwd=ROOT,env=env,stdout=log,
                                        stderr=subprocess.STDOUT,timeout=1800)
            if result.returncode:
                raise RuntimeError(f"{name}: exit {result.returncode}; inspect log")
            summary = json.loads((output / "rollout_summary.json").read_text())
            assert summary["ok"]
        row = {"noise_scale":scale,"noise_seed":seed,"output":str(output),
               **{k:summary.get(k) for k in ("steps","end_reason","episode_return",
                  "grf_penetration_max_m","action_clipped_fraction","phase_valid_cycle_count")}}
        print(json.dumps(row),flush=True)
        return row
    rows=[]
    started=time.monotonic()
    with ThreadPoolExecutor(max_workers=4) as executor:
        for future in as_completed([executor.submit(run_one,job) for job in jobs]):
            rows.append(future.result())
            data={"ok":False,"completed":len(rows),"total":len(jobs),"rows":rows}
            (BASE / "diagnosis_progress.json").write_text(json.dumps(data,indent=2)+"\n")
    results={**protocol,"ok":True,"wall_seconds":time.monotonic()-started,
             "rows":sorted(rows,key=lambda row:(row['noise_scale'],row['noise_seed']))}
    (BASE / "diagnosis_results.json").write_text(json.dumps(results,indent=2)+"\n")
    print(f"Completed {len(rows)} diagnostic rollouts",flush=True)


if __name__ == "__main__":
    main()
