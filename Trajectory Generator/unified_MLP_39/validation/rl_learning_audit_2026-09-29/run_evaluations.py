"""Prepare and run paired physical evaluations, at most two at a time."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from run_audit import HERE, PACKAGE, REPO, SOURCE, OUTPUT, sha256, write_json

MANIFEST = HERE / "evaluation_manifest.json"
ROLLOUTS = PACKAGE / "runs/rollout/rl_learning_audit_2026-09-29"


def prepare():
    if MANIFEST.exists():
        raise FileExistsError(MANIFEST)
    snapshot = OUTPUT / "ppo_audit/snapshots/first_pass/rl_module_default_policy"
    exported = HERE / "evaluation_modules/first_pass"
    shutil.copytree(snapshot, exported)
    for filename in ("training_cfg.resolved.yaml", "unified_contract.json"):
        shutil.copy2(SOURCE / filename, exported / filename)
    write_json(exported / "audit_export_provenance.json", {
        "source_snapshot": str(snapshot), "source_module_state_sha256": sha256(snapshot / "module_state.pkl"),
        "runtime_and_contract_source": str(SOURCE), "full_training_checkpoint": False,
        "description": "Evaluation-only module export; first pass may include part of the next epoch",
    })
    checkpoints = {"before": SOURCE, "after": OUTPUT / "checkpoint_last", "first_pass": exported}
    cases = []
    for role, checkpoint in checkpoints.items():
        scenarios = [("nominal", 123)]
        if role != "first_pass":
            scenarios += [("noisy", seed) for seed in (123, 125, 127)]
        for scenario, seed in scenarios:
            name = f"{role}_{scenario}" + (f"_{seed}" if scenario == "noisy" else "")
            case = {"name": name, "role": role, "scenario": scenario,
                    "checkpoint": str(checkpoint), "output_dir": str(ROLLOUTS / name),
                    "environment_seed": 123, "noise_seed": seed, "maximum_steps": 500,
                    "sigma_mode": "own", "render": True}
            if role == "before" and scenario == "nominal":
                case["reuse_from"] = str(PACKAGE / "runs/rollout/general_method_diagnosis_2026-09-29/LEFT_baseline")
            path = HERE / "cases" / f"{name}.json"
            write_json(path, case)
            cases.append({**case, "case_json": str(path)})
    write_json(MANIFEST, {"cases": cases, "max_parallelism": 2,
                         "primary": "before/after nominal and paired own-sigma stochastic seeds123/125/127",
                         "first_pass": "Exploratory nominal evaluation; not a separate training run"})
    print(json.dumps({"prepared": str(MANIFEST), "cases": len(cases)}))


def run_case(case):
    output = Path(case["output_dir"])
    receipt = output / "evaluation_complete.json"
    if receipt.exists():
        saved = json.loads(receipt.read_text())
        if any(sha256(output / name) != digest for name, digest in saved["artifact_hashes"].items()):
            raise RuntimeError(f"Completed evaluation changed: {case['name']}")
        return {"name": case["name"], "reused_completed": True, "returncode": 0}
    logs = HERE / "evaluation_logs"
    logs.mkdir(exist_ok=True)
    env = os.environ.copy()
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[key] = "1"
    start = time.monotonic()
    command = [sys.executable, "-u", str(HERE / "evaluate.py"), "--case-json", case["case_json"]]
    with (logs / f"{case['name']}.log").open("a", encoding="utf-8") as stream:
        result = subprocess.run(command, cwd=REPO, env=env, stdout=stream, stderr=subprocess.STDOUT)
    row = {"name": case["name"], "returncode": result.returncode,
           "elapsed_seconds": time.monotonic() - start, "receipt_exists": receipt.exists()}
    write_json(logs / f"{case['name']}_execution.json", row)
    return row


def run(roles):
    manifest = json.loads(MANIFEST.read_text())
    selected = [case for case in manifest["cases"] if case["role"] in roles]
    results = []
    with ThreadPoolExecutor(max_workers=2) as pool:
        pending = {pool.submit(run_case, case): case for case in selected}
        for future in as_completed(pending):
            result = future.result()
            results.append(result)
            print(json.dumps(result), flush=True)
    write_json(HERE / ("evaluation_execution_" + "_".join(sorted(roles)) + ".json"), results)
    if any(row["returncode"] != 0 or not row.get("receipt_exists", row.get("reused_completed")) for row in results):
        raise SystemExit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("prepare", "run"))
    parser.add_argument("--roles", nargs="+", choices=("before", "after", "first_pass"), default=["before", "after", "first_pass"])
    args = parser.parse_args()
    prepare() if args.command == "prepare" else run(args.roles)
