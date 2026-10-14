"""Separate four-case physical evaluation plan for the same-batch correction.

`prepare` writes cases only; it never starts a simulation. `verify` requires the
completed, exactly reproduced control replay and freezes the corrected export.
`run` is an explicit separate action, using the unchanged evaluator and at most
two cases concurrently. Schedule it after the original evaluation runner exits
to maintain the experiment-wide limit of two physical simulations.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
from pathlib import Path
import shutil

from run_audit import HERE, PACKAGE, OUTPUT, sha256, write_json
from run_evaluations import run_case

MANIFEST = HERE / "corrected_evaluation_manifest.json"
READY = HERE / "corrected_evaluation_ready.json"
REPLAY = HERE / "boundary_replay_v2"
EXPORT = REPLAY / "corrected_boundary/rl_module"
PACKAGED = HERE / "evaluation_modules/corrected"
ROLLOUTS = PACKAGE / "runs/rollout/rl_learning_audit_2026-09-29"


def planned_cases(checkpoint=PACKAGED):
    cases = []
    for scenario, seed in (("nominal", 123), ("noisy", 123), ("noisy", 125), ("noisy", 127)):
        name = "corrected_" + scenario + (f"_{seed}" if scenario == "noisy" else "")
        cases.append({"name": name, "role": "after", "comparison_role": "corrected",
            "update_variant": "same_batch_boundary_bootstrap_corrected",
            "scenario": scenario, "checkpoint": str(Path(checkpoint).resolve()),
            "output_dir": str(ROLLOUTS / name), "environment_seed": 123,
            "noise_seed": seed, "maximum_steps": 500, "sigma_mode": "own", "render": True,
            "case_json": str(HERE / "corrected_cases" / f"{name}.json")})
    return cases


def prepare(module_export=EXPORT, replay=REPLAY):
    if MANIFEST.exists():
        raise FileExistsError("Corrected evaluation manifest already exists; preserve the plan")
    from evaluate import validate_case
    cases = planned_cases()
    for case in cases:
        validate_case(case)
        if Path(case["output_dir"]).exists() or Path(case["case_json"]).exists():
            raise FileExistsError(case["name"])
    for case in cases:
        write_json(case["case_json"], case)
    code = [Path(__file__), HERE / "evaluate.py", HERE / "run_evaluations.py"]
    write_json(MANIFEST, {"schema": "same_batch_corrected_evaluation_v1", "cases": cases,
        "replay_directory": str(Path(replay).resolve()), "checkpoint": str(PACKAGED.resolve()),
        "source_module_export": str(Path(module_export).resolve()),
        "max_parallelism": 2, "prepared_without_simulation": True,
        "evaluation_role_alias": "role=after is the unchanged evaluator API; comparison_role=corrected identifies this separate branch",
        "primary": "One nominal deployment and three paired own-sigma training-connector evaluations (123,125,127)",
        "scheduling": "Run only after original evaluation workers exit; global physical concurrency stays at two",
        "required_replay_evidence": ["replay_complete.json", "control_verification.json", "input_hashes.json", "corrected_boundary/boundary_correction.json"],
        "code_hashes": {str(p.resolve()): sha256(p) for p in code},
        "case_hashes": {case["case_json"]: sha256(case["case_json"]) for case in cases}})
    print(json.dumps({"prepared": str(MANIFEST), "cases": len(cases), "physical_simulations_started": 0}))


def verify_ready():
    manifest = json.loads(MANIFEST.read_text())
    for group in ("code_hashes", "case_hashes"):
        for path, digest in manifest[group].items():
            if sha256(path) != digest:
                raise RuntimeError(f"Frozen corrected evaluation input changed: {path}")
    replay = Path(manifest["replay_directory"])
    receipt = json.loads((replay / "replay_complete.json").read_text())
    control = json.loads((replay / "control_verification.json").read_text())
    if not (receipt.get("complete") and control.get("pass") and receipt.get("control_reproduction", {}).get("pass")):
        raise RuntimeError("Corrected evaluation requires completed replay with exact control reproduction")
    if not receipt.get("all_source_inputs_unchanged") or not all(receipt["unchanged_batch_columns"].values()):
        raise RuntimeError("Replay changed unrelated source inputs or batch columns")
    if receipt.get("fresh_sampling") or receipt.get("physical_simulations") or receipt.get("ray_initialized"):
        raise RuntimeError("Expected isolated same-batch learner replay")
    source_export = Path(manifest["source_module_export"])
    if Path(receipt["corrected_boundary"]["module_export"]).resolve() != source_export.resolve():
        raise RuntimeError("Corrected export path differs from the completed replay receipt")
    for name in ("module_state.pkl", "class_and_ctor_args.pkl", "training_cfg.resolved.yaml"):
        if not (source_export / name).is_file():
            raise FileNotFoundError(source_export / name)
    contract = next((p for p in (OUTPUT / "unified_contract.json", OUTPUT / "checkpoint_last/unified_contract.json") if p.is_file()), None)
    if contract is None:
        raise FileNotFoundError("Captured audit checkpoint observation/action contract")
    export = Path(manifest["checkpoint"])
    source_files = [p for p in source_export.rglob("*") if p.is_file() and "__pycache__" not in p.parts]
    if not export.exists():
        shutil.copytree(source_export, export)
        shutil.copy2(contract, export / "unified_contract.json")
        write_json(export / "audit_export_provenance.json", {
            "source_module_export": str(source_export), "source_module_state_sha256": sha256(source_export / "module_state.pkl"),
            "contract_source": str(contract), "contract_sha256": sha256(contract),
            "replay_receipt": str(replay / "replay_complete.json"), "full_training_checkpoint": False,
            "description": "Evaluation-only copy of same-batch corrected module; source replay remains unchanged"})
    for source in source_files:
        if sha256(source) != sha256(export / source.relative_to(source_export)):
            raise RuntimeError(f"Packaged module differs from source export: {source}")
    if sha256(contract) != sha256(export / "unified_contract.json"):
        raise RuntimeError("Packaged observation/action contract differs")
    files = [p for p in export.rglob("*") if p.is_file() and "__pycache__" not in p.parts]
    files += [replay / name for name in manifest["required_replay_evidence"]]
    files += [MANIFEST, contract, *source_files]
    hashes = {str(p.resolve()): sha256(p) for p in files}
    if READY.exists():
        prior = json.loads(READY.read_text())
        if hashes != prior["input_hashes"]:
            raise RuntimeError("Corrected export or completed replay evidence changed after readiness freeze")
    else:
        write_json(READY, {"ready": True, "control_exact": True, "input_hashes": hashes,
            "physical_simulations_started_by_verification": 0})
    return manifest


def run():
    manifest = verify_ready()
    results = []
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = {pool.submit(run_case, case): case for case in manifest["cases"]}
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            print(json.dumps(result), flush=True)
    write_json(HERE / "corrected_evaluation_execution.json", results)
    verify_ready()
    if any(r["returncode"] != 0 or not r.get("receipt_exists", r.get("reused_completed")) for r in results):
        raise SystemExit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare", "verify", "run"))
    parser.add_argument("--module-export", type=Path, default=EXPORT)
    parser.add_argument("--replay", type=Path, default=REPLAY)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.module_export, args.replay)
    elif args.command == "verify":
        verified = verify_ready()
        print(json.dumps({"ready": True, "cases": len(verified["cases"]), "physical_simulations_started": 0}))
    else:
        run()
