"""Prepare/run frozen evaluations of existing pilot checkpoints; never train.

Uses the existing bounded rollout launcher with an empty training branch map.
All measured references and checkpoint bytes are frozen in a separate manifest;
original pilot manifests, state, code and rollout artifacts remain untouched.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE.parent))
from unified_MLP_39.reward_tuning import pilot
from unified_MLP_39.reward_tuning.compare import DEFAULTS

EXPERIMENT = PACKAGE / "runs/reward_tuning/left_pilot_diagnosis_2026-09-28"


def prepare(experiment=EXPERIMENT):
    experiment = Path(experiment).resolve()
    output = experiment / "nominal"
    if (output / "pilot_manifest.json").exists():
        raise FileExistsError("Manifest already prepared; use run to resume its remaining budget")
    old10 = PACKAGE / "runs/reward_tuning/left_best80_2026-09-25/pilot"
    old5 = PACKAGE / "runs/reward_tuning/left_best80_ankle5_2026-09-25/pilot"
    m10 = pilot.read_json(old10 / "pilot_manifest.json")
    m5 = pilot.read_json(old5 / "pilot_manifest.json")
    hashes = dict(m5["input_hashes"])
    pilot.verify_hashes(ROOT, hashes)

    def pin(path):
        path = Path(path).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        hashes[str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)] = pilot.sha256(path)

    for folder in (old10, old5):
        state = pilot.read_json(folder / "pilot_state.json")
        if state["status"] != "completed" or state["manifest_sha256"] != pilot.sha256(folder / "pilot_manifest.json"):
            raise ValueError("Reference pilot is not completed with an intact manifest")
        pin(folder / "pilot_manifest.json")
        pin(folder / "pilot_state.json")
    pin(Path(__file__))
    baseline = copy.deepcopy(next(c for c in m5["evaluation_cases"] if c["role"] == "best80" and c["scenario"] == "nominal"))
    sigma = Path(baseline["sigma_path"])
    pin(sigma)

    def seal_case(case):
        path = Path(case["output_dir"])
        receipt_path = path / "evaluation_complete.json"
        receipt = pilot.read_json(receipt_path)
        expected_role = case.get("receipt_role", case["role"])
        if receipt["role"] != expected_role or receipt["scenario"] != case["scenario"]:
            raise ValueError("Reference receipt identity mismatch")
        required = {"rollout_summary.json", "rollout_policy_trace.json", "paired_noise_receipt.json"}
        if not required <= set(receipt.get("artifact_hashes", {})):
            raise ValueError("Reference receipt missing required artifacts")
        pilot.verify_hashes(path, receipt["artifact_hashes"])
        for name in receipt["artifact_hashes"]:
            pin(path / name)
        pin(receipt_path)

    seal_case(baseline)
    branches = {"control": m10["branches"]["control"], "ankle_5": m5["branches"]["ankle_5"],
                "ankle_10": m10["branches"]["ankle_10"]}
    cases, checkpoints, endpoints = [baseline], {}, []
    for role, branch in branches.items():
        run_dir = Path(branch["output_dir"])
        pin(run_dir / "train_iterations.jsonl")
        for iteration in (82, 83):
            checkpoint = run_dir / f"milestone_iteration_{iteration:06d}" / "checkpoint_last"
            if pilot.checkpoint_iteration(checkpoint) != iteration:
                raise ValueError(f"Incomplete checkpoint: {checkpoint}")
            training = pilot.read_json(checkpoint / "unified_training_state.json")
            if training.get("last_completed_phase") != "imitation" or training.get("imitation_critic_rewarm_remaining") != 0:
                raise ValueError(f"Unexpected training phase: {checkpoint}")
            for path in checkpoint.rglob("*"):
                if path.is_file():
                    pin(path)
            pin(checkpoint.parent / "checkpoint_last_meta.json")
            identifier = f"{role}_{iteration}"
            destination = PACKAGE / "runs/rollout" / f"left_pilot_diagnosis_2026-09-28_{identifier}_nominal"
            if destination.exists():
                raise FileExistsError(destination)
            case = {**baseline, "role": identifier, "checkpoint": str(checkpoint),
                    "freeze_sigma": False, "output_dir": str(destination)}
            cases.append(case)
            checkpoints[identifier] = {"branch": role, "iteration": iteration, "ppo_updates": iteration - 81,
                                       "checkpoint": str(checkpoint)}
        manifest = m5 if role == "ankle_5" else m10
        endpoint = copy.deepcopy(next(c for c in manifest["evaluation_cases"] if c["role"] == role and c["scenario"] == "nominal"))
        endpoint["role"], endpoint["receipt_role"] = role + "_84", role
        seal_case(endpoint)
        endpoints.append(endpoint)

    manifest = {"schema": "reward_tuning_evaluation_only_v1", "repo_root": str(ROOT),
        "output_dir": str(output), "python": sys.executable, "branches": {}, "training_permitted": False,
        "input_hashes": hashes, "evaluation_cases": cases, "endpoint_cases": endpoints,
        "checkpoint_roles": checkpoints, "budget_seconds": 7200, "evaluation_parallelism": 4,
        "per_process_numeric_threads": 1, "criteria": DEFAULTS, "analysis_window_s": [13.15, 17.95],
        "plot_dir": "plot/09_28_2026_LEFT_pilot_diagnosis",
        "reference_reuse": {"best80_nominal": baseline["output_dir"], "source_pilots": [str(old10), str(old5)]},
        "screening_protocol": {"references": ["best80", "control_at_same_iteration"],
            "nominal_gate": "All physical constraints in full and common window; >=10% positive ankle area reduction against both references",
            "conditional_noise_seeds": [123, 125, 127], "noise_only_after_nominal_gate": True,
            "noise_requires_matched_control": True, "sigma_path": str(sigma),
            "no_new_training": True},
        "cost_estimate": {"training_updates": 0, "new_nominal_evaluations": 6, "reused_nominal_evaluations": 4,
                          "conditional_noise_evaluations": "Only nominal qualifiers and their matched controls; at most 18 new evaluations",
                          "total_execution_budget_minutes": 120}}
    pilot.verify_hashes(ROOT, hashes)
    output.mkdir(parents=True, exist_ok=True)
    pilot.write_json(output / "pilot_manifest.json", manifest)
    pilot.write_json(output / "pilot_state.json", {"schema": manifest["schema"],
        "manifest_sha256": pilot.sha256(output / "pilot_manifest.json"), "status": "prepared", "elapsed_seconds": 0,
        "stages": {"eval_best80_nominal": {"status": "completed", "attempts": 0, "reused_from": str(old5)}},
        "evaluation": {"best80": {"nominal": baseline["output_dir"]}}, "sigma_sha256": pilot.sha256(sigma)})
    return manifest


def run(experiment=EXPERIMENT):
    output = Path(experiment).resolve() / "nominal"
    manifest = pilot.read_json(output / "pilot_manifest.json")
    if manifest.get("schema") != "reward_tuning_evaluation_only_v1" or manifest.get("training_permitted") is not False or manifest["branches"]:
        raise ValueError("This command accepts evaluation-only manifests with no training branches")
    return pilot.run(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("prepare", "run"))
    parser.add_argument("--experiment", type=Path, default=EXPERIMENT)
    args = parser.parse_args()
    if args.operation == "prepare":
        result = prepare(args.experiment)
        print(json.dumps({"status": "prepared", "new_training": False, "cases": len(result["evaluation_cases"]),
                          "input_hashes": len(result["input_hashes"])}, indent=2))
    else:
        state = run(args.experiment)
        print(json.dumps({"status": state["status"], "elapsed_seconds": state["elapsed_seconds"], "new_training": False}, indent=2))


if __name__ == "__main__":
    main()
