"""Frozen actor inference on saved resets, with critic-suffix counterfactuals.

No environment is constructed or stepped. The original baseline evaluator
loads the inference stack and exported best47 RLModule. This checks direct
dependence of inference on privileged inputs, not privileged training signals.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import platform
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

AUDIT = Path(__file__).resolve().parent
REPO = AUDIT.parents[2]
EVALUATOR = REPO / "Trajectory Generator/baseline_MLP/rollout_eval.py"
CHECKPOINT = REPO / "Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def checkpoint_hashes():
    return {path.relative_to(CHECKPOINT).as_posix(): sha256(path)
            for path in sorted(CHECKPOINT.rglob("*")) if path.is_file()}


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def tensor_state_digest(module):
    digest = hashlib.sha256()
    for name, value in sorted(module.state_dict().items()):
        array = value.detach().cpu().contiguous().numpy()
        digest.update(name.encode("utf-8"))
        digest.update(str(array.dtype).encode("ascii"))
        digest.update(str(array.shape).encode("ascii"))
        digest.update(array.tobytes())
    return digest.hexdigest()


def load_evaluator():
    spec = importlib.util.spec_from_file_location("reset_privilege_original_evaluator", EVALUATOR)
    evaluator = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = evaluator
    spec.loader.exec_module(evaluator)
    evaluator._load_inference_stack()
    return evaluator


def forward(evaluator, module, observation):
    tensor = evaluator.torch.as_tensor(evaluator.np.asarray(observation), dtype=evaluator.torch.float32).reshape(1, -1)
    with evaluator.torch.no_grad():
        result = module.forward_inference({"obs": tensor})
    return result["action_dist_inputs"].detach().cpu().numpy().copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", type=Path, default=AUDIT / "rollouts/passive_reset_probe/reset_probe.json")
    parser.add_argument("--output", type=Path, default=AUDIT / "privilege_invariance_receipt.json")
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"Preserve previous receipt; select a new --output: {args.output}")
    started = time.monotonic()
    receipt = {"schema_version": 1, "instruction_check_token": "CMC_AGENT_OK_2026",
               "status": "started", "created_utc": datetime.now(timezone.utc).isoformat(),
               "checkpoint": str(CHECKPOINT), "checkpoint_logical_iteration": 47,
               "probe": str(args.probe.resolve()), "environment_constructions": 0,
               "integration_steps": 0, "policy_updates": 0,
               "runtime": {"python": sys.version, "executable": sys.executable, "platform": platform.platform()},
               "interpretation": "Exact policy distribution invariance under changes to critic-only suffix tests direct inference dependence. It does not establish autonomous biological support, remove IK reset information from actor state, or test independence from privileged training rewards/advantages."}
    try:
        probe_bytes = args.probe.read_bytes()
        probe = json.loads(probe_bytes)
        if probe["integration_steps"] != 0:
            raise ValueError("Expected passive reset observations without integration")
        receipt["probe_sha256_before"] = hashlib.sha256(probe_bytes).hexdigest()
        receipt["source_sha256"] = {str(EVALUATOR.relative_to(REPO)): sha256(EVALUATOR),
                                    str(Path(__file__).resolve().relative_to(REPO)): sha256(Path(__file__))}
        receipt["checkpoint_sha256_before"] = checkpoint_hashes()
        evaluator = load_evaluator()
        module = evaluator.RLModule.from_checkpoint(CHECKPOINT.resolve())
        module.eval()
        n_actor, n_full = int(module._n_actor), int(module._n_full)
        if (n_actor, n_full) != (35, 84):
            raise ValueError(f"Unexpected best47 contract: actor={n_actor}, full={n_full}")
        receipt.update(actor_feature_count=n_actor, full_feature_count=n_full, privileged_suffix_count=n_full - n_actor)
        receipt["module_state_digest_before"] = tensor_state_digest(module)
        receipt["runtime"].update(numpy=evaluator.np.__version__, torch=evaluator.torch.__version__)
        rng = evaluator.np.random.default_rng(41012)
        records = []
        for record in probe["records"]:
            observation = evaluator.np.asarray(record["observation"], dtype=evaluator.np.float32)
            if observation.shape != (n_full,) or not evaluator.np.isfinite(observation).all():
                raise ValueError("Saved observation has incompatible shape or nonfinite values")
            feature_names = record["reset_info"]["observation_feature_names"]
            if len(feature_names) != n_full:
                raise ValueError("Saved feature-name contract mismatch")
            reference = forward(evaluator, module, observation)
            action, mean, std, noise = evaluator._policy_action_with_diagnostics(module, observation, (1, 2), action_selection="deterministic")
            if not evaluator.np.isfinite(reference).all():
                raise ValueError("Nonfinite policy distribution inputs")
            changes = {
                "suffix_zero": evaluator.np.zeros(n_full - n_actor, dtype=evaluator.np.float32),
                "suffix_permuted": observation[n_actor:][rng.permutation(n_full - n_actor)],
                "suffix_large_finite": (evaluator.np.where(evaluator.np.arange(n_full - n_actor) % 2, 1.0, -1.0) * 1e6).astype(evaluator.np.float32),
            }
            checks = {}
            for name, suffix in changes.items():
                counterfactual = observation.copy()
                counterfactual[n_actor:] = suffix
                if not evaluator.np.array_equal(counterfactual[:n_actor], observation[:n_actor]):
                    raise RuntimeError("Actor prefix changed in a critic-only counterfactual")
                logits = forward(evaluator, module, counterfactual)
                changed_action = evaluator._deterministic_action(module, counterfactual, (1, 2))
                checks[name] = {"actor_prefix_preserved_exactly": True,
                                "suffix_changed": not evaluator.np.array_equal(counterfactual[n_actor:], observation[n_actor:]),
                                "distribution_inputs_equal_exactly": bool(evaluator.np.array_equal(logits, reference)),
                                "distribution_max_abs_delta": float(evaluator.np.max(evaluator.np.abs(logits - reference))),
                                "deterministic_action_equal_exactly": bool(evaluator.np.array_equal(changed_action, action))}
            state = record["state"]
            actor_map = {name: float(observation[i]) for i, name in enumerate(feature_names[:n_actor])}
            records.append({"case": record["case"], "seed": record["seed"], "reset_time_s": state["time_s"],
                            "actor_observation": actor_map, "distribution_inputs": reference.tolist(),
                            "first_raw_deterministic_action": action.tolist(), "first_clipped_action": evaluator.np.clip(action, -1, 1).tolist(),
                            "gaussian_mean": mean.tolist(), "gaussian_std": std.tolist(),
                            "deterministic_noise": noise.tolist(), "counterfactual_checks": checks,
                            "reset_ik_q": state["q_ik"], "reset_ik_qdot": state["qdot_ik"], "reset_ik_qddot": state["qddot_ik"],
                            "actual_q_after_reset": state["actual_q"], "actual_qdot_after_reset": state["actual_qdot"],
                            "served_reference_after_reset": state["served_reference_after_reset"],
                            "sea_motor_speed_after_reset": state["sea_motor_speed"],
                            "full_initial_state": state["full_state_after_reset"],
                            "served_acceleration_actor_channels": {key: value for key, value in actor_map.items() if key.endswith("served_ref_accel")},
                            "online_left_period_actor_channel": actor_map.get("online_left_cycle_duration_s")})
        nominal = [record for record in records if record["case"]["id"] == "nominal"]
        if [record["seed"] for record in nominal] != [123, 124, 125]:
            raise ValueError("Expected exactly three nominal fixed-start resets, seeds 123/124/125")
        receipt["nominal_first_actions_identical_across_seeds"] = all(record["first_raw_deterministic_action"] == nominal[0]["first_raw_deterministic_action"] for record in nominal[1:])
        receipt["nominal_distribution_inputs_identical_across_seeds"] = all(record["distribution_inputs"] == nominal[0]["distribution_inputs"] for record in nominal[1:])
        receipt["all_critic_suffix_distribution_inputs_invariant_exactly"] = all(check["distribution_inputs_equal_exactly"] for record in records for check in record["counterfactual_checks"].values())
        receipt["all_critic_suffix_actions_invariant_exactly"] = all(check["deterministic_action_equal_exactly"] for record in records for check in record["counterfactual_checks"].values())
        receipt["records"] = records
        receipt["module_state_digest_after"] = tensor_state_digest(module)
        receipt["checkpoint_sha256_after"] = checkpoint_hashes()
        receipt["probe_sha256_after"] = sha256(args.probe)
        receipt["module_state_unchanged"] = receipt["module_state_digest_after"] == receipt["module_state_digest_before"]
        receipt["checkpoint_files_unchanged"] = receipt["checkpoint_sha256_after"] == receipt["checkpoint_sha256_before"]
        receipt["probe_unchanged"] = receipt["probe_sha256_after"] == receipt["probe_sha256_before"]
        checks = ("nominal_first_actions_identical_across_seeds", "nominal_distribution_inputs_identical_across_seeds",
                  "all_critic_suffix_distribution_inputs_invariant_exactly", "all_critic_suffix_actions_invariant_exactly",
                  "module_state_unchanged", "checkpoint_files_unchanged", "probe_unchanged")
        receipt["status"] = "PASS" if all(receipt[key] for key in checks) else "FAIL"
        receipt["observation_count"] = len(records)
        receipt["critic_suffix_counterfactual_count"] = len(records) * len(changes)
        return_code = 0 if receipt["status"] == "PASS" else 1
    except Exception as exc:
        receipt.update(status="ERROR", error=f"{type(exc).__name__}: {exc}")
        traceback.print_exc()
        return_code = 1
    receipt["elapsed_wall_s"] = time.monotonic() - started
    write_json(args.output, receipt)
    print(json.dumps({"status": receipt["status"], "receipt": str(args.output),
                      "observations": receipt.get("observation_count"),
                      "counterfactuals": receipt.get("critic_suffix_counterfactual_count")}))
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
