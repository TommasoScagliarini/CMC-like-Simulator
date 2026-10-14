"""Replay one captured PPO batch locally; no Algorithm, env or Ray cluster.

First reproduces the recorded stock update and refuses the correction unless
module tensors, Adam tensors and every minibatch input/order are bit-exact.
The correction changes TD-lambda targets on real rows only, retains stock
artificial-row targets, and retains normalization over all elongated rows.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import pickle
import random
import shutil
import sys

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / "AGENTS.md").exists())
TG = REPO / "Trajectory Generator"
DEFAULT_RUN = TG / "unified_MLP_39/runs/training/MLP39_LEFT_best80_rl_audit_2026-09-29"
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[key] = "1"
sys.path.insert(0, str(TG))
sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True

import numpy as np
import torch
import ray
from ray.rllib.connectors.connector_v2 import ConnectorV2
from ray.rllib.env.single_agent_episode import SingleAgentEpisode

import learner_probe
from learner_analysis import array, clean, digest, load_pickle, write_json


def compare_trees(left, right, prefix="", failures=None):
    """Exact numeric state comparison; serialized pickle bytes need not match."""
    failures = [] if failures is None else failures
    if isinstance(left, (np.ndarray, torch.Tensor)):
        a, b = array(left), array(right)
        if a.shape != b.shape or a.dtype != b.dtype or not np.array_equal(a, b):
            item = {"path": prefix, "left_shape": list(a.shape), "right_shape": list(b.shape),
                    "left_dtype": str(a.dtype), "right_dtype": str(b.dtype)}
            if a.shape == b.shape and np.issubdtype(a.dtype, np.number):
                item["max_abs_error"] = float(np.max(np.abs(a.astype(float) - b.astype(float)))) if a.size else 0.
            failures.append(item)
    elif isinstance(left, dict):
        if not isinstance(right, dict) or left.keys() != right.keys():
            failures.append({"path": prefix, "reason": "dictionary keys differ"})
        else:
            for key in left:
                compare_trees(left[key], right[key], f"{prefix}/{key}", failures)
    elif isinstance(left, (tuple, list)):
        if len(left) != len(right):
            failures.append({"path": prefix, "reason": "sequence length differs"})
        else:
            for index, (a, b) in enumerate(zip(left, right)):
                compare_trees(a, b, f"{prefix}/{index}", failures)
    elif left != right:
        failures.append({"path": prefix, "left": clean(left), "right": clean(right)})
    return failures


def corrected_real_targets(batch, values, original_episodes, gamma, lam):
    """Keep artificial stock targets; change only ordinary real-transition GAE."""
    original = array(batch["value_targets"])
    targets = original.copy()
    v = array(values).reshape(-1)
    rewards = array(batch["rewards"])
    mask = array(batch["loss_mask"]).astype(bool)
    expected_rows = sum(len(episode) + 1 for episode in original_episodes)
    if len(v) != expected_rows or mask.shape != (expected_rows,):
        raise ValueError("Unexpected flattened episode layout")
    position = 0
    for episode in original_episodes:
        n = len(episode)
        boundary = position + n
        if not np.all(mask[position:boundary]) or mask[boundary]:
            raise ValueError("Artificial rows do not match original episode boundaries")
        expected_obs = np.asarray(episode.get_observations())
        if not np.array_equal(array(batch["obs"])[position:boundary + 1], expected_obs):
            raise ValueError("Original observation order changed before correction")
        advantage = 0.
        for step in range(n - 1, -1, -1):
            index = position + step
            terminal = bool(episode.is_terminated and step == n - 1)
            delta = float(rewards[index]) + gamma * (not terminal) * float(v[index + 1]) - float(v[index])
            advantage = delta + gamma * lam * (not terminal) * advantage
            targets[index] = advantage + float(v[index])
        position = boundary + 1
    if not np.array_equal(targets[~mask], original[~mask]):
        raise AssertionError("Correction changed masked-row targets")
    raw_advantages = targets - v
    normalized = (raw_advantages - raw_advantages.mean()) / max(1e-4, raw_advantages.std())
    return targets, normalized


class CorrectRealBoundaryGAE(ConnectorV2):
    def __init__(self, learner, states):
        super().__init__()
        self.learner = learner
        self.original_episodes = [SingleAgentEpisode.from_state(copy.deepcopy(state)) for state in states]
        self.receipt = None

    def __call__(self, *, batch, **kwargs):
        if self.receipt is not None:
            raise RuntimeError("Only one immutable batch is authorized")
        if set(batch) != {"default_policy"}:
            raise ValueError("Correction supports the captured single-module batch")
        mid = "default_policy"
        probe = self.learner._rl_learning_audit_probe
        values = probe.gae_values[mid]
        source = batch[mid]
        config = self.learner.config.get_config_for_module(mid)
        before_targets = array(source["value_targets"]).copy()
        before_advantages = array(source["advantages"]).copy()
        targets, advantages = corrected_real_targets(source, values, self.original_episodes, config.gamma, config.lambda_)
        mask = array(source["loss_mask"]).astype(bool)
        source["value_targets"] = torch.as_tensor(targets, dtype=source["value_targets"].dtype,
                                                  device=source["value_targets"].device)
        source["advantages"] = torch.as_tensor(advantages, dtype=source["advantages"].dtype,
                                               device=source["advantages"].device)
        self.receipt = {
            "real_rows": int(mask.sum()), "masked_rows": int((~mask).sum()),
            "masked_targets_unchanged": bool(np.array_equal(targets[~mask], before_targets[~mask])),
            "real_target_max_abs_change": float(np.max(np.abs(targets[mask] - before_targets[mask]))),
            "real_advantage_sign_change_fraction": float(np.mean(np.sign(advantages[mask]) != np.sign(before_advantages[mask]))),
            "normalization": "Same stock convention: standardize target-minus-V over ALL elongated rows, including masked rows",
            "scope": "Correct real-row TD-lambda targets only; retain original flags, masks, raw actions/logp, rewards, observations and masked-row targets",
            "upstream_relation": "Isolated boundary correction, not a wholesale backport of upstream connector/normalization changes",
            "extra_value_forwards": 0,
        }
        return batch


def restore_rng(state):
    random.setstate(state["python"])
    np.random.set_state(state["numpy"])
    torch.set_rng_state(state["torch_cpu"])
    if state.get("torch_cuda"):
        raise ValueError("The captured audit should be CPU-only")


def optimizer_tensors_preserving_python_scalars(value, device):
    """Undo get_state's tensor-to-NumPy conversion without rounding Python LR/betas."""
    if isinstance(value, np.ndarray):
        return torch.from_numpy(value.copy()).to(device)
    if isinstance(value, torch.Tensor):
        return value.detach().clone().to(device)
    if isinstance(value, dict):
        return {key: optimizer_tensors_preserving_python_scalars(item, device) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return type(value)(optimizer_tensors_preserving_python_scalars(item, device) for item in value)
    return copy.deepcopy(value)


def build_restored_learner(run, probe):
    ctor = load_pickle(run / "checkpoint_last/learner_group/learner/class_and_ctor_args.pkl")
    args, kwargs = ctor["ctor_args_and_kwargs"]
    config = kwargs["config"]
    if config.num_learners != 0 or config.num_gpus_per_learner != 0:
        raise ValueError("Replay only permits the original local CPU learner")
    learner = ctor["class"](*args, **kwargs)
    learner.build()
    state = load_pickle(probe / "snapshots/before/learner_state.pkl")
    learner.set_state(state)
    # RLlib's generic convert_to_torch_tensor also casts Python optimizer group
    # scalars (e.g. LR) to float32. Restore their captured precision explicitly.
    for name, optimizer_state in state["optimizer"].items():
        learner._named_optimizers[name].load_state_dict(
            optimizer_tensors_preserving_python_scalars(optimizer_state["state"], learner._device))
    metadata = json.loads((probe / "snapshots/before/metadata.json").read_text())
    scalar = metadata["scalars"]
    for mid, value in scalar["kl_coeff"].items():
        current = learner.curr_kl_coeffs_per_module[mid]
        current.data.copy_(torch.as_tensor(value, dtype=current.dtype, device=current.device))
    for mid, expected in scalar["entropy_coeff"].items():
        actual = float(learner.entropy_coeff_schedulers_per_module[mid].get_current_value())
        if actual != expected:
            raise ValueError(f"Entropy schedule differs for {mid}: {actual} != {expected}")
    for name, expected in scalar["optimizer_lrs"].items():
        actual = [float(group["lr"]) for group in learner._named_optimizers[name].param_groups]
        if actual != expected:
            raise ValueError(f"Optimizer LR restore differs: {name}: actual={actual}, expected={expected}")
    restored = learner.get_state()
    failures = compare_trees(state["rl_module"], restored["rl_module"], "initial/module")
    compare_trees(state["optimizer"], restored["optimizer"], "initial/optimizer", failures)
    if failures:
        raise ValueError(f"Initial restore not exact: {failures[:5]}")
    if ray.is_initialized():
        raise RuntimeError("Unexpected Ray cluster initialization")
    return learner


def verify_control(original_probe, replay_probe):
    expected = load_pickle(original_probe / "snapshots/final/learner_state.pkl")
    actual = load_pickle(replay_probe / "snapshots/final/learner_state.pkl")
    state_failures = compare_trees(expected["rl_module"], actual["rl_module"], "final/module")
    optimizer_failures = compare_trees(expected["optimizer"], actual["optimizer"], "final/optimizer")
    expected_files = sorted((original_probe / "minibatches").glob("*.pkl"))
    actual_files = sorted((replay_probe / "minibatches").glob("*.pkl"))
    input_failures, forward_failures = [], []
    if [p.name for p in expected_files] != [p.name for p in actual_files]:
        input_failures.append({"reason": "minibatch file sets differ"})
    else:
        for left, right in zip(expected_files, actual_files):
            a, b = load_pickle(left), load_pickle(right)
            compare_trees(a["batch"], b["batch"], left.name + "/input", input_failures)
            for key in ("current_logits_before_update", "current_values_before_update", "ratio", "kl"):
                compare_trees(a[key], b[key], left.name + "/" + key, forward_failures)
    left_metadata = json.loads((original_probe / "snapshots/final/metadata.json").read_text())
    right_metadata = json.loads((replay_probe / "snapshots/final/metadata.json").read_text())
    scalar_failures = compare_trees(left_metadata["scalars"], right_metadata["scalars"], "final/scalars")
    return {"pass": not (state_failures or optimizer_failures or input_failures or forward_failures or scalar_failures),
            "final_module_bit_exact": not state_failures, "final_adam_bit_exact": not optimizer_failures,
            "all_minibatch_inputs_and_order_bit_exact": not input_failures,
            "all_minibatch_forward_outputs_bit_exact": not forward_failures,
            "final_scalars_exact": not scalar_failures, "minibatches_checked": len(expected_files),
            "state_failures": state_failures, "optimizer_failures": optimizer_failures,
            "input_failures": input_failures, "forward_failures": forward_failures, "scalar_failures": scalar_failures}


def run_branch(run, source_probe, output, name, states, manifest):
    path = output / name
    path.mkdir(parents=True, exist_ok=False)
    learner = build_restored_learner(run, source_probe)
    correction = None
    if name == "corrected_boundary":
        correction = CorrectRealBoundaryGAE(learner, states)
        learner._learner_connector.append(correction)
    learner_probe.attach_probe(learner, path / "ppo_audit")
    restore_rng(load_pickle(source_probe / "snapshots/before/rng_state.pkl"))
    try:
        episodes = [SingleAgentEpisode.from_state(copy.deepcopy(state)) for state in states]
        learner.update(episodes=episodes, **manifest["update_arguments"])
    finally:
        receipt = learner_probe.detach_probe(learner)
    if not receipt.get("complete"):
        raise RuntimeError("Replay update did not complete")
    if correction is not None:
        write_json(path / "boundary_correction.json", correction.receipt)
    export = path / "rl_module"
    shutil.copytree(path / "ppo_audit/snapshots/final/rl_module_default_policy", export)
    shutil.copy2(run / "training_cfg.resolved.yaml", export / "training_cfg.resolved.yaml")
    shutil.copy2(run / "effective_rllib_configuration.json", path / "effective_rllib_configuration.json")
    if ray.is_initialized():
        raise RuntimeError("Replay initialized Ray")
    return {"name": name, "module_export": str(export.resolve()), "probe": str((path / "ppo_audit").resolve()),
            "minibatches": receipt["minibatches_completed"], "ray_initialized": False,
            "fresh_sampling": False, "boundary_correction": correction.receipt if correction else None}


def run_replay(run, output):
    run, output = Path(run).resolve(), Path(output).resolve()
    source_probe = run / "ppo_audit"
    manifest = json.loads((source_probe / "manifest.json").read_text())
    if not manifest.get("complete"):
        raise ValueError("Captured source update is incomplete")
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite replay: {output}")
    if ray.is_initialized():
        raise RuntimeError("Replay must start without a Ray cluster")
    inputs = {str((source_probe / name).resolve()): sha for name, sha in json.loads((source_probe / "artifact_hashes.json").read_text()).items()}
    for path in (run / "checkpoint_last/learner_group/learner/class_and_ctor_args.pkl", run / "training_cfg.resolved.yaml",
                 run / "effective_rllib_configuration.json", Path(__file__), HERE / "learner_probe.py"):
        inputs[str(path)] = digest(path)
    failed = [name for name, expected in inputs.items() if digest(name) != expected]
    if failed:
        raise ValueError(f"Source hashes differ: {failed}")
    output.mkdir(parents=True)
    write_json(output / "input_hashes.json", inputs)
    states = load_pickle(source_probe / "episodes_pre_gae.pkl")
    torch.set_num_threads(1)
    control = run_branch(run, source_probe, output, "control", states, manifest)
    verification = verify_control(source_probe, output / "control/ppo_audit")
    write_json(output / "control_verification.json", verification)
    if not verification["pass"]:
        raise RuntimeError("Stock replay is not bit-exact; corrected branch was NOT run")
    corrected = run_branch(run, source_probe, output, "corrected_boundary", states, manifest)
    # Every non-target batch column must remain bit-identical across branches.
    left = load_pickle(output / "control/ppo_audit/batch_post_gae.pkl")["default_policy"]
    right = load_pickle(output / "corrected_boundary/ppo_audit/batch_post_gae.pkl")["default_policy"]
    unchanged = {key: not compare_trees(left[key], right[key], key)
                 for key in left if key not in ("advantages", "value_targets")}
    if not all(unchanged.values()):
        raise RuntimeError(f"Correction changed an unrelated batch column: {unchanged}")
    changed_inputs = [name for name, expected in inputs.items() if digest(name) != expected]
    if changed_inputs:
        raise RuntimeError(f"Replay inputs changed: {changed_inputs}")
    receipt = {"complete": True, "control": control, "corrected_boundary": corrected,
               "control_reproduction": verification, "unchanged_batch_columns": unchanged,
               "all_source_inputs_unchanged": True, "ray_initialized": ray.is_initialized(),
               "fresh_sampling": False, "physical_simulations": False,
               "limitation": "One batch A/B; corrected actor still needs independent physical evaluation. No global stability or causal sufficiency claim."}
    write_json(output / "replay_complete.json", receipt)
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--output", type=Path, default=HERE / "boundary_replay")
    args = parser.parse_args()
    result = run_replay(args.run, args.output)
    print(json.dumps(clean(result), indent=2))


if __name__ == "__main__":
    main()
