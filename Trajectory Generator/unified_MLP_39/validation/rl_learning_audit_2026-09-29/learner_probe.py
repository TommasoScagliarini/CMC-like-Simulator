"""Isolated observation-only hooks for the local RLlib 2.55 PPO learner.

Call ``attach_probe(learner, directory)`` after checkpoint restore and before one
Learner.update. Call ``detach_probe`` afterwards. The original methods perform
every forward, loss, gradient and optimizer operation. The probe only copies
their inputs/outputs; it never samples a distribution or runs another forward.
No Ray initialization or simulator imports occur here.

Artifacts use pickle/torch serialization and must only be loaded as trusted
local files. Full episode states include infos and the boundary observation.
"""
from __future__ import annotations

import hashlib
import json
import math
import pickle
import random
import types
from pathlib import Path
from typing import Any

import numpy as np
import torch


def _copy(value):
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().clone()
    if isinstance(value, np.ndarray):
        return value.copy()
    if isinstance(value, dict):
        return {key: _copy(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return type(value)(_copy(item) for item in value)
    return value


def _array(value):
    if isinstance(value, torch.Tensor):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def _jsonable(value):
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (np.ndarray, torch.Tensor)):
        return _jsonable(_array(value).tolist())
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    return value


def _write_json(path, value):
    Path(path).write_text(json.dumps(_jsonable(value), indent=2, allow_nan=False) + "\n")


def _dump(path, value):
    with Path(path).open("wb") as stream:
        pickle.dump(_copy(value), stream, protocol=pickle.HIGHEST_PROTOCOL)


def _rng_state():
    state = {"python": random.getstate(), "numpy": np.random.get_state(),
             "torch_cpu": torch.get_rng_state().clone()}
    # The project learner is CPU-only. Do not initialize a CUDA runtime here.
    if torch.cuda.is_initialized():
        state["torch_cuda"] = [x.clone() for x in torch.cuda.get_rng_state_all()]
    return state


def _rng_hash(state):
    h = hashlib.sha256()
    h.update(pickle.dumps(state["python"], protocol=4))
    ns = state["numpy"]
    h.update(pickle.dumps((ns[0], ns[2:]), protocol=4))
    h.update(ns[1].tobytes())
    h.update(state["torch_cpu"].numpy().tobytes())
    for item in state.get("torch_cuda", []):
        h.update(item.cpu().numpy().tobytes())
    return h.hexdigest()


def _stats(values):
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    finite = values[np.isfinite(values)]
    result = {"count": int(values.size), "nonfinite_count": int(values.size - finite.size)}
    if finite.size:
        result.update(mean=float(finite.mean()), std=float(finite.std()),
                      min=float(finite.min()), max=float(finite.max()),
                      p05=float(np.quantile(finite, .05)), p50=float(np.quantile(finite, .5)),
                      p95=float(np.quantile(finite, .95)))
    return result


def _mask(batch):
    n = len(batch["actions"])
    mask = _array(batch.get("loss_mask", np.ones(n, dtype=bool))).astype(bool)
    if mask.shape != (n,):
        raise ValueError("Probe supports flat, non-recurrent batches only")
    return mask


def _max_error(a, b):
    return float(np.max(np.abs(np.asarray(a) - np.asarray(b)))) if np.size(a) else 0.0


def _gae_targets(values, rewards, terminateds, truncateds, gamma, lam):
    """Independent recurrence matching RLlib's elongated-episode convention."""
    values = np.asarray(values)
    v = values * (1.0 - np.asarray(terminateds))
    next_v = np.r_[v[1:], 0.0]
    out = np.empty(len(v), dtype=np.float32)
    carry = 0.0
    for index in range(len(v) - 1, -1, -1):
        carry = (rewards[index] + gamma * (1.0 - lam) * next_v[index]
                 + (1.0 - terminateds[index]) * gamma * lam * carry)
        out[index] = carry
        if truncateds[index]:
            carry = v[index]
    return out


def batch_diagnostics(batch, *, gamma, lam, gae_values=None, gae_batch=None):
    """No policy forward: audit stored Gaussian likelihood and GAE bookkeeping."""
    mask = _mask(batch)
    actions = _array(batch["actions"])
    logits = _array(batch["action_dist_inputs"])
    logp = _array(batch["action_logp"])
    if logits.ndim != 2 or logits.shape[1] != 2 * actions.shape[1]:
        raise ValueError("Expected two-action diagonal Gaussian logits")
    mean, logstd = np.split(logits.astype(np.float64), 2, axis=-1)
    evaluated_logp = (-.5 * ((actions - mean) / np.exp(logstd)) ** 2
                     - logstd - .5 * np.log(2 * np.pi)).sum(axis=-1)
    clipped_logp = (-.5 * ((np.clip(actions, -1, 1) - mean) / np.exp(logstd)) ** 2
                   - logstd - .5 * np.log(2 * np.pi)).sum(axis=-1)
    result = {
        "rows_total": int(len(mask)), "rows_real": int(mask.sum()),
        "rows_masked": int((~mask).sum()), "observation_shape": list(batch["obs"].shape),
        "columns": sorted(batch),
        "stored_logp_vs_raw_gaussian_max_abs_error": _max_error(evaluated_logp[mask], logp[mask]),
        "stored_logp_vs_clipped_gaussian_max_abs_error": _max_error(clipped_logp[mask], logp[mask]),
        "sampled_action_clip_fraction_any_joint": float(np.any(np.abs(actions[mask]) > 1, axis=-1).mean()),
        "sampled_action_clip_fraction_per_joint": (np.abs(actions[mask]) > 1).mean(axis=0),
        "advantages_actual_real": _stats(_array(batch["advantages"])[mask]),
        "value_targets_actual_real": _stats(_array(batch["value_targets"])[mask]),
        "gaussian_diagnostic_assumption": "Unsquashed diagonal Gaussian; verify action distribution class in manifest.",
    }
    if gae_values is not None and gae_batch is not None:
        values = _array(gae_values).reshape(-1)
        source = gae_batch
        targets = _gae_targets(values, _array(source["rewards"]),
                               _array(source["terminateds"]), _array(source["truncateds"]),
                               gamma, lam)
        raw_adv = targets - values
        normalized = (raw_adv - raw_adv.mean()) / max(1e-4, raw_adv.std())
        result["gae_recomputed"] = {
            "rows": len(values), "raw_advantages_all": _stats(raw_adv),
            "normalization_includes_artificial_rows": True,
            "gamma": float(gamma), "lambda": float(lam),
        }
        # The stock learner retains artificial rows. Custom compaction can alter
        # their order/count; never claim an aligned equality in that situation.
        aligned = (len(values) == len(mask)
                   and np.array_equal(_array(source["obs"]), _array(batch["obs"])))
        result["gae_recomputed"]["post_batch_order_identical"] = aligned
        if aligned:
            result["gae_recomputed"].update(
                targets_max_abs_error=_max_error(targets, _array(batch["value_targets"])),
                advantages_max_abs_error=_max_error(normalized, _array(batch["advantages"])),
                raw_advantages_real=_stats(raw_adv[mask]),
                bootstrap_values=_stats(values[~mask]),
            )
    return result


class LearnerProbe:
    def __init__(self, learner, directory, *, snapshot_minibatches=(8,)):
        self.learner = learner
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        if (self.directory / "manifest.json").exists():
            raise FileExistsError(f"Probe output already exists: {self.directory}")
        self.snapshot_minibatches = set(snapshot_minibatches)
        self.originals = []
        self.context = None
        self.gae_values = {}
        self.gae_batches = {}
        self.loss_values = {}
        self.minibatch_index = 0
        self.rows_seen = {}
        self.total_rows = {}
        self.first_pass_saved = False
        self.update_calls = 0
        self.records = []
        self.manifest = {
            "schema": "rl_learning_audit_probe_v1", "directory": str(self.directory),
            "observation_only": True, "extra_policy_forwards": 0,
            "episode_capture": "SingleAgentEpisode.get_state before learner connectors, including full infos and final observation",
            "batch_capture": "Post-connector, pre-minibatch-shuffle batch; artificial rows preserved",
            "snapshots": {}, "complete": False,
        }

    def _patch(self, obj, name, callback):
        had_local = name in obj.__dict__
        previous_local = obj.__dict__.get(name)
        original = getattr(obj, name)
        self.originals.append((obj, name, had_local, previous_local))
        def bound(_obj, *args, **kwargs):
            return callback(original, *args, **kwargs)
        setattr(obj, name, types.MethodType(bound, obj))

    def _snapshot(self, name, **metadata):
        before = _rng_hash(_rng_state())
        path = self.directory / "snapshots" / name
        path.mkdir(parents=True, exist_ok=False)
        learner = self.learner
        _dump(path / "learner_state.pkl", learner.get_state())
        _dump(path / "rng_state.pkl", _rng_state())
        scalars = {"kl_coeff": {mid: float(value.detach().cpu())
                                  for mid, value in learner.curr_kl_coeffs_per_module.items()},
                   "optimizer_lrs": {name: [float(group["lr"]) for group in optimizer.param_groups]
                                     for name, optimizer in learner._named_optimizers.items()}}
        if hasattr(learner, "entropy_coeff_schedulers_per_module"):
            scalars["entropy_coeff"] = {
                mid: float(scheduler.get_current_value())
                for mid, scheduler in learner.entropy_coeff_schedulers_per_module.items()}
        for mid, module in learner.module.items():
            module.save_to_path(path / f"rl_module_{mid}")
        self.manifest["snapshots"][name] = {
            "path": str(path), "minibatches_completed": self.minibatch_index,
            "rows_seen": dict(self.rows_seen), "scalars": scalars, **metadata,
        }
        _write_json(path / "metadata.json", self.manifest["snapshots"][name])
        if before != _rng_hash(_rng_state()):
            raise RuntimeError("Snapshot unexpectedly changed an RNG state")

    def install(self):
        for mid, module in self.learner.module.items():
            if module.is_stateful():
                raise ValueError("This probe is restricted to the feedforward MLP")
            self.manifest.setdefault("modules", {})[mid] = {
                "class": type(module).__name__,
                "train_distribution": module.get_train_action_dist_cls().__name__,
                "exploration_distribution": module.get_exploration_action_dist_cls().__name__,
            }
            def capture_values(original, *args, _mid=mid, **kwargs):
                result = original(*args, **kwargs)
                if self.context == "batch_build":
                    if _mid in self.gae_values:
                        raise RuntimeError("Unexpected multiple value forwards during batch construction")
                    batch = args[0] if args else kwargs["batch"]
                    self.gae_values[_mid] = _copy(result)
                    self.gae_batches[_mid] = _copy(batch)
                elif self.context == "minibatch":
                    self.loss_values[_mid] = _copy(result)
                return result
            self._patch(module, "compute_values", capture_values)
        self._patch(self.learner, "_make_batch_if_necessary", self._make_batch)
        self._patch(self.learner, "compute_loss_for_module", self._compute_loss)
        self._patch(self.learner, "_update", self._update)
        self._patch(self.learner, "update", self._whole_update)
        _write_json(self.directory / "manifest.json", self.manifest)
        return self

    def _make_batch(self, original, training_data, **kwargs):
        if self.total_rows:
            raise RuntimeError("Probe captures exactly one batch; detach before another update")
        episodes = training_data.episodes
        if episodes is not None:
            _dump(self.directory / "episodes_pre_gae.pkl", [episode.get_state() for episode in episodes])
            rows = [{"id": episode.id_, "length": len(episode), "t": episode.t,
                     "t_started": episode.t_started, "terminated": bool(episode.is_terminated),
                     "truncated": bool(episode.is_truncated), "module_id": episode.module_id}
                    for episode in episodes]
            _write_json(self.directory / "episodes_pre_gae.json", rows)
        self.context = "batch_build"
        try:
            batch = original(training_data=training_data, **kwargs)
        finally:
            self.context = None
        _dump(self.directory / "batch_post_gae.pkl", dict(batch.policy_batches))
        _dump(self.directory / "gae_values_and_input.pkl", {"values": self.gae_values, "batches": self.gae_batches})
        diagnostics = {}
        for mid, module_batch in batch.policy_batches.items():
            self.total_rows[mid] = len(module_batch["actions"])
            self.rows_seen[mid] = 0
            cfg = self.learner.config.get_config_for_module(mid)
            diagnostics[mid] = batch_diagnostics(module_batch, gamma=cfg.gamma, lam=cfg.lambda_,
                                                 gae_values=self.gae_values.get(mid), gae_batch=self.gae_batches.get(mid))
        self.manifest["batch_diagnostics"] = diagnostics
        _write_json(self.directory / "batch_diagnostics.json", diagnostics)
        return batch

    def _compute_loss(self, original, *, module_id, config, batch, fwd_out):
        result = original(module_id=module_id, config=config, batch=batch, fwd_out=fwd_out)
        # Distributions are constructed from existing logits, with no sampling.
        # All probe calculations are detached from the optimization graph.
        before = _rng_hash(_rng_state())
        with torch.no_grad():
            module = self.learner.module[module_id].unwrapped()
            current = module.get_train_action_dist_cls().from_logits(fwd_out["action_dist_inputs"].detach())
            previous = module.get_exploration_action_dist_cls().from_logits(batch["action_dist_inputs"].detach())
            logp = current.logp(batch["actions"])
            ratio = torch.exp(logp - batch["action_logp"])
            kl = previous.kl(current)
            mask = _mask(batch)
            ratio_np = _array(ratio)[mask]
            advantages = _array(batch["advantages"])[mask]
            values = self.loss_values.get(module_id)
            if config.use_critic and values is None:
                raise RuntimeError("Original PPO loss did not expose its value computation")
            record = {
                "minibatch": self.minibatch_index + 1, "module_id": module_id,
                "rows_total": len(mask), "rows_real": int(mask.sum()),
                "ratio": _stats(ratio_np), "ratio_max_abs_deviation_from_one": _max_error(ratio_np, 1),
                "kl": _stats(_array(kl)[mask]),
                "ratio_outside_clip_fraction": float(np.mean(np.abs(ratio_np - 1) > config.clip_param)),
                "surrogate_actually_clipped_fraction": float(np.mean(((advantages > 0) & (ratio_np > 1 + config.clip_param))
                                                                     | ((advantages < 0) & (ratio_np < 1 - config.clip_param)))),
                "kl_coeff_before_update": float(self.learner.curr_kl_coeffs_per_module[module_id].detach().cpu()),
                "original_total_loss": float(result.detach().cpu()),
                "optimizer_lrs": {name: [float(group["lr"]) for group in optimizer.param_groups]
                                  for name, optimizer in self.learner._named_optimizers.items()},
            }
            if values is not None:
                error = (_array(values) - _array(batch["value_targets"]))[mask]
                sq_error = error ** 2
                record.update(vf_error=_stats(error), vf_squared_error=_stats(sq_error),
                              vf_capped_loss=_stats(np.minimum(sq_error, config.vf_clip_param)),
                              vf_capped_fraction=float(np.mean(sq_error >= config.vf_clip_param)))
            artifact = {"batch": dict(batch), "current_logits_before_update": fwd_out["action_dist_inputs"],
                        "current_values_before_update": values, "ratio": ratio, "kl": kl}
            minibatches = self.directory / "minibatches"
            minibatches.mkdir(exist_ok=True)
            _dump(minibatches / f"{self.minibatch_index + 1:06d}_{module_id}.pkl", artifact)
            self.records.append(record)
            with (self.directory / "minibatch_metrics.jsonl").open("a") as stream:
                stream.write(json.dumps(_jsonable(record), allow_nan=False) + "\n")
        if before != _rng_hash(_rng_state()):
            raise RuntimeError("Loss observation unexpectedly changed RNG state")
        return result

    def _update(self, original, batch, **kwargs):
        parameters_before = {
            mid: {name: parameter.detach().cpu().clone()
                  for name, parameter in self.learner.module[mid].named_parameters()}
            for mid in batch}
        self.context = "minibatch"
        self.loss_values.clear()
        try:
            result = original(batch, **kwargs)
        finally:
            self.context = None
        self.minibatch_index += 1
        step_metrics = {"minibatch": self.minibatch_index, "modules": {},
                        "gradient_semantics": "Parameter gradients remaining after original gradient postprocessing and optimizer step"}
        for mid, module_batch in batch.items():
            self.rows_seen[mid] += len(module_batch["actions"])
            rows = {}
            for name, parameter in self.learner.module[mid].named_parameters():
                delta = parameter.detach().cpu() - parameters_before[mid][name]
                grad = parameter.grad
                rows[name] = {"update_l2": float(torch.linalg.vector_norm(delta)),
                              "update_max_abs": float(delta.abs().max()),
                              "gradient_l2": float(torch.linalg.vector_norm(grad.detach())) if grad is not None else None,
                              "gradient_nonfinite_count": int((~torch.isfinite(grad.detach())).sum()) if grad is not None else None}
            step_metrics["modules"][mid] = rows
        with (self.directory / "optimizer_step_metrics.jsonl").open("a") as stream:
            stream.write(json.dumps(_jsonable(step_metrics), allow_nan=False) + "\n")
        if self.minibatch_index in self.snapshot_minibatches:
            self._snapshot(f"minibatch_{self.minibatch_index:06d}")
        if not self.first_pass_saved and all(self.rows_seen[mid] >= n for mid, n in self.total_rows.items()):
            self._snapshot("first_pass", exact_epoch_boundary=all(self.rows_seen[mid] == n for mid, n in self.total_rows.items()),
                           note="First optimizer step after all rows have been consumed; may include rows of the next pass")
            self.first_pass_saved = True
        return result

    def _whole_update(self, original, *args, **kwargs):
        self.update_calls += 1
        if self.update_calls != 1:
            raise RuntimeError("Detach the single-update probe before any subsequent update")
        self.manifest["update_arguments"] = {
            key: value for key, value in kwargs.items()
            if key in ("num_epochs", "minibatch_size", "shuffle_batch_per_epoch", "num_total_minibatches", "timesteps")}
        self._snapshot("before")
        result = original(*args, **kwargs)
        self._snapshot("final", note="After original PPO after_gradient_based_update, including normal adaptive KL update")
        self.manifest.update(complete=True, minibatches_completed=self.minibatch_index,
                             total_rows=self.total_rows, rows_seen=self.rows_seen,
                             extra_policy_forwards=0)
        _write_json(self.directory / "manifest.json", self.manifest)
        self._hash_artifacts()
        return result

    def _hash_artifacts(self):
        hashes = {str(path.relative_to(self.directory)): hashlib.sha256(path.read_bytes()).hexdigest()
                  for path in sorted(self.directory.rglob("*"))
                  if path.is_file() and path.name != "artifact_hashes.json"}
        _write_json(self.directory / "artifact_hashes.json", hashes)

    def detach(self):
        for obj, name, had_local, previous_local in reversed(self.originals):
            if had_local:
                setattr(obj, name, previous_local)
            else:
                delattr(obj, name)
        self.originals.clear()
        _write_json(self.directory / "manifest.json", self.manifest)
        self._hash_artifacts()
        return self.manifest


def attach_probe(learner, directory, *, snapshot_minibatches=(8,)):
    """Attach to one local, already-built learner; return small serializable metadata."""
    if hasattr(learner, "_rl_learning_audit_probe"):
        raise RuntimeError("Learner already has a probe")
    probe = LearnerProbe(learner, directory, snapshot_minibatches=snapshot_minibatches)
    try:
        probe.install()
    except Exception:
        probe.detach()
        raise
    learner._rl_learning_audit_probe = probe
    return {"directory": str(probe.directory), "attached": True}


def detach_probe(learner):
    """Restore original bound methods and finalize hashes, including on failure."""
    probe = learner._rl_learning_audit_probe
    result = probe.detach()
    del learner._rl_learning_audit_probe
    return result
