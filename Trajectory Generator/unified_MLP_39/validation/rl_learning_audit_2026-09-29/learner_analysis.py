"""Offline audit of the frozen, single-batch PPO probe. Never runs a simulator.

Usage: python learner_analysis.py [--probe PATH] [--heldout NPZ_OR_DIRECTORY ...]
The probe manifest must be complete and its artifact hashes must validate.
Only trusted local pickle/checkpoint artifacts may be supplied.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import pickle
import sys

HERE = Path(__file__).resolve().parent
REPO = next(path for path in HERE.parents if (path / "AGENTS.md").exists())
TG = REPO / "Trajectory Generator"
DEFAULT_PROBE = TG / "unified_MLP_39/runs/training/MLP39_LEFT_best80_rl_audit_2026-09-29/ppo_audit"
DEFAULT_OUTPUT = HERE / "learner_analysis"
DEFAULT_PLOTS = REPO / "plot/09_29_2026_rl_learning_audit/learner"
sys.path.insert(0, str(TG))
sys.dont_write_bytecode = True
os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".matplotlib"))
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[key] = "1"

import numpy as np
import torch


def array(value):
    return value.detach().cpu().numpy() if isinstance(value, torch.Tensor) else np.asarray(value)


def clean(value):
    if isinstance(value, dict):
        return {str(key): clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    if isinstance(value, (np.ndarray, torch.Tensor)):
        return clean(array(value).tolist())
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(clean(value), indent=2, allow_nan=False) + "\n")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_pickle(path):
    with Path(path).open("rb") as stream:
        return pickle.load(stream)


def stats(values):
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    finite = values[np.isfinite(values)]
    result = {"count": len(values), "nonfinite_count": int(len(values) - len(finite))}
    if len(finite):
        result.update(mean=float(finite.mean()), std=float(finite.std()), min=float(finite.min()),
                      max=float(finite.max()), p05=float(np.quantile(finite, .05)),
                      p50=float(np.quantile(finite, .5)), p95=float(np.quantile(finite, .95)))
    return result


def max_error(a, b):
    return float(np.max(np.abs(np.asarray(a) - np.asarray(b)))) if np.size(a) else 0.


def correlation(a, b):
    a, b = np.asarray(a), np.asarray(b)
    if len(a) < 2 or a.std() == 0 or b.std() == 0:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def explained_variance(targets, values):
    variance = np.var(targets)
    return float(1 - np.var(targets - values) / variance) if variance > 0 else None


def critic_metrics(values, targets, vf_cap):
    error = np.asarray(values) - np.asarray(targets)
    sq_error = error ** 2
    return {"error": stats(error), "squared_error": stats(sq_error),
            "rmse": float(np.sqrt(np.mean(sq_error))),
            "capped_mse": float(np.mean(np.minimum(sq_error, vf_cap))),
            "cap_fraction": float(np.mean(sq_error >= vf_cap)),
            "explained_variance": explained_variance(targets, values),
            "target_variance": float(np.var(targets))}


def infer(module, observations, chunk_size=512):
    logits, values = [], []
    with torch.no_grad():
        for start in range(0, len(observations), chunk_size):
            obs = torch.as_tensor(observations[start:start + chunk_size], dtype=torch.float32)
            batch = {"obs": obs}
            logits.append(array(module.forward_inference(batch)["action_dist_inputs"]))
            values.append(array(module.compute_values(batch)))
    return np.concatenate(logits), np.concatenate(values)


def distribution_outputs(module, logits, old_logits, actions, old_logp):
    with torch.no_grad():
        current = module.get_train_action_dist_cls().from_logits(torch.as_tensor(logits))
        previous = module.get_exploration_action_dist_cls().from_logits(torch.as_tensor(old_logits))
        logp = array(current.logp(torch.as_tensor(actions)))
        return {"logp": logp, "logp_delta": logp - old_logp,
                "ratio": np.exp(logp - old_logp), "kl": array(previous.kl(current)),
                "entropy": array(current.entropy())}


def snapshot_metrics(module, logits, values, batch, mask, config, *, kl_coeff, entropy_coeff):
    old_logits, actions = array(batch["action_dist_inputs"]), array(batch["actions"])
    outputs = distribution_outputs(module, logits, old_logits, actions, array(batch["action_logp"]))
    advantages = array(batch["advantages"])[mask]
    ratio = outputs["ratio"][mask]
    clip = config["clip_param"]
    unclipped = advantages * ratio
    clipped = advantages * np.clip(ratio, 1 - clip, 1 + clip)
    surrogate = np.minimum(unclipped, clipped)
    vf = critic_metrics(values[mask], array(batch["value_targets"])[mask], config["vf_clip_param"])
    mean, logstd = np.split(logits, 2, axis=-1)
    old_mean, old_logstd = np.split(old_logits, 2, axis=-1)
    delta = mean - old_mean
    applied_delta = np.clip(mean, -1, 1) - np.clip(old_mean, -1, 1)
    policy_loss = float(-surrogate.mean())
    kl = float(outputs["kl"][mask].mean())
    entropy = float(outputs["entropy"][mask].mean())
    signed = np.sign(advantages) * outputs["logp_delta"][mask]
    nonzero = np.abs(advantages) > 1e-8
    metrics = {
        "rows_real": int(mask.sum()), "ratio": stats(ratio),
        "logp_delta": stats(outputs["logp_delta"][mask]),
        "initial_ratio_max_abs_deviation_from_one": max_error(ratio, 1.),
        "kl_old_to_current": stats(outputs["kl"][mask]), "entropy": stats(outputs["entropy"][mask]),
        "ratio_outside_clip_fraction": float(np.mean(np.abs(ratio - 1) > clip)),
        "surrogate_actually_clipped_fraction": float(np.mean(((advantages > 0) & (ratio > 1 + clip))
                                                              | ((advantages < 0) & (ratio < 1 - clip)))),
        "policy_loss_fixed_advantages": policy_loss,
        "unclipped_surrogate_mean": float(unclipped.mean()),
        "critic_fixed_gae_targets": vf,
        "total_objective_fixed_coefficients": (policy_loss + config["vf_loss_coeff"] * vf["capped_mse"]
                                                + kl_coeff * kl - entropy_coeff * entropy),
        "objective_coefficients": {"kl": kl_coeff, "entropy": entropy_coeff,
                                   "vf_loss": config["vf_loss_coeff"], "vf_cap": config["vf_clip_param"]},
        "advantage_probability_alignment": {
            "mean_advantage_times_delta_logp": float(np.mean(advantages * outputs["logp_delta"][mask])),
            "correlation_advantage_delta_logp": correlation(advantages, outputs["logp_delta"][mask]),
            "fraction_matching_sign_nonzero_advantages": float(np.mean(signed[nonzero] > 0)) if np.any(nonzero) else None,
            "positive_advantage_logp_delta": stats(outputs["logp_delta"][mask][advantages > 0]),
            "negative_advantage_logp_delta": stats(outputs["logp_delta"][mask][advantages < 0]),
        },
        "actor_same_observations": {
            "mean_delta_rms_per_joint": np.sqrt(np.mean(delta[mask] ** 2, axis=0)),
            "mean_delta_max_abs_per_joint": np.max(np.abs(delta[mask]), axis=0),
            "deterministic_clipped_mean_delta_rms_per_joint": np.sqrt(np.mean(applied_delta[mask] ** 2, axis=0)),
            "mean_delta_over_old_sigma_rms_per_joint": np.sqrt(np.mean((delta[mask] / np.exp(old_logstd[mask])) ** 2, axis=0)),
            "sigma_mean_per_joint": np.exp(logstd[mask]).mean(axis=0),
            "sigma_ratio_to_old_mean_per_joint": np.exp(logstd[mask] - old_logstd[mask]).mean(axis=0),
            "logstd_delta_max_abs_per_joint": np.max(np.abs(logstd[mask] - old_logstd[mask]), axis=0),
            "joint_order": ["knee", "ankle"], "action_units": "normalized; deterministic clipping is a same-state diagnostic",
        },
    }
    return metrics, outputs


def reconstruct_boundaries(states, batch, gae_values, gamma, lam):
    """Independent ordinary TD/GAE recursion on each original real fragment."""
    from ray.rllib.env.single_agent_episode import SingleAgentEpisode
    episodes = [SingleAgentEpisode.from_state(state) for state in states]
    mask = array(batch["loss_mask"]).astype(bool)
    obs = array(batch["obs"])
    rewards = array(batch["rewards"])
    expected_obs = np.concatenate([np.asarray(episode.get_observations()) for episode in episodes])
    order_error = max_error(obs, expected_obs) if obs.shape == expected_obs.shape else None
    aligned = obs.shape == expected_obs.shape and np.array_equal(obs, expected_obs)
    result = {"episode_chunks": len(episodes), "original_transition_count": sum(len(e) for e in episodes),
              "terminated_chunks": sum(bool(e.is_terminated) for e in episodes),
              "truncated_chunks": sum(bool(e.is_truncated) for e in episodes),
              "unfinished_fragments": sum(not (e.is_terminated or e.is_truncated) for e in episodes),
              "chunks_starting_at_episode_reset": sum(e.t_started == 0 for e in episodes),
              "complete_episodes_in_batch": sum(e.t_started == 0 and (e.is_terminated or e.is_truncated) for e in episodes),
              "full_true_terminal_episodes_in_batch": sum(e.t_started == 0 and e.is_terminated for e in episodes),
              "expected_observation_shape": list(expected_obs.shape), "batch_observation_shape": list(obs.shape),
              "episode_to_batch_order_exact": aligned, "observation_max_abs_error": order_error,
              "normalization_note": "Stock GAE standardizes all elongated rows, then PPO loss masks bootstrap rows.",
              "return_note": "An unfinished/truncated fragment supports bootstrapped targets, not an observed infinite-horizon return."}
    if not aligned:
        return result, [], None
    rows = []
    independent_targets = np.full(len(mask), np.nan)
    raw_advantages = np.full(len(mask), np.nan)
    position = 0
    for episode in episodes:
        n = len(episode)
        start, boundary = position, position + n
        rv = np.asarray(episode.get_rewards(), dtype=np.float64)
        values = np.asarray(gae_values[start:boundary + 1], dtype=np.float64)
        bootstrap = 0. if episode.is_terminated else values[-1]
        advantage = 0.
        targets = np.empty(n)
        for step in range(n - 1, -1, -1):
            terminal = bool(episode.is_terminated and step == n - 1)
            next_value = bootstrap if step == n - 1 else values[step + 1]
            delta = rv[step] + gamma * (not terminal) * next_value - values[step]
            advantage = delta + gamma * lam * (not terminal) * advantage
            raw_advantages[start + step] = advantage
            targets[step] = advantage + values[step]
        independent_targets[start:boundary] = targets
        infos = episode.get_infos()
        last = infos[-1] if len(infos) and isinstance(infos[-1], dict) else {}
        rows.append({"episode_id": episode.id_, "episode_t_start": episode.t_started,
                     "episode_t_end": episode.t, "real_rows": n, "batch_row_start": start,
                     "bootstrap_row": boundary, "terminated": bool(episode.is_terminated),
                     "truncated": bool(episode.is_truncated),
                     "unfinished_fragment": not (episode.is_terminated or episode.is_truncated),
                     "bootstrap_value_predicted": values[-1], "independent_bootstrap_value_used": bootstrap,
                     "stock_bootstrap_value_used": float(values[-1] * (not bool(array(batch["terminateds"])[boundary]))),
                     "boundary_terminated_column": bool(array(batch["terminateds"])[boundary]),
                     "boundary_truncated_column": bool(array(batch["truncateds"])[boundary]),
                     "mask_correct": bool(np.all(mask[start:boundary]) and not mask[boundary]),
                     "raw_reward_max_abs_error": max_error(rewards[start:boundary], rv),
                     "independent_target_max_abs_error": max_error(array(batch["value_targets"])[start:boundary], targets),
                     "end_reason": last.get("end_reason"),
                     "final_segment_completed": last.get("segment_completed"),
                     "final_boundary_time_s": last.get("time")})
        position = boundary + 1
    result.update(mask_boundaries_all_correct=all(row["mask_correct"] for row in rows),
                  independent_real_target_max_abs_error=max_error(independent_targets[mask], array(batch["value_targets"])[mask]),
                  independent_raw_advantages=stats(raw_advantages[mask]))
    normalized_real = (raw_advantages[mask] - np.mean(raw_advantages[mask])) / max(1e-4, np.std(raw_advantages[mask]))
    result["actual_vs_independent_targets_and_real_normalization_max_abs_difference"] = max_error(normalized_real, array(batch["advantages"])[mask])
    stock_raw = array(batch["value_targets"])[mask] - np.asarray(gae_values)[mask]
    stock_real_normalized = (stock_raw - stock_raw.mean()) / max(1e-4, stock_raw.std())
    result["actual_vs_stock_targets_real_rows_only_normalization_max_abs_difference"] = max_error(stock_real_normalized, array(batch["advantages"])[mask])
    result["same_critic_vs_independent_real_targets"] = critic_metrics(np.asarray(gae_values)[mask], independent_targets[mask], 10.)
    difference = independent_targets[mask] - array(batch["value_targets"])[mask]
    result["independent_minus_stock_real_targets"] = stats(difference)
    result["target_difference_fraction_above_1"] = float(np.mean(np.abs(difference) > 1.))
    return result, rows, independent_targets


def discounted_returns(rewards, gamma, bootstrap=0.):
    result = np.empty(len(rewards), dtype=np.float64)
    value = float(bootstrap)
    for index in range(len(rewards) - 1, -1, -1):
        value = float(rewards[index]) + gamma * value
        result[index] = value
    return result


def native_pipeline_bootstrap_reproducer(gamma=.99, lam=.9):
    """Two synthetic transitions through the installed full default pipeline.

    The critic returns observation[0], so no neural fit, environment or random
    sampling is involved. This checks the composition of the actual connectors.
    """
    import gymnasium as gym
    from ray.rllib.algorithms.ppo import PPOConfig
    from ray.rllib.connectors.learner import AddOneTsToEpisodesAndTruncate, GeneralAdvantageEstimation
    from ray.rllib.core.rl_module.apis.value_function_api import ValueFunctionAPI
    from ray.rllib.core.rl_module.multi_rl_module import MultiRLModule
    from ray.rllib.core.rl_module.rl_module import RLModuleSpec
    from ray.rllib.core.rl_module.torch.torch_rl_module import TorchRLModule
    from ray.rllib.env.single_agent_episode import SingleAgentEpisode

    class KnownValues(TorchRLModule, ValueFunctionAPI):
        def setup(self):
            pass

        def compute_values(self, batch, embeddings=None):
            return batch["obs"][..., 0]

    obs_space = gym.spaces.Box(-100., 100., (1,), dtype=np.float32)
    action_space = gym.spaces.Box(-1., 1., (1,), dtype=np.float32)
    module = MultiRLModule(rl_module_specs={"default_policy": RLModuleSpec(
        module_class=KnownValues, observation_space=obs_space, action_space=action_space, model_config={})})
    results = {}
    for name, terminal, truncated in (("unfinished_fragment", False, False),
                                       ("time_truncation", False, True), ("true_termination", True, False)):
        episode = SingleAgentEpisode(
            id_=name, observation_space=obs_space, action_space=action_space,
            observations=[np.array([value], dtype=np.float32) for value in (2., 3., 4.)],
            actions=[np.array([0.], dtype=np.float32)] * 2, rewards=[1., 2.], infos=[{}, {}, {}],
            terminated=terminal, truncated=truncated, len_lookback_buffer=0)
        pipeline = PPOConfig().build_learner_connector(obs_space, action_space, device="cpu")
        pipeline.prepend(AddOneTsToEpisodesAndTruncate())
        pipeline.append(GeneralAdvantageEstimation(gamma=gamma, lambda_=lam))
        batch = pipeline(rl_module=module, batch={}, episodes=[episode], shared_data={})["default_policy"]
        expected_last = 2. if terminal else 2. + gamma * 4.
        expected_first = 1. + gamma * ((1 - lam) * 3. + lam * expected_last)
        results[name] = {
            "known_values_at_observations": [2., 3., 4.], "rewards": [1., 2.],
            "stock_targets_real": array(batch["value_targets"])[:2],
            "ordinary_td_lambda_targets_real": [expected_first, expected_last],
            "target_max_abs_difference": max_error(array(batch["value_targets"])[:2], [expected_first, expected_last]),
            "terminateds_after_connectors": array(batch["terminateds"]),
            "truncateds_after_connectors": array(batch["truncateds"]),
            "loss_mask": array(batch["loss_mask"]), "no_training_or_simulation": True}
    return results


def analyze_heldout(paths, modules, config):
    results = []
    for path in paths:
        path = Path(path)
        if path.is_dir():
            path = path / "critic_transitions.npz"
        if not path.exists():
            results.append({"path": str(path), "available": False})
            continue
        if not (path.parent / "evaluation_complete.json").exists():
            results.append({"path": str(path), "available": False, "reason": "evaluation_complete receipt absent"})
            continue
        receipt = json.loads((path.parent / "evaluation_complete.json").read_text())
        expected = receipt.get("artifact_hashes", {}).get(path.name)
        if expected is None or digest(path) != expected:
            raise ValueError(f"Heldout transition hash not verified by completion receipt: {path}")
        data = dict(np.load(path))
        obs, next_obs = data["observation"], data["next_observation"]
        reward = np.asarray(data["reward"], dtype=float)
        terminated = np.asarray(data["terminated"], dtype=bool)
        truncated = np.asarray(data["truncated"], dtype=bool)
        if len(reward) == 0 or np.any(terminated[:-1] | truncated[:-1]):
            raise ValueError("Heldout NPZ must contain one nonempty uninterrupted episode or fragment")
        values, next_values = {}, {}
        for name, module in modules.items():
            _, values[name] = infer(module, obs)
            _, next_values[name] = infer(module, next_obs)
        terminal = bool(terminated[-1])
        finite = discounted_returns(reward, config["gamma"], 0.)
        bootstrap = 0. if terminal else float(next_values["before"][-1])
        fixed_bootstrap_return = discounted_returns(reward, config["gamma"], bootstrap)
        fixed_td_target = reward + config["gamma"] * (~terminated) * next_values["before"]
        common = {"path": str(path.resolve()), "available": True, "rows": len(reward),
                  "completion_receipt_hash_verified": True, "undiscounted_reward_sum": float(reward.sum()),
                  "completed_segment_rows": int(np.asarray(data.get("segment_completed", np.ones(len(reward))), dtype=bool).sum()),
                  "last_terminated": terminal, "last_truncated": bool(truncated[-1]),
                  "terminal_return_observed": terminal,
                  "fixed_bootstrap_value_from_before": bootstrap,
                  "finite_horizon_note": "Zero-boundary discounted observed rewards. A truncation/fragment does not establish this as the continuing-task value.",
                  "bootstrap_note": "Same boundary V_before for every evaluated critic; not an independent truth estimate.",
                  "same_observation_comparison": True, "snapshots": {}}
        summary_path = path.parent / "rollout_summary.json"
        if summary_path.exists():
            expected_summary = receipt.get("artifact_hashes", {}).get(summary_path.name)
            if expected_summary is None or digest(summary_path) != expected_summary:
                raise ValueError(f"Heldout summary hash mismatch: {summary_path}")
            summary = json.loads(summary_path.read_text())
            common.update(role=summary.get("evaluation_protocol", {}).get("role"),
                          scenario=summary.get("evaluation_protocol", {}).get("scenario"), end_reason=summary.get("end_reason"))
        for name in modules:
            common["snapshots"][name] = {
                "vs_finite_horizon_observed_rewards": critic_metrics(values[name], finite, config["vf_clip_param"]),
                "vs_fixed_before_bootstrap_return": critic_metrics(values[name], fixed_bootstrap_return, config["vf_clip_param"]),
                "vs_fixed_before_one_step_td_target": critic_metrics(values[name], fixed_td_target, config["vf_clip_param"]),
                "own_value_td_residual": stats(reward + config["gamma"] * (~terminated) * next_values[name] - values[name])}
        results.append(common)
    return results


def write_csv(path, rows):
    if not rows:
        return
    keys = list(dict.fromkeys(key for row in rows for key in row))
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, keys)
        writer.writeheader()
        writer.writerows(clean(rows))


def plot_results(folder, metrics, predictions, batch, mask, minibatches):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False,
                         "savefig.dpi": 190, "figure.facecolor": "white"})
    folder.mkdir(parents=True, exist_ok=True)
    saved = []
    def save(fig, name):
        path = folder / name
        fig.savefig(path, bbox_inches="tight")
        plt.close(fig)
        saved.append(path)
    names = list(predictions)
    fig, axes = plt.subplots(2, 2, figsize=(11, 8), constrained_layout=True)
    for index, name in enumerate(names):
        row = metrics["snapshots"][name]
        axes[0, 0].bar(index, row["kl_old_to_current"]["mean"])
        axes[0, 1].bar(index, row["policy_loss_fixed_advantages"])
        axes[1, 0].bar(index - .18, row["critic_fixed_gae_targets"]["squared_error"]["mean"], width=.36, label="raw MSE" if index == 0 else None)
        axes[1, 0].bar(index + .18, row["critic_fixed_gae_targets"]["capped_mse"], width=.36, label="capped MSE" if index == 0 else None)
        axes[1, 1].bar(index, 100 * row["surrogate_actually_clipped_fraction"])
    for ax, title in zip(axes.flat, ["KL(old || snapshot), all real rows", "PPO policy loss, fixed advantages", "Critic error, fixed GAE targets", "Active surrogate clipping [%]"]):
        ax.set_title(title)
        ax.set_xticks(range(len(names)), [name.replace("minibatch_000008", "8 updates") for name in names], rotation=15)
        ax.grid(axis="y", alpha=.2)
    axes[1, 0].legend()
    fig.suptitle("One on-policy batch: offline snapshot comparison\nTraining objective only; physical performance evaluated separately")
    save(fig, "01_fixed_batch_objective.png")
    fig, axes = plt.subplots(2, 2, figsize=(11, 8), constrained_layout=True)
    targets = array(batch["value_targets"])[mask]
    advantages = array(batch["advantages"])[mask]
    for name in ("before", "first_pass", "final"):
        if name not in predictions:
            continue
        p = predictions[name]
        error = p["values"][mask] - targets
        axes[0, 0].hist(error, bins=65, histtype="step", density=True, label=name)
        quantile = np.linspace(0, 1, 101)
        axes[0, 1].plot(quantile, np.quantile(error ** 2, quantile), label=name)
    final = predictions["final"]
    axes[1, 0].scatter(advantages, final["logp_delta"][mask], s=5, alpha=.22, rasterized=True)
    axes[1, 0].axhline(0, color="grey", lw=.8)
    axes[1, 0].axvline(0, color="grey", lw=.8)
    old_mean = array(batch["action_dist_inputs"])[mask, :2]
    dmean = final["logits"][mask, :2] - old_mean
    for joint, label in enumerate(("knee", "ankle")):
        axes[1, 1].hist(dmean[:, joint], bins=55, histtype="step", label=label)
    axes[0, 0].set(title="Critic residual distribution", xlabel="V - fixed GAE target", ylabel="Density")
    axes[0, 1].set(title="Squared-error quantiles", xlabel="Quantile", ylabel="Squared error", yscale="symlog")
    axes[0, 1].axhline(metrics["config"]["vf_clip_param"], color="grey", ls="--", label="VF loss cap")
    axes[1, 0].set(title="Final sampled-action probability update", xlabel="Fixed normalized advantage", ylabel="log πfinal(a|s) - log πold(a|s)")
    axes[1, 1].set(title="Final mean displacement on same observations", xlabel="Δ mean [normalized action]", ylabel="Count")
    for ax in (axes[0, 0], axes[0, 1], axes[1, 1]):
        ax.legend()
    save(fig, "02_critic_and_advantage_alignment.png")
    if minibatches:
        fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
        x = [row["minibatch"] for row in minibatches]
        axes[0].plot(x, [row["kl"]["mean"] for row in minibatches], label="pre-step minibatch KL")
        axes[0].axhline(metrics["config"].get("kl_target", .01), color="grey", ls="--", label="configured target")
        axes[1].plot(x, [100 * row["surrogate_actually_clipped_fraction"] for row in minibatches], label="active surrogate clipping")
        axes[1].plot(x, [100 * row["vf_capped_fraction"] for row in minibatches], label="VF squared error at/above cap")
        axes[2].plot(x, [row["vf_squared_error"]["mean"] for row in minibatches], label="raw VF MSE")
        axes[2].plot(x, [row["vf_capped_loss"]["mean"] for row in minibatches], label="capped VF loss")
        for ax in axes:
            ax.legend()
            ax.grid(alpha=.2)
        axes[0].set_ylabel("KL")
        axes[1].set_ylabel("Rows [%]")
        axes[2].set(xlabel="Optimizer step (minibatch)", ylabel="Squared error")
        fig.suptitle("Actual PPO update trajectory\nDifferent minibatches: individual points are not a fixed-data convergence curve")
        save(fig, "03_minibatch_update_trajectory.png")
    return saved


def report_text(metrics):
    initial = metrics["initial_policy_replay"]
    before, final = metrics["snapshots"]["before"], metrics["snapshots"]["final"]
    boundary = metrics["boundaries"]
    corrected = bool(metrics.get("boundary_correction"))
    heldout = sum(bool(row.get("available")) for row in metrics["heldout"])
    lines = ["# Audit offline del learner PPO — 2026-09-29", "",
             "Confronto delle policy e dei critic sullo stesso batch on-policy, con advantages e target GAE congelati. Nessun training o rollout è eseguito da questa analisi.", "",
             f"Il batch contiene **{int(metrics['real_rows'])} transizioni reali** e {int(metrics['masked_rows'])} righe mascherate. I frammenti originali sono {boundary['episode_chunks']}: {boundary['terminated_chunks']} terminati, {boundary['truncated_chunks']} troncati e {boundary['unfinished_fragments']} ancora aperti. Episodi terminali osservati integralmente: {boundary['full_true_terminal_episodes_in_batch']}.", "",
             "## Riproduzione della policy prima dell'update", "",
             f"Sul batch intero, l'errore massimo fra logits archiviati e inferenza prima dell'update è {initial['batched_vs_stored_logits_max_abs_error']:.8g}; con inferenza una riga per volta è {initial['rowwise_vs_stored_logits_max_abs_error']:.8g}. Lo scarto massimo del ratio da 1 è {initial['rowwise_ratio_max_abs_deviation_from_one']:.8g} con inferenza per riga. Il confronto tra le due modalità quantifica l'effetto numerico float32 della dimensione del batch; non equivale automaticamente a una differenza di policy.", "",
             f"La penalità KL del confronto usa il coefficiente realmente ripristinato ({before['objective_coefficients']['kl']:.8g}), letto dallo snapshot prima dell'update. Il valore `kl_coeff` nel JSON di configurazione appartiene alla costruzione dell'algoritmo, prima del ripristino dello stato ausiliario.", "",
             "## Obiettivo congelato", "",
             "| Snapshot | KL media | Policy loss | VF MSE raw | VF loss capped | VF oltre cap | EV |", "|---|---:|---:|---:|---:|---:|---:|"]
    for name, row in metrics["snapshots"].items():
        vf = row["critic_fixed_gae_targets"]
        ev = vf["explained_variance"]
        ev_text = f"{ev:.4f}" if ev is not None else "n/d"
        lines.append(f"| {name} | {row['kl_old_to_current']['mean']:.6g} | {row['policy_loss_fixed_advantages']:.5g} | {vf['squared_error']['mean']:.5g} | {vf['capped_mse']:.5g} | {100*vf['cap_fraction']:.2f}% | {ev_text} |")
    lines += ["", f"La loss di policy passa da {before['policy_loss_fixed_advantages']:.6g} a {final['policy_loss_fixed_advantages']:.6g}; la MSE critic sullo stesso target passa da {before['critic_fixed_gae_targets']['squared_error']['mean']:.6g} a {final['critic_fixed_gae_targets']['squared_error']['mean']:.6g}. Questi sono risultati di adattamento sul batch utilizzato per l'update, non validazione indipendente.", "",
              "La VF loss locale satura ogni errore quadratico al valore di cap: sopra soglia il termine capped è localmente piatto. La sua media può quindi nascondere una coda di errori grandi. Quota e distribuzione sono misurate, ma non identificano da sole la causa delle coppie o della qualità imitativa.", "",
              "## Confini, bootstrap e interpretazione", "",
              f"L'allineamento esatto tra osservazioni degli episodi e righe del batch è `{boundary['episode_to_batch_order_exact']}`. La ricostruzione indipendente TD/GAE usa bootstrap zero soltanto sulle vere terminazioni, conservando il valore della prossima osservazione ai limiti del frammento.",
              "Nel CSV `independent_bootstrap_value_used` appartiene alla ricostruzione corretta; `stock_bootstrap_value_used` è il valore dopo l'azzeramento del helper stock (prima dell'eventuale correzione isolata). `bootstrap_value_predicted` è la predizione originale del critic alla riga artificiale.",
              f"**Discordanza verificata nei target:** errore massimo {boundary.get('independent_real_target_max_abs_error', 'n/d')}. Nel primo frammento aperto il target dell'ultima azione equivale alla sola reward, senza il valore della successiva osservazione. La composizione dei connector installati marca terminale la riga artificiale e il calcolo GAE annulla quel valore. Il riproduttore sintetico end-to-end, salvato in `bootstrap_connector_reproducer.json`, distingue frammento aperto, truncation e terminale vero.", "",
              "Questo è un errore verificabile nella costruzione del segnale di apprendimento della versione locale. Non dimostra, da solo, che spieghi tutta la differenza LEFT/RIGHT o le curve di coppia. L'A/B sul medesimo batch deve distinguere il suo effetto dall'ottimizzazione e dalla risposta fisica.", "",
              "La normalizzazione stock delle advantages comprende anche le righe artificiali, poi escluse dalla loss. Le metriche distinguono la sola variazione della normalizzazione dall'effetto combinato della correzione dei target; nessuna di queste modifiche è applicata dall'analizzatore.", "",
              "`minibatch_000008` indica otto aggiornamenti. `first_pass` indica il primo aggiornamento che supera o raggiunge il numero di righe del batch: consultare `exact_epoch_boundary` nei metadati prima di chiamarlo una singola epoca esatta.", "",
              f"Sono disponibili {heldout} dataset fisici indipendenti con osservazioni complete del critic. Per ciascuno, tutti i critic sono valutati sulle stesse osservazioni. Ritorni a orizzonte finito e ritorni con bootstrap V_before sono riportati separatamente: senza vera terminazione nessuno dei due è una misura osservata del ritorno infinito.", "",
              "## Decisioni supportate e limiti", "",
              "- Un mismatch iniziale oltre l'errore numerico richiede verificare pesi, versione del sampler e semantica raw/logprob prima di interpretare l'update.",
              "- Un miglioramento della surrogate sul batch seguito da peggioramento dei rollout sposta l'indagine verso ampiezza dell'update, distribuzione visitata e obiettivo, senza provare una causa unica.",
              "- Il confronto critic prima/dopo sul target fisso distingue adattamento numerico da validazione; un critic con errore residuo non è automaticamente il responsabile del fallimento RL.",
              "- Un solo batch e un solo seed non dimostrano convergenza, impossibilità di PPO, stabilità globale o superiorità statistica.", "",
              "Dati: `metrics.json`, `snapshot_metrics.csv`, `episode_boundaries.csv`, `predictions.npz`, `minibatch_metrics.csv`. Figure nella cartella root `plot/09_29_2026_rl_learning_audit/learner/`. Provenienza e integrità in `input_hashes.json` e `verification.json`."]
    if corrected:
        lines.insert(4, "**Ramo A/B con correzione boundary isolata.** I target delle righe reali sono corretti; quelli delle righe artificiali e la convenzione di normalizzazione rimangono stock. Le metriche di obiettivo di questo ramo non vanno confrontate direttamente con quelle stock senza tener conto del diverso target.")
        lines = [line.replace("**Discordanza verificata nei target:**", "**Verifica dei target del ramo corretto:**")
                 .replace("Nel primo frammento aperto il target dell'ultima azione equivale alla sola reward, senza il valore della successiva osservazione.",
                          "Nel ramo stock originale il target dell'ultima azione dei frammenti aperti equivaleva alla sola reward; questo ramo ripristina il bootstrap.")
                 for line in lines]
    return "\n".join(lines) + "\n"


def run(probe_dir, output_dir, plot_dir, heldout_paths=()):
    from ray.rllib.core.rl_module.rl_module import RLModule
    probe_dir, output_dir, plot_dir = (Path(path).expanduser().resolve() for path in (probe_dir, output_dir, plot_dir))
    manifest_path = probe_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if not manifest.get("complete"):
        raise ValueError("Probe manifest is not complete; refusing partial-data analysis")
    hashes_path = probe_dir / "artifact_hashes.json"
    hashes = json.loads(hashes_path.read_text())
    if not hashes:
        raise ValueError("Probe artifact manifest is empty")
    failed = [name for name, expected in hashes.items() if digest(probe_dir / name) != expected]
    if failed:
        raise ValueError(f"Probe hash mismatch: {failed}")
    config_path = probe_dir.parent / "effective_rllib_configuration.json"
    config = json.loads(config_path.read_text())
    batches = load_pickle(probe_dir / "batch_post_gae.pkl")
    if set(batches) != {"default_policy"}:
        raise ValueError("Expected one default policy")
    batch = batches["default_policy"]
    mask = array(batch["loss_mask"]).astype(bool)
    observations = array(batch["obs"])
    if observations.ndim != 2 or observations.shape[1] != 88 or mask.ndim != 1:
        raise ValueError("Expected flat 88-feature observations")
    torch.set_num_threads(1)
    modules, predictions, snapshots = {}, {}, {}
    before_scalars = manifest["snapshots"]["before"]["scalars"]
    kl_coeff = before_scalars["kl_coeff"]["default_policy"]
    entropy_coeff = before_scalars["entropy_coeff"]["default_policy"]
    names = [name for name in ("before", "minibatch_000008", "first_pass", "final") if name in manifest["snapshots"]]
    for name in names:
        module = RLModule.from_checkpoint(str(probe_dir / "snapshots" / name / "rl_module_default_policy"))
        module.eval()
        modules[name] = module
        logits, values = infer(module, observations)
        snapshot, outputs = snapshot_metrics(module, logits, values, batch, mask, config,
                                             kl_coeff=kl_coeff, entropy_coeff=entropy_coeff)
        snapshots[name] = snapshot
        predictions[name] = {"logits": logits, "values": values, **outputs}
    if "before" not in predictions or "final" not in predictions:
        raise ValueError("Both before and final snapshots are required")
    row_logits, row_values = infer(modules["before"], observations, chunk_size=1)
    old_logits = array(batch["action_dist_inputs"])
    row_outputs = distribution_outputs(modules["before"], row_logits, old_logits,
                                       array(batch["actions"]), array(batch["action_logp"]))
    initial = {
        "batched_inference_size": 512, "comparison_rows": int(mask.sum()),
        "batched_vs_stored_logits_max_abs_error": max_error(predictions["before"]["logits"][mask], old_logits[mask]),
        "rowwise_vs_stored_logits_max_abs_error": max_error(row_logits[mask], old_logits[mask]),
        "batched_vs_rowwise_logits_max_abs_error": max_error(predictions["before"]["logits"][mask], row_logits[mask]),
        "rowwise_ratio_max_abs_deviation_from_one": max_error(row_outputs["ratio"][mask], 1.),
        "rowwise_logp_error": stats(row_outputs["logp_delta"][mask]),
        "rowwise_logits_bit_exact": bool(np.array_equal(row_logits[mask], old_logits[mask])),
        "rowwise_logits_allclose_rtol1e_5_atol1e_5": bool(np.allclose(row_logits[mask], old_logits[mask], rtol=1e-5, atol=1e-5)),
        "batched_vs_rowwise_value_max_abs_error": max_error(predictions["before"]["values"][mask], row_values[mask]),
        "numerical_note": "Sampler has one environment per runner; batched GEMM can round differently from single-row inference. Both comparisons use the whole batch."}
    states = load_pickle(probe_dir / "episodes_pre_gae.pkl")
    gae = load_pickle(probe_dir / "gae_values_and_input.pkl")
    boundaries, boundary_rows, independent_targets = reconstruct_boundaries(
        states, batch, array(gae["values"]["default_policy"]), config["gamma"], config["lambda_"])
    metrics = {"schema": "rl_learning_audit_learner_analysis_v1", "probe": str(probe_dir.resolve()),
               "real_rows": int(mask.sum()), "masked_rows": int((~mask).sum()),
               "config": config, "initial_policy_replay": initial, "snapshots": snapshots,
               "snapshot_boundaries": {name: manifest["snapshots"][name] for name in names},
               "boundaries": boundaries, "heldout": analyze_heldout(heldout_paths, modules, config),
               "training_performed": False, "physical_simulations_performed": False,
               "scope": "One fixed on-policy batch; policy likelihood on raw sampled actions, not clipped commands"}
    correction_path = probe_dir.parent / "boundary_correction.json"
    if correction_path.exists():
        metrics["boundary_correction"] = json.loads(correction_path.read_text())
    metrics["native_pipeline_bootstrap_reproducer"] = native_pipeline_bootstrap_reproducer(config["gamma"], config["lambda_"])
    minibatches = [json.loads(line) for line in (probe_dir / "minibatch_metrics.jsonl").read_text().splitlines()]
    optimizer_path = probe_dir / "optimizer_step_metrics.jsonl"
    optimizer_rows = []
    if optimizer_path.exists():
        for line in optimizer_path.read_text().splitlines():
            item = json.loads(line)
            for module_id, parameters in item["modules"].items():
                for parameter, measures in parameters.items():
                    optimizer_rows.append({"minibatch": item["minibatch"], "module_id": module_id,
                                           "parameter": parameter, **measures})
        metrics["optimizer_trajectory"] = {
            "minibatches": len({row["minibatch"] for row in optimizer_rows}),
            "parameters": sorted({row["parameter"] for row in optimizer_rows}),
            "nonfinite_gradient_elements": sum(row["gradient_nonfinite_count"] or 0 for row in optimizer_rows),
            "parameter_update_l2_distribution": stats([row["update_l2"] for row in optimizer_rows]),
            "note": "Actual optimizer deltas; gradient norms are those remaining after original gradient processing and step."}
    output_dir.mkdir(parents=True, exist_ok=True)
    write_json(output_dir / "metrics.json", metrics)
    write_json(output_dir / "bootstrap_connector_reproducer.json", metrics["native_pipeline_bootstrap_reproducer"])
    snapshot_rows = []
    for name, row in snapshots.items():
        vf = row["critic_fixed_gae_targets"]
        snapshot_rows.append({"snapshot": name, "kl_mean": row["kl_old_to_current"]["mean"],
                              "policy_loss": row["policy_loss_fixed_advantages"], "vf_mse_raw": vf["squared_error"]["mean"],
                              "vf_mse_capped": vf["capped_mse"], "vf_cap_fraction": vf["cap_fraction"],
                              "vf_explained_variance": vf["explained_variance"],
                              "surrogate_clipped_fraction": row["surrogate_actually_clipped_fraction"],
                              "ratio_outside_clip_fraction": row["ratio_outside_clip_fraction"],
                              "total_objective_fixed_coefficients": row["total_objective_fixed_coefficients"]})
    write_csv(output_dir / "snapshot_metrics.csv", snapshot_rows)
    write_csv(output_dir / "episode_boundaries.csv", boundary_rows)
    write_csv(output_dir / "optimizer_parameter_steps.csv", optimizer_rows)
    write_csv(output_dir / "minibatch_metrics.csv", [
        {"minibatch": row["minibatch"], "kl_mean": row["kl"]["mean"], "ratio_min": row["ratio"]["min"],
         "ratio_max": row["ratio"]["max"], "surrogate_clipped_fraction": row["surrogate_actually_clipped_fraction"],
         "vf_cap_fraction": row["vf_capped_fraction"], "vf_mse_raw": row["vf_squared_error"]["mean"],
         "vf_mse_capped": row["vf_capped_loss"]["mean"]} for row in minibatches])
    arrays = {f"{name}__{key}": value for name, data in predictions.items() for key, value in data.items()}
    arrays.update(mask=mask, before_rowwise_logits=row_logits, before_rowwise_values=row_values)
    if independent_targets is not None:
        arrays["independent_gae_real_targets"] = independent_targets
    np.savez_compressed(output_dir / "predictions.npz", **arrays)
    plots = plot_results(plot_dir, metrics, predictions, batch, mask, minibatches)
    (output_dir / "report.md").write_text(report_text(metrics))
    inputs = {str((probe_dir / name).resolve()): sha for name, sha in hashes.items()}
    inputs.update({str(hashes_path.resolve()): digest(hashes_path), str(config_path.resolve()): digest(config_path),
                   str(Path(__file__).resolve()): digest(__file__)})
    if correction_path.exists():
        inputs[str(correction_path.resolve())] = digest(correction_path)
    for raw_path in heldout_paths:
        path = Path(raw_path)
        if path.is_dir():
            path = path / "critic_transitions.npz"
        if path.exists():
            inputs[str(path.resolve())] = digest(path)
            for sibling in ("evaluation_complete.json", "rollout_summary.json"):
                if (path.parent / sibling).exists():
                    inputs[str((path.parent / sibling).resolve())] = digest(path.parent / sibling)
    write_json(output_dir / "input_hashes.json", inputs)
    changed = [name for name, expected in inputs.items() if digest(name) != expected]
    if changed:
        raise ValueError(f"Analysis inputs changed: {changed}")
    outputs = [output_dir / name for name in ("metrics.json", "snapshot_metrics.csv", "episode_boundaries.csv", "minibatch_metrics.csv", "optimizer_parameter_steps.csv", "bootstrap_connector_reproducer.json", "predictions.npz", "report.md")]
    outputs += plots
    verification = {"probe_hashes_valid": True, "input_files_checked": len(inputs), "all_inputs_unchanged": True,
                    "plot_paths": [str(path.resolve()) for path in plots],
                    "output_hashes": {str(path.resolve()): digest(path) for path in outputs if path.exists()},
                    "partial_probe_refused": True}
    write_json(output_dir / "verification.json", verification)
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", type=Path, default=DEFAULT_PROBE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--plots", type=Path, default=DEFAULT_PLOTS)
    parser.add_argument("--heldout", nargs="*", type=Path, default=[])
    args = parser.parse_args()
    result = run(args.probe, args.output, args.plots, args.heldout)
    print(json.dumps(clean({"real_rows": result["real_rows"], "initial_policy_replay": result["initial_policy_replay"],
                            "snapshot_names": list(result["snapshots"]), "heldout_count": len(result["heldout"])}), indent=2))


if __name__ == "__main__":
    main()
