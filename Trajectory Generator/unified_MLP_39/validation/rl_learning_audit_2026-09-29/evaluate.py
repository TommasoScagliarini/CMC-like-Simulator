"""Explicit own-sigma evaluations for the one-batch PPO learning audit.

API: python evaluate.py --case-json case.json [--worker]
Required case fields: checkpoint, output_dir, role (before/after), scenario
(nominal/noisy). Defaults: environment_seed=123, noise_seed=123,
maximum_steps=500, render=true. The parent process supplies a watchdog unless
--worker is specified. Optional reuse_from is allowed for nominal cases only:
the actor is replayed on all archived observations before copying the outputs.

No training, reward overrides, sigma freezing, or production-code changes.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import time

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / "AGENTS.md").is_file())
PACKAGE = REPO / "Trajectory Generator/unified_MLP_39"
PLOT_ROOT = REPO / "plot/09_29_2026_rl_learning_audit"
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[key] = "1"
os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".matplotlib"))
sys.path.insert(0, str(PACKAGE.parent))
sys.dont_write_bytecode = True


def prepare_plot_cache():
    # Reuse this machine's already generated font index on macOS, where a
    # restricted system_profiler invocation cannot discover system fonts.
    # Other platforms retain their normal Matplotlib font discovery.
    if sys.platform == "darwin":
        prior = HERE.parent / "general_method_diagnosis_2026-09-29/quality/.matplotlib"
        current = Path(os.environ["MPLCONFIGDIR"])
        current.mkdir(parents=True, exist_ok=True)
        for source in prior.glob("fontlist-*.json"):
            target = current / source.name
            if not target.exists():
                shutil.copy2(source, target)


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temp.replace(path)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def paired_noise(np, seed, steps, action_dim=2):
    return np.random.Generator(np.random.PCG64(seed)).standard_normal((steps, action_dim)).astype(np.float32)


def noise_digest(draws):
    return hashlib.sha256(draws.astype("<f4").tobytes(order="C")).hexdigest()


def own_sigma_selector(torch, np, gaussian, records, *, nominal=False):
    """Float32 DiagGaussian arithmetic; z is paired, sigma follows current state."""
    def select(module, obs, action_shape, **_kwargs):
        index = len(records)
        if index >= len(gaussian):
            raise ValueError("Evaluation exceeded the pre-generated noise stream")
        obs_t = torch.as_tensor(np.asarray(obs), dtype=torch.float32).reshape(1, -1)
        with torch.no_grad():
            forward = module.forward_inference if nominal else module.forward_exploration
            logits = forward({"obs": obs_t})["action_dist_inputs"]
            dim = logits.shape[-1] // 2
            if dim != 2 or logits.shape[-1] != 4:
                raise ValueError("Audit requires the trained two-action diagonal Gaussian")
            mean, sigma = logits[..., :dim], torch.exp(logits[..., dim:])
            if not nominal:
                distribution = module.get_exploration_action_dist_cls().from_logits(logits)
                if not (torch.equal(distribution._dist.mean, mean)
                        and torch.equal(distribution._dist.stddev, sigma)):
                    raise ValueError("Checkpoint exploration distribution differs from DiagGaussian")
            z = torch.zeros_like(mean) if nominal else torch.as_tensor(gaussian[index]).reshape_as(mean)
            perturbation = sigma * z
            action = mean + perturbation
            if not all(bool(torch.isfinite(v).all()) for v in (mean, sigma, action)):
                raise ValueError("Non-finite policy action distribution")
        def array(value):
            return value.detach().cpu().numpy().reshape(action_shape).astype(np.float32)
        records.append({"index": index, "standard_normal": array(z).reshape(-1).tolist(),
                        "observation_before": np.asarray(obs, dtype=np.float32).reshape(-1).tolist(),
                        "action_dist_inputs": logits.detach().cpu().numpy().reshape(-1).tolist(),
                        "policy_mean": array(mean).reshape(-1).tolist(),
                        "learned_sigma": array(sigma).reshape(-1).tolist(),
                        "applied_noise": array(perturbation).reshape(-1).tolist(),
                        "raw_action": array(action).reshape(-1).tolist()})
        select.module = module
        return array(action), array(mean), array(sigma), array(perturbation)
    return select


def validate_case(case):
    result = {"environment_seed": 123, "noise_seed": 123, "maximum_steps": 500,
              "render": True, **case}
    if result["role"] not in ("before", "after", "first_pass") or result["scenario"] not in ("nominal", "noisy"):
        raise ValueError("role must be before/after/first_pass and scenario nominal/noisy")
    if result["role"] == "first_pass" and result["scenario"] != "nominal":
        raise ValueError("first_pass is a descriptive nominal snapshot, not a primary paired evaluation")
    if int(result["maximum_steps"]) < 1:
        raise ValueError("maximum_steps must be positive")
    if "sigma_mode" in result and result["sigma_mode"] != "own":
        raise ValueError("This primary audit evaluates the current policy's own sigma only")
    for key in ("checkpoint", "output_dir", "reuse_from"):
        if key in result:
            result[key] = str(Path(result[key]).expanduser().resolve())
    output = Path(result["output_dir"])
    output.relative_to(PACKAGE / "runs/rollout")
    if output == PACKAGE / "runs/rollout":
        raise ValueError("A per-case output directory is required")
    if result.get("reuse_from") and result["scenario"] != "nominal":
        raise ValueError("Archived stochastic draws cannot substitute this own-sigma protocol")
    return result


def checkpoint_evidence(ev, checkpoint):
    from unified_MLP_39 import actor_transition
    module = actor_transition.resolve_module_export(checkpoint)
    files = sorted(p for p in module.rglob("*") if p.is_file() and "__pycache__" not in p.parts)
    for parent in (Path(checkpoint), Path(checkpoint).parent, Path(checkpoint).parent.parent):
        path = parent / "training_cfg.resolved.yaml"
        if path.is_file():
            files.append(path)
            break
    if not any(p.name == "training_cfg.resolved.yaml" for p in files):
        raise ValueError("Checkpoint resolved runtime snapshot is required")
    return {str(p): digest(p) for p in dict.fromkeys(files)}


def render_canonical(folder, *, write_rollout_receipt=True, output_name=None):
    """Seven unchanged data figures, explicit LEFT target labels, root plot folder."""
    folder = Path(folder)
    summary = json.loads((folder / "rollout_summary.json").read_text())
    out = PLOT_ROOT / (output_name or folder.name)
    out.mkdir(parents=True, exist_ok=True)
    prepare_plot_cache()
    spec = importlib.util.spec_from_file_location("rl_learning_audit_plotter", REPO / "plot/plotter.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    context = module.valid_mlp_rollout_context(folder)
    if context is None:
        raise ValueError("Incomplete canonical-plot context")
    from matplotlib.text import Text
    original = module.save_figure
    saved = []
    def save(fig, out_dir, filename):
        if filename.startswith("07_"):
            for label in fig.findobj(match=Text):
                text = label.get_text().replace("Shifted Sound Leg Target", "Prescribed Left Phase Target")
                text = text.replace("shifted sound leg target", "prescribed left phase target")
                text = text.replace("shifted sound leg - policy C2", "left phase target - policy C2")
                label.set_text(text)
        original(fig, out_dir, filename)
        saved.append(out_dir / filename)
    module.save_figure = save
    module.next_output_dir = lambda *_: out
    module.latest_mlp_rollout_context = lambda: context
    old_argv = sys.argv
    try:
        sys.argv = [str(REPO / "plot/plotter.py"), "--mlp", "--results-dir", str(context.sim_outputs_dir),
                    "--prefix", "rollout_episode", "--setup", str(summary["setup_xml_path"]),
                    "--gait-side", "left", "--out-root", str(REPO / "plot")]
        module.main()
    finally:
        sys.argv = old_argv
    if len(saved) != 7 or len(set(saved)) != 7:
        raise RuntimeError(f"Expected seven canonical figures, got {len(saved)}")
    receipt = {"rollout": str(folder), "checkpoint": summary["checkpoint"],
               "evaluation_protocol": summary.get("evaluation_protocol"),
               "figure7_target": "recorded prescribed LEFT phase target; raw sound leg contextual only",
               "figures": {str(p.relative_to(REPO)): digest(p) for p in saved}}
    write(out / "plot_receipt.json", receipt)
    if write_rollout_receipt:
        write(folder / "plot_receipt.json", receipt)
    return receipt


def reuse_nominal(ev, args, case, select, records):
    """Verify weights/config and every observed nominal command before reuse."""
    from unified_MLP_39 import actor_transition
    source, output = Path(case["reuse_from"]), Path(case["output_dir"])
    summary = json.loads((source / "rollout_summary.json").read_text())
    trace = json.loads((source / "rollout_policy_trace.json").read_text())
    if summary["action_selection"] != "deterministic" or summary["action_seed"] != case["environment_seed"]:
        raise ValueError("Reused baseline must have matching deterministic seed protocol")
    if len(trace) != case["maximum_steps"] or summary["end_reason"] != "episode_time_limit":
        raise ValueError("Reuse requires a complete nominal episode")
    intervention = summary.get("diagnostic_intervention", {})
    if intervention and intervention.get("mode") != "baseline":
        raise ValueError("An intervention is not a nominal-policy baseline")
    prior = Path(summary["checkpoint"])
    if not prior.exists():
        prior = PACKAGE.parent / str(prior).replace("\\", "/").split("Trajectory Generator/", 1)[1]
    state = actor_transition.resolve_module_export(case["checkpoint"]) / "module_state.pkl"
    prior_state = actor_transition.resolve_module_export(prior) / "module_state.pkl"
    current_cfg = ev.training_config.load_resolved_for_checkpoint(case["checkpoint"])
    prior_cfg = ev.training_config.load_resolved_for_checkpoint(prior)
    if current_cfg != prior_cfg:
        raise ValueError("Archived baseline checkpoint runtime differs")
    module = ev.RLModule.from_checkpoint(actor_transition.resolve_module_export(case["checkpoint"]))
    prior_module = ev.RLModule.from_checkpoint(actor_transition.resolve_module_export(prior))
    current_state, prior_actor_state = module.get_state(), prior_module.get_state()
    actor_keys = sorted(key for key in prior_actor_state if not key.startswith(("vf.", "vf_encoder.")))
    if not actor_keys or not any(key.startswith("pi.") for key in actor_keys):
        raise ValueError("Cannot identify archived actor state")
    for key in actor_keys:
        if key not in current_state or not ev.np.array_equal(ev.np.asarray(current_state[key]), ev.np.asarray(prior_actor_state[key])):
            raise ValueError(f"Requested actor or phase buffer differs from archive: {key}")
    for key in ("_n_actor", "_n_full", "_action_dim"):
        if getattr(module, key) != getattr(prior_module, key):
            raise ValueError(f"Actor architecture differs from archive: {key}")
    if actor_transition.read_contract(case["checkpoint"]) != actor_transition.read_contract(prior):
        raise ValueError("Actor observation/action contract differs from archive")
    max_error = 0.0
    for row in trace:
        action, _, _, _ = select(module, row["actor_observation_vector_before"], (2,))
        archived = ev.np.asarray(row["raw_policy_action"], dtype=ev.np.float32)
        max_error = max(max_error, float(ev.np.max(ev.np.abs(action - archived))))
        if not ev.np.array_equal(action, archived):
            raise ValueError("Nominal replay is not exactly equal to archived float32 actions")
    output.mkdir(parents=True, exist_ok=True)
    copied = []
    for name in ("rollout_summary.json", "rollout_policy_trace.json", "training_cfg.resolved.yaml"):
        shutil.copy2(source / name, output / name)
        copied.append(name)
    shutil.copytree(source / "sim_outputs", output / "sim_outputs")
    copied.extend(str(p.relative_to(source)) for p in (source / "sim_outputs").glob("*.sto"))
    receipt = {"source": str(source), "source_hashes": {name: digest(source / name) for name in copied},
               "all_nominal_commands_reproduced_exactly": True, "max_action_error": max_error,
               "actor_tensors_phase_buffers_and_resolved_runtime_equal": True,
               "actor_state_keys_compared": actor_keys,
               "requested_module_state_sha256": digest(state), "archived_module_state_sha256": digest(prior_state),
               "full_pickle_equal": digest(state) == digest(prior_state),
               "critic_note": "Full learner checkpoint may include critic tensors absent from archived inference export; reused nominal contains no privileged critic data."}
    write(output / "nominal_reuse_receipt.json", receipt)
    return summary


def execute_case(case):
    case = validate_case(case)
    output = Path(case["output_dir"])
    allowed = {"watchdog_state.json", "watchdog_summary.json"}
    if output.exists() and any(p.name not in allowed for p in output.iterdir()):
        raise FileExistsError(f"Refusing to overwrite evaluation artifacts: {output}")
    from unified_MLP_39 import rollout_eval as ev
    argv = [str(PACKAGE / "rollout_eval.py"), "--checkpoint", case["checkpoint"], "--output-dir", str(output),
            "--seed", str(case["environment_seed"]), "--action-selection", "deterministic" if case["scenario"] == "nominal" else "stochastic",
            "--record-outputs", "--record-policy-trace", "--max-steps", str(case["maximum_steps"]),
            "--exploration-noise-hold-steps", "1", "--no-progress"]
    old_argv = sys.argv
    try:
        sys.argv = argv
        args = ev.parse_args()
    finally:
        sys.argv = old_argv
    ev._load_inference_stack()
    ev.torch.set_num_threads(1)
    evidence = checkpoint_evidence(ev, case["checkpoint"])
    gaussian = paired_noise(ev.np, int(case["noise_seed"]), int(case["maximum_steps"]))
    records = []
    selector = own_sigma_selector(ev.torch, ev.np, gaussian, records, nominal=case["scenario"] == "nominal")
    protocol = {**case, "sigma_mode": "own", "noise_scale": 0.0 if case["scenario"] == "nominal" else 1.0,
                "sampler": "numpy.PCG64 pre-generated standard normals, cast float32; torch float32 mean + exp(logstd) * z",
                "noise_hold_steps": 1, "pairing": "same standardized Gaussian by decision index; each policy uses its own state-dependent sigma",
                "environment_seed_semantics": "reset seed; independent of exploration noise_seed",
                "training_performed_by_evaluator": False, "checkpoint_input_hashes": evidence}
    protocol["action_transform"] = ({"protocol": "training_connector", "normalize_actions": True, "clip_actions": False,
            "function": "ray.rllib.utils.spaces.space_utils.unsquash_action on flattened Box[-1,1]",
            "raw_action_semantics": "untransformed Gaussian actor sample",
            "environment_input_semantics": "RLlib unsquash float32 arithmetic and bounds clamp before reward/environment wrapper",
            "clipping_diagnostic_semantics": "summary action_clipped_* still describes raw Gaussian exceedance; reward wrapper receives the connector output and therefore does not see that raw excursion",
            "deployment_equivalent_arithmetic": False} if case["scenario"] == "noisy" else
        {"protocol": "nominal_deployment", "normalize_actions": False, "clip_actions": False,
         "function": "raw deterministic actor mean passed to ordinary evaluator/env wrappers",
         "deployment_equivalent_arithmetic": True})
    original = ev._policy_action_with_diagnostics
    original_factory = ev.env_factory.make_cmc_env
    transitions = []
    if not case.get("reuse_from"):
        import gymnasium as gym
        from ray.rllib.utils.spaces.space_utils import unsquash_action
        class RecordTransitions(gym.Wrapper):
            def step(self, action):
                raw = ev.np.asarray(action, dtype=ev.np.float32).copy()
                if case["scenario"] == "noisy":
                    ev.np.testing.assert_array_equal(self.action_space.low, [-1., -1.])
                    ev.np.testing.assert_array_equal(self.action_space.high, [1., 1.])
                    command = unsquash_action(raw, self.action_space)
                else:
                    command = raw
                records[-1]["action_for_environment"] = ev.np.asarray(command, dtype=ev.np.float32).reshape(-1).tolist()
                next_obs, reward, terminated, truncated, info = self.env.step(command)
                transitions.append({"next_observation": ev.np.asarray(next_obs, dtype=ev.np.float32).reshape(-1),
                                    "reward": float(reward), "terminated": bool(terminated), "truncated": bool(truncated)})
                return next_obs, reward, terminated, truncated, info
        ev.env_factory.make_cmc_env = lambda config: RecordTransitions(original_factory(config))
    ev._policy_action_with_diagnostics = selector
    started = time.monotonic()
    try:
        summary = (reuse_nominal(ev, args, case, selector, records) if case.get("reuse_from") else ev.run(args))
    finally:
        ev._policy_action_with_diagnostics = original
        ev.env_factory.make_cmc_env = original_factory
        write(output / "paired_noise_receipt.json", {"protocol": protocol,
              "full_standard_normal_stream_sha256": noise_digest(gaussian),
              "draws_used": len(records), "records": records,
              "note": "Nominal rows use zero perturbation; noisy policies share z, not sigma or realized action noise."})
    if any(digest(path) != sha for path, sha in evidence.items()):
        raise ValueError("Checkpoint/config artifacts changed during evaluation")
    trace = json.loads((output / "rollout_policy_trace.json").read_text())
    if len(trace) != len(records) or len(trace) != summary["steps"]:
        raise ValueError("Trace, distribution records and summary have inconsistent lengths")
    if case["scenario"] == "noisy":
        # rollout_eval records its own raw clip before the observer. Preserve
        # that diagnostic separately and record the actual connector result.
        for row, record in zip(trace, records):
            row["deployment_clip_of_raw_action"] = row["applied_policy_action"]
            row["action_for_environment"] = record["action_for_environment"]
            row["applied_policy_action"] = record["action_for_environment"]
        write(output / "rollout_policy_trace.json", trace)
    if transitions:
        if len(transitions) != len(records):
            raise ValueError("Transition observer missed a physical step")
        np, torch = ev.np, ev.torch
        obs = np.asarray([r["observation_before"] for r in records], dtype=np.float32)
        next_obs = np.asarray([r["next_observation"] for r in transitions], dtype=np.float32)
        if obs.shape != next_obs.shape or obs.shape[1] != int(summary["n_observation"]):
            raise ValueError("Critic transition observations have wrong dimension")
        # Post-rollout value inference cannot influence the episode or its RNG.
        def values(data):
            with torch.no_grad():
                return selector.module.compute_values({"obs": torch.as_tensor(data)}).detach().cpu().numpy().reshape(-1)
        critic = hasattr(selector.module, "vf_encoder") and hasattr(selector.module, "vf")
        value_fields = {"value": values(obs), "next_value": values(next_obs)} if critic else {}
        np.savez_compressed(output / "critic_transitions.npz", observation=obs, next_observation=next_obs,
                            action_dist_inputs=np.asarray([r["action_dist_inputs"] for r in records], dtype=np.float32),
                            raw_action=np.asarray([r["raw_action"] for r in records], dtype=np.float32),
                            action_for_environment=np.asarray([r["action_for_environment"] for r in records], dtype=np.float32),
                            reward=np.asarray([r["reward"] for r in transitions]),
                            terminated=np.asarray([r["terminated"] for r in transitions]),
                            truncated=np.asarray([r["truncated"] for r in transitions]),
                            segment_completed=np.asarray([r.get("segment_completed", True) for r in trace]), **value_fields)
        protocol["critic_transitions"] = {"path": "critic_transitions.npz", "observation_dimension": int(obs.shape[1]),
                "value_estimates_available": critic,
                "values_computed": "post-rollout, same checkpoint; no action or random-number intervention",
                "bootstrap_note": "Store terminated/truncated separately; finite episode returns are not infinite-horizon MC without a boundary assumption."}
    else:
        protocol["critic_transitions"] = {"available": False, "reason": "Reused archive contains only actor observations, not privileged critic observations."}
    completed = sum(bool(row.get("segment_completed", True)) for row in trace)
    full = completed == len(trace) == case["maximum_steps"] and summary["end_reason"] == "episode_time_limit"
    summary.update(checkpoint=case["checkpoint"], completed_full_episode=full, evaluation_protocol=protocol,
                   evaluation_wall_seconds=time.monotonic() - started,
                   action_seed_semantics="rollout_eval legacy action_seed is the environment seed; exploration seed is evaluation_protocol.noise_seed")
    write(output / "rollout_summary.json", summary)
    if case["render"]:
        render_canonical(output)
    artifacts = [output / name for name in ("rollout_summary.json", "rollout_policy_trace.json", "paired_noise_receipt.json", "training_cfg.resolved.yaml")]
    artifacts += sorted((output / "sim_outputs").glob("*.sto"))
    if (output / "plot_receipt.json").exists():
        artifacts.append(output / "plot_receipt.json")
    if (output / "critic_transitions.npz").exists():
        artifacts.append(output / "critic_transitions.npz")
    write(output / "evaluation_complete.json", {"role": case["role"], "scenario": case["scenario"],
          "noise_seed": case["noise_seed"], "steps": len(trace), "complete_segments": completed,
          "completed_full_episode": full, "checkpoint_unchanged": True,
          "artifact_hashes": {str(p.relative_to(output)): digest(p) for p in artifacts}})
    print(json.dumps({k: summary.get(k) for k in ("steps", "end_reason", "episode_return", "completed_full_episode", "grf_penetration_max_m")}), flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-json", type=Path)
    parser.add_argument("--worker", action="store_true", help="Run directly; caller owns the watchdog")
    parser.add_argument("--render-only", type=Path, help="Render an already finalized case, without simulation")
    args = parser.parse_args()
    if args.render_only:
        render_canonical(args.render_only.resolve())
        return
    if args.case_json is None:
        parser.error("--case-json is required unless --render-only is selected")
    case = validate_case(json.loads(args.case_json.read_text(encoding="utf-8")))
    if args.worker or case.get("reuse_from"):
        execute_case(case)
        return
    output = Path(case["output_dir"])
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    output.mkdir(parents=True)
    from baseline_MLP import process_watchdog
    result = process_watchdog.supervise_process(
        [sys.executable, str(Path(__file__).resolve()), "--case-json", str(args.case_json.resolve()), "--worker"],
        heartbeat_file=output / "watchdog_state.json", summary_file=output / "watchdog_summary.json",
        startup_timeout_s=300., stall_timeout_s=120., run_timeout_s=2400., label=output.name)
    raise SystemExit(result["returncode"] if result["timeout_reason"] is None else 124)


if __name__ == "__main__":
    main()
